#include "it.h"



IT::IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object){
       
    this->sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->objectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
     // to compute voronoi diagram over all points
    this->sceneCloud   = scene;
    this->objectCloud  = object;
    

}


void IT::calculate(){
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //calculate the Region of Interest (RoI) for further calculations
    //a sphere with radio equal to the diagonal of the box which surroundsthe object
    pcl::PointXYZ min, max, middlePointObject;
    float radio;
    
    pcl::getMinMax3D(*objectCloud,min,max);
    
    middlePointObject.x=(max.x+min.x)/2;
    middlePointObject.y=(max.y+min.y)/2;
    middlePointObject.z=(max.z+min.z)/2;
    
    radio=pcl::L2_Norm(min.getArray3fMap(),max.getArray3fMap(),3);
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // extract volume of interest from the scene
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudFiltered;
    
    sceneCloudFiltered = Util::sphereExtraction(sceneCloud, middlePointObject,radio);

    

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculate the Interaction Bisector Surface (IBS)
    IBS ibs_calculator(sceneCloudFiltered,objectCloud);
StopWatch sw;
sw.Restart();
    ibs_calculator.calculate();
std::cout << "TIMER: ISB calculation " << sw.ElapsedMs() << std::endl;
    
    
sw.Restart();
    //as IBS extend to infinity, filter IBS
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered;
    
    ibsFiltered = Util::sphereExtraction(ibs_calculator.getIBS(), middlePointObject,radio);
    
    
//     //TODO erase this, it is only for developing porpouses
//     pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile("./tmp/ibs_clouds_prefiltered_filtered.pcd", *ibsFiltered);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Searching nearest neighbours (from IBS to ) and calculating smoot provenance vector
    
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSCN;
    std::vector<int>                NNidSC(1);
    std::vector<float>              NNdistSC(1);
    
    kdtreeSCN.setInputCloud(sceneCloudFiltered);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr  field(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr       smoothField(new pcl::PointCloud<pcl::Normal>);
    
    
    float   maxV      = std::numeric_limits<float>::min();
    float   minV      = std::numeric_limits<float>::max();
    float   maxS      = std::numeric_limits<float>::min();
    float   minS      = std::numeric_limits<float>::max();
    float   sum       = 0;
    double  sumSmooth = 0;
    int     knn       = 5;
    
   
    std::vector<int> bad_ids;
    // Try to ingnore "bad vectors" which would be something very large (silly)
    //int somethingSilly=1000; // TODO this must change
    int somethingSilly = 0;
    
    //std::cout<<"Checking for normals_z greater than "<<somethingSilly<<std::endl;
    
    for(int i=0;i<ibsFiltered->size();i++)
    {
               
        pcl::PointXYZ ibs_point = ibsFiltered->at(i);
        
        int num_neighbours = kdtreeSCN.nearestKSearch(ibs_point,knn,NNidSC,NNdistSC);
        
        if( num_neighbours > 0 )
        {
            
            Eigen::Vector3f resultant(0,0,0);
            Eigen::Vector3f scaled_v;
            float norm_resultant;
            float norm_scaled_v;
            
            for(int j=0;j<knn;j++)
            {
                
                pcl::PointXYZ sc;
                
                Eigen::Vector3f component;
                float norm_component;
                
                
                sc             = sceneCloudFiltered->at( NNidSC.at(j) );
                component      = Eigen::Vector3f( sc.x-ibs_point.x,sc.y-ibs_point.y,sc.z-ibs_point.z );
                norm_component = component.norm();
                
                
                if(component[2]>somethingSilly && j==0){
                    //this means the provenance vector goes in a different direction to the scene POSSIBLE penetration of the IBS
                    bad_ids.push_back(i);
                }
                if(j==0)
                {
                    // first NN is used for provenance vectors
                    pcl::PointNormal pn;
                    
                    pn.x = ibs_point.x; //TODO tengo que comprobar que la informaci[on se encuentra coorectamente asignada
                    pn.y = ibs_point.y;
                    pn.z = ibs_point.z;
                    
                    pn.normal_x = component[0];
                    pn.normal_y = component[1];
                    pn.normal_z = component[2];
                    
                    scaled_v      = component;
                    norm_scaled_v = norm_component;
                    
                    if(minV > norm_component)
                        minV = norm_component;
                    if(maxV < norm_component)
                        maxV = norm_component;
                    
                    sum += norm_component;      // it seems that this is unuseful
                
                    
                    field->push_back(pn);
                }
                
                resultant += component;
            }
            
            //"smmother" provenance vector
            scaled_v       = norm_scaled_v * resultant.normalized();
            norm_resultant = resultant.norm();
            
            smoothField->push_back( pcl::Normal(scaled_v[0],scaled_v[1],scaled_v[2]) );

            if( minS > norm_resultant )
                minS = norm_resultant;
            if( maxS < norm_resultant )
                maxS = norm_resultant;

            sumSmooth += scaled_v.norm();   //it seems unuseful
        }
    }
    
    
std::cout << "TIMER: Provenance vectors " << sw.ElapsedMs() << std::endl;

    std::cout<<"Tensor before filtering "<<ibsFiltered->size() <<endl;
 
sw.Restart();

    std::cout<<"Tensor before filtering "<<ibsFiltered->size();
    
    //If there were some "bad" provenance vectors remove them
    if(bad_ids.size()>0)
    {
        pcl::ExtractIndices<pcl::Normal> extractN;
        pcl::ExtractIndices<pcl::PointNormal> extractF;
        pcl::ExtractIndices<pcl::PointXYZ> extractP;
        
        pcl::PointIndices::Ptr outliers (new pcl::PointIndices ());
        
        outliers->indices = bad_ids;
        
        // Extract the outliers
        extractN.setInputCloud (smoothField);
        extractF.setInputCloud(field);
        extractP.setInputCloud(ibsFiltered);


        extractN.setIndices (outliers);
        extractF.setIndices(outliers);
        extractP.setIndices(outliers);


        extractN.setNegative (true);
        extractF.setNegative(true);
        extractP.setNegative(true);

        extractN.filter (*smoothField);
        extractF.filter (*field);
        extractP.filter (*ibsFiltered);
    }
    
    //TODO this seems unecesary
    //This is a cleaner tensor/ibs, which does not have those bad prov vectors
    pcl::PointCloud<pcl::PointXYZ>::Ptr copyIBS(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*ibsFiltered,*copyIBS);

std::cout << "TIMER: Filtering " << sw.ElapsedMs() << std::endl;

    // Print out some data about the tensor and filtering/cleaning
    std::cout<<"Tensor after filtering "<<copyIBS->size()<<std::endl;
    std::cout<<"Min: "<<minV<<" Max: "<<maxV<<std::endl;
    std::cout<<"Sum: "<<sum<<std::endl;
    std::cout<<"======================"<<std::endl;
    std::cout<<"MinS: "<<minS<<" MaxS: "<<maxS<<std::endl;
    std::cout<<"SumSmooth: "<<sumSmooth<<std::endl;
    
    
sw.Restart();    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // it is possible that this section is useless
    
    // Save the original provenance vectors bacause we are going to normalize
    pcl::PointCloud<pcl::Normal>::Ptr normals_backup(new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*field,*normals_backup);
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Generate weights for sampling of vizualization in provenance vectors   
    
    float max_out = 100;
    float min_out = 1;
    
    //Init probabilities for sampling
    std::vector<float> probs( field->size() );
   
    //The mapping or normalization options
    int myMap = 2;  //0-> [0-1], 1->[min_out,max_out], 2->no mapping

    // Newer min/max after filtering needs to be recomputed
    float nMax = std::numeric_limits<float>::min();
    float nMin = std::numeric_limits<float>::max();
    
    // Map every vector in the tensor
    for(int i=0;i<field->size();i++)
    {
        Eigen::Vector3f oldNormal( field->at(i).normal_x, field->at(i).normal_y, field->at(i).normal_z);
        //float map_prob = Util_iT::getValueProporcionsRule( oldNormal.norm(), minV, maxV, 0, 1);
        //(oldNormal.norm() - minV) * (1- 0) / (maxV - minV) + 0;
        
//         std::cout <<"map_prob "<< map_prob << " =  " << ((oldNormal.norm() - minV) * (1- 0) / (maxV - minV) + 0 ) << endl;
//         assert(map_prob == ((oldNormal.norm() - minV) * (1- 0) / (maxV - minV) + 0 ) && "map_prob");
        
        
        // Probability is inverse of magnitude
        // Longer provenance vectors -> lower prob of being sampled
        // Smaller provenance vectors are associated to regions where objects are closer together
        probs.at(i) = Util_iT::getValueProporcionsRule( oldNormal.norm(), minV, maxV, 1, 0);
        
//         std::cout <<"probs.at("<<i<<") "<< Util_iT::getValueProporcionsRule( oldNormal.norm(), minV, maxV, 1, 0) << " =  " << 1-map_prob << endl;
//         assert( ( Util_iT::getValueProporcionsRule( oldNormal.norm(), minV, maxV, 1, 0) == probs.at(i) ) && "probs.at");
        

        
        Eigen::Vector3f oldNormalSmooth(smoothField->at(i).normal_x,smoothField->at(i).normal_y,smoothField->at(i).normal_z);
        
        Eigen::Vector3f newNormal,newNormalSmooth;
        
        if(myMap==1)
        {
            float mapped_mag  = Util_iT::getValueProporcionsRule( oldNormal.norm(), minV, maxV, min_out, max_out );
//             std::cout <<"mapped_mag   "<< mapped_mag << " =  " << (( oldNormal.norm() - minV) * (max_out- min_out) / (maxV - minV) + min_out )<< endl;
//             assert(( mapped_mag == (( oldNormal.norm() - minV) * (max_out- min_out) / (maxV - minV) + min_out )) && " mapped_mag no corresponding value");
            
            
            
            float mapped_mag2 = Util_iT::getValueProporcionsRule( oldNormalSmooth.norm(), minV, maxV, min_out, max_out );
//             std::cout <<"mapped_mag2  "<< mapped_mag2 << " =  " << (( oldNormalSmooth.norm() - minV) * (max_out- min_out) / (maxV - minV) + min_out) << endl;
//             assert((mapped_mag2 == (( oldNormalSmooth.norm() - minV) * (max_out- min_out) / (maxV - minV) + min_out)) && "mapped_mag2");

            newNormal         = ( 1 / mapped_mag) * oldNormal.normalized();
            newNormalSmooth   = ( 1 / mapped_mag2) * oldNormalSmooth.normalized();
    
            
        }
        if(myMap==0)
        {
            newNormal       = probs.at(i) * oldNormal.normalized();
            newNormalSmooth = probs.at(i) * oldNormalSmooth.normalized();
        }
        if(myMap==2)
        {
            newNormal       = oldNormal;
            newNormalSmooth = oldNormalSmooth;
        }
        
        field->at(i).normal_x = newNormal[0];
        field->at(i).normal_y = newNormal[1];
        field->at(i).normal_z = newNormal[2];

        smoothField->at(i).normal_x=newNormalSmooth[0];
        smoothField->at(i).normal_y=newNormalSmooth[1];
        smoothField->at(i).normal_z=newNormalSmooth[2];

        //Check/save new max/min in tensor field
        float mag = newNormal.norm();
        if(mag<nMin)    nMin=mag;
        if(mag>nMax)    nMax=mag;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Sampling provenance vector to construct the descriptor
    
    // Sample size to take from tensor
    int sampleSize = 512;
    
    // Sample with prob: Probability inversly proportional to provenance vector length
    // These are affordance keypoints
    std::vector<int> keypoint_ids = sampleWithProbability( probs, sampleSize );
    
    // Aux containers for uniform sampling
    std::vector<int> keypoints_uniform = sampleUniformProbability( field->size() ,sampleSize );
    

    //std::cout<<"Got "<<keypoint_ids.size()<<" keypoints"<<std::endl;
    
    

    // Container to sort affordance keypoint
    // "first" : id , "second" : magnitude
    std::vector< std::pair<int, double> > sortablePoints(keypoint_ids.size());
    std::vector< std::pair<int, double> > sortableUnidorm(keypoints_uniform.size());

    // Fill the sortable containers
    for(int i=0;i<keypoint_ids.size();i++)
    {
        //sampled by weight
        std::pair<int, double>  pW;
        pcl::Normal n;
        pW.first      = keypoint_ids.at(i);
        n             = normals_backup->at(pW.first);
        pW.second     = Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z).norm();
        sortablePoints.at(i) = pW;
        
        
        //sampled uniformly
        std::pair<int, double>  pwU;
        pcl::Normal nU;
        pwU.first    = keypoints_uniform.at(i);
        nU           = normals_backup->at(pwU.first);
        pwU.second   = Eigen::Vector3f(nU.normal_x, nU.normal_y, nU.normal_z).norm();
        sortableUnidorm.at(i) = pwU;
    }
    
std::cout << "TIMER: Sampling " << sw.ElapsedMs() << std::endl;
    
    
    //This is a cleaner tensor/ibs, which does not have those bad prov vectors
    PointCloud::Ptr copyIBS(new PointCloud);
//     if(bad_ids.size()<1)
        pcl::copyPoint(*ibsFiltered,*copyIBS);
//     else
//     {
//         int aux_id=0;
//         int bad=bad_ids.at(aux_id);
//         for(int i=0;i<ibsFiltered->size( );i++)
//         {
//             if(i==bad)
//             {
//                 aux_id+=1;
//                 if(aux_id<bad_ids.size())
//                     bad=bad_ids.at(aux_id);
//                 continue;
//             }
//             else
//             {
//                 copyIBS->push_back(ibsFiltered->at(i));
//             }
//         }
//     }
}


std::vector<int> IT::sampleWithProbability( std::vector<float> weights,int sampleSize ){
    
    std::vector<int> idxSamples(sampleSize);
    
    int  indexer=0;
    int nrolls = 2*weights.size();
    
    //random numers generator
    std::default_random_engine generator;
    // distribution used for generate number randomly
    std::discrete_distribution<int> distribution (weights.begin(),weights.end());
   
    // Sampled data, it will be the vote bins  to define keypoints 
    std::vector< std::pair<int,int> > bins;
    
    //inizializing container of votes
    bins.reserve(weights.size());
	std::generate_n(std::back_inserter(bins), weights.size(), [indexer]()mutable { return std::make_pair (indexer++,0); });
    
    
    //full filling the vote containers throught the generator of random numbers
    for (indexer=0; indexer<nrolls; ++indexer)    {
        int number = distribution(generator);
        bins.at(number).second += 1;
    }
    
    //ordering votes and indexs associated in a descendent way 
    std::sort(bins.begin(),bins.end(), 
              [](const std::pair <int, int> & a, const std::pair <int, int> & b) -> bool{ return a.second > b.second; });
    
    //sampling
    for(indexer=0;indexer<sampleSize;indexer++)    {
        idxSamples.at(indexer) = bins.at( indexer ).first;
        //std::cout << "index "<< indexer<< ", votes " <<  bins.at( indexer ).first << " : " << bins.at( indexer ).second <<  std::endl;
    }
    
    return idxSamples;
}


std::vector<int> IT::sampleUniformProbability(int originalSize,int sampleSize ){
    
    // Aux containers for uniform sampling
    std::vector<int> idxSamples;
    std::vector<int> aux_ids;
    
    aux_ids.reserve(originalSize);
    
    int n = 0;
    
    std::generate_n(std::back_inserter(aux_ids), originalSize, [n]()mutable { return n++; });
    std::srand (unsigned(std::time(0)));

    // By now aux_ids is filled in ascending order [0-field-size)
    // Shuffle them ramdomly and get the sample
    std::random_shuffle ( aux_ids.begin(), aux_ids.end() );
    
    idxSamples.assign(aux_ids.begin(),aux_ids.begin()+sampleSize);

    
    return idxSamples;
    std::cout << "Filtered: " << ibsFiltered->width << endl;    
}
