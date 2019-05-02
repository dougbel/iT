#include "it.h"



IT::IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, std::string name_affordance, std::string name_object){
       
    this->sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->objectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
     // to compute voronoi diagram over all points
    this->sceneCloud   = scene;
    this->objectCloud  = object;
    this->affordanceName = name_affordance;
    this->objectName   = name_object;
    

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

    
 StopWatch sw;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculate the Interaction Bisector Surface (IBS)
//     IBS ibs_calculator(sceneCloudFiltered,objectCloud);

// sw.Restart();
//     ibs_calculator.calculate();
// std::cout << "TIMER: ISB calculation " << sw.ElapsedMs() << std::endl;
//     
//     
// sw.Restart();
//     //as IBS extend to infinity, filter IBS
//     pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered;
//     ibsFiltered = Util::sphereExtraction(ibs_calculator.getIBS(), middlePointObject,radio);
    
    
//     //TODO erase this, it is only for developing porpouses
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./tmp/ibs_clouds_prefiltered_filtered.pcd", *ibsFiltered);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Searching nearest neighbours (from IBS to ) and calculating smoot provenance vector
    
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSCN;
    std::vector<int>                NNidSC(1);
    std::vector<float>              NNdistSC(1);
    
    kdtreeSCN.setInputCloud(sceneCloudFiltered);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr  field(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  smoothField(new pcl::PointCloud<pcl::PointNormal>);
    
    
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
            
            pcl::PointNormal pn;
		  pn.x = field->back().x; //TODO tengo que comprobar que la informaci[on se encuentra coorectamente asignada
		  pn.y = field->back().y;
		  pn.z = field->back().z;
		  pn.normal_x = scaled_v[0];
		  pn.normal_y = scaled_v[1];
		  pn.normal_z = scaled_v[2];
            smoothField->push_back( pn );

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
    //If there were some "bad" provenance vectors remove them
    if(bad_ids.size()>0)
    {
        pcl::ExtractIndices<pcl::PointNormal> extractN;
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
    std::vector<float> probs = getSamplingProbabilities(field, minV, maxV);
   
    //The mapping or normalization options
    int myMap = 0;  //0-> [0-1], 1->[min_out,max_out], 2->no mapping

    // Newer min/max after filtering needs to be recomputed
    float nMax = std::numeric_limits<float>::min();
    float nMin = std::numeric_limits<float>::max();
     
    
    //BEGINING Mapping magnitudes of normal [0,1]
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothFieldNewSampling(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*smoothField,*smoothFieldNewSampling);
    //pcl::PointCloud<pcl::PointNormal>::Ptr fieldNewSampling(new pcl::PointCloud<pcl::Normal>);
    //pcl::copyPointCloud(*field,*fieldNewSampling);
    //TODO change this by smoothField and field repectively
    Util_iT::mapMagnitudes( *smoothFieldNewSampling, minV, maxV, 1, 0 );
    //Util_iT::mapMagnitudes( *fieldNewSampling, minV, maxV, 1, 0 );
    nMin=0;
    nMax=1;
    //END Mapping magnitudes of normal [0,1]
    
    
    
    
    
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
        //probs.at(i) = Util_iT::getValueProporcionsRule( oldNormal.norm(), minV, maxV, 1, 0);
	   
	   //std::cout<<probsNew.at(i)<< "probs "<<probs.at(i)<< std::endl;
	   //assert(probsNew.at(i)==probs.at(i));
        
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
	
	   
	   //BEGIN THIS ZONE IS FOR TESTING mapping [0,1]
// 	   std::cout << Util_iT::round4decimals(smoothField->at(i).normal_x) <<"*x*"<< Util_iT::round4decimals(smoothFieldNewSampling->at(i).normal_x) <<"   "<<
// 		  Util_iT::round4decimals(smoothField->at(i).normal_y)  <<"*y*"<<  Util_iT::round4decimals(smoothFieldNewSampling->at(i).normal_y)<<"   "<<
// 		  Util_iT::round4decimals(smoothField->at(i).normal_z) <<"*z*"<< Util_iT::round4decimals(smoothFieldNewSampling->at(i).normal_z) <<std::endl;
// 	   std::cout << smoothField->at(i).normal_x <<"*x*"<< smoothFieldNewSampling->at(i).normal_x <<"   "<<
// 		  smoothField->at(i).normal_y  <<"*y*"<<  smoothFieldNewSampling->at(i).normal_y <<"   "<<
// 		 smoothField->at(i).normal_z <<"*z*"<< smoothFieldNewSampling->at(i).normal_z <<std::endl;
// 		  
// 	   if(Util_iT::round4decimals(smoothField->at(i).normal_x) != Util_iT::round4decimals(smoothFieldNewSampling->at(i).normal_x) || 
// 		  Util_iT::round4decimals(smoothField->at(i).normal_y) != Util_iT::round4decimals(smoothFieldNewSampling->at(i).normal_y) ||
// 		  Util_iT::round4decimals(smoothField->at(i).normal_z) != Util_iT::round4decimals(smoothFieldNewSampling->at(i).normal_z)
// 	   )
// 		  std::cout << "checale"<<std::endl;
	   //END TESTING ZONE FINALIZED
		   

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
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Sorting and extracting provenance vectors

    // Actual sort according to provenance vector length (ascending way)
    // Small (high weight) vectors come first
    //std::cout<<"sorting new sample...";
    std::sort(sortablePoints.begin(),sortablePoints.end(), [](const std::pair <int, double> & a, const std::pair <int, double> & b) -> bool{ return a.second < b.second; });
    //std::cout<<"done"<<std::endl;
    //std::cout<<"sorting uniform sample...";
    std::sort(sortableUnidorm.begin(),sortableUnidorm.end(),[](const std::pair <int, double> & a, const std::pair <int, double> & b) -> bool{ return a.second < b.second; });
    //std::cout<<"done"<<std::endl;
    
    
    //Sample points sorted according to weight are copied to pointcloud format and saved
    pcl::PointCloud<PointWithVector>::Ptr new_sampleCloud2(new pcl::PointCloud<PointWithVector>);
    pcl::PointCloud<PointWithVector>::Ptr new_sampleCloudU(new pcl::PointCloud<PointWithVector>);

    //Save mags in sampled mapped in 0-1 based on full tensor mags
    std::vector<float> mags_c(sampleSize);
    std::vector<float>mags_cU(sampleSize);
    
    pcl::PointCloud<pcl::Normal>::Ptr provenanceToPlot(new pcl::PointCloud<pcl::Normal>);
    PointCloudT::Ptr provenanceVectorsAnchor(new PointCloudT);
    std::cout<<"extracting new sample...";
    for(int i=0;i<sampleSize;i++)
    {
        
        //Saving provenance vectors sampled by WEIGHTS
        PointWithVector pv;
        pv.x=ibsFiltered->at(sortablePoints.at(i).first).x;
        pv.y=ibsFiltered->at(sortablePoints.at(i).first).y;
        pv.z=ibsFiltered->at(sortablePoints.at(i).first).z;
        pv.v1=normals_backup->at(sortablePoints.at(i).first).normal_x;
        pv.v2=normals_backup->at(sortablePoints.at(i).first).normal_y;
        pv.v3=normals_backup->at(sortablePoints.at(i).first).normal_z;
        new_sampleCloud2->push_back(pv);
        //pcl::Normal n(pv.v1,pv.v2,pv.v3);
        //normals_check->push_back(n);
        Eigen::Vector3f n(pv.v1,pv.v2,pv.v3);
        //Save mags in sampled mapped in 0-1 based on full tensor mags
        mags_c.at(i)=Util_iT::getValueProporcionsRule( n.norm(), nMin, nMax, 1, 0 );
        
        //std::cout <<"mags_c.at(" <<i<< ")  "<< mags_c.at(i) << " =  " << (1 - ((n.norm()-nMin)*(1-0)/(nMax-nMin)+0 )) << endl;
        //assert( mags_c.at(i) == (1 - ((n.norm()-nMin)*(1-0)/(nMax-nMin)+0 )) && "mags_c.at(i)");
        
        
        
        //Saving provenance vectors sampled UNIFORMILY
        PointWithVector pvU;
        pvU.x=ibsFiltered->at(sortableUnidorm.at(i).first).x;
        pvU.y=ibsFiltered->at(sortableUnidorm.at(i).first).y;
        pvU.z=ibsFiltered->at(sortableUnidorm.at(i).first).z;
        pcl::PointXYZ provenanceAnchor(pvU.x,pvU.y,pvU.z);
        provenanceVectorsAnchor->push_back(provenanceAnchor);
        pvU.v1=normals_backup->at(sortableUnidorm.at(i).first).normal_x;
        pvU.v2=normals_backup->at(sortableUnidorm.at(i).first).normal_y;
        pvU.v3=normals_backup->at(sortableUnidorm.at(i).first).normal_z;
        new_sampleCloudU->push_back(pvU);
        provenanceToPlot->push_back(normals_backup->at(sortableUnidorm.at(i).first));
        Eigen::Vector3f nU(pvU.v1,pvU.v2,pvU.v3);
        //Save mags in sampled mapped in 0-1 based on full tensor mags
        mags_cU.at(i) = Util_iT::getValueProporcionsRule( nU.norm(), nMin, nMax, 1, 0 );
        
        //std::cout <<"mags_cU.at(" <<i<< ")  "<< mags_cU.at(i) << " =  " << (1-( (nU.norm()-nMin)*(1-0)/(nMax-nMin)+0 )) << endl;
        //assert( (1-( (nU.norm()-nMin)*(1-0)/(nMax-nMin)+0 )) && "mapped_mag2");
        
        
    }
    std::cout<<"done"<<std::endl;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Saving iT    
    
    // Some file names to save data
    // Some of the data saved in this file is only kept
    // to not break previous code but for most recent
    // version is not used.
    
    //Check a path exists
    std::string aff_path = this->affordanceName + "/";
    if (!boost::filesystem::exists( this->affordanceName.c_str() ) )
    {
        std::cout << "New affordance? Creating dir -> " <<aff_path<< std::endl;
        //std::string command="mkdir "+aff_path;
        //command=exec(command.c_str());
        boost::filesystem::path dir(aff_path);
		boost::filesystem::create_directory(dir);
    }
    else
    {
        std::cout << "Found affordance dir -> " <<aff_path<< std::endl;
    }


    // Some data for info file
    // Closest point in Tensor to scene and in object to scene
    // were used previously to estimate the pose of Tensor/Object
    // relative to scene. These are still computed and save but no longer
    // used, the new pose is computed using center of bounding boxes.
    
    std::cout<<"Getting closest point in Tensor to scene"<<std::endl;
    //TODO resolver como se ajustan estas relaciones, de "PUNTOS DE REFERENCIA"
    pcl::PointXYZ p_scene;
    int idxScene = Util_iT::indexOfClosestPointInACloud ( sceneCloudFiltered, middlePointObject );
    p_scene      = sceneCloudFiltered->at(idxScene);
    int one      = Util_iT::indexOfClosestPointInACloud ( ibsFiltered, p_scene); //"ClosestTensor"
    
    std::cout<<"Getting closest from object to scene"<<std::endl;
    int two  = Util_iT::indexOfClosestPointInACloud ( objectCloud, p_scene );
    int three= Util_iT::indexOfClosestPointInACloud ( sceneCloudFiltered, p_scene ); //"ClosestSCN");
    Eigen::Vector3f toIBS,toObject;
    toIBS[0]=ibsFiltered->at(one).x-p_scene.x;
    toIBS[1]=ibsFiltered->at(one).y-p_scene.y;
    toIBS[2]=ibsFiltered->at(one).z-p_scene.z;
    toObject[0]=objectCloud->at(two).x-p_scene.x;
    toObject[1]=objectCloud->at(two).y-p_scene.y;
    toObject[2]=objectCloud->at(two).z-p_scene.z;

    // Info file name
    {
    std::string file_name= aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + ".txt";
    std::ofstream output_file(file_name.c_str());
    int clusters=1;
    float size=0;

    // Previously some clustering was performed over IBS points to estimate
    // clusters, only one (referencing=single) of this clusters was then used to estimate poses but
    // we dropped that. Data is still saved.
    std::string referencing="Single";

    //Start saving to file
    if(output_file.is_open())
    {
        std::string scene_name = "table";
        output_file<<"Scene name:"<<scene_name<<"\n"; //TODO This could be unnecesary
        output_file<<"Object name:"<<this->objectName<<"\n";
        output_file<<"Clusters:"<<clusters<<"\n";
        for(int i=0;i<clusters;i++)
            output_file<<ibsFiltered->at(one).x<<","<<ibsFiltered->at(one).y<<","<<ibsFiltered->at(one).z<<"\n";
        output_file<<"Distance threshold:"<<size<<"\n";
        output_file<<"Reference:"<<referencing<<"\n";
        output_file<<one<<":"<<ibsFiltered->at(one).x<<","<<ibsFiltered->at(one).y<<","<<ibsFiltered->at(one).z<<"\n";
        output_file<<"ScenePoint\n";
        output_file<<three<<":"<<sceneCloudFiltered->at(three).x<<","<<sceneCloudFiltered->at(three).y<<","<<sceneCloudFiltered->at(three).z<<"\n";
        output_file<<"IbsPointVector\n";
        output_file<<one<<":"<<toIBS[0]<<","<<toIBS[1]<<","<<toIBS[2]<<"\n";
        output_file<<"ObjPointVector\n";
        output_file<<two<<":"<<toObject[0]<<","<<toObject[1]<<","<<toObject[2]<<"\n";
        output_file<<"Object Transformation\n";
        //TODO this is unnecesary
//         output_file<<centroidObjFinal[0]-centroidObjInit[0]<<","<<centroidObjFinal[1]-centroidObjInit[1]<<","<<centroidObjFinal[2]-centroidObjInit[2]<<"\n";
//         output_file<<ob_angle<<"\n"; 
//         output_file<<"Object Transformation Box\n";
//         output_file<<box2.x-box1.x<<","<<box2.y-box1.y<<","<<box2.z-box1.z<<"\n";
//         output_file<<"SceneToBoxCentroid\n";
//         output_file<<box2.x-p_scene.x<<","<<box2.y-p_scene.y<<","<<box2.z-p_scene.z<<"\n";
    }
    else
    {
        std::cout<<"Problem with data file "<<std::endl;
    }
    output_file.close();
    }
    
    // Scene-to-IBS and Scene-to-object are saved in affordance keypoints file
    // As commented earlier it was used to align pointclouds
    // at test time. No longer used but still kept in files.
    PointWithVector secondtolast;
    secondtolast.x=toIBS[0];
    secondtolast.y=toIBS[1];
    secondtolast.z=toIBS[2];
    secondtolast.v1=one;
    secondtolast.v2=secondtolast.v3=0;
    
    
    PointWithVector last;
    last.x=toObject[0];
    last.y=toObject[1];
    last.z=toObject[2];
    last.v1=two;
    last.v2=last.v3=0;

    
    new_sampleCloud2->push_back(secondtolast);
    new_sampleCloud2->push_back(last);
    
    new_sampleCloudU->push_back(secondtolast);
    new_sampleCloudU->push_back(last);
    
    
    // Save everything
    std::string new_ibs_field   = aff_path + this->affordanceName + "_" + this->objectName + "_field.pcd";
    std::string new_ibs_sample  = aff_path + "ibs_sample_512_" + this->affordanceName + "_" + this->objectName + "_better.pcd";
    std::string new_ibs_sampleU = aff_path + "ibs_sample_512_" + this->affordanceName + "_" + this->objectName + "_betterUniform.pcd";
    
    std::string smoother_field  = aff_path + this->affordanceName + "_" +this->objectName + "_smoothfield.pcd";
    std::string clean_ibs       = aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + "_clean.pcd";  // TODO It doesn't have any sense
    std::string full_ibs        = aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + ".pcd";
    
    pcl::io::savePCDFileASCII( new_ibs_field.c_str(), *field);
    pcl::io::savePCDFile( new_ibs_sample.c_str(), *new_sampleCloud2);
    pcl::io::savePCDFile( new_ibs_sampleU.c_str(), *new_sampleCloudU);
    
    pcl::io::savePCDFile( smoother_field.c_str(), *smoothField);
    pcl::io::savePCDFile( clean_ibs, *copyIBS);
    pcl::io::savePCDFile( full_ibs, *ibsFiltered);
    
    std::cout<<"Done and saved as "<<new_ibs_field<<std::endl;
    
    
 
    // By default compute spin cloud for 8 orientations
    // Can be changed and passed as parameter
    // int n_orientations=8;
    // It was simpler to compute and store the descriptor for X-orientations than
    // for 1 orientation and then rotate X-times at test time.
    // So we compute this X-orientations and store them for testing.

    // Spin cloud for weight-sampled
    createSpin( new_sampleCloud2, ibsFiltered, aff_path );
    // New representation for agglomerative descriptor
    // In following release, multiple affordaces can be detected
    // at same time, single affordance representation (this code)
    // is adapated to work with newer code. This "adaptation" is
    // basically wrap (or format) the descriptor in a highly
    // parallelizble way.

    if(getAggloRepresentation(mags_c,aff_path))
    {
        std::cout<<"Everything ok"<<std::endl;
    }
    
    // Spin cloud for uniform sampled
    createSpin(new_sampleCloudU,ibsFiltered,aff_path,8,true);
    if(getAggloRepresentation(mags_cU,aff_path,true))
    {
        std::cout<<"Everything ok"<<std::endl;
    }
    
}


 std::vector<float> IT::getSamplingProbabilities(pcl::PointCloud<pcl::PointNormal>::Ptr clout_in, float minV, float maxV){
	 std::vector<float> probs( clout_in->size() );
	 
	 for(int i=0;i<clout_in->size();i++)
	 {
	   Eigen::Vector3f oldNormal( clout_in->at(i).normal_x, clout_in->at(i).normal_y, clout_in->at(i).normal_z);
	   probs.at(i) = Util_iT::getValueProporcionsRule( oldNormal.norm(), minV, maxV, 1, 0);
	 }
	 return probs;
	  
 }


bool IT::getAggloRepresentation(std::vector<float> &mags, std::string pathh, bool uniform)
{

    int sampleSize=vectors.rows();
    PointCloudT::Ptr aux_cloud(new PointCloudT);
    aux_cloud->resize(descriptor.rows());
    PointCloudT::Ptr useful_cloud(new PointCloudT);
    useful_cloud->resize(descriptor.rows());
    PointCloudT::Ptr better_approx(new PointCloudT);
    better_approx->resize(descriptor.rows());
    PointCloudT::Ptr bare_points(new PointCloudT);
    bare_points->resize(descriptor.rows());
    PointCloudT::Ptr vector_ids_agglomerative(new PointCloudT);
    vector_ids_agglomerative->resize(descriptor.rows());
    PointCloudT::Ptr vectors_data_cloud(new PointCloudT);
    vectors_data_cloud->resize(descriptor.rows());
    // point counts per affordance per orientation
    // Mostly useful for normalization in multiple affordance prediction
    // for single affordance case: 1x8 matrix with sampleSize in each element
    int n_orientations=descriptor.rows()/vectors.rows();
    Eigen::MatrixXf data_individual(1,n_orientations);
    data_individual<<Eigen::MatrixXf::Zero(1,n_orientations);


    for(int i=0;i<descriptor.rows();i++)
    {
        int orientation_id=std::floor(i/sampleSize); // [0-nOrientations) default: 8 orientations
        data_individual(0,orientation_id)+=1;
        int smaller_id=i-(sampleSize*orientation_id); // [0-sampleSize)
        aux_cloud->at(i).x=1;
        aux_cloud->at(i).y=i;
        aux_cloud->at(i).z=0;

        useful_cloud->at(i).x=1;
        useful_cloud->at(i).y=orientation_id;
        useful_cloud->at(i).z=smaller_id;

        better_approx->at(i).x=bare_points->at(i).x=descriptor(i,0);
        better_approx->at(i).y=bare_points->at(i).y=descriptor(i,1);
        better_approx->at(i).z=bare_points->at(i).z=descriptor(i,2);

        vector_ids_agglomerative->at(i).x=vectors(smaller_id,0);
        vector_ids_agglomerative->at(i).y=vectors(smaller_id,1);
        vector_ids_agglomerative->at(i).z=vectors(smaller_id,2);

        Eigen::Vector3f aVector=vectors.row(smaller_id);
        vectors_data_cloud->at(i).x=aVector.norm();
        vectors_data_cloud->at(i).y=mags.at(smaller_id);
        vectors_data_cloud->at(i).z=0;
    }
    std::cout<<"Point counts "<<data_individual<<std::endl;
    std::string base_name;
    std::stringstream ii;
    ii<<n_orientations;
    // Save everything with a correct name
    // if uniform sampling or different
    if(uniform)
        base_name = pathh + "UNew_"+ this->affordanceName + "_" + this->objectName + "_descriptor_" + ii.str();
    else
        base_name = pathh + "New_" + this->affordanceName + "_" + this->objectName + "_descriptor_" + ii.str();
    
    std::string file_name=base_name+"_members.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*aux_cloud);
    
    file_name=base_name+"_extra.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*useful_cloud);
    
    file_name=base_name+".pcd";
    pcl::io::savePCDFile(file_name.c_str(),*better_approx);
    
    file_name=base_name+"_points.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*bare_points);
    
    file_name=base_name+"_vectors.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*vector_ids_agglomerative);
    
    file_name=base_name+"_vdata.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*vectors_data_cloud);
    
    file_name=pathh+this->affordanceName+"_"+this->objectName+"_point_count.dat";
    std::ofstream ofs (file_name, std::ofstream::out);
    pcl::saveBinary(data_individual,ofs);
    return true;
}

//TODO Erase me, this way of calculating translations is not necesary
void IT::translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation){//TODO  no comprendo por que se toma como referencia el cetroide de la nube de puntos
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*in,centroid);
    
    pcl::PointXYZ reference;
    reference.getVector4fMap() = centroid;
    
    translateCloud( in,  out,  translation,  reference);
    
//     transform.translation() << translation.x-centroid[0],translation.y-centroid[1],translation.z-centroid[2];
//     transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
//     pcl::transformPointCloud (*in, *out, transform);
}
     
void IT::translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation, pcl::PointXYZ reference){
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    transform.translation() << translation.x-reference.x,translation.y-reference.y,translation.z-reference.z;
    transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*in, *out, transform);
}


//TODO origin is always setted as true some sections in this code is unreachable
void IT::rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,bool origin){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*cloud_in, centroid);
    Eigen::Vector3f ax;
    switch(axis)
    {
    case 'x': ax = Eigen::Vector3f::UnitX();
        break;
    case 'y': ax = Eigen::Vector3f::UnitY();
        break;
    case 'z': ax = Eigen::Vector3f::UnitZ();
        break;
    }
    Eigen::Matrix3f rotation(Eigen::AngleAxisf (angle, ax));
    transform.rotate(rotation);
    transform.translation()<<0,0,0;
    pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
    if (!origin)   //TODO This condition is never reached, it is not necesary this way of calculating translations
    {
        Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*cloud_out, centroid_new);
        //Eigen::Vector4f diff=centroid-centroid_new; //unused variable
        translateCloud(cloud_out,cloud_out,pcl::PointXYZ(centroid[0],centroid[1],centroid[2]));
    }
    
}
    
    
void IT::rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot){
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
    Eigen::Vector3f ax;
    //pcl::PointXYZ pivot;
    int id=cloud_in->size();
    cloud_in->push_back(pivot);
    //pivot.x=cloud_in->at(pivot_id).x;
    //pivot.y=cloud_in->at(pivot_id).y;
    //pivot.z=cloud_in->at(pivot_id).z;
    switch(axis)
    {
    case 'x':ax=Eigen::Vector3f::UnitX();
        break;
    case 'y':ax=Eigen::Vector3f::UnitY();
        break;
    case 'z':ax=Eigen::Vector3f::UnitZ();
        break;
    }
    Eigen::Matrix3f rotation(Eigen::AngleAxisf (angle, ax));
    transform.rotate(rotation);
    transform.translation()<<0,0,0;
    pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
    pcl::PointXYZ newPivot=cloud_out->at(id);
    //newPivot.x=cloud_out->at(pivot_id).x;
    //newPivot.y=cloud_out->at(pivot_id).y;
    //newPivot.z=cloud_out->at(pivot_id).z;
    translateCloud(cloud_out,cloud_out,pivot,newPivot);
    PointCloud::iterator iter=cloud_out->end();
    --iter;
    cloud_out->erase(iter);
    
}

void IT::getSpinMatrix(pcl::PointCloud<PointWithVector>::Ptr sample, int orientations, pcl::PointCloud<pcl::PointXYZ>::Ptr full){
    
    pcl::PointCloud<PointWithVector>::iterator iter=sample->end();
    PointCloudT::Ptr relativePoints(new PointCloud);
    PointWithVector p=sample->at(sample->size()-2);
    pcl::PointXYZ refPointIBS(p.x,p.y,p.z);
    int refPointIBSId=int(p.v1);
    std::cout<<"ref: "<<refPointIBSId<<std::endl;
    pcl::PointXYZ ref(full->at(refPointIBSId).x-refPointIBS.x,full->at(refPointIBSId).y-refPointIBS.y,full->at(refPointIBSId).z-refPointIBS.z);
    //pcl::PointXYZ ref2=full->at(refPointIBSId);//unused variable

    //pcl::PointXYZ actual_ref()
    --iter;
    sample->erase(iter);
    --iter;
    sample->erase(iter);

    descriptor.resize(sample->size()*orientations,3);
    vectors.resize(sample->size(),3);
    PointCloud::Ptr xyz_target(new PointCloud);
    std::cout<<"REf: "<<ref<<std::endl;
    for(int j=0;j<sample->size();j++)
    {
        PointWithVector aP;
        aP.x=sample->at(j).x-ref.x;
        aP.y=sample->at(j).y-ref.y;
        aP.z=sample->at(j).z-ref.z;
        relativePoints->push_back(pcl::PointXYZ(aP.x,aP.y,aP.z));
        //descriptor.row(j)=aP.getVector3fMap();
        Eigen::Vector3f v(sample->at(j).v1,sample->at(j).v2,sample->at(j).v3);
        vectors.row(j)=v;
        //mags.at(j)=(v.norm()-v.norm()*.2)*(v.norm()-v.norm()*.2);
        //lengths.at(j)=1-((v.norm()- minW) * (1 - 0) / (maxW - minW) + 0);
        //xyz_target->push_back(pcl::PointXYZ(aP.x,aP.y,aP.z));
    }
    PointCloudC::Ptr anchor(new PointCloudC);
    pcl::PointXYZRGB coloredanchor(0,255,0);
    coloredanchor.x=ref.x;
    coloredanchor.y=ref.y;
    coloredanchor.z=ref.z;
    anchor->push_back(coloredanchor);
    //viewer->addPointCloud(anchor,"Anchor");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"Anchor");
    PointCloud::Ptr spinCloud(new PointCloud);
    PointCloud::Ptr relative_spin(new PointCloud);
    PointCloud::Ptr xyz_2(new PointCloud);
    pcl::copyPointCloud(*sample,*xyz_target);
    pcl::copyPointCloud(*xyz_target,*spinCloud);
    pcl::copyPointCloud(*relativePoints,*relative_spin);
    std::cout<<"Spining "<<xyz_2->size()<<" points"<<std::endl;
    //if(viewer->contains("Spincloud"))
    //  viewer->updatePointCloud(relative_spin,"Spincloud");
    //else
    //  viewer->addPointCloud(relative_spin,"Spincloud");
    //while(!viewer->wasStopped())
    //        viewer->spinOnce(100);
    //    viewer->resetStoppedFlag();

    for (int i=1;i<orientations;i++)
    {
        pcl::copyPointCloud(*sample,*xyz_2);
        rotateCloud(xyz_2,xyz_target, i*2*M_PI/orientations,'z',ref);
        *spinCloud+=*xyz_target;
        pcl::copyPointCloud(*relativePoints,*xyz_2);
        rotateCloud(xyz_2,xyz_target, i*2*M_PI/orientations,'z',true);
        *relative_spin+=*xyz_target;
        //std::cout<<"Spincloud: "<<xyz_target->size()<<std::endl;
        //if(viewer->contains("Spincloud"))
        //            viewer->updatePointCloud(relative_spin,"Spincloud");
        //        else
        //            viewer->addPointCloud(relative_spin,"Spincloud");
        //        while(!viewer->wasStopped())
        //            viewer->spinOnce(100);
        //        viewer->resetStoppedFlag();

    }
    for(int i=0;i<spinCloud->size();i++)
        descriptor.row(i)=Eigen::Vector3f(relative_spin->at(i).x,relative_spin->at(i).y,relative_spin->at(i).z);
    std::cout<<"Descriptor "<<descriptor.rows()<<std::endl;
}

bool IT::createSpin( pcl::PointCloud<PointWithVector>::Ptr sample,  pcl::PointCloud<pcl::PointXYZ>::Ptr full_ibs, std::string pathh,int orientations,  bool uniform){
    std::stringstream ii;
    ii<<orientations;
    getSpinMatrix(sample,orientations,full_ibs);
    std::string spin_file;
    std::string spinvectors_file;
    if(uniform)
    {
        spin_file        = pathh + this->affordanceName + "_" + this->objectName + "_spinU_" + ii.str() + ".dat";
        spinvectors_file = pathh + this->affordanceName + "_" + this->objectName + "_spinUvectors_" + ii.str() + ".dat";
    }
    else
    {
        spin_file        = pathh + this->affordanceName + "_" + this->objectName + "_spin_"+ii.str()+".dat";
        spinvectors_file = pathh + this->affordanceName + "_" + this->objectName + "_spinvectors_"+ii.str()+".dat";
    }
    std::ofstream file(spin_file.c_str());
    pcl::saveBinary(descriptor,file);
    std::ofstream file2(spinvectors_file.c_str());
    pcl::saveBinary(vectors,file2);
    std::cout<<"Wrote: "<<spin_file<<" and "<<spinvectors_file<<std::endl;
    //if reached this point everything is ok
    return true;
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
}
