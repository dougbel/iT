#include "it.h"



IT::IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object){
       
    sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    objectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
     // to compute voronoi diagram over all points
    this->sceneCloud        = scene;
    this->objectCloud  = object;
    

}


void IT::calculate(){
    
    /////////////////////////////////////////////////////////////////////////
    //calculate the Region of Interest (RoI) for further calculations
    //a sphere with radio equal to the diagonal of the box which surroundsthe object
    pcl::PointXYZ min, max, middlePointObject;
    float radio;
    
    pcl::getMinMax3D(*objectCloud,min,max);
    
    middlePointObject.x=(max.x+min.x)/2;
    middlePointObject.y=(max.y+min.y)/2;
    middlePointObject.z=(max.z+min.z)/2;
    
    radio=pcl::L2_Norm(min.getArray3fMap(),max.getArray3fMap(),3);
    
    
    
    /////////////////////////////////////////////////////////////////////////
    // extract volume of interest from the scene
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudFiltered;
    
    sceneCloudFiltered = Util::sphereExtraction(sceneCloud, middlePointObject,radio);

    

    /////////////////////////////////////////////////////////////////////////
    // calculate the Interaction Bisector Surface (IBS)
    IBS ibs_calculator(sceneCloudFiltered,objectCloud);
    ibs_calculator.calculate();
    //as IBS extend to infinity, filter IBS
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered;
    
    ibsFiltered = Util::sphereExtraction(ibs_calculator.getIBS(), middlePointObject,radio);
    

    
    /////////////////////////////////////////////////////////////////////////
    // Searching nearest neighbours 
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
    
    std::cout<<"Checking for normals_z greater than "<<somethingSilly<<std::endl;
    
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

    
    
    
    std::cout << "Whole:    " << ibs_calculator.getIBS()->width << endl;
    std::cout << "Filtered: " << ibsFiltered->width << endl;    
}
