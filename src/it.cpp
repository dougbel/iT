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
    
//     std::vector<std::vector<int> > allNeighbors(ibsFiltered->size(),std::vector<int>(knn,1)); this variable is not used
    
    
    std::vector<int> bad_ids;
    // Try to ingnore "bad vectors" which would be something very large (silly)
    //int somethingSilly=1000; // TODO this must change
    int somethingSilly = 0;
    
    std::cout<<"Checking for normals_z greater than "<<somethingSilly<<std::endl;
    
    for(int i=0;i<ibsFiltered->size();i++)
    {
        
//         pcl::PointCloud<pcl::PointXYZ>::Ptr     disp_n(new PointCloud); //this variable is not used
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr  disp_centre(new PointCloudC); //this variable is not used
        
        pcl::PointXYZ ibs_point = ibsFiltered->at(i);
        
        int num_neighbours = kdtreeSCN.nearestKSearch(ibs_point,knn,NNidSC,NNdistSC);
        
        if( num_neighbours > 0 )
        {
            
            Eigen::Vector3f resultant(0,0,0);
            Eigen::Vector3f scaled_v;
            
            for(int j=0;j<knn;j++)
            {
//                 allNeighbors.at(i).at(j) = NNidSC.at(j); // this variable is not used
                
                pcl::PointXYZ sc = sceneCloudFiltered->at( NNidSC.at(j) );
                
                Eigen::Vector3f component( sc.x-ibs_point.x,sc.y-ibs_point.y,sc.z-ibs_point.z );
                
                if(component[2]>somethingSilly && j==0){
                    
                    //this means the provenance vector goes in a different direction to the scene POSSIBLE penetration of the IBS
                    bad_ids.push_back(i);
//                     bad_flag=true;
                }
                resultant += component;
//                 disp_n->push_back(sc);
                if(j==0)
                {
                    // first NN is used for provenance vectors
//                     pcl::PointXYZRGB dp(0,255,0);
//                     dp.x=ibs_point.x;
//                     dp.y=ibs_point.y;
//                     dp.z=ibs_point.z;
//                     disp_centre->push_back(dp); // this variable is not used
                    pcl::PointNormal pn;
                    pn.x = ibs_point.x;
                    pn.y = ibs_point.y;
                    pn.z = ibs_point.z;
                    
//                     pcl::PointXYZ scp=sceneCloudFiltered->at(NNidSC.at(j));
//                     Eigen::Vector3f nVector(scp.x-ibs_point.x,scp.y-ibs_point.y,scp.z-ibs_point.z);
//                     scaled_v=nVector;
                    scaled_v = component;
                    
//                     if(i==0)  // I assigned extrem values to avoid this "if"
//                     {
//                         minV=maxV=nVector.norm();
//                     }
//                     else
//                     {
                    if(minV > component.norm())
                        minV = component.norm();
                    if(maxV < component.norm())
                        maxV = component.norm();
//                     }
                    sum += component.norm();
                    //nVector.normalize();
                    pn.normal_x = component[0];
                    pn.normal_y = component[1];
                    pn.normal_z = component[2];
                    field->push_back(pn);
                }
            }
            
            //"smmother" provenance vector
            scaled_v = scaled_v.norm() * resultant.normalized();
            smoothField->push_back(pcl::Normal(scaled_v[0],scaled_v[1],scaled_v[2]));
//             if(i==0)  // I assigned extrem values to avoid this "if"
//             {
//                 minS=maxS=resultant.norm();
//             }
//             else
//             {
                if( minS > resultant.norm() )
                    minS = resultant.norm();
                if( maxS < resultant.norm() )
                    maxS = resultant.norm();
//             }
            sumSmooth += scaled_v.norm();
        }
//         bad_flag=false;
    }
    
    

   

    
    
    
    std::cout << "Whole:    " << ibs_calculator.getIBS()->width << endl;
    std::cout << "Filtered: " << ibsFiltered->width << endl;    
}
