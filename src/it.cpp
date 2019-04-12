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
    
    std::cout << "saving meshes" << std::endl;
    pcl::io::savePCDFile("ibs_filtered.pcd",*ibsFiltered  );
    pcl::io::savePLYFile("ibs_filtered.ply", Util_iT::meshFromPointCloud(ibsFiltered)  );
    //pcl::io::savePCDFile("ibs.pcd",*ibs_calculator.getIBS()  );
    //pcl::io::savePLYFile("ibs.ply", Util_iT::meshFromPointCloud( ibs_calculator.getIBS() ) );
    
    /////////////////////////////////////////////////////////////////////////
    // Get provenance vectors
//     pcl::search::KdTree<pcl::PointXYZ> tree_b;
//     tree_b.setInputCloud (sceneCloudFiltered);
//     for (size_t i = 0; i < ibsFiltered->points.size (); ++i)
//     {
//         std::vector<int> indices (1);
//         std::vector<float> sqr_distances (1);
// 
//         tree_b.nearestKSearch (ibsFiltered->points[i], 1, indices, sqr_distances);
//         
//     }

    
    
    
    std::cout << "Whole:    " << ibs_calculator.getIBS()->width << endl;
    std::cout << "Filtered: " << ibsFiltered->width << endl;    
}
