#include "it.h"



IT::IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object){
       
    sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    objectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
     // to compute voronoi diagram over all points
    this->sceneCloud        = scene;
    this->objectCloud  = object;
    

}


void IT::calculate(){
    
    pcl::PointXYZ min,max,middlePointObject;
    pcl::getMinMax3D(*objectCloud,min,max);
    
    middlePointObject.x=(max.x+min.x)/2;
    middlePointObject.y=(max.y+min.y)/2;
    middlePointObject.z=(max.z+min.z)/2;
    
    float radio=pcl::L2_Norm(min.getArray3fMap(),max.getArray3fMap(),3);
    
    

    IBS ibs_calculator(sceneCloud,objectCloud);
    ibs_calculator.setROI ( middlePointObject, radio);
    ibs_calculator.calculate();
}

// void IT::setROI ( pcl::PointXYZ pivotPoint, float radio ) {
//     
//     this->center_ROI = pivotPoint;
//     this->radio_ROI = radio;
//     
//     this->soi_bigCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
//     
//     this->soi_sceneCloud = Util::sphereExtraction(sceneCloud , pivotPoint, radio);
//     this->soi_queryObjectCloud = Util::sphereExtraction(queryObjectCloud , pivotPoint, radio);
//     
//     pcl::copyPointCloud( *this->soi_queryObjectCloud, *this->soi_bigCloud );
//     
//     *this->soi_bigCloud += *this->soi_sceneCloud;
//     
//     this->is_ROI = true;
// }
// 
// 
// 
// 
// pcl::PointCloud<pcl::PointXYZ>::Ptr IT::getIBS(){
//    
// 
// }
