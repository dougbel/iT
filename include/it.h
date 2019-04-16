#ifndef IT_H
#define IT_H

#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include <limits>



#include "ibs.h"
#include "util_it.h"


/**
 * @todo write docs
 */
class IT
{
public:
    IT(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object);
    
    void calculate();
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr getIT();
    
//     void setROI ( pcl::PointXYZ pivotPoint, float radio );
        
//     bool isSet_ROI();
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr bigCloud;
    
    
    
private:
    
    
//     pcl::PointXYZ center_ROI;
//     float radio_ROI;
//     bool is_ROI;
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr soi_sceneCloud;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr soi_queryObjectCloud;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr soi_bigCloud;
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr sphereExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in , pcl::PointXYZ point_pivot, float radio);

};

#endif // IT_H
