#include <iostream>
#include <assert.h>  

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include "it.h"


int main(int argc, char *argv[])
{
    //full scene point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_scene = "../../test/Place/scene_object.pcd";
    pcl::io::loadPCDFile(file_scene, *cloud_scene);
    
    //query object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_object = "../../test/Place/query_object.pcd";
    pcl::io::loadPCDFile(file_object, *cloud_object);
    
    
   IT itcalculator(cloud_scene,cloud_object);
    
    
    
    
    
    return EXIT_SUCCESS;
}
