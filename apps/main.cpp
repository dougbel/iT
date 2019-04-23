#include <iostream>
#include <assert.h>  

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include "it.h"
#include "StopWatch.h"


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
    
    
   IT itcalculator( cloud_scene, cloud_object, "Place", "bowl");
    
   itcalculator.calculate();
   /*  
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ibs_filtered (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPCDFile("./tmp/ibs_clouds_prefiltered_filtered.pcd", *cloud_ibs_filtered);
   
   pcl::PolygonMesh mesh ;
   
   StopWatch sw;
   mesh = Util_iT::meshFromPointCloud(cloud_ibs_filtered);
   std::cout << "tiempo 1 " << sw.ElapsedMs() << std::endl;
   
   
   pcl::io::savePLYFile ("mesh.ply", mesh);
  
   sw.Restart();
   mesh = Util_iT::meshFromPointCloudSlower(cloud_ibs_filtered);
   std::cout << "tiempo 2 " << sw.ElapsedMs() << std::endl; 
   
   pcl::io::savePLYFile ("meshslow.ply", mesh);*/
   
    return EXIT_SUCCESS;
}
