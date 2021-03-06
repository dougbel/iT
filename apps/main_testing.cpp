
#include <iostream>
#include <assert.h>  

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include "it.h"
#include "Testing/it_testing.h"


int main(int argc, char *argv[])
{

    
    IT_Testing testing("../../test/testing_data", "testing.json");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene( new pcl::PointCloud<pcl::PointXYZ> );
    std::string file_scene = "../../test/testing_data/scenes/kitchen5_d.pcd";
    
    pcl::io::loadPCDFile(file_scene, *scene );
    
    testing.testInteractions("kitchen",scene);

   
   
   //std::cout << argv[0] << " - " << argv[1] << " - " << argv[2] << " - " << argv[3] << std::endl;
   
   
    return EXIT_SUCCESS;
}
