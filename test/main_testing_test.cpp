
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include "it.h"
#include "Testing/it_testing.h"


void distanceXYZRGBClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud2, float &points_distance, float &rgb_distance ){
    
    rgb_distance =0;
    points_distance = 0;
   
     // compare A to B
    pcl::search::KdTree<pcl::PointXYZRGB> tree_b;
    tree_b.setInputCloud( (*cloud2).makeShared() );
    
    for(int i =0; i < cloud1->size(); i++){
        std::vector<int> index (1);
        std::vector<float> sqr_distances (1);

        tree_b.nearestKSearch (cloud1->points[i], 1, index, sqr_distances);
                
        points_distance += sqr_distances[0];
        
        rgb_distance += Eigen::Vector3f(  cloud1->points[i].r - cloud2->points[ index[0] ].r, 
                                            cloud1->points[i].g - cloud2->points[ index[0] ].g, 
                                            cloud1->points[i].b - cloud2->points[ index[0] ].b ).norm();
    }
    
    
    
    
}


int main(int argc, char *argv[])
{
   float distance;
   float distance_rgb;
   std::string file_name;
   
   std::string time_stamp;
   
   
    IT_Testing testing("../../test/testing_data/for_testing", "01_testing.json");
    testing.toTest = true;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene( new pcl::PointCloud<pcl::PointXYZ> );
    std::string file_scene = "../../test/testing_data/scenes/kitchen5_d.pcd";
    
    pcl::io::loadPCDFile(file_scene, *scene );
    
    time_stamp = testing.testInteractions("kitchen",scene);
    
    
    
    
    
    
    //////////////////////  STARTING TESTING STAGE
   
    
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr goodPoints_precalculated(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr goodPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
   
   pcl::io::loadPCDFile("../../test/testing_data/for_testing/01_testing_goodPoints.pcd", *goodPoints_precalculated);
  
   file_name = "../../test/testing_data/for_testing/output/"+time_stamp+"_goodPoints.pcd";
   pcl::io::loadPCDFile(file_name, *goodPoints);
   
   distanceXYZRGBClouds(goodPoints_precalculated, goodPoints, distance, distance_rgb );
   
   
   
   std::cout << "Distance: [" << "Points: "  <<  distance << ", RGB: "  << distance_rgb << " ]" << std::endl;
    
    if ( distance >= 0.0001 || distance_rgb >= 0.0001){
        std::cout << "TEST Failure on goodPoints!!!"<<std::endl<<std::endl;
        //return EXIT_FAILURE;
    }
    else{
        std::cout << "TEST on goodPoints SUCCESS!" <<std::endl <<std::endl;
    }
   
   
   
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr goodPointsX_precalculated(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr goodPointsX(new pcl::PointCloud<pcl::PointXYZRGB>);
   
   pcl::io::loadPCDFile("../../test/testing_data/for_testing/01_testing_goodPointsX.pcd", *goodPointsX_precalculated);
   
   file_name  = "../../test/testing_data/for_testing/output/"+time_stamp+"_goodPointsX.pcd";
   pcl::io::loadPCDFile(file_name, *goodPointsX);
   
   distanceXYZRGBClouds(goodPointsX_precalculated, goodPointsX, distance, distance_rgb );
  
   
   
   std::cout << "Distance: [" << "Points: "  <<  distance << ", RGB: "  << distance_rgb << " ]" << std::endl;
    
    if ( distance >= 0.0001 || distance_rgb >= 0.0001){
        std::cout << "TEST Failure on goodPointsX!!!"<<std::endl<<std::endl;
        //return EXIT_FAILURE;
    }
    else{
        std::cout << "TEST on goodPointsX SUCCESS!" <<std::endl <<std::endl;
    }
    
    
   
   
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr samplePoints_precalculated(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr samplePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
   
   pcl::io::loadPCDFile("../../test/testing_data/for_testing/01_testing_samplePoints.pcd", *samplePoints_precalculated);
   
   file_name = "../../test/testing_data/for_testing/output/"+time_stamp+"_samplePoints.pcd";
   pcl::io::loadPCDFile( file_name, *samplePoints);
   
   distanceXYZRGBClouds(samplePoints_precalculated, samplePoints, distance, distance_rgb );
   
   
   
   std::cout << "Distance: [" << "Points: "  <<  distance << ", RGB: "  << distance_rgb << " ]" << std::endl;
    
    if ( distance >= 0.0001 || distance_rgb >= 0.0001){
        std::cout << "TEST Failure on samplePoints!!!"<<std::endl<<std::endl;
        //return EXIT_FAILURE;
    }
    else{
        std::cout << "TEST on samplePoints SUCCESS!" <<std::endl <<std::endl;
    }
   
   
   
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr alldata_precalculated(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr alldata(new pcl::PointCloud<pcl::PointXYZRGB>);
    
   pcl::io::loadPCDFile("../../test/testing_data/for_testing/01_testing_Place_bowl_kitchen5_d.pcd", *alldata_precalculated);
   
   file_name = "../../test/testing_data/for_testing/output/"+time_stamp+"_Place_bowl_kitchen.pcd";
   pcl::io::loadPCDFile( file_name, *alldata);
   
   distanceXYZRGBClouds(alldata_precalculated, alldata, distance, distance_rgb );
   
   
   
   std::cout << "Distance: [" << "Points: "  <<  distance << ", RGB: "  << distance_rgb << " ]" << std::endl;
    
    if ( distance >= 0.0001 || distance_rgb >= 0.0001){
        std::cout << "TEST Failure on alldata!!!"<<std::endl<<std::endl;
        //return EXIT_FAILURE;
    }
    else{
        std::cout << "TEST on alldata SUCCESS!" <<std::endl <<std::endl;
    }
    
    std::cout << "ERASE  ../../test/testing_data/for_testing/output   TO AVOID CONFUSIONS" << std::endl;
    
    return EXIT_SUCCESS;
}
