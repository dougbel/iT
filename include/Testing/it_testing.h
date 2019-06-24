#ifndef IT_TESTING_H
#define IT_TESTING_H


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <Eigen/Core>

#include <iostream>
#include <string>

#include <agglomerator_it.h>
#include <util_it.h>

#include <pcl/octree/octree.h>
#include <pcl/filters/random_sample.h>

// For GPU search/containers
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/initialization.h>

#include "Testing/dev_array.h"



class IT_Testing
{
public:
    IT_Testing(std::string path, std::string paremeters_file);
    
    
    void testInteractions(pcl::PointCloud<pcl::PointXYZ>::Ptr whole_scene);
    
    
    std::vector <Agglomerator_IT> interactions;
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_clouds;
    
    std::vector <float> object_diags_size;
    
private:
    
    /**
     * @brief Directory of configurations and interaction descriptors
     * 
     */
    std::string path_data;
    /**
     * @brief Point cloud scene percentage to test
     * 
     */
    
    float sample_percentage; //a.k.a  sampleSize
  
  
  /**
   * @brief Stablish de debug level. 0: No debug, 1: vizualization, >2: full debug
   * 
   */
  int log_level;
  
  
  float agg_th;
  
  
  float pred_t;
  
  
  
  /**
   * @brief to divide the scene and search for interaction opportunities
   * 
   */
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree;
  
  
  //Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeVectors;
  //Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeDescriptor;
  

  
  
  
  
  
};

#endif // IT_TESTING_H
