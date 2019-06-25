#ifndef IT_TESTING_H
#define IT_TESTING_H

#include <iostream>
#include <string>
#include <ctime>

#include <Eigen/Core>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <pcl/octree/octree.h>
#include <pcl/filters/random_sample.h>
// For GPU search/containers
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/console/time.h>

#include "agglomerator_it.h"
#include "util_it.h"

#include "Testing/dev_array.h"
#include "Testing/myCuda.h"



class IT_Testing
{
public:
    IT_Testing(std::string path, std::string paremeters_file);
    
    std::string testInteractions(std::string scn_name, pcl::PointCloud<pcl::PointXYZ>::Ptr whole_scene);
    
    std::string saveClouds(std::string scn_name, std::string affordance_name, std::string ob_name, pcl::PointCloud<pcl::PointXYZ>::Ptr sample ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr data);
    
    bool toTest = false;
    
    //Interaction informaction
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_clouds;
    std::vector <Agglomerator_IT> interactions;
    std::vector <float> object_diags_size;
    std::vector <std::string> affordance_names;
    std::vector <std::string> aff_paths;
    std::vector <std::string> object_names;
    
private:
    
    
    
    // Estimate test vectors and compute score using cuda
    std::vector<float> computeScore(pcl::PointCloud<pcl::PointXYZ>::Ptr local_nn,float *samplePoint, dev_array<float> & kpData,dev_array<float> & pvData,dev_array<float> & wData,dev_array<float> & mData, dev_array<int> & ppC, dev_array<int> & startppC,dev_array<float> & ppCData);
    
    // Get all the good predictions per test-point. Used when text-only mode is enabled.
        
    int getMinMaxMulti(std::vector<float>& bigScores, std::vector <float> &score, std::vector<int> & orientation, std::vector<int> & aff_id, float thresholds, pcl::PointCloud< pcl::PointXYZ >::Ptr data,pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudToSave);
    
    
    void setSample();
    
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
  
   
  
};

#endif // IT_TESTING_H
