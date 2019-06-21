#ifndef IT_TESTING_H
#define IT_TESTING_H


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <Eigen/Core>

#include <iostream>
#include <string>

#include <agglomerator_it.h>
#include <it.h>

class IT_Testing
{
public:
    IT_Testing(std::string path, std::string paremeters_file);
    
    
    void testInteractions(pcl::PointCloud<pcl::PointXYZ> whole_scene);
    
    
    std::vector <Agglomerator_IT> interactions;
    
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
  
  
  
  //Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeVectors;
  //Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeDescriptor;
  

  
  
  
  
  
};

#endif // IT_TESTING_H
