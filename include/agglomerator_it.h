#ifndef AGGLOMERATOR_IT_H
#define AGGLOMERATOR_IT_H

#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>

class Agglomerator_IT
{
public:
    
    Agglomerator_IT(Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> vectors , 
                    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> descriptor,
                    std::vector<float> mags );
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud;      //members
    pcl::PointCloud<pcl::PointXYZ>::Ptr useful_cloud;   //extra
    pcl::PointCloud<pcl::PointXYZ>::Ptr better_approx;  //descriptor
    pcl::PointCloud<pcl::PointXYZ>::Ptr bare_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr vector_ids_agglomerative;
    pcl::PointCloud<pcl::PointXYZ>::Ptr vectors_data_cloud; //vdata
    
    Eigen::MatrixXf data_individual;
    
    int sampleSize;
    int n_orientations;
        
    void compileAgglomeration();
    static Agglomerator_IT loadFiles(std::string path, std::string affordance_name, std::string object_name, int sampleSize);
    
private:
    
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> vectors;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> descriptor;
    
    std::vector<float> mags;
        
    
    Agglomerator_IT(){};
    
    
    
    
};

#endif // AGGLOMERATOR_IT_H
