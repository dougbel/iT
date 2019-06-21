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
    
    Agglomerator_IT(Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> vectors ,         //this has sampleSize rows
                    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> descriptor,       //this has sampleSize*numOrientations
                    std::vector<float> pv_norms ,
                    std::vector<float> pv_norms_mapped  );
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud;      //members
    pcl::PointCloud<pcl::PointXYZ>::Ptr useful_cloud;   //extra
    pcl::PointCloud<pcl::PointXYZ>::Ptr better_approx;  //descriptor
    pcl::PointCloud<pcl::PointXYZ>::Ptr bare_points;    //points
    pcl::PointCloud<pcl::PointXYZ>::Ptr vector_ids_agglomerative; //_vector
    pcl::PointCloud<pcl::PointXYZ>::Ptr vectors_data_cloud; //vdata
    
    Eigen::MatrixXf data_individual;            //this matrix values are (sampleSize, sampleSize, sampleSize, ... , sampleSize) as many as numOrientations
    
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeVectors;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> vectors;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> descriptor;
    std::vector<int> ppCentroid;
    std::vector<int> startppCentroid;
    std::vector<float> large_mags;        //provenance vector NORM of the point asociated in the sample
    std::vector<float> large_lengths;     //MAPPED provenance vector NORM to [1, 0]
    float alternative_data_counts;        // NOOOOO SÉ, en el codigo original se trata de la suma de mags
    
    std::vector<float> pvNormsMapped;
    std::vector<float> pvNorms;
    
    float sumMags;
    
    int sampleSize;
    int n_orientations;
        
    void compileAgglomeration();
    static Agglomerator_IT loadFiles(std::string path, std::string affordance_name, std::string object_name, int sampleSize,  int numOrientations);
    
private:
    
        
    
    
    
    
};

#endif // AGGLOMERATOR_IT_H
