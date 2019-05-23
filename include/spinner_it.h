
#ifndef SPINNER_IT_H
#define SPINNER_IT_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include "type_point_it.h"
#include "definitions.h"

/**
 * @todo write docs
 */
class Spinner_iT
{
public:
    
    pcl::PointCloud<PointWithVector>::Ptr sample;    
    pcl::PointXYZ spiningPoint;
    int orientations;
    
        //SPINNING SECTION
    // Definition of Matrix in Eigen
    //  Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    // Eigen::RowMajor is the storage order   ===>> Normaly Eigen saves them colum by colum
    // These are matrixs with n number of rows, 3 colums that are stored in memory row by row
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> descriptor; //TODO this variables are used to calculate the spin of iT
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> vectors;    //TODO this variables are used to calculate the spin of iT
    
    /**
     * @todo write docs
     */
    Spinner_iT(pcl::PointCloud<PointWithVector>::Ptr sample, pcl::PointXYZ spiningPoint, int orientations);
    
    void calculateSpinings();

private:
    void rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,bool origin);
    
    void rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot);
   
    void translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in,  pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation);
     
    void translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in,  pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation, pcl::PointXYZ reference);
};

#endif // SPINNER_IT_H
