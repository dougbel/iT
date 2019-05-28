#include "agglomerator_it.h"


Agglomerator_IT::Agglomerator_IT(Eigen::Matrix< float, Eigen::Dynamic, 3 , Eigen::RowMajor > vectors, 
                                 Eigen::Matrix< float, Eigen::Dynamic, 3 , Eigen::RowMajor > descriptor,
                                 std::vector<float> mags )
{
    this->vectors    = vectors;
    this->descriptor = descriptor;
    this->mags       = mags;
    
    this->sampleSize    =vectors.rows();
    this->n_orientations=descriptor.rows()/vectors.rows();
    
    this->aux_cloud    = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->useful_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->better_approx= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->bare_points  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->vector_ids_agglomerative = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->vectors_data_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // point counts per affordance per orientation
    // Mostly useful for normalization in multiple affordance prediction
    // for single affordance case: 1x8 matrix with sampleSize in each element
    this->data_individual =  Eigen::MatrixXf(1,n_orientations);
    this->data_individual << Eigen::MatrixXf::Zero(1,n_orientations);
}

void Agglomerator_IT::compileAgglomeration()
{
    
    aux_cloud->resize(descriptor.rows());
    useful_cloud->resize(descriptor.rows());
    better_approx->resize(descriptor.rows());
    bare_points->resize(descriptor.rows());
    vector_ids_agglomerative->resize(descriptor.rows());
    vectors_data_cloud->resize(descriptor.rows());
    
    
    for(int i=0;i<descriptor.rows();i++)
    {
        int orientation_id=std::floor(i/sampleSize); // [0-nOrientations) default: 8 orientations
        int smaller_id=i-(sampleSize*orientation_id); // [0-sampleSize)
        
        data_individual(0,orientation_id) += 1;
        
        aux_cloud->at(i).x = 1;
        aux_cloud->at(i).y = i;
        aux_cloud->at(i).z = 0;
        
        useful_cloud->at(i).x = 1;
        useful_cloud->at(i).y = orientation_id;
        useful_cloud->at(i).z = smaller_id;
        
        better_approx->at(i).x = descriptor(i,0);
        better_approx->at(i).y = descriptor(i,1);
        better_approx->at(i).z = descriptor(i,2);
        
        bare_points->at(i).x = descriptor(i,0);
        bare_points->at(i).y = descriptor(i,1);
        bare_points->at(i).z = descriptor(i,2);
        
        vector_ids_agglomerative->at(i).x = vectors(smaller_id,0);
        vector_ids_agglomerative->at(i).y = vectors(smaller_id,1);
        vector_ids_agglomerative->at(i).z = vectors(smaller_id,2);
        
        Eigen::Vector3f aVector = vectors.row(smaller_id);
        vectors_data_cloud->at(i).x = aVector.norm();
        vectors_data_cloud->at(i).y = mags.at(smaller_id);
        vectors_data_cloud->at(i).z = 0;
    }
}
