#include "uniformsampler_it.h"


Sampler_iT UniformSampler_iT::getSampler(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size){
    return UniformSampler_iT(point_normal_cloud, sample_size);
}



void UniformSampler_iT::calculateSample() {
    std::cout << "to be implemented"<<std::endl;
}


UniformSampler_iT::UniformSampler_iT(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size){
    this->sample_size = sample_size;
    this->pointNormals = point_normal_cloud;
}
