#include "sampleruniform_it.h"


Sampler_iT SamplerUniform_iT::getSampler(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size){
    return SamplerUniform_iT(point_normal_cloud, sample_size);
}



SamplerUniform_iT::SamplerUniform_iT(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size){
    this->sampleSize = sample_size;
    this->pointNormals = point_normal_cloud;
}



std::vector<int> SamplerUniform_iT::sampleProbability(){
    
    // Aux containers for uniform sampling
    std::vector<int> idxSamples;
    std::vector<int> aux_ids;
    
    int originalSize = this->pointNormals->size();
    
    aux_ids.reserve( originalSize );
    
    int n = 0;
    
    std::generate_n(std::back_inserter(aux_ids), originalSize, [n]()mutable { return n++; });
    std::srand (unsigned(std::time(0)));

    // By now aux_ids is filled in ascending order [0-field-size)
    // Shuffle them ramdomly and get the sample
    std::random_shuffle ( aux_ids.begin(), aux_ids.end() );
    
    idxSamples.assign(aux_ids.begin(),aux_ids.begin()+sampleSize);

    
    return idxSamples;
}

























