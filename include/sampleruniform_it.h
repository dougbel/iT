#ifndef SAMPLERUNIFORM_IT_H
#define SAMPLERUNIFORM_IT_H

#include "sampler_it.h"

/**
 * @todo write docs
 */
class SamplerUniform_iT : Sampler_iT
{
    public:
        Sampler_iT getSampler(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size);
        
    protected:
        std::vector<int> sampleProbability();
        
    private:
        
        SamplerUniform_iT(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size);
    
};

#endif // SAMPLERUNIFORM_IT_H
