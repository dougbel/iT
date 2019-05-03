#ifndef UNIFORMSAMPLER_H
#define UNIFORMSAMPLER_H

#include "sampler_it.h"

/**
 * @todo write docs
 */
class UniformSampler_iT :  Sampler_iT
{
    public:
        Sampler_iT getSampler(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size);
        void calculateSample();
        
    private:
        
        UniformSampler_iT(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size);
    
};

#endif // UNIFORMSAMPLER_H
