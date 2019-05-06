#ifndef SAMPLERUNIFORM_IT_H
#define SAMPLERUNIFORM_IT_H

#include "sampler_it.h"

/**
 * @todo write docs
 */
class SamplerUniform_iT : public Sampler_iT
{
    public:
        SamplerUniform_iT(pcl::PointCloud<pcl::PointNormal>::Ptr provenance_vectors, int sample_size);
        
    protected:
        std::vector<int> sampleProbability();

    
};

#endif // SAMPLERUNIFORM_IT_H
