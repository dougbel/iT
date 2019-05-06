#ifndef SAMPLERUWEIGHTED_IT_H
#define SAMPLERUWEIGHTED_IT_H

#include "sampler_it.h"
#include "util_it.h"

/**
 * @todo write docs
 */
class SamplerWeighted_iT : public Sampler_iT
{
    public:
        
        SamplerWeighted_iT(pcl::PointCloud<pcl::PointNormal>::Ptr provenance_vectors, float min_value_norm, float max_value_norm, int sample_size);
        
    protected:
        std::vector<int> sampleProbability();
        
    private:
        
        float maxValueNorm;
        
        float minValueNorm;
        
        std::vector<float> weightsDistribution;
        
        
        void calculateWeightsDistribution();
    
};

#endif // SAMPLERUWEIGHTED
