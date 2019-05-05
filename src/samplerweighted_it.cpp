#include "samplerweighted_it.h"


Sampler_iT SamplerWeighted_iT::getSampler(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, float min_value_norm, float max_value_norm, int sample_size){
    return SamplerWeighted_iT(point_normal_cloud, min_value_norm, max_value_norm, sample_size);
}



SamplerWeighted_iT::SamplerWeighted_iT(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, float min_value_norm, float max_value_norm, int sample_size){
    this->sampleSize = sample_size;
    this->minValueNorm = min_value_norm;
    this->maxValueNorm = max_value_norm;
    this->pointNormals = point_normal_cloud;
}



std::vector<int> SamplerWeighted_iT::sampleProbability(){
    
    int nrolls;
    int  indexer=0;
    
    std::vector<int> idxSamples(sampleSize);
    
      
    calculateWeightsDistribution();
    
    nrolls = 2*this->weightsDistribution.size();
    
    //random numers generator
    std::default_random_engine generator;
    // distribution used for generate number randomly
    std::discrete_distribution<int> distribution (this->weightsDistribution.begin(),this->weightsDistribution.end());
    
    // Sampled data, it will be the vote bins  to define keypoints 
    std::vector< std::pair<int,int> > bins;
    
    //inizializing container of votes
    bins.reserve(this->weightsDistribution.size());
    std::generate_n(std::back_inserter(bins), this->weightsDistribution.size(), [indexer]()mutable { return std::make_pair (indexer++,0); });
    
    
    //full filling the vote containers throught the generator of random numbers
    for (indexer=0; indexer<nrolls; ++indexer)    {
        int number = distribution(generator);
        bins.at(number).second += 1;
    }
    
    //ordering votes and indexs associated in a descendent way 
    std::sort(bins.begin(),bins.end(), 
              [](const std::pair <int, int> & a, const std::pair <int, int> & b) -> bool{ return a.second > b.second; });
    
    //sampling
    for(indexer=0;indexer<sampleSize;indexer++)    {
        idxSamples.at(indexer) = bins.at( indexer ).first;
        //std::cout << "index "<< indexer<< ", votes " <<  bins.at( indexer ).first << " : " << bins.at( indexer ).second <<  std::endl;
    }
    
    return idxSamples;
}



std::vector<float> SamplerWeighted_iT::calculateWeightsDistribution(){
    this->weightsDistribution.reserve( this->pointNormals->size() );
    
    
    for(int i = 0; i<pointNormals->size(); i++ )
    {
        Eigen::Vector3f oldNormal( pointNormals->at(i).normal_x, pointNormals->at(i).normal_y, pointNormals->at(i).normal_z);
        //the smaller magnitude of norm the biggest weight`
        this->weightsDistribution.at(i) = Util_iT::getValueProporcionsRule( oldNormal.norm(), this->minValueNorm, this->maxValueNorm, 1, 0);
    }
    //return this->weightsDistribution;
    
}






















