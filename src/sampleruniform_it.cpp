#include "sampleruniform_it.h"





SamplerUniform_iT::SamplerUniform_iT(pcl::PointCloud<pcl::PointNormal>::Ptr provenance_vectors, int sample_size){
    this->sampleSize = sample_size;
    this->provenanceVectors = provenance_vectors;
}



std::vector<int> SamplerUniform_iT::sampleProbability(){
    
    // Aux containers for uniform sampling
    std::vector<int> idxSamples;
    std::vector<int> aux_ids;
    
    int originalSize = this->provenanceVectors->size();
    
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

























