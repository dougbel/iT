#ifndef SAMPLER_IT_H
#define SAMPLER_IT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "type_point_it.h"
#include <vector>

class Sampler_iT
 {
    public:
                
        pcl::PointCloud<PointWithVector>::Ptr sample;
        std::vector<float> vectorsNorms;
        
        std::vector< std::pair<int, double>> getIdxNormKeypoints()
        {
            return  this->idxNormKeypoints;
        }
        
        
        void calculateSample(){
            
            this->keypoints = sampleProbability();
            
            sortKeypoints();
            
            extractSample();
        } 
  
    
    protected:
        int sampleSize;
        
        // Aux containers for sampling
        std::vector<int> keypoints;
        
        //this is the pair (index in the provenance vector cloud , norm of the provenance vector)
        std::vector< std::pair<int, double>> idxNormKeypoints;
        
        pcl::PointCloud<pcl::PointNormal>::Ptr provenanceVectors;
  
        virtual std::vector<int> sampleProbability()=0;
        
        
        
        void sortKeypoints(){
            // Container to sort affordance keypoint
            // "first" : id , "second" : magnitude
            idxNormKeypoints.reserve(sampleSize);
            
            // Fill the sortable containers
            for( int i=0; i < this->sampleSize; i++ )
            {
                //provenance vector id and norm (magnitude)
                std::pair<int, double>  pw;
                pcl::PointNormal pv;
                pw.first    = this->keypoints.at(i);
                pv          = this->provenanceVectors->at(pw.first);
                pw.second   = Eigen::Vector3f(pv.normal_x, pv.normal_y, pv.normal_z).norm();
                idxNormKeypoints.push_back( pw );
            }
            
            
            // Actual sort according to provenance vector length (ascending way)
            // Small (high weight) vectors come first
            std::sort( idxNormKeypoints.begin(), idxNormKeypoints.end(), [](const std::pair <int, double> & a, const std::pair <int, double> & b) -> bool{ return a.second < b.second; });
            
 
        }

        void extractSample( ){
            
            this->sample = pcl::PointCloud<PointWithVector>::Ptr(new pcl::PointCloud<PointWithVector>);
            this->vectorsNorms.reserve(sampleSize);
                        
            //std::cout<<"extracting new sample...";
            for(int i=0;i<sampleSize;i++)
            {
                
                //Saving provenance vectors sampled
                PointWithVector pv;
                pcl::PointNormal pn;
                int index;
                                
                index   = this->idxNormKeypoints.at(i).first;
                pn      = this->provenanceVectors->at( index );
                                
                pv.x    = pn.x;
                pv.y    = pn.y;
                pv.z    = pn.z;
                pv.v1   = pn.normal_x;
                pv.v2   = pn.normal_y;
                pv.v3   = pn.normal_z;
                
                this->sample->push_back( pv );
                
                
                float pv_norm;
                
                pv_norm = this->idxNormKeypoints.at(i).second;
                
                this->vectorsNorms.push_back( pv_norm );
                
            }
        }
};




#endif // IT_H
