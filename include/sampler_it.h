#ifndef SAMPLER_IT_H
#define SAMPLER_IT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "type_point_it.h"

class Sampler_iT
 {
    public:
                
        pcl::PointCloud<PointWithVector>::Ptr sample;
        
        void calculateSample(){
            // Aux containers for sampling
            std::vector<int> keypoints_uniform;
            std::vector< std::pair<int, double> > sortablePoints;
            
            keypoints_uniform = sampleProbability();
            sortablePoints = sortKeypoints(keypoints_uniform);
            
            extractSample(sortablePoints);
        } 
        
        virtual Sampler_iT getSampler(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size){} ;
  
    
    protected:
        int sampleSize;
        pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals;
  
        virtual std::vector<int> sampleProbability(){};
        
        std::vector< std::pair<int, double> > sortKeypoints(std::vector<int> keypoints){
            // Container to sort affordance keypoint
            // "first" : id , "second" : magnitude
            std::vector< std::pair<int, double> > sortable(keypoints.size());
            
            // Fill the sortable containers
            for(int i=0;i<keypoints.size();i++)
            {
                //provenance vector id and norm (magnitude)
                std::pair<int, double>  pw;
                pcl::PointNormal pv;
                pw.first    = keypoints.at(i);
                pv           = this->pointNormals->at(pw.first);
                pw.second   = Eigen::Vector3f(pv.normal_x, pv.normal_y, pv.normal_z).norm();
                sortable.at(i) = pw;
            }
            
            
            // Actual sort according to provenance vector length (ascending way)
            // Small (high weight) vectors come first
            std::sort(sortable.begin(),sortable.end(), [](const std::pair <int, double> & a, const std::pair <int, double> & b) -> bool{ return a.second < b.second; });
            

            return sortable;    
        }

        pcl::PointCloud<PointWithVector>::Ptr extractSample( std::vector< std::pair<int, double> > infoSortedKeypoints ){
            //std::cout<<"extracting new sample...";
            for(int i=0;i<sampleSize;i++)
            {
                
                //Saving provenance vectors sampled
                PointWithVector pv;
                pcl::PointNormal pn;
                int index;
                
                index   = infoSortedKeypoints.at(i).first;
                pn      = pointNormals->at(index);
                                
                pv.x    = pn.x;
                pv.y    = pn.y;
                pv.z    = pn.z;
                pv.v1   = pn.normal_x;
                pv.v2   = pn.normal_y;
                pv.v3   = pn.normal_z;
                
                this->sample->push_back(pv);
                
            }
        }
};




#endif // IT_H
