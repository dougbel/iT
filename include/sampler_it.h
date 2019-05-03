#ifndef SAMPLER_IT_H
#define SAMPLER_IT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "type_point_it.h"

class Sampler_iT
 {
    public:
                
        pcl::PointCloud<PointWithVector>::Ptr sample;
        
        virtual void calculateSample(){} ;
        virtual  Sampler_iT getSampler(pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud, int sample_size){} ;
        
    
    protected:
        int sample_size;
        pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals;
     
};




#endif // IT_H
