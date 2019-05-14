#ifndef PROVENANCEVECTORS_IT_H
#define PROVENANCEVECTORS_IT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include "type_point_it.h"

class ProvenanceVectors_iT
 {
    public:
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr ibs;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
        
        pcl::PointCloud<pcl::PointNormal>::Ptr  rawProvenanceVectors;
        
        pcl::PointCloud<pcl::PointNormal>::Ptr  smoothedProvenanceVectors;
        
        float   maxV;
        float   minV;
        
        float   maxS;
        float   minS;
        
        float   sum;
        double  sumSmooth;
        
        
        ProvenanceVectors_iT( pcl::PointCloud<pcl::PointXYZ>::Ptr ibs, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud );
        
        void calculateProvenanceVectors(int knnToSmooth);
        
        
       
};




#endif // PROVENANCEVECTORS_IT_H
