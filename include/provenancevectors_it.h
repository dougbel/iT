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
        
        int     knnToSmooth;
        
        
        ProvenanceVectors_iT( pcl::PointCloud<pcl::PointXYZ>::Ptr ibs, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud );
        
        void calculateProvenanceVectors(int knnToSmooth);
        
       friend std::ostream &operator<< (std::ostream& stream, const ProvenanceVectors_iT& pv_it_withOriginal){
            // Print out some data about the tensor and filtering/cleaning
            stream<<"Tensor after filtering "<<pv_it_withOriginal.ibs->size()<<std::endl;
            stream<<"Min: "<<pv_it_withOriginal.minV<<" Max: "<<pv_it_withOriginal.maxV<<std::endl;
            stream<<"Sum: "<<pv_it_withOriginal.sum<<std::endl;
            stream<<"======================"<<std::endl;
            stream<<"MinS: "<<pv_it_withOriginal.minS<<" MaxS: "<<pv_it_withOriginal.maxS<<std::endl;
            stream<<"SumSmooth: "<<pv_it_withOriginal.sumSmooth;
            
            return stream;
        }
        
    private:
        std::vector<int> bad_ids;
        
        void determine();
        void filter();
        void updateMinMaxValues();
       
    
};



#endif // PROVENANCEVECTORS_IT_H
