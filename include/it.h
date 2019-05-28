#ifndef IT_H
#define IT_H



#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include <boost/filesystem.hpp>

#include <limits>



#include "ibs.h"
#include "util_it.h"
#include "type_point_it.h"
#include "provenancevectors_it.h"
#include "samplerweighted_it.h"
#include "sampleruniform_it.h"
#include "spinner_it.h"
#include "agglomerator_it.h"
#include "StopWatch.h"


/**
 * @todo write docs
 */
class IT
{
public:
    IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, std::string name_affordance, std::string name_object);
    
    void calculate();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud;
    std::string affordanceName;
    std::string objectName;
    
    
    //filtered point clouds for working
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudFiltered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered;
    
    
    //reference points
    pcl::PointXYZ   refPointScene;      // reference point in SCENE
    int             idxRefScene;
    pcl::PointXYZ   refPointIBS;        //reference point in IBS
    int             idxRefIBS;
    pcl::PointXYZ   refPointObject;     //reference point in OBJECT
    int             idxRefObject;
    Eigen::Vector3f vectSceneToIBS;
    Eigen::Vector3f vectSceneToObject;
    
    
    //sampling realized
    pcl::PointCloud<PointWithVector>::Ptr new_sampleCloud2;
    pcl::PointCloud<PointWithVector>::Ptr new_sampleCloudU;
    
    //Mags in sampled mapped in 0-1 based on full tensor mags
    std::vector<float> mags_c;
    std::vector<float> mags_cU;
    
    
    //Sample size to take from tensor
    static const int sampleSize = 512; 
    
private:
      
    
    //SPINNING SECTION
    
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> descriptor; //TODO ERASE THEM! this variables are used to calculate the spin of iT
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> vectors;    //TODO ERASE THEM! this variables are used to calculate the spin of iT
    
    
    /**
    * @brief Generate the sampling probabilities, the smaller norm value, the most probability assigned
    * 
    * @param clout_in The point cloud to analyse (norms of normals)
    * @param minV min value, associated with the higher probability
    * @param maxV max value, associated with the lowesr probability
    * @return The vector of probabilities associated to each norm in the point cloud
    */
    std::vector<float> getSamplingProbabilities(pcl::PointCloud<pcl::PointNormal>::Ptr clout_in, float minV, float maxV);
    
    
    bool getAggloRepresentation(std::vector<float> &mags, std::string pathh,bool uniform=false);
    
    bool createSpin(pcl::PointCloud<PointWithVector>::Ptr sample, std::string pathh, int orientations=8,bool uniform=false);
    
    std::string prepareDirectory();
    
    void defineReferences( pcl::PointXYZ anchorPoint );
    
    void saveBasicInfo( std::string aff_path );
    
    //void getSpinMatrix(pcl::PointCloud<PointWithVector>::Ptr sample, int orientations, pcl::PointCloud<pcl::PointXYZ>::Ptr full);
    
    //void rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,bool origin);
    
    //void rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot);
    
    //void translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in,  pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation);
     
    //void translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in,  pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation, pcl::PointXYZ reference);
    
    //std::vector<int> savingInfoFileName(std:string  ); //TODO
    

};

#endif // IT_H
