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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <limits>
#include <sstream>



#include "ibs.h"
#include "util_it.h"
#include "type_point_it.h"
#include "provenancevectors_it.h"
#include "samplerweighted_it.h"
#include "sampleruniform_it.h"
#include "spinner_it.h"
#include "agglomerator_it.h"
#include "StopWatch.h"

#include <string>

/**
 * @todo write docs
 */
class IT
{
public:
    IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, std::string name_affordance, std::string name_object);
    
    static IT loadFiles(std::string affordance_name, std::string object_name);
    
    static std::string getDirectory(std::string affordance_name, std::string object_name);
    
    void calculate();
    
    void saveFiles();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud;
    std::string affordanceName;
    std::string objectName;
    
    
    //filtered point clouds for working
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudFiltered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered;
    
    
    //sample
    pcl::PointCloud<PointWithVector>::Ptr sampleW;
    pcl::PointCloud<PointWithVector>::Ptr sampleU;
    
    //calculations spinned
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> spinnedDescriptorW;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> spinnedVectorsW;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> spinnedDescriptorU;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> spinnedVectorsU;
    
    //reference points
    pcl::PointXYZ   refPointScene;      // reference point in SCENE
    int             idxRefScene;
    pcl::PointXYZ   refPointIBS;        //reference point in IBS
    int             idxRefIBS;
    pcl::PointXYZ   refPointObject;     //reference point in OBJECT
    int             idxRefObject;
    Eigen::Vector3f vectSceneToIBS;     //vector from reference point in scene to IBS
    Eigen::Vector3f vectSceneToObject;  //vector from reference point in scene to Object
    

    //Sample size to take from tensor
    int sampleSize = 512;
    int numOrientations = 8;
    
private:
    
    ProvenanceVectors_iT* pv_it;
    
    Agglomerator_IT* agglomeratorW;
    
    Agglomerator_IT* agglomeratorU;
    
    
    /**
    * @brief Generate the sampling probabilities, the smaller norm value, the most probability assigned
    * 
    * @param clout_in The point cloud to analyse (norms of normals)
    * @param minV min value, associated with the higher probability
    * @param maxV max value, associated with the lowesr probability
    * @return The vector of probabilities associated to each norm in the point cloud
    */
    std::vector<float> getSamplingProbabilities(pcl::PointCloud<pcl::PointNormal>::Ptr clout_in, float minV, float maxV);
    
    std::string prepareDirectory();
    
    void defineReferences( pcl::PointXYZ middlePointObject );
    
    bool saveAggloRepresentation(Agglomerator_IT* agglomerator, std::string pathh,bool uniform=false);
    
    bool saveSpin( std::string pathh, bool uniform=false);    
    
    void saveBasicInfo( std::string aff_path );
    
    void saveSceneAndQueryObjects(std::string aff_path );
    
    void saveProvenanceIBS( std::string aff_path);
    

    

};

#endif // IT_H
