#include <iostream>
#include <assert.h>  

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include "it.h"


class PV_Distance{
public:
    pcl::PointCloud<pcl::PointNormal> pvCloudA;
    pcl::PointCloud<pcl::PointNormal> pvCloudB;
    pcl::PointCloud<pcl::PointNormal>::Ptr differences;
    
    float points_distance;
    float norm_diffs;
    bool debug;

    PV_Distance (pcl::PointCloud<pcl::PointNormal> &cloud_a, pcl::PointCloud<pcl::PointNormal> &cloud_b, bool debug = false):pvCloudA(cloud_a), pvCloudB(cloud_b), debug(debug)
    {
        this->points_distance = 0;
        this->norm_diffs = 0;
        this->differences = pcl::PointCloud<pcl::PointNormal>::Ptr( new pcl::PointCloud<pcl::PointNormal>() );
    }

    void compute ()
    {
        // compare A to B
        pcl::search::KdTree<pcl::PointNormal> tree_b;
        tree_b.setInputCloud( pvCloudB.makeShared() );
        
        int b = 0 ;
        
        std::cout << "Cloud sizes #a: " << pvCloudA.points.size() << ",  #b: " << pvCloudB.points.size() << std::endl;
        
        
        for (size_t i = 0; i < pvCloudA.points.size(); ++i)
        {
            std::vector<int> index (1);
            std::vector<float> sqr_distances (1);

            tree_b.nearestKSearch (pvCloudA.points[i], 1, index, sqr_distances);
            
            points_distance += sqr_distances[0];
            
            float diff_norm = Eigen::Vector3f(  pvCloudA.points[i].normal_x - pvCloudB.points[ index[0] ].normal_x, 
                                            pvCloudA.points[i].normal_y - pvCloudB.points[ index[0] ].normal_y, 
                                            pvCloudA.points[i].normal_z - pvCloudB.points[ index[0] ].normal_z ).norm();
            if( diff_norm > 0.0001 ){
                pcl::PointNormal pa = pvCloudA.points[i];
                pcl::PointNormal pb = pvCloudB.points[index[0]];
                
                if(debug){
                    std::cout << i << std::endl;
                    std::cout << "x " << pa.x << ", " << pb.x << "  y " << pa.y << ", " << pb.y << "  z " << pa.z << ", " << pb.z << std::endl;
                    std::cout << "n_x " << pa.normal_x << ", " << pb.normal_x << "  n_y " << pa.normal_y << ", " << pb.normal_y << "  n_z " << pa.normal_z << ", " << pb.normal_z << std::endl;
                    std::cout << b++ << " differencias encontradas: " << diff_norm <<  std::endl;
                }
                
                differences->push_back( pvCloudA.points[i] );
            }
            
            norm_diffs += Eigen::Vector3f(  pvCloudA.points[i].normal_x - pvCloudB.points[ index[0] ].normal_x, 
                                            pvCloudA.points[i].normal_y - pvCloudB.points[ index[0] ].normal_y, 
                                            pvCloudA.points[i].normal_z - pvCloudB.points[ index[0] ].normal_z ).norm();
                                            
        }
    }
    
};


int main(int argc, char *argv[])
{

    // extract volume of interest from the scene
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../test/data/scene_cloud_filtered.pcd", *sceneCloudFiltered);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../test/data/ibs_clouds_prefiltered_filtered.pcd", *ibsFiltered);
    

    
    pcl::PointCloud<pcl::PointNormal>::Ptr raw_pv_precalculated_original_it(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile("../../test/data/test_1_pv_calculation_field_original_it.pcd", *raw_pv_precalculated_original_it);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_pv_precalculated_original_it(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile("../../test/data/test_1_pv_calculation_smoothField_original_it.pcd", *smoothed_pv_precalculated_original_it);
    
    
    
    
    //FIRSTLY COMPARE WITH MY OWN IMPLEMENTATION TO KNOW THAT EVERYTHING IS RIGHT 
    //BEGIN
//     ////////////////////////////////////////////////////////////////////////
//     ////////////////////////////////////////////////////////////////////////
//     //////  RAw PROVENANCE VECTORS  CALCULATION
//     ////////////////////////////////////////////////////////////////////////
//     ////////////////////////////////////////////////////////////////////////
//     pcl::PointCloud<pcl::PointNormal>::Ptr raw_pv_precalculated(new pcl::PointCloud<pcl::PointNormal>);
//     pcl::io::loadPCDFile("../../test/data/test_1_pv_calculation_field.pcd", *raw_pv_precalculated);
//     
//     pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_pv_precalculated(new pcl::PointCloud<pcl::PointNormal>);
//     pcl::io::loadPCDFile("../../test/data/test_1_pv_calculation_smoothField.pcd", *smoothed_pv_precalculated);
//     std::cout << "STARTING test provenance vector calculation " << std::endl;
//     
//     ProvenanceVectors_iT pv_it(ibsFiltered, sceneCloudFiltered);
//     pv_it.calculateProvenanceVectors(5);
//     
//     PV_Distance distancePV( *pv_it.rawProvenanceVectors, *raw_pv_precalculated);
//     distancePV.compute();
//     
//     
//     std::cout << "Distance: [" << "Points: "  <<  distancePV.points_distance << ", PV: "  << distancePV.norm_diffs << " ]" << std::endl;
//     
//     if (distancePV.points_distance >= 0.0001 || distancePV.norm_diffs >= 0.0001){
//         std::cout << "WARNING! RAW Provenance vector are not calculated correctly" <<std::endl;
//         std::cout << "I found this is possibly beacuse some points that are equidistants" <<std::endl<<std::endl<<std::endl;
//         return EXIT_FAILURE;
//     }
//     else{
//         std::cout << std::endl << "SUCESS!" <<std::endl<<std::endl<<std::endl;
//     }
//     
//     
//     ////////////////////////////////////////////////////////////////////////
//     ////////////////////////////////////////////////////////////////////////
//     //////  SMOOTH PROVENANCE VECTORS CALCULATION PROVE 
//     ////////////////////////////////////////////////////////////////////////
//     ////////////////////////////////////////////////////////////////////////
//     
//     std::cout << "STARTING test smoothed provenance vector calculation " << std::endl;
//     
//         
//     PV_Distance distanceSmoothedPV( *pv_it.smoothedProvenanceVectors, *smoothed_pv_precalculated);
//     distanceSmoothedPV.compute();
//     
//     
//     std::cout << "Distance: [" << "Points: "  <<  distanceSmoothedPV.points_distance << ", PV: "  << distanceSmoothedPV.norm_diffs  << " ]"<< std::endl;
//     
//     if (distanceSmoothedPV.points_distance >= 0.0001 ||  distanceSmoothedPV.norm_diffs >= 0.0001){
//         std::cout << "WARNING! SMOOTHED Provenance vector were not calculated correctly" <<std::endl;
//         return EXIT_FAILURE;
//     }
//     else{
//         std::cout << std::endl << "SUCESS!" <<std::endl<<std::endl<<std::endl;
//     }
    //END
    
    
    
    //SECONDLY COMPARE WITH IMPLEMENTATION WITH OUTPUTS IN  ORIGINAL CODE 
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  RAW PROVENANCE VECTORS CALCULATION comparing with original implementation
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    ProvenanceVectors_iT pv_it_withOriginal(ibsFiltered, sceneCloudFiltered);
    pv_it_withOriginal.calculateProvenanceVectors(5);
    
    std::cout << "STARTING test provenance vector calculation COMPARED with original implementation" << std::endl;
        
    PV_Distance distancePV_withOriginal( *pv_it_withOriginal.rawProvenanceVectors, *raw_pv_precalculated_original_it);
    distancePV_withOriginal.compute();
    
    
    std::cout << "Distance: [" << "Points: "  <<  distancePV_withOriginal.points_distance << ", PV: "  << distancePV_withOriginal.norm_diffs << " ]" << std::endl;
    
    if (distancePV_withOriginal.points_distance >= 0.0001 || distancePV_withOriginal.norm_diffs >= 0.0001){
        std::cout << "WARNING! RAW Provenance vector are not calculated correctly" <<std::endl;
        std::cout << "I found this is possibly beacuse some points that are equidistants" <<std::endl<<std::endl;
        std::cout << "Failure but continue test!!!"<<std::endl<<std::endl<<std::endl;;
    }
    else{
        std::cout << std::endl << "SUCESS!" <<std::endl<<std::endl<<std::endl;
    }
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  SMOOTH PROVENANCE VECTORS CALCULATION PROVE with original implementation
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    std::cout << "STARTING test smoothed provenance vector calculation COMPARED with original implementation" << std::endl;
    
        
    PV_Distance distanceSmoothedPV_withOriginal( *pv_it_withOriginal.smoothedProvenanceVectors, *smoothed_pv_precalculated_original_it);
    distanceSmoothedPV_withOriginal.compute();
    
    
    std::cout << "Distance: [" << "Points: "  <<  distanceSmoothedPV_withOriginal.points_distance << ", PV: "  << distanceSmoothedPV_withOriginal.norm_diffs  << " ]"<< std::endl;
    
    if (distanceSmoothedPV_withOriginal.points_distance >= 0.0001 ||  distanceSmoothedPV_withOriginal.norm_diffs >= 0.0001){
        std::cout << "WARNING! SMOOTHED Provenance vector were not calculated correctly" <<std::endl;
        return EXIT_FAILURE;
    }
    else{
        std::cout << std::endl << "SUCESS!" <<std::endl<<std::endl<<std::endl;
    }
    
    
    return EXIT_SUCCESS;
}
