#include <iostream>
#include <assert.h>  

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <iterator>


#include "it.h"
#include "util_it.h"

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




void printPointWithVector(PointWithVector p){
    std::cout << std::fixed;
    std::cout << p.x << "," << p.y << "," << p.z << "," << " normal: " <<p.v1 << ","  << p.v2 << ","  << p.v3 << ","  << std::endl;
}



int main(int argc, char *argv[])
{
    
    std::cout.precision(15);
    std::fixed;

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
    //BEGIN
    ProvenanceVectors_iT pv_it_withOriginal(ibsFiltered, sceneCloudFiltered);
    pv_it_withOriginal.calculateProvenanceVectors(5);
    
    // Print out some data about the tensor and filtering/cleaning
    std::cout<<pv_it_withOriginal<<endl;
    
    
    //Util_iT::savePCtoCVS(*ibsFiltered, "ibs_clouds_prefiltered_filtered.csv" );
    //Util_iT::savePCtoCVS(*sceneCloudFiltered, "scene_cloud_filtered.csv" );
    
    std::cout<<endl << "STARTING TEST 1: test provenance vector calculation COMPARED with original implementation" << std::endl;
        
    PV_Distance distancePV_withOriginal( *pv_it_withOriginal.rawProvenanceVectors, *raw_pv_precalculated_original_it, false);
    distancePV_withOriginal.compute();
    
    
    std::cout << "Distance: [" << "Points: "  <<  distancePV_withOriginal.points_distance << ", PV: "  << distancePV_withOriginal.norm_diffs << " ]" << std::endl;
    
    if (distancePV_withOriginal.points_distance >= 0.0001 || distancePV_withOriginal.norm_diffs >= 0.0001){
        std::cout << "WARNING! RAW Provenance vector are not calculated correctly" <<std::endl;
        std::cout << "I found this is possibly beacuse some points that are equidistants" <<std::endl<<std::endl;
        std::cout << "TEST 1 Failure but continue test!!!"<<std::endl<<std::endl<<std::endl;;
    }
    else{
        std::cout << std::endl << "TEST 1 SUCESS!" <<std::endl<<std::endl<<std::endl;
    }
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  SMOOTH PROVENANCE VECTORS CALCULATION PROVE with original implementation
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    std::cout << "STARTING TEST 2: test smoothed provenance vector calculation COMPARED with original implementation" << std::endl;
    
        
    PV_Distance distanceSmoothedPV_withOriginal( *pv_it_withOriginal.smoothedProvenanceVectors, *smoothed_pv_precalculated_original_it, false);
    distanceSmoothedPV_withOriginal.compute();
    
    
    std::cout << "Distance: [" << "Points: "  <<  distanceSmoothedPV_withOriginal.points_distance << ", PV: "  << distanceSmoothedPV_withOriginal.norm_diffs  << " ]"<< std::endl;
    
    if (distanceSmoothedPV_withOriginal.points_distance >= 0.0001 ||  distanceSmoothedPV_withOriginal.norm_diffs >= 0.0001){
        std::cout << "TEST 2: WARNING! SMOOTHED Provenance vector were not calculated correctly" <<std::endl;
        return EXIT_FAILURE;
    }
    else{
        std::cout << std::endl << "TEST 2 SUCESS!" <<std::endl<<std::endl<<std::endl;
    }
    //END
    
    
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  MAPPING MAGNITUDES OF SAMPLING
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // extract volume of interest from the scene
    pcl::PointCloud<PointWithVector>::Ptr sampled_by_weights(new pcl::PointCloud<PointWithVector>);
    pcl::io::loadPCDFile("../../test/data/test_2_field_sample_with_weights.pcd", *sampled_by_weights);
    
    pcl::PointCloud<PointWithVector>::Ptr sampled_uniformly(new pcl::PointCloud<PointWithVector>);
    pcl::io::loadPCDFile("../../test/data/test_2_field_sample_uniformly.pcd", *sampled_uniformly);
    
    std::vector<float> mags_c_precalculated;
    std::vector<float> mags_cU_precalculated;
    
    mags_c_precalculated = Util_iT::read_vector_from_file( "../../test/data/test_2_mags_c.txt" );
   
    mags_cU_precalculated = Util_iT::read_vector_from_file( "../../test/data/test_2_mags_cU.txt" );
    
    
    float nMin;
    float nMax;
    Util_iT::getMinMaxMagnitudes(*raw_pv_precalculated_original_it, nMin, nMax);


    std::cout << "STARTING TEST 3: test mapped magnitides COMPARED with original implementation" << std::endl;
    
    
    std::vector<float> mags_c  = Util_iT::calculatedMappedMagnitudesToVector( *sampled_by_weights, nMin, nMax, 1, 0 );
    std::vector<float> mags_cU = Util_iT::calculatedMappedMagnitudesToVector( *sampled_uniformly, nMin, nMax, 1, 0 );
    
    for( int i = 0 ; i < mags_c.size(); i ++){
        if( Util_iT::roundDecimals( mags_c[i] ) != Util_iT::roundDecimals( mags_c_precalculated[i] ) ){
            std::cout << "error weighted" <<endl;
            printPointWithVector(sampled_by_weights->at(i));
            std::cout << mags_c[i] << " - " << mags_c_precalculated[i] <<endl;
            return EXIT_FAILURE;
        }
        
        if( Util_iT::roundDecimals( mags_cU[i] ) != Util_iT::roundDecimals( mags_cU_precalculated[i] ) ){
            std::cout << "error uniform" <<endl;
            printPointWithVector(sampled_uniformly->at(i));
            std::cout << mags_cU[i] << " - " << mags_cU_precalculated[i] <<endl;
            return EXIT_FAILURE;
        }
    }
    
    std::cout << std::endl << "TEST 3 MAPPED MAGNITUDES SUCESS!" <<std::endl<<std::endl<<std::endl;
    
    
    return EXIT_SUCCESS;
}
