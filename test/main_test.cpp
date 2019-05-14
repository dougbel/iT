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
    float points_distance;
    float norm_diffs;
    

    PV_Distance (pcl::PointCloud<pcl::PointNormal> &cloud_a, pcl::PointCloud<pcl::PointNormal> &cloud_b)
    {
        this->pvCloudA = cloud_a;
        this->pvCloudB = cloud_b;
        this->points_distance = 0;
        this->norm_diffs = 0;
    }

    void compute ()
    {
        // compare A to B
        pcl::search::KdTree<pcl::PointNormal> tree_b;
        tree_b.setInputCloud( pvCloudB.makeShared() );
        
        int b = 0 ;
        
        std::cout << "Cloud sizes #a: " << pvCloudA.points.size() << ",  #b: " << pvCloudB.points.size() << endl << endl;
        pcl::PointCloud<pcl::PointNormal>::Ptr differences( new pcl::PointCloud<pcl::PointNormal>() );
        
        for (size_t i = 0; i < pvCloudA.points.size(); ++i)
        {
            std::vector<int> index (1);
            std::vector<float> sqr_distances (1);

            tree_b.nearestKSearch (pvCloudA.points[i], 1, index, sqr_distances);
            
            points_distance += sqr_distances[0];
            
            float diff_norm = Eigen::Vector3f(  pvCloudA.points[i].normal_x - pvCloudB.points[ index[0] ].normal_x, 
                                            pvCloudA.points[i].normal_y - pvCloudB.points[ index[0] ].normal_y, 
                                            pvCloudA.points[i].normal_z - pvCloudB.points[ index[0] ].normal_z ).norm();
            if( diff_norm > 0.001 ){
                pcl::PointNormal pa = pvCloudA.points[i];
                pcl::PointNormal pb = pvCloudB.points[index[0]];
                
                std::cout << i << std::endl;
                std::cout << "x " << pa.x << ", " << pb.x << "  y " << pa.y << ", " << pb.y << "  z " << pa.z << ", " << pb.z << endl;
                std::cout << "n_x " << pa.normal_x << ", " << pb.normal_x << "  n_y " << pa.normal_y << ", " << pb.normal_y << "  n_z " << pa.normal_z << ", " << pb.normal_z << endl;
                
                std::cout << b++ << " differencias encontradas: " << diff_norm <<  endl;
                differences->push_back( pvCloudA.points[i] );
            }
            
            norm_diffs += Eigen::Vector3f(  pvCloudA.points[i].normal_x - pvCloudB.points[ index[0] ].normal_x, 
                                            pvCloudA.points[i].normal_y - pvCloudB.points[ index[0] ].normal_y, 
                                            pvCloudA.points[i].normal_z - pvCloudB.points[ index[0] ].normal_z ).norm();
                                            
        }
        if(differences->size() > 0){
            
            pcl::io::savePCDFileASCII( "differences.pcd", *differences); 
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
    
    pcl::PointCloud<pcl::PointNormal>::Ptr raw_pv_precalculated(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile("../../test/data/test_1_pv_calculation_field.pcd", *raw_pv_precalculated);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_pv_precalculated(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile("../../test/data/test_1_pv_calculation_smoothField.pcd", *smoothed_pv_precalculated);
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  RAw PROVENANCE VECTORS  CALCULATION
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    std::cout << "STARTING test provenance vector calculation " << endl;
    
    ProvenanceVectors_iT pv_it(ibsFiltered, sceneCloudFiltered);
    pv_it.calculateProvenanceVectors(5);
    
    PV_Distance distancePV( *pv_it.rawProvenanceVectors, *raw_pv_precalculated);
    distancePV.compute();
    
    
    std::cout << "Distance: [" << "Points: "  <<  distancePV.points_distance << ", PV: "  << distancePV.norm_diffs << " ]" << endl;
    
    if (distancePV.points_distance >= 0.001 || distancePV.norm_diffs >= 0.001){
        std::cout << "ERROR! RAW Provenance vector are not calculated correctly" <<endl;
        //return EXIT_FAILURE;
    }
    else{
        std::cout << endl << "SUCESS!" <<endl<<endl<<endl;
    }
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  SMOOTH PROVENANCE VECTORS  CALCULATION TEST
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    std::cout << "STARTING test smoothed provenance vector calculation " << endl;
    
        
    PV_Distance distanceSmoothedPV( *pv_it.smoothedProvenanceVectors, *smoothed_pv_precalculated);
    distanceSmoothedPV.compute();
    
    
    std::cout << "Distance: [" << "Points: "  <<  distanceSmoothedPV.points_distance << ", PV: "  << distanceSmoothedPV.norm_diffs  << " ]"<< endl;
    
    if (distanceSmoothedPV.points_distance >= 0.001 ||  distanceSmoothedPV.norm_diffs >= 0.001){
        std::cout << "ERROR! SMOOTHED Provenance vector were not calculated correctly" <<endl;
        return EXIT_FAILURE;
    }
    else{
        std::cout << endl << "SUCESS!" <<endl<<endl<<endl;
    }
    
    
    return EXIT_SUCCESS;
}
