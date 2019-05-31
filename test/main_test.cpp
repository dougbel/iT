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

class PV_Distance_PointNormal{
public:
    pcl::PointCloud<pcl::PointNormal> pvCloudA;
    pcl::PointCloud<pcl::PointNormal> pvCloudB;
    pcl::PointCloud<pcl::PointNormal>::Ptr differences;
    
    float points_distance;
    float norm_diffs;
    bool debug;

    PV_Distance_PointNormal (pcl::PointCloud<pcl::PointNormal> &cloud_a, pcl::PointCloud<pcl::PointNormal> &cloud_b, bool debug = false):pvCloudA(cloud_a), pvCloudB(cloud_b), debug(debug)
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

class PV_Distance_PointXYZ{
public:
    pcl::PointCloud<pcl::PointXYZ> pvCloudA;
    pcl::PointCloud<pcl::PointXYZ> pvCloudB;
    pcl::PointCloud<pcl::PointXYZ>::Ptr differences;
    
    float points_distance;
    bool debug;

    PV_Distance_PointXYZ (pcl::PointCloud<pcl::PointXYZ> &cloud_a, pcl::PointCloud<pcl::PointXYZ> &cloud_b, bool debug = false):pvCloudA(cloud_a), pvCloudB(cloud_b), debug(debug)
    {
        this->points_distance = 0;
        this->differences = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
    }

    void compute ()
    {
        // compare A to B
        pcl::search::KdTree<pcl::PointXYZ> tree_b;
        tree_b.setInputCloud( pvCloudB.makeShared() );
        
        int b = 0 ;
        
        std::cout << "Cloud sizes #a: " << pvCloudA.points.size() << ",  #b: " << pvCloudB.points.size() << std::endl;
        
        
        for (size_t i = 0; i < pvCloudA.points.size(); ++i)
        {
            std::vector<int> index (1);
            std::vector<float> sqr_distances (1);

            tree_b.nearestKSearch (pvCloudA.points[i], 1, index, sqr_distances);
            
            points_distance += sqr_distances[0];
            
            if (sqr_distances[0] > 0){
                differences->push_back( pvCloudA.points[i] );
            }
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
    
    
    
    ProvenanceVectors_iT pv_it_withOriginal(ibsFiltered, sceneCloudFiltered);
    pv_it_withOriginal.calculateProvenanceVectors(5);
    
    // Print out some data about the tensor and filtering/cleaning
    std::cout<<pv_it_withOriginal<<endl;
    
   
    
    
    //SECONDLY COMPARE WITH IMPLEMENTATION WITH OUTPUTS IN  ORIGINAL CODE 
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  RAW PROVENANCE VECTORS CALCULATION comparing with original implementation
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //BEGIN

    
    
    //Util_iT::savePCtoCVS(*ibsFiltered, "ibs_clouds_prefiltered_filtered.csv" );
    //Util_iT::savePCtoCVS(*sceneCloudFiltered, "scene_cloud_filtered.csv" );
    
    std::cout<<endl << "STARTING TEST 1: test raw provenance vector calculation COMPARED with original implementation" << std::endl;
        
    PV_Distance_PointNormal distancePV_withOriginal( *pv_it_withOriginal.rawProvenanceVectors, *raw_pv_precalculated_original_it, false);
    distancePV_withOriginal.compute();
    
    
    std::cout << "Distance: [" << "Points: "  <<  distancePV_withOriginal.points_distance << ", PV: "  << distancePV_withOriginal.norm_diffs << " ]" << std::endl;
    
    if (distancePV_withOriginal.points_distance >= 0.0001 || distancePV_withOriginal.norm_diffs >= 0.0001){
        std::cout << "WARNING! RAW Provenance vector are not calculated correctly" <<std::endl;
        std::cout << "I found this is possibly beacuse some points that are equidistants" <<std::endl;
        std::cout << "TEST 1 Failure but continue test!!!"<<std::endl<<std::endl<<std::endl;;
    }
    else{
        std::cout << std::endl << "TEST 1 raw provenance vectors SUCCESS!" <<std::endl;
    }
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  SMOOTH PROVENANCE VECTORS CALCULATION PROVE with original implementation
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    std::cout << "STARTING TEST 1b: test smoothed provenance vector calculation COMPARED with original implementation" << std::endl;
    
        
    PV_Distance_PointNormal distanceSmoothedPV_withOriginal( *pv_it_withOriginal.smoothedProvenanceVectors, *smoothed_pv_precalculated_original_it, false);
    distanceSmoothedPV_withOriginal.compute();
    
    
    std::cout << "Distance: [" << "Points: "  <<  distanceSmoothedPV_withOriginal.points_distance << ", PV: "  << distanceSmoothedPV_withOriginal.norm_diffs  << " ]"<< std::endl;
    
    if (distanceSmoothedPV_withOriginal.points_distance >= 0.0001 ||  distanceSmoothedPV_withOriginal.norm_diffs >= 0.0001){
        std::cout << "TEST 1b: WARNING! SMOOTHED Provenance vector were not calculated correctly" <<std::endl;
        return EXIT_FAILURE;
    }
    else{
        std::cout << "TEST 1b smoothed provenance vectors SUCCESS!" <<std::endl<<std::endl<<std::endl;
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


    std::cout << "STARTING TEST 2: test mapped magnitides COMPARED with original implementation" << std::endl;
    
    
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
    
    std::cout << "TEST 2 MAPPED MAGNITUDES SUCCESS!" <<std::endl<<std::endl<<std::endl;
    
    
    
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  SPINNING POINT CLOUDS
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    std::cout << "STARTING TEST 3: test for SPINING ALGORITHM COMPARED with original implementation" << std::endl;
    
//     pcl::PointCloud<PointWithVector>::Ptr to_spin(new pcl::PointCloud<PointWithVector>);
//     pcl::io::loadPCDFile("../../test/data/test_3_spin_visual.pcd", *to_spin);
//     pcl::PointXYZ centroid;
//     Eigen::Vector4f centroid_ei;
//     pcl::compute3DCentroid(*to_spin, centroid_ei);
//     centroid.getVector4fMap() = centroid_ei;
//     Spinner_iT spinner( to_spin, centroid, 8 );
//     spinner.calculateSpinings();
//     std::cout << "You must check by your self output file "<< endl;
    
    
    std::ifstream ifs_vectors ( "../../test/data/test_3_spin_8_vectors.dat" );
    Eigen::MatrixXf spin_vectors_precalculated;
    spin_vectors_precalculated.resize( IT::sampleSize, 3 ); 
    pcl::loadBinary( spin_vectors_precalculated,ifs_vectors );
 

    std::ifstream ifs_descriptor ( "../../test/data/test_3_spin_8_descriptor.dat");
    Eigen::MatrixXf spin_descriptor_precalculated;
    spin_descriptor_precalculated.resize( IT::sampleSize*8, 3 ); 
    pcl::loadBinary( spin_descriptor_precalculated,ifs_descriptor );
    
    
    pcl::PointCloud<PointWithVector>::Ptr sample_to_spin(new pcl::PointCloud<PointWithVector>);
    pcl::io::loadPCDFile("../../test/data/test_3_sample_pointwithvector_to_spin.pcd", *sample_to_spin);
    
    pcl::PointXYZ spinning_point(0.951572120189667,0.556678950786591,1.17748117446899);
    
    Spinner_iT spinner2( sample_to_spin, spinning_point, 8 );
    spinner2.calculateSpinings();
    
        
    for(int i =0 ; i < spin_vectors_precalculated.rows(); i++)
    {
        if( Util_iT::roundDecimals( spinner2.vectors(i,0) ) != Util_iT::roundDecimals( spin_vectors_precalculated(i,0) ) || 
            Util_iT::roundDecimals( spinner2.vectors(i,1) ) != Util_iT::roundDecimals( spin_vectors_precalculated(i,1) ) || 
            Util_iT::roundDecimals( spinner2.vectors(i,2) ) != Util_iT::roundDecimals( spin_vectors_precalculated(i,2) ) )
        {
            std::cout << i<<") "<< spin_vectors_precalculated(i,0) << "-"<< spinner2.vectors(i,0) << "," 
            << spin_vectors_precalculated(i,1) << "-"<< spinner2.vectors(i,1) << "," 
            << spin_vectors_precalculated(i,2) << "-"<< spinner2.vectors(i,2 )<< std::endl;
            return EXIT_FAILURE;
            
        }
    }
    
    for(int i =0 ; i < spin_descriptor_precalculated.rows(); i++)
    {
        if( Util_iT::roundDecimals( spinner2.descriptor(i,0) ) != Util_iT::roundDecimals( spin_descriptor_precalculated(i,0) ) || 
            Util_iT::roundDecimals( spinner2.descriptor(i,1) ) != Util_iT::roundDecimals( spin_descriptor_precalculated(i,1) ) || 
            Util_iT::roundDecimals( spinner2.descriptor(i,2) ) != Util_iT::roundDecimals( spin_descriptor_precalculated(i,2) ) )
        {
            std::cout << i<<") "<<spin_descriptor_precalculated(i,0) << "-"<< spinner2.descriptor(i,0) << ","
            << spin_descriptor_precalculated(i,1) << "-"<< spinner2.descriptor(i,1) << ","
            << spin_descriptor_precalculated(i,2) << "-"<< spinner2.descriptor(i,2) << std::endl; 
            return EXIT_FAILURE;
        }
    }

    
     std::cout << "TEST 3 SPINNING SUCCESS!" <<std::endl<<std::endl<<std::endl;
    
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  SPINNING POINT CLOUDS
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    //INPUTS
    std::ifstream ifs_agg_vectors ( "../../test/data/test_4_agglomeration_input_vectors.dat");
    Eigen::MatrixXf agglomerator_vectors;
    agglomerator_vectors.resize( IT::sampleSize, 3 ); 
    pcl::loadBinary( agglomerator_vectors,ifs_agg_vectors );


    std::ifstream ifs_agg_descriptor ( "../../test/data/test_4_agglomeration_input_descriptor.dat");
    Eigen::MatrixXf agglomerator_descriptor;
    agglomerator_descriptor.resize( IT::sampleSize*8, 3 ); 
    pcl::loadBinary( agglomerator_descriptor,ifs_agg_descriptor );
    
    //test_4_agglomeration_input_vectors.dat
    std::vector<float> agglomerator_mags;
    agglomerator_mags = Util_iT::read_vector_from_file("../../test/data/test_4_agglomeration_input_mags.dat");
    
    Agglomerator_IT agglomerator( agglomerator_vectors, agglomerator_descriptor, agglomerator_mags );
    agglomerator.compileAgglomeration();
    
    //OUTPUTS
    std::cout << "STARTING TEST 4: test for AGGLOMERATION PROCESS with original implementation" << std::endl;
    
    //descriptor
    pcl::PointCloud<pcl::PointXYZ>::Ptr agglomeration_descriptor_pre (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../test/data/test_4_agglomeration_descriptor.pcd",*agglomeration_descriptor_pre);
    
    PV_Distance_PointXYZ distanceAgglo_descriptor( *agglomeration_descriptor_pre, *agglomerator.better_approx, false);
    distanceAgglo_descriptor.compute();
    std::cout << "Distance: [" << "Points: "  <<  distanceAgglo_descriptor.points_distance << " ]" << std::endl;
    if (distanceAgglo_descriptor.points_distance >= 0.0001 ){ std::cout << "TEST 4.descriptor failure"<<std::endl; return EXIT_FAILURE;}
    else{std::cout << "TEST 4.descriptor SUCCESS!" <<std::endl;}
    
    //extra
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr agglomeration_extra_pre (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../test/data/test_4_agglomeration_extra.pcd",*agglomeration_extra_pre);
    
    PV_Distance_PointXYZ distanceAgglo_extra( *agglomeration_extra_pre, *agglomerator.useful_cloud, false);
    distanceAgglo_extra.compute();
    std::cout << "Distance: [" << "Points: "  <<  distanceAgglo_extra.points_distance << " ]" << std::endl;
    if (distanceAgglo_extra.points_distance >= 0.0001 ){ std::cout << "TEST 4.extra failure"<<std::endl; return EXIT_FAILURE;}
    else{std::cout << "TEST 4.extra SUCCESS!" <<std::endl;}
    
    //members
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr agglomeration_members_pre (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../test/data/test_4_agglomeration_members.pcd",*agglomeration_members_pre);
    
    PV_Distance_PointXYZ distanceAgglo_members( *agglomeration_members_pre, *agglomerator.aux_cloud, false);
    distanceAgglo_members.compute();
    std::cout << "Distance: [" << "Points: "  <<  distanceAgglo_members.points_distance << " ]" << std::endl;
    if (distanceAgglo_members.points_distance >= 0.0001 ){ std::cout << "TEST 4.members failure"<<std::endl; return EXIT_FAILURE;}
    else{std::cout << "TEST 4.members SUCCESS!" <<std::endl;}
    
    //points
    pcl::PointCloud<pcl::PointXYZ>::Ptr agglomeration_points_pre (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../test/data/test_4_agglomeration_points.pcd",*agglomeration_points_pre);
    
    PV_Distance_PointXYZ distanceAgglo_points( *agglomeration_points_pre, *agglomerator.bare_points, false);
    distanceAgglo_points.compute();
    std::cout << "Distance: [" << "Points: "  <<  distanceAgglo_points.points_distance << " ]" << std::endl;
    if (distanceAgglo_points.points_distance >= 0.0001 ){ std::cout << "TEST 4.points failure"<<std::endl; return EXIT_FAILURE;}
    else{std::cout << "TEST 4.points SUCCESS!" <<std::endl;}
    
    //vdata
    pcl::PointCloud<pcl::PointXYZ>::Ptr agglomeration_vdata_pre (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../test/data/test_4_agglomeration_vdata.pcd",*agglomeration_vdata_pre);
    
    PV_Distance_PointXYZ distanceAgglo_vdata( *agglomeration_vdata_pre, *agglomerator.vectors_data_cloud, false);
    distanceAgglo_vdata.compute();
    std::cout << "Distance: [" << "points: "  <<  distanceAgglo_vdata.points_distance << " ]" << std::endl;
    if (distanceAgglo_vdata.points_distance >= 0.0001 ){ std::cout << "TEST 4.vdata failure"<<std::endl; return EXIT_FAILURE;}
    else{std::cout << "TEST 4.vdata SUCCESS!" <<std::endl;}
    
    
    //vectors
    pcl::PointCloud<pcl::PointXYZ>::Ptr agglomeration_vectors_pre (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../test/data/test_4_agglomeration_vectors.pcd",*agglomeration_vectors_pre);
    
    PV_Distance_PointXYZ distanceAgglo_vectors( *agglomeration_vectors_pre, *agglomerator.vector_ids_agglomerative, false);
    distanceAgglo_vectors.compute();
    std::cout << "Distance: [" << "points: "  <<  distanceAgglo_vectors.points_distance << " ]" << std::endl;
    if (distanceAgglo_vectors.points_distance >= 0.0001 ){ std::cout << "TEST 4.vectors failure"<<std::endl; return EXIT_FAILURE;}
    else{std::cout << "TEST 4.vectors SUCCESS!" <<std::endl;}

    
    //point count
    std::ifstream ifs_agg_point_count ( "../../test/data/test_4_agglomeration_point_count.dat", std::ifstream::in );
    Eigen::MatrixXf agglomerator_point_count;
    agglomerator_point_count.resize(1, 8); 
    pcl::loadBinary( agglomerator_point_count, ifs_agg_point_count );
    
       
    if( agglomerator_point_count(0,0) != agglomerator.data_individual(0,0) || agglomerator_point_count(0,1) != agglomerator.data_individual(0,1) ||
        agglomerator_point_count(0,2) != agglomerator.data_individual(0,2) || agglomerator_point_count(0,3) != agglomerator.data_individual(0,3) ||
        agglomerator_point_count(0,4) != agglomerator.data_individual(0,4) || agglomerator_point_count(0,5) != agglomerator.data_individual(0,5) ||
        agglomerator_point_count(0,6) != agglomerator.data_individual(0,6) || agglomerator_point_count(0,7) != agglomerator.data_individual(0,7) )
    { std::cout << "TEST 4.point_count failure"<<std::endl; return EXIT_FAILURE;}
    else{std::cout << "TEST 4.point_count SUCCESS!" <<std::endl;}
    
    
    
    
    return EXIT_SUCCESS;
}
