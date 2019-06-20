#ifndef UTIL_IT
#define UTIL_IT


#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "type_point_it.h"

#include <Eigen/Dense>

#include <boost/tokenizer.hpp>

#include <math.h>
#include <iostream>
#include <fstream>




class Util_iT{
public:
    /**
    * @brief Generates a mesh having as input a point cloud, using the Greedy Projection Triangulation methodrchMethod
    * 
    * @param cloud The input point cloud
    * @return The mesh calculated
    */
    static  pcl::PolygonMesh meshFromPointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
        
        pcl::PolygonMesh mesh;
        
        
        //////////////////////////////////////////////////////////////////////////////
        //combine points and normals
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
        
        cloud_smoothed_normals = Util_iT::getPointCloudWithNormals(cloud);

        
        //////////////////////////////////////////////////////////////////////////////
        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_smoothed_normals);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;


        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius (0.2);

        // Set typical values for the parameters
        gp3.setMu (7.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gp3.setMinimumAngle(M_PI/18); // 10 degrees
        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        gp3.setNormalConsistency(true);

        // Get result
        gp3.setInputCloud (cloud_smoothed_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (mesh);
        
        
        // Additional vertex information
        //std::vector<int> parts = gp3.getPartIDs();
        //std::vector<int> states = gp3.getPointStates();
        
        
        //pcl::io::savePLYFile ("mesh.ply", mesh);
        
        return mesh;
    }
    
    

    /**
    * @brief Generates a mesh having as input a point cloud, using the Poisson methodrchMethod
    * 
    * @param cloud : The input point cloud
    * @return The mesh calculated
    */
    static  pcl::PolygonMesh meshFromPointCloudSlowerButBetter( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
        
        pcl::PolygonMesh mesh;
        
        
        //combine points and normals
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
        
        cloud_smoothed_normals = Util_iT::getPointCloudWithNormals(cloud);

      
        //begin poisson reconstruction
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth(9);
        poisson.setInputCloud(cloud_smoothed_normals);
        
        poisson.reconstruct(mesh);
    
      
        
        //pcl::io::savePLYFile ("meshslow.ply", mesh);
        
        return mesh;
    }
    
    
    
    /**
    * @brief Calculates the normals associated to a point cloud and return an structure with points and normals
    * 
    * @param cloud Input point cloud
    * @return Point cloud with 3D points and normals associated to them
    */
    static  pcl::PointCloud<pcl::PointNormal>::Ptr getPointCloudWithNormals( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
       
        
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
        
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
        
        //normal estimation
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNumberOfThreads(8);
        ne.setInputCloud(cloud);
        ne.setRadiusSearch(0.05);
        
        Eigen::Vector4f centroid;
        compute3DCentroid(*cloud, centroid);
        
        ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
        ne.compute(*cloud_normals);
        
        //reverse normal direction
        for(size_t i = 0; i < cloud_normals->size(); ++i){
            cloud_normals->points[i].normal_x *= -1;
            cloud_normals->points[i].normal_y *= -1;
            cloud_normals->points[i].normal_z *= -1;
        }

        //combine points and normals
        concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);

        
        return cloud_smoothed_normals;
    }
    
    
    
    /**
    * @brief Get the max and min norms in the provenance vector associated with the point cloud
    * 
    * @param point_normal_cloud The point cloud to analyze
    * @param min_mag Min found magnitude
    * @param max_mag Max found magnitude
    */
    static void getMinMaxMagnitudes(pcl::PointCloud<pcl::PointNormal> point_normal_cloud, float &min_mag, float &max_mag){
        
        float max_original = std::numeric_limits<float>::min();
        float min_original = std::numeric_limits<float>::max();
        
        pcl::PointNormal n;
        float mag ;
        
        for(int i = 0; i < point_normal_cloud.size(); i++){
            
            n = point_normal_cloud.at(i);
            
            mag = Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z).norm();
            
            if( min_original > mag )  min_original = mag;
            
            if( max_original < mag )  max_original = mag;
        }
        
        min_mag = min_original;
        max_mag = max_original;
    }
    
    
    static std::vector<float> calculatedMappedMagnitudesToVector(pcl::PointCloud<PointWithVector> &point_normal_cloud, float min_original_range, float max_original_range, float min_mapped, float max_mapped){
        
        int size = point_normal_cloud.size();
        
        std::vector<float> mags(size);
        
        for( int i=0; i<size; i++ )
        {
            PointWithVector backup = point_normal_cloud.at( i );
            Eigen::Vector3f backupNormal( backup.v1, backup.v2, backup.v3);
        
            //Save mags in sampled mapped in 0-1 based on full tensor mags
            mags.at(i)=Util_iT::getValueProporcionsRule( backupNormal.norm(), min_original_range, max_original_range, min_mapped, max_mapped ); 
        
        }
        
        return mags;
    
    }
    
    
    /**
    * @brief Realized the mapping of magnitudes in a Point Cloud of Normals, from a given range [min_original_range, max_original_range] to a desired one [min_mapped, max_mapped]. If values in the given point cloud exced the ones established the rule of proportions will be applied and the output values also will exced the stablished mapped values
    * 
    * @param point_normal_cloud The point cloud to be modified 
    * @param min_original_range value of minimum original magnitude
    * @param max_original_range value of maximum oroginal magnitude
    * @param min_mapped Value of minimun mapped value
    * @param max_mapped Value of maximun mapped value
    * @return point_normal_cloud will have the values altered
    */
    static void mapMagnitudes(pcl::PointCloud<pcl::PointNormal> &point_normal_cloud, float min_original_range, float max_original_range, float min_mapped, float max_mapped){
        
        pcl::PointNormal n;
        
        Eigen::Vector3f oldNormal;
        Eigen::Vector3f newNormal;
        
        float mapped_mag;
        
        // Map every vector in the tensor
        for(int i=0;i<point_normal_cloud.size();i++)
        {
            n = point_normal_cloud.at(i);
            
            oldNormal = Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
            
            mapped_mag = Util_iT::getValueProporcionsRule( oldNormal.norm(), min_original_range, max_original_range, min_mapped, max_mapped );
            
            newNormal= mapped_mag * oldNormal.normalized();
            
            //mappedCloud->at(i) = pcl::Normal(newNormal[0], newNormal[1], newNormal[2]);
            
            point_normal_cloud.at(i).normal_x = newNormal[0];
            point_normal_cloud.at(i).normal_y = newNormal[1];
            point_normal_cloud.at(i).normal_z = newNormal[2];
        }
    }
    
   /**
    * @brief Realized the mapping of magnitudes in a Point Cloud of Normals, from a given range [min_original_range, max_original_range] to a desired one [min_mapped, max_mapped] and applies the function 1/x to suchs values. If mapped values (before apply function) in the given point cloud exced the ones established the rule of proportions will be applied and the output values also will exced the stablished mapped values
    * 
    * @param point_normal_cloud The point cloud to be modified 
    * @param min_original_range value of minimum original magnitude
    * @param max_original_range value of maximum oroginal magnitude
    * @param min_mapped Value of minimun mapped value
    * @param max_mapped Value of maximun mapped value
    * @return point_normal_cloud will have the values altered
    */
    static void mapMagnitudesRationalFunction(pcl::PointCloud<pcl::PointNormal> &point_normal_cloud, float min_original_range, float max_original_range, float min_mapped, float max_mapped){
     
        Eigen::Vector3f oldNormal;
        Eigen::Vector3f newNormal;
        float mapped_mag;
        
        for(int i=0;i<point_normal_cloud.size();i++)
        {
            oldNormal = Eigen::Vector3f( point_normal_cloud.at(i).normal_x, point_normal_cloud.at(i).normal_y, point_normal_cloud.at(i).normal_z);
            
            mapped_mag  = Util_iT::getValueProporcionsRule( oldNormal.norm(), min_original_range, max_original_range, min_mapped, max_mapped );
            
            //this function is the one used to mapping values
            //check file "Mapping values iT graph.R" for visualize mapping
            newNormal   = ( 1 / mapped_mag) * oldNormal.normalized();
            
            point_normal_cloud.at(i).normal_x = newNormal[0];
            point_normal_cloud.at(i).normal_y = newNormal[1];
            point_normal_cloud.at(i).normal_z = newNormal[2];
        }
    }
    

    /**
    * @brief Maps a given value from one given range of values (min_in, max_in)  to another(min_mapped, max_mapped)
    * 
    * @param value_in Value to be mapped
    * @param min_original_range Min value of the original range of values
    * @param max_original_range Max value of the original range of values
    * @param min_mapped Min value of mapped range
    * @param max_mapped Max valur of mapped range
    * @return The mapped value in the mapped range
    */
    static inline float getValueProporcionsRule(float value_in, float min_original_range, float max_original_range, float min_mapped, float max_mapped){
        return ( value_in - min_original_range) * (max_mapped- min_mapped) / (max_original_range - min_original_range) + min_mapped;
    }

        
    
    /**
    * @brief Find the index cloud asociated to the closest point to a given "query_point"   
    * 
    * @param cloud The point cloud in which the search will be realized.
    * @param query_point The point
    * @return Index asociated to the nearest point in the point cloud
    */
    static int indexOfClosestPointInACloud ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ query_point)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        std::vector<int> NNidx(1);
        std::vector<float> NNdist(1);
        
        if(kdtree.nearestKSearch (query_point, 1, NNidx, NNdist) > 0 )
        {
            return NNidx.at(0);
        }
        else
        {
            std::cout<<" NN not found "<<std::endl;
            return -1;
        }
    }
    
    
    /**
    * @brief Saves data from a point cloud to a CVS format file
    * 
    * @param file_name The name of the file (better have a .cvs extension)
    * @param cloud The point cloud
    */
    static void savePCtoCVS(pcl::PointCloud<pcl::PointNormal> cloud, std::string file_name){
        
        std::ofstream file;
        
        file.open (file_name);
        file << "x,y,z,normal_x,normal_y,normal_z" << std::endl;
        
        for(int i=0; i<cloud.size(); i++)
        {
            file << cloud.at(i).x << "," << cloud.at(i).y << "," << cloud.at(i).z << "," << cloud.at(i).normal_x << "," << cloud.at(i).normal_y << "," << cloud.at(i).normal_z << std::endl;
        }
        
        file.close();
    }
    
    
    /**
    * @brief Saves data from a point cloud to a CVS format file
    * 
    * @param file_name The name of the file (better have a .cvs extension)
    * @param cloud The point cloud
    */
    static void savePCtoCVS(pcl::PointCloud<pcl::PointXYZ> cloud, std::string file_name){
        
        std::ofstream file;
                
        file.open (file_name);
        file << "x,y,z" << std::endl;
        
        for(int i=0; i<cloud.size(); i++)
        {
            file << cloud.at(i).x << "," << cloud.at(i).y << "," << cloud.at(i).z << std::endl;
        }
        
        file.close();
    }
    
    
    /**
     * @brief Write values of a std::vector<float> to a file
     * 
     * @param v The vector to save
     * @param file The name of file to store the vector
     * @return void
     */
    static void save_vector_to_file(const std::vector<float>& myVector, std::string filename)
    {
        Eigen::VectorXf v(myVector.size());
        for(int i = 0 ; i < myVector.size(); i++){
            v(i)= myVector.at(i);
        }
        
        std::ofstream file(filename.c_str());
        pcl::saveBinary(v,file);
    }
    
    
    
    /**
     * @brief Read from a file values of a std::vector<float>
     * 
     * @param filename The file name to be readed
     * @return The vector which values were readed from file
     */
    static std::vector<float> read_vector_from_file(std::string filename, int size)
    {    
        std::vector<float> newVector;
        
        Eigen::VectorXf v;
        
        std::ifstream is(filename.c_str());
        
        pcl::loadBinary(v, is);
        
        for(int i = 0 ; i < size; i++){
            newVector.push_back(v(i));
        }
        
        return newVector;
    }
    
    
    /**
     * @brief Return a float value "floor rounded" to 6 decimals
     * 
     * @return float
     */
    static float roundDecimals(float val)
    {
        return floorf(val * 1000000) / 1000000;
    }
    
    
    
    /**
     * @brief Print in a legible way values in a PointWithvector 
     * 
     * @return void
     */
    static void printPointWithVector(PointWithVector p){
        std::cout << std::fixed;
        std::cout << p.x << "," << p.y << "," << p.z << "," << " normal: " <<p.v1 << ","  << p.v2 << ","  << p.v3 << ","  << std::endl;
    }   
    
    
    /**
     * @brief Generates a pcl::PointXYZ from a string comma-separated as "0.951572,0.556679,1.17748"
     *
     * @return The generated pcl::PointXYZ 
     */
    static pcl::PointXYZ stringToPoint(std::string str){
        
        pcl::PointXYZ output;
        
        boost::char_separator<char> sep{","};
        boost::tokenizer<boost::char_separator<char>> tokens{str, sep} ;
        boost::tokenizer<boost::char_separator<char>>::iterator tok_iter = tokens.begin();
        output.x = std::stof(*tok_iter); 
        tok_iter++;
        output.y = std::stof(*tok_iter);
        tok_iter++;
        output.z = std::stof(*tok_iter);
        
        return output;
    }

    
};

    

    
    

#endif
