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
#include <Eigen/Dense>





class Util_iT{
public:
    /**
    * Generates a mesh having as input a point cloud, using the Greedy Projection Triangulation methodrchMethod
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
    * Generates a mesh having as input a point cloud, using the Poisson methodrchMethod
    * 
    * @param cloud : The input point cloud
    * @return The mesh calculated
    */
    static  pcl::PolygonMesh meshFromPointCloudSlower( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
        
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
    * Calculates the normals associated to a point cloud and return an structure with points and normals
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
};


#endif
