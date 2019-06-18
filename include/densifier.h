#ifndef DENSIFIER_H
#define DENSIFIER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

class Densifier{
    public :

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr densecloud(std::string input, int sample_points);
private:
        void uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZRGBA> & cloud_out);

        void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p);

        inline double uniform_deviate (int seed);

        void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p) ;

};


#endif // DENSIFIER_H
