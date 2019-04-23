#ifndef TYPE_POINT_IT
#define TYPE_POINT_IT
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


struct PointWithVector{
  PCL_ADD_POINT4D;
  float v1;
  float v2;
  float v3;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointWithVector,
    (float, x,x)
    (float, y,y)
    (float, z,z)
    (float, v1, v1)
    (float, v2, v2)
    (float, v3, v3)  )


#endif
