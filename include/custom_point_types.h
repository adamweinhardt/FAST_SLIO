// custom_point_types.h

#ifndef CUSTOM_POINT_TYPES_H
#define CUSTOM_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <Eigen/Core>
#include <cstdint>

struct EIGEN_ALIGN16 PointXYZNRGBL {
  PCL_ADD_POINT4D;  // Macro to add the XYZ coordinates and padding

  union {
    struct {
      float normal_x;
      float normal_y;
      float normal_z;
      float curvature;
    };
    float data_n[4];
  };

  union {
    struct {
      PCL_ADD_UNION_RGB;  // Macro to add RGB union
      float intensity;
      std::uint16_t label; // Label field
      float time;          // Add time field
      std::uint16_t ring;  // Add ring field
    };
    float data_c[4];
  };

  PCL_ADD_EIGEN_MAPS_NORMAL4D;  // Macro to add Eigen maps for normals
  PCL_ADD_EIGEN_MAPS_RGB;       // Macro to add Eigen maps for RGB
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZNRGBL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
    (float, rgb, rgb)
    (float, intensity, intensity)
    (std::uint16_t, label, label) // Ensure the label field is registered
    (float, time, time)           // Register the time field
    (std::uint16_t, ring, ring)   // Register the ring field
)

#endif // CUSTOM_POINT_TYPES_H
