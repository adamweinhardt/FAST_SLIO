#include "custom_point_types.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/instantiate.hpp>

// Explicit instantiation of PCL classes for PointXYZNRGBL
PCL_INSTANTIATE(VoxelGrid, PointXYZNRGBL)
PCL_INSTANTIATE(Filter, PointXYZNRGBL)
