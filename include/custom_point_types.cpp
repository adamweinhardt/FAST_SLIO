#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/filter.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/impl/instantiate.hpp>
#include "custom_point_types.h"

template class pcl::VoxelGrid<PointXYZNRGBL>;
template class pcl::Filter<PointXYZNRGBL>;
template class pcl::PCLBase<PointXYZNRGBL>;
