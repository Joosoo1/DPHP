//
// Created by caochao on 06/03/20.
//

#include "explorer/pointcloud_utils.h"

namespace pointcloud_utils_ns
{
    VerticalSurfaceExtractor::VerticalSurfaceExtractor() :
        kRadiusThreshold(0.3), kZDiffMax(1.0), kZDiffMin(0.3), kNeighborThreshold(1)
    {
        extractor_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        extractor_kdtree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
    }

} // namespace pointcloud_utils_ns
