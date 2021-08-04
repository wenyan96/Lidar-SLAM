#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/time.h>
#include <eigen3/Eigen/Core>
#include <string>

using PointTI = pcl::PointXYZI;
using PointCloudTI = pcl::PointCloud<PointTI>;
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

void viewRegistration(const PointCloudTI::Ptr& cloud_last,
                      const PointCloudTI::Ptr& cloud_current,
                      const Eigen::Matrix4d& pose,
                      const int& i);

const std::string WORK_SPACE_PATH = "/home/wenyan/Documents/MasterThesis/mymt_ws";
//const std::string WORK_SPACE_PATH = "@WORK_SPACE_PATH@";

