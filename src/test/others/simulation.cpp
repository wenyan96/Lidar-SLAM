#include "simulation.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char *argv[]){


    Simulation r2;
    PointCloudTI::Ptr cloud_in2 (new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>(argv[1], *cloud_in2);
    //"/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sph1000.pcd"
    //unit: mm
    //pcl::VoxelGrid<pcl::PointXYZI> sor;
    //sor.setInputCloud(cloud_in2);
    //sor.setLeafSize(2.0f, 2.0f, 2.0f);
    //sor.filter(*cloud_in2);
    r2.display_cloud(cloud_in2, "after downsampling");



    cv::Mat range_mat2 = cv::Mat(BeamNum, HorizonNum, CV_64F,cv::Scalar::all(2000.0));


    r2.initialize();
    r2.set_params(cloud_in2,
                  range_mat2);
    r2.precompute3();
    r2.to_rangeimg3();
    r2.to_colormap();
    r2.calculate_normal3(true);
    r2.view_cloudwithnormal();   //changed coordinates


}
