
#include <iostream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;


int main() {

    //读取poses
    std::string read_out = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/odo_lidar_gt_try.txt";
    std::fstream in;
    in.open(read_out, std::ios::in);//打开一个file
    //读取pcd文件
    std::string path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/";


    Eigen::Matrix4d pose_1_n1 = Eigen::Matrix4d::Identity();
    PointCloudTI::Ptr cloud_in(new PointCloudTI);
    PointCloudTI::Ptr transformed(new PointCloudTI);
    std::string window_name = "oh yeah";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));


    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "oh yeah");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(15);

    pcl::VoxelGrid<PointTI> downsamp_filter_;
    downsamp_filter_.setLeafSize(1.1, 1.1, 1.1);

    std::string buff;
    int k = 0;//行数i
    while (getline(in, buff)) {

        std::vector<double> nums;
        char *s_input = (char *) buff.c_str();
        const char *split = " ";

        char *p = strtok(s_input, split);
        double a;
        while (p != NULL) {
            a = atof(p);
            nums.
                    push_back(a);
            p = strtok(NULL, split);
        }

        for (int j = 0; j < nums.size(); j++) {
            pose_1_n1(int(j / 4), j % 4) = nums[j];
        }

        pcl::io::loadPCDFile<PointTI>(path + std::to_string(k) + ".pcd", *cloud_in);
        pcl::transformPointCloud(*cloud_in, *transformed, pose_1_n1);

        downsamp_filter_.setInputCloud(transformed);
        downsamp_filter_.filter(*transformed);
        pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_current_color(transformed, 10, 220, 90);


        viewer->addPointCloud(transformed, cloud_current_color, "cloud_current"+std::to_string(k));  //转换后的current


        if (k == 400) break;
        k++;
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    viewer->close();
//读入T矩阵

//转换点云
//添加进窗口显示

}