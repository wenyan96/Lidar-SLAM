#include "common/utils.h"
#include <string>

void viewRegistration(const PointCloudTI::Ptr& cloud_last,
                      const PointCloudTI::Ptr& cloud_current,
                      const Eigen::Matrix4d& pose,
                      const int& i) {

    std::string window_name = "Registration Viewer | current_frame:" + std::to_string(i);
    //Visualization
    pcl::visualization::PCLVisualizer viewer(window_name);
    // 创建两个观察视点
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 1.0, 0.5, v1);
    viewer.createViewPort(0.0, 0.5, 1.0, 1.0, v2);

    // 定义显示的颜色信息
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    // 原始的点云设置为白色的
    pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_last_color(cloud_last, 0, 250, 0);
    viewer.addPointCloud(cloud_last, cloud_last_color, "cloud_last_v1", v1);
    pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_current_color(cloud_current, 180, 20, 20);
    viewer.addPointCloud(cloud_current, cloud_current_color, "cloud_current_v1", v1);

    PointCloudTI::Ptr cloud_tmp(new PointCloudTI);
    pcl::transformPointCloud (*cloud_current, *cloud_tmp, pose);
    for (int i = 0; i < cloud_current->size(); ++i) {
        cloud_tmp->points[i].x =
                pose(0, 0) * cloud_current->points[i].x + pose(0, 1) * cloud_current->points[i].y +
                pose(0, 2) * cloud_current->points[i].z + pose(0,3);
        cloud_tmp->points[i].y =
                pose(1, 0) * cloud_current->points[i].x + pose(1, 1) * cloud_current->points[i].y +
                pose(1, 2) * cloud_current->points[i].z + pose(1,3);
        cloud_tmp->points[i].z =
                pose(2, 0) * cloud_current->points[i].x + pose(2, 1) * cloud_current->points[i].y +
                pose(2, 2) * cloud_current->points[i].z + pose(2,3);

    }


    viewer.addPointCloud(cloud_tmp, cloud_current_color, "cloud_current_v2", v2);  //转换后的current
    viewer.addPointCloud(cloud_last, cloud_last_color, "cloud_last_v2", v2);


    //"info_1"是添加字符串的ID标志，（10，15）为坐标16为字符大小 后面分别是RGB值
    viewer.addText("Before \nGreen: last frame\nRed: current frame", 10, 15, 16,
                   txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info_1", v1);
    viewer.addText("After \nGreen: last frame\nRed: current frame", 10, 15, 16, txt_gray_lvl,
                   txt_gray_lvl, txt_gray_lvl, "info_2", v2);

    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // 设置相机的坐标和方向
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1096, 3024);  // 可视化窗口的大小

    //显示
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

}



//void checkAssociation(){

//}

