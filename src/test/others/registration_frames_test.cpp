
#include "registration_frames.h"
#include <pcl/console/time.h>


int main(){

    PointCloudTI::Ptr cloud_src(new PointCloudTI);
    PointCloudTI::Ptr cloud_target(new PointCloudTI);
    PointCloudTI::Ptr cloud_src_down(new PointCloudTI);
    PointCloudTI::Ptr cloud_target_down(new PointCloudTI);
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/5.pcd", *cloud_target);
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/6.pcd", *cloud_src);

    pcl::VoxelGrid<PointTI> vg;
    vg.setInputCloud(cloud_src);
    vg.setLeafSize(1.0f, 1.0f, 1.0f);
    vg.filter(*cloud_src_down);

    vg.setInputCloud(cloud_target);
    vg.setLeafSize(0.8f, 0.8f, 0.8f);
    vg.filter(*cloud_target_down);
    std::cout<<"Remain "<< cloud_src_down->size() <<" data points of input file after Downsampling"<< std::endl;
    std::cout<<"Remain "<< cloud_target_down->size() <<" data points of target file after Downsampling"<< std::endl;


    std::vector<Eigen::Vector3d> target_pts;
    std::vector<Eigen::Vector3d> src_pts;
    std::vector<Eigen::Vector3d> target_pts_down;
    std::vector<Eigen::Vector3d> src_pts_down;
    target_pts_down.resize(cloud_target_down->size());
    src_pts_down.resize(cloud_src_down->size());
    src_pts.resize(cloud_src->size());

    for(int i=0; i<cloud_target_down->points.size(); ++i){
        target_pts_down[i][0] = cloud_target_down->points[i].x;
        target_pts_down[i][1] = cloud_target_down->points[i].y;
        target_pts_down[i][2] = cloud_target_down->points[i].z;
    }

    for(int i=0; i<cloud_src_down->points.size(); ++i){
        src_pts_down[i][0] = cloud_src_down->points[i].x;
        src_pts_down[i][1] = cloud_src_down->points[i].y;
        src_pts_down[i][2] = cloud_src_down->points[i].z;
    }

    for(int i=0; i<cloud_src->points.size(); ++i){
        src_pts[i][0] = cloud_src->points[i].x;
        src_pts[i][1] = cloud_src->points[i].y;
        src_pts[i][2] = cloud_src->points[i].z;
    }


    double initial_pose[6] = {0,0,0,0,0,0};

    pcl::console::TicToc tt;

    RegistrationFrames reg{30, initial_pose, src_pts_down, target_pts_down};   //要固定的一帧放在前面
    tt.tic();
    reg.get_pose();
    std::cout<<"[Time for Registration: " <<tt.toc() << "ms]" <<"\n";


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("before Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud_target, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud_target, color, "target cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color2(cloud_src, 220, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud_src, color2, "src cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "src cloud");
    viewer->addCoordinateSystem (3.0);
    viewer->setBackgroundColor(0,0,0);
    while (!viewer->wasStopped()){
        viewer->spinOnce(50);
    }

    reg.view_registration(src_pts, cloud_target);

}