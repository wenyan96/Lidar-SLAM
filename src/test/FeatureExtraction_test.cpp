#include "feature_extraction/FeatureExtraction.h"
#include "feature_extraction/FeatureExtraction_interface.h"
#include <pcl/console/time.h>
#include "preprocessing/Preprocessing.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include "common/utils.h"
#include <pcl/point_types_conversion.h>
#include <pcl/io/pcd_io.h>  //文件输入输出
#include <fstream>
#include <iostream>
#include <pcl/point_types.h>  //点类型相关定义

int main(){

    //*****************CHECK Feature Point and Normal for one frame***************

    pcl::console::TicToc tt;
    pcl::console::TicToc tt1;
    tt.tic();
    PointCloudTI::Ptr cloud_in(new PointCloudTI);
    //pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/0.pcd",
    //                              *cloud_in);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/test50.pcd",
                                          *cloud_in);

    Preprocessing pre;
    tt1.tic();
    pre.preprocessingExecution(cloud_in);
    auto cloud_order = pre.orderResult();
    std::cout<< "Time: " << tt.toc() << "ms" <<std::endl;
    std::cout<< "Time without initialization: " << tt1.toc() << "ms" <<std::endl;
    auto cloud_segment = pre.segmentResult();
    std::cout<<"check 0" << std::endl;
    auto seg_n = pre.segmentNormals();
    std::cout<<"check 1" << std::endl;
    pre.normalDisplay(cloud_segment, seg_n);
    FeatureExtraction::index_s index_info;
    index_info.start = pre.getStartInd();
    index_info.end = pre.getEndInd();
    std::cout<<"check 2" << std::endl;
    index_info.col = pre.getColInd();
    index_info.range = pre.getSegRangeMat();
    std::cout<<"check 4" << std::endl;

    //FeatureExtraction(cloud_segment, index_info, seg_n);
    FeatureExtraction fe(cloud_segment, index_info, seg_n);
    fe.runExtraction(4,10,4, "normal");
    auto surf_less_flat = fe.surfLessFlatResult();
    pre.cloudDisplay(surf_less_flat, cloud_order, "surf_less_flat");
    auto surf_flat = fe.surfFlatResult();
    pre.cloudDisplay(surf_flat, cloud_order, "surf_flat");
    auto edge_less_sharp = fe.edgeLessSharpResult();
    pre.cloudDisplay(edge_less_sharp, cloud_order, "edge_less_sharp");
    auto edge_sharp = fe.edgeSharpResult();
    pre.cloudDisplay(edge_sharp, cloud_order,  "edge_sharp");
    std::cout << "size of surf_less" << surf_less_flat->size() << std::endl;

    auto ground = pre.groundResult();
    auto ground_less = pre.groundDsResult();
    pre.cloudDisplay(ground, cloud_order,  "ground");
    pre.cloudDisplay(ground_less, cloud_order,  "ground_less");

    //PointCloudT::Ptr edge_less_last_T(new PointCloudT);
    //pcl::copyPointCloud(*edge_less_last, *edge_less_last_T);
    //pcl::io::savePCDFileASCII ("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/target_edge.pcd",
    //                           *edge_less_last_T);
    //std::cout << "edge_less_last_T size " << edge_less_last_T->size() <<std::endl;



    //*****************CHECK Feature Points for two frames***************
    /*
    PointCloudTI::Ptr cloud_in(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/0.pcd",
                                  *cloud_in);
    PointCloudTI::Ptr cloud_in2(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/1.pcd",
                                  *cloud_in2);

    Preprocessing pre;
    pre.preprocessingExecution(cloud_in);
    auto cloud_order = pre.orderResult();
    auto cloud_segment = pre.segmentResult();
    auto seg_n = pre.segmentNormals();
    pre.normalDisplay(cloud_segment, seg_n);

    Preprocessing pre2;
    pre2.preprocessingExecution(cloud_in2);
    auto cloud_order2 = pre2.orderResult();
    auto cloud_segment2 = pre2.segmentResult();
    auto seg_n2 = pre2.segmentNormals();
    pre2.normalDisplay(cloud_segment2, seg_n2);

    PointCloudT::Ptr cloud_in_xyz(new PointCloudT);
    PointCloudT::Ptr cloud_in_xyz2(new PointCloudT);
    cloud_in_xyz->resize(cloud_in->size());
    cloud_in_xyz2->resize(cloud_in2->size());
    copyPointCloud(*cloud_segment, *cloud_in_xyz);
    copyPointCloud(*cloud_segment2, *cloud_in_xyz2);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Normals Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_in_xyz, 0, 138, 238);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_in_xyz2, 0, 250, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in_xyz, color, "sample cloud for normal");
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in_xyz2, color2, "sample cloud for normal2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                             "sample cloud for normal");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                             "sample cloud for normal2");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_in_xyz, seg_n, 1, 0.3, "normals");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_in_xyz2, seg_n2, 1, 0.3, "normals2");
    viewer->addCoordinateSystem(3.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }
    */



    //*****************CHECK Normal for two frames***************
    /*
    PointCloudTI::Ptr cloud_in(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/0.pcd",
                                  *cloud_in);
    PointCloudTI::Ptr cloud_in2(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/1.pcd",
                                  *cloud_in2);

    Preprocessing pre;
    pre.preprocessingExecution(cloud_in);
    auto cloud_order = pre.orderResult();
    auto cloud_segment = pre.segmentResult();
    auto seg_n = pre.segmentNormals();
    FeatureExtraction::index_s index_info;
    index_info.start = pre.getStartInd();
    index_info.end = pre.getEndInd();
    index_info.col = pre.getColInd();
    index_info.range = pre.getSegRangeMat();
    //FeatureExtraction(cloud_segment, index_info, seg_n);
    FeatureExtraction fe(cloud_segment, index_info, seg_n);
    fe.runExtraction(4,20,4);
    auto surf_less_flat = fe.surfLessFlatResult();
    auto surf_flat = fe.surfFlatResult();
    auto edge_less_sharp = fe.edgeLessSharpResult();
    auto edge_sharp = fe.edgeSharpResult();
    auto ground = pre.groundResult();
    auto ground_less = pre.groundDsResult();

    Preprocessing pre2;
    pre2.preprocessingExecution(cloud_in2);
    auto cloud_order2 = pre2.orderResult();
    auto cloud_segment2 = pre2.segmentResult();
    auto seg_n2 = pre2.segmentNormals();
    FeatureExtraction::index_s index_info2;
    index_info2.start = pre2.getStartInd();
    index_info2.end = pre2.getEndInd();
    index_info2.col = pre2.getColInd();
    index_info2.range = pre2.getSegRangeMat();
    //FeatureExtraction(cloud_segment, index_info, seg_n);
    FeatureExtraction fe2(cloud_segment2, index_info2, seg_n2);
    fe2.runExtraction(4,20,4);
    auto surf_less_flat2 = fe2.surfLessFlatResult();
    auto surf_flat2 = fe2.surfFlatResult();
    auto edge_less_sharp2 = fe2.edgeLessSharpResult();
    auto edge_sharp2 = fe2.edgeSharpResult();
    auto ground2 = pre2.groundResult();
    auto ground_less2 = pre2.groundDsResult();


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Features Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud_in, 250, 238, 238);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color2(cloud_in2, 220, 230, 200);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color3(surf_less_flat, 250, 20, 50);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color4(surf_flat2, 50, 200, 50); //green
    viewer->addPointCloud<pcl::PointXYZI>(cloud_in, color, "sample cloud");
    viewer->addPointCloud<pcl::PointXYZI>(cloud_in2, color2, "sample cloud2");
    viewer->addPointCloud<pcl::PointXYZI>(surf_less_flat, color3, "sample cloud3");
    viewer->addPointCloud<pcl::PointXYZI>(surf_flat2, color4, "sample cloud4");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                             "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                             "sample cloud2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                             "sample cloud3");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                             "sample cloud4");
    viewer->addCoordinateSystem(3.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }
     */




}