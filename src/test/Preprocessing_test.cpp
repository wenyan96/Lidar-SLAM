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

    pcl::console::TicToc tt;
    pcl::console::TicToc tt1;
    tt.tic();
    PointCloudTI::Ptr cloud_in(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/454.pcd",
                                 *cloud_in);

    Preprocessing pre;
    tt1.tic();
    pre.preprocessingExecution(cloud_in);
    std::cout<< "Time: " << tt.toc() << "ms" <<std::endl;
    std::cout<< "Time without initialization: " << tt1.toc() << "ms" <<std::endl;
    auto cloud_order = pre.orderResult();
    auto cloud_ground = pre.groundResult();
    auto cloud_ground_ds = pre.groundDsResult();
    auto cloud_outliers = pre.outliersResult();
    auto normals = pre.normalsResult();
    pre.normalDisplay(cloud_order, normals);
    std::cout<<"check 0" << std::endl;
    auto ground = pre.groundResult();
    std::cout<<"check 01" << std::endl;

    auto ground_n = pre.groundNormals();
    std::cout<<"check 1" << std::endl;

    pre.normalDisplay(ground, ground_n);

    auto seg = pre.segmentResult();
    auto seg_n = pre.segmentNormals();
    pre.normalDisplay(seg, seg_n);
    std::cout<<"check 2" << std::endl;


    /*
    for(int i =0; i< 160; i++){
        std::cout << ground->points[i].intensity << " ";
    }
     */
    //pre.normalDisplay(seg, seg_n);


    pre.cloudDisplay(cloud_order, "cloud_order");
    pre.cloudDisplay(cloud_ground, "cloud_ground");
    pre.cloudDisplay(cloud_ground_ds, "cloud_ground_ds");
    //pre.cloudDisplay(cloud_segment, "cloud_segment");
    pre.cloudDisplay(cloud_outliers, "cloud_outliers");
    pre.normalDisplay(ground, ground_n);

    pre.normalDisplay(cloud_order, normals);



    /*
    auto surf_flat = pre.surfFlatResult();
    auto surf_less = pre.surfLessFlatResult();
    auto edge_sharp = pre.edgeSharpResult();
    auto edge_less = pre.edgeLessSharpResult();
    pre.cloudDisplay(surf_flat, "surf flat");
    pre.cloudDisplay(surf_less, "surf less flat");
    pre.cloudDisplay(edge_sharp, "edge sharp");
    pre.cloudDisplay(edge_less, "edge less sharp");
    */

}

