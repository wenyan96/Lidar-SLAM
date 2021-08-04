#include "preprocessing/Preprocessing.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include "common/utils.h"
#include <pcl/point_types_conversion.h>

int main(){

    pcl::console::TicToc tt;
    pcl::console::TicToc tt1;
    tt.tic();
    PointCloudTI::Ptr cloud_in(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/453.pcd",
                                 *cloud_in);

    Preprocessing pre;
    tt1.tic();
    pre.preprocessingExecution(cloud_in, 4, 20, 4);
    std::cout<< "Time: " << tt.toc() << "ms" <<std::endl;
    std::cout<< "Time without initialization: " << tt1.toc() << "ms" <<std::endl;
    auto cloud_order = pre.orderResult();
    auto cloud_ground = pre.groundResult();
    auto cloud_segment = pre.segmentResult();
    auto cloud_segment_pure = pre.segmentPureResult();
    auto cloud_outliers = pre.outliersResult();
    auto normals = pre.normalsResult();
    //pre.cloudDisplay(cloud_order, "cloud_order");
    //pre.cloudDisplay(cloud_ground, "cloud_ground");
    //pre.cloudDisplay(cloud_segment, "cloud_segment");
    //pre.cloudDisplay(cloud_segment_pure, "cloud_segment_pure");
    //pre.cloudDisplay(cloud_outliers, "cloud_outliers");
    //pre.normalDisplay();


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

