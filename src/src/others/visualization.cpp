#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/program_options.hpp>
#include "visualization.h"



Visualization::Visualization() {};

void Visualization::view_clouds(std::vector<PointCloudTI::Ptr> clouds, bool feature_flag )
{
    int size = clouds.size();
    pcl::visualization::PCLVisualizer viewer("multiple clouds");

    viewer.setBackgroundColor(0,0,0);
    viewer.addCoordinateSystem (3.0);

    int point_size = 2;
    if(!feature_flag){
        for( int i =0; i< size; i++) {
            pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color(clouds[i], 175, 238, 238);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,std::to_string(i));
            viewer.addPointCloud(clouds[i], cloud_color, std::to_string(i));
            std::cout<<"num" << i<< std::endl;
        }
    }

    else{
        int r[] = {230, 255, 205, 0, 0};  //lavender, orange, IndianRed, blue, deepskyblue
        int g[] = {230, 165, 92, 0, 191};
        int b[] = {250, 0,  92, 255, 255};

        for( int i =0; i< size; i++){
            //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("multiple clouds"));
            //pcl::visualization::PointCloudColorHandlerGenericField<PointTI> point_cloud_color_handler(cloud, "intensity");

            //pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color(clouds[i], 175, 238, 238);

            pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color(clouds[i], r[i], g[i], b[i]);
            viewer.addPointCloud(clouds[i], cloud_color, std::to_string(i));
            if(i>0)
                point_size = 5;
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,std::to_string(i));

            std::cout<<"num" << i<< std::endl;
        }
    }


    while (!viewer.wasStopped())
    {
        viewer.spinOnce(50);
    }
    //viewer.spinOnce(1000);
    //viewer.close();

}


int main(){
    /*
    PointCloudTI::Ptr c1(new PointCloudTI);
    PointCloudTI::Ptr c2(new PointCloudTI);
    PointCloudTI::Ptr c3(new PointCloudTI);
    std::vector<PointCloudTI::Ptr> clds;

    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd", *c1);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/20.pcd", *c2);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/40.pcd", *c3);
    clds.push_back(c1);
    clds.push_back(c2);
    clds.push_back(c3);

    Visualization vv;
    vv.view_clouds(clds, true);
     */

}
