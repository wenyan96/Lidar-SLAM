#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/program_options.hpp>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;

void displayCloud(PointCloudTI::Ptr cloud, const std::string& window_name)
{
    if (cloud->size() < 1)
    {
        std::cout << window_name << " display failure. Cloud contains no points\n";
        return;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));

    pcl::visualization::PointCloudColorHandlerGenericField<PointTI> point_cloud_color_handler(cloud, "intensity");

    viewer->addPointCloud< PointTI >(cloud, point_cloud_color_handler, "id");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "id");
    viewer->setBackgroundColor(255,255,255);
    //viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
    bool close_window = false;
    while (!viewer->wasStopped() && !close_window){
        viewer->spinOnce(50);
    }

    viewer->close();
}

int main(){
    std::string cloud_path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd";
    PointCloudTI::Ptr cloud(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>(cloud_path, *cloud);

    const std::string window_name1= "visualize_intensity";
    const std::string window_name2= "visualize_scan_order";
    const std::string window_name3= "visualize_z_value";
    displayCloud(cloud, window_name1);

    for(int i=0; i<cloud->size(); i++){
        cloud->points[i].intensity = i;
    }
    displayCloud(cloud, window_name2);

    for(int i=0; i<cloud->size(); i++){
        cloud->points[i].intensity = cloud->points[i].z;
    }
    displayCloud(cloud, window_name3);


}