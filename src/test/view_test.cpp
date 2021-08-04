#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;

using PointTI  = pcl::PointXYZI;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer){
    viewer.setBackgroundColor (1, 1, 1);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer){
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    viewer.setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_last_color(cloud, 0, 250, 0);
    viewer.addPointCloud(cloud, cloud_last_color, "in");

    //FIXME: possible race condition here:
    user_data++;}

int main (){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile ("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/1.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //blocks until the cloud is actually rendered


    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);

    while (!viewer.wasStopped ())
    {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...


        user_data++;
    }
    return 0;}

