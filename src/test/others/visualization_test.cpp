#include "range_img.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;

int main(){

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr yyy(new pcl::PointCloud<pcl::PointXYZINormal>);
    yyy->width = 5;
    yyy->height = 1;
    yyy->points.resize(yyy->width * yyy->height);
    for(int i=0; i<5; ++i){
        yyy->points[i].x = i;
        yyy->points[i].y = i;
        yyy->points[i].z = 2;
        yyy->points[i].intensity = 2;
        yyy->points[i].normal_x = 0;
        yyy->points[i].normal_y = 0;
        yyy->points[i].normal_z = 1;
    }

    pcl::visualization::PCLVisualizer viewer("multiple clouds");
    viewer.setBackgroundColor(0,0,0);
    viewer.addCoordinateSystem (3.0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal>visColor_2 (yyy, "intensity");
    viewer.addPointCloud<pcl::PointXYZINormal>(yyy , visColor_2, "cloud_name");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,"cloud_name");


    while (!viewer.wasStopped())
    {
        viewer.spinOnce(50);
    }



}
