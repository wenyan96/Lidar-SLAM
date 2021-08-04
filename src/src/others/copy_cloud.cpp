#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointTI> PointCloudTI;

void xyz2xyzi(PointCloudT cloudin, PointCloudTI& cloudout){

    cloudout.width    = cloudin.width ;
    cloudout.height   = cloudin.height;
    cloudout.is_dense = cloudin.is_dense;
    cloudout.points.resize(cloudin.width * cloudin.height);
    for (int i=0; i<cloudin.points.size(); i++){
        cloudout.points[i].x = cloudin.points[i].x;
        cloudout.points[i].y = cloudin.points[i].y;
        cloudout.points[i].z = cloudin.points[i].z;
        cloudout.points[i].intensity = 0;
    }
}

void xyzi2xyzi(PointCloudT::Ptr cloudin, PointCloudTI::Ptr& cloudout){

    *cloudout.width    = *cloudin.width ;
    *cloudout.height   = *cloudin.height;
    *cloudout.is_dense = *cloudin.is_dense;
    *cloudout.points.resize(*cloudin.width * *cloudin.height);
    for (int i=0; i<cloudin.points.size(); i++){
        *cloudout.points[i].x = *cloudin.points[i].x;
        *cloudout.points[i].y = *cloudin.points[i].y;
        *cloudout.points[i].z = *cloudin.points[i].z;
        *cloudout.points[i].intensity = 0;
    }
}

//cloudout之后会随着in变化一起变化吗？
void xyz2xyzi_ptr(PointCloudT::Ptr cloudin, PointCloudTI::Ptr& cloudout){

    cloudout->width    = cloudin->width ;
    cloudout->height   = cloudin->height;
    cloudout->is_dense = cloudin->is_dense;
    cloudout->points.resize(cloudin->width * cloudin->height);
    for (int i=0; i<cloudin->points.size(); i++){
        cloudout->points[i].x = cloudin->points[i].x;
        cloudout->points[i].y = cloudin->points[i].y;
        cloudout->points[i].z = cloudin->points[i].z;
        cloudout->points[i].intensity = 0;
    }
}

int main(){

    //PointCloudT::Ptr cloud(new PointCloudT);
    PointCloudT cloud_r;
    //PointCloudTI::Ptr cloudI(new PointCloudTI);
    PointCloudTI cloudI_r;
    pcl::io::loadPCDFile<PointT>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd", cloud_r);
    xyz2xyzi(cloud_r, cloudI_r);
    pcl::io::savePCDFileASCII<PointTI>("copy_cloud_i4.pcd", cloudI_r);
}
