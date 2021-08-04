#include "../../include/range_img/feature_3dspace.h"
#include "../../include/range_img/visualization.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>

int main() {

    // Methods List
    // "1.0" : normal estimation
    // "1.1" : normal estimation OMP
    // "   " : partial normal estimation with indices (How to get indices properly?)
    // "1.5" : curvature estimation (edge points)
    // "2.0" : surface estimation + edge estimation
    //         [time for extract ground surface (RANSAC) : 61.3133 ms ]

    float method_ver = 1.0;


    if (method_ver == 1.0) {
        PointCloudT::Ptr cloud_in(new PointCloudT);
        pcl::io::loadPCDFile("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd",
                             *cloud_in);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

        Feature3Dspace nn;
        nn.calculate_allnormal(cloud_in, cloud_normals);
        nn.view_normals(cloud_in, cloud_normals);
    }

    if (method_ver == 1.1) {}

    if (method_ver == 1.5) {

        PointCloudT::Ptr cloud_in(new PointCloudT);
        pcl::io::loadPCDFile("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd",
                             *cloud_in);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        PointCloudT::Ptr cloud_cur(new PointCloudT);
        PointCloudT::Ptr cloud_curxs(new PointCloudT);
        cloud_cur->resize(12000);
        Feature3Dspace nn;

        nn.calculate_curv(cloud_in, cloud_normals, cloud_cur, cloud_curxs);

        std::vector<PointCloudT::Ptr> clouds;
        clouds.push_back(cloud_in);
        clouds.push_back(cloud_cur);
        clouds.push_back(cloud_curxs);
        nn.view_clouds(clouds, true);
    }

    if(method_ver == 2.0) {
        //extract surface
        PointCloudTI::Ptr cloud_in(new PointCloudTI);
        PointCloudTI::Ptr ground_cloud(new PointCloudTI);
        PointCloudTI::Ptr facade_cloud(new PointCloudTI);
        PointCloudTI::Ptr facade_cloud2(new PointCloudTI);
        pcl::ModelCoefficients::Ptr coefficient1(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr coefficient2(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr coefficient3(new pcl::ModelCoefficients);
        pcl::io::loadPCDFile("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd",
                             *cloud_in);
        Feature3Dspace surf_obj;
        surf_obj.extract_surface(cloud_in, ground_cloud, facade_cloud,facade_cloud2,
                                 coefficient1, coefficient2, coefficient3);

        pcl::ModelCoefficients::Ptr coefficient_line(new pcl::ModelCoefficients);
        surf_obj.extract_edge(coefficient1,
                              coefficient2,
                              coefficient_line);
    }


}

