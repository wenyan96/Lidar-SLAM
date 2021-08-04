#include "feature_3dspace.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/time.h>
#include<pcl/features/principal_curvatures.h>

Feature3Dspace::Feature3Dspace(){};

void Feature3Dspace::initialize(PointCloudTI::Ptr cloud_in){
    //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_(new pcl::PointCloud<pcl::Normal>);
    cloud_in_ = cloud_in;
    //cloud_curvature_ = new float[128000];
}



void Feature3Dspace::calculate_allnormal(const PointCloudT::Ptr& cloud_in,
                                         pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals){
    //计算法线
    pcl::NormalEstimation<PointT, pcl::Normal> nor;
    nor.setInputCloud(cloud_in);
    pcl::console::TicToc tt;
    pcl::console::TicToc tt2;
    tt.tic();
    tt2.tic();
    //建立kdtree来进行近邻点集搜索
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    std::cout << "[time for build kdTree: " << tt.toc() << " ms ]" << std::endl;
    //为kdtree添加点运数据
    tree->setInputCloud(cloud_in);
    std::cout << "[time for build kdTree: " << tt.toc() << " ms ]" << std::endl;

    nor.setSearchMethod(tree);
    //点云法向计算时，需要所搜的近邻点大小 ne.setRadiusSearch (0.03);
    nor.setKSearch(20);
    //开始进行法向计算
    nor.setViewPoint (0.0,  0.0, 9999.0);
    nor.compute(*cloud_normals);
    std::cout << "[time for normal calculation: " << tt2.toc() << " ms ]" << std::endl;

    std::cout<< "cloud_normals size " << cloud_normals->size() << std::endl;

};

void Feature3Dspace::view_normals(const PointCloudT::Ptr& cloud_in,
                                  pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals) {


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Normals Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_in, 0, 238, 238);
    viewer->addPointCloud<pcl::PointXYZ> (cloud_in, color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_in, cloud_normals, 1, 0.2, "normals");
    viewer->addCoordinateSystem (3.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }

}


void Feature3Dspace::calculate_curv(PointCloudT::Ptr &cloud_in,
                                    pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                                    PointCloudT::Ptr &cloud_cur,
                                    PointCloudT::Ptr &cloud_curxs) {

    //计算法线
    pcl::NormalEstimation<PointT, pcl::Normal> nor;
    nor.setInputCloud(cloud_in);
    pcl::console::TicToc tt3;
    tt3.tic();
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    std::cout << "[time for build kdTree: " << tt3.toc() << " ms ]" << std::endl;
    tree->setInputCloud(cloud_in);
    std::cout << "[time for setting data: " << tt3.toc() << " ms ]" << std::endl;

    nor.setSearchMethod(tree);
    nor.setKSearch(20);
    nor.setViewPoint(0.0, 0.0, 9999.0);
    nor.compute(*cloud_normals);
    std::cout << "[time for computing curv : " << tt3.toc() << " ms ]" << std::endl;


    //该处曲率:最小特征值与三个特征值之和的比值
    //TODO:用排序算法
    for (int i = 0; i < cloud_in->points.size(); i++) {
        if (cloud_normals->points[i].curvature > 0.05) {
            cloud_cur->push_back(cloud_in->points[i]); //orange
            if (cloud_normals->points[i].curvature > 0.12) {
                cloud_curxs->push_back(cloud_in->points[i]); //blue
            }
        }
    }

    /*
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>pc;
    pc.setInputCloud(cloud_in);
    pc.setInputNormals(cloud_normals);
    pc.setSearchMethod(tree);
    pc.setKSearch(10);
    pc.compute(*cloud_curvatures);

    for(int i=0; i<100; ++i) {
        pcl::PrincipalCurvatures descriptor = cloud_curvatures->points[i];
        std::cout << descriptor << " " << std::endl;
    }
     */
}

void Feature3Dspace::calculate_allnormal_omp(const PointCloudT::Ptr& cloud_in){
    //计算法线
    pcl::NormalEstimationOMP<PointT, pcl::Normal> nor;
    nor.setNumberOfThreads(8);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    nor.setInputCloud(cloud_in);

    //建立kdtree来进行近邻点集搜索
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    //为kdtree添加点运数据
    tree->setInputCloud(cloud_in);

    nor.setSearchMethod(tree);
    //点云法向计算时，需要所搜的近邻点大小 ne.setRadiusSearch (0.03);
    nor.setKSearch(20);
    //开始进行法向计算
    nor.compute(*cloud_normals);
    nor.setViewPoint (0.0,  0.0,  0.0);
    std::cout<< "cloud_normals size " << cloud_normals->size() << std::endl;

};

void Feature3Dspace::calculate_normal(const PointCloudT::Ptr cloud_in, const std::vector<int>& indices,
                      Eigen::Vector4f& plane_params, float& curvature){

    //indices表示来自点云的k-最近邻集合
    pcl::NormalEstimation<PointT, pcl::Normal> nor;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    nor.setInputCloud(cloud_in);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_in);
    nor.setSearchMethod(tree);
    //ne.setRadiusSearch (0.03);
    nor.setKSearch(20);
    nor.setViewPoint (0.0,  0.0,  0.0);  //default

    nor.computePointNormal(*cloud_in, indices, plane_params, curvature);
    nor.compute(*cloud_normals);
    std::cout<< "cloud_normals size " << cloud_normals->size() << std::endl;
};


bool Feature3Dspace::extract_surface(const PointCloudTI::Ptr& cloud_in,
                                     PointCloudTI::Ptr& ground_cloud,
                                     PointCloudTI::Ptr& facade_cloud,
                                     PointCloudTI::Ptr& facade_cloud2,
                                     pcl::ModelCoefficients::Ptr& coefficient1,
                                     pcl::ModelCoefficients::Ptr& coefficient2,
                                     pcl::ModelCoefficients::Ptr& coefficient3) {

    pcl::console::TicToc tt;
    tt.tic();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients (true);

    //提取地面
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.4);

    seg.setInputCloud (cloud_in);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        return false;
    }

    // estimated plane parameters (in ax + by + cz + d = 0 form).
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    *coefficient1 = *coefficients;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    //for (std::size_t i = 0; i < inliers->indices.size (); ++i)
    //    for (const auto& idx: inliers->indices)
    //        std::cerr << idx << "    " << cloud_in->points[idx].x << " "
    //                  << cloud_in->points[idx].y << " "
    //                  << cloud_in->points[idx].z << std::endl;
    std::vector<int> ground = inliers->indices;
    pcl::copyPointCloud(*cloud_in, ground, *ground_cloud);

    PointCloudTI::Ptr tmp(new PointCloudTI);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*tmp);
    std::cout << "[time for extract ground surface (RANSAC) : " << tt.toc() << " ms ]" << std::endl;


    //提取立面
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setDistanceThreshold (0.4);
    seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
    seg.setEpsAngle(0.35);  //pcl::deg2rad(15)  0.35rad ~ 20 deg

    seg.setInputCloud(tmp);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        return false;
    }
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    *coefficient2 = *coefficients;
    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

    std::vector<int> facade = inliers->indices;

    pcl::copyPointCloud(*tmp, facade, *facade_cloud);

    extract.setInputCloud(tmp);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*tmp);

    //facade2
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setDistanceThreshold (0.4);
    seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
    seg.setEpsAngle(0.4);
    seg.setInputCloud(tmp);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        return false;
    }
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    *coefficient3 = *coefficients;
    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

    std::vector<int> facade2 = inliers->indices;

    pcl::copyPointCloud(*tmp, facade2, *facade_cloud2);


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("surface extraction"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (3.0, "global");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color1(cloud_in, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZI> (cloud_in, cloud_color1,  "input cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color2(ground_cloud, 0, 0, 238);
    viewer->addPointCloud<pcl::PointXYZI> (ground_cloud, cloud_color2,  "surf cloud");

    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "surface extraction");
    pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color3(facade_cloud, 0, 238, 160);
    viewer->addPointCloud<pcl::PointXYZI> (facade_cloud, cloud_color3,  "facade cloud");
    //viewer->initCameraParameters ();

    pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color4(facade_cloud2, 140, 50, 160);
    viewer->addPointCloud<pcl::PointXYZI> (facade_cloud2, cloud_color4,  "facade cloud2");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
    return true;
}



bool Feature3Dspace::extract_edge(pcl::ModelCoefficients::Ptr& coefficient1,
                                  pcl::ModelCoefficients::Ptr& coefficient2,
                                  pcl::ModelCoefficients::Ptr& coefficient_line){
    //方向向量n=n1×n2=(b1*c2-c1*b2,c1*a2-a1*c2,a1*b2-b1*a2)
    pcl::ModelCoefficients temcoefs;
    double a1, b1, c1, d1, a2,b2, c2, d2;
    double tempy, tempz;
    a1= coefficient1->values[0];
    b1= coefficient1->values[1];
    c1= coefficient1->values[2];
    d1= coefficient1->values[3];
    a2= coefficient2->values[0];
    b2= coefficient2->values[1];
    c2= coefficient2->values[2];
    d2= coefficient2->values[3];
    tempz= -(d1 / b1 - d2 / b2) / (c1 / b1 - c2 / b2);
    tempy= (-c1 / b1)*tempz - d1 / b1;
    //令x=0; 后三个为方向向量
    coefficient_line->values.resize (6);
    coefficient_line->values[0] = 0.0;
    coefficient_line->values[1] = tempy;
    coefficient_line->values[2] = tempz;
    coefficient_line->values[3] = b1*c2 - c1*b2;
    coefficient_line->values[4] = c1*a2 - a1*c2;
    coefficient_line->values[5] = a1*b2 - b1*a2;

    std::cerr << "Line coefficients: " << coefficient_line->values[0] << " "
              << coefficient_line->values[1] << " "
              << coefficient_line->values[2] << " "
              << coefficient_line->values[3] << " "
              << coefficient_line->values[4] << " "
              << coefficient_line->values[5] << " "<<std::endl;

    /*
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("line extraction"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (3.0, "global");
    pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color1(cloud_in, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZI> (cloud_in, cloud_color1,  "input cloud");

    //pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color2(coefficient_line, 0, 0, 238);
    //TODO: Problem!! viewer->addLine
    //pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color(0, 238, 160);

    while (!viewer->wasStopped ()){

        viewer->spinOnce(100);
    }
     */

    return true;
}


void Feature3Dspace::xyz2xyzi_ptr(const PointCloudT::Ptr cloudin, PointCloudTI::Ptr& cloudout){

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


void Feature3Dspace::view_clouds(std::vector<PointCloudTI::Ptr> clouds, bool feature_flag) {
    int size = clouds.size();
    pcl::visualization::PCLVisualizer viewer("multiple clouds");

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(3.0);

    int point_size = 2;
    if (!feature_flag) {
        for (int i = 0; i < size; i++) {
            pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color(clouds[i], 175, 238, 238);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
                                                    std::to_string(i));
            viewer.addPointCloud(clouds[i], cloud_color, std::to_string(i));
            std::cout << "num" << i << std::endl;
        }
    } else {
        int r[] = {230, 255, 205, 0, 0};  //lavender, orange, IndianRed, blue, deepskyblue
        int g[] = {230, 165, 92, 0, 191};
        int b[] = {250, 0, 92, 255, 255};

        for (int i = 0; i < size; i++) {
            pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color(clouds[i], r[i], g[i], b[i]);
            viewer.addPointCloud(clouds[i], cloud_color, std::to_string(i));
            if (i > 0)
                point_size = 5;
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
                                                    std::to_string(i));

            std::cout << "num" << i << std::endl;
        }
    }


    while (!viewer.wasStopped()) {
        viewer.spinOnce(50);
    }
    //viewer.spinOnce(1000);
    //viewer.close();

}


void Feature3Dspace::view_clouds(std::vector<PointCloudT::Ptr> clouds, bool feature_flag) {
    int size = clouds.size();
    pcl::visualization::PCLVisualizer viewer("multiple clouds");

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(3.0);

    int point_size = 2;
    if (!feature_flag) {
        for (int i = 0; i < size; i++) {
            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(clouds[i], 175, 238, 238);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
                                                    std::to_string(i));
            viewer.addPointCloud(clouds[i], cloud_color, std::to_string(i));
            std::cout << "num" << i << std::endl;
        }
    } else {
        int r[] = {230, 255,  0, 205, 0};  //lavender, orange, IndianRed, blue, deepskyblue
        int g[] = {230, 165,  0, 92, 191};
        int b[] = {250, 0,  255, 92, 255};

        for (int i = 0; i < size; i++) {
            //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("multiple clouds"));
            //pcl::visualization::PointCloudColorHandlerGenericField<PointTI> point_cloud_color_handler(cloud, "intensity");

            //pcl::visualization::PointCloudColorHandlerCustom<PointTI> cloud_color(clouds[i], 175, 238, 238);

            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(clouds[i], r[i], g[i], b[i]);
            viewer.addPointCloud(clouds[i], cloud_color, std::to_string(i));
            if (i > 0)
                point_size = 5;
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
                                                    std::to_string(i));

            std::cout << "num" << i << std::endl;
        }
    }


    while (!viewer.wasStopped()) {
        viewer.spinOnce(50);
    }

}


