#include "sri_derivative.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char *argv[]) {


    //SRI r2;
    //PointCloudTI::Ptr cloud_in(new PointCloudTI);
    //pcl::io::loadPCDFile<PointTI>(argv[1], *cloud_in);

    //r2.display_cloud(cloud_in, "point cloud");
    //cv::Mat range_mat = cv::Mat(BeamNum, HorizonNum, CV_64F,cv::Scalar::all(500.0));

    int h = 64;
    int w = 1800;
    int h_1 = h - 1;
    int w_1 = w - 1;
    PointCloudT::Ptr cloud_in(new PointCloudT);
    PointCloudT::Ptr cloud_order(new PointCloudT);
    cloud_order->resize(w * h);
    PointT nanPoint;
    nanPoint.x = std::numeric_limits<double>::quiet_NaN();
    nanPoint.y = std::numeric_limits<double>::quiet_NaN();
    nanPoint.z = std::numeric_limits<double>::quiet_NaN();
    std::fill(cloud_order->points.begin(), cloud_order->points.end(), nanPoint);

    pcl::io::loadPCDFile<PointT>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/9.pcd",
                                 *cloud_in);

    std::vector<double> Theta;
    std::vector<double> Phi;
    cv::Mat range_mat = cv::Mat(h, w, CV_64F, cv::Scalar::all(255.0));
    //cv::Mat range_mat = cv::Mat(h, w, CV_64F, cv::Scalar::all(std::numeric_limits<double>::max()));

    int cloud_size = cloud_in->size();
    for (int i = 0; i < cloud_size; i++) {
        double x = cloud_in->points[i].x;
        double y = cloud_in->points[i].y;
        double z = cloud_in->points[i].z;

        //too close point is invalid
        double lidar_range = std::sqrt(x * x + y * y + z * z);
        if (lidar_range < 0.1) {
            continue;
        }

        double horizontal_angle = std::atan2(y, x);
        int col_index = round(-(w/360) * horizontal_angle * 180 / M_PI + w/2);
        if (col_index >= w)
            col_index -= w;
        if (col_index < 0 || col_index >= w) {   //?这里可以这样处理吗？
            continue;
        }


        //row_index_存储的是原点云pcd中点所对应的投影row
        double vertical_angle = std::asin(z / lidar_range);
        double row = (-vertical_angle * 180 / M_PI + 2) / (26.8/(h_1));
        int row_index = round(row);
        if (row_index < 0 || row_index >= h) {
            continue;
        }


        double range_val = range_mat.at<double>(row_index, col_index);
        if ( range_val != 255.0 && lidar_range != 255.0) {

            if (range_val >= lidar_range) {
                continue;
            } else {
                range_mat.at<double>(row_index, col_index) = lidar_range;
                int index = col_index + row_index * w;
                cloud_order->points[index].x = x;
                cloud_order->points[index].y = y;
                cloud_order->points[index].z = z;
            }
        } else {
            range_mat.at<double>(row_index, col_index) = lidar_range;
            int index = col_index + row_index * w;
            cloud_order->points[index].x = x;
            cloud_order->points[index].y = y;
            cloud_order->points[index].z = z;
        }

    }

    std::cout<<"--------------range mat---------------" << std::endl;
    for(int i = 0; i<30; i++){
        for(int j=61; j<88; j++){
            std::cout<< range_mat.at<double>(i,j) <<" ";
        }
        std::cout<< std::endl;
    }
    pcl::io::savePCDFile<PointT>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/cloud_order0.pcd", *cloud_order);
    pcl::visualization::PCLVisualizer::Ptr viewer_o(new pcl::visualization::PCLVisualizer("cloud_order"));
    viewer_o->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> coloro(cloud_order, 0, 238, 238);
    viewer_o->addPointCloud<pcl::PointXYZ>(cloud_order, coloro, "sample clou");
    viewer_o->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample clou");
    viewer_o->addCoordinateSystem(3.0);
    viewer_o->initCameraParameters();

    while (!viewer_o->wasStopped()) {
        viewer_o->spinOnce(1);
    }





    for (int i = 0; i < w; i++) {
        double k = -(i - w / 2.0) / (w / 2.0) * M_PI;
        //index0 - Pi, index一半 - 0   index w_1 - (-Pi)
        Theta.push_back(k);
    }

    for (int j = 0; j < h; j++) {
        double kk = -(j - h_1 / 2.0) / (h_1 / 2.0) * M_PI / 180 * 13.4 - 11.4 * M_PI / 180;
        //index0 - pi/3;    indexh_1 - (-pi/3)    范围是正负60（共120）;
        Phi.push_back(kk);
    }



    PointCloudT::Ptr input(new PointCloudT);
    input->resize(h * w);
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            input->points[i * w + j].x = range_mat.at<double>(i, j) * cos(Theta[j]) * cos(Phi[i]);
            input->points[i * w + j].y = range_mat.at<double>(i, j) * sin(Theta[j]) * cos(Phi[i]);
            input->points[i * w + j].z = range_mat.at<double>(i, j) * sin(Phi[i]);
        }
    }


    pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("input Viewer1"));
    viewer1->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(input, 0, 238, 238);
    viewer1->addPointCloud<pcl::PointXYZ>(input, color1, "sample cloud1");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
    viewer1->addCoordinateSystem(3.0);
    viewer1->initCameraParameters();

    while (!viewer1->wasStopped()) {
        viewer1->spinOnce(1);
    }

    //interpolation
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat res;
    //cv::morphologyEx(range_mat_, res, cv::MORPH_CLOSE, kernel);  //0初始化
    cv::morphologyEx(range_mat, res, cv::MORPH_OPEN, kernel);   //255初始化
    //cv::GaussianBlur(res, res, cv::Size( 3, 3), 0, 0 );
    //cv::medianBlur(res, res, 3);
    cv::Mat color_img = cv::Mat(h, w, CV_8U);

    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            color_img.at<unsigned char>(i, j) = (unsigned char) (range_mat.at<double>(i, j));
        }
    }
    applyColorMap(color_img, color_img, cv::COLORMAP_JET);
    cv::imshow("colorMap", color_img);
    cv::waitKey();
    double min, max;
    cv::minMaxIdx(range_mat, &min, &max);
    std::cout << "min lidar_range= " << min << std::endl;
    std::cout << "max lidar_range= " << max << std::endl;
    PointCloudT::Ptr input_after(new PointCloudT);
    input_after->resize(h * w);
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {

            input_after->points[i * w + j].x = res.at<double>(i, j) * cos(Theta[j]) * cos(Phi[i]);
            input_after->points[i * w + j].y = res.at<double>(i, j) * sin(Theta[j]) * cos(Phi[i]);
            input_after->points[i * w + j].z = res.at<double>(i, j) * sin(Phi[i]);
        }
    }




    /*
    for (int i = 40; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (res.at<double>(i, j) == 255.0) {
                res.at<double>(i, j) = 0;
            }
        }
    }
    */

    pcl::console::TicToc tt0;
    tt0.tic();
    cv::Mat dRdth = cv::Mat(h, w, CV_64F, cv::Scalar::all(0));
    cv::Mat dRdph = cv::Mat(h, w, CV_64F, cv::Scalar::all(0));
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            //if (j < w - 2) {
            //    dRdth.at<double>(i,j) = (res.at<double>(i, j + 2) - res.at<double>(i, j)) / (2 * (Theta[0] - Theta[1]));
            //}
            //if (i < h - 2) {
            //    dRdph.at<double>(i,j)  = (res.at<double>(i + 2, j) - res.at<double>(i, j)) / (2 * (Phi[0] - Phi[1]));
            //}

            if (j < w - 8) {
                dRdth.at<double>(i,j) = (res.at<double>(i, j + 5) + res.at<double>(i, j + 6) + res.at<double>(i, j + 7)
                                        + res.at<double>(i, j + 8) - res.at<double>(i, j) - res.at<double>(i, j+1)
                                        - res.at<double>(i, j+2) - res.at<double>(i, j+3))/ (8 * (Theta[0] - Theta[1]));
            }
            if (i < h - 2) {
                dRdph.at<double>(i,j)  = (res.at<double>(i + 2, j)  - res.at<double>(i, j))/ (2 * (Phi[0] - Phi[1]));
            }

        }
    }
    std::cout<< "[time tt0] " << tt0.toc() << std::endl;

    cv::Mat color_img2 = cv::Mat(h, w, CV_8U);
    double min2, max2;
    cv::minMaxIdx(dRdth, &min2, &max2);
    int k = 255/ (max2 - min2);
    std::cout<<"max2" << max2;
    std::cout<<"min2" << min2;

    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            color_img2.at<unsigned char>(i, j) = (unsigned char) (k * dRdth.at<double>(i, j));
        }
    }
    applyColorMap(color_img2, color_img2, cv::COLORMAP_JET);
    imshow("colorMap2", color_img2);
    cv::waitKey();
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->resize(w * h);

    Eigen::Matrix3d R_theta;
    Eigen::Matrix3d R_phi;
    Eigen::Matrix3d R;
    Eigen::Vector3d Partial;
    Eigen::Vector3d normal;

    pcl::console::TicToc tt;
    tt.tic();

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {

            R_theta << cos(Theta[j]), -sin(Theta[j]), 0,
                    sin(Theta[j]), cos(Theta[j]), 0,
                    0, 0, 1;
            R_phi << cos(Phi[i]), 0, -sin(Phi[i]),
                    0, 1, 0,
                    sin(Phi[i]), 0, cos(Phi[i]);
            R = R_theta * R_phi;
            Partial << 1,
                    1 / (res.at<double>(i, j) * cos(Phi[i])) * dRdth.at<double>(i, j),
                    1 / res.at<double>(i, j) * dRdph.at<double>(i, j);
            normal = -R * Partial;
            double norm = sqrt(normal.transpose() * normal);

            normals->points[i * w + j].normal_x = normal[0] / norm;
            normals->points[i * w + j].normal_y = normal[1] / norm;
            normals->points[i * w + j].normal_z = normal[2] / norm;

        }
    }

    std::cout<< "[time] " << tt.toc() << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer11(new pcl::visualization::PCLVisualizer("final Normals Viewer"));
    viewer11->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color11(cloud_order, 0, 238, 238);
    viewer11->addPointCloud<pcl::PointXYZ>(cloud_order, color11, "sample cloud for normal");
    viewer11->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                               "sample cloud for normal");
    viewer11->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_order, normals, 1, 0.3, "normals");
    viewer11->addCoordinateSystem(3.0);
    viewer11->initCameraParameters();

    while (!viewer11->wasStopped()) {
        viewer11->spinOnce(50);
    }




/*
    r2.initialize();
    r2.set_params(cloud_in,
                  range_mat);
    r2.precompute2();
    r2.to_rangeimg();
    r2.to_colormap();
    r2.calculate_normal2(true);
    r2.view_cloudwithnormal();   //changed coordinates
*/

}
