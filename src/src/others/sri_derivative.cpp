
#include "sri_derivative.h"

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <cmath>
#include <deque>
#include <iostream>
#include <string>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

extern const int BeamNum = 16;
extern const int HorizonNum = 100 ;
extern const float AngRes_x = 360.0 / float(HorizonNum);  // 0.2
extern const float AngRes_y = 180 / float(BeamNum - 1); //1 deg
extern const float AngBottom = -32;   //m2




void SRI::set_params(const PointCloudTI::Ptr &cloud_in,
                     cv::Mat range_mat) {
    cloud_in->is_dense = false;
    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    std::cout << "cloud size before: " << cloud_in->size() << "\n";
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in_, *indices);
    std::cout << "size after NaNremoval: " << cloud_in_->points.size() << std::endl;
    for (auto &point : cloud_in_->points) {
        point.intensity = -1;
    }

    range_mat_ = range_mat;
}

void SRI::initialize() {

    cloud_in_.reset(new PointCloudTI);
    cloud_ordered_.reset(new PointCloudTI);
    cloud_ordered_->points.resize(BeamNum * HorizonNum);
    cloud_normal_.reset(new PointCloudT);
    cloud_normal_->points.resize(BeamNum * HorizonNum);
    valid_cloud_.reset(new PointCloudTI);

    PointTI nanPoint;
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;
    std::fill(cloud_ordered_->points.begin(), cloud_ordered_->points.end(), nanPoint);

    cos_phi_.resize(BeamNum * HorizonNum);
    std::fill(cos_phi_.begin(), cos_phi_.end(), 0.0001);

    rhat_map_.resize(BeamNum * HorizonNum);


}

void SRI::to_colormap() {
    cv::Mat color_img = cv::Mat(BeamNum, HorizonNum, CV_8U);
    double min, max;
    cv::minMaxIdx(range_mat_, &min, &max);
    std::cout << "min lidar_range= " << min << std::endl;
    std::cout << "max lidar_range= " << max << std::endl;

    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {
            color_img.at<unsigned char>(i, j) = (unsigned char) (range_mat_.at<double>(i, j));
            range_vec_.push_back(range_mat_.at<double>(i, j));
        }
    }
    applyColorMap(color_img, color_img, cv::COLORMAP_JET);
    imshow("colorMap", color_img);
    cv::waitKey(0);
}


void SRI::interpolation() {
    //cv::Mat range_mat_ 如果初始化为0，则使用闭运算;如果初始化为255,使用开运算
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat res;
    cv::morphologyEx(range_mat_, res, cv::MORPH_OPEN, kernel);   //1000初始化

    cv::Mat color_img = cv::Mat(BeamNum, HorizonNum, CV_8U);
    double min, max;
    cv::minMaxIdx(res, &min, &max);
    std::cout << "min lidar_range= " << min << std::endl;
    std::cout << "max lidar_range= " << max << std::endl;

    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {
            color_img.at<unsigned char>(i, j) = (unsigned char) (range_mat_.at<double>(i, j));
        }
    }
    applyColorMap(color_img, color_img, cv::COLORMAP_JET);
    imshow("colorMap", color_img);
    cv::waitKey(0);

    range_mat_ = res;

}


void SRI::display_cloud(const PointCloudTI::Ptr &cloud, const std::string &window_name) {
    if (!cloud->empty()) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
        pcl::visualization::PointCloudColorHandlerGenericField<PointTI> point_cloud_color_handler(cloud, "intensity");

        viewer->addPointCloud<PointTI>(cloud, point_cloud_color_handler, "id");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "id");
        viewer->setBackgroundColor(0, 0, 0);
        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
        viewer->close();
    } else {
        std::cout << window_name << " display failure. Cloud contains no points\n";
    }
}


void SRI::view_cloudwithnormal() {

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->resize(BeamNum * HorizonNum);
    for (int i = 0; i < BeamNum * HorizonNum; ++i) {

        //换回激光雷达坐标系
        cloud_normal_->points[i].x = cloud_ordered_->points[i].z;
        cloud_normal_->points[i].y = -cloud_ordered_->points[i].x;
        cloud_normal_->points[i].z = -cloud_ordered_->points[i].y;
        normals->points[i].normal_x = normal_vec_[i][2];
        normals->points[i].normal_y = -normal_vec_[i][0];
        normals->points[i].normal_z = -normal_vec_[i][1];
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Normals Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_normal_, 0, 238, 238);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_normal_, color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_normal_, normals, 1, 13, "normals");
    viewer->addCoordinateSystem(3.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }
}


void SRI::precompute() {
    float row2phi_map[65];   //数组必须要在这里changshu定义好，如果是数组指针就可以到initial里面定义了
    float col2theta_map[180];  //TODO 记得修改！


    cv::getDerivKernels(kx_dx_, ky_dx_, 1, 0, 3, true, CV_64F);
    cv::getDerivKernels(kx_dy_, ky_dy_, 0, 1, 3, true, CV_64F);
    kx_dx_ = kx_dx_ / AngRes_x * 150;
    ky_dy_ = ky_dy_ / AngRes_y * 150;

    for (int i = 0; i < BeamNum; ++i) {
        row2phi_map[i] = (AngBottom + i * AngRes_y) * M_PI / 180;    // deg to rad
    }

    for (int i = 0; i < HorizonNum; ++i) {
        col2theta_map[i] = (i * AngRes_x) * M_PI / 180;
    }

    // R_hat;
    for (int i = 0; i < BeamNum; ++i) {
        float phi = row2phi_map[i];
        for (int j = 0; j < HorizonNum; ++j) {
            float theta = col2theta_map[j];
            Eigen::Matrix3f R_hat;
            Eigen::Matrix3f R_d;
            Eigen::Matrix3f R_theta;
            Eigen::Matrix3f R_phi;
            R_d << 0, 1, 0,
                    0, 0, 1,
                    1, 0, 0;
            R_theta << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;
            R_phi << cos(phi), 0, -sin(phi),
                    0, 1, 0,
                    sin(phi), 0, cos(phi);
            R_hat = R_d * R_theta * R_phi;

            for (int k = 0; i < 3; i++) {
                R_hat(k, 1) /= cos(phi);
            }
            R_hat(0, 0) -= 2 * cos(phi) * sin(theta);
            R_hat(1, 0) -= 2 * sin(phi);
            R_hat(2, 0) -= 2 * cos(phi) * cos(theta);

            rhat_map_.push_back(R_hat);
        }
    }
}


void SRI::precompute2() {
    float row2phi_map[65];   //数组必须要在这里changshu定义好，如果是数组指针就可以到initial里面定义了
    float col2theta_map[180];

    for (int i = 0; i < BeamNum; ++i) {
        row2phi_map[i] = (AngBottom + i * AngRes_y) * M_PI / 180;    // deg to rad
    }

    for (int j = 0; j < HorizonNum; ++j) {
        col2theta_map[j] = (j * AngRes_x - 180) * M_PI / 180;
    }

    // R_hat;
    for (int i = 0; i < BeamNum; ++i) {
        float phi = row2phi_map[i];
        for (int j = 0; j < HorizonNum; ++j) {
            float theta = col2theta_map[j];
            Eigen::Matrix3f R_hat;
            Eigen::Matrix3f R_d;
            Eigen::Matrix3f R_theta;
            Eigen::Matrix3f R_phi;
            //R_d << 0, 1, 0,
            //        0, 0, 1,
            //        1, 0, 0;
            R_theta << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;
            R_phi << cos(phi), 0, -sin(phi),
                    0, 1, 0,
                    sin(phi), 0, cos(phi);
            //R_hat = R_d * R_theta * R_phi;
            R_hat = R_theta * R_phi;

            //for(int k=0; i<3; i++){
            //    R_hat(k,1)  /= cos(phi);
            //}
            //R_hat(0,0) -= 2*cos(phi) * sin(theta);
            //R_hat(1,0) -= 2*sin(phi);
            //R_hat(2,0) -= 2*cos(phi) * cos(theta);

            rhat_map_[j + i * HorizonNum] = R_hat;
        }
    }
}

void SRI::calculate_normal(bool interpolation_flag) {

    if (interpolation_flag) {
        interpolation();
    }


    cv::Mat dr_dtheta, dr_dphi;
    cv::sepFilter2D(range_mat_, dr_dtheta, range_mat_.depth(), kx_dx_, ky_dx_);
    cv::sepFilter2D(range_mat_, dr_dphi, range_mat_.depth(), kx_dy_, ky_dy_);


    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {

            float r = range_mat_.at<float>(i, j);
            //对rangeimg的这一片区域 3×3初定，进行prewitt滤波

            //矩阵的索引是行，列
            //float dr_dtheta = theta_mat.at<float>(i,j);
            //float dr_dphi = phi_mat.at<float>(i,j);
            Eigen::Vector3f partial_d;
            partial_d << 1,
                    dr_dtheta.at<float>(i, j) / r,
                    dr_dphi.at<float>(i, j) / r;

            Eigen::Vector3f tmp;
            tmp = rhat_map_[i * HorizonNum + j] * partial_d;
            normal_vec_.push_back(tmp);
        }
    }

    for (int i = 0; i < normal_vec_.size(); ++i) {
        float x0 = normal_vec_[i][0];
        float x1 = normal_vec_[i][1];
        float x2 = normal_vec_[i][2];
        float n = sqrt(x0 * x0 + x1 * x1 + x2 * x2);

        //if (normal_vec_[i][2] > 0) {
        //    normal_vec_[i][0] = - x2 / n;
        //    normal_vec_[i][1] = x0 / n;
        //    normal_vec_[i][2] = x1 / n;
        //} else {
        //    normal_vec_[i][0] = x2 / n;
        //    normal_vec_[i][1] = -x0 / n;
        //    normal_vec_[i][2] = -x1 / n;


        //if(normal_vec_[i][2] > 0) {
        //    normal_vec_[i][0] = - x0 / n;
        //    normal_vec_[i][1] = - x1 / n;
        //    normal_vec_[i][2] = - x2 / n;
        //}
        //else{
        normal_vec_[i][0] = x0 / n;
        normal_vec_[i][1] = x1 / n;
        normal_vec_[i][2] = x2 / n;
        //}

    }

}


void SRI::calculate_normal2(bool interpolation_flag) {

    if (interpolation_flag) {
        interpolation();
    }

    float dNaN = std::numeric_limits<float>::quiet_NaN();
    cv::Mat dr_dtheta = cv::Mat(range_mat_.rows, range_mat_.cols, CV_64F, cv::Scalar(dNaN));
    cv::Mat dr_dphi = cv::Mat(range_mat_.rows, range_mat_.cols, CV_64F, cv::Scalar(dNaN));

    for (int i = 0; i < BeamNum; i++) {
        for (int j = 0; j < HorizonNum - 2; j++) {
            dr_dtheta.at<float>(i, j + 1) =
                    (range_mat_.at<float>(i, j + 2) - range_mat_.at<float>(i, j)) / (2 * AngRes_x);
        }
    }

    for (int i = 0; i < BeamNum - 2; i++) {
        for (int j = 0; j < HorizonNum; j++) {
            dr_dphi.at<float>(i + 1, j) =
                    (range_mat_.at<float>(i + 2, j) - range_mat_.at<float>(i, j)) / (2 * AngRes_y);
        }
    }

    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {

            float r = range_mat_.at<float>(i, j);
            //对rangeimg的这一片区域 3×3初定，进行prewitt滤波

            //矩阵的索引是行，列
            Eigen::Vector3f partial_d;
            partial_d << 1,
                        dr_dtheta.at<float>(i, j) / (r * cos_phi_[i * HorizonNum + j]) ,
                        dr_dphi.at<float>(i, j) / r;

            Eigen::Vector3f tmp;
            tmp = rhat_map_[i * HorizonNum + j] * partial_d;
            normal_vec_.push_back(tmp);
        }
    }

    for (int i = 0; i < normal_vec_.size(); ++i) {
        float x0 = normal_vec_[i][0];
        float x1 = normal_vec_[i][1];
        float x2 = normal_vec_[i][2];
        float n = sqrt(x0 * x0 + x1 * x1 + x2 * x2);

        //if (normal_vec_[i][2] > 0) {
        //    normal_vec_[i][0] = - x2 / n;
        //    normal_vec_[i][1] = x0 / n;
        //    normal_vec_[i][2] = x1 / n;
        //} else {
        //    normal_vec_[i][0] = x2 / n;
        //    normal_vec_[i][1] = -x0 / n;
        //    normal_vec_[i][2] = -x1 / n;


        //if(normal_vec_[i][2] > 0) {
        //    normal_vec_[i][0] = - x0 / n;
        //    normal_vec_[i][1] = - x1 / n;
        //    normal_vec_[i][2] = - x2 / n;
        //}
        //else{
        normal_vec_[i][0] = x0 / n;
        normal_vec_[i][1] = x1 / n;
        normal_vec_[i][2] = x2 / n;
        //}

    }

}


void SRI::to_rangeimg() {
    int cloud_size = cloud_in_->size();
    int index;

    spherical_coord tmp;

    for (int i = 0; i < cloud_size; i++) {
        float x = -cloud_in_->points[i].y;
        float y = -cloud_in_->points[i].z;
        float z = cloud_in_->points[i].x;

        //too close point is invalid
        float lidar_range = std::sqrt(x * x + y * y + z * z);
        if (lidar_range < 0.1) {
            tmp.dist = -1;
            continue;
        }


        //row_index_存储的是原点云pcd中点所对应的投影row
        float vertical_angle = (float) (std::asin(y / lidar_range) * 180 / M_PI);   //phi deg
        row_index_[i] = int((vertical_angle - AngBottom) / AngRes_y + 0.5);  //四舍五入

        if (row_index_[i] < 0 || row_index_[i] >= BeamNum) {
            spherical_vec_.push_back(tmp);
            continue;
        }


        float horizontal_angle = (float) (std::atan2(x, z) * 180 / M_PI);  //theta (-180,180]
        col_index_[i] = int((horizontal_angle + 180) / AngRes_x + 0.5);
        if (col_index_[i] >= HorizonNum)
            col_index_[i] -= HorizonNum;
        if (col_index_[i] < 0 || col_index_[i] >= HorizonNum) {   //?这里可以这样处理吗？
            spherical_vec_.push_back(tmp);
            continue;
        }


        tmp.dist = lidar_range;
        tmp.phi = vertical_angle;
        tmp.theta = horizontal_angle;
        tmp.rangemat_rnc = (float) row_index_[i] + (float) col_index_[i] / 10000.0;

        float range_val = range_mat_.at<double>(row_index_[i], col_index_[i]);
        if (range_val > 0 && range_val != 2000 && lidar_range != 2000) {

            if (range_val >= lidar_range) {
                continue;
            } else {
                range_mat_.at<double>(row_index_[i], col_index_[i]) = lidar_range;
            }
        } else {
            range_mat_.at<double>(row_index_[i], col_index_[i]) = lidar_range;
        }


        index = col_index_[i] + row_index_[i] * HorizonNum;
        cos_phi_[index] = cos(vertical_angle * M_PI / 180);  //use rad to calculate cos value
        cloud_ordered_->points[index].intensity = range_mat_.at<float>(row_index_[i], col_index_[i]);
        cloud_ordered_->points[index].x = x;
        cloud_ordered_->points[index].y = y;
        cloud_ordered_->points[index].z = z;
        //opencv coordinate for cloud_ordered
        std::cout << "size of cloud_ordered: " << cloud_ordered_->points.size() << "\n";


    }
}

