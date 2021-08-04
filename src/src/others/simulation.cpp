
#include "simulation.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <vector>
#include <cmath>
#include <deque>
#include <iostream>
#include <ctime>
#include <iterator>
#include <string>
#include <limits>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common_headers.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/io.hpp>
#include <eigen3/Eigen/StdVector>
#include <pcl/features/normal_3d.h>
#include <opencv2/imgproc.hpp>

extern const int BeamNum = 65;
extern const int HorizonNum = 360;
extern const float AngRes_x = 360.0/float(HorizonNum);  // 1.0
//extern const float AngRes_y = 26.8/float(BeamNum-1); //0.425
//extern const float AngBottom = -24.8;
extern const float AngRes_y = 64/float(BeamNum-1); //1 deg
//extern const float AngBottom = -32.0;  //m1
//extern const float AngBottom = 122.0;   //m2
extern const float AngBottom = -32.0;   //m2




void Simulation::set_params(const PointCloudTI::Ptr& cloud_in,
                          cv::Mat range_mat){
    cloud_in->is_dense = false;
    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    std::cout<< "cloud size before: "<< cloud_in->size() << "\n";
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in_, *indices);
    std::cout << "size after NaNremoval: " << cloud_in_->points.size () << std::endl;
    for (auto & point : cloud_in_->points){
        point.intensity = -1;
    }

    //cloud_ordered_ = cloud_ordered;
    //cloud_spherical_ = cloud_spherical;
    range_mat_ = range_mat;
}

void Simulation::initialize(){

    cloud_in_.reset(new PointCloudTI);
    cloud_ordered_.reset(new PointCloudTI);
    cloud_ordered_->points.resize(BeamNum * HorizonNum);
    cloud_normal_.reset(new PointCloudT);
    cloud_normal_->points.resize(BeamNum * HorizonNum);
    valid_cloud_.reset(new PointCloudTI);
    //row_index_.resize(128000);
    PointTI nanPoint;
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;
    std::fill(cloud_ordered_->points.begin(), cloud_ordered_->points.end(), nanPoint);

    cos_phi_.resize(BeamNum*HorizonNum);
    std::fill(cos_phi_.begin(), cos_phi_.end(), 0.0001);

    //range_mat_ = cv::Mat(BeamNum, HorizonNum, CV_32F, cv::Scalar::all(FLT_MAX));
    spherical_vec_.resize(128000);
    range_vec_.assign(BeamNum * HorizonNum, 0);
    range_info_.resize(BeamNum*HorizonNum);
    //range_info_.assign(BeamNum * HorizonNum, 0);
    start_ind_.assign(BeamNum, 0);
    end_ind_.assign(BeamNum, 0);

    picked_label_ = new int[BeamNum * HorizonNum];
    point_pick_ = new int[BeamNum * HorizonNum];



    downsamp_filter_.setLeafSize(0.2, 0.2, 0.2);

    //map
    //坐标系：
    //row-xy平面为0度
    //col-从y轴开始向x正方向 0～180, 向x负方向0～ -180


}


void Simulation::to_rangeimg() {
    int cloud_size = cloud_in_->size();
    int index;

    spherical_coord tmp;

    for (int i = 0; i < cloud_size; i++) {
        float x = cloud_in_->points[i].x;
        float y = cloud_in_->points[i].y;
        float z = cloud_in_->points[i].z;

        //too close point is invalid
        float lidar_range = std::sqrt(x * x + y * y + z * z);
        if (lidar_range < 0.1) {
            tmp.dist = -1;
            spherical_vec_.push_back(tmp);
            continue;
        }


        //row_index_存储的是原点云pcd中点所对应的投影row
        float xy_range = std::sqrt(x * x + y * y);
        float vertical_angle = (float)(std::atan2(z, xy_range) * 180 / M_PI);   //phi
        row_index_[i] =  (int((vertical_angle - AngBottom) / AngRes_y));
        if (row_index_[i] < 0 || row_index_[i] >= BeamNum) {
            spherical_vec_.push_back(tmp);
            continue;
        }

        float horizontal_angle = (float)(std::atan2(x, y) * 180 / M_PI);  //theta
        col_index_[i] = -round((horizontal_angle - 90) / AngRes_x) + HorizonNum / 2;
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

        float range_val = range_mat_.at<float>(row_index_[i], col_index_[i]);
        if (range_val > 0 && range_val != 2000 &&lidar_range!=2000) {
            if (range_val >= lidar_range) {
                continue;
            } else {
                range_mat_.at<double>(row_index_[i], col_index_[i]) = lidar_range;
            }
        } else {
            range_mat_.at<double>(row_index_[i], col_index_[i]) = lidar_range;
        }

        index = col_index_[i] + row_index_[i] * HorizonNum;
        cloud_ordered_->points[index].intensity = range_mat_.at<float>(row_index_[i], col_index_[i]);
        cloud_ordered_->points[index].x = cloud_in_->points[i].x;
        cloud_ordered_->points[index].y = cloud_in_->points[i].y;
        cloud_ordered_->points[index].z = cloud_in_->points[i].z;

        /*
        int order_size = 0;
        for(int i = 0; i< BeamNum; ++i){
            start_ind_[i] = order_size - 1 + 5;
            for( int j=0; j<HorizonNum; ++j){
                if(range_mat_.at<float>(i,j) == 1000){
                    continue;
                }
                //只包含有效的数据的点云指针
                valid_cloud_->points[order_size] = cloud_ordered_->points[j + i*HorizonNum];
                valid_col_.push_back(j);
                valid_range_.push_back(range_mat_.at<float>(i,j));
                ++order_size;
            }
            end_ind_[i] = order_size - 1 - 5;
        }
        */

        /*
        cloud_in_->points[i].intensity = (float) (row_index_ +  col_index_ / 10000.0);
        float range_val = range_mat_.at<float>(row_index_, col_index_);
        if (range_val > 0 && range_val != 255 && range_val != 255 ) {  ///
            range_mat_.at<float>(row_index_, col_index_) = std::max(range_val, lidar_range);
            range_info_[index].range = range_mat_.at<float>(row_index_, col_index_);
            range_info_[index].index = i;
            spherical_vec_.push_back(tmp);
        } else if (range_val == 255){
            range_mat_.at<float>(row_index_, col_index_) = lidar_range;
            range_info_[index].range = range_mat_.at<float>(row_index_, col_index_);
            range_info_[index].index = i;
            spherical_vec_.push_back(tmp);
        }
         */
    }

    std::cout<<"size of cloud_ordered: " << cloud_ordered_->points.size() << "\n";

    //Test
    std::cout << "------------check------------" << std::endl;
    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < 30; ++j) {
            std::cout << range_mat_.at<float>(i, j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "------------end------------" << std::endl;

}


void Simulation::to_colormap(){
    cv::Mat color_img = cv::Mat(BeamNum, HorizonNum, CV_8U);
    double min, max;
    cv::minMaxIdx(range_mat_, &min, &max);
    std::cout << "min lidar_range= " << min << std::endl;
    std::cout << "max lidar_range= " << max << std::endl;
    //float k = 255/ (max-min) ;
    //range_mat_.convertTo(color_img, CV_8U, 255);

    for(int i=0; i< BeamNum; ++i){
        for(int j=0; j<HorizonNum; ++j){
            color_img.at<unsigned char>(i, j) = (unsigned char)(range_mat_.at<double>(i,j));
            range_vec_.push_back(range_mat_.at<double>(i,j));
        }
    }
    applyColorMap(color_img, color_img, cv::COLORMAP_JET);
    imshow("colorMap", color_img);
    cv::waitKey(0);
}


void Simulation::interpolation() {

    //cv::Mat range_mat_ 如果初始化为0，则使用闭运算;如果初始化为255,使用开运算
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::Mat res;
    cv::morphologyEx(range_mat_, res, cv::MORPH_OPEN, kernel);   //1000初始化

    cv::Mat color_img = cv::Mat(BeamNum, HorizonNum, CV_8U);
    double min, max;
    cv::minMaxIdx(res, &min, &max);
    std::cout << "min lidar_range= " << min << std::endl;
    std::cout << "max lidar_range= " << max << std::endl;
    //float k = 255/ (max-min) ;
    for(int i=0; i< BeamNum; ++i){
        for(int j=0; j<HorizonNum; ++j){
            color_img.at<unsigned char>(i, j) = (unsigned char)(range_mat_.at<double>(i,j));
            //range_vec_.push_back(range_mat_.at<float>(i,j));
        }
    }
    applyColorMap(color_img, color_img, cv::COLORMAP_JET);
    imshow("colorMap", color_img);
    cv::waitKey(0);


    //Test
    std::cout << "------------check------------" << std::endl;
    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < 30; ++j) {
            std::cout << res.at<double>(i, j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "------------end------------" << std::endl;

    range_mat_ = res;

}


void Simulation::precompute(){
    float row2phi_map[65];   //数组必须要在这里changshu定义好，如果是数组指针就可以到initial里面定义了
    float col2theta_map[360];

    for (int i =0; i< BeamNum; ++i){
        row2phi_map[i] = (AngBottom + AngRes_y * i)* M_PI / 180;
    }

    for (int i=0; i< HorizonNum/4; ++i){
        col2theta_map[i] = (-90 - AngRes_x * i) * M_PI / 180;
    }
    for (int i= HorizonNum/4; i< HorizonNum; ++i){
        col2theta_map[i] = (180 - AngRes_x * (i-90)) * M_PI / 180;
        // 90 = HorizonNum/4
    }
    // R_hat;
    for( int i = 0; i<BeamNum ; ++i){
        for (int j = 0; j< HorizonNum; ++j){
            float theta = col2theta_map[j];
            float phi = row2phi_map[i];
            Eigen::Matrix3f R_hat;
            R_hat << sin(theta)* cos(phi), cos(theta)/cos(phi), -sin(theta)*sin(phi),
                    cos(theta)*cos(phi),  -sin(theta)/cos(phi), -sin(theta)*cos(phi),
                    sin(phi),                    0,             cos(phi);

            rhat_map_.push_back(R_hat);
        }
    }
}

void Simulation::calculate_normal(bool interpolation_flag){

    if(interpolation_flag){
        interpolation();
    }
    //对rangeimg求导数
    //cv::Mat kernel_theta = (cv::Mat_<double>(3,3)<< -30, -100, -30, 0, 0, 0, 30, 100, 30);
    //cv::Mat kernel_phi = (cv::Mat_<double>(3,3)<< -30, 0, 30, -100, 0, 100, -30, 0, 30);
    /*cv::Mat kernel_theta = (cv::Mat_<double>(3,3)<< -1, 0, 1, -2, 0, 2, -1, 0, 1);
    cv::Mat kernel_phi = (cv::Mat_<double>(3,3)<< -1, -2, -1, 0, 0, 0, 1, 2, 1);

    cv::Mat theta_mat, phi_mat;
    theta_mat = cv::Mat(BeamNum, HorizonNum, CV_64F);
    phi_mat = cv::Mat(BeamNum, HorizonNum, CV_64F);
    cv::Point anchor(-1, -1);
    cv::filter2D(range_mat_, theta_mat,-1,  kernel_theta, anchor, 0, cv::BORDER_DEFAULT);
    cv::filter2D(range_mat_, phi_mat, -1, kernel_phi, anchor, 0, cv::BORDER_DEFAULT);
    */


    cv::getDerivKernels(kx_dx, ky_dx, 1, 0, 3, true, CV_64F);
    cv::getDerivKernels(kx_dy, ky_dy, 0, 1, 3, true, CV_32F);
    kx_dx /= AngRes_x;
    ky_dy /= AngRes_y;



    for( int i = 0; i<BeamNum ; ++i){
        for (int j = 0; j< HorizonNum; ++j){

            float r = range_mat_.at<float>(i,j);
            //对rangeimg的这一片区域 3×3初定，进行prewitt滤波

            //矩阵的索引是行，列
            //float dr_dtheta = theta_mat.at<float>(i,j);
            //float dr_dphi = phi_mat.at<float>(i,j);
            cv::sepFilter2D(range_mat_, dr_dtheta, range_mat_.depth(), kx_dx, ky_dx);
            cv::sepFilter2D(range_mat_, dr_dtheta, range_mat_.depth(), kx_dy, ky_dy);

            Eigen::Vector3f partial_d;
            partial_d << 1,
                         dr_dtheta / r ,
                         dr_dphi / r;

            Eigen::Vector3f tmp;
            //Eigen::Matrix3f check_rhat;
            //check_rhat = rhat_map_[i* HorizonNum + j];
            tmp = rhat_map_[i* HorizonNum + j] * partial_d;

            normal_vec_.push_back(tmp);
        }
    }

    for(int i=0; i<normal_vec_.size(); ++i){
        float x0 = normal_vec_[i][0];
        float x1 = normal_vec_[i][1];
        float x2 = normal_vec_[i][2];
        float n = sqrt(x0*x0 + x1*x1 + x2*x2);
        normal_vec_[i][0] /= n;
        normal_vec_[i][1] /= n;
        normal_vec_[i][2] /= n;
    }

    std::cout<< "--------normal_vector--------" << std::endl;
    for(int i =0; i<20; ++i){
        std::cout<<normal_vec_[i] << "\n";
        std::cout<< " ------------- " << std::endl;
    }


}



void Simulation::to_rangeimg2() {
    int cloud_size = cloud_in_->size();
    int index;

    spherical_coord tmp;

    for (int i = 0; i < cloud_size; i++) {
        float x = cloud_in_->points[i].x;
        float y = cloud_in_->points[i].y;
        float z = cloud_in_->points[i].z;

        //too close point is invalid
        float lidar_range = std::sqrt(x * x + y * y + z * z);
        if (lidar_range < 0.1) {
            tmp.dist = -1;
            spherical_vec_.push_back(tmp);
            continue;
        }


        //row_index_存储的是原点云pcd中点所对应的投影row
        //TODO : use acos or asin?
        float vertical_angle = (float) ((std::acos(z/lidar_range) * 180 / M_PI));   //phi
        row_index_[i] = (int((vertical_angle - AngBottom) / AngRes_y));
        if (row_index_[i] < 0 || row_index_[i] >= BeamNum) {
            spherical_vec_.push_back(tmp);
            continue;
        }

        float horizontal_angle = (float) (std::atan2(y, x) * 180 / M_PI + 180);  //theta (0,2pi]
        col_index_[i] = round((horizontal_angle - 180) / AngRes_x) + HorizonNum;
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

        float range_val = range_mat_.at<float>(row_index_[i], col_index_[i]);
        if (range_val > 0 && range_val != 2000 && lidar_range != 2000) {
            range_mat_.at<float>(row_index_[i], col_index_[i]) = std::max(range_val, lidar_range);
        } else {
            range_mat_.at<float>(row_index_[i], col_index_[i]) = lidar_range;
        }

        index = col_index_[i] + row_index_[i] * HorizonNum;
        cloud_ordered_->points[index].intensity = range_mat_.at<float>(row_index_[i], col_index_[i]);
        cloud_ordered_->points[index].x = cloud_in_->points[i].x;
        cloud_ordered_->points[index].y = cloud_in_->points[i].y;
        cloud_ordered_->points[index].z = cloud_in_->points[i].z;





    }
    std::cout << "size of cloud_ordered: " << cloud_ordered_->points.size() << "\n";
}

void Simulation::precompute2(){
    float row2phi_map[65];   //数组必须要在这里changshu定义好，如果是数组指针就可以到initial里面定义了
    float col2theta_map[360];

    for (int i =0; i< BeamNum; ++i){
        row2phi_map[i] = (AngRes_y*(i-(BeamNum - 1)) + AngBottom)* M_PI / 180;
    }

    for (int i=0; i< HorizonNum; ++i){
        col2theta_map[i] = (i - HorizonNum + 180) * M_PI / 180;
    }

    // R_hat;
    for( int i = 0; i<BeamNum ; ++i){
        for (int j = 0; j< HorizonNum; ++j){
            float theta = col2theta_map[j];
            float phi = row2phi_map[i];
            Eigen::Matrix3f R_hat;
            R_hat << cos(theta)* sin(phi), -sin(theta)/sin(phi), cos(theta)*cos(phi),
                    sin(theta)*sin(phi),  -cos(theta)/sin(phi),  sin(theta)*cos(phi),
                    cos(phi),                    0,              sin(phi);

            rhat_map_.push_back(R_hat);
        }
    }
}


void Simulation::calculate_normal2(bool interpolation_flag){

    if(interpolation_flag){
        interpolation();
    }
    //对rangeimg求导数
    //cv::Mat kernel_theta = (cv::Mat_<char>(3,3)<< -1, 0, 1, -2, 0, 2, -1, 0, 1);
    //cv::Mat kernel_phi = (cv::Mat_<char>(3,3)<< -1, -2, -1, 0, 0, 0, 1, 2, 1);
    cv::Mat kernel_theta = (cv::Mat_<char>(3,3)<< -300, 0, 300, -1000, 0, 1000, -300, 0, 300);
    cv::Mat kernel_phi = (cv::Mat_<char>(3,3)<< -300, -1000, -300, 0, 0, 0, 300, 1000, 300);
    cv::Mat theta_mat, phi_mat;
    theta_mat = cv::Mat(BeamNum, HorizonNum, CV_32F);
    phi_mat = cv::Mat(BeamNum, HorizonNum, CV_32F);
    cv::Point anchor(-1, -1);
    cv::filter2D(range_mat_, theta_mat,-1,  kernel_theta, anchor, 0, cv::BORDER_DEFAULT);
    cv::filter2D(range_mat_, phi_mat, -1, kernel_phi, anchor, 0, cv::BORDER_DEFAULT);



    for( int i = 0; i<BeamNum ; ++i){
        for (int j = 0; j< HorizonNum; ++j){

            float r = range_mat_.at<float>(i,j);
            //对rangeimg的这一片区域 3×3初定，进行prewitt滤波

            //矩阵的索引是行，列
            float dr_dtheta = theta_mat.at<float>(i,j);
            float dr_dphi = phi_mat.at<float>(i,j);


            Eigen::Vector3f partial_d;
            partial_d << 1,
                         1 / r * dr_dtheta,
                         1 / r * dr_dphi;

            Eigen::Vector3f tmp;
            Eigen::Matrix3f check_rhat;
            check_rhat = rhat_map_[i* HorizonNum + j];
            tmp = rhat_map_[i* HorizonNum + j] * partial_d;

            normal_vec_.push_back(tmp);
        }
    }

    for(int i=0; i<normal_vec_.size(); ++i){
        float x0 = normal_vec_[i][0];
        float x1 = normal_vec_[i][1];
        float x2 = normal_vec_[i][2];
        float n = sqrt(x0*x0 + x1*x1 + x2*x2);
        normal_vec_[i][0] /= n;
        normal_vec_[i][1] /= n;
        normal_vec_[i][2] /= n;
    }

    std::cout<< "--------normal_vector--------" << std::endl;
    for(int i =0; i<20; ++i){
        std::cout<<normal_vec_[i] << "\n";
        std::cout<< " ------------- " << std::endl;
    }
}



void Simulation::display_cloud(const PointCloudTI::Ptr& cloud, const std::string& window_name) {
    if (cloud->size() < 1) {
        std::cout << window_name << " display failure. Cloud contains no points\n";
        return;
    }
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
    pcl::visualization::PointCloudColorHandlerGenericField<PointTI> point_cloud_color_handler(cloud, "intensity");

    viewer->addPointCloud<PointTI>(cloud, point_cloud_color_handler, "id");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "id");
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    viewer->close();
}


void Simulation::view_cloudwithnormal(){

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->resize(BeamNum * HorizonNum);
    for(int i=0; i< BeamNum* HorizonNum; ++i){

        //cloud_normal_->points[i].x = cloud_ordered_->points[i].x;
        //cloud_normal_->points[i].y = cloud_ordered_->points[i].y;
        //cloud_normal_->points[i].z = cloud_ordered_->points[i].z;

        cloud_normal_->points[i].x = - cloud_ordered_->points[i].y;
        cloud_normal_->points[i].y = - cloud_ordered_->points[i].z;
        cloud_normal_->points[i].z = cloud_ordered_->points[i].x;
        normals->points[i].normal_x = normal_vec_[i][0];
        normals->points[i].normal_y = normal_vec_[i][1];
        normals->points[i].normal_z = normal_vec_[i][2];
    }

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Normals Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_normal_, 0, 238, 238);
    viewer->addPointCloud<pcl::PointXYZ> (cloud_normal_, color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_normal_,normals, 1, 16, "normals");
    viewer->addCoordinateSystem (3.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }
}



//---------------coordinate in paper: method 3----------

void Simulation::precompute3(){
    float row2phi_map[65];   //数组必须要在这里changshu定义好，如果是数组指针就可以到initial里面定义了
    float col2theta_map[360];

    for (int i =0; i< BeamNum; ++i){
        //row2phi_map[i] = (AngBottom + AngRes_y * i)* M_PI / 180;
        row2phi_map[i] = (i - 32.0) * M_PI / 180;    // deg to rad
    }

    for (int i=0; i< HorizonNum; ++i){
        col2theta_map[i] = (- i + 180) * M_PI / 180;
    }

    // R_hat;
    for( int i = 0; i<BeamNum ; ++i){
        for (int j = 0; j< HorizonNum; ++j){
            float theta = col2theta_map[j];
            float phi = row2phi_map[i];
            Eigen::Matrix3f R_hat;
            Eigen::Matrix3f R_d;
            Eigen::Matrix3f R_theta;
            Eigen::Matrix3f R_phi;
            R_d << 0, 1, 0,
                   0, 0, 1,
                   1, 0, 0;
            R_theta << cos(theta), -sin(theta), 0,
                       sin(theta), cos(theta),  0,
                       0,          0,           1;
            R_phi << cos(phi), 0, -sin(phi),
                     0,        1, 0,
                     sin(phi), 0, cos(phi);
            R_hat << R_d * R_theta * R_phi;

            rhat_map_.push_back(R_hat);
        }
    }
}

void Simulation::calculate_normal3(bool interpolation_flag) {

    if (interpolation_flag) {
        interpolation();
    }
    //对rangeimg求导数
    //cv::Mat kernel_theta = (cv::Mat_<char>(3, 3) << -1, 0, 1, -1, 0, 1, -1, 0, 1);
    //cv::Mat kernel_phi = (cv::Mat_<char>(3, 3) << -1, -1, -1, 0, 0, 0, 1, 1, 1);
    //cv::Mat kernel_theta = (cv::Mat_<char>(3,3)<< -300, -1000, -300, 0, 0, 0, 300, 1000, 300);
    //cv::Mat kernel_phi = (cv::Mat_<char>(3,3)<< -300, 0, 300, -1000, 0, 1000, -300, 0, 300);

    /*
    cv::Mat theta_mat, phi_mat;
    theta_mat = cv::Mat(BeamNum, HorizonNum, CV_32F);
    phi_mat = cv::Mat(BeamNum, HorizonNum, CV_32F);
    cv::Point anchor(-1, -1);
    cv::filter2D(range_mat_, theta_mat, -1, kernel_theta, anchor, 0, cv::BORDER_DEFAULT);
    cv::filter2D(range_mat_, phi_mat, -1, kernel_phi, anchor, 0, cv::BORDER_DEFAULT);
    */

    /*
    std::cout<< "--------theta_mat--------" << std::endl;
    for(int i =10; i<25; ++i){
        for(int j=10; j<15; ++j )
            std::cout<<theta_mat.at<float>(i,j) << " ";
        std::cout<<std::endl;
    }
    std::cout<< " ------------- " << std::endl;

    std::cout<< "--------phi_mat--------" << std::endl;
    for(int i =10; i<25; ++i){
        for(int j=10; j<15; ++j )
            std::cout<<phi_mat.at<float>(i,j) << " ";
        std::cout<< std::endl;
    }
    std::cout<< " ------------- " << std::endl;
     */
    int i0id = 0;
    int j0id = 0;

    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {

            if(i==0 || j==0 || i==BeamNum -1 || j == HorizonNum - 1){
                if(i==0){
                    ++i0id;
                }
                if(j==0){
                    ++j0id;
                }
                continue;
            }

            float r = range_mat_.at<double>(i, j);
            //对rangeimg的这一片区域 3×3初定，进行prewitt滤波

            //矩阵的索引是行，列
            //float dr_dtheta = theta_mat.at<float>(i, j);
            //float dr_dphi = phi_mat.at<float>(i, j);

            float dr_dtheta = (10*range_mat_.at<float>(i-1,j) + 3*range_mat_.at<float>(i-1,j-1) + 3*range_mat_.at<float>(i-1,j+1)-
                                    10*range_mat_.at<float>(i+1,j) - 3*range_mat_.at<float>(i+1,j-1) - 3*range_mat_.at<float>(i+1,j+1));
            float dr_dphi = (10*range_mat_.at<float>(i,j-1) + 3*range_mat_.at<float>(i-1,j-1) + 3*range_mat_.at<float>(i+1,j-1)-
                                  10*range_mat_.at<float>(i,j+1) - 3*range_mat_.at<float>(i-1,j+1) - 3*range_mat_.at<float>(i+1,j+1));





            Eigen::Vector3f partial_d;
            partial_d << 1,
                         dr_dtheta / (r * cos_phi_[i*HorizonNum + j]),
                         dr_dphi / r;

            Eigen::Vector3f tmp;
            Eigen::Matrix3f check_rhat;
            //check_rhat = rhat_map_[i * HorizonNum + j];
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
        normal_vec_[i][0] =  x0 / n;
        normal_vec_[i][1] =  x1 / n;
        normal_vec_[i][2] =  x2 / n;

        //}

    }

    std::cout << "--------normal_vector--------" << std::endl;
    for (int i = 0; i < 20; ++i) {
        std::cout << normal_vec_[i] << "\n";
        std::cout << " ------------- " << std::endl;
    }

    std::cout << "number of points with i=0: " << i0id <<std::endl;
    std::cout << "number of points with j=0: " << j0id <<std::endl;


}



void Simulation::to_rangeimg3() {
    int cloud_size = cloud_in_->size();
    int index;

    spherical_coord tmp;

    for (int i = 0; i < cloud_size; i++) {
        float x = - cloud_in_->points[i].y;
        float y = - cloud_in_->points[i].z;
        float z = cloud_in_->points[i].x;

        //too close point is invalid
        float lidar_range = std::sqrt(x * x + y * y + z * z);
        if (lidar_range < 0.1) {
            tmp.dist = -1;
            spherical_vec_.push_back(tmp);
            continue;
        }


        //row_index_存储的是原点云pcd中点所对应的投影row
        float xz_range = std::sqrt(x * x + z * z);
        float vertical_angle = (float) ( std::asin(y / lidar_range) * 180 / M_PI);   //phi deg
        row_index_[i] = int(vertical_angle/1 - (-32.0));   //AngB -32.0
        if (row_index_[i] < 0 || row_index_[i] >= BeamNum) {
            spherical_vec_.push_back(tmp);
            continue;
        }

        float horizontal_angle = (float) (std::atan2(x, z) * 180 / M_PI);  //theta (-pi,pi]
        col_index_[i] = round((- horizontal_angle/ 1 ) + HorizonNum/2 );  //AngRX
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

            if(range_val >= lidar_range){
                continue;
            }
            else{
                range_mat_.at<double>(row_index_[i], col_index_[i]) = lidar_range;
            }
        } else {
            range_mat_.at<double>(row_index_[i], col_index_[i]) = lidar_range;
        }



        index = col_index_[i] + row_index_[i] * HorizonNum;
        cos_phi_[index] = cos(vertical_angle  * M_PI / 180);
        cloud_ordered_->points[index].intensity = range_mat_.at<float>(row_index_[i], col_index_[i]);
        cloud_ordered_->points[index].x = cloud_in_->points[i].x;
        cloud_ordered_->points[index].y = cloud_in_->points[i].y;
        cloud_ordered_->points[index].z = cloud_in_->points[i].z;


        std::cout << "size of cloud_ordered: " << cloud_ordered_->points.size() << "\n";


    }
}

