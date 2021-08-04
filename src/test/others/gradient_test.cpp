#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <eigen3/Eigen/StdVector>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const int BeamNum = 64;
const int HorizonNum = 1800;
const double AngRes_x = 360.0/double(HorizonNum);  // 0.192
const double AngRes_y = 26.8/double(BeamNum-1); //0.425
const double AngBottom = -24.8;

extern const double edgeThreshold = 2;
extern const double surfThreshold = 0.005;



//project point cloud into a range image
//gradient method on the range image  sobelx, sobely
//colormap: original range image & image after gradient operation


void to_rangeimg(const PointCloudTI::Ptr& cloud_input,
                 cv::Mat& range_mat) {
    int cloud_size = cloud_input->size();
    int col_index_[128000];  //std:array
    int row_index_[128000];



    for (int i = 0; i < cloud_size; i++) {
        double x = cloud_input->points[i].x;
        double y = cloud_input->points[i].y;
        double z = cloud_input->points[i].z;
        //row_index_存储的是原点云pcd中点所对应的投影row
        double xy_range = std::sqrt(x * x + y * y);
        double vertical_angle = std::atan2(z, xy_range) * 180 / M_PI;   //phi
        row_index_[i] = (int((vertical_angle - AngBottom) / AngRes_y));  // 地面index0
        if (row_index_[i] < 0 || row_index_[i] >= BeamNum) {
            row_index_[i] = -1;
            col_index_[i] = -1;
            continue;
        }

        double horizontal_angle = (double)std::atan2(x, y) * 180 / M_PI;  //theta
        col_index_[i] = -round((horizontal_angle - 90) / AngRes_x) + HorizonNum / 2;
        if (col_index_[i] >= HorizonNum)
            col_index_[i] -= HorizonNum;
        if (col_index_[i] < 0 || col_index_[i] >= HorizonNum) {   //?这里可以这样处理吗？
            col_index_[i] = -1;  //+= HorizonNum;
            continue;
        }

        //too close point is invalid
        double lidar_range = std::sqrt(x * x + y * y + z * z);

        if (lidar_range < 0.1) {
            continue;
        }


        cloud_input->points[i].intensity = (double) row_index_[i] + (double) col_index_[i] / 10000.0;

        double range_val = range_mat.at<double>(row_index_[i], col_index_[i]);
        if (range_val > 0 && range_val != 255 &&lidar_range!=255) {
            if(range_val >= lidar_range){
                continue;
            }
            else{
                range_mat.at<double>(row_index_[i], col_index_[i]) = lidar_range;
            }
        } else {
            range_mat.at<double>(row_index_[i], col_index_[i]) = lidar_range;
        }
    }
}


void to_colormap(cv::Mat& range_mat,
                 cv::Mat& color_img){

    double min, max;
    cv::minMaxIdx(range_mat, &min, &max);
    std::cout << "min lidar_range= " << min << std::endl;
    std::cout << "max lidar_range= " << max << std::endl;

    for(int i=0; i< BeamNum; ++i){
        for(int j=0; j<HorizonNum; ++j){
            color_img.at<unsigned char>(i, j) = (unsigned char)(range_mat.at<double>(i,j));
        }
    }
    applyColorMap(color_img, color_img, cv::COLORMAP_JET);
    imshow("colorMap", color_img);
    cv::waitKey(0);
}


void to_gradient(cv::Mat range_mat){
    cv::Mat kernel_theta = (cv::Mat_<char>(3,3)<< 1, 0, -1, 2, 0, -2, 1, 0, -1);
    cv::Mat kernel_phi = (cv::Mat_<char>(3,3)<< 1, 2, 1, 0, 0, 0, -1, -2, -1);
    cv::Mat color_img = cv::Mat(BeamNum, HorizonNum, CV_8U);
    cv::Mat color_img2 = cv::Mat(BeamNum, HorizonNum, CV_8U);
    cv::Mat theta_mat, phi_mat;
    theta_mat = cv::Mat(BeamNum, HorizonNum, CV_32F);
    phi_mat = cv::Mat(BeamNum, HorizonNum, CV_32F);
    cv::Point anchor(-1, -1);
    cv::filter2D(range_mat, theta_mat,-1,  kernel_theta, anchor, 0, cv::BORDER_DEFAULT);
    cv::filter2D(range_mat, phi_mat, -1, kernel_phi, anchor, 0, cv::BORDER_DEFAULT);

    for(int i=0; i< BeamNum; ++i){
        for(int j=0; j<HorizonNum; ++j){
            color_img.at<unsigned char>(i, j) = (unsigned char)(theta_mat.at<double>(i,j));
        }
    }
    applyColorMap(color_img, color_img, cv::COLORMAP_JET);
    imshow("colorMap", color_img);
    cv::waitKey(0);

    for(int i=0; i< BeamNum; ++i){
        for(int j=0; j<HorizonNum; ++j){
            color_img2.at<unsigned char>(i, j) = (unsigned char)(phi_mat.at<double>(i,j));
        }
    }
    applyColorMap(color_img2, color_img2, cv::COLORMAP_JET);
    imshow("colorMap", color_img2);
    cv::waitKey(0);





}



int main(){

    PointCloudTI::Ptr cloud_in(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/2.pcd",
                                  *cloud_in);

    cv::Mat range_mat = cv::Mat(64, 1800, CV_32F, cv::Scalar::all(255));
    cv::Mat color_img = cv::Mat(64, 1800, CV_8U);

    to_rangeimg(cloud_in, range_mat);
    for(int i=0; i< BeamNum; ++i){
        for(int j=0; j<HorizonNum; ++j){
            color_img.at<unsigned char>(i, j) = (unsigned char)(range_mat.at<double>(i,j));
        }
    }
    //applyColorMap(color_img, color_img, cv::COLORMAP_JET);
    imshow("colorMap", color_img);
    cv::waitKey(0);
    //to_colormap(range_mat, color_img);
    to_gradient(range_mat);
}

