#include "range_img.h"

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <deque>
#include <iostream>
#include <string>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common_headers.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/time.h>

extern const int BeamNum = 64;
extern const int HorizonNum = 1800;
extern const double AngRes_x = 360.0 / double(HorizonNum);  // 0.192
extern const double AngRes_y = 26.8 / double(BeamNum - 1); //0.425
extern const double AngBottom = -24.8;

extern const double edgeThreshold = 2;
extern const double surfThreshold = 2;


RangeImg::RangeImg() {}

RangeImg::RangeImg(cv::Mat &range_mat,
                   const bool &ground_flag) :
        range_mat_(range_mat), ground_flag_(ground_flag) {}


void RangeImg::run_extraction() {

    pcl::console::TicToc tt;
    tt.tic();
    remove_ground();
    std::cout << "[time for removing ground : " << tt.toc() << " ms ]" << std::endl;
    tt.tic();
    normal_kdtree(); //true:包括地面点， false：不包括地面点
    std::cout << "[time for calculating normal vector : " << tt.toc() << " ms ]" << std::endl;
    //view_normals();
    tt.tic();
    to_rangeimg();   //对已知非地面点的点进行投影 法向量也对应到相应的位置
    //std::cout << "[time for range Image : " << tt.toc() << " ms ]" << std::endl;
    //to_colormap();
    tt.tic();
    calculate_smoothness();
    mark_occlusion();
    //r1.extract_edgepts(2, 30);
    //r1.extract_surfpts(5);
    extract_egdeandsurfpts(20, 80);
    //std::cout << "[time from smoothness to feature points: " << tt.toc() << " ms ]" << std::endl;
}

void RangeImg::initialize(const PointCloudTI::Ptr &cloud_in) {
    cloud_in_.reset(new PointCloudTI);
    cloud_in_ = cloud_in;

    cloud_ground_.reset(new PointCloudTI);
    cloud_nonground_.reset(new PointCloudTI);
    cloud_ordered_.reset(new PointCloudTI);
    cloud_ordered_->points.resize(BeamNum * HorizonNum);
    cloud_normal_.reset(new PointCloudT);
    cloud_normal_->points.resize(128000);
    valid_cloud_.reset(new PointCloudTI);
    valid_cloud_->points.resize(BeamNum * HorizonNum);
    //row_index_.resize(128000);
    PointTI nanPoint;
    nanPoint.x = std::numeric_limits<double>::quiet_NaN();
    nanPoint.y = std::numeric_limits<double>::quiet_NaN();
    nanPoint.z = std::numeric_limits<double>::quiet_NaN();
    nanPoint.intensity = -1;
    std::fill(cloud_ordered_->points.begin(), cloud_ordered_->points.end(), nanPoint);

    normals_.reset(new pcl::PointCloud <pcl::Normal>);
    kdnormals_vec_.resize(BeamNum * HorizonNum);  //


    spherical_vec_.resize(128000);
    range_vec_.assign(BeamNum * HorizonNum, 0);
    start_ind_.assign(BeamNum, 0);
    end_ind_.assign(BeamNum, 0);
    cos_phi_.resize(BeamNum * HorizonNum);
    std::fill(cos_phi_.begin(), cos_phi_.end(), 0.0001);

    cloud_smoothness_.resize(128000);

    edgepts_sharp_.reset(new PointCloudTI());
    edgepts_less_.reset(new PointCloudTI());
    surfpts_flat_.reset(new PointCloudTI());
    surfpts_less_.reset(new PointCloudTI());
    surfpts_less_beam_.reset(new PointCloudTI());
    surfpts_lessDS_beam_.reset(new PointCloudTI());
    more_featurepts_.reset(new PointCloudTI());
    less_featurepts_.reset(new PointCloudTI());

    downsamp_filter_.setLeafSize(3, 3, 3);

    //map
    //坐标系：
    //row-xy平面为0度
    //col-从y轴开始向x正方向 0～180, 向x负方向0～ -180
    for (int i = 0; i < BeamNum; ++i) {
        row2phi_map_[i] = (-AngBottom + AngRes_y * i) * M_PI / 180;
    }

    for (int i = 0; i < HorizonNum / 4; ++i) {
        col2theta_map_[i] = (-90 - AngRes_x * i) * M_PI / 180;
    }
    for (int i = HorizonNum / 4; i < HorizonNum; ++i) {
        col2theta_map_[i] = (180 - AngRes_x * (i - HorizonNum / 4)) * M_PI / 180;
    }
}

void RangeImg::reset_params() {
    cloud_in_->clear();
    cloud_ordered_->clear();
    cloud_normal_->clear();
    valid_cloud_->clear();
    //range_mat_ = cv::Mat(BeamNum, HorizonNum, CV_32F, cv::Scalar::all(FLT_MAX));
}


PointCloudTI::Ptr RangeImg::get_egdesharp() {
    return edgepts_sharp_;
}

PointCloudTI::Ptr RangeImg::get_egdeless() {
    return edgepts_less_;
}

PointCloudTI::Ptr RangeImg::get_surfaceflat() {
    return surfpts_flat_;
}

PointCloudTI::Ptr RangeImg::get_surfaceless() {
    return surfpts_less_;
}

PointCloudTI::Ptr RangeImg::get_feature4target() {
    return more_featurepts_;

}

PointCloudTI::Ptr RangeImg::get_feature4src() {
    return less_featurepts_;
}

std::vector<Eigen::Vector3d> RangeImg::get_normalvector() {
    std::vector<Eigen::Vector3d> output;
    for(int i=0; i<vec_less_indice_.size(); ++i){
        output.push_back(valid_vec_[i]);
    }
    return output;
}


void RangeImg::remove_ground() {
    //对cloud_in_进行分离地面和非地面的操作
    //默认地面是一个激光点云中点个数最多的平面（RANSAC）
    cloud_in_->is_dense = false;
    boost::shared_ptr<std::vector < int>>
    indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_in_, *cloud_in_, *indices);
    for (auto &point : cloud_in_->points) {
        point.intensity = -1;
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation <pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.46);  /// Prameter

    seg.setInputCloud(cloud_in_);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }

    // estimated plane parameters in ax + by + cz + d = 0 form for coefficients->values[0-4].

    std::vector<int> ground_indices = inliers->indices;
    pcl::copyPointCloud(*cloud_in_, ground_indices, *cloud_ground_);

    //non-ground
    pcl::ExtractIndices <pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_in_);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_nonground_);
}


void RangeImg::normal_kdtree() {

    /*
    PointCloudT::Ptr cloud_in_xyz(new PointCloudT);
    double search_radius;
    if (ground_flag_) {
        cloud_in_xyz = xyzi2xyz(cloud_in_);
        search_radius = 0.5;
    } else {
        cloud_in_xyz = xyzi2xyz(cloud_nonground_);
        search_radius = 0.8;
    }

    pcl::NormalEstimation <PointT, pcl::Normal> nor;
    nor.setInputCloud(cloud_in_xyz);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree <PointT>);
    tree->setInputCloud(cloud_in_xyz);
    nor.setSearchMethod(tree);
    nor.setRadiusSearch(search_radius);
    nor.compute(*normals_);
    for (int i = 0; i < cloud_in_xyz->size(); ++i) {
        Eigen::Vector3d vec_pt;
        Eigen::Vector3d vec_nor;
        vec_pt << cloud_in_xyz->points[i].x, cloud_in_xyz->points[i].y, cloud_in_xyz->points[i].z;
        vec_nor << normals_->points[i].normal_x, normals_->points[i].normal_y, normals_->points[i].normal_z;
        if (vec_pt.transpose() * vec_nor > 0) {
            normals_->points[i].normal_x *= (-1);
            normals_->points[i].normal_y *= (-1);
            normals_->points[i].normal_z *= (-1);
        }
    }
     */

    //std::cout << "normals size of the point cloud: " << normals_->size() << std::endl;

    //normal not for working
    PointCloudT::Ptr cloud_in_xyz(new PointCloudT);
    if (ground_flag_) {
        cloud_in_xyz = xyzi2xyz(cloud_in_);
    } else {
        cloud_in_xyz = xyzi2xyz(cloud_nonground_);
    }
    normals_->resize(cloud_in_xyz->size());
    for (int i = 0; i < cloud_in_xyz->size(); ++i) {
        normals_->points[i].normal_x =0;
        normals_->points[i].normal_y =0;
        normals_->points[i].normal_z =0;
    }
}


void RangeImg::to_rangeimg() {
    PointCloudTI::Ptr cloud_arg(new PointCloudTI);
    if (ground_flag_) {
        cloud_arg = cloud_in_;
    } else {
        cloud_arg = cloud_nonground_;
    }

    int cloud_size = cloud_arg->size();
    int index;
    spherical_coord tmp;

    for (int i = 0; i < cloud_size; i++) {
        double x = cloud_arg->points[i].x;
        double y = cloud_arg->points[i].y;
        double z = cloud_arg->points[i].z;
        //row_index_存储的是原点云pcd中点所对应的投影row
        double xy_range = std::sqrt(x * x + y * y);
        double vertical_angle = std::atan2(z, xy_range) * 180 / M_PI;   //phi   unit:deg
        row_index_[i] = (int((vertical_angle - AngBottom) / AngRes_y));  // 地面index0
        //row_index_[i] = (int((vertical_angle - AngBottom) / AngRes_y));  // 地面index0
        if (row_index_[i] < 0 || row_index_[i] >= BeamNum) {
            row_index_[i] = -1;
            col_index_[i] = -1;
            spherical_vec_.push_back(tmp);
            continue;
        }

        double horizontal_angle = (double) std::atan2(x, y) * 180 / M_PI;  //theta
        col_index_[i] = -round((horizontal_angle - 90) / AngRes_x) + HorizonNum / 2;
        if (col_index_[i] >= HorizonNum)
            col_index_[i] -= HorizonNum;
        if (col_index_[i] < 0 || col_index_[i] >= HorizonNum) {   //?这里可以这样处理吗？
            col_index_[i] = -1;  //+= HorizonNum;
            spherical_vec_.push_back(tmp);
            continue;
        }

        //too close point is invalid
        double lidar_range = std::sqrt(x * x + y * y + z * z);

        if (lidar_range < 0.1) {
            spherical_vec_.push_back(tmp);
            continue;
        }

        tmp.dist = lidar_range;
        tmp.phi = vertical_angle;
        tmp.theta = horizontal_angle;
        spherical_vec_.push_back(tmp);

        cloud_arg->points[i].intensity = (double) row_index_[i] + (double) col_index_[i] / 10000.0;

        double range_val = range_mat_.at<double>(row_index_[i], col_index_[i]);
        if (range_val > 0 && range_val != 255 && lidar_range != 255) {
            if (range_val >= lidar_range) {
                continue;
            } else {
                range_mat_.at<double>(row_index_[i], col_index_[i]) = lidar_range;
            }
        } else {
            range_mat_.at<double>(row_index_[i], col_index_[i]) = lidar_range;
        }

        //落在同一个格子里的点只取一个,除了有值的点，其他都是nan点
        index = col_index_[i] + row_index_[i] * HorizonNum;
        cos_phi_[index] = cos(vertical_angle / 180 * M_PI);   //unit should be rad, rather than deg
        cloud_ordered_->points[index].intensity = range_mat_.at<double>(row_index_[i], col_index_[i]);
        cloud_ordered_->points[index].x = cloud_arg->points[i].x;
        cloud_ordered_->points[index].y = cloud_arg->points[i].y;
        cloud_ordered_->points[index].z = cloud_arg->points[i].z;

        //将点投入rangeimg后，法向量也根据对应位置存储, 无效点对应的就是向量0
        kdnormals_vec_[index][0] = normals_->points[i].normal_x;
        kdnormals_vec_[index][1] = normals_->points[i].normal_y;
        kdnormals_vec_[index][2] = normals_->points[i].normal_z;

    }


    int order_size = 0;
    for (int i = 0; i < BeamNum; ++i) {
        start_ind_[i] = order_size - 1 + 5;
        for (int j = 0; j < HorizonNum; ++j) {
            if (range_mat_.at<double>(i, j) == 255) {
                continue;
            }
            //只包含有效的数据的点云指针
            valid_cloud_->points[order_size] = cloud_ordered_->points[j + i * HorizonNum];
            valid_col_.push_back(j);
            valid_range_.push_back(range_mat_.at<double>(i, j));
            valid_vec_.push_back(kdnormals_vec_[j + i * HorizonNum]);
            ++order_size;
        }
        end_ind_[i] = order_size - 1 - 5;
    }

    /*
    for(int i=0; i<10; ++i){
        std::cout<<"valid_cloud: "<<valid_cloud_->points[i] << " " <<std::endl;
    }

        //Test
        std::cout << "------------check------------" << std::endl;
        for (int i = 0; i < BeamNum; ++i) {
            for (int j = 0; j < 30; ++j) {
                std::cout << range_mat_.at<double>(i, j) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "------------end------------" << std::endl;
    */

}


void RangeImg::calculate_smoothness() {
    int beam = -1;
    auto *cloud_curvature = new double[BeamNum * HorizonNum];
    int cloud_size = valid_range_.size();
    for (int i = 5; i < cloud_size - 5; i++) {
        double diff = valid_range_[i - 5] + valid_range_[i - 4] + valid_range_[i - 3] + valid_range_[i - 2]
                      + valid_range_[i - 1] - 10 * valid_range_[i] + valid_range_[i + 1] + valid_range_[i + 2]
                      + valid_range_[i + 3] + valid_range_[i + 4] + valid_range_[i + 5];

        cloud_curvature[i] = diff * diff;

        picked_label_[i] = 0;   //是否经过筛选
        point_pick_[i] = 0;     //是否被选取为特征点
        cloud_smoothness_[i].value = cloud_curvature[i];
        cloud_smoothness_[i].ind = i;

    }

}

//阻塞点：点云之间相互遮挡，又靠的很近的点 （？）
void RangeImg::mark_occlusion() {
    int cloud_size = valid_cloud_->points.size();
    for (int i = 5; i < cloud_size - 6; ++i) {

        double depth1 = valid_range_[i];
        double depth2 = valid_range_[i + 1];
        int col_diff = std::abs(int(valid_col_[i + 1] - valid_col_[i]));

        if (col_diff < 10) {
            // 选择距离较远的那些点，并将他们标记为1,表示已经经过筛选，不会作为特征点
            if (depth1 - depth2 > 0.3) {
                picked_label_[i - 5] = 1;
                picked_label_[i - 4] = 1;
                picked_label_[i - 3] = 1;
                picked_label_[i - 2] = 1;
                picked_label_[i - 1] = 1;
                picked_label_[i] = 1;
            } else if (depth2 - depth1 > 0.3) {
                picked_label_[i + 1] = 1;
                picked_label_[i + 2] = 1;
                picked_label_[i + 3] = 1;
                picked_label_[i + 4] = 1;
                picked_label_[i + 5] = 1;
                picked_label_[i + 6] = 1;
            }
        }

        double diff1 = std::abs(double(valid_range_[i - 1] - valid_range_[i]));
        double diff2 = std::abs(double(valid_range_[i + 1] - valid_range_[i]));

        // 选择距离变化较大的点，并将他们标记为1
        if (diff1 > 0.02 * valid_range_[i] && diff2 > 0.02 * valid_range_[i]) {
            picked_label_[i] = 1;
        }
    }
}


//point_label的含义为：
// -1  flat point;        0 less flat point;
// 1   less edge point;   2 sharp edge point;
//picked_label表示是否已经被筛选过
void RangeImg::extract_egdeandsurfpts(int edge_num, int surf_num) {
    edgepts_sharp_->clear();
    edgepts_less_->clear();
    surfpts_flat_->clear();
    surfpts_less_->clear();

    for (int i = 0; i < BeamNum; i++) {
        //将range img按照行平均分为6份
        for (int j = 0; j < 6; j++) {

            int sp = (start_ind_[i] * (6 - j) + end_ind_[i] * j) / 6;
            int ep = (start_ind_[i] * (5 - j) + end_ind_[i] * (j + 1)) / 6 - 1;
            if (sp >= ep)
                continue;
            //sort rule: small -> large
            std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep, valueSort);

            int largest_picknum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloud_smoothness_[k].ind;
                //default: 边点2个， 次边点20 （有两个没有打标签）
                if (picked_label_[ind] == 0 && cloud_smoothness_[ind].value > edgeThreshold) {
                    largest_picknum++;

                    if (largest_picknum <= edge_num) {
                        point_pick_[ind] = 2;
                        //edgepts_sharp_->push_back(valid_cloud_->points[ind]);
                        less_featurepts_->push_back(valid_cloud_->points[ind]);
                        vec_less_indice_.push_back(ind);
                        more_featurepts_->push_back(valid_cloud_->points[ind]);
                        vec_more_indice_.push_back(ind);
                    } else {
                        point_pick_[ind] = 1;
                        break;
                        //edgepts_less_->push_back(valid_cloud_->points[ind]);
                    }

                    picked_label_[ind] = 1;

                    //考虑到特征点分布的均匀，特征点左右5个若有距离大于阈值，那么还有成为特征点的可能
                    //如果和已经找到的特征点index差别小），那么就标记为已经筛查过，不会再作为备选特征点了
                    for (int l = 1; l <= 5; l++) {
                        int col_diff = std::abs(int(valid_col_[ind + l] - valid_col_[ind + l - 1]));
                        if (col_diff > 10)
                            break;
                        picked_label_[ind + l] = 1;
                    }

                    for (int l = -1; l >= -5; l--) {
                        int col_diff = std::abs(int(valid_col_[ind + l] - valid_col_[ind + l + 1]));
                        if (col_diff > 10)
                            break;
                        picked_label_[ind + l] = 1;
                    }
                }
            }
        }
    }


    //将range img按照每一条Beam平均分为6份
    for (int i = 0; i < BeamNum; i++) {
        surfpts_less_beam_->clear();
        for (int j = 0; j < 6; j++) {
            int sp = (start_ind_[i] * (6 - j) + end_ind_[i] * j) / 6;
            int ep = (start_ind_[i] * (5 - j) + end_ind_[i] * (j + 1)) / 6 - 1;
            if (sp >= ep)
                continue;
            std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep, valueSort);

            //平面点
            int smallest_picknum = 0;
            for (int k = sp; k < ep; k++) {
                int ind = cloud_smoothness_[k].ind;
                if (picked_label_[ind] == 0 && cloud_smoothness_[ind].value < surfThreshold) {
                    if (smallest_picknum >= surf_num) {
                        break;
                    }
                    point_pick_[ind] = -1;
                    //surfpts_flat_->push_back(valid_cloud_->points[ind]);
                    less_featurepts_->push_back(valid_cloud_->points[ind]);
                    vec_less_indice_.push_back(ind);
                    more_featurepts_->push_back(valid_cloud_->points[ind]);
                    vec_more_indice_.push_back(ind);
                    smallest_picknum++;
                    //flat平面点4个 其他都当作次平面点

                    picked_label_[ind] = 1;

                    for (int l = 1; l <= 12; l++) {
                        int col_diff = std::abs(int(valid_col_[ind + l] - valid_col_[ind + l - 1]));
                        if (col_diff > 10)
                            break;
                        picked_label_[ind + l] = 1;
                    }

                    for (int l = -1; l >= -12; l--) {
                        int col_diff = std::abs(int(valid_col_[ind + l] - valid_col_[ind + l + 1]));
                        if (col_diff > 10 )
                            break;
                        picked_label_[ind + l] = 1;
                    }
                }
            }

            //如果可以先把地面提取出来，那么就可以只在地面查找平面点
            //这里先不用划分地面的情况
            //选出次平面点
            for (int k = sp; k <= ep; k++) {
                if (point_pick_[k] <= 0) {
                    int ind = cloud_smoothness_[k].ind;
                    surfpts_less_beam_->push_back(valid_cloud_->points[k]);
                    //more_featurepts_->push_back(valid_cloud_->points[ind]);
                    //vec_more_indice_.push_back(ind);
                }
            }
        }

        //对平面点进行降采样
        downsamp_filter_.setInputCloud(surfpts_less_beam_);
        downsamp_filter_.filter(*surfpts_lessDS_beam_);
        *surfpts_less_ += (*surfpts_lessDS_beam_);
    }


}


void RangeImg::to_colormap() {
    cv::Mat color_img = cv::Mat(BeamNum, HorizonNum, CV_8U);
    double min, max;
    cv::minMaxIdx(range_mat_, &min, &max);
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
}

void RangeImg::interpolation() {

    //cv::Mat range_mat_ 如果初始化为0，则使用闭运算;如果初始化为255,使用开运算
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat res;
    //cv::morphologyEx(range_mat_, res, cv::MORPH_CLOSE, kernel);  //0初始化
    cv::morphologyEx(range_mat_, res, cv::MORPH_OPEN, kernel);   //255初始化

    cv::imshow("after interpolation", res);
    cv::waitKey(0);

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


    //Test
    std::cout << "------------check------------" << std::endl;
    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < 30; ++j) {
            std::cout << res.at<double>(i, j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "------------end------------" << std::endl;

}

void RangeImg::precompute() {
    double row2phi_map[BeamNum];   //数组必须要在这里changshu定义好，如果是数组指针就可以到initial里面定义了
    double col2theta_map[HorizonNum];

    for (int i = 0; i < BeamNum; ++i) {
        row2phi_map[i] = (AngBottom + AngRes_y * i) * M_PI / 180;
    }

    for (int i = 0; i < HorizonNum / 4; ++i) {
        col2theta_map[i] = (-90 - AngRes_x * i) * M_PI / 180;
    }
    for (int i = HorizonNum / 4; i < HorizonNum; ++i) {
        col2theta_map[i] = (180 - AngRes_x * (i - 450)) * M_PI / 180;
        // 450 = HorizonNum/4
    }
    // R_hat;
    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {
            double theta = col2theta_map[j];
            double phi = row2phi_map[i];
            Eigen::Matrix3f R_hat;
            R_hat << sin(theta) * cos(phi), cos(theta), -sin(theta) * sin(phi),
                    cos(theta) * cos(phi), -sin(theta), -sin(theta) * cos(phi),
                    sin(phi), 0, cos(phi);

            rhat_map_.push_back(R_hat);
        }
    }
}


void RangeImg::calculate_normal(bool interpolation_flag) {

    if (interpolation_flag) {
        interpolation();
    }
    //对rangeimg求导数
    cv::Mat kernel_theta = (cv::Mat_<double>(3, 3) << 1, 0, -1, 2, 0, -2, 1, 0, -1);
    cv::Mat kernel_phi = (cv::Mat_<double>(3, 3) << 1, 2, 1, 0, 0, 0, -1, -2, -1);     ///TODO
    cv::Mat theta_mat, phi_mat;
    theta_mat = cv::Mat(BeamNum, HorizonNum, CV_32F);
    phi_mat = cv::Mat(BeamNum, HorizonNum, CV_32F);
    cv::Point anchor(-1, -1);
    cv::filter2D(range_mat_, theta_mat, -1, kernel_theta, anchor, 0, cv::BORDER_DEFAULT);
    cv::filter2D(range_mat_, phi_mat, -1, kernel_phi, anchor, 0, cv::BORDER_DEFAULT);


    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {
            double r = range_mat_.at<double>(i, j);
            //对rangeimg的这一片区域 3×3初定，进行prewitt滤波

            //矩阵的索引是行，列
            double dr_dtheta = theta_mat.at<double>(i, j);
            double dr_dphi = phi_mat.at<double>(i, j);

            Eigen::Vector3f partial_d;
            partial_d << 1,
                    dr_dtheta / (r * cos_phi_[i * HorizonNum + j]),
                    dr_dphi / r;

            Eigen::Vector3f tmp;
            Eigen::Matrix3f check_rhat;
            check_rhat = rhat_map_[i * HorizonNum + j];
            tmp = rhat_map_[i * HorizonNum + j] * partial_d;

            normal_vec_.push_back(tmp);
        }
    }

    //normalization
    for (int i = 0; i < normal_vec_.size(); ++i) {
        double x0 = normal_vec_[i][0];
        double x1 = normal_vec_[i][1];
        double x2 = normal_vec_[i][2];
        double n = sqrt(x0 * x0 + x1 * x1 + x2 * x2);
        normal_vec_[i][0] /= n;
        normal_vec_[i][1] /= n;
        normal_vec_[i][2] /= n;
    }

};


void RangeImg::display_cloud(PointCloudTI::Ptr cloud, const std::string &window_name) {
    if (cloud->size() < 1) {
        std::cout << window_name << " display failure. Cloud contains no points\n";
        return;
    }
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
    pcl::visualization::PointCloudColorHandlerGenericField <PointTI> point_cloud_color_handler(cloud, "intensity");

    viewer->addPointCloud<PointTI>(cloud, point_cloud_color_handler, "id");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "id");
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    viewer->close();
}


void RangeImg::view_clouds(std::vector <PointCloudTI::Ptr> clouds, bool feature_flag) {
    int size = clouds.size();
    pcl::visualization::PCLVisualizer viewer;
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCLVisualizer"));

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(3.0);

    int point_size = 2;
    if (!feature_flag) {
        for (int i = 0; i < size; i++) {
            pcl::visualization::PointCloudColorHandlerCustom <PointTI> cloud_color(clouds[i], 175, 238, 238);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
                                                    std::to_string(i));
            viewer.addPointCloud(clouds[i], cloud_color, std::to_string(i));
        }
    } else {
        int r[] = {230, 255, 0, 205, 0};
        int g[] = {230, 215, 0, 92, 191};
        int b[] = {250, 0, 255, 92, 255};
        //yellow    edge_sharp
        //deepblue  edge_less
        //Red       surf_flat
        //skyblue   surf_less

        for (int i = 0; i < size; i++) {

            pcl::visualization::PointCloudColorHandlerCustom <PointTI> cloud_color(clouds[i], r[i], g[i], b[i]);
            viewer.addPointCloud(clouds[i], cloud_color, std::to_string(i));
            if (i == 2 || i == 4)
                point_size = 4;
            if (i == 1 || i == 3)
                point_size = 6;
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
                                                    std::to_string(i));

        }
    }
    while (!viewer.wasStopped()) {
        viewer.spinOnce(50);
    }

}


void RangeImg::view_normals() {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    //normals->resize(BeamNum * HorizonNum);
    normals->resize(128000);
    int size = cloud_nonground_->size();
    if (ground_flag_) {
        size = cloud_in_->size();
    }

    for (int i = 0; i < size/*BeamNum* HorizonNum*/; ++i) {
        //cloud_normal_->points[i].x = cloud_ordered_->points[i].x;
        //cloud_normal_->points[i].y = cloud_ordered_->points[i].y;
        //cloud_normal_->points[i].z = cloud_ordered_->points[i].z;
        //normals->points[i].normal_x = normal_vec_[i][0];
        //normals->points[i].normal_y = normal_vec_[i][1];
        //normals->points[i].normal_z = normal_vec_[i][2];

        if (ground_flag_) {
            cloud_normal_->points[i].x = cloud_in_->points[i].x;
            cloud_normal_->points[i].y = cloud_in_->points[i].y;
            cloud_normal_->points[i].z = cloud_in_->points[i].z;
        } else {
            cloud_normal_->points[i].x = cloud_nonground_->points[i].x;
            cloud_normal_->points[i].y = cloud_nonground_->points[i].y;
            cloud_normal_->points[i].z = cloud_nonground_->points[i].z;
        }

        normals->points[i].normal_x = normals_->points[i].normal_x;
        normals->points[i].normal_y = normals_->points[i].normal_y;
        normals->points[i].normal_z = normals_->points[i].normal_z;

        //normals->points[i].normal_x = kdnormals_vec_[i][0];
        //normals->points[i].normal_y = kdnormals_vec_[i][1];
        //normals->points[i].normal_z = kdnormals_vec_[i][2];
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Normals Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> color(cloud_normal_, 0, 238, 238);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_normal_, color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_normal_, normals, 1, 0.5, "normals");
    viewer->addCoordinateSystem(3.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }
}

std::vector <Eigen::Vector3d> RangeImg::ptptr2vec(const PointCloudTI::Ptr &cloud) {
    vector_.clear();
    vector_.resize(cloud->points.size());
    for (int i = 0; i < cloud->points.size(); ++i) {
        vector_[i][0] = cloud->points[i].x;
        vector_[i][1] = cloud->points[i].y;
        vector_[i][2] = cloud->points[i].z;
    }
    return vector_;

}


PointCloudT::Ptr RangeImg::xyzi2xyz(const PointCloudTI::Ptr &cloud_in) {

    PointCloudT::Ptr cloud_out(new PointCloudT);
    cloud_out->width = cloud_in->width;
    cloud_out->height = cloud_in->height;
    cloud_out->is_dense = cloud_in->is_dense;
    cloud_out->points.resize(cloud_in->width * cloud_in->height);
    for (int i = 0; i < cloud_in->points.size(); i++) {
        cloud_out->points[i].x = cloud_in->points[i].x;
        cloud_out->points[i].y = cloud_in->points[i].y;
        cloud_out->points[i].z = cloud_in->points[i].z;
    }
    return cloud_out;
}







