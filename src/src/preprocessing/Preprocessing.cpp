#include "preprocessing/Preprocessing.h"
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <vector>
#include <cmath>
#include <deque>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/time.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <yaml-cpp/yaml.h>

//#include <glog/logging.h>


Preprocessing::Preprocessing() {
    loadConfig();
    initialization();   //分配内存
    normalsPrecompute();

    std::pair<int, int> neighbor;
    // left, up, down, right
    neighbor.first = -1;
    neighbor.second = 0;
    neighborIterator_.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = 1;
    neighborIterator_.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = -1;
    neighborIterator_.push_back(neighbor);
    neighbor.first = 1;
    neighbor.second = 0;
    neighborIterator_.push_back(neighbor);
}


void Preprocessing::loadConfig() {
    /*
    BeamNum = 64;
    HorizonNum = 1800;
    StartScanId = 10;
    GroundThreshold = 40;
    AngRes_x = 360.0 / double(HorizonNum);
    AngRes_y = 26.8 / (BeamNum - 1.0);
    AngUp = 2;
    AngBottom = -24.8;
    AngRangeHalf = (AngUp - AngBottom) / 2;
    SegmentAngle = 35.0;
    SegmentValidPointNum1 = 30;
    SegmentValidPointNum2 = 7;
    SegmentValidLineNum = 4;

    GroundSize = 11;
    GroundDsSize = 22;
    */


    BeamNum = 128;
    HorizonNum = 1800;
    StartScanId = 10;
    GroundThreshold = 90;
    AngRes_x = 360.0 / double(HorizonNum);
    AngRes_y = 40.0 / (BeamNum - 1.0);
    AngUp = 15.0;
    AngBottom = -25.0;
    AngRangeHalf = (AngUp - AngBottom) / 2;
    SegmentAngle = 45.0;
    SegmentValidPointNum1 = 30;
    SegmentValidPointNum2 = 7;
    SegmentValidLineNum = 4;

    GroundSize = 11;
    GroundDsSize = 22;

    /*
    std::string config_file_path = WORK_SPACE_PATH +"/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    if (config_node["HorizonNum"]) {
        std::cout << config_node["HorizonNum"].as<std::string>();
    }
    BeamNum = config_node["BeamNum"].as<double>();
    std::cout << "B" << BeamNum;
    HorizonNum = config_node["HorizonNum"].as<int>();
    GroundScanId = config_node["GroundScanId"].as<int>();
    AngRes_x = config_node["AngRes_x"].as<double>();
    AngRes_y = config_node["AngRes_y"].as<double>();
    AngUp = config_node["AngUp"].as<double>();
    AngBottom = config_node["AngBottom"].as<double>();
    AngRangeHalf = config_node["AngRangeHalf"].as<double>();
    SegmentAngle = config_node["SegmentAngle"].as<double>();
    SegmentValidPointNum = config_node["SegmentValidPointNum"].as<int>();
    SegmentValidLineNum = config_node["SegmentValidLineNum"].as<int>();
     */
}


void Preprocessing::preprocessingExecution(const PointCloudTI::Ptr &cloud_in) {
    // add initial() ???
    cloud_in_ = cloud_in;
    pcl::console::TicToc tt0;
    tt0.tic();

    cloudProjection();
    std::cout << " ---> Time for cloudProjection : " << tt0.toc() << std::endl;
    normalsCalculation();
    std::cout << " ---> Time for normalsCalculation : " << tt0.toc() << std::endl;
    groundRemoval();
    std::cout << " ---> Time for groundRemoval : " << tt0.toc() << std::endl;

    cloudSegmentation();
    std::cout << " ---> Time for cloudSegmentation : " << tt0.toc() << std::endl;
    std::cout << " *****label_count : " << label_count_ << std::endl;

}

void Preprocessing::initialization() {
    cloud_in_.reset(new PointCloudTI);
    cloud_order_.reset(new PointCloudTI);
    cloud_order_->resize(BeamNum * HorizonNum);
    PointTI nanPoint;
    nanPoint.x = std::numeric_limits<double>::quiet_NaN();
    nanPoint.y = std::numeric_limits<double>::quiet_NaN();
    nanPoint.z = std::numeric_limits<double>::quiet_NaN();
    nanPoint.intensity = -1;
    std::fill(cloud_order_->points.begin(), cloud_order_->points.end(), nanPoint);

    cloud_ground_.reset(new PointCloudTI);
    ground_ds_less_.reset(new PointCloudTI);
    ground_ds_.reset(new PointCloudTI);
    cloud_outliers_.reset(new PointCloudTI);
    cloud_segment_.reset(new PointCloudTI);

    normals_.reset(new pcl::PointCloud<pcl::Normal>);
    normals_->resize(BeamNum * HorizonNum);
    ground_normals_.reset(new pcl::PointCloud<pcl::Normal>);
    ground_ds_normals_.reset(new pcl::PointCloud<pcl::Normal>);
    segment_normals_.reset(new pcl::PointCloud<pcl::Normal>);

    range_mat_ = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(0.0));
    ground_mat_ = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(0));
    label_mat_ = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(0));


    edge_sharp_.reset(new PointCloudTI);
    edge_less_sharp_.reset(new PointCloudTI);
    surf_flat_.reset(new PointCloudTI);
    surf_less_flat_.reset(new PointCloudTI);
    surf_less_beam_.reset(new PointCloudTI);
    surf_lessDS_beam_.reset(new PointCloudTI);

    //valid_col_.assign(BeamNum * HorizonNum, -1);
    start_ind_.assign(BeamNum, 0);
    end_ind_.assign(BeamNum, 0);
    //segment_col_ind_.assign(BeamNum, 0);
    //cv::Mat segment_range_mat_ = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(0.0));


    queue_idx_ = new int[BeamNum * HorizonNum];
    queue_idy_ = new int[BeamNum * HorizonNum];
    allPushed_idx_ = new int[BeamNum * HorizonNum];
    allPushed_idy_ = new int[BeamNum * HorizonNum];
    label_count_ = 1;

}


void Preprocessing::normalsPrecompute() {
    //index_0 - Pi[-x], index_half - 0   index_w_1 - (-Pi)
    for (int i = 0; i < HorizonNum; i++) {
        double theta = -(i - HorizonNum / 2.0) / (HorizonNum / 2.0) * M_PI;
        Theta_.push_back(theta);
        cosTheta_.push_back(cos(theta));
        sinTheta_.push_back(sin(theta));
    }


    //index_0 - AngUp;    index_h_1 - AngBottom
    for (int j = 0; j < BeamNum; j++) {
        double phi = (-(j - (BeamNum - 1) / 2.0) / ((BeamNum - 1) / 2.0) * M_PI / 180 * AngRangeHalf -
                      (AngRangeHalf - AngUp) * M_PI / 180);
        Phi_.push_back(phi);
        cosPhi_.push_back(cos(phi));
        sinPhi_.push_back(sin(phi));
    }

}

void Preprocessing::cloudDisplay(const PointCloudTI::Ptr &cloud, const PointCloudTI::Ptr &order, const std::string &window_name) {
    if (cloud->empty()) {
        std::cout << window_name << " display failure. Cloud contains no points\n";
        return;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
    //pcl::visualization::PointCloudColorHandlerGenericField<PointTI> point_cloud_color_handler(cloud, "intensity");


    pcl::visualization::PointCloudColorHandlerCustom<PointTI> single_color1(cloud, 180, 180, 50);
    pcl::visualization::PointCloudColorHandlerCustom<PointTI> single_color2(order, 255, 255, 200);
    viewer->addPointCloud<PointTI>(cloud, single_color1, "id");
    viewer->addPointCloud<PointTI>(order, single_color2, "origin");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "id");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.3);
    //viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);

    }

    viewer->close();
}

void Preprocessing::cloudProjection() {

    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(*cloud_in_, *cloud_in_, indices);

    int cloud_size = cloud_in_->size();
    std::cout << "cloud_in" << cloud_size << std::endl;
    for (int i = 0; i < cloud_size; i++) {
        double x = cloud_in_->points[i].x;
        double y = cloud_in_->points[i].y;
        double z = cloud_in_->points[i].z;

        //too close point is invalid
        double lidar_range = std::sqrt(x * x + y * y + z * z);
        if (lidar_range < 0.1) {
            continue;
        }

        double horizontal_angle = std::atan2(y, x);
        int col_index = round(-(HorizonNum / 360.0) * horizontal_angle * 180.0 / M_PI + HorizonNum / 2.0);

        if (col_index >= HorizonNum)
            col_index -= HorizonNum;
        if (col_index < 0 || col_index >= HorizonNum) {   //?这里可以这样处理吗？
            continue;
        }


        //row_index_存储的是原点云pcd中点所对应的投影row
        double vertical_angle = std::asin(z / lidar_range);   // unit: rad
        int row_index = round((-vertical_angle * 180.0 / M_PI + 2.0) / AngRes_y);
        if (row_index < 0 || row_index >= BeamNum) {
            continue;
        }


        double range_val = range_mat_.at<double>(row_index, col_index);
        if (range_val != 0.0 && lidar_range != 0.0) {
            if (range_val >= lidar_range) {
                continue;
            } else {
                range_mat_.at<double>(row_index, col_index) = lidar_range;
                int index = col_index + row_index * HorizonNum;
                cloud_order_->points[index].x = x;
                cloud_order_->points[index].y = y;
                cloud_order_->points[index].z = z;
                cloud_order_->points[index].intensity = row_index + col_index / 10000.0;
            }
        } else {
            range_mat_.at<double>(row_index, col_index) = lidar_range;
            int index = col_index + row_index * HorizonNum;
            cloud_order_->points[index].x = x;
            cloud_order_->points[index].y = y;
            cloud_order_->points[index].z = z;
            cloud_order_->points[index].intensity = row_index + col_index / 10000.0;
        }

    }
    std::cout << "cloud_order size" << cloud_order_->size() << std::endl;

}

void Preprocessing::normalsCalculation() {
    //For Check

    /*
    cv::Mat color_img0 = cv::Mat(BeamNum, HorizonNum, CV_8U);
    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {
             color_img0.at<unsigned char>(i, j) = (unsigned char) 3 * (unsigned char) (range_mat_.at<double>(i, j));
        }
    }
    applyColorMap(color_img0, color_img0, cv::COLORMAP_VIRIDIS);
    cv::imshow("colorMap before", color_img0);
    cv::waitKey();
    */


    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 3));

    cv::Mat res = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(0.0));;
    cv::morphologyEx(range_mat_, res, cv::MORPH_DILATE, kernel);   //255初始化
    cv::GaussianBlur(res, res, cv::Size(3, 3), 35, 35);

    //check img after blur
    /*
    cv::Mat color_img = cv::Mat(BeamNum, HorizonNum, CV_8U);
    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizonNum; ++j) {
            color_img.at<unsigned char>(i, j) = (unsigned char) 3 * (unsigned char) (res.at<double>(i, j));
        }
    }
    applyColorMap(color_img, color_img, cv::COLORMAP_VIRIDIS);
    cv::imshow("colorMap after", color_img);
    cv::waitKey();
    */
    double min, max;
    cv::minMaxIdx(range_mat_, &min, &max);
    std::cout << "min lidar_range= " << min << std::endl;
    std::cout << "max lidar_range= " << max << std::endl;

    cv::Mat dr_dth = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(0));
    cv::Mat dr_dph = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(0));
    for (int i = 0; i < BeamNum; i++) {
        for (int j = 0; j < HorizonNum; j++) {
            if (j > 0 && j < HorizonNum - 1) {

                dr_dth.at<double>(i, j) = (res.at<double>(i, j + 1) - res.at<double>(i, j - 1)) /
                                          (2 * (Theta_[0] - Theta_[1]));


                /*
                 dr_dth.at<double>(i, j) = (-res.at<double>(i, j + 2) + 8 * res.at<double>(i, j + 1) -
                                           8 * res.at<double>(i, j - 1) + res.at<double>(i, j - 2)) /
                                          (12 * (Theta_[0] - Theta_[1]));
                                          */


            }

            if (i < BeamNum - 2) {
                dr_dph.at<double>(i, j) =
                        (res.at<double>(i + 2, j) - res.at<double>(i, j)) / (2 * (Phi_[0] - Phi_[1]));
            }

            /*if (i> 1&& i < BeamNum - 2) {
                dr_dph.at<double>(i, j) =
                        (res.at<double>(i + 2, j) + res.at<double>(i + 1, j)- res.at<double>(i-1, j) -
                                res.at<double>(i-2, j)) / (6 * (Phi_[0] - Phi_[1]));*/
            //}

        }
    }

    Eigen::Matrix3d r_theta;
    Eigen::Matrix3d r_phi;
    Eigen::Matrix3d r;
    Eigen::Vector3d partial;
    Eigen::Vector3d normal;

    pcl::console::TicToc tt;
    tt.tic();

    bool check = true;
    for (int i = 0; i < BeamNum; i++) {
        for (int j = 0; j < HorizonNum; j++) {

            r_theta << cosTheta_[j], -sinTheta_[j], 0,
                    sinTheta_[j], cosTheta_[j], 0,
                    0, 0, 1;
            r_phi << cosPhi_[i], 0, -sinPhi_[i],
                    0, 1, 0,
                    sinPhi_[i], 0, cosPhi_[i];
            r = r_theta * r_phi;
            partial << 1,
                    1 / (res.at<double>(i, j) * cosPhi_[i]) * dr_dth.at<double>(i, j),
                    1 / res.at<double>(i, j) * dr_dph.at<double>(i, j);
            normal = -r * partial;
            double norm = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            normals_->points[i * HorizonNum + j].normal_x = normal[0] / norm;
            normals_->points[i * HorizonNum + j].normal_y = normal[1] / norm;
            normals_->points[i * HorizonNum + j].normal_z = normal[2] / norm;
        }
    }

}

void Preprocessing::groundRemoval() {
    // ground_mat
    // -1, no valid info to check if ground or not
    //  0, initial value, after validation, means not ground
    //  1, ground

    //from GroundScanId to BeamNum : definitely ground points (except points.z higher than a value)
    //then the point is not on the ground but in the air


    for (int i = StartScanId; i < BeamNum; i++) {
        for (int j = 0; j < HorizonNum; j++) {
            int index = j + i * HorizonNum;
            if (cloud_order_->points[index].intensity == -1) {
                ground_mat_.at<int>(i, j) = -1;
                continue;
            }

            if (cloud_order_->points[index].z > -2.6 &&
                cloud_order_->points[index].z < -1.0 &&
                normals_->points[index].normal_z > 0.90) {
                ground_mat_.at<int>(i, j) = 1;
            }

        }
    }

    //label_mat
    // -1  - do not for segmentation: it could be ground or invalid point
    for (int i = 0; i < BeamNum; i++) {
        for (int j = 0; j < HorizonNum; j++) {
            if (ground_mat_.at<int>(i, j) == 1 || ground_mat_.at<int>(i, j) == -1) {
                label_mat_.at<int>(i, j) = -1;
                if (ground_mat_.at<int>(i, j) == 1) {
                    cloud_ground_->push_back(cloud_order_->points[j + i * HorizonNum]);
                    //double x = cloud_order_->points[j + i * HorizonNum].x;
                    //double y = cloud_order_->points[j + i * HorizonNum].y;
                    //double z = cloud_order_->points[j + i * HorizonNum].z;
                    //if (j % 11 == 0 && x * x + y * y + z * z <= GroundThreshold * GroundThreshold) {
                    //    ground_ds_->push_back(cloud_order_->points[j + i * HorizonNum]);
                    //}
                    //if (j % 15 == 0 && x * x + y * y + z * z <= GroundThreshold * GroundThreshold) {
                    //    ground_ds_less_->push_back(cloud_order_->points[j + i * HorizonNum]);
                    //}

                }
            }
        }
    }
    std::cout << "ground_size (all)" << cloud_ground_->size() << std::endl;

    PointCloudTI::Ptr cloud_tmp(new PointCloudTI);
    for (auto point : *cloud_ground_) {
        if (point.x * point.x + point.y * point.y + point.z * point.z <= 1600) {
            cloud_tmp->push_back(point);
        }
    }

    ground_filter_.setInputCloud(cloud_tmp);
    ground_filter_.setLeafSize(0.4f, 0.4f, 0.4f);
    ground_filter_.filter(*ground_ds_);
    ground_filter_.setLeafSize(0.6f, 0.6f, 0.6f);
    ground_filter_.filter(*ground_ds_less_);

    std::cout << "size ground_ds : " << ground_ds_->size() << std::endl;
    std::cout << "size ground_ds_less : " << ground_ds_less_->size() << std::endl;

}


void Preprocessing::cloudSegmentation() {
    for (int i = 0; i < BeamNum; i++) {
        for (int j = 0; j < HorizonNum; j++) {
            //label_mat_
            //label > 0 : label of objects
            //label = 0 : initial status, not segmented into one object
            //label = 9999 : outliers
            //label for ground/invalid points ： -1
            //try clustering only for the non-ground points and valid points

            if (label_mat_.at<int>(i, j) == 0) {
                componentsLabeling(i, j, label_mat_, range_mat_);
            }
        }
    }

    int cloud_segment_size = 0;

    for (int i = 0; i < BeamNum; i++) {
        start_ind_[i] = cloud_segment_size + 5;  //TODO: 需要-1？？有误？
        for (int j = 0; j < HorizonNum; j++) {
            if (label_mat_.at<int>(i, j) > 0 || ground_mat_.at<int>(i, j) == 1) {
                if (label_mat_.at<int>(i, j) == 9999) {
                    //if (i < GroundScanId && j % 5 == 0) {
                    //for non-ground points and every 5 points (dowmsample?)
                    cloud_outliers_->push_back(cloud_order_->points[j + i * HorizonNum]);
                    continue;
                    //}else
                    //continue;
                }

                if (ground_mat_.at<int>(i, j) == 1) {
                    //skip all ground points
                    continue;
                }

                //cloud_segment 不会有标记为地面的点（实际上，有些地面点没有被标记成功，也可能会加进来）
                //segmentedCloud_range[cloud_segment_size] = range_mat_.at<double>(i, j);
                segment_col_ind_.push_back(j);
                segment_range_mat_.push_back(range_mat_.at<double>(i, j));
                cloud_segment_->push_back(cloud_order_->points[j + i * HorizonNum]);
                ++cloud_segment_size;
            }
        }

        end_ind_[i] = cloud_segment_size - 5;
    }
    std::cout << " cloud_outliers_size " << cloud_outliers_->size() << std::endl;
    std::cout << " cloud_segment_size " << cloud_segment_size << std::endl;

}

void Preprocessing::componentsLabeling(int row, int col, cv::Mat &label_mat, cv::Mat &range_mat) {

    double d1, d2, alpha, angle;
    int from_idx, from_idy, this_idx, this_idy;

    std::vector<char> lineCountFlag(BeamNum, 'F');

    //手写队列，把range放入
    queue_idx_[0] = row;
    queue_idy_[0] = col;
    int queue_size = 1;
    int queue_start = 0;
    int queue_end = 1;


    allPushed_idx_[0] = row;
    allPushed_idy_[0] = col;
    int allPushid_size = 1;

    while (queue_size > 0) {
        //pop point
        from_idx = queue_idx_[queue_start];  //起点
        from_idy = queue_idy_[queue_start];
        --queue_size;
        ++queue_start;

        //mark popped point
        label_mat.at<int>(from_idx, from_idy) = label_count_;
        //loop through all neighboring grids of popped grid
        for (const auto &iter : neighborIterator_) {
            //new index
            //iter.first - 垂直方向
            //iter.second - 水平360
            this_idx = from_idx + iter.first;
            this_idy = from_idy + iter.second;
            //index should be within the boundary
            if (this_idx < 0 || this_idx >= BeamNum) continue;

            if (this_idy < 0) this_idy = HorizonNum - 1;
            if (this_idy > HorizonNum) this_idy = 0;

            //如果这一个已经计算了，退出
            if (label_mat.at<int>(this_idx, this_idy) != 0) continue;

            d1 = std::max(range_mat.at<double>(from_idx, from_idy),
                          range_mat.at<double>(this_idx, this_idy));
            d2 = std::min(range_mat.at<double>(from_idx, from_idy),
                          range_mat.at<double>(this_idx, this_idy));

            if (iter.first == 0) {
                alpha = AngRes_x / 180 * M_PI;  //360grad
            } else {
                alpha = AngRes_y / 180 * M_PI;
            }

            angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

            if (angle > SegmentAngle / 180 * M_PI) {
                queue_idx_[queue_end] = this_idx;
                queue_idy_[queue_end] = this_idy;
                ++queue_size;
                ++queue_end;

                label_mat.at<int>(this_idx, this_idy) = label_count_;
                lineCountFlag[this_idx] = 'T';  //TODO

                allPushed_idx_[allPushid_size] = this_idx;
                allPushed_idy_[allPushid_size] = this_idy;
                ++allPushid_size;
            }
        }

    }

    //check if the segment is valid
    bool feasibleSegment = false;
    if (allPushid_size >= SegmentValidPointNum1) {
        feasibleSegment = true;  //一个聚类
    } else if (allPushid_size >= SegmentValidPointNum2) {
        //点数大于5 且至少在3条Beam上，也ok
        int line_count = 0;
        for (char i : lineCountFlag) {
            if (i == 'T') {
                ++line_count;
            }
        }
        if (line_count >= SegmentValidLineNum) {
            feasibleSegment = true;
        }
    }

    //if segment is valid, mark these points
    if (feasibleSegment) {
        ++label_count_;
    } else {
        for (int i = 0; i < allPushid_size; i++) {
            label_mat_.at<int>(allPushed_idx_[i], allPushed_idy_[i]) = 9999;
        }
    }

}


void Preprocessing::normalDisplay(const PointCloudTI::Ptr &cloud_in, pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    PointCloudT::Ptr cloud_in_xyz(new PointCloudT);
    cloud_in_xyz->resize(cloud_in->size());
    copyPointCloud(*cloud_in, *cloud_in_xyz);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Normals Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_in_xyz, 0, 138, 238);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in_xyz, color, "sample cloud for normal");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                             "sample cloud for normal");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_in_xyz, normals, 1, 0.3, "normals");
    viewer->addCoordinateSystem(3.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }
}


PointCloudTI::Ptr Preprocessing::orderResult() {
    return cloud_order_;
}

PointCloudTI::Ptr Preprocessing::outliersResult() {
    return cloud_outliers_;
}

PointCloudTI::Ptr Preprocessing::groundResult() {
    return ground_ds_;
}

PointCloudTI::Ptr Preprocessing::groundDsResult() {
    return ground_ds_less_;
}

PointCloudTI::Ptr Preprocessing::segmentResult() {
    return cloud_segment_;
}


pcl::PointCloud<pcl::Normal>::Ptr Preprocessing::normalsResult() {
    return normals_;
}

pcl::PointCloud<pcl::Normal>::Ptr Preprocessing::segmentNormals() {
    for (int i = 0; i < cloud_segment_->size(); i++) {
        double intensity = cloud_segment_->points[i].intensity;
        int row = int(intensity);
        int col = ((intensity - int(intensity)) * 10000.0);
        //std::cout << "col" << col << " ";
        segment_normals_->push_back(normals_->points[row * HorizonNum + col]);
    }
    return segment_normals_;
}

pcl::PointCloud<pcl::Normal>::Ptr Preprocessing::groundNormals() {
    for (int i = 0; i < ground_ds_->size(); i++) {
        double intensity = ground_ds_->points[i].intensity;
        int row = int(intensity);
        int col = ((intensity - int(intensity)) * 10000.0);
        //std::cout << "col" << col << std::endl;
        ground_normals_->push_back(normals_->points[row * HorizonNum + col]);
    }
    return ground_normals_;
}

pcl::PointCloud<pcl::Normal>::Ptr Preprocessing::groundDsNormals() {
    for (int i = 0; i < ground_ds_less_->size(); i++) {
        double intensity = ground_ds_less_->points[i].intensity;
        int row = int(intensity);
        int col = ((intensity - int(intensity)) * 10000.0);
        std::cout << "col" << col << std::endl;
        ground_ds_normals_->push_back(normals_->points[row * HorizonNum + col]);
    }
    return ground_ds_normals_;
}


std::vector<int> Preprocessing::getStartInd() {
    return start_ind_;
}

std::vector<int> Preprocessing::getEndInd() {
    return end_ind_;
}

std::vector<int> Preprocessing::getColInd() {
    return segment_col_ind_;
}

std::vector<double> Preprocessing::getSegRangeMat() {
    return segment_range_mat_;
}


Preprocessing::~Preprocessing() = default;
