//
// Created by wenyan on 04.01.21.
//

#include "mymt/frontend_registration.h"
#include <cmath>
#include <pcl/common/transforms.h>
#include "glog/logging.h"
#include "mymt/lidar_data.h"

FrontEndRegistration::FrontEndRegistration()
    :ndt_ptr_(new pcl::NormalDistributionsTransform<LidarData::pcl::PointXYZ, LidarData::pcl::PointXYZ>()),
    local_map_ptr_(new LidarData::pcl::PointXYZ()),
    global_map_ptr_(new LidarData::pcl::PointXYZ()),
    result_cloud_ptr_(new LidarData::pcl::PointXYZ()){

    //default parameter setting
    cloud_filter_.setLeafSize(1.3, 1.3 1.3);
    local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
    display_filter_.setLeafSize(0.5, 0.5, 0.5);
    ndt_ptr_->setResolution(1.0);
    ndt_ptr_->setStepSize(0.1);
    ndt_ptr_->setTransformationEpsilon(0.01);
    ndt_ptr_->setMaximumIterations(30);
}


// use ndt registration and generate key frame
// use key frame pose to joint maps
Eigen::Matrix4f FrontEndRegistration::Update(const LidarData &lidar_data) {
    current_frame_.lidar_data.time = lidar_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*lidar_data.cloud_ptr, *current_frame_.lidar_data.cloud_ptr, indices);

    LidarData::pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new LidarData::pcl::PointCloud<pcl::PointXYZ>());
    cloud_filter_.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    cloud_filter_.filter(*filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    //for local map, no key frame
    //sees the current frame as first key frame and update local& global map

    // for the first frame (without local map)
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateNewFrame(current_frame_);
        return current_frame_.pose;
    }
    // for the non-first frames,get Transformation matrix from ndt registration
    ndt_ptr->setInputSource(filtered_cloud_ptr);
    ndt_ptr->align(*result_cloud_ptr_, predict_pose);
    current_frame_.pose = ndt_ptr_->getFinalTransformation();

    //generate key frame based on distance   How to tune these params?
    if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
        fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
        fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) > 2.0) {
        UpdateNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return current_frame_.pose;
}

bool FrontEndRegistration::SetInitPose(const int &init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEndRegistration::SetPredictPose(const int &predict_pose) {
    predict_pose_ = predict_pose;
    return true;
}

bool FrontEndRegistration::GetNewLocalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_ptr) {
    //?????????????????????????????
    if(has_new_local_map_){
        display_filter_.setInputCloud(local_map_ptr);
        display_filter_.filter(*local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEndRegistration::GetNewGlobalMap(LidarData::pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr) {
    //?????????????????????????????xingcan?
    if(has_new_global_map_){
        display_filter_.setInputCloud(global_map_ptr);
        display_filter_.filter(*global_map_ptr);
        return true;
    }
    return false;
}

bool FrontEndRegistration::GetCurrentScan(LidarData::pcl:PointCloud<pcl::PointXYZ>::Ptr current_scan_ptr){
    display_filter_.setInputCloud(result_cloud_ptr_);
    display_filter_.filter(*current_scan_ptr);
    return true;
}


void FrontEndRegistration::UpdateNewFrame(const Frame& new_key_frame){

    Frame key_frame = new_key_frame;
    //????????????????????????????????????????????
    key_frame.lidar_data.cloud_ptr.reset(new LidarData::pcl::PointCloud<pcl::PointXYZ>
                                                 (*new_key_frame.cloud_data.cloud_ptr));
    LidarData::pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new LidarData::pcl::PointCloud<
                                                             pcl::PointXYZ>());

    //update local map
    local_map_frames_.push_back(key_frame);
    while(local_map_frames_.size() > 20){
        local_map_frames_.pop_front(); //only saving 20 key frames to construct local map
    }
    local_map_ptr_.reset(new LidarData::pcl::PointCloud<pcl::PointXYZ>());
    for(int i = 0; i<local_map_frames_.size(); ++i){
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,
                                 *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map = true;

    //update ndt registration target cloud.
    // use just few local map frames
    if(local_map_frames_.size() <10){
        ndt_ptr_->setInputTarget(local_map_ptr_);
    }
    else{
        LidarData::pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_local_map_ptr(new LidarData::pcl::PointCloud<
                                                                               pcl::PointXYZ>());
        //???????????????ptr  new??????cloud??????
        local_map_filter_.setInputCloud(local_map_ptr);
        local_map_filter_.filter(*filtered_local_map_ptr);
        ndt_ptr->setInputTarget(filtered_local_map_ptr);

    }

    //update global map
    global_map_frames_.push_back(key_frame);
    if(global_map_frames_.size() % 100 !=0){
        return;
    }
    else{
        global_map_ptr_.reset(new LidarData::pcl::PointCloud<pcl::PointXYZ>());//?????????
        for (int i = 0; i< global_map_frames_.size(); ++i){
            pcl::transformPointCloud(*global_map_frames_.at(i).lidar_data.cloud_ptr,
                                     *transformed_cloud_ptr,
                                     global_map_frames_.at(i).pose);
            *global_map_ptr_ += *transformed_cloud_ptr;
        }
        has_new_global_map_ = true;

    }
}

