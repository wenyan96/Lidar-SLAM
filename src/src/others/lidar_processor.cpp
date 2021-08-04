//
// Created by wenyan on 04.01.21.
//

#include <glog/logging.h>
#include "mymt/lidar_processor.h"
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>

const string folder_path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/";
const string format = ".pcd";

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

LidarProcesser::LidarProcessor() {
}

void LidarProcessor::readCloud(PointCloudT::Ptr cloud_ptr, int cloud_index){
    LidarData lidar_data;

    string file_path= folder_path + std::to_string(cloud_index) + format;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud_source)== -1)
    {
        PCL_ERROR("PCLError:Couldn't read source file2 \n");
        LOG(ERROR)<<"GLOGError: Couldn't read source file \n";
        return (-1);
    }

    pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud_ptr);
    cloud_ptr_ = cloud_ptr;


    //读取时间


}


void LidarProcessor::preprocess( ){

    //Remove NaN points with x, y, or z equal to NaN.
    std::vector<int> indices;  //to save the indices
    pcl::removeNaNFromPointCloud(*cloud_ptr_, *cloud_ptr_, indices);
    std::cout<< "remove *cloud_ptr nan\n  size:"<<indices.size() <<std::endl;
    std::cout<<"Remain "<< cloud_ptr_->size() <<" data points of file after NaNremoval"<< std::endl;


    //Perform actual filtering: for each point, retrieves the number of neighbors within a certain
    //radius. The point will be considered an outlier if it has too few neighbors
    pcl::RadiusOutlierRemoval<PointT> outrem;
    //build the filter
    outrem.setInputCloud(cloud_ptr_);
    outrem.setRadiusSearch(4);
    outrem.setMinNeighborsInRadius(80);
    outrem.filter(*cloud_ptr_);   //the result of outlier removal
    std::cout<<"Remain "<< cloud_ptr_->size() <<" data points of target file after Radius outlierRemoval"<< std::endl;

    //Downsampling clouds
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_ptr_);
    vg.setLeafSize(1.50f, 1.50f, 1.50f);
    vg.filter(*cloud_ptr_);
    std::cout<<"Remain "<< cloud_ptr_->size() <<" data points of target file after Downsampling"<< std::endl;
}


void LidarProcessor::saveData(std::deque<LidarData>& clouds_data){
    if (new_cloud_data_.size() > 0){
        clouds_data.insert(clouds_data.end(), new_cloud_data_.begin(), new_cloud_Data_.end());
        new_cloud_data_.clear();
    }
}

