#pragma once

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
#include <common/utils.h>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class Preprocessing {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Preprocessing();

    struct index_s{
        std::vector<int> start;
        std::vector<int> end;
        std::vector<int> col;
        cv::Mat range;
    };

    ~Preprocessing();

    void preprocessingExecution(const PointCloudTI::Ptr &cloud_in);

    void loadConfig();

    void initialization();

    void normalsPrecompute();  //构造函数里解决
    void cloudProjection();

    void normalsCalculation();

    void groundRemoval();

    void cloudSegmentation();

    void componentsLabeling(int row, int col, cv::Mat &label_mat, cv::Mat &range_mat);

    //viewer
    void cloudDisplay(const PointCloudTI::Ptr &cloud, const PointCloudTI::Ptr &order,const std::string &window_name);

    void normalDisplay(const PointCloudTI::Ptr &cloud_in, pcl::PointCloud<pcl::Normal>::Ptr &normals);


    PointCloudTI::Ptr orderResult();
    PointCloudTI::Ptr outliersResult();
    PointCloudTI::Ptr segmentResult();
    PointCloudTI::Ptr groundResult();
    PointCloudTI::Ptr groundDsResult();
    std::vector<int> getStartInd();
    std::vector<int> getEndInd();
    std::vector<int> getColInd();
    std::vector<double> getSegRangeMat();


    pcl::PointCloud<pcl::Normal>::Ptr normalsResult();
    pcl::PointCloud<pcl::Normal>::Ptr segmentNormals();
    //ground
    pcl::PointCloud<pcl::Normal>::Ptr groundNormals();
    pcl::PointCloud<pcl::Normal>::Ptr groundDsNormals();

private:

    PointCloudTI::Ptr cloud_in_;
    PointCloudTI::Ptr cloud_order_;
    PointCloudTI::Ptr cloud_ground_;
    PointCloudTI::Ptr ground_ds_less_;
    PointCloudTI::Ptr ground_ds_;
    PointCloudTI::Ptr cloud_outliers_;
    PointCloudTI::Ptr cloud_segment_;   //including part of ground

    pcl::PointCloud<pcl::Normal>::Ptr normals_;  // all normals
    //for ground
    pcl::PointCloud<pcl::Normal>::Ptr ground_normals_;
    pcl::PointCloud<pcl::Normal>::Ptr ground_ds_normals_;
    //for the points after segmentation
    pcl::PointCloud<pcl::Normal>::Ptr segment_normals_;

    std::vector<double> Theta_;
    std::vector<double> cosTheta_;
    std::vector<double> sinTheta_;
    std::vector<double> Phi_;
    std::vector<double> cosPhi_;
    std::vector<double> sinPhi_;
    cv::Mat range_mat_;
    cv::Mat ground_mat_;
    cv::Mat label_mat_;

    PointCloudTI::Ptr edge_sharp_;
    PointCloudTI::Ptr edge_less_sharp_;
    PointCloudTI::Ptr surf_flat_;
    PointCloudTI::Ptr surf_less_flat_;
    PointCloudTI::Ptr surf_less_beam_;
    PointCloudTI::Ptr surf_lessDS_beam_;

    //std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > segment_normals_;
    //std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > segment_normals_;
    //点云中每个线束有效的起始和终止index
    std::vector<int> start_ind_;
    std::vector<int> end_ind_;
    std::vector<int> segment_col_ind_;
    std::vector<double> segment_range_mat_;
    //cv::Mat segment_range_mat_;

    pcl::VoxelGrid<PointTI> ground_filter_;

    std::vector<std::pair<int, int> > neighborIterator_;
    int *allPushed_idx_; // array for tracking points of a segmented object
    int *allPushed_idy_;
    int *queue_idx_; // array for breadth-first search process of segmentation, for speed
    int *queue_idy_;
    int label_count_;


    //parameters with config
    int BeamNum;
    int HorizonNum;
    int StartScanId;
    double AngRes_x;
    double AngRes_y;
    double AngUp;
    double AngBottom;
    double AngRangeHalf;

    double GroundSize;
    double GroundDsSize;
    double GroundThreshold;

    double SegmentAngle;
    int SegmentValidPointNum1;
    int SegmentValidPointNum2;
    int SegmentValidLineNum;

};





