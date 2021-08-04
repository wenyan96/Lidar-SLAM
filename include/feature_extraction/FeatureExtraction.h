#pragma once
#include "FeatureExtraction_interface.h"


class FeatureExtraction: public FeatureExtractionInterface{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //input +normal info

    struct normals_t {
        double diff;
        int index;
        double curvature = -1;
    };

    struct curvature_t {
        double curv;
        int index;
    };


    FeatureExtraction(const PointCloudTI::Ptr & cloud_in,
                      const index_s & index_info,
                      const pcl::PointCloud<pcl::Normal>::Ptr& cloud_in_n);

    void loadConfig();
    void runExtraction(const int &edge_sharp_num,
                       const int &edge_less_sharp_num,
                       const int &surf_flat_num,
                       const std::string& method);

    void runMixedMethod(const int &edge_sharp_num,
                        const int &edge_less_sharp_num,
                        const int &surf_flat_num);
    void runNormalMethod(const int &edge_sharp_num,
                        const int &edge_less_sharp_num,
                        const int &surf_flat_num);
    void runLegoMethod(const int &edge_sharp_num,
                       const int &edge_less_sharp_num,
                       const int &surf_flat_num);
    void runLoamMethod(const int &edge_sharp_num,
                       const int &edge_less_sharp_num,
                       const int &surf_flat_num);

    void loadIndexInfo(const std::vector<int>& start_ind,
                       const std::vector<int>& end_ind,
                       const std::vector<int>& col_ind,
                       const std::vector<double>& seg_range);

    double diffCalculation(const int & neighbor_num, const int & ind);
    double curvatureCalculation(const int & neighbor_num, const int & ind);
    void normalsVariation();
    void searchNeighbors(const int &neighbor_num, const int& col_diff_threshold,const int &ind);
    void markOccludedPoints();
    void featuresExtraction(const int &edge_sharp_num,
                            const int &edge_less_sharp_num,
                            const int &surf_flat_num);


    static bool valueSort_n(const normals_t &a, const normals_t &b) {
        return a.diff < b.diff;
    }
    static bool valueSort_c(const normals_t &a, const normals_t &b) {
        return a.curvature < b.curvature;
    }


private:
    pcl::VoxelGrid<PointTI> downsamp_filter_;
    int edgeThreshold;
    int surfThreshold;

    pcl::PointCloud<pcl::Normal>::Ptr cloud_in_n_;

    std::vector<int> picked_flag_;
    std::vector<int> feature_label_;
    std::vector<normals_t> normals_info_;
    std::vector<curvature_t> curvature_info_;
    std::string method_;
};

