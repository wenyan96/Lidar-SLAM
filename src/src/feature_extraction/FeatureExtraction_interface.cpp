#include "feature_extraction/FeatureExtraction_interface.h"


FeatureExtractionInterface::FeatureExtractionInterface(const PointCloudTI::Ptr & cloud_in,
                                                       const index_s & index_info){

    //config params
    BeamNum = 64;
    HorizonNum = 1800;

    surf_flat_.reset(new PointCloudTI);
    surf_less_flat_.reset(new PointCloudTI);
    surf_less_beam_.reset(new PointCloudTI);
    surf_lessDS_beam_.reset(new PointCloudTI);
    edge_sharp_.reset(new PointCloudTI);
    edge_less_sharp_.reset(new PointCloudTI);
    cloud_in_.reset(new PointCloudTI);
    cloud_in_ = cloud_in;

    //start_ind_.assign(BeamNum, 0);
    //end_ind_.assign(BeamNum, 0);
    //segment_col_ind_.assign(BeamNum * HorizonNum, -1);
    //segment_range_mat_ = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(0.0));
    start_ind_ = index_info.start;
    end_ind_ = index_info.end;
    segment_col_ind_ = index_info.col;
    segment_range_mat_ = index_info.range;
    /*
    std::string config_file_path = WORK_SPACE_PATH +"/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    AngRes_x = config_node["edgeThreshold"].as<int>();
    AngRes_y = config_node["surfThreshold"].as<int>();
    */
}


PointCloudTI::Ptr FeatureExtractionInterface::surfFlatResult() {
    return surf_flat_;
}

PointCloudTI::Ptr FeatureExtractionInterface::surfLessFlatResult() {
    return surf_less_flat_;
}

PointCloudTI::Ptr FeatureExtractionInterface::edgeSharpResult() {
    return edge_sharp_;
}

PointCloudTI::Ptr FeatureExtractionInterface::edgeLessSharpResult() {
    return edge_less_sharp_;
}

