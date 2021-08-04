#pragma once

#include <Eigen/Dense>
#include <common/utils.h>
#include "preprocessing/Preprocessing.h"
#include <common/utils.h>

class FeatureExtractionInterface{
public:
    virtual ~FeatureExtractionInterface() = default;
    struct index_s{
        std::vector<int> start;
        std::vector<int> end;
        std::vector<int> col;
        std::vector<double> range;
    };

    FeatureExtractionInterface(const PointCloudTI::Ptr & cloud_in,
                               const index_s& index_info);

    PointCloudTI::Ptr surfFlatResult();
    PointCloudTI::Ptr surfLessFlatResult();
    PointCloudTI::Ptr edgeSharpResult();
    PointCloudTI::Ptr edgeLessSharpResult();



protected:

    std::vector<int> start_ind_;
    std::vector<int> end_ind_;
    std::vector<int> segment_col_ind_;
    std::vector<double> segment_range_mat_;

    //Input
    PointCloudTI::Ptr cloud_in_;

    //Output
    PointCloudTI::Ptr surf_flat_;
    PointCloudTI::Ptr surf_less_flat_;
    PointCloudTI::Ptr surf_less_beam_;
    PointCloudTI::Ptr surf_lessDS_beam_;
    PointCloudTI::Ptr edge_sharp_;
    PointCloudTI::Ptr edge_less_sharp_;

    int BeamNum;
    int HorizonNum;

private:


};