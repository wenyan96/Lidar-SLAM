
#include "feature_extraction/FeatureExtraction.h"


FeatureExtraction::FeatureExtraction(const PointCloudTI::Ptr &cloud_in,
                                     const index_s &index_info,
                                     const pcl::PointCloud<pcl::Normal>::Ptr &cloud_in_n) :
        FeatureExtractionInterface(cloud_in, index_info) {
    cloud_in_n_.reset(new pcl::PointCloud<pcl::Normal>);
    cloud_in_n_ = cloud_in_n;
    normals_info_.resize(cloud_in->size());
    picked_flag_.resize(cloud_in->size());
    feature_label_.resize(cloud_in->size());
    start_ind_ = index_info.start;
    end_ind_ = index_info.end;
    segment_col_ind_ = index_info.col;
    segment_range_mat_ = index_info.range;
    loadConfig();

}


void FeatureExtraction::loadConfig() {
    edgeThreshold = 2;
    surfThreshold = 2;
}


void FeatureExtraction::runMixedMethod(const int &edge_sharp_num,
                                       const int &edge_less_sharp_num,
                                       const int &surf_flat_num) {
    normalsVariation();
    featuresExtraction(edge_sharp_num, edge_less_sharp_num, surf_flat_num);
}

void FeatureExtraction::runNormalMethod(const int &edge_sharp_num,
                                        const int &edge_less_sharp_num,
                                        const int &surf_flat_num) {
    normalsVariation();
    markOccludedPoints();
    featuresExtraction(edge_sharp_num, edge_less_sharp_num, surf_flat_num);

}

void FeatureExtraction::runLoamMethod(const int &edge_sharp_num,
                                      const int &edge_less_sharp_num,
                                      const int &surf_flat_num) {
    markOccludedPoints();
}

void FeatureExtraction::runLegoMethod(const int &edge_sharp_num,
                                      const int &edge_less_sharp_num,
                                      const int &surf_flat_num) {
    normalsVariation();
    markOccludedPoints();
    featuresExtraction(edge_sharp_num, edge_less_sharp_num, surf_flat_num);

}

void FeatureExtraction::runExtraction(const int &edge_sharp_num,
                                      const int &edge_less_sharp_num,
                                      const int &surf_flat_num,
                                      const std::string &method) {

    method_ = method;
    if (method_ == "mixed") {
        runMixedMethod(edge_sharp_num, edge_less_sharp_num, surf_flat_num);
    } else if (method_ == "normal") {
        runNormalMethod(edge_sharp_num, edge_less_sharp_num, surf_flat_num);
    } else if (method_ == "lego") {
        runLegoMethod(edge_sharp_num, edge_less_sharp_num, surf_flat_num);
    } else if (method_ == "loam") {
        runLoamMethod(edge_sharp_num, edge_less_sharp_num, surf_flat_num);
    }
}

void FeatureExtraction::loadIndexInfo(const std::vector<int> &start_ind,
                                      const std::vector<int> &end_ind,
                                      const std::vector<int> &col_ind,
                                      const std::vector<double> &seg_range) {
    //FeatureExtraction::index_s index_info;
    start_ind_ = start_ind;
    end_ind_ = end_ind;
    segment_col_ind_ = col_ind;
    segment_range_mat_ = seg_range;
}


double FeatureExtraction::diffCalculation(const int &neighbor_num, const int &ind) {
    double diff_normal = 0;
    for (int l = 1; l <= neighbor_num; l++) {
        diff_normal = cloud_in_n_->points[ind + l].normal_x * cloud_in_n_->points[ind].normal_x +
                      cloud_in_n_->points[ind + l].normal_y * cloud_in_n_->points[ind].normal_y +
                      cloud_in_n_->points[ind + l].normal_z * cloud_in_n_->points[ind].normal_z +
                      cloud_in_n_->points[ind - l].normal_x * cloud_in_n_->points[ind].normal_x +
                      cloud_in_n_->points[ind - l].normal_y * cloud_in_n_->points[ind].normal_y +
                      cloud_in_n_->points[ind - l].normal_z * cloud_in_n_->points[ind].normal_z;

    }

    return diff_normal;
}

double FeatureExtraction::curvatureCalculation(const int &neighbor_num, const int &ind) {

    double diffX, diffY, diffZ;
    for (int i = 1; i <= neighbor_num; i++) {
        diffX += cloud_in_->points[ind - i].x - cloud_in_->points[ind].x +
                 cloud_in_->points[ind + i].x - cloud_in_->points[ind].x;
        diffY += cloud_in_->points[ind - i].y - cloud_in_->points[ind].y +
                 cloud_in_->points[ind + i].y - cloud_in_->points[ind].y;
        diffZ += cloud_in_->points[ind - i].z - cloud_in_->points[ind].z +
                 cloud_in_->points[ind + i].z - cloud_in_->points[ind].z;
    }
    double distance = diffX * diffX + diffY * diffY + diffZ * diffZ;
    return distance;
}


void FeatureExtraction::normalsVariation() {

    int cloud_size = cloud_in_->size();
    for (int i = 5; i < cloud_size - 5; i++) {
        //before:
        //double diff_normals_large = diffCalculation(5, i);
        //double diff_normals_small = diffCalculation(2, i);
        //double diff_normals = abs(diff_normals_large) + abs(diff_normals_small);

        double diff_normals_small = diffCalculation(3, i);
        double diff_normals = abs(diff_normals_small);
        double smoothness = curvatureCalculation(5, i);
        picked_flag_[i] = 0;   //是否经过筛选
        feature_label_[i] = 0;     //是否被选取为特征点
        normals_info_[i].diff = diff_normals;
        normals_info_[i].index = i;
        normals_info_[i].curvature = smoothness;
    }
}

void FeatureExtraction::markOccludedPoints() {
    int size = cloud_in_->points.size();
    for (int i = 5; i < size - 5; ++i) {  //ori: -6

        float depth1 = segment_range_mat_[i];
        float depth2 = segment_range_mat_[i + 1];
        int columnDiff = std::abs(int(segment_col_ind_[i + 1] - segment_col_ind_[i]));

        if (columnDiff < 10) {
            if (depth1 - depth2 > 0.3) {
                picked_flag_[i - 5] = 1;
                picked_flag_[i - 4] = 1;
                picked_flag_[i - 3] = 1;
                picked_flag_[i - 2] = 1;
                picked_flag_[i - 1] = 1;
                picked_flag_[i] = 1;
            } else if (depth2 - depth1 > 0.3) {
                picked_flag_[i + 1] = 1;
                picked_flag_[i + 2] = 1;
                picked_flag_[i + 3] = 1;
                picked_flag_[i + 4] = 1;
                picked_flag_[i + 5] = 1;
                //picked_flag_[i + 6] = 1;
            }
        }

        float diff1 = std::abs(float(segment_range_mat_[i - 1] - segment_range_mat_[i]));
        float diff2 = std::abs(float(segment_range_mat_[i + 1] - segment_range_mat_[i]));

        if (diff1 > 0.02 * segment_range_mat_[i] && diff2 > 0.02 * segment_range_mat_[i])
            picked_flag_[i] = 1;
    }
}

void FeatureExtraction::searchNeighbors(const int &neighbor_num, const int &col_diff_threshold, const int &ind) {
    for (int l = 1; l <= neighbor_num; l++) {
        int col_diff = std::abs(int(segment_col_ind_.at(ind + l) - segment_col_ind_.at(ind + l - 1)));
        if (col_diff > col_diff_threshold)
            break;
        picked_flag_.at(ind + l) = 1;
    }

    for (int l = -1; l >= -neighbor_num; l--) {
        int col_diff = std::abs(int(segment_col_ind_.at(ind + l) - segment_col_ind_.at(ind + l)));
        if (col_diff > col_diff_threshold)
            break;
        picked_flag_.at(ind + l) = 1;
    }
}

void FeatureExtraction::featuresExtraction(const int &edge_sharp_num,
                                           const int &edge_less_sharp_num,
                                           const int &surf_flat_num) {

    //feature_label_ ：
    // -1  flat point;        0 less flat point;
    // 1   less edge point;   2 sharp edge point;

    //picked_flag_ :
    // 0 not checked          1 checked
    edge_sharp_->clear();
    edge_less_sharp_->clear();
    surf_flat_->clear();
    surf_less_flat_->clear();
    //TODO:start_ind & segment_normals
    int edge_diff_num = 0;

    for (int i = 0; i < BeamNum; i++) {
        surf_less_beam_->clear();
        for (int j = 0; j < 6; j++) {
            int sp = (start_ind_.at(i) * (6 - j) + end_ind_.at(i) * j) / 6;
            int ep = (start_ind_.at(i) * (5 - j) + end_ind_.at(i) * (j + 1)) / 6 - 1;
            if (sp >= ep)
                continue;

            //this means to extract edge point first
            if (method_ == "lego") {
                std::sort(normals_info_.begin() + sp, normals_info_.begin() + ep, valueSort_c);
            } else if (method_ == "normal") {
                std::sort(normals_info_.begin() + sp, normals_info_.begin() + ep, valueSort_n);
            } else if (method_ == "mixed") {
                std::sort(normals_info_.begin() + sp, normals_info_.begin() + ep, valueSort_c);
            }

            int largest_picknum = 0;
            //sort rule: small -> large
            //std::cout << "*** CHECK edge normal info ***" <<std::endl;
            //Edge Points
            for (int k = sp; k <= ep; k++) {
            ///TODO modified params c&edge int k = ep; k >= sp; k--
            ///n&edge int k = sp; k <= ep; k++
                int ind = normals_info_.at(k).index;
                //default: 边点2个， 次边点20 （有两个没有打标签）
                //CHECK the edge value
                if (picked_flag_.at(ind) == 0 /*&& normals_info_.at(ind).diff < edgeThreshold &&
                    normals_info_.at(ind).curvature > 1.0*/) {  ///TODO < or >

                    largest_picknum++;

                    if (largest_picknum <= edge_sharp_num) {
                        feature_label_.at(ind) = 2;
                        edge_sharp_->push_back(cloud_in_->points.at(ind));
                        //std::cout<<" normal info curv: " << normals_info_.at(ind).curvature << "\n";
                        //std::cout << "edge_diff:" << normals_info_.at(ind).diff << " ";
                        edge_diff_num++;
                        edge_less_sharp_->push_back(cloud_in_->points.at(ind));
                    } else if (largest_picknum <= edge_less_sharp_num) {
                        feature_label_.at(ind) = 1;
                        edge_less_sharp_->push_back(cloud_in_->points.at(ind));
                    } else {
                        break;
                    }
                    picked_flag_.at(ind) = 1;
                    //考虑到特征点分布的均匀，特征点左右5个若有距离大于阈值，那么还有成为特征点的可能
                    //如果和已经找到的特征点index差别小），那么就标记为已经筛查过，不会再作为备选特征点了
                    searchNeighbors(5, 10, ind);
                }
            }

            //平面点
            int smallest_picknum = 0;
            for (int k = ep; k >= sp; k--) {   ///TODO modified params
                ///TODO modified params c&surf int k = sp; k <= ep; k++
                ///n&surf  int k = ep; k >= sp; k--
                int ind = normals_info_.at(k).index;
                //TODO: 地面信息？
                if (picked_flag_.at(ind) == 0 /*&& normals_info_[ind].diff > surfThreshold &&
                        normals_info_.at(ind).curvature < 0.1*/) {   ///TODO < or >
                    feature_label_.at(ind) = -1;
                    surf_flat_->push_back(cloud_in_->points.at(ind));  //TODO  normals_info.index??
                    smallest_picknum++;
                    //flat平面点4个 其他都当作次平面点
                    if (smallest_picknum >= surf_flat_num) {
                        break;
                    }
                    picked_flag_.at(ind) = 1;

                    searchNeighbors(5, 10, ind);
                }
            }

            //如果可以先把地面提取出来，那么就可以只在地面查找平面点
            //这里先不用划分地面的情况
            //选出次平面点
            //int k = ep; k >= sp; k--
            //for (int k = sp; k <= ep; k++) {
            for (int k = ep; k >= sp; k--) {
                if (feature_label_.at(k) <= 0) {
                    surf_less_beam_->push_back(cloud_in_->points.at(k));
                }
            }
        }

        //对平面点进行降采样
        surf_lessDS_beam_->clear();
        downsamp_filter_.setInputCloud(surf_less_beam_);
        downsamp_filter_.setLeafSize(0.4f, 0.4f, 0.4f);
        downsamp_filter_.filter(*surf_lessDS_beam_);
        *surf_less_flat_ += (*surf_lessDS_beam_);
        //surf_lessDS_beam_->clear();
        //downsamp_filter_.setInputCloud(surf_less_beam_);
        //downsamp_filter_.filter(*surf_lessDS_beam_);
        //*surf_less_flat_ += (*surf_less_beam_);
    }

    start_ind_.clear();
    end_ind_.clear();
    segment_col_ind_.clear();


    std::cout << "****edge normal diff ****" << edge_diff_num << std::endl;
}










