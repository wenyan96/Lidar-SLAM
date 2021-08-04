#include "registration_frames.h"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "nanoflann.hpp"
#include <utility>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>


RegistrationFrames::RegistrationFrames(const int &max_iterations,
                                       Eigen::Matrix4d &initial_guess,
                                       std::vector<Eigen::Vector3d> vec_src,   //src-curr求解之后直接把特征src点云进行了变换
                                       std::vector<Eigen::Vector3d> vec_target,
                                       std::vector<Eigen::Vector3d> vec_normal) :   //target-prev
        max_iter_(max_iterations), initial_guess_(initial_guess),
        vec_src_(std::move(vec_src)), vec_target_(std::move(vec_target)), vec_normal_(std::move(vec_normal)),
        final_rot_(Eigen::Matrix3d::Identity()), final_trans_(Eigen::Matrix<double, 3, 1>::Zero()) {

    vec_src_filtered_.resize(vec_src_.size());
    vec_corres4src_.resize(vec_src_.size());
    Eigen::Matrix3d rotation_mat;
    rotation_mat = initial_guess_.block<3, 3>(0, 0);
    Eigen::Vector3d euler_angle;
    euler_angle = rotation_mat.eulerAngles(2, 1, 0);

    initial_guess_array_[0] = euler_angle(0);
    initial_guess_array_[1] = euler_angle(1);
    initial_guess_array_[2] = euler_angle(2);
    initial_guess_array_[3] = initial_guess_(0, 3);
    initial_guess_array_[4] = initial_guess_(1, 3);
    initial_guess_array_[5] = initial_guess_(2, 3);

}


void RegistrationFrames::get_closestneighbor() {

    const int k_num = 1;
    Eigen::Vector3d src_pt;
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>, double> mykdtree_type;

    mykdtree_type cloud_index(3, vec_target_, 10);
    cloud_index.index->buildIndex();

    std::vector<double> query_pt(3);
    double max_dist_sqr = 0;
    count_ = 0;

    for (auto &i : vec_src_) {

        std::vector<unsigned long> tmp_index(1);
        std::vector<double> tmp_dist_sqr(1);

        query_pt[0] = i[0];
        query_pt[1] = i[1];
        query_pt[2] = i[2];

        nanoflann::KNNResultSet<double> results(k_num);
        results.init(&tmp_index[0], &tmp_dist_sqr[0]);
        cloud_index.index->findNeighbors(results, &query_pt[0], nanoflann::SearchParams(10));

        if (max_dist_sqr < tmp_dist_sqr[0]) {
            max_dist_sqr = tmp_dist_sqr[0];
        }
        if (tmp_dist_sqr[0] < 25) {  //2帧间匹配：点对之间距离小于5m，若大于就不考虑，排除噪声点异常值
            vec_src_filtered_[count_] = i;
            vec_corres4src_[count_] = (vec_target_[tmp_index[0]]);
            ++count_;
        }
        indices_.push_back(tmp_index[0]);

    }
    //std::cout<< "Max dist of point pairs: " <<sqrt(max_dist_sqr) <<" m" <<std::endl;
}

void RegistrationFrames::solve_transform() {

    ceres::Problem problem;
    for (int i = 0; i < count_; ++i) {
        ceres::CostFunction *cost_function = IcpCostFunctor::Create(vec_corres4src_[i],
                                                                    vec_src_filtered_[i],
                                                                    vec_normal_[indices_[i]]);
        problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.3), initial_guess_array_);
    }


    ceres::Solver::Options options;
    options.num_threads = 8;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    //options.initial_trust_region_radius = 10e5;
    //options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);


    Eigen::Matrix<double, 3, 3> rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(initial_guess_array_[0], Eigen::Vector3d::UnitX()) *    //rotation
                      Eigen::AngleAxisd(initial_guess_array_[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(initial_guess_array_[2], Eigen::Vector3d::UnitZ());

    Eigen::Vector3d translation_vec;
    translation_vec << initial_guess_array_[3], initial_guess_array_[4], initial_guess_array_[5];
    if (initial_guess_array_[3] < 0.001 && initial_guess_array_[4] < 0.001 && initial_guess_array_[5] < 0.001) {
        if (initial_guess_array_[0] < 0.0002 && initial_guess_array_[1] < 0.0002 &&
            initial_guess_array_[2] < 0.0002)  //~deg:0.00115
            fulfill_tole_flag_ = true;
    }
    final_rot_ *= rotation_matrix;
    final_trans_ += translation_vec;

    for (auto &i : vec_src_) {
        i = rotation_matrix * i;
        i += translation_vec;
    }

    ///TODO: check plz!!
    for (auto &i : vec_normal_) {
        i = rotation_matrix * i;
        i += translation_vec;
    }


    //std::cout<< "[   ------ pose "<<iter_num_<<" -----  ]" << "\n";
    //std::cout<< translation_vec << "\n";
    //std::cout<< "roll pitch yaw (deg): "
    //                        << initial_guess_[0] /M_PI * 180 <<" "
    //                        << initial_guess_[1] /M_PI * 180 <<" "
    //                        << initial_guess_[2] /M_PI * 180 <<"\n";
    //std::cout<< rotation_matrix << "\n";
    //std::cout<< summary.FullReport()<<"\n";
}


Eigen::Matrix4d RegistrationFrames::get_current_pose() {
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
    res.block<3,3>(0,0) = final_rot_;
    res.block<3,1>(0,3) = final_trans_;
    return res;

}

std::vector<Eigen::Vector3d> RegistrationFrames::get_transformedfpts_vec() {
    return vec_src_;
}

std::vector<Eigen::Vector3d> RegistrationFrames::get_transformedfpts_nor() {
    return vec_normal_;
}

Eigen::Matrix3d RegistrationFrames::get_rotation() {
    return final_rot_;
}

Eigen::Vector3d RegistrationFrames::get_translation() {
    return final_trans_;
}

void RegistrationFrames::get_pose() {
    while (iter_num_ < max_iter_ && !fulfill_tole_flag_) {
        ++iter_num_;
        //pcl::console::TicToc tt;
        //pcl::console::TicToc tt2;
        //tt.tic();
        //tt2.tic();
        get_closestneighbor();
        //std::cout<<"[Time for Association: " <<tt.toc() << "ms]" <<"\n";
        //tt2.tic();
        solve_transform();
        //std::cout<<"[Time for Transform: " <<tt2.toc() << "ms]" <<"\n";
    }

    //std::cout<< "[----total iter_num    "<< iter_num_ << "    ----]"<< "\n";
    //std::cout<< "final rot: " << final_rot_ <<"\n";
    //std::cout<< "final trans (m): " << final_trans_ <<"\n";

}

PointCloudTI::Ptr RegistrationFrames::get_transformedcloud(const PointCloudTI::Ptr &cloud_in) {
    PointCloudTI::Ptr cloud_out(new PointCloudTI);
    cloud_out->resize(cloud_in->size());
    for (int i = 0; i < cloud_in->points.size(); ++i) {
        cloud_out->points[i].x = final_rot_(0, 0) * cloud_in->points[i].x + final_rot_(0, 1) * cloud_in->points[i].y +
                                 final_rot_(0, 2) * cloud_in->points[i].z + final_trans_[0];
        cloud_out->points[i].y = final_rot_(1, 0) * cloud_in->points[i].x + final_rot_(1, 1) * cloud_in->points[i].y +
                                 final_rot_(1, 2) * cloud_in->points[i].z + final_trans_[1];
        cloud_out->points[i].z = final_rot_(2, 0) * cloud_in->points[i].x + final_rot_(2, 1) * cloud_in->points[i].y +
                                 final_rot_(2, 2) * cloud_in->points[i].z + final_trans_[2];

    }
    return cloud_out;
}

void RegistrationFrames::view_registration(const std::vector<Eigen::Vector3d> &src_pts,
                                           const PointCloudTI::Ptr &cloud_target) {


    PointCloudTI::Ptr src(new PointCloudTI);
    src->resize(src_pts.size());

    for (int i = 0; i < src_pts.size(); ++i) {
        src->points[i].x =
                final_rot_(0, 0) * src_pts[i][0] + final_rot_(0, 1) * src_pts[i][1] + final_rot_(0, 2) * src_pts[i][2] +
                final_trans_[0];
        src->points[i].y =
                final_rot_(1, 0) * src_pts[i][0] + final_rot_(1, 1) * src_pts[i][1] + final_rot_(1, 2) * src_pts[i][2] +
                final_trans_[1];
        src->points[i].z =
                final_rot_(2, 0) * src_pts[i][0] + final_rot_(2, 1) * src_pts[i][1] + final_rot_(2, 2) * src_pts[i][2] +
                final_trans_[2];

    }
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("compare Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud_target, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud_target, color, "target cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color2(src, 220, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI>(src, color2, "src cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "src cloud");
    viewer->addCoordinateSystem(3.0);
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }

    viewer->close();

}
