
#include "lidar_odometry/LidarOdometry.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "common/utils.h"
#include <pcl/console/time.h>


LidarOdometry::LidarOdometry() {

    kdtree_edge_last_.reset(new pcl::KdTreeFLANN<PointTI>());
    kdtree_ground_last_.reset(new pcl::KdTreeFLANN<PointTI>());
    kdtree_surf_last_.reset(new pcl::KdTreeFLANN<PointTI>());
    search_id_.clear();
    search_sqdist_.clear();

    surf_current_.reset(new PointCloudTI);
    surf_last_.reset(new PointCloudTI);
    ground_current_.reset(new PointCloudTI);
    ground_last_.reset(new PointCloudTI);
    edge_current_.reset(new PointCloudTI);
    edge_last_.reset(new PointCloudTI);

    edge_count_ = 0;

    initial_guess_array_ = new double[6];
    for (int i = 0; i < 6; i++) {
        initial_guess_array_[i] = 0;
    }

    opt_array_ = new double[3];
    for (int i = 0; i < 3; i++) {
        opt_array_[i] = 0;
    }

    final_transform_ = Eigen::Matrix<double,4,4>::Identity();
    fulfill_tole_flag_ = false;


    initial_pose_ = Eigen::Matrix4d::Identity();

}


void LidarOdometry::setParameters(const Eigen::Matrix4d &initial_guess,
                                  const PointCloudTI::Ptr &surf_current, const PointCloudTI::Ptr &surf_last,
                                  const PointCloudTI::Ptr &edge_current, const PointCloudTI::Ptr &edge_last,
                                  const PointCloudTI::Ptr &ground_current, const PointCloudTI::Ptr &ground_last) {

    Eigen::Matrix3d rotation_mat;
    rotation_mat = initial_guess.block<3, 3>(0, 0);
    Eigen::Vector3d euler_angle;
    euler_angle = rotation_mat.eulerAngles(2, 1, 0);
    initial_pose_ = initial_guess;
    //ZYX order:  euler_angle 0-2 yaw pitch roll

    surf_current_ = surf_current;
    surf_last_ = surf_last;
    ground_current_ = ground_current;
    ground_last_ = ground_last;
    edge_current_ = edge_current;
    edge_last_ = edge_last;

    // intialguess array: 0-2 roll pitch yaw
    initial_guess_array_[0] = euler_angle(2);   // ZYX顺序，euler angle 0-2 yaw,pitch,roll
    initial_guess_array_[1] = euler_angle(1);
    initial_guess_array_[2] = euler_angle(0);
    initial_guess_array_[3] = initial_guess(0, 3);
    initial_guess_array_[4] = initial_guess(1, 3);
    initial_guess_array_[5] = initial_guess(2, 3);

    std::cout << "[******initial_guess at first******] : \n" <<
    "roll: " << initial_guess_array_[0] <<" " << "pitch: "<<initial_guess_array_[1] <<" " <<
    "yaw: " << initial_guess_array_[2] << " " << "x: " << initial_guess_array_[3] <<" " <<
    "y: " << initial_guess_array_[4] <<" " << "z: "<< initial_guess_array_[5] <<std::endl;

    //final_rot_ = initial_guess.block<3,3>(0,0);  ///
    //final_trans_ = initial_guess.block<3,1>(0,3);  ///
}


Eigen::Matrix4d LidarOdometry::poseSolver(const std::string &method) {
    //poseCalculation1 : solve 6d pose all together
    //poseCalculation2 : solve 3d+3d pose ground first and then edge
    method_ = method;
    if (method_ == "ff_6d") {
        return poseCalculation1();
    }
    //if (method_ == "ff_3d+3d") {
    return poseCalculation2();
}


Eigen::Matrix4d LidarOdometry::poseCalculation1() {

    //kdtree_edge_last_->setInputCloud(edge_last_);
    kdtree_ground_last_->setInputCloud(ground_last_);
    kdtree_surf_last_->setInputCloud(surf_last_);
    final_transform_ = initial_pose_;
    int i = 0;
    while (i < max_iter_ && !fulfill_tole_flag_) {
        ++i;
        findGroundCorrespondence(ground_current_, ground_last_, "Ground");
        findGroundCorrespondence(surf_current_, surf_last_, "Surf");
        //findEdgeCorrespondence();
        solveTransform();
    }

    Eigen::Matrix3d rotation_mat;
    rotation_mat = final_transform_.block<3, 3>(0, 0);
    Eigen::Vector3d euler_angle;
    euler_angle = rotation_mat.eulerAngles(2, 1, 0);
    std::cout << "[******final_transform_ in the end******] : \n" <<
              "roll: " << euler_angle[2] <<" " << "pitch: "<<euler_angle[1] <<" " <<
              "yaw: " << euler_angle[0] << " " << "x: " << final_transform_(0,3) <<" " <<
              "y: " << final_transform_(1,3) <<" " << "z: "<< final_transform_(2,3) <<std::endl;

    return final_transform_;

}


Eigen::Matrix4d LidarOdometry::poseCalculation2() {
    kdtree_edge_last_->setInputCloud(edge_last_);
    kdtree_surf_last_->setInputCloud(ground_last_);
    int i = 0;
    while (i < max_iter_ && !fulfill_tole_flag_) {
        ++i;
        findGroundCorrespondence(ground_current_, ground_last_, "Ground");
        solveTransform();
    }
    /*
    roll_pre_ = initial_guess_array_[0];   //只是每一个小循环的解
    pitch_pre_ = initial_guess_array_[1];
    y_pre_ = initial_guess_array_[4];
    z_pre_ = initial_guess_array_[5];
    params_pre_ << roll_pre_, pitch_pre_, z_pre_;
     */
    // ZYX顺序，yaw,pitch,roll
    Eigen::Vector3d euler = final_transform_.block<3,3>(0,0).eulerAngles(2,1,0);
    params_pre_ << euler[2], euler[1], final_transform_(2,3);
    std::cout<< "params_pre: \n" << params_pre_ << std::endl;
    std::cout<< "final_transform_ : \n" << final_transform_ << std::endl;

    i = 0;
    while (i < max_iter_ && !fulfill_tole_flag_) {
        ++i;
        findEdgeCorrespondence();
        solveTransformOpt();
    }

    std::cout<< "final_transform_ : \n" << final_transform_ << std::endl;

    Eigen::Matrix3d rotation_mat;
    rotation_mat = final_transform_.block<3, 3>(0, 0);
    Eigen::Vector3d euler_angle;
    euler_angle = rotation_mat.eulerAngles(2, 1, 0);
    std::cout << "[******final_transform_ in the end******] : \n" <<
              "roll: " << euler_angle[2] <<" " << "pitch: "<<euler_angle[1] <<" " <<
              "yaw: " << euler_angle[0] << " " << "x: " << final_transform_(0,3) <<" " <<
              "y: " << final_transform_(1,3) <<" " << "z: "<< final_transform_(2,3) <<std::endl;

    return final_transform_;
}


void LidarOdometry::findEdgeCorrespondence() {

    // use the egde less sharp cloud of last frame to build the kdtree
    // and then find the nearest neighbor of current edge_sharp frame
    //kdtree_edge_last_->setInputCloud(edge_last_);
    int count = 0;
    PointCloudTI::Ptr transformed_current(new PointCloudTI);
    pcl::transformPointCloud(*edge_current_, *transformed_current, final_transform_);
    for (int i = 0; i < edge_current_->size(); i++) {

        kdtree_edge_last_->nearestKSearch(transformed_current->points[i], 1, search_id_, search_sqdist_);
        int closestPoint_id = -1;
        int minPoint_id2 = -1;

        if (search_sqdist_[0] < 25) {
            //find the first point
            closestPoint_id = search_id_[0];
            int closestPoint_beam = int(edge_last_->points[closestPoint_id].intensity);

            double point_sqr_dist, minPoint_sqdist2 = 25;
            for (int j = closestPoint_id + 1; j < edge_last_->size(); j++) {
                if (int(edge_last_->points[j].intensity) > closestPoint_beam + 1.5) {
                    break;
                }
                if (int(edge_last_->points[j].intensity) > closestPoint_beam) {
                    point_sqr_dist = (edge_last_->points[j].x - edge_last_->points[closestPoint_id].x) *
                                     (edge_last_->points[j].x - edge_last_->points[closestPoint_id].x) +
                                     (edge_last_->points[j].y - edge_last_->points[closestPoint_id].y) *
                                     (edge_last_->points[j].y - edge_last_->points[closestPoint_id].y) +
                                     (edge_last_->points[j].z - edge_last_->points[closestPoint_id].z) *
                                     (edge_last_->points[j].z - edge_last_->points[closestPoint_id].z);

                    if (point_sqr_dist < minPoint_sqdist2) {
                        minPoint_sqdist2 = point_sqr_dist;
                        minPoint_id2 = j;
                    }
                }
            }
            for (int j = closestPoint_id - 1; j >= 0; j--) {
                if (int(edge_last_->points[j].intensity) < closestPoint_beam - 1.5) {
                    break;
                }
                if (int(edge_last_->points[j].intensity) > closestPoint_beam) {
                    point_sqr_dist = (edge_last_->points[j].x - edge_last_->points[closestPoint_id].x) *
                                     (edge_last_->points[j].x - edge_last_->points[closestPoint_id].x) +
                                     (edge_last_->points[j].y - edge_last_->points[closestPoint_id].y) *
                                     (edge_last_->points[j].y - edge_last_->points[closestPoint_id].y) +
                                     (edge_last_->points[j].z - edge_last_->points[closestPoint_id].z) *
                                     (edge_last_->points[j].z - edge_last_->points[closestPoint_id].z);

                    if (point_sqr_dist < minPoint_sqdist2) {
                        minPoint_sqdist2 = point_sqr_dist;
                        minPoint_id2 = j;
                    }
                }
            }
        }

        if (minPoint_id2 >= 0) {
            Eigen::Vector3d edge_sel(edge_current_->points[i].x,
                                     edge_current_->points[i].y,
                                     edge_current_->points[i].z);
            Eigen::Vector3d tripod1(edge_last_->points[closestPoint_id].x,
                                    edge_last_->points[closestPoint_id].y,
                                    edge_last_->points[closestPoint_id].z);
            Eigen::Vector3d tripod2(edge_last_->points[minPoint_id2].x,
                                    edge_last_->points[minPoint_id2].y,
                                    edge_last_->points[minPoint_id2].z);

            if(method_ == "ff_6d"){
                ceres::CostFunction *cost_function = EdgeFunctor::Create(edge_sel, tripod1, tripod2);
                problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), initial_guess_array_);
            }
            if(method_ == "ff_3d+3d"){
                //opt_array_ : yaw, x, y
                ceres::CostFunction *cost_function = EdgeFunctor2::Create(edge_sel, tripod1, tripod2, params_pre_);
                /// add init : should be zero as initial status??!!
                /*
                opt_array_[0] = initial_guess_array_[2];
                opt_array_[1] = initial_guess_array_[3];
                opt_array_[2] = initial_guess_array_[4];
                 */
                problem_opt.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), opt_array_);
            }
            count++;
        }
    }
    if (count < 10) {
        std::cout << " too few edge feature point!" << std::endl;
    }
}

/*
void LidarOdometry::findSurfCorrespondence() {

    for (int i = 0; i < surf_current_->size(); i++) {

        kdtree_surf_last_->nearestKSearch(surf_current_->points[i], 1, search_id_, search_sqdist_);
        int closestPoint_id = -1;
        int minPoint_id2 = -1;
        int minPoint_id3 = -1;

        if (search_sqdist_[0] < 25) {
            //find the first point
            closestPoint_id = search_id_[0];
            int closestPoint_beam = int(surf_last_->points[closestPoint_id].intensity);

            double point_sqr_dist, minPoint_sqdist2 = 25, minPoint_sqdist3 = 25;
            for (int j = closestPoint_id + 1; j < surf_last_->size(); j++) {
                if (int(surf_last_->points[j].intensity) > closestPoint_beam + 1.5) {
                    break;
                }

                point_sqr_dist = (surf_last_->points[j].x - surf_last_->points[closestPoint_id].x) *
                                 (surf_last_->points[j].x - surf_last_->points[closestPoint_id].x) +
                                 (surf_last_->points[j].y - surf_last_->points[closestPoint_id].y) *
                                 (surf_last_->points[j].y - surf_last_->points[closestPoint_id].y) +
                                 (surf_last_->points[j].z - surf_last_->points[closestPoint_id].z) *
                                 (surf_last_->points[j].z - surf_last_->points[closestPoint_id].z);
                if (int(surf_last_->points[j].intensity) <= closestPoint_beam) {   //only = occurs
                    if (point_sqr_dist < minPoint_sqdist2) {
                        minPoint_sqdist2 = point_sqr_dist;
                        minPoint_id2 = j;
                    }
                } else {
                    if (point_sqr_dist < minPoint_sqdist3) {
                        minPoint_sqdist3 = point_sqr_dist;
                        minPoint_id3 = j;
                    }
                }
            }
            for (int j = closestPoint_id - 1; j >= 0; j--) {
                if (int(surf_last_->points[j].intensity) < closestPoint_beam - 1.5) {
                    break;
                }
                point_sqr_dist = (surf_last_->points[j].x - surf_last_->points[closestPoint_id].x) *
                                 (surf_last_->points[j].x - surf_last_->points[closestPoint_id].x) +
                                 (surf_last_->points[j].y - surf_last_->points[closestPoint_id].y) *
                                 (surf_last_->points[j].y - surf_last_->points[closestPoint_id].y) +
                                 (surf_last_->points[j].z - surf_last_->points[closestPoint_id].z) *
                                 (surf_last_->points[j].z - surf_last_->points[closestPoint_id].z);
                if (int(surf_last_->points[j].intensity) >= closestPoint_beam) {  //only = occurs
                    if (point_sqr_dist < minPoint_sqdist2) {
                        minPoint_sqdist2 = point_sqr_dist;
                        minPoint_id2 = j;
                    }
                } else {
                    if (point_sqr_dist < minPoint_sqdist3) {
                        minPoint_sqdist3 = point_sqr_dist;
                        minPoint_id3 = j;
                    }
                }
            }
        }

        if (minPoint_id2 >= 0 && minPoint_id3 >= 0) {
            Eigen::Vector3d surf_sel(surf_current_->points[i].x,
                                     surf_current_->points[i].y,
                                     surf_current_->points[i].z);
            Eigen::Vector3d tripod1(surf_last_->points[closestPoint_id].x,
                                    surf_last_->points[closestPoint_id].y,
                                    surf_last_->points[closestPoint_id].z);
            Eigen::Vector3d tripod2(surf_last_->points[minPoint_id2].x,
                                    surf_last_->points[minPoint_id2].y,
                                    surf_last_->points[minPoint_id2].z);
            Eigen::Vector3d tripod3(surf_last_->points[minPoint_id3].x,
                                    surf_last_->points[minPoint_id3].y,
                                    surf_last_->points[minPoint_id3].z);
            ceres::CostFunction *cost_function = SurfFunctor::Create(surf_sel, tripod1, tripod2, tripod3);
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), initial_guess_array_);
            surf_count_++;
        }
    }
    if (surf_count_ < 10) {
        std::cout << " too few surf feature point!" << std::endl;
    }

}
*/


void LidarOdometry::findGroundCorrespondence(const PointCloudTI::Ptr& current_cloud,
                                             const PointCloudTI::Ptr& last_cloud,
                                             const std::string surf_type) {

    //kdtree_surf_last_->setInputCloud(ground_last_);
    int count = 0;
    PointCloudTI::Ptr transformed_current(new PointCloudTI);
    pcl::transformPointCloud(*current_cloud, *transformed_current, final_transform_);
    for (int i = 0; i < current_cloud->size(); i++) {

        search_sqdist_.clear();
        search_id_.clear();
        if(surf_type == "Ground") {
            kdtree_ground_last_->nearestKSearch(transformed_current->points[i], 1, search_id_, search_sqdist_);
        }
        if(surf_type == "Surf") {
            kdtree_surf_last_->nearestKSearch(transformed_current->points[i], 1, search_id_, search_sqdist_);
        }
        int closestPoint_id = -1;
        int minPoint_id2 = -1;
        int minPoint_id3 = -1;

        if (search_sqdist_[0] < 25) {
            //find the first point
            closestPoint_id = search_id_[0];
            int closestPoint_beam = int(last_cloud->points[closestPoint_id].intensity);

            double point_sqr_dist, minPoint_sqdist2 = 25, minPoint_sqdist3 = 25;
            for (int j = closestPoint_id + 1; j < last_cloud->size(); j++) {
                if (int(last_cloud->points[j].intensity) > closestPoint_beam + 1.5) {
                    break;
                }

                point_sqr_dist = (last_cloud->points[j].x - last_cloud->points[closestPoint_id].x) *
                                 (last_cloud->points[j].x - last_cloud->points[closestPoint_id].x) +
                                 (last_cloud->points[j].y - last_cloud->points[closestPoint_id].y) *
                                 (last_cloud->points[j].y - last_cloud->points[closestPoint_id].y) +
                                 (last_cloud->points[j].z - last_cloud->points[closestPoint_id].z) *
                                 (last_cloud->points[j].z - last_cloud->points[closestPoint_id].z);
                if (int(last_cloud->points[j].intensity) <= closestPoint_beam) {   //only = occurs
                    if (point_sqr_dist < minPoint_sqdist2) {
                        minPoint_sqdist2 = point_sqr_dist;
                        minPoint_id2 = j;
                    }
                } else {
                    if (point_sqr_dist < minPoint_sqdist3) {
                        minPoint_sqdist3 = point_sqr_dist;
                        minPoint_id3 = j;
                    }
                }
            }
            for (int j = closestPoint_id - 1; j >= 0; j--) {
                if (int(last_cloud->points[j].intensity) < closestPoint_beam - 1.5) {
                    break;
                }
                point_sqr_dist = (last_cloud->points[j].x - last_cloud->points[closestPoint_id].x) *
                                 (last_cloud->points[j].x - last_cloud->points[closestPoint_id].x) +
                                 (last_cloud->points[j].y - last_cloud->points[closestPoint_id].y) *
                                 (last_cloud->points[j].y - last_cloud->points[closestPoint_id].y) +
                                 (last_cloud->points[j].z - last_cloud->points[closestPoint_id].z) *
                                 (last_cloud->points[j].z - last_cloud->points[closestPoint_id].z);
                if (int(last_cloud->points[j].intensity) >= closestPoint_beam) {  //only = occurs
                    if (point_sqr_dist < minPoint_sqdist2) {
                        minPoint_sqdist2 = point_sqr_dist;
                        minPoint_id2 = j;
                    }
                } else {
                    if (point_sqr_dist < minPoint_sqdist3) {
                        minPoint_sqdist3 = point_sqr_dist;
                        minPoint_id3 = j;
                    }
                }
            }
        }

        if (minPoint_id2 >= 0 && minPoint_id3 >= 0) {
            Eigen::Vector3d surf_sel(current_cloud->points[i].x,
                                     current_cloud->points[i].y,
                                     current_cloud->points[i].z);
            Eigen::Vector3d tripod1(last_cloud->points[closestPoint_id].x,
                                    last_cloud->points[closestPoint_id].y,
                                    last_cloud->points[closestPoint_id].z);
            Eigen::Vector3d tripod2(last_cloud->points[minPoint_id2].x,
                                    last_cloud->points[minPoint_id2].y,
                                    last_cloud->points[minPoint_id2].z);
            Eigen::Vector3d tripod3(last_cloud->points[minPoint_id3].x,
                                    last_cloud->points[minPoint_id3].y,
                                    last_cloud->points[minPoint_id3].z);
            ceres::CostFunction *cost_function;
            //if(surf_type == "Ground") {
            cost_function = GroundFunctor::Create(surf_sel, tripod1, tripod2, tripod3);
            //}else if(surf_type == "Surf"){
            //    cost_function = SurfFunctor::Create(surf_sel, tripod1, tripod2, tripod3);
            //}
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), initial_guess_array_);  //TODO应该改大
            count++;
        }
    }
    if (count < 10) {
        std::cout << " too few ground/surf feature point!" << std::endl;
    }

}

void LidarOdometry::solveTransform() {

    ceres::Solver::Options options;
    options.num_threads = 8;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.initial_trust_region_radius = 10e5;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 6;
    options.max_line_search_step_contraction = 1e-2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);


    Eigen::Matrix<double, 3, 3> rotation_matrix;

    rotation_matrix = Eigen::AngleAxisd(initial_guess_array_[2], Eigen::Vector3d::UnitZ()) *    //rotation
                      Eigen::AngleAxisd(initial_guess_array_[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(initial_guess_array_[0], Eigen::Vector3d::UnitX());

    Eigen::Vector3d translation_vec;
    translation_vec << initial_guess_array_[3], initial_guess_array_[4], initial_guess_array_[5];
    if (initial_guess_array_[3] < 0.001 && initial_guess_array_[4] < 0.001 && initial_guess_array_[5] < 0.001) {
        if (initial_guess_array_[0] < 0.0002 && initial_guess_array_[1] < 0.0002 &&
            initial_guess_array_[2] < 0.0002)  //~deg:0.00115
            fulfill_tole_flag_ = true;
    }

    Eigen::Matrix<double,4,4> tmp = Eigen::Matrix4d::Identity();;
    tmp.block<3,3>(0,0) = rotation_matrix;
    tmp.block<3,1>(0,3) = translation_vec;
    final_transform_ = tmp /* final_transform_*/;  //TODO

    ///add initialization
    for (int i = 0; i < 6; i++) {
        initial_guess_array_[i] = 0;
    }

}


void LidarOdometry::solveTransformOpt() {

    ceres::Solver::Options options;
    options.num_threads = 8;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.initial_trust_region_radius = 10e5;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 6;
    options.max_line_search_step_contraction = 1e-2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_opt, &summary);


    Eigen::Matrix<double, 3, 3> rotation_matrix;
    //// ZYX顺序，0-2 yaw,pitch,roll
    rotation_matrix = Eigen::AngleAxisd(opt_array_[0], Eigen::Vector3d::UnitZ()) *    //rotation
                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    std::cout<< "rotation matrix in solveTransformOpt \n" << rotation_matrix << std::endl;

    Eigen::Vector3d translation_vec;
    //pose -> opt_array_ : yaw, x, y
    translation_vec << opt_array_[1], opt_array_[2]/**0.7 + y_pre_*0.3*/, 0;
    if (opt_array_[1] < 0.0001 && opt_array_[2] < 0.0001) {
        if (opt_array_[0] < 0.00007)  //~deg:0.00115
            fulfill_tole_flag_ = true;
    }

    Eigen::Matrix<double,4,4> tmp = Eigen::Matrix4d::Identity();
    tmp.block<3,3>(0,0) = rotation_matrix;
    tmp.block<3,1>(0,3) = translation_vec;
    std::cout << "tmp_matrix<4,4> in solveProblemOpt: \n" << tmp<< std::endl;
    final_transform_ = tmp * final_transform_;
    std::cout << "final_transform_<4,4> in solveProblemOpt: \n" << final_transform_<< std::endl;

    //pcl::transformPointCloud(*surf_current_, *surf_current_, transform_matrix);
    pcl::transformPointCloud(*ground_current_, *ground_current_, tmp);
    pcl::transformPointCloud(*edge_current_, *edge_current_, tmp);

    ///add init

    for (int i = 0; i < 3; i++) {
        opt_array_[i] = 0;
    }



}
