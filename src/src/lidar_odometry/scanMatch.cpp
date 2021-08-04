#include "lidar_odometry/scanMatch.h"
#include "common/utils.h"
#include <Eigen/Eigenvalues>

ScanMatch::ScanMatch() {
    std::cout << "run the constructor --" <<std::endl;
    grd_target_.reset(new PointCloudT);
    edge_target_.reset(new PointCloudT);
    kdtree_g2m_.reset(new pcl::KdTreeFLANN <PointT>);
    kdtree_e2m_.reset(new pcl::KdTreeFLANN <PointT>);
    input_grd_.reset(new PointCloudT);
    input_edge_.reset(new PointCloudT);
    fulfill_tole_flag_ = false;

    pose_array_ = new double[6];
    for (int i = 0; i < 6; i++) {
        pose_array_[i] = 0;
    }
}

void ScanMatch::setTargetCloud(const PointCloudT::Ptr &grd_target, const PointCloudT::Ptr &edge_target) {
    std::cout << "&&&&&&&&&&&&&&&&&checkaa00004" <<std::endl;
    grd_target_ = grd_target;
    std::cout << "&&&&&&&&&&&&&&&&&checkaa04a" <<std::endl;
    edge_target_ = edge_target;
}

void ScanMatch::frame2mapRegistration(const PointCloudT::Ptr &input_grd, const PointCloudT::Ptr &input_edge,
                                      const Eigen::Matrix4d& predict_pose,
                                      PointCloudT::Ptr &result_grd, PointCloudT::Ptr &resutlt_edge,
                                      Eigen::Matrix4d& result_pose_grd,  Eigen::Matrix4d& result_pose_edge) {
    ///initial guess could be imu/odometry/motion assumption...
    final_transform_ = predict_pose;

    input_grd_ = input_grd;
    input_edge_ = input_edge;

    Eigen::Matrix3d rotation_mat;
    rotation_mat = final_transform_.block<3, 3>(0, 0);
    Eigen::Vector3d euler_angle;
    euler_angle = rotation_mat.eulerAngles(2, 1, 0);
    pose_array_[0] = euler_angle[2];
    pose_array_[1] = euler_angle[1];
    pose_array_[2] = euler_angle[0];
    pose_array_[3] = final_transform_(0,3);
    pose_array_[4] = final_transform_(1,3);
    pose_array_[5] = final_transform_(2,3);

    int i = 0;
    while (i < max_iter_ && !fulfill_tole_flag_) {
        ++i;
        findEdgeCorrespondence();
        findGroundCorrespondence();
        std::cout<<i<<". iter: before"<<pose_array_[0] <<" "<<pose_array_[1] <<" "<<pose_array_[2]<<" "
          << pose_array_[3] << " " << pose_array_[4] << " "<< pose_array_[5]<<std::endl;
        solveProblem();
        std::cout<<i<<". iter: after"<<*pose_array_ <<std::endl;
    }

    result_pose_grd =  final_transform_;
    result_pose_edge =  final_transform_;


    //reset parameters
    grd_target_.reset(new PointCloudT);
    edge_target_.reset(new PointCloudT);
    kdtree_g2m_.reset(new pcl::KdTreeFLANN <PointT>);
    kdtree_e2m_.reset(new pcl::KdTreeFLANN <PointT>);
    input_grd_.reset(new PointCloudT);
    input_edge_.reset(new PointCloudT);
    fulfill_tole_flag_ = false;

    pose_array_ = new double[6];
    for (int i = 0; i < 6; i++) {
        pose_array_[i] = 0;
    }

}


void ScanMatch::findGroundCorrespondence() {

    int valid_plane_count = 0;
    kdtree_g2m_->setInputCloud(grd_target_);
    for (int i = 0; i < input_grd_->size(); i++) {

        std::vector<float> search_sqdist;
        std::vector<int> search_id;

        //要把当前点转到世界坐标系下 再跟map寻找最近邻. 假定input已经经过了坐标转换
        //使用target建立kdtree，搜索input当前关键帧的近邻点
        PointT point_sel = input_grd_->points[i];
        kdtree_g2m_->nearestKSearch(point_sel, 5, search_id, search_sqdist);


        Eigen::Matrix<double, 5, 3> mat_points;   //A
        Eigen::Matrix<double, 5, 1> mat_r;   //b
        mat_r.fill(-1);
        Eigen::Matrix<double, 3, 1> mat_x0;

        if (search_sqdist[4] < 2.0) {
            //find the first point
            for (int j = 0; j < 5; j++) {
                mat_points(j, 0) = grd_target_->points[search_id[j]].x;
                mat_points(j, 1) = grd_target_->points[search_id[j]].y;
                mat_points(j, 2) = grd_target_->points[search_id[j]].z;
            }

            mat_x0 = mat_points.colPivHouseholderQr().solve(mat_r);

            // 法向量参数 ax+by+cz +d = 0
            double pa = mat_x0(0, 0);
            double pb = mat_x0(1, 0);
            double pc = mat_x0(2, 0);
            double pd = 1;

            double ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * grd_target_->points[search_id[j]].x +
                         pb * grd_target_->points[search_id[j]].y +
                         pc * grd_target_->points[search_id[j]].z + pd) > 0.2) {  //0.2m

                    planeValid = false;
                    break;
                }
            }

            // 是有效的平面
            if (planeValid) {
                valid_plane_count++;
                //double pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;
                ceres::CostFunction *cost_function = GroundFunctor::Create(point_sel, pa, pb, pc, pd);
                problem_.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), pose_array_);
            }

        }
        if (input_grd_->size() < 10) {
            std::cout << " too few ground feature point!" << std::endl;
        }
    }
    std::cout << "valid_plane_count: " << valid_plane_count << std::endl;
}

void ScanMatch::findEdgeCorrespondence() {
    int valid_edge_count = 0;
    kdtree_e2m_->setInputCloud(edge_target_);
    for (int i = 0; i < input_edge_->size(); i++) {

        std::vector<float> search_sqdist;
        std::vector<int> search_id;

        //要把当前点转到世界坐标系下 再跟map寻找最近邻. 假定input已经经过了坐标转换
        //使用target建立kdtree，搜索input当前关键帧的近邻点
        PointT point_sel = input_edge_->points[i];
        kdtree_e2m_->nearestKSearch(point_sel, 5, search_id, search_sqdist);

        Eigen::Matrix<double, 3, 3> mat_a = Eigen::Matrix3d::Zero();   //A  covariance matrix
        Eigen::Vector3d mat_d = Eigen::Vector3d::Zero();  //d   eigen value
        Eigen::Matrix<double, 3, 3> mat_v = Eigen::Matrix3d::Zero();   //v  eigen vector
        Eigen::Matrix<double, 3, 5> mat_points;

        if (search_sqdist[4] < 2.0) {
            //find the point on the average

            double avr_x = 0, avr_y = 0, avr_z = 0;
            for (int j = 0; j < 5; j++) {
                avr_x += edge_target_->points[search_id[j]].x;
                avr_y += edge_target_->points[search_id[j]].y;
                avr_z += edge_target_->points[search_id[j]].z;
                mat_points.block<3, 1>(0, j) << edge_target_->points[search_id[j]].x,
                        edge_target_->points[search_id[j]].y,
                        edge_target_->points[search_id[j]].z;
            }

            avr_x /= 5;  //expectation
            avr_y /= 5;
            avr_z /= 5;
            Eigen::Matrix<double, 3, 1> mat_avr = {avr_x, avr_y, avr_z};
            mat_points.colwise() -= mat_avr;

            mat_a = mat_points * mat_points.transpose() / 5;  //covariance matrix 3*3
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> sol(mat_a);
            mat_d = sol.eigenvalues(); //eigen value in a matrix  original = v * d * v^(-1)
            //eigen value : small -> large
            mat_v = sol.eigenvectors();  //eigen vector : colwise


            if (mat_d(2, 0) > 3 * mat_d(1, 0)) {
                valid_edge_count++;

                //局部map对应中心角点，沿着特征向量（直线方向）方向前后各取一个点
                double x1 = avr_x + 0.1 * mat_v(0, 2);
                double y1 = avr_y + 0.1 * mat_v(1, 2);
                double z1 = avr_z + 0.1 * mat_v(2, 2);
                Eigen::Vector3d point1 = {x1, y1, z1};
                double x2 = avr_x - 0.1 * mat_v(0, 2);
                double y2 = avr_y - 0.1 * mat_v(1, 2);
                double z2 = avr_z - 0.1 * mat_v(2, 2);
                Eigen::Vector3d point2 = {x2, y2, z2};

                ceres::CostFunction *cost_function = EdgeFunctor::Create(point_sel, point1, point2);
                problem_.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), pose_array_);
                ///TODO  如果pose是指针数组，输出的其实也是在原来的基础上？？ 或者直接是全局的？？
            }

            if (input_edge_->size() < 10) {
                std::cout << " too few edge feature point!" << std::endl;
            }


        }
    }
    std::cout << "valid_edge_count: " << valid_edge_count << std::endl;
}

void ScanMatch::solveProblem(){
    ceres::Solver::Options options;
    options.num_threads = 8;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.initial_trust_region_radius = 10e5;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 8;
    options.max_line_search_step_contraction = 1e-2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);


    Eigen::Matrix<double, 3, 3> rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(pose_array_[2], Eigen::Vector3d::UnitZ()) *    //rotation
                      Eigen::AngleAxisd(pose_array_[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(pose_array_[0], Eigen::Vector3d::UnitX());
    std::cout<< "rotation matrix in solveTransformOpt \n" << rotation_matrix << std::endl;

    Eigen::Vector3d translation_vec;
    translation_vec << pose_array_[3], pose_array_[4], pose_array_[5];
    if (pose_array_[3] < 0.001 && pose_array_[4] < 0.001 && pose_array_[5] < 0.001) {
        if (pose_array_[0] < 0.0002 && pose_array_[1] < 0.0002 &&
                pose_array_[2] < 0.0002)  //~deg:0.00115
            fulfill_tole_flag_ = true;
    }

    Eigen::Matrix<double,4,4> tmp = Eigen::Matrix4d::Identity();;
    tmp.block<3,3>(0,0) = rotation_matrix;
    tmp.block<3,1>(0,3) = translation_vec;
    final_transform_ = tmp * final_transform_;          //？

    pcl::transformPointCloud(*input_edge_, *input_edge_, tmp);
    pcl::transformPointCloud(*input_grd_, *input_grd_, tmp);

    ///add initialization
    for (int i = 0; i < 6; i++) {
        pose_array_[i] = 0;
    }

}



