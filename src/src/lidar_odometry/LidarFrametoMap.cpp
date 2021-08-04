#include "lidar_odometry/LidarFrametoMap.h"
#include "lidar_odometry/scanMatch.h"


LidarFrametoMap::LidarFrametoMap() {

    grd_target_.reset(new PointCloudTI);
    edge_target_.reset(new PointCloudTI);
    kdtree_g2m_.reset(new pcl::KdTreeFLANN <PointTI>);
    kdtree_e2m_.reset(new pcl::KdTreeFLANN <PointTI>);
    input_grd_.reset(new PointCloudTI);
    input_edge_.reset(new PointCloudTI);
    curr_pose_ = Eigen::Matrix4d::Identity();
    last_pose_ = Eigen::Matrix4d::Identity();

    pose_array_ = new double[6];
    for (int i = 0; i < 6; i++) {
        pose_array_[i] = 0;
    }

    last_key_frame_pose_ = Eigen::Matrix4d::Identity();

    current_grd_frame_.cloud_data.reset(new PointCloudTI);
    current_edge_frame_.cloud_data.reset(new PointCloudTI);

    grd_local_map_.reset(new PointCloudTI);
    edge_local_map_.reset(new PointCloudTI);

}

void LidarFrametoMap::setParameters(const PointCloudTI::Ptr &cloud_grd,
                                    const PointCloudTI::Ptr &cloud_edge,
                                    const Eigen::Matrix4d &initial_guess) {
    current_grd_frame_.cloud_data = cloud_grd;
    current_edge_frame_.cloud_data = cloud_edge;
    initial_predict_pose_ = initial_guess;
    //now the initial guess is a invalid parameter.
}

Eigen::Matrix<double, 3, 3> LidarFrametoMap::eulerAngletoRotation(double& row, double& pitch, double& yaw) {
    Eigen::Matrix<double, 3, 3> rz;
    rz << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    Eigen::Matrix<double, 3, 3> ry;
    ry << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
    Eigen::Matrix<double, 3, 3> rx;
    rx << 1, 0, 0, 0, cos(row), -sin(row), 0, sin(row), cos(row);
    return rz * ry * rx;
}

Eigen::Matrix4d LidarFrametoMap::solveTransform() {
    fulfill_tole_flag_ = false;

    //Eigen::Matrix4d predict_pose = curr_pose_ * ( last_pose_.inverse() * curr_pose_);
    //last_pose_ = curr_pose_;
    //curr_pose_ = predict_pose;
    curr_pose_ = initial_predict_pose_;

    //from curr_pose to pose_array (x,y,z)
    Eigen::Matrix3d rotation_mat;
    rotation_mat = curr_pose_.block<3, 3>(0, 0);
    Eigen::Vector3d euler_angle;
    euler_angle = rotation_mat.eulerAngles(2, 1, 0);
    pose_array_[0] =euler_angle[2];
    pose_array_[1] =euler_angle[1];
    pose_array_[2] =euler_angle[0];
    pose_array_[3] =curr_pose_(0,3);
    pose_array_[4] =curr_pose_(1,3);
    pose_array_[5] =curr_pose_(2,3);

    //the first frame

    if (local_grdmap_frames_.empty() && local_edgemap_frames_.empty()) {
        current_grd_frame_.pose = init_pose_;
        current_edge_frame_.pose = init_pose_;
        updateLocalMap(current_grd_frame_, current_edge_frame_);
        std::cout <<" THIS IS THE FIRST FRAME" << std::endl;
    }
    else {
        PointCloudTI::Ptr result_grd(new PointCloudTI);
        PointCloudTI::Ptr result_edge(new PointCloudTI);

        frame2mapRegistration(current_grd_frame_.cloud_data, current_edge_frame_.cloud_data,
                              curr_pose_,  /*as a guess, return the better solution*/
                              result_grd, result_edge, /* from lidar to world*/
                              current_grd_frame_.pose, current_edge_frame_.pose);

        //优化完之后得到求解出的pose_array_
        //得到相对于第一帧的pose并放入frame对应的pose中

        //if it is a keyframe
        if (fabs(last_key_frame_pose_(0, 3) - current_grd_frame_.pose(0, 3)) +
            fabs(last_key_frame_pose_(1, 3) - current_grd_frame_.pose(1, 3)) +
            fabs(last_key_frame_pose_(2, 3) - current_grd_frame_.pose(2, 3)) > key_frame_distance_) {
            updateLocalMap(current_grd_frame_, current_edge_frame_);
            last_key_frame_pose_ = current_grd_frame_.pose;
        }
    }
    return current_grd_frame_.pose;
}


/////////////////

void LidarFrametoMap::updateLocalMap(const Frame& current_grd_frame, const Frame& current_edge_frame) {

    ////give grd_local_map_ to grd_target_

    local_grdmap_frames_.push_back(current_grd_frame);
    local_edgemap_frames_.push_back(current_edge_frame);
    while (local_grdmap_frames_.size() > local_frame_num_) {
        local_grdmap_frames_.pop_front();
        local_edgemap_frames_.pop_front();
    }
    //如果有弹出就重置local_map，并存入当前帧
    grd_local_map_.reset(new PointCloudTI);
    edge_local_map_.reset(new PointCloudTI);
    PointCloudTI::Ptr transformed_grd(new PointCloudTI);
    PointCloudTI::Ptr transformed_edge(new PointCloudTI);
    for (int i = 0; i < local_grdmap_frames_.size(); i++) {
        pcl::transformPointCloud(*local_grdmap_frames_.at(i).cloud_data,
                                 *transformed_grd, local_grdmap_frames_.at(i).pose);   //转到第一帧的坐标系
        pcl::transformPointCloud(*local_edgemap_frames_.at(i).cloud_data,
                                 *transformed_edge, local_edgemap_frames_.at(i).pose);
        *grd_local_map_ += *transformed_grd;
        *edge_local_map_ += *transformed_edge;
    }

    if (local_grdmap_frames_.size() < 10) {
        //更新要匹配的目标点云local map
        setTargetCloud(grd_local_map_, edge_local_map_);
    } else {
        PointCloudTI::Ptr grd_filtered_map(new PointCloudTI);
        PointCloudTI::Ptr edge_filtered_map(new PointCloudTI);
        //滤波之后再设置为匹配时的目标点云
        filterLocalMap(grd_local_map_, edge_local_map_,
                       grd_filtered_map, edge_filtered_map);
        setTargetCloud(grd_filtered_map, edge_filtered_map);
    }

    std::cout <<"grd_target_.size()" << grd_target_->size() <<"\n";
    std::cout <<"edge_target_.size()" << edge_target_->size() <<"\n";
}


void LidarFrametoMap::filterLocalMap(const PointCloudTI::Ptr &grd_local_map, const PointCloudTI::Ptr &edge_local_map,
                                     PointCloudTI::Ptr &grd_filtered, PointCloudTI::Ptr &edge_filtered) {

    filter_.setLeafSize(0.4f, 0.4f, 0.4f);
    filter_.setInputCloud(grd_local_map);
    filter_.filter(*grd_filtered);
    filter_.setLeafSize(0.2f, 0.2f, 0.2f);
    filter_.setInputCloud(edge_local_map);
    filter_.filter(*edge_filtered);
    //only XYZ for a filter downsampling function
}

void LidarFrametoMap::setTargetCloud(const PointCloudTI::Ptr &grd_target, const PointCloudTI::Ptr &edge_target) {
    grd_target_ = grd_target;
    edge_target_ = edge_target;
}

void LidarFrametoMap::frame2mapRegistration(const PointCloudTI::Ptr &input_grd, const PointCloudTI::Ptr &input_edge,
                                            Eigen::Matrix4d& predict_pose,
                                            PointCloudTI::Ptr &result_grd, PointCloudTI::Ptr &result_edge,
                                            Eigen::Matrix4d& result_pose_grd,  Eigen::Matrix4d& result_pose_edge) {
    ///initial guess could be imu/odometry/motion assumption...
    //final_transform_ = predict_pose;


    //input_edge_ = input_edge;

    Eigen::Matrix3d rotation_mat;
    rotation_mat = predict_pose.block<3, 3>(0, 0);
    Eigen::Vector3d euler_angle;
    euler_angle = rotation_mat.eulerAngles(2, 1, 0);
    pose_array_[0] = euler_angle[2];
    pose_array_[1] = euler_angle[1];
    pose_array_[2] = euler_angle[0];
    pose_array_[3] = predict_pose(0,3);
    pose_array_[4] = predict_pose(1,3);
    pose_array_[5] = predict_pose(2,3);

    int i = 0;
    kdtree_g2m_->setInputCloud(grd_target_);
    kdtree_e2m_->setInputCloud(edge_target_);
    while (i < max_iter_ && !fulfill_tole_flag_) {
        ++i;
        findEdgeCorrespondence(input_edge, predict_pose);
        findGroundCorrespondence(input_grd, predict_pose);
        std::cout<<i<<". iter: before"<<pose_array_[0] <<" "<<pose_array_[1] <<" "<<pose_array_[2]<<" "
                 << pose_array_[3] << " " << pose_array_[4] << " "<< pose_array_[5]<<std::endl;
        solveProblem();
        std::cout<<i<<". iter: after"<<pose_array_[0] <<" "<<pose_array_[1] <<" "<<pose_array_[2]<<" "
                                     << pose_array_[3] << " " << pose_array_[4] << " "<< pose_array_[5] <<std::endl;
    }

    Eigen::Matrix3d rot = eulerAngletoRotation(pose_array_[0], pose_array_[1], pose_array_[2]);
    result_pose_grd.block<3,3>(0,0) = rot;
    result_pose_grd.block<3,1>(0,3) << pose_array_[3], pose_array_[4], pose_array_[5];
    result_pose_edge.block<3,3>(0,0) = rot;
    result_pose_edge.block<3,1>(0,3) << pose_array_[3], pose_array_[4], pose_array_[5];

    predict_pose = result_pose_edge;
    //input_grd_.reset(new PointCloudTI);
    //input_edge_.reset(new PointCloudTI);

}


void LidarFrametoMap::findGroundCorrespondence(const PointCloudTI::Ptr &input_grd, const Eigen::Matrix4d& predict_pose) {

    int valid_plane_count = 0;

    pcl::transformPointCloud(*input_grd, *input_grd_, predict_pose);



    for (int i = 0; i < input_grd_->size(); i++) {

        std::vector<float> search_sqdist;
        std::vector<int> search_id;

        //要把当前点转到世界坐标系下 再跟map寻找最近邻. 假定input已经经过了坐标转换
        //使用target建立kdtree，搜索input当前关键帧的近邻点
        kdtree_g2m_->nearestKSearch(input_grd_->points[i], 5, search_id, search_sqdist);




        Eigen::Matrix<double, 5, 3> mat_points;   //A
        Eigen::Matrix<double, 5, 1> mat_r;   //b
        mat_r.fill(-1);
        Eigen::Matrix<double, 3, 1> mat_x0;

        if (search_sqdist[4] < 1.0) {   //default : 1.0
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
                PointTI point_sel = input_grd->points[i];
                /*
                Eigen::Vector3d ave = mat_points.colwise().sum() / 5;
                pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Association Viewer"));
                viewer->setBackgroundColor (0, 0, 0);
                pcl::visualization::PointCloudColorHandlerCustom<PointTI> input_grd_color(input_grd, 250, 0, 0);
                viewer->addPointCloud<pcl::PointXYZI> (input_grd, input_grd_color, "input_grd");
                pcl::visualization::PointCloudColorHandlerCustom<PointTI> target_grd_color(grd_target_, 0, 250, 0);
                viewer->addPointCloud<pcl::PointXYZI> (grd_target_, target_grd_color, "target_grd");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_grd");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target_grd");
                viewer->addCoordinateSystem (3.0);
                viewer->initCameraParameters ();
                pcl::ModelCoefficients plane_coeff;
                plane_coeff.values.resize(4);
                plane_coeff.values[0] = pa;
                plane_coeff.values[1] = pb;
                plane_coeff.values[2] = pc;
                plane_coeff.values[3] = pd;
                viewer->addPlane(plane_coeff, ave(0), ave(1),ave(2));
                viewer->addLine<pcl::PointXYZI> (input_grd_->points[i],
                                                 grd_target_->points[search_id[0]], "line"+std::to_string(i)+"0");
                viewer->addLine<pcl::PointXYZI> (input_grd_->points[i],
                                                 grd_target_->points[search_id[1]], "line"+std::to_string(i)+"1");
                viewer->addLine<pcl::PointXYZI> (input_grd_->points[i],
                                                 grd_target_->points[search_id[2]], "line"+std::to_string(i)+"2");
                viewer->addLine<pcl::PointXYZI> (input_grd_->points[i],
                                                 grd_target_->points[search_id[3]], "line"+std::to_string(i)+"3");
                viewer->addLine<pcl::PointXYZI> (input_grd_->points[i],
                                                 grd_target_->points[search_id[4]], "line"+std::to_string(i)+"4");


                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"0");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"1");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"2");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"3");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"4");

                while (!viewer->wasStopped ())
                {
                    viewer->spinOnce (100);
                }
                 */


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

void LidarFrametoMap::findEdgeCorrespondence(const PointCloudTI::Ptr &input_edge, const Eigen::Matrix4d& predict_pose) {
    int valid_edge_count = 0;

    //input_edge : in lidar coord
    //input_edge_: in world coord
    std::cout <<"initial_predict_pose " << "\n" <<predict_pose;
    pcl::transformPointCloud(*input_edge, *input_edge_, predict_pose);
    for (int i = 0; i < input_edge_->size(); i++) {

        std::vector<float> search_sqdist;
        std::vector<int> search_id;

        //要把当前点转到世界坐标系下 再跟map寻找最近邻. 假定input已经经过了坐标转换
        //使用target建立kdtree，搜索input当前关键帧的近邻点

        kdtree_e2m_->nearestKSearch(input_edge_->points[i], 5, search_id, search_sqdist);

        Eigen::Matrix<double, 3, 3> mat_a = Eigen::Matrix3d::Zero();   //A  covariance matrix
        Eigen::Vector3d mat_d = Eigen::Vector3d::Zero();  //d   eigen value
        Eigen::Matrix<double, 3, 3> mat_v = Eigen::Matrix3d::Zero();   //v  eigen vector
        Eigen::Matrix<double, 3, 5> mat_points;

        if (search_sqdist[4] < 3.0) {
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
                PointTI point_sel = input_edge->points[i];
                ceres::CostFunction *cost_function = EdgeFunctor::Create(point_sel, point1, point2);
                problem_.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), pose_array_);

                /*
                PointTI l1;
                l1.x = avr_x + 2 * mat_v(0, 2);
                l1.y = avr_y + 2 * mat_v(1, 2);
                l1.z = avr_z + 2 * mat_v(2, 2);
                PointTI l2;
                l2.x = avr_x - 2 * mat_v(0, 2);
                l2.y = avr_y - 2 * mat_v(1, 2);
                l2.z = avr_z - 2 * mat_v(2, 2);
                pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Association Viewer"));
                viewer->setBackgroundColor (0, 0, 0);
                pcl::visualization::PointCloudColorHandlerCustom<PointTI> input_grd_color(input_edge_, 250, 0, 0);
                viewer->addPointCloud<pcl::PointXYZI> (input_edge_, input_grd_color, "input_edge");
                pcl::visualization::PointCloudColorHandlerCustom<PointTI> target_grd_color(edge_target_, 0, 250, 0);
                viewer->addPointCloud<pcl::PointXYZI> (edge_target_, target_grd_color, "target_edge");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_edge");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target_edge");
                viewer->addCoordinateSystem (3.0);
                viewer->initCameraParameters ();
                viewer->addLine<pcl::PointXYZI> (input_edge_->points[i],
                                                 edge_target_->points[search_id[0]], "line"+std::to_string(i)+"0");
                viewer->addLine<pcl::PointXYZI> (input_edge_->points[i],
                                                 edge_target_->points[search_id[1]], "line"+std::to_string(i)+"1");
                viewer->addLine<pcl::PointXYZI> (input_edge_->points[i],
                                                 edge_target_->points[search_id[2]], "line"+std::to_string(i)+"2");
                viewer->addLine<pcl::PointXYZI> (input_edge_->points[i],
                                                 edge_target_->points[search_id[3]], "line"+std::to_string(i)+"3");
                viewer->addLine<pcl::PointXYZI> (input_edge_->points[i],
                                                 edge_target_->points[search_id[4]], "line"+std::to_string(i)+"4");


                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"0");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"1");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"2");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"3");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "line"+std::to_string(i)+"4");

                viewer->addLine<pcl::PointXYZI> (l1,l2, 0, 0, 200, "line");
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,4, "line");


                while (!viewer->wasStopped ())
                {
                    viewer->spinOnce (100);
                }
                 */

                if(valid_edge_count%1000 ==0){
                    std::cout << "valid_edge: " << "\n"  <<
                        "point1 = " << point1.transpose() << " \n" <<
                        "point2 = " << point2.transpose() << " \n" <<
                        "point_sel (lidar coord)= " << input_edge->points[i] << " \n";
                }
            }

            if (input_edge_->size() < 10) {
                std::cout << " too few edge feature point!" << std::endl;
            }
        }
    }
    std::cout << "valid_edge_count: " << valid_edge_count << std::endl;
}

void LidarFrametoMap::solveProblem(){
    ceres::Solver::Options options;
    options.num_threads = 4;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.initial_trust_region_radius = 10e5;
    //options.check_gradients = true; //?
    //options.gradient_check_relative_precision = 1e-4; //??
    options.minimizer_progress_to_stdout = false;
    //options.
    options.max_num_iterations = 4;
    options.max_line_search_step_contraction = 1e-2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);


    Eigen::Matrix<double, 3, 3> rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(pose_array_[2], Eigen::Vector3d::UnitZ()) *    //rotation
                      Eigen::AngleAxisd(pose_array_[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(pose_array_[0], Eigen::Vector3d::UnitX());

    Eigen::Vector3d translation_vec;
    /*
    translation_vec << pose_array_[3], pose_array_[4], pose_array_[5];
    if (pose_array_[3] < 0.001 && pose_array_[4] < 0.001 && pose_array_[5] < 0.001) {
        if (pose_array_[0] < 0.0002 && pose_array_[1] < 0.0002 &&
            pose_array_[2] < 0.0002) {  //~deg:0.00115
            fulfill_tole_flag_ = true;
            std::cout<< "=====flag = true=====" <<"\n";
        }

    }*/

    //Eigen::Matrix<double,4,4> tmp = Eigen::Matrix4d::Identity();
    //std::cout << "tmp: " <<"\n" << tmp<< "\n";
    //tmp.block<3,3>(0,0) = rotation_matrix;
    //tmp.block<3,1>(0,3) = translation_vec;
    //final_transform_ = tmp * final_transform_;          //？

    //pcl::transformPointCloud(*input_edge_, *input_edge_, tmp);
    //pcl::transformPointCloud(*input_grd_, *input_grd_, tmp);
    //std::cout<< "input_edge_.size() " << input_edge_->size() << "\n";

    ///add initialization
    //for (int i = 0; i < 6; i++) {
    //    pose_array_[i] = 0;
    //}

}