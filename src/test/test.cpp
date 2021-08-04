#include <ceres/ceres.h>
#include <iostream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/time.h>
#include <eigen3/Eigen/Core>
#include <string>

using PointTI = pcl::PointXYZI;
using PointCloudTI = pcl::PointCloud<PointTI>;
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;



struct EdgeFunctor {
    EdgeFunctor(const PointT &point_sel,
                const Eigen::Vector3d& point1,
                const Eigen::Vector3d& point2) :
            point_sel_(point_sel),
            point1_(point1),
            point2_(point2){};

    template<typename T>
    bool operator()(const T *const pose, T *residual) const {
        //pose : [roll, pitch, yaw, x, y, z]

        T processed_curr[3];
        Eigen::Matrix<T, 3, 3> rz;
        rz << T(cos(pose[2])), T(-sin(pose[2])), T(0), T(sin(pose[2])), T(cos(pose[2])), T(0), T(0), T(0), T(1);
        Eigen::Matrix<T, 3, 3> ry;
        ry << T(cos(pose[1])), T(0), T(sin(pose[1])), T(0), T(1), T(0), T(-sin(pose[1])), T(0), T(cos(pose[1]));
        Eigen::Matrix<T, 3, 3> rx;
        rx << T(1), T(0), T(0), T(0), T(cos(pose[0])), T(-sin(pose[0])), T(0), T(sin(pose[0])), T(cos(pose[0]));
        Eigen::Matrix<T, 3, 3> rotation_matrix = rz * ry * rx;
        processed_curr[0] = rotation_matrix(0, 0) * T(point_sel_.x) + rotation_matrix(0, 1) * T(point_sel_.y) +
                            rotation_matrix(0, 2) * T(point_sel_.z) + pose[3];
        processed_curr[1] = rotation_matrix(1, 0) * T(point_sel_.x) + rotation_matrix(1, 1) * T(point_sel_.y) +
                            rotation_matrix(1, 2) * T(point_sel_.z)  + pose[4];
        processed_curr[2] = rotation_matrix(2, 0) * T(point_sel_.x) + rotation_matrix(2, 1) * T(point_sel_.y) +
                            rotation_matrix(2, 2) * T(point_sel_.z)  + pose[5];
        T x = T(point_sel_.x);
        T y = T(point_sel_.y);
        T z = T(point_sel_.z);
        T x1 = T(point1_(0));
        T y1 = T(point1_(1));
        T z1 = T(point1_(2));
        T x2 = T(point2_(0));
        T y2 = T(point2_(1));
        T z2 = T(point2_(2));
        // | (p0 - p1) x (p0 - p2) |
        //向量OA = (x0 - x1, y0 - y1, z0 - z1), 向量OB = (x0 - x2, y0 - y2, z0 - z2)，向量AB = （x1 - x2, y1 - y2, z1 - z2）
        //向量OA OB的向量积(即叉乘)为：
        //|  i      j      k  |
        //|x0-x1  y0-y1  z0-z1|
        //|x0-x2  y0-y2  z0-z2|

        T a012 = sqrt(
                ((x - x1) * (y - y2) - (x - x2) * (y - y1)) * ((x - x1) * (y - y2) - (x - x2) * (y - y1))
                + ((x - x1) * (z - z2) - (x - x2) * (z - z1)) * ((x - x1) * (z - z2) - (x - x2) * (z - z1))
                + ((y - y1) * (z - z2) - (y - y2) * (z - z1)) * ((y - y1) * (z - z2) - (y - y2) * (z - z1)));
        // |(p1 - p2)|
        T l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

        // 点到直线距离
        T ld2 =  a012 / l12;
        T weight = T(1) - T(0.9)* T(ld2>=T(0)? ld2:-ld2);
        residual[0] = weight * ld2;
        return true;
    }

    static ceres::CostFunction *Create(const PointT point_sel,
                                       const Eigen::Vector3d point1,
                                       const Eigen::Vector3d point2) {
        return (new ceres::AutoDiffCostFunction<EdgeFunctor, 1, 6>(
                new EdgeFunctor(point_sel, point1, point2)));
    }
    const PointT point_sel_;
    const Eigen::Vector3d point1_;
    const Eigen::Vector3d point2_;
};



void solution(const PointCloudT::Ptr& input_edge, const PointCloudT::Ptr& target_edge,
                            double* pose_array,
                            Eigen::Matrix4d& final_transform) {
    int valid_edge_count = 0;

    ceres::Problem problem;
    pcl::KdTreeFLANN <PointT>::Ptr kdtree_e2m(new pcl::KdTreeFLANN <PointT>);
    kdtree_e2m->setInputCloud(target_edge);

    int k=0;
    bool fulfill_tole_flag = false;
    while (k < 5 && !fulfill_tole_flag) {
        ++k;
        for (int i = 0; i < input_edge->size(); i++) {

            std::vector<float> search_sqdist;
            std::vector<int> search_id;

            //要把当前点转到世界坐标系下 再跟map寻找最近邻. 假定input已经经过了坐标转换
            //使用target建立kdtree，搜索input当前关键帧的近邻点
            PointT point_sel = input_edge->points[i];
            kdtree_e2m->nearestKSearch(point_sel, 5, search_id, search_sqdist);

            Eigen::Matrix<double, 3, 3> mat_a = Eigen::Matrix3d::Zero();   //A  covariance matrix
            Eigen::Vector3d mat_d = Eigen::Vector3d::Zero();  //d   eigen value
            Eigen::Matrix<double, 3, 3> mat_v = Eigen::Matrix3d::Zero();   //v  eigen vector
            Eigen::Matrix<double, 3, 5> mat_points;

            if (search_sqdist[4] < 2.0) {
                //find the point on the average

                double avr_x = 0, avr_y = 0, avr_z = 0;
                for (int j = 0; j < 5; j++) {
                    avr_x += target_edge->points[search_id[j]].x;
                    avr_y += target_edge->points[search_id[j]].y;
                    avr_z += target_edge->points[search_id[j]].z;
                    mat_points.block<3, 1>(0, j) << target_edge->points[search_id[j]].x,
                            target_edge->points[search_id[j]].y,
                            target_edge->points[search_id[j]].z;
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
                    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), pose_array);

                }

                if (input_edge->size() < 10) {
                    std::cout << " too few edge feature point!" << std::endl;
                }


            }
        }
        std::cout << "valid_edge_count: " << valid_edge_count << std::endl;

        ceres::Solver::Options options;
        options.num_threads = 8;
        options.linear_solver_type = ceres::DENSE_QR;
        //options.initial_trust_region_radius = 10e5;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 8;
        options.max_line_search_step_contraction = 1e-2;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        Eigen::Matrix<double, 3, 3> rotation_matrix;
        rotation_matrix = Eigen::AngleAxisd(pose_array[2], Eigen::Vector3d::UnitZ()) *    //rotation
                          Eigen::AngleAxisd(pose_array[1], Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(pose_array[0], Eigen::Vector3d::UnitX());

        Eigen::Vector3d translation_vec;
        translation_vec << pose_array[3], pose_array[4], pose_array[5];
        if (pose_array[3] < 0.001 && pose_array[4] < 0.001 && pose_array[5] < 0.001) {
            if (pose_array[0] < 0.0002 && pose_array[1] < 0.0002 &&
                pose_array[2] < 0.0002)  //~deg:0.00115
                fulfill_tole_flag = true;
        }

        Eigen::Matrix<double, 4, 4> tmp = Eigen::Matrix4d::Identity();
        std::cout << "tmp: " << "\n" << tmp << "\n";
        tmp.block<3, 3>(0, 0) = rotation_matrix;
        tmp.block<3, 1>(0, 3) = translation_vec;
        final_transform = tmp * final_transform;          //？

        pcl::transformPointCloud(*input_edge, *input_edge, tmp);
        std::cout << "input_edge_.size() " << input_edge->size() << "\n";

        ///add initialization
        for (int i = 0; i < 6; i++) {
            pose_array[i] = 0;
        }

    }
}


int main(){

    double *pose_array;
    pose_array = new double[6];
    for (int i = 0; i < 6; i++) {
        pose_array[i] = 0;
    }

    std::string path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/";
    Eigen::Matrix4d final_transform = Eigen::Matrix4d::Identity();
    PointCloudT::Ptr input_edge(new PointCloudT);
    PointCloudT::Ptr target_edge(new PointCloudT);
    pcl::io::loadPCDFile<PointT>(path +  "input_edge.pcd", *input_edge);
    pcl::io::loadPCDFile<PointT>(path +  "target_edge.pcd", *target_edge);
    solution(input_edge, target_edge, pose_array, final_transform);
    std::cout << "final" << "\n" <<final_transform;

}