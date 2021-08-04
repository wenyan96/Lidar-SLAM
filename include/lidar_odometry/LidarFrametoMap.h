#pragma once
#include "common/utils.h"
//#include "lidar_odometry/scanMatch.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

class LidarFrametoMap{

public:
    struct Frame{
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        PointCloudTI::Ptr cloud_data;
    };

    LidarFrametoMap();

    //set input
    void setParameters(const PointCloudTI::Ptr& cloud_grd,
                       const PointCloudTI::Ptr& cloud_edge,
                       const Eigen::Matrix4d& initial_guess);

    //solve step
    Eigen::Matrix4d solveTransform();


    void updateLocalMap(const Frame& current_grd_frame_, const Frame& current_edge_frame_);

    void filterLocalMap(const PointCloudTI::Ptr& grd_local_map, const PointCloudTI::Ptr& edge_local_map,
                        PointCloudTI::Ptr& grd_filtered, PointCloudTI::Ptr& edge_filtered);



    void setTargetCloud(const PointCloudTI::Ptr &grd_target, const PointCloudTI::Ptr &edge_target);

    void frame2mapRegistration(const PointCloudTI::Ptr &input_grd, const PointCloudTI::Ptr &input_edge,
                               Eigen::Matrix4d& predict_pose,
                               PointCloudTI::Ptr &result_grd, PointCloudTI::Ptr &resutlt_edge,
                               Eigen::Matrix4d& result_pose_grd,  Eigen::Matrix4d& result_pose_edge);


    void findGroundCorrespondence(const PointCloudTI::Ptr &input_grd, const Eigen::Matrix4d& predict_pose);

    void findEdgeCorrespondence(const PointCloudTI::Ptr &input_edge, const Eigen::Matrix4d& predict_pose);

    void solveProblem();

    Eigen::Matrix<double, 3, 3> eulerAngletoRotation(double& row, double& pitch, double& yaw);

    struct GroundFunctor {
        GroundFunctor(const PointTI &point_sel,
                      const double &pa,
                      const double &pb,
                      const double &pc,
                      const double &pd) :
                point_sel_(point_sel), pa_(pa),
                pb_(pb), pc_(pc) , pd_(pd){};

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
            //std::cout<< "in ceres: rotation :" << rotation_matrix <<"\n";
            processed_curr[0] = rotation_matrix(0, 0) * T(point_sel_.x) + rotation_matrix(0, 1) * T(point_sel_.y) +
                                rotation_matrix(0, 2) * T(point_sel_.z) + pose[3];
            processed_curr[1] = rotation_matrix(1, 0) * T(point_sel_.x) + rotation_matrix(1, 1) * T(point_sel_.y) +
                                rotation_matrix(1, 2) * T(point_sel_.z)  + pose[4];
            processed_curr[2] = rotation_matrix(2, 0) * T(point_sel_.x) + rotation_matrix(2, 1) * T(point_sel_.y) +
                                rotation_matrix(2, 2) * T(point_sel_.z)  + pose[5];

            residual[0] =  T(pa_) * T(processed_curr[0]) + T(pb_) * T(processed_curr[1]) + T(pc_) * T(processed_curr[2]) + T(pd_);
            //std::cout<<"=======GroundFactor residual: " << residual[0]<<std::endl;
            return true;
        }

        static ceres::CostFunction *Create(const PointTI &point_sel,
                                           const double &pa, const double &pb,
                                           const double &pc, const double &pd) {
            return (new ceres::AutoDiffCostFunction<GroundFunctor, 1, 6>(
                    new GroundFunctor(point_sel, pa, pb, pc, pd)));
        }

        const PointTI point_sel_;
        const double pa_;
        const double pb_;
        const double pc_;
        const double pd_;
    };


    struct EdgeFunctor {
        EdgeFunctor(const PointTI &point_sel,
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
            T x = T(processed_curr[0]);
            T y = T(processed_curr[1]);
            T z = T(processed_curr[2]);
            T x1 = T(point1_(0));
            T y1 = T(point1_(1));
            T z1 = T(point1_(2));
            T x2 = T(point2_(0));
            T y2 = T(point2_(1));
            T z2 = T(point2_(2));
            //叉积的模|axb|=a*b*sin(theta)
            T a012 = sqrt(
                    ((x - x1) * (y - y2) - (x - x2) * (y - y1)) * ((x - x1) * (y - y2) - (x - x2) * (y - y1))
                    + ((x - x1) * (z - z2) - (x - x2) * (z - z1)) * ((x - x1) * (z - z2) - (x - x2) * (z - z1))
                    + ((y - y1) * (z - z2) - (y - y2) * (z - z1)) * ((y - y1) * (z - z2) - (y - y2) * (z - z1)));

            //Eigen::Matrix<T,3,1> papb_cross = {T((x - x1) * (y - y2) - (x - x2) * (y - y1)),
            //                                   T((x - x1) * (z - z2) - (x - x2) * (z - z1)),
            //                                   T((y - y1) * (z - z2) - (y - y2) * (z - z1))};
            //Eigen::Matrix<T,3,1> ab ={ T(x1-x2), T(y1-y2), T(z1-z2)};
            T l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

            // 三角形的高，也就是点到直线距离
            residual[0] = a012 / l12;
            //residual[0] = T(papb_cross(0,0)) / ab.norm().template cast<T>;
            //residual[1] = T(papb_cross(1,0)) / ab.norm().template cast<T>;
            //residual[2] = T(papb_cross(2,0)) / ab.norm().template cast<T>;
            return true;
        }

        static ceres::CostFunction *Create(const PointTI &point_sel,
                                           const Eigen::Vector3d& point1,
                                           const Eigen::Vector3d& point2) {
            return (new ceres::AutoDiffCostFunction<EdgeFunctor, 1, 6>(
                    new EdgeFunctor(point_sel, point1, point2)));
        }
        const PointTI point_sel_;
        const Eigen::Vector3d point1_;
        const Eigen::Vector3d point2_;
    };




private:
    Eigen::Matrix4d curr_pose_;
    Eigen::Matrix4d last_pose_;
    Eigen::Matrix4d last_key_frame_pose_;

    Eigen::Matrix4d initial_predict_pose_;

    PointCloudTI::Ptr grd_target_;
    PointCloudTI::Ptr edge_target_;
    pcl::KdTreeFLANN<PointTI>::Ptr kdtree_g2m_;
    pcl::KdTreeFLANN<PointTI>::Ptr kdtree_e2m_;
    PointCloudTI::Ptr input_grd_;
    PointCloudTI::Ptr input_edge_;

    double *pose_array_;  //roll pitch yaw x y z
    ceres::Problem problem_;

    int max_iter_ = 6;
    bool fulfill_tole_flag_ = false;

    pcl::VoxelGrid<pcl::PointXYZI> filter_;
    Frame current_grd_frame_;
    Frame current_edge_frame_;

    double key_frame_distance_ = 2.0;

    int local_frame_num_ = 15;
    std::deque<Frame> local_grdmap_frames_;
    std::deque<Frame> local_edgemap_frames_;
    PointCloudTI::Ptr grd_local_map_; //指向整个local map点云
    PointCloudTI::Ptr edge_local_map_; //指向整个local map点云

    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();

};