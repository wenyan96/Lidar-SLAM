#pragma once

#include "common/utils.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

class ScanMatch {
public:
    ScanMatch();

    void setTargetCloud(const PointCloudT::Ptr &grd_target, const PointCloudT::Ptr &edge_target);

    void frame2mapRegistration(const PointCloudT::Ptr &input_grd, const PointCloudT::Ptr &input_edge,
                               const Eigen::Matrix4d& predict_pose,
                               PointCloudT::Ptr &result_grd, PointCloudT::Ptr &resutlt_edge,
                               Eigen::Matrix4d& result_pose_grd,  Eigen::Matrix4d& result_pose_edge);


    void findGroundCorrespondence();

    void findEdgeCorrespondence();

    void solveProblem();

    struct GroundFunctor {
        GroundFunctor(const PointT &point_sel,
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
            processed_curr[0] = rotation_matrix(0, 0) * T(point_sel_.x) + rotation_matrix(0, 1) * T(point_sel_.y) +
                                rotation_matrix(0, 2) * T(point_sel_.z) + pose[3];
            processed_curr[1] = rotation_matrix(1, 0) * T(point_sel_.x) + rotation_matrix(1, 1) * T(point_sel_.y) +
                                rotation_matrix(1, 2) * T(point_sel_.z)  + pose[4];
            processed_curr[2] = rotation_matrix(2, 0) * T(point_sel_.x) + rotation_matrix(2, 1) * T(point_sel_.y) +
                                rotation_matrix(2, 2) * T(point_sel_.z)  + pose[5];

            residual[0] =  T(pa_) * T(point_sel_.x) + T(pb_) * T(point_sel_.y) + T(pc_) * T(point_sel_.z) + T(pd_);
            return true;
        }

        static ceres::CostFunction *Create(const PointT &point_sel,
                                           const double &pa, const double &pb,
                                           const double &pc, const double &pd) {
            return (new ceres::AutoDiffCostFunction<GroundFunctor, 1, 6>(
                    new GroundFunctor(point_sel, pa, pb, pc, pd)));
        }

        const PointT point_sel_;
        const double pa_;
        const double pb_;
        const double pc_;
        const double pd_;
    };


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
            //叉积的模|axb|=a*b*sin(theta)
            T a012 = sqrt(
                    ((x - x1) * (y - y2) - (x - x2) * (y - y1)) * ((x - x1) * (y - y2) - (x - x2) * (y - y1))
                    + ((x - x1) * (z - z2) - (x - x2) * (z - z1)) * ((x - x1) * (z - z2) - (x - x2) * (z - z1))
                    + ((y - y1) * (z - z2) - (y - y2) * (z - z1)) * ((y - y1) * (z - z2) - (y - y2) * (z - z1)));
            T l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

            // 三角形的高，也就是点到直线距离
            residual[0] = a012 / l12;
            return true;
        }

        static ceres::CostFunction *Create(const PointT &point_sel,
                                           const Eigen::Vector3d& point1,
                                           const Eigen::Vector3d& point2) {
            return (new ceres::AutoDiffCostFunction<EdgeFunctor, 1, 6>(
                    new EdgeFunctor(point_sel, point1, point2)));
        }
        const PointT point_sel_;
        const Eigen::Vector3d point1_;
        const Eigen::Vector3d point2_;
    };


private:
    PointCloudT::Ptr grd_target_;
    PointCloudT::Ptr edge_target_;
    pcl::KdTreeFLANN<PointT>::Ptr kdtree_g2m_;
    pcl::KdTreeFLANN<PointT>::Ptr kdtree_e2m_;
    PointCloudT::Ptr input_grd_;
    PointCloudT::Ptr input_edge_;

    double *pose_array_;
    ceres::Problem problem_;

    int max_iter_ = 8;
    bool fulfill_tole_flag_ = false;
    Eigen::Matrix4d final_transform_ = Eigen::Matrix4d::Identity();
};