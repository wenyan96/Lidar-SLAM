#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <ceres/rotation.h>
#include <ceres/ceres.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;


class LidarOdometry {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LidarOdometry();

    struct Frame{
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        PointCloudTI::Ptr cloud_data;
    };

    void setParameters(const Eigen::Matrix4d &initial_guess,
                       const PointCloudTI::Ptr &surf_current, const PointCloudTI::Ptr &surf_last,
                       const PointCloudTI::Ptr &edge_current, const PointCloudTI::Ptr &edge_last,
                       const PointCloudTI::Ptr &ground_current, const PointCloudTI::Ptr &ground_last);

    void findEdgeCorrespondence();

    void findSurfCorrespondence();

    void findGroundCorrespondence(const PointCloudTI::Ptr& current_cloud,
                                  const PointCloudTI::Ptr& last_cloud,
                                  const std::string surf_type);

    void solveTransform();

    void solveTransformOpt();

    Eigen::Matrix4d poseSolver(const std::string &method);

    Eigen::Matrix4d poseCalculation1();

    Eigen::Matrix4d poseCalculation2();


    struct EdgeFunctor {
        EdgeFunctor(const Eigen::Vector3d &point_curr,
                    const Eigen::Vector3d &closest_last,
                    const Eigen::Vector3d &min2_last) :
                point_curr_(point_curr), closest_last_(closest_last), min2_last_(min2_last) {}

        template<typename T>
        bool operator()(const T *const pose, T *residual) const {
            //pose : [roll, pitch, yaw, x, y, z]
            T processed_curr[3];
            /*
            rz = Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ()).template cast<T>();
            ry = Eigen::AngleAxisd(pose[1], Eigen::Vector3d::UnitY()).template cast<T>();
            rx = Eigen::AngleAxisd(pose[0], Eigen::Vector3d::UnitX()).template cast<T>();*/
            Eigen::Matrix<T, 3, 3> rz;
            rz << T(cos(pose[2])), T(-sin(pose[2])), T(0), T(sin(pose[2])), T(cos(pose[2])), T(0), T(0), T(0), T(1);
            Eigen::Matrix<T, 3, 3> ry;
            ry << T(cos(pose[1])), T(0), T(sin(pose[1])), T(0), T(1), T(0), T(-sin(pose[1])), T(0), T(cos(pose[1]));
            Eigen::Matrix<T, 3, 3> rx;
            rx << T(1), T(0), T(0), T(0), T(cos(pose[0])), T(-sin(pose[0])), T(0), T(sin(pose[0])), T(cos(pose[0]));
            Eigen::Matrix<T, 3, 3> rotation_matrix = rz * ry * rx;
            processed_curr[0] = rotation_matrix(0, 0) * point_curr_[0] + rotation_matrix(0, 1) * point_curr_[1] +
                                rotation_matrix(0, 2) * point_curr_[2] + pose[3];
            processed_curr[1] = rotation_matrix(1, 0) * point_curr_[0] + rotation_matrix(1, 1) * point_curr_[1] +
                                rotation_matrix(1, 2) * point_curr_[2] + pose[4];
            processed_curr[2] = rotation_matrix(2, 0) * point_curr_[0] + rotation_matrix(2, 1) * point_curr_[1] +
                                rotation_matrix(2, 2) * point_curr_[2] + pose[5];


            T oa[3];
            oa[0] = processed_curr[0] - T(closest_last_[0]);
            oa[1] = processed_curr[1] - T(closest_last_[1]);
            oa[2] = processed_curr[2] - T(closest_last_[2]);
            /*
            oa[0] = processed_curr[0] - closest_last_[0].template  cast<T>();
            oa[1] = processed_curr[1] - closest_last_[1].template  cast<T>();
            oa[2] = processed_curr[2] - closest_last_[2].template  cast<T>();
             */
            //Eigen::Matrix<T, 3, 1> ob = processed_curr - (T)min2_last_;
            T ob[3];
            ob[0] = processed_curr[0] - T(min2_last_[0]);
            ob[1] = processed_curr[1] - T(min2_last_[1]);
            ob[2] = processed_curr[2] - T(min2_last_[2]);

            T tmp_last[3];
            tmp_last[0] = T(closest_last_[0]) - T(min2_last_[0]);
            tmp_last[1] = T(closest_last_[1]) - T(min2_last_[1]);
            tmp_last[2] = T(closest_last_[2]) - T(min2_last_[2]);
            T ab_norm = sqrt(tmp_last[0] * tmp_last[0] + tmp_last[1] * tmp_last[1] + tmp_last[2] * tmp_last[2]);


            T normal[3];
            normal[0] = oa[1] * ob[2] - oa[2] * ob[1];
            normal[1] = oa[2] * ob[0] - oa[0] * ob[2];
            normal[2] = oa[0] * ob[1] - oa[1] * ob[0];
            T area = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            residual[0] = area / ab_norm;
            return true;
        }
        static ceres::CostFunction *Create(const Eigen::Vector3d point_curr, const Eigen::Vector3d closest_last,
                                           const Eigen::Vector3d min2_last) {
            return (new ceres::AutoDiffCostFunction<EdgeFunctor, 1, 6>(
                    new EdgeFunctor(point_curr, closest_last, min2_last)));
        }
        const Eigen::Vector3d point_curr_;
        const Eigen::Vector3d closest_last_;
        const Eigen::Vector3d min2_last_;
    };



    struct EdgeFunctor2 {
        EdgeFunctor2(const Eigen::Vector3d &point_curr,
                    const Eigen::Vector3d &closest_last,
                    const Eigen::Vector3d &min2_last,
                    const Eigen::Vector3d &parameter_pre) :
                point_curr_(point_curr), closest_last_(closest_last),
                min2_last_(min2_last), parameter_pre_(parameter_pre) {}

        template<typename T>
        bool operator()(const T *const pose, T *residual) const {

            T processed_curr[3];
            //parameter_pre 0-2   roll pitch z
            /*
            rz = Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ()).template cast<T>();
            ry = Eigen::AngleAxisd(pose[1], Eigen::Vector3d::UnitY()).template cast<T>();
            rx = Eigen::AngleAxisd(pose[0], Eigen::Vector3d::UnitX()).template cast<T>();*/
            ////pose -> opt_array_ : yaw, x, y
            Eigen::Matrix<T, 3, 3> rz;
            rz << T(cos(pose[0])), T(-sin(pose[0])), T(0), T(sin(pose[0])), T(cos(pose[0])), T(0), T(0), T(0), T(1);
            Eigen::Matrix<T, 3, 3> ry;
            ry << T(cos(0)), T(0), T(sin(0)), T(0),
                    T(1), T(0), T(-sin(0)), T(0), T(cos(0));
            Eigen::Matrix<T, 3, 3> rx;
            rx << T(1), T(0), T(0), T(0), T(cos(0)), T(-sin(0)), T(0),
                    T(sin(0)), T(cos(0));
            Eigen::Matrix<T, 3, 3> rotation_matrix = rz * ry * rx;
            processed_curr[0] = rotation_matrix(0, 0) * point_curr_[0] + rotation_matrix(0, 1) * point_curr_[1] +
                                rotation_matrix(0, 2) * point_curr_[2] + pose[1];
            processed_curr[1] = rotation_matrix(1, 0) * point_curr_[0] + rotation_matrix(1, 1) * point_curr_[1] +
                                rotation_matrix(1, 2) * point_curr_[2] + pose[2];
            processed_curr[2] = rotation_matrix(2, 0) * point_curr_[0] + rotation_matrix(2, 1) * point_curr_[1] +
                                rotation_matrix(2, 2) * point_curr_[2] ;


            T oa[3];
            oa[0] = processed_curr[0] - T(closest_last_[0]);
            oa[1] = processed_curr[1] - T(closest_last_[1]);
            oa[2] = processed_curr[2] - T(closest_last_[2]);
            /*
            oa[0] = processed_curr[0] - closest_last_[0].template  cast<T>();
            oa[1] = processed_curr[1] - closest_last_[1].template  cast<T>();
            oa[2] = processed_curr[2] - closest_last_[2].template  cast<T>();
             */
            //Eigen::Matrix<T, 3, 1> ob = processed_curr - (T)min2_last_;
            T ob[3];
            ob[0] = processed_curr[0] - T(min2_last_[0]);
            ob[1] = processed_curr[1] - T(min2_last_[1]);
            ob[2] = processed_curr[2] - T(min2_last_[2]);

            T tmp_last[3];
            tmp_last[0] = T(closest_last_[0]) - T(min2_last_[0]);
            tmp_last[1] = T(closest_last_[1]) - T(min2_last_[1]);
            tmp_last[2] = T(closest_last_[2]) - T(min2_last_[2]);
            T ab_norm = sqrt(tmp_last[0] * tmp_last[0] + tmp_last[1] * tmp_last[1] + tmp_last[2] * tmp_last[2]);


            T normal[3];
            normal[0] = oa[1] * ob[2] - oa[2] * ob[1];
            normal[1] = oa[2] * ob[0] - oa[0] * ob[2];
            normal[2] = oa[0] * ob[1] - oa[1] * ob[0];
            T area = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            residual[0] = area / ab_norm;
            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector3d point_curr, const Eigen::Vector3d closest_last,
                                           const Eigen::Vector3d min2_last, const Eigen::Vector3d parameter_pre) {
            return (new ceres::AutoDiffCostFunction<EdgeFunctor2, 1, 3>(
                    new EdgeFunctor2(point_curr, closest_last, min2_last, parameter_pre)));
        }
        const Eigen::Vector3d point_curr_;
        const Eigen::Vector3d closest_last_;
        const Eigen::Vector3d min2_last_;
        const Eigen::Vector3d parameter_pre_;
    };

    struct SurfFunctor {
        SurfFunctor(const Eigen::Vector3d &point_curr,
                    const Eigen::Vector3d &closest_last,
                    const Eigen::Vector3d &min2_last,
                    const Eigen::Vector3d &min3_last) :
                point_curr_(point_curr), closest_last_(closest_last),
                min2_last_(min2_last), min3_last_(min3_last) {};

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
            processed_curr[0] = rotation_matrix(0, 0) * point_curr_[0] + rotation_matrix(0, 1) * point_curr_[1] +
                                rotation_matrix(0, 2) * point_curr_[2] + pose[3];
            processed_curr[1] = rotation_matrix(1, 0) * point_curr_[0] + rotation_matrix(1, 1) * point_curr_[1] +
                                rotation_matrix(1, 2) * point_curr_[2] + pose[4];
            processed_curr[2] = rotation_matrix(2, 0) * point_curr_[0] + rotation_matrix(2, 1) * point_curr_[1] +
                                rotation_matrix(2, 2) * point_curr_[2] + pose[5];


            /*
            Eigen::Matrix<T, 3, 1> jm = (closest_last_ - min2_last_).template cast<T>();
            Eigen::Matrix<T, 3, 1> jl = (closest_last_ - min3_last_).template cast<T>();
            Eigen::Matrix<T, 3, 1> normal = jl.cross(jm).template  cast<T>();
            */
            T jm[3];
            jm[0] = T(closest_last_[0]) - T(min2_last_[0]);
            jm[1] = T(closest_last_[1]) - T(min2_last_[1]);
            jm[2] = T(closest_last_[2]) - T(min2_last_[2]);
            T jl[3];
            jl[0] = T(closest_last_[0]) - T(min3_last_[0]);
            jl[1] = T(closest_last_[1]) - T(min3_last_[1]);
            jl[2] = T(closest_last_[2]) - T(min3_last_[2]);

            T normal[3];
            normal[0] = jm[1] * jl[2] - jm[2] * jl[1];
            normal[1] = jm[2] * jl[0] - jm[0] * jl[2];
            normal[2] = jm[0] * jl[1] - jm[1] * jl[0];
            //Eigen::Matrix<T, 3, 1> normal = jl.cross(jm).template  cast<T>();

            /*
            Eigen::Matrix<T, 3, 1> normal;
            normal[0] = jm[1] * jl[2] - jm[2] * jl[1];
            normal[1] = jm[2] * jl[0] - jm[0] * jl[2];
            normal[2] = jm[0] * jl[1] - jm[1] * jl[0];
             */
            T normal_norm;
            normal_norm = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            //Eigen::Matrix<T, 3, 1> ij;
            T ij[3];
            ij[0] = processed_curr[0] - T(closest_last_[0]);
            ij[1] = processed_curr[1] - T(closest_last_[1]);
            ij[2] = processed_curr[2] - T(closest_last_[2]);

            //ij = processed_curr.template  cast<T>() - closest_last_.template  cast<T>();
            //residual[0] = ij[0] * normal[0] + ij[1] * normal[1] + ij[2] * normal[2] / normal_norm;
            residual[0] = (ij[0] * normal[0] + ij[1] * normal[1] + ij[2] * normal[2]) / normal_norm;

            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector3d point_curr, const Eigen::Vector3d closest_last,
                                           const Eigen::Vector3d min2_last, const Eigen::Vector3d min3_last) {
            return (new ceres::AutoDiffCostFunction<SurfFunctor, 1, 6>(
                    new SurfFunctor(point_curr, closest_last, min2_last, min3_last)));
        }

        const Eigen::Vector3d point_curr_;
        const Eigen::Vector3d closest_last_;
        const Eigen::Vector3d min2_last_;
        const Eigen::Vector3d min3_last_;
    };


    struct GroundFunctor {
        GroundFunctor(const Eigen::Vector3d &point_curr,
                      const Eigen::Vector3d &closest_last,
                      const Eigen::Vector3d &min2_last,
                      const Eigen::Vector3d &min3_last) :
                point_curr_(point_curr), closest_last_(closest_last),
                min2_last_(min2_last), min3_last_(min3_last) {};

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
            processed_curr[0] = rotation_matrix(0, 0) * point_curr_[0] + rotation_matrix(0, 1) * point_curr_[1] +
                                rotation_matrix(0, 2) * point_curr_[2] + pose[3];
            processed_curr[1] = rotation_matrix(1, 0) * point_curr_[0] + rotation_matrix(1, 1) * point_curr_[1] +
                                rotation_matrix(1, 2) * point_curr_[2] + pose[4];
            processed_curr[2] = rotation_matrix(2, 0) * point_curr_[0] + rotation_matrix(2, 1) * point_curr_[1] +
                                rotation_matrix(2, 2) * point_curr_[2] + pose[5];

            T jm[3];
            jm[0] = T(closest_last_[0]) - T(min2_last_[0]);
            jm[1] = T(closest_last_[1]) - T(min2_last_[1]);
            jm[2] = T(closest_last_[2]) - T(min2_last_[2]);
            T jl[3];
            jl[0] = T(closest_last_[0]) - T(min3_last_[0]);
            jl[1] = T(closest_last_[1]) - T(min3_last_[1]);
            jl[2] = T(closest_last_[2]) - T(min3_last_[2]);

            T normal[3];
            normal[0] = jm[1] * jl[2] - jm[2] * jl[1];
            normal[1] = jm[2] * jl[0] - jm[0] * jl[2];
            normal[2] = jm[0] * jl[1] - jm[1] * jl[0];

            T normal_norm;
            normal_norm = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);

            T ij[3];
            ij[0] = processed_curr[0] - T(closest_last_[0]);
            ij[1] = processed_curr[1] - T(closest_last_[1]);
            ij[2] = processed_curr[2] - T(closest_last_[2]);

            residual[0] = T(0.5) * (ij[0] * normal[0] + ij[1] * normal[1] + ij[2] * normal[2]) / normal_norm;

            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector3d point_curr, const Eigen::Vector3d closest_last,
                                           const Eigen::Vector3d min2_last, const Eigen::Vector3d min3_last) {
            return (new ceres::AutoDiffCostFunction<GroundFunctor, 1, 6>(
                    new GroundFunctor(point_curr, closest_last, min2_last, min3_last)));
        }

        const Eigen::Vector3d point_curr_;
        const Eigen::Vector3d closest_last_;
        const Eigen::Vector3d min2_last_;
        const Eigen::Vector3d min3_last_;
    };


private:

    pcl::KdTreeFLANN<PointTI>::Ptr kdtree_edge_last_;
    pcl::KdTreeFLANN<PointTI>::Ptr kdtree_surf_last_;
    pcl::KdTreeFLANN<PointTI>::Ptr kdtree_ground_last_;
    std::vector<int> search_id_;     // to store the order of neighbor
    std::vector<float> search_sqdist_;   // to store the squared distance of neighbor and point

    PointCloudTI::Ptr surf_current_;
    PointCloudTI::Ptr surf_last_;
    PointCloudTI::Ptr ground_current_;
    PointCloudTI::Ptr ground_last_;
    PointCloudTI::Ptr edge_current_;
    PointCloudTI::Ptr edge_last_;
    int edge_count_ = 0;

    double *initial_guess_array_;
    double *opt_array_;
    double roll_pre_;
    double pitch_pre_;
    double z_pre_;
    double y_pre_;
    Eigen::Vector3d params_pre_;
    //[6] = {0, 0, 0, 0, 0, 0};  TODO:放在构造函数里?
    Eigen::Matrix<double,4,4> final_transform_;
    bool fulfill_tole_flag_;

    const int max_iter_ = 2;

    ceres::Problem problem;
    ceres::Problem problem_opt;

    std::string method_;
    Eigen::Matrix4d initial_pose_;


    //frame-map
    Frame current_frame_;
    double key_frame_distance_ = 2.0;
    int local_frame_num_ = 15;
    std::deque<Frame> local_map_frames_;
    PointCloudTI::Ptr local_map_;


};