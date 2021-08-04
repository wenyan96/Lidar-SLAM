//
// Created by wenyan on 27.02.21.
//

#include "icp_ceres.h"
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>

namespace ICP_Ceres{

    ceres::Solver::Options getOptions(){
        ceres::Solver::Options options;

        options.linear_solver_type = ceres::DENSE_QR;  //TODO densse / sparse??
        options.max_num_iterations = 50;

        std::cout<< "Ceres Solver getOptions()" << std::endl;
        std::cout<< "Ceres preconditioner type: "<<options.preconditioner_type << std::endl;
        std::cout<< "Ceres linear algebra type: "<< options.sparse_linear_algebra_library_type << std::endl;
        std::cout<< "Ceres linear solver type: "<< options.linear_solver_type <<std::endl;

        return options;
    }


    void solve(ceres::Problem &problem){
        ceres::Solver::Summary summary;
        ceres::Solve(getOptions(), &probelm, &summary);
        std::cout<< "Final Report: \n" << summary.FullReport();
    }

    void isoToAngleAxis(const Eigen::Isometry3d& pose, double* cam){
        RotationMatrixToAngleAxis(ColumnMajorAdapter4x3(pose.linear().data(), cam));
        Eigen::Vector3d t(pose.translation());
        cam[3] = t.x();
        cam[4] = t.y();
        cam[5] = t.z();
    }

    Eigen::Isometry3d axisAngleToIso(const double* cam){
        Eigen::Isometry3d poseFinal = Eigen::Isometry3d::Identity();
        Eigen::Matrix3d rot;
        ceres::AngleAxisToRotationMatrix(cam, rot.data());
        poseFinal.linear() = rot;
        poseFinal.translation() = Eigen::Vector3d(cam[3], ccam[4]. cam[5]);
        return poseFinal;
    }

    Eigen::Isometry3d pointToPoint_AngleAxis(std::vector<Eigen::Vector3d>&src, std::vector<Eigen::Vector3d>&dst){
        double cam[6] = {0, 0, 0, 0, 0, 0};
        ceres::Problem problem;

        for(int i=0; i<src.size(); ++i){
            ceres::CostFunction* cost_function = ICPCostFunction::PointToPointError::Create(dst[i], src[i]);
            problem.AddResidualBlock(cost_function, NULL, cam);
        }

        solve(problem);

        return axisAngleToIso(cam);
    }

    Eigen::Isometry3d pointToPlane_AngleAxis(std::vector<Eigen::Vector3d> &src,
                                             std::vector<Eigen::Vector3d> &dst,
                                             std::vector<Eigen::Vector3d> &nor){

        double cam[6] = {0,0,0,0,0,0};

        ceres::Problem problem;

        for (int i = 0; i < src.size(); ++i) {
            // first viewpoint : dstcloud, fixed
            // second viewpoint: srcCloud, moves
            // nor is normal of dst
            ceres::CostFunction* cost_function = ICPCostFunctions::PointToPlaneError_AngleAxis::Create(dst[i],src[i],nor[i]);
            problem.AddResidualBlock(cost_function, NULL, cam);
        }

        solve(problem);

        return axisAngleToIso(cam);
    }


    void ceresOptimizer_AngleAxis(std::vector<std::shared_ptr<Frame> >& frames, bool pointToPlane, bool robust){

        ceres::Problem problem;
        double* cameras = new double[frames.size()*6];  ///??

        for(int i=0; i<frames.size(); i++){
            isoToAngleAxis(frames[i]->pose, &cameras[i*6]);

            if(i==0){
                frames[i]->fixed=true;
            }
        }

        //add edges
        for(int src_id = 0; src_id<frames.size(); src_id++){

            Frame& srcCloud = *frames[src_id];
            if(srcCloud.fixed)
                continue;

            for(int j=0; j<srcCloud.neighbors.size(); ++j){

                OutgoingEdge& dstEdge = srcCloud.neighbor[j];
                Frame& dstCloud = *frames.at(dstEdge.neighborIdx);

                int dst_id = dstEdge.neighborIdx;

                for(auto corr: dstEdge.correspondances){
                    if(pointToPlane){
                        cost_function = ICPCostFunction::PointToPlaneError_AngleAxis::Create(dstCloud.pts[corr.second],
                                                                                             srcCloud.pts[corr.first],
                                                                                             dstCloud.nor[corr.second]);

                    }
                    else{
                        cost_function = ICPCostFunction::PointToPointError::Create(dstCloud.pts[corr.second],
                                                                                   srcCloud.pts[corr.first]);

                    }

                    ceres::LossFunction* loss = NULL;
                    if(robust)
                        loss = new ceres::SoftLOneLoss(dstEdge.weight);   ///?

                    problem.AddResidualBlock(cost_function, loss, &cameras[src_id * 6], &cameras[dst_id*6]);
                }
            }
        }

        for(int i=0; i<frames.size(); ++i){
            if(frames[i]->fixed){
                std::cout<< i << " fixed" << std::endl;
                problem.SetParameterBlockConstant(&cameras[i*6]);
            }
        }

        solve(problem);

        //update cam poses
        for(int i=0; i<frames.size(); ++i){
            frames[i] ->pose = axsiAngleToIso(&caeras[i*6]);
        }
    }


}

