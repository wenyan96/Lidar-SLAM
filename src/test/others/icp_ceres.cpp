
#include "icp_ceres.h"
//#include "CPUTimer.h"

std::DEFINE_bool(pointToPlane, false, "pointToPlane");
std::DEFINE_bool(ceres, true, "Also run ceres");

int main(int argc, char *argv[]){
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::vector<Eigen::Vector3d> pts, nor;
    loadXYZ(".../...xyz", pts, nor);

    for(int i=0; i<10; ++i){
        std::cout<< pts[i].transpose() <<"\t";
        std::cout<< nor[i].transpose() << std::endl;
    }

    Eigen::Matrix3Xd ptsMat = vec2mat(pts);
    Eigen::Matrix3Xd norMat = vec2mat(nor);

    Eigen::Quaterniond q= Eigen::Quaterniond::Identity();
    q = q * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX());
    q = q * Eigen::AngleAxisd(1,Eigen::Vector3d::UnitY());
    q = q * Eigen::AngleAxisd(-0.2,Eigen::Vector3d::UnitZ());

    Eigen::Vector4d rot(q.x(),q.y(),q.z(),q.w());

    Eigen::Vector3d tra(.01,-0.01,-0.005);//,0.5,0.01);

    Eigen::Isometry3d Pclean = Translation3d(tra)*Quaterniond(rot);

    Eigen::Isometry3d P = addNoise(Pclean,0.1,0.1);


    Eigen::Matrix3Xd ptsTra = P * ptsMat;
    std::vector<Eigen::Vector3d> ptsTraVec = mat2vec(ptsTra);

    Eigen::Isometry3d Ptest;
    Eigen::Isometry3d PtestCeres;


    //CPUTimer timer;

    if(FLAGS_pointToPlane){
        Eigen::Matrix3Xd norTra = P.linear() * norMat;
        std::vector<Eigen::Vector3d> norTraVec = mat2vec(norTra);
        //timer.tic();
        Ptest = ICP_Closedform::pointToPlane(pts,ptsTraVec,norTraVec);
        //timer.toc("closed");
        //timer.tic();

        if(FLAGS_ceres){
            PtestCeres = ICP_Ceres::pointToPlane_AngleAxis(pts,ptsTraVec,norTraVec);
            //timer.toc("ceres CeresAngleAxis");

        }
    }else{

        //timer.tic();

        if(FLAGS_ceres){
            PtestCeres = ICP_Ceres::pointToPoint_CeresAngleAxis(pts,ptsTraVec);
            //timer.toc("ceres CeresAngleAxis");

        }
    }
}