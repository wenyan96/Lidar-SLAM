#include "registration_frames.h"
#include "range_img.h"
#include <immintrin.h>

//firstly, find the feature points
//secondly, use these feature points to do the registration


#include "registration_frames.h"
#include <pcl/console/time.h>
#include "range_img.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <string>
#include <fstream>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char *argv[]) {


    PointCloudTI::Ptr cloud_src(new PointCloudTI);
    PointCloudTI::Ptr cloud_target(new PointCloudTI);
    std::string path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/";

    std::string posefile = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/lidarpose_regis2.txt";
    std::ofstream outfile(posefile.c_str());
    outfile << 1.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "
            << 0.0 << " " << 1.0 << " " << 0.0 << " " << 0.0 << " "
            << 0.0 << " " << 0.0 << " " << 1.0 << " " << 0.0 << std::endl;

    Eigen::Matrix3d tmp_rot;
    Eigen::Vector3d tmp_trans;  //between 2 frame
    Eigen::Matrix3d rot(Eigen::Matrix3d::Identity());
    Eigen::Vector3d trans(Eigen::Vector3d::Zero());
    Eigen::Matrix<double, 3, 4> pose_result;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d last_pose;
    Eigen::Matrix4d current_pose;
    Eigen::Matrix4d step_pose = Eigen::Matrix4d::Identity();
    for (int i = 1; i < 1001; i++) {
        pcl::io::loadPCDFile<pcl::PointXYZI>(path + std::to_string(i) + ".pcd", *cloud_target);
        pcl::io::loadPCDFile<pcl::PointXYZI>(path + std::to_string(i + 1) + ".pcd", *cloud_src);

        std::vector<PointCloudTI::Ptr> CloudFrames;
        CloudFrames.push_back(cloud_target);
        CloudFrames.push_back(cloud_src);

        std::vector<PointCloudTI::Ptr> more_feature;
        std::vector<PointCloudTI::Ptr> less_feature;
        std::vector<std::vector<Eigen::Vector3d> > normals;


        for (int j = 0; j < CloudFrames.size(); ++j) {
            //如果进行求导，range_mat初始化为0;如果为了求特征点，初始化为FLT_MAX ///?
            cv::Mat range_mat = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(255.0));

            pcl::console::TicToc tt;
            tt.tic();
            RangeImg r1{range_mat, false};
            r1.initialize(CloudFrames[j]);
            r1.run_extraction();
            //std::cout << "[time for extract feature points : " << tt.toc() << " ms ]" << std::endl;
            auto lfeature = r1.get_feature4src();
            auto mfeature = r1.get_feature4target();
            auto normal = r1.get_normalvector();

            more_feature.push_back(mfeature);
            less_feature.push_back(lfeature);
            normals.push_back(normal);

        }

        RangeImg r2;
        auto target_pts = r2.ptptr2vec(cloud_target);
        auto src_pts = r2.ptptr2vec(cloud_src);
        auto target_pts_fps = r2.ptptr2vec(more_feature[0]);
        auto src_pts_fps = r2.ptptr2vec(more_feature[1]);

        if (i == 1) {
            last_pose = initial_guess;
        }

        //保存上一循环求出的对于第一帧的位姿


        pcl::console::TicToc tt;
        tt.tic();
        RegistrationFrames reg{50, initial_guess, src_pts_fps, target_pts_fps, normals[0]};
        reg.get_pose();
        //std::cout << "[Time for Registration: " << tt.toc() << "ms]" << "\n";
        //std::cout << "src points counts: " << src_pts_fps.size() << "\n";
        tmp_rot = reg.get_rotation();
        tmp_trans = reg.get_translation();
        current_pose = reg.get_current_pose();

        step_pose = last_pose.inverse() * current_pose;  //current是求解出来的当前的位姿;
        initial_guess = current_pose * step_pose;
        last_pose = current_pose;

        rot *= tmp_rot;
        trans += tmp_trans;
        //pose *= current_pose;
        pose_result.block<3, 3>(0, 0) = rot;
        pose_result.block<3, 1>(0, 3) = trans;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++)
                if (i == 2 && j == 3) {
                    outfile << (double) pose_result(i, j) << std::endl;
                } else {
                    outfile << (double) pose_result(i, j) << " ";
                }
        }

        //可视化初始情况 匹配前
        //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("before Viewer"));
        //viewer->setBackgroundColor(0, 0, 0);
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud_target, 0, 255, 0);
        //viewer->addPointCloud<pcl::PointXYZI>(cloud_target, color, "target cloud");
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color2(cloud_src, 220, 0, 0);
        //viewer->addPointCloud<pcl::PointXYZI>(cloud_src, color2, "src cloud");
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "src cloud");
        //viewer->addCoordinateSystem(3.0);
        //viewer->setBackgroundColor(0, 0, 0);
        //while (!viewer->wasStopped()) {
        //    viewer->spinOnce(50);
        //}

        //reg.view_registration(src_pts, cloud_target);
    }

    outfile.close();

}
