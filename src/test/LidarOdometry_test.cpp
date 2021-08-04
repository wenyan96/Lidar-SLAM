#include "lidar_odometry/LidarOdometry.h"
#include "preprocessing/Preprocessing.h"
#include "feature_extraction/FeatureExtraction_interface.h"
#include "feature_extraction/FeatureExtraction.h"
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <fstream>
#include "common/utils.h"


int main(int argc, char *argv[]) {

    //google::InitGoogleLogging(argv[0]);  // 初始化 glog
    //google::ParseCommandLineFlags(&argc, &argv, true);  // 初始化 gflags

    //std::string path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/";
    std::string path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/";

    std::string posefile = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/lego_ground+surf_uniform_n.txt";
    std::string relative_posefile = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/lego_ground+surf_uniform_n_relative.txt";
    std::string initialfile = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/initial_poses.txt";
    std::ofstream outfile(posefile.c_str());  // outfile for poses
    std::ofstream outfile2(relative_posefile.c_str());
    std::fstream initial;  //
    initial.open(initialfile, std::ios::in);//打开一个file
    std::string buff;

    outfile << 1.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "
            << 0.0 << " " << 1.0 << " " << 0.0 << " " << 0.0 << " "
            << 0.0 << " " << 0.0 << " " << 1.0 << " " << 0.0 << std::endl;

    PointCloudTI::Ptr cloud_last(new PointCloudTI);
    PointCloudTI::Ptr cloud_current(new PointCloudTI);

    PointCloudTI::Ptr surf_less_last(new PointCloudTI); ///
    PointCloudTI::Ptr edge_less_last(new PointCloudTI);
    PointCloudTI::Ptr ground_last(new PointCloudTI);

    PointCloudTI::Ptr surf_flat_current(new PointCloudTI); ///
    PointCloudTI::Ptr edge_sharp_current(new PointCloudTI);
    PointCloudTI::Ptr ground_current(new PointCloudTI);

    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d last_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d current_pose;
    Eigen::Matrix4d step_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d cum_pose = Eigen::Matrix4d::Identity();



    Preprocessing pre;
    for (int m = 0; m < 2000; m++) {
        if (m == 0) {

            pcl::io::loadPCDFile<PointTI>(path + std::to_string(m) + ".pcd", *cloud_last);
            pre.preprocessingExecution(cloud_last);
            ground_last = pre.groundResult();
            auto cloud_segment = pre.segmentResult();
            auto segment_n = pre.segmentNormals();
            FeatureExtraction::index_s index_info;
            index_info.start = pre.getStartInd();
            index_info.end = pre.getEndInd();
            index_info.col = pre.getColInd();
            index_info.range = pre.getSegRangeMat();
            FeatureExtraction fe(cloud_segment, index_info, segment_n);
            fe.runExtraction(4,20,4, "normal");
            edge_less_last = fe.edgeLessSharpResult();
            surf_less_last = fe.surfLessFlatResult();
            std::cout << "less surf point num: " <<surf_less_last->size() << std::endl;
            //PointCloudT::Ptr edge_less_last_T(new PointCloudT);
            //pcl::copyPointCloud(*edge_less_last, *edge_less_last_T);
            //pcl::io::savePCDFileASCII ("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/target_edge.pcd",
            //                           *edge_less_last_T);
            //std::cout << "edge_less_last_T size " << edge_less_last_T->size() <<std::endl;


        } else {
            pcl::console::TicToc timer;
            timer.tic();
            pre.initialization();
            pcl::io::loadPCDFile<pcl::PointXYZI>(path + std::to_string(m) + ".pcd", *cloud_current);
            pre.preprocessingExecution(cloud_current);
            ground_current = pre.groundDsResult(); ///

            auto cloud_segment = pre.segmentResult();
            auto segment_n = pre.segmentNormals();
            FeatureExtraction::index_s index_info;
            index_info.start = pre.getStartInd();
            index_info.end = pre.getEndInd();
            index_info.col = pre.getColInd();
            index_info.range = pre.getSegRangeMat();
            FeatureExtraction fe(cloud_segment, index_info, segment_n);
            fe.runExtraction(4,20,4, "normal");
            edge_sharp_current = fe.edgeSharpResult();  ///
            surf_flat_current = fe.edgeSharpResult();
            std::cout << "surf flat point num: " <<surf_flat_current->size() << std::endl;
            //PointCloudT::Ptr edge_sharp_current_T(new PointCloudT);
            //pcl::copyPointCloud(*edge_sharp_current, *edge_sharp_current_T);
            //pcl::io::savePCDFileASCII ("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/input_edge.pcd",
            //                           *edge_sharp_current_T);
            //std::cout << "edge_sharp_current_T size " << edge_sharp_current_T->size() <<std::endl;

            /*
            getline(initial, buff);
            std::vector<double> nums;
            char *s_input = (char *)buff.c_str();
            const char * split = " ";
            char *p = strtok(s_input, split);
            double a;
            while (p != nullptr) {
                a = atof(p);
                nums.push_back(a);
                p = strtok(nullptr, split);
            }
            for (int j = 0; j < nums.size(); j++) {
                initial_guess(int(j/4), j%4) = nums[j];
            }
             */


            std::cout<< "*****  Current frame - " << m <<std::endl;
            std::cout<< "initial guess: "<< initial_guess <<std::endl;
            LidarOdometry lo;
            lo.setParameters(initial_guess,
                             surf_flat_current, surf_less_last,
                             edge_sharp_current, edge_less_last,
                             ground_current, ground_last);

            std::cout<< "*****  Check - " << m <<std::endl;
            current_pose =  lo.poseSolver("ff_6d") ;   //pose between neighboring two frames
            std::cout<<  "TOTAL TIME : " << timer.toc() << " ms *****" <<std::endl;
            //viewRegistration(cloud_last, cloud_current, current_pose, m);
            std::cout << "current_POSE : "  << current_pose <<std::endl;

            //update
            surf_less_last = fe.surfLessFlatResult();
            edge_less_last = fe.edgeLessSharpResult();
            ground_last = pre.groundResult();

            cloud_last->clear();
            *cloud_last = *cloud_current;
            cloud_current->clear();

            //saving results
            cum_pose = cum_pose * /*initial_guess*/  current_pose;
            initial_guess = current_pose;

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++)
                    if (i == 2 && j == 3) {
                        outfile << (double) cum_pose(i, j) << std::endl;
                        outfile2 << (double) current_pose(i, j) << std::endl;
                    } else {
                        outfile << (double) cum_pose(i, j) << " ";
                        outfile2 << (double) current_pose(i, j) << " ";
                    }
            }


        }
    }
    outfile.close();
    outfile2.close();

}




