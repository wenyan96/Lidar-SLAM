#include "lidar_odometry/LidarFrametoMap.h"
#include "lidar_odometry/scanMatch.h"
#include "preprocessing/Preprocessing.h"
#include "feature_extraction/FeatureExtraction_interface.h"
#include "feature_extraction/FeatureExtraction.h"
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <fstream>
#include "common/utils.h"
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>


int main(){

    std::string path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/velodyne_pcd/";
    std::string relative_posefile = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/lidarpose_fm_1.txt";
    std::string initialfile = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/lidar_gt2000.txt";
    std::ofstream outfile2(relative_posefile.c_str());
    std::fstream initial;  //
    initial.open(initialfile, std::ios::in);//打开一个file
    std::string buff;

    /*outfile << 1.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "
            << 0.0 << " " << 1.0 << " " << 0.0 << " " << 0.0 << " "
            << 0.0 << " " << 0.0 << " " << 1.0 << " " << 0.0 << std::endl;*/

    PointCloudTI::Ptr cloud_current(new PointCloudTI);
    PointCloudTI::Ptr current_frame_edge(new PointCloudTI);
    PointCloudTI::Ptr current_frame_ground(new PointCloudTI);

    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d current_pose = Eigen::Matrix4d::Identity();
    LidarFrametoMap lm; //to save the local map all the time
    Preprocessing pre;
    for (int m = 0; m < 2000; m++) {
        pcl::console::TicToc tt0;
        tt0.tic();
        pcl::io::loadPCDFile<PointTI>(path + std::to_string(m) + ".pcd", *cloud_current);
        pre.initialization();
        pre.preprocessingExecution(cloud_current);
        current_frame_ground = pre.groundResult();
        auto cloud_segment = pre.segmentResult();
        auto segment_n = pre.segmentNormals();
        FeatureExtraction::index_s index_info;
        index_info.start = pre.getStartInd();
        index_info.end = pre.getEndInd();
        index_info.col = pre.getColInd();
        index_info.range = pre.getSegRangeMat();
        FeatureExtraction fe(cloud_segment, index_info, segment_n);
        fe.runExtraction(4, 40, 4);
        current_frame_edge = fe.edgeLessSharpResult();

        //read the gt initial pose
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


        //if initialize the matrix with z = 0;
        //initial_guess(2,3) = 0;

        lm.setParameters(current_frame_ground, current_frame_edge, initial_guess);
        current_pose = lm.solveTransform();   //relative to the first frame
        std::cout << "===" <<m<< " Frame===" <<"\n"<< current_pose <<std::endl;
        std::cout << "=== Time: " << tt0.toc() << "ms" << " ===" <<std::endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++)
                if (i == 2 && j == 3) {
                    outfile2 << (double) current_pose(i, j) << std::endl;
                } else {
                    outfile2 << (double) current_pose(i, j) << " ";
                }
        }
    }

}
