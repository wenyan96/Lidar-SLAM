#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <fstream>


int main() {


    //calculate the ground truth relative pose between two poses
    //original data is based on the first frame

    //f_1n+1 = f_1n * f_nn+1

    Eigen::Matrix4d pose_1_n1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_1_n = Eigen::Matrix4d::Identity(); //初始化为poses的第一行
    Eigen::Matrix4d pose_n_n1 = Eigen::Matrix4d::Identity();

    //读取poses
    std::string read_out = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/odo_lidar_gt_try.txt";
    std::fstream in;
    in.open(read_out, std::ios::in);//打开一个file

    //写入
    std::string write_in = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/initial_poses.txt";
    std::ofstream outfile(write_in.c_str());

    std::string buff;
    int k = 0;//行数i
    while (getline(in, buff)) {

        std::vector<double> nums;
        char *s_input = (char *)buff.c_str();
        const char * split = " ";

        char *p = strtok(s_input, split);
        double a;
        while (p != NULL) {
            a = atof(p);
            nums.push_back(a);
            p = strtok(NULL, split);
        }

        for (int j = 0; j < nums.size(); j++) {
            pose_1_n1(int(j/4), j%4) = nums[j];
        }


        if(k==0){
            pose_1_n = pose_1_n1;
            k++;
            continue;
        }

        //读入了cam相对第一帧的矩阵后，进行运算并写入txt文档
        pose_n_n1 = pose_1_n.inverse() * pose_1_n1;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++)
                if (i == 2 && j == 3) {
                    outfile << (double) pose_n_n1(i, j) << std::endl;
                } else {
                    outfile << (double) pose_n_n1(i, j) << " ";
                }
        }
        pose_1_n = pose_1_n1;
        k++;

        if(k==2000){
            break;
        }
    }

    outfile.close();
    in.close();

}
