#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>


int main() {


    Eigen::Matrix<double, 4, 4> Tr;
    Tr << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
        -7.210626507497e-03,  8.081198471645e-03,  -9.999413164504e-01,  -5.403984729748e-02,
         9.999738645903e-01,  4.859485810390e-04,  -7.206933692422e-03,  -2.921968648686e-01,
         0 , 0,   0,  1;

    Eigen::Matrix<double, 4, 4> Tr_inv = Tr.inverse();
    Eigen::Matrix<double, 4, 4> cam2start(Eigen::MatrixXd::Identity(4,4));
    Eigen::Matrix<double, 4, 4> lidar2start_gt;


    //读取cam2start
    std::string read_out = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/data_odometry_labels/dataset/sequences/00/poses.txt";
    std::fstream in;
    in.open(read_out, std::ios::in);//打开一个file

    //写入
    std::string write_in = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/sequence00/odo_lidar_gt_try.txt";
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
            cam2start(int(j/4), j%4) = nums[j];
        }

        //读入了cam相对第一帧的矩阵后，进行运算并写入txt文档
        lidar2start_gt = Tr_inv * cam2start * Tr ;


        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++)
                if (i == 2 && j == 3) {
                    outfile << (double) lidar2start_gt(i, j) << std::endl;
                } else {
                    outfile << (double) lidar2start_gt(i, j) << " ";
                }
        }
        k++;

        //if(k==2000){
        //    break;
        //}
    }

    outfile.close();
    in.close();

}