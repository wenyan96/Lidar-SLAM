//
// Created by wenyan on 02.02.21.
//
#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>

struct imu_t{
    float roll;
    float yaw;
    float pitch;
};

void read_imu(std::fstream &in, std::string filepath, imu_t& imu_angle){
    in.open(filepath, std::ios::in);
    if(!in.is_open()){
        std::cout<< "can't find the imu data file " << filepath << std::endl;
        system("pause");
    }
    std::string buff;
    //int i =0; //行数
    while (getline(in, buff)){
        std::vector<double> nums;
        //string -> char *
        char *s_input = (char* )buff.c_str();
        const char* split = " ";
        char *p = strtok(s_input, split);
        double a;
        while(p!= NULL){
            // char* -> int
            a = atof(p);
            nums.push_back(a);
            p = strtok(NULL, split);
        }

        imu_angle.roll = nums[3];
        imu_angle.pitch = nums[4];
        imu_angle.yaw = nums[5];
        //i++;
    }
    in.close();
}


int main(int argc, char **argv) {
    std::string imupath = "/home/wenyan/Documents/Dataset/kitti/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/oxts/data/0000000000.txt";
    imu_t imu_angle0;
    std::fstream in;
    read_imu(in, imupath, imu_angle0);
    std::cout << imu_angle0.roll << "\n";
    std::cout << imu_angle0.yaw << "\n";
    std::cout << imu_angle0.pitch << "\n";
}