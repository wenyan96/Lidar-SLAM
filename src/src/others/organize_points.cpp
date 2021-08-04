#include <iostream>
#include <fstream>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common_headers.h>
#include <cmath>
#include <pcl/filters/filter.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/io.hpp>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>


typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;
typedef pcl::PointXYZRGB PointTRGB;
typedef pcl::PointCloud<PointTRGB> PointCloudTRGB;

/*
const int BeamNum = 64;
const int HorizontalNum = 1800;
const float segmentAlphaX = 0.2 / 180.0 * M_PI;
const float segmentAlphaY = 0.42 / 180.0 * M_PI;
const float segmentTheta = 55.0 / 180.0 * M_PI;  // decrease this value may improve accuracy
const int segmentValidPointNum = 5;
const int segmentValidLineNum = 3;
const int groundScanInd = 50;
const int hori_resolution = 0.42;
int label_count = 1;
int sizeofSegCloud = 0;
 */

void displayCloud(PointCloudTI::Ptr cloud, const std::string &window_name) {
    if (cloud->size() < 1) {
        std::cout << window_name << " display failure. Cloud contains no points\n";
        return;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
    pcl::visualization::PointCloudColorHandlerGenericField<PointTI> point_cloud_color_handler(cloud, "intensity");

    viewer->addPointCloud<PointTI>(cloud, point_cloud_color_handler, "id");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "id");
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);

    }

    viewer->close();
}

void groundRemoval(PointCloudTI::Ptr &cloud_t, PointCloudTI::Ptr &cloud_gdseg,
                   const cv::Mat &range_mat, cv::Mat &label_mat, cv::Mat &ground_mat) {
    int lower_id, upper_id;
    float diffX, diffY, diffZ, angle;
    // groundMat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    //从0到60 地面上的线

    //50-BeamNum scanline haven't done the ground check
    for (int j = 0; j < HorizontalNum; ++j) {
        for (int i = 0; i < groundScanInd; ++i) {

            lower_id = j + (i) * HorizontalNum;
            upper_id = j + (i + 1) * HorizontalNum;

            if (cloud_t->points[lower_id].intensity == -1 ||
                cloud_t->points[upper_id].intensity == -1) {
                // no info to check, invalid points
                // can't calculate the angle
                ground_mat.at<int>(i, j) = -1;
                continue;
            }

            diffX = cloud_t->points[upper_id].x - cloud_t->points[lower_id].x;
            diffY = cloud_t->points[upper_id].y - cloud_t->points[lower_id].y;
            diffZ = cloud_t->points[upper_id].z - cloud_t->points[lower_id].z;

            angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

            //std::cout<< "ground_mat_element"<<std::endl;
            // 相邻圈对应点俯仰角小于10度认为是地面点
            if (abs(angle - 0) <= 10) {  //sensorMountAngle=0?
                ground_mat.at<int>(i, j) = 1;
                ground_mat.at<int>(i + 1, j) = 1;
                //std::cout<< ground_mat.at<int>(i,j) << " ";
            }
        }
    }
    // extract ground cloud (groundMat == 1)
    // mark entry that doesn't need to label (ground and invalid point) for segmentation
    // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
    for (int i = 0; i < BeamNum; ++i) {
        for (int j = 0; j < HorizontalNum; ++j) {
            cloud_gdseg->points[i * HorizontalNum + j] = cloud_t->points[i * HorizontalNum + j];
            cloud_gdseg->points[i * HorizontalNum + j].intensity = ground_mat.at<int>(i, j);
            if (ground_mat.at<int>(i, j) == 1 || range_mat.at<float>(i, j) == FLT_MAX) {
                label_mat.at<int>(i, j) = -1;
                //地面点或者深度图中无效点都设为-1，因为他们不需要在后续被聚类标记
            }
        }
    }
}


void label_components(int row, int col, cv::Mat &label_mat, cv::Mat &range_mat) {
    float d1, d2, alpha, angle;
    int from_idx, from_idy, this_idx, this_idy;
    bool lineCountFlag[BeamNum] = {false};

    std::pair<int, int> neighbor;
    std::vector<std::pair<int, int> > neighborIterator;
    neighbor.first = -1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = 1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = -1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    uint16_t *allPushed_idx; // array for tracking points of a segmented object
    uint16_t *allPushed_idy;
    uint16_t *queue_idx; // array for breadth-first search process of segmentation, for speed
    uint16_t *queue_idy;
    queue_idx = new uint16_t[BeamNum * HorizontalNum];  //手写队列，把range放入
    queue_idy = new uint16_t[BeamNum * HorizontalNum];

    queue_idx[0] = row;
    queue_idy[0] = col;
    int queue_size = 1;
    int queue_start = 0;
    int queue_end = 1;

    allPushed_idx = new uint16_t[BeamNum * HorizontalNum];
    allPushed_idy = new uint16_t[BeamNum * HorizontalNum];

    allPushed_idx[0] = row;
    allPushed_idy[0] = col;
    int allPushid_size = 1;

    while (queue_size > 0) {
        //pop point
        from_idx = queue_idx[queue_start];
        from_idy = queue_idy[queue_start];
        --queue_size;
        ++queue_start;

        //mark popped point
        label_mat.at<int>(from_idx, from_idy) = label_count;
        //loop through all neighboring grids of popped grid
        for (const auto& iter : neighborIterator) {
            //new index
            this_idx = from_idx + iter.first;
            this_idy = from_idy + iter.second;
            //index should be within the boundary
            if (this_idx < 0 || this_idx >= BeamNum) continue;

            if (this_idy < 0) this_idy = HorizontalNum - 1;
            if (this_idy > HorizontalNum) this_idy = 0;

            //如果这一个已经计算了，退出
            if (label_mat.at<int>(this_idx, this_idy) != 0) continue;

            d1 = std::max(range_mat.at<float>(from_idx, from_idy),
                          range_mat.at<float>(this_idx, this_idy));
            d2 = std::min(range_mat.at<float>(from_idx, from_idy),
                          range_mat.at<float>(this_idx, this_idy));

            if (iter.first == 0) {
                alpha = segmentAlphaX;
            } else {
                alpha = segmentAlphaY;
            }

            angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

            if (angle > segmentTheta) {
                queue_idx[queue_end] = this_idx;
                queue_idy[queue_end] = this_idy;
                ++queue_size;
                ++queue_end;

                label_mat.at<int>(this_idx, this_idy) = label_count;
                lineCountFlag[this_idx] = true;  //TODO

                allPushed_idx[allPushid_size] = this_idx;
                allPushed_idy[allPushid_size] = this_idy;
                ++allPushid_size;
            }
        }

    }

    //check if the segment is valid
    bool feasibleSegment = false;
    if (allPushid_size >= 30) feasibleSegment = true;  //一个聚类
    else if (allPushid_size >= segmentValidPointNum) {
        //点数大于5 且至少在3条Beam上，也ok
        int line_count = 0;
        for (const bool i : lineCountFlag) {
            if (i) {
                ++line_count;
            }
        }
        if (line_count >= segmentValidLineNum) {
            feasibleSegment = true;
        }
    }

    //if segment is valid, mark these points
    if (feasibleSegment == true) {
        ++label_count;
        std::cout << "label count= " << label_count << std::endl;
    } else {
        for (int i = 0; i < allPushid_size; i++) {
            label_mat.at<int>(allPushed_idx[i], allPushed_idy[i]) = 9999;
            //std::cout<< "feasibleSegment == false" <<std::endl;
        }
    }


}


void cloudSegmentation(PointCloudTI::Ptr &segmentedCloud,
                       PointCloudTI::Ptr &cloud_t,
                       PointCloudTI::Ptr &segmentedCloudPure,
                       PointCloudTI::Ptr &outlier_cloud,
                       cv::Mat &label_mat, cv::Mat &ground_mat, cv::Mat &range_mat,
                       std::vector<float>& segmentedCloud_range,
                       std::vector<int>& segmentedCloud_colind) {
    for (int i = 0; i < BeamNum; i++) {
        for (int j = 0; j < HorizontalNum; j++) {
            //since label for ground/invalid points is -1
            //clustering only for the non-ground points and valid points
            if (label_mat.at<int>(i, j) == 0) label_components(i, j, label_mat, range_mat);
        }
    }

    int segcloud_size = 0;

    for (int i = 0; i < BeamNum; i++) {
        for (int j = 0; j < HorizontalNum; j++) {
            if (label_mat.at<int>(i, j) > 0) {
                if (label_mat.at<int>(i, j) == 9999) {
                    if (i > groundScanInd && j % 5 == 0) {  //how necessary? why j%5?
                        outlier_cloud->push_back(cloud_t->points[j + i * HorizontalNum]);//TODO
                        continue;
                    } else continue;
                }

                //去掉两头
                if (ground_mat.at<int>(i, j) == 1) {
                    if (j % 5 != 0 && j > 5 && j < HorizontalNum - 5) //TODO:TRICK?
                        continue;
                }

                segmentedCloud_range[segcloud_size] = range_mat.at<float>(i,j);
                segmentedCloud_colind[segcloud_size] = j;
                segmentedCloud->push_back(cloud_t->points[j + i * HorizontalNum]); //TODO
                // size of seg cloud
                ++segcloud_size;
            }
        }
    }

    //可视化
    for (int i = 0; i < BeamNum; i++) {
        for (int j = 0; j < HorizontalNum; j++) {
            if (label_mat.at<int>(i, j) > 0 && label_mat.at<int>(i, j) != 9999) {
                segmentedCloudPure->push_back(cloud_t->points[j + i * HorizontalNum]);
                segmentedCloudPure->points.back().intensity = label_mat.at<int>(i, j);
            }
        }
    }
}

struct imu_t{
    float roll;
    float pitch;
    float yaw;
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

struct smoothness_t{
    float value;
    int ind;
};

void calculate_smoothness(PointCloudTI::Ptr& segmentedCloud,
                          const std::vector<float>& segmentedCloud_range){
    std::vector<smoothness_t> cloud_smoothness;
    float* cloud_curvature = new float[BeamNum * HorizontalNum];
    int cloud_size = segmentedCloud->points.size();
    for(int i=5; i<cloud_size-5; i++){
        float diff = segmentedCloud_range[i-5] + segmentedCloud_range[i-4] + segmentedCloud_range[i-3]
                   + segmentedCloud_range[i-2] + segmentedCloud_range[i-1] - 10* segmentedCloud_range[i]
                   + segmentedCloud_range[i+1] + segmentedCloud_range[i+2] + segmentedCloud_range[i+3]
                   + segmentedCloud_range[i+4] + segmentedCloud_range[i+5];

        cloud_curvature[i] = diff * diff;
        cloud_smoothness[i].value = cloud_curvature[i];
        cloud_smoothness[i].ind = i;
    }
}

//阻塞点：点云之间相互遮挡，又靠的很近的点 （？）
void mark_occluded_pts(PointCloudTI::Ptr& segmentedCloud,
                       const std::vector<float>& segmentedCloud_range,
                       std::vector<int>& segmentedCloud_colind){
    int* cloudNeighborPicked = new int[BeamNum * HorizontalNum];
    int cloud_size = segmentedCloud->points.size();
    for(int i=5; i<cloud_size-6; i++){
        float depth1 = segmentedCloud_range[i];
        float depth2 = segmentedCloud_range[i+1];
        int col_diff = std::abs(int(segmentedCloud_colind[i+1] - segmentedCloud_colind[i]));
        if(col_diff < 10){
            // 选择距离较远的那些点，并将他们标记为1
            if(depth1 - depth2 > 0.3){
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }
            else if (depth2 - depth1 > 0.3){
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }

        float diff1 = std::abs(float(segmentedCloud_range[i-1] - segmentedCloud_range[i]));
        float diff2 = std::abs(float(segmentedCloud_range[i+1] - segmentedCloud_range[i]));

        // 选择距离变化较大的点，并将他们标记为1
        if(diff1>0.02 * segmentedCloud_range[i] && diff2>0.02 * segmentedCloud_range[i]){
            cloudNeighborPicked[i] = 1;
        }
    }
}

void extractFeatures()
{
    cornerPointsSharp->GroundScanId();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();

    for (int i = 0; i < BeamNum; i++) {

        surfPointsLessFlatScan->clear();

        for (int j = 0; j < 6; j++) {

            // sp和ep的含义是什么???startPointer,endPointer?
            int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
            int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            // 按照cloudSmoothness.value从小到大排序
            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                // 每次ind的值就是等于k??? 有什么意义?
                // 因为上面对cloudSmoothness进行了一次从小到大排序，所以ind不一定等于k了
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > edgeThreshold &&
                    segInfo.segmentedCloudGroundFlag[ind] == false) {

                    largestPickedNum++;
                    if (largestPickedNum <= 2) {
                        // 论文中nFe=2,cloudSmoothness已经按照从小到大的顺序排列，
                        // 所以这边只要选择最后两个放进队列即可
                        // cornerPointsSharp标记为2
                        cloudLabel[ind] = 2;
                        cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                    } else if (largestPickedNum <= 20) {
                        // 塞20个点到cornerPointsLessSharp中去
                        // cornerPointsLessSharp标记为1
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        // 从ind+l开始后面5个点，每个点index之间的差值，
                        // 确保columnDiff<=10,然后标记为我们需要的点
                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        // 从ind+l开始前面五个点，计算差值然后标记
                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++) {
                int ind = cloudSmoothness[k].ind;
                // 平面点只从地面点中进行选择? 为什么要这样做?
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < surfThreshold &&
                    segInfo.segmentedCloudGroundFlag[ind] == true) {

                    cloudLabel[ind] = -1;
                    surfPointsFlat->push_back(segmentedCloud->points[ind]);

                    // 论文中nFp=4，将4个最平的平面点放入队列中
                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        // 从前面往后判断是否是需要的邻接点，是的话就进行标记
                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        // 从后往前开始标记
                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                if (cloudLabel[k] <= 0) {
                    surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                }
            }
        }

        // surfPointsLessFlatScan中有过多的点云，如果点云太多，计算量太大
        // 进行下采样，可以大大减少计算量
        surfPointsLessFlatScanDS->clear();
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.filter(*surfPointsLessFlatScanDS);

        *surfPointsLessFlat += *surfPointsLessFlatScanDS;
    }
}

int main(int argc, char **argv) {



    // initialize cloud_t and cloud_r
    PointCloudTI::Ptr cloud_t(new PointCloudTI); // cloud intensity - relative time index
    PointCloudTI::Ptr cloud_r(new PointCloudTI); // cloud intensity - lidar range
    cloud_t->points.resize(BeamNum * HorizontalNum);
    cloud_r->points.resize(BeamNum * HorizontalNum);
    PointTI nanPoint;
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;
    PointTRGB nanrgbPoint;
    //nanrgbPoint.x = std::numeric_limits<float>::quiet_NaN();
    //nanrgbPoint.y = std::numeric_limits<float>::quiet_NaN();
    //nanrgbPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanrgbPoint.x = 0;
    nanrgbPoint.y = 0;
    nanrgbPoint.z = 0;
    nanrgbPoint.r = 255;
    nanrgbPoint.g = 255;
    nanrgbPoint.b = 255;
    std::fill(cloud_t->points.begin(), cloud_t->points.end(), nanPoint);
    std::fill(cloud_r->points.begin(), cloud_r->points.end(), nanPoint);


    //
    PointCloudTI::Ptr cloud(new PointCloudTI); // Create a point cloud (pointer)
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd",
                                         *cloud);
    //pcl::io::loadPCDFile<pcl::PointXYZI>("/home/wenyan/Documents/MasterThesis/mymt_ws/ground_i.pcd",
    //                                    *cloud);

    std::cout << "total points= " << cloud->size() << std::endl;
    std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    //std::cout << "remain " << cloud->size() << "points" << std::endl;
    /* 起始和终止角度在去运动畸变中使用
    auto point = cloud->points[0];
    double start_orientation = -std::atan2(point.y, point.x);     // 起始角度radian
    point = cloud->points[cloud->size() - 1];
    double end_orientation = -std::atan2(point.y, point.x) + 2 * M_PI;       //  终止角度 + 360

    //normalization
    if (end_orientation - start_orientation > 3 * M_PI) {     // 起始角度和终止角度 放在0~360之间
        end_orientation -= 2 * M_PI;
    } else if (end_orientation - start_orientation < M_PI) {
        end_orientation += 2 * M_PI;
    }
    double orientationDiff = end_orientation - start_orientation;// 终止与起始角度差

    */
    //cv::Mat range_image = cv::Mat::zeros(cv::Size(BeamNum,HorizontalNum), CV_8UC1);

    //FLT_MAX: 1E+37 or greater
    cv::Mat range_mat = cv::Mat(BeamNum, HorizontalNum, CV_32F, cv::Scalar::all(FLT_MAX));

    int cloud_size = cloud->size();
    int min_z = 0;
    int count_z_2m = 0;
    int count_z_3m = 0;
    int index;

    for (int i = 0; i < cloud_size; i++) {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;

        float xy_range = std::sqrt(x * x + y * y);
        float vertical_angle = std::atan2(z, xy_range) * 180 / M_PI;
        int row_index = (vertical_angle - (-24.9f)) / 0.42;  // 地面index0
        if (row_index < 0 || row_index >= BeamNum) continue;

        float horizontal_angle = std::atan2(x, y) * 180 / M_PI;
        int col_index = -round((horizontal_angle - 90) / 0.42) + HorizontalNum / 2;
        if (col_index >= HorizontalNum) col_index -= HorizontalNum;
        if (col_index < 0 || col_index >= HorizontalNum) continue;

        //too close point is invalid
        float lidar_range = std::sqrt(x * x + y * y + z * z);
        if (z < min_z) {
            min_z = z;
            if (i < cloud_size && z < -2) {
                count_z_2m++;
                if (z < -3) count_z_3m++;
            }
        }

        if (lidar_range < 0.1) continue;

        range_mat.at<float>(row_index, col_index) = lidar_range;
        cloud->points[i].intensity = (float) row_index + (float) col_index / 10000.0;

        index = col_index + row_index * HorizontalNum;
        cloud_t->points[index] = cloud->points[i];
        cloud_r->points[index] = cloud->points[i];
        cloud_r->points[index].intensity = lidar_range;
    }

    double min, max;
    cv::minMaxIdx(range_mat, &min, &max);
    std::cout << "min lidar_range= " << min << std::endl;
    std::cout << "max lidar_range= " << max << std::endl;
    std::cout << "min_z= " << min_z << std::endl;
    std::cout << "count_z_2m= " << count_z_2m << std::endl;
    std::cout << "count_z_3m= " << count_z_3m << std::endl;

    /*
    for (int i = 0; i < 40; i++) {
        for (int j = 200; j < 240; j++) {
            std::cout << range_mat.at<float>(i, j) << ' ';
        }
        std::cout << std::endl;
    }
    */

    //displayCloud(cloud_r, "lidar_range map");
    //displayCloud(cloud_t, "index map: displaying the scanline");
    //cv::Mat adjImg;
    //range_image.convertTo(adjImg, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
    //cv::Mat colors_Img(BeamNum, HorizontalNum, CV_8UC1);
    //cv::applyColorMap(range_image, colors_Img, cv::COLORMAP_JET);
    //cv::imshow("range_image", range_mat);
    //cv::waitKey(0);

    cv::Mat ground_mat = cv::Mat(BeamNum, HorizontalNum, CV_32S, cv::Scalar::all(0));
    cv::Mat label_mat = cv::Mat(BeamNum, HorizontalNum, CV_32S, cv::Scalar::all(0));
    PointCloudTI::Ptr cloud_gdseg(new PointCloudTI);
    cloud_gdseg->points.resize(BeamNum * HorizontalNum);
    std::fill(cloud_gdseg->points.begin(), cloud_gdseg->points.end(), nanPoint);

    groundRemoval(cloud_t, cloud_gdseg, range_mat, label_mat, ground_mat);

    //displayCloud(cloud_gdseg, "ground segmentation map");

    //visualization
    PointCloudTRGB::Ptr cloud_rgb(new PointCloudTRGB);
    cloud_rgb->resize(BeamNum * HorizontalNum);
    std::fill(cloud_rgb->points.begin(), cloud_rgb->points.end(), nanrgbPoint);
    for (int i = 0; i < BeamNum * HorizontalNum; i++) {
        cloud_rgb->points[i].x = cloud_gdseg->points[i].x;
        cloud_rgb->points[i].y = cloud_gdseg->points[i].y;
        cloud_rgb->points[i].z = cloud_gdseg->points[i].z;
        if (cloud_gdseg->points[i].intensity == -1) {
            cloud_rgb->points[i].r = 255;
            cloud_rgb->points[i].g = 0;
            cloud_rgb->points[i].b = 0;
        } else if (cloud_gdseg->points[i].intensity == 0) {
            cloud_rgb->points[i].r = 0;
            cloud_rgb->points[i].g = 255;
            cloud_rgb->points[i].b = 0;
        } else if (cloud_gdseg->points[i].intensity == 1) {
            cloud_rgb->points[i].r = 0;
            cloud_rgb->points[i].g = 0;
            cloud_rgb->points[i].b = 255;
        } else {
            cloud_rgb->points[i].r = 255;
            cloud_rgb->points[i].g = 255;
            cloud_rgb->points[i].b = 255;
        }

    }
    pcl::visualization::PCLVisualizer::Ptr viewer_rgb(new pcl::visualization::PCLVisualizer("Viewer_rgb"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
    viewer_rgb->setBackgroundColor(0, 0, 0);
    viewer_rgb->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, "sample cloud");
    viewer_rgb->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer_rgb->addCoordinateSystem(1.0);
    viewer_rgb->initCameraParameters();

    while (!viewer_rgb->wasStopped()) {
        viewer_rgb->spinOnce(100);
    }


    std::cout << "cloud_t after seg:" << *cloud_gdseg << std::endl;

    cloud_gdseg->width = 1;
    cloud_gdseg->height = cloud_gdseg->points.size();
    pcl::io::savePCDFileASCII<PointTI>("ground_segment.pcd", *cloud_gdseg);
    pcl::io::savePCDFileASCII<PointTRGB>("ground_segment_rgb.pcd", *cloud_rgb);

    PointCloudTI::Ptr segmentedCloud(new PointCloudTI);
    PointCloudTI::Ptr segmentedCloudPure(new PointCloudTI);
    PointCloudTI::Ptr outlier_cloud(new PointCloudTI);
    cloudSegmentation(segmentedCloud, cloud_t, segmentedCloudPure, outlier_cloud,
                      label_mat, ground_mat, range_mat);
    displayCloud(segmentedCloudPure, "clusters: segmentedCloudPure");
    std::cout << "outlier_cloud size= " << outlier_cloud->size() << std::endl;
    displayCloud(segmentedCloud, "clusters: segmentedCloud");
    pcl::io::savePCDFileASCII<PointTI>("clusters_f1.pcd", *segmentedCloudPure);


    //calculate




}








