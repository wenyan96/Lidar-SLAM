#include "registration_frames.h"
#include "range_img.h"

//firstly, find the feature points
//secondly, use these feature points to do the registration


#include "registration_frames.h"
#include <pcl/console/time.h>
#include "range_img.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <string>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(){


    std::vector<Eigen::Vector3d> fp_localmap;
    std::vector<Eigen::Vector3d> fp_localmap_normals;
    std::vector<Eigen::Vector3d> src_fpts_vector;
    PointCloudTI::Ptr cloud_visualization(new PointCloudTI);

    int set_f = 4;
    std::string folder_path = "/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/";
    PointCloudTI::Ptr cloud_target(new PointCloudTI);
    pcl::io::loadPCDFile<pcl::PointXYZI>(folder_path+"1.pcd", *cloud_target);
    *cloud_visualization += (*cloud_target);


    for(int fr=1; fr<set_f; fr++){

        PointCloudTI::Ptr cloud_src(new PointCloudTI);
        pcl::io::loadPCDFile<pcl::PointXYZI>(folder_path+std::to_string(fr+3)+".pcd", *cloud_src);
        std::vector<Eigen::Vector3d> normal;  //法向量构成的vector

        //提取特征点
        if(fr ==1){
            //每(set_f-1)帧需要对其中第一帧target点云进行提取特征点处理，之后就不需要了
            cv::Mat range_mat1 = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(255.0));
            pcl::console::TicToc tt;
            tt.tic();
            RangeImg r1{range_mat1, false};
            r1.initialize(cloud_target);
            r1.run_extraction();
            std::cout << "[time for extract target feature points : " << tt.toc() << " ms ]" << std::endl;
            auto target_fpts = r1.get_feature4target();  //点云指针 - 用于可视化
            auto normal1 = r1.get_normalvector();
            auto target_fpts_vector1 = r1.ptptr2vec(target_fpts); //vector变量 用于之后的匹配
            if(target_fpts_vector1.size() ==0){
                std::cout<<"target fpts vector size 0" <<"\n";
            }
            if(normal1.size() ==0){
                std::cout<<"target normal1 0" <<"\n";
            }
            fp_localmap = target_fpts_vector1;
            fp_localmap_normals = normal1;


            //src
            cv::Mat range_mat2 = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(255.0));
            tt.tic();
            RangeImg r2{range_mat2, false};
            r2.initialize(cloud_src);
            r2.run_extraction();
            std::cout << "[time for extract src feature points : " << tt.toc() << " ms ]" << std::endl;
            auto src_fpts = r2.get_feature4target();   //获得该src的特征点点云
            normal = r2.get_normalvector();
            src_fpts_vector = r2.ptptr2vec(src_fpts); //vector变量 用于之后的匹配

            //auto target_fpts = r2.ptptr2vec(cloud_target);
            //auto src_fpts = r2.ptptr2vec(cloud_src);
        }
        else{
            //src
            cv::Mat range_mat = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(255.0));
            pcl::console::TicToc tt;
            tt.tic();
            RangeImg r2{range_mat, false};
            r2.initialize(cloud_src);
            r2.run_extraction();
            std::cout << "[time for extract src feature points : " << tt.toc() << " ms ]" << std::endl;
            auto src_fpts = r2.get_feature4target();
            normal = r2.get_normalvector();
            src_fpts_vector = r2.ptptr2vec(src_fpts); //vector变量 用于之后的匹配
        }


        //新的一帧与局部特征子地图进行配准
        double initial_pose[6] = {0,0,0,0,0,0};
        pcl::console::TicToc tt;
        tt.tic();
        //check
        if(fp_localmap.size()==0){
            std::cout<<"fp_localmap size 0" <<"\n";
        }
        if(fp_localmap_normals.size()==0){
            std::cout<<"fp_localmap normals 0" << "\n";
        }
        RegistrationFrames reg{30, initial_pose, src_fpts_vector, fp_localmap, fp_localmap_normals};
        reg.get_pose();
        std::cout<<"[2Time for Registration: " <<tt.toc() << "ms]" <<"\n";
        std::cout<<"2src points counts: " <<src_fpts_vector.size()<<"\n";  //变换后坐标

        //求解一个转换后的src特征点云
        auto src_fpts_vec = reg.get_transformedfpts_vec();   //输出真的转化后的vector
        auto src_fpts_nor = reg.get_transformedfpts_nor();
        fp_localmap.insert(fp_localmap.end(), src_fpts_vec.begin(), src_fpts_vec.end());
        std::cout<<"++after insert: fp_localmap " << fp_localmap.size() << std::endl;
        fp_localmap_normals.insert(fp_localmap_normals.end(), src_fpts_nor.begin(), src_fpts_nor.end());
        std::cout<<"++after insert: fp_localmap_normals " << fp_localmap_normals.size() << std::endl;

        auto cloud = reg.get_transformedcloud(cloud_src);
        *cloud_visualization += (*cloud);
        std::cout<<"size of cloud_visualization "<<cloud_visualization->points.size()<<"\n";

        src_fpts_vector.clear();

    }



    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("nframes Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud_visualization, 200, 255, 230);
    viewer->addPointCloud<pcl::PointXYZI> (cloud_visualization, color, "all cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "all cloud");
    viewer->addCoordinateSystem (3.0);
    viewer->setBackgroundColor(0,0,0);
    while (!viewer->wasStopped()){
        viewer->spinOnce(50);
    }




}
