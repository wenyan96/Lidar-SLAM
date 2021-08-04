#include "range_img.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<PointTI> PointCloudTI;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main() {



    PointCloudTI::Ptr cloud_in(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/2.pcd",
                                  *cloud_in);

    //如果进行求导，range_mat初始化为0;如果为了求特征点，初始化为FLT_MAX
    //cv::Mat range_mat = cv::Mat(BeamNum, HorizonNum, CV_32F, cv::Scalar::all(FLT_MAX));
    cv::Mat range_mat = cv::Mat(BeamNum, HorizonNum, CV_64F, cv::Scalar::all(255.0));
    //for(int i=0; i<BeamNum; ++i){
    //    for(int j=0; j<HorizonNum; ++j){
    //        range_mat.at<double>(i,j) = 255.0;
    //    }

    //cv::Mat range_mat = cv::Mat(BeamNum, HorizonNum, CV_32F, cv::Scalar::all(255));

    //pcl::console::TicToc tt;
    //tt.tic();
    RangeImg r1;
    r1.initialize();
    r1.set_params(cloud_in,
                  range_mat);

    r1.remove_ground();  //提取非地面点
    r1.normal_kdtree(false); //对非地面点求法向量
    r1.view_normals();
    r1.to_rangeimg();   //对已知非地面点的点进行投影 法向量也对应到相应的位置
    //r1.to_colormap();


    r1.calculate_smoothness();
    r1.mark_occlusion();
    r1.extract_edgepts(2, 20);
    r1.extract_surfpts(4);
    //std::cout << "[time for extract feature points : " << tt.toc() << " ms ]" << std::endl;
    //[time for extract feature points : 56.5423 ms ]

    PointCloudTI::Ptr edge_sharp(new PointCloudTI);
    PointCloudTI::Ptr edge_less(new PointCloudTI);
    PointCloudTI::Ptr surf_flat(new PointCloudTI);
    PointCloudTI::Ptr surf_less(new PointCloudTI);
    r1.get_result(edge_sharp, edge_less, surf_flat, surf_less);

    //yellow    edge_sharp
    //deepblue  edge_less
    //Red       surf_flat
    //skyblue   surf_less
    std::vector<PointCloudTI::Ptr> clds;
    clds.push_back(cloud_in);
    clds.push_back(edge_sharp);
    clds.push_back(edge_less);
    clds.push_back(surf_flat);
    clds.push_back(surf_less);
    r1.view_clouds(clds, true);
    std::cout<<"Size of egde_sharp: "<<edge_sharp->size()<<"\n";
    std::cout<<"Size of egde_less: "<<edge_less->size()<<"\n";







    //---------------------normal calculate-----------------------

    /*
    PointCloudTI::Ptr cloud_in2(new PointCloudTI);
    pcl::io::loadPCDFile<PointTI>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd",
                                  *cloud_in2);

    //如果进行求导，range_mat初始化为0;如果为了求特征点，初始化为FLT_MAX
    cv::Mat range_mat2 = cv::Mat(BeamNum, HorizonNum, CV_32F, cv::Scalar::all(255));
    RangeImg r2;
    //r2.reset_params();
    r2.initialize();
    r2.set_params(cloud_in2,
                  range_mat2);
    r2.precompute();
    r2.to_rangeimg();
    r2.to_colormap();
    r2.calculate_normal(true);
    r2.view_normals();
    */
}
