
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/time.h>
#include <eigen3/Eigen/Core>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char** argv){

    //create pointcloud pointer
    PointCloudT::Ptr cloud_in_source(new PointCloudT); //original source cloud
    PointCloudT::Ptr cloud_in_trans(new PointCloudT); //used to calculate the transformed source cloud
    //clouds for processing
    PointCloudT::Ptr cloud_source(new PointCloudT);
    PointCloudT::Ptr cloud_target(new PointCloudT);//the target output point cloud, last frame;
    PointCloudT::Ptr cloud_icp(new PointCloudT); //after icp

    pcl::StopWatch timer;

    //Read the pcd file
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/1.pcd", *cloud_target)==-1){
        PCL_ERROR("couldn't read target file1 \n");
        return (-1);
    }
    std::cout<<"Loaded "<< cloud_target->size() <<" data points from target file1"<< std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/5.pcd", *cloud_source)== -1)
    {
        PCL_ERROR("Couldn't read source file2 \n");
        return (-1);
    }

    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/5.pcd", *cloud_in_trans);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wenyan/Documents/MasterThesis/mymt_ws/dataset/out_0002_sync/5.pcd", *cloud_in_source);
    std::cout << "Loaded " << cloud_source->size() << " data points from source file2" << std::endl;
    std::cout << "Loaded " << cloud_in_source->size() << " data points from source file2" << std::endl;


    //Remove NaN points with x, y, or z equal to NaN.
    std::vector<int> mapping_in;  //to save the indices
    std::vector<int> mapping_out;
    pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, mapping_in);
    pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, mapping_out);
    std::cout<< "remove *cloud_source nan\n  size:"<<mapping_in.size() <<std::endl;
    std::cout<<"Remain "<< cloud_target->size() <<" data points of target file after NaNremoval"<< std::endl;


    //Perform actual filtering: for each point, retrieves the number of neighbors within a certain
    //radius. The point will be considered an outlier if it has too few neighbors
    /*
    pcl::RadiusOutlierRemoval<PointT> outrem;
    //build the filter
    outrem.setInputCloud(cloud_source);
    outrem.setRadiusSearch(4);
    outrem.setMinNeighborsInRadius(80);
    outrem.filter(*cloud_source);   //the result of outlier removal
    std::cout<<"Remain "<< cloud_source->size() <<" data points of target file after Radius outlierRemoval"<< std::endl;

    //build the filter
    outrem.setInputCloud(cloud_target);
    outrem.setRadiusSearch(4);
    outrem.setMinNeighborsInRadius(80);
    outrem.filter(*cloud_target);
    std::cout<<"Remain "<< cloud_target->size() <<" data points of target file after Radius outlierRemoval"<< std::endl;
    */



    //Downsampling clouds
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_source);
    vg.setLeafSize(1.0f, 1.0f, 1.0f);
    vg.filter(*cloud_source);

    vg.setInputCloud(cloud_target);
    vg.setLeafSize(0.8f, 0.8f, 0.8f);
    vg.filter(*cloud_target);
    std::cout<<"Remain "<< cloud_source->size() <<" data points of input file after Downsampling"<< std::endl;
    std::cout<<"Remain "<< cloud_target->size() <<" data points of target file after Downsampling"<< std::endl;
    std::cout << "Remain " << cloud_in_source->size() << " data points from source file2" << std::endl;

    timer.reset();


    //icp??????
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp1; //??????ICP???????????????ICP??????
    icp1.setMaximumIterations(200);    //????????????????????????iterations=true
    icp1.setMaxCorrespondenceDistance(0.3);  //??????????????????????????????????????????????????????????????????????????????
    icp1.setTransformationEpsilon(1e-6);  //?????????????????????????????????????????????????????????1e-10?????????
    icp1.setEuclideanFitnessEpsilon(1); //?????????????????????????????????????????????????????????
    icp1.setInputCloud(cloud_source); //??????????????????
    icp1.setInputTarget(cloud_target); //???????????????????????????????????????????????????????????????????????????
    icp1.align(*cloud_icp);          //??????????????????


    //icp.setMaximumIterations(1);  // ?????????1??????????????????
    if (icp1.hasConverged())//icp.hasConverged ()=1???true???????????????????????????????????????
    {
        std::cout << "\nICP has converged, score is: " << icp1.getFitnessScore() << std::endl;
        std::cout << "\nICP has converged, Epsilon is: " << icp1.getEuclideanFitnessEpsilon() << std::endl;
        std::cout << "\nICP transformation is \n " << icp1.getFinalTransformation() << std::endl;
        //print4x4Matrix(transformation_matrix);
    }
    else
    {
        PCL_ERROR("\nICP has not converged.\n");
        return (-1);
    }

    std::cout << "ICP run time: " << timer.getTimeSeconds() << " s" << std::endl;

    //Save pcd file
    //pcl::io::savePCDFileASCII("after_icp.pcd", *cloud_icp);


    //Visualization
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    // ????????????????????????
    int v1(0);
    int v2(1);


    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // ???????????????????????????
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    // ?????????????????????????????????
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_aim_color_h(cloud_target, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
                                                                               (int)255 * txt_gray_lvl);

    viewer.addPointCloud(cloud_target, cloud_aim_color_h, "cloud_aim_v1", v1);//??????????????????????????????????????????
    //viewer.addPointCloud(cloud_target, cloud_aim_color_h, "cloud_aim_v2", v2);

    // ???????????????????????????????????????
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in_source, 180, 20, 20);
    viewer.addPointCloud(cloud_in_source, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud(cloud_in_source, cloud_in_color_h, "cloud_in_v2", v2);

    // Eigen::Matrix<float,4,4> Trans = icp1.getFinalTransformation();
    //cloud_in_trans = cloud_in_trans * Trans;  //target = source * trans

    // ICP???????????????????????????
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icped_color_h(cloud_icp, 20, 180, 20);
    viewer.addPointCloud(cloud_icp, cloud_icped_color_h, "cloud_icped_v2", v2);
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icped_color_h(cloud_in_trans, 20, 180, 20);
    //viewer.addPointCloud(cloud_in_trans, cloud_icped_color_h, "cloud_icped_v2", v2);

    // ?????????????????????????????????????????????
    //???????????????viewport=v1??????????????????white?????????"icp_info_1"?????????????????????ID????????????10???15????????????16??????????????? ???????????????RGB???
    viewer.addText("White: the target cloud/ last frame\nRed: the source cloud/ current frame", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("Red: the source cloud/ current frame\nGreen: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    // ??????????????????
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // ??????????????????????????????
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1960, 1024);  // ????????????????????????

    //??????
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}

















