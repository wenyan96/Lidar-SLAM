cmake_minimum_required(VERSION 2.8)
project(odometry)


#set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_BUILD_TYPE Release )
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories( ../include)
#include_directories("/home/wenyan/Documents/try_ws/include")

#  Boost
find_package(Boost COMPONENTS program_options REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#  Pcl
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#  Eigen
include_directories("/usr/local/include/eigen3")

#  Opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#  Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# yaml
#add_library(YAML_LIB include/yaml-cpp/yaml.h )
#find_package(PkgConfig)
#pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
#find_package(yaml-cpp REQUIRED )
#include_directories(${YAML_INCLUDE_DIRS})
#include_directories("/home/wenyan/Documents/MasterThesis/mymt_ws/include/yaml-cpp")
#include_directories("/usr/local/include/yaml-cpp/include/")  #head file
#link_directories("/usr/local/lib/")

# GTest
#find_package(GTest REQUIRED)
#include_directories(${GTEST_INCLUDE_DIRS})

#GFlags
#find_package(GFLAGS REQUIRED)
#include_directories(${GFLAGS_INCLUDE_DIRS})

#GLog
#find_package(GLOG REQUIRED)
#include_directories(${GLOG_INCLUDE_DIRS})


#add_executable(preprocessing src/preprocessing/Preprocessing.cpp test/Preprocessing_test.cpp)
add_executable(feature_extraction src/preprocessing/Preprocessing.cpp src/feature_extraction/FeatureExtraction.cpp src/feature_extraction/FeatureExtraction_interface.cpp
        test/FeatureExtraction_test.cpp)
add_executable(lidar_odometry src/common/utils.cpp src/preprocessing/Preprocessing.cpp src/feature_extraction/FeatureExtraction.cpp src/feature_extraction/FeatureExtraction_interface.cpp
        src/lidar_odometry/LidarOdometry.cpp test/LidarOdometry_test.cpp)
#add_executable(lidar_frame2map src/lidar_odometry/scanMatch.cpp src/preprocessing/Preprocessing.cpp src/feature_extraction/FeatureExtraction.cpp src/feature_extraction/FeatureExtraction_interface.cpp src/lidar_odometry/LidarFrametoMap.cpp test/LidarFrametoMap_test.cpp)


#target_link_libraries(preprocessing  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(feature_extraction  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(lidar_odometry  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
#target_link_libraries(lidar_frame2map  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

