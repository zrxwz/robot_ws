cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# --- 添加 CUDA 配置 ---
option(USE_CUDA "Enable use of CUDA linear algebra solvers." ON)

if (USE_CUDA)
  if (CMAKE_VERSION VERSION_LESS 3.17)
    # 旧版本的 CMake (低于 3.17) 不支持 CUDAToolkit，使用传统的 CUDA 查找方式
    find_package(CUDA QUIET)
    if (CUDA_FOUND)
      message("-- Found CUDA version ${CUDA_VERSION} installed in: "
              "${CUDA_TOOLKIT_ROOT_DIR} via legacy (< 3.17) CMake module. "
              "Using the legacy CMake module means that any installation of "
              "A-LOAM will require that the CUDA libraries be installed in a "
              "location included in the LD_LIBRARY_PATH.")
      enable_language(CUDA)

      # 声明 CUDA 库
      macro(DECLARE_IMPORTED_CUDA_TARGET COMPONENT)
        add_library(CUDA::${COMPONENT} INTERFACE IMPORTED)
        target_include_directories(CUDA::${COMPONENT} INTERFACE ${CUDA_INCLUDE_DIRS})
        target_link_libraries(CUDA::${COMPONENT} INTERFACE ${CUDA_${COMPONENT}_LIBRARY} ${ARGN})
      endmacro()

      # 导入常见的 CUDA 库
      declare_imported_cuda_target(cublas)
      declare_imported_cuda_target(cusolver)
      declare_imported_cuda_target(cusparse)
      declare_imported_cuda_target(cudart ${CUDA_LIBRARIES})

      set(CUDAToolkit_BIN_DIR ${CUDA_TOOLKIT_ROOT_DIR}/bin)

    else (CUDA_FOUND)
      message("-- Did not find CUDA, disabling CUDA support.")
      update_cache_variable(USE_CUDA OFF)
    endif (CUDA_FOUND)
  else (CMAKE_VERSION VERSION_LESS 3.17)
    # 对于 CMake 3.17 及以上，使用 CUDAToolkit 查找
    find_package(CUDAToolkit QUIET)
    if (CUDAToolkit_FOUND)
      message("-- Found CUDA version ${CUDAToolkit_VERSION} installed in: "
              "${CUDAToolkit_TARGET_DIR}")
      set(CUDAToolkit_DEPENDENCY
          "find_dependency(CUDAToolkit ${CUDAToolkit_VERSION})")
      enable_language(CUDA)
      if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.18")
        # 设置支持的 GPU 架构
        set(CMAKE_CUDA_ARCHITECTURES "50;60;70;80")
        message("-- Setting CUDA Architecture to ${CMAKE_CUDA_ARCHITECTURES}")
      endif()
      list(APPEND CERES_CUDA_LIBRARIES
        CUDA::cublas
        CUDA::cudart
        CUDA::cusolver
        CUDA::cusparse)
      set(CMAKE_CUDA_RUNTIME_LIBRARY NONE)
    else (CUDAToolkit_FOUND)
      message("-- Did not find CUDA, disabling CUDA support.")
      update_cache_variable(USE_CUDA OFF)
    endif (CUDAToolkit_FOUND)
  endif (CMAKE_VERSION VERSION_LESS 3.17)
endif (USE_CUDA)
# --- CUDA 配置结束 ---

# 查找所需的 ROS 和其他依赖
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

# 定义并链接可执行文件
add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
