cmake_minimum_required(VERSION 3.10)
project(oh_my_loam)

# 根据编译器选择正确的标志
if(MSVC)
  # MSVC编译器标志
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_compile_options(/W3 /O2)
  add_definitions(-D_USE_MATH_DEFINES)
  # 添加UTF-8支持
  add_compile_options(/utf-8)
else()
  # GCC/Clang编译器标志
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

# 查找依赖包
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# 如果找到OpenMP，启用OpenMP支持
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 确保glog正确包含 - 解决glog头文件错误
add_definitions(-DGOOGLE_GLOG_DLL_DECL=)

# 创建完整版的oh_my_loam库
add_library(oh_my_loam_lib STATIC
  # 日志和配置
  common/log/log.cc
  
  # 特征提取
  oh_my_loam/feature/feature_extraction.cc
  
  # 里程计
  oh_my_loam/odometry/odometry_run.cc
  oh_my_loam/odometry/lidar_factor.cc
  
  # 建图
  oh_my_loam/mapping/mapping_run.cc
  
  # 数学工具
  oh_my_loam/math/math_utils.cc
  oh_my_loam/math/transformation.cc
)

# 添加必要的定义
target_compile_definitions(oh_my_loam_lib PRIVATE
  -DNOMINMAX  # 避免Windows.h中的宏冲突
)

# 设置包含目录 - 使用target_include_directories
target_include_directories(oh_my_loam_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/common
  ${CMAKE_CURRENT_SOURCE_DIR}/common/pcl
  ${CMAKE_CURRENT_SOURCE_DIR}/oh_my_loam
  ${CMAKE_CURRENT_SOURCE_DIR}/oh_my_loam/base
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 链接依赖库
target_link_libraries(oh_my_loam_lib PUBLIC
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  glog::glog
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(oh_my_loam_lib PUBLIC OpenMP::OpenMP_CXX)
endif()

# 设置导出目标
set_target_properties(oh_my_loam_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
)

# 导出头文件
install(DIRECTORY 
  ${CMAKE_CURRENT_SOURCE_DIR}/common
  ${CMAKE_CURRENT_SOURCE_DIR}/oh_my_loam
  DESTINATION include/oh_my_loam
  FILES_MATCHING PATTERN "*.h"
)
