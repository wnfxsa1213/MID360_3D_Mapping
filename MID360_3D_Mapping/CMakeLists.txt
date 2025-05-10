cmake_minimum_required(VERSION 3.14)

# 添加vcpkg工具链文件设置
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake")
endif()

project(MID360_3D_Mapping VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 手动设置Qt路径（如果环境变量中没有QTDIR）
if(NOT DEFINED ENV{QTDIR})
  set(ENV{QTDIR} "D:/QT6/6.9.0/msvc2022_64")  # 修改为MSVC版本
endif()

set(CMAKE_PREFIX_PATH $ENV{QTDIR})

# 为MSVC添加必要的编译选项
if(MSVC)
  add_compile_options(/Zc:__cplusplus)
  # 添加Unicode支持，解决中文路径问题
  add_compile_options(/utf-8)
endif()

# 设置各种依赖库的路径
set(PCL_DIR "D:/vcpkg/installed/x64-windows/share/pcl")
set(Eigen3_DIR "D:/vcpkg/installed/x64-windows/share/eigen3")
set(LZ4_DIR "D:/vcpkg/installed/x64-windows/share/lz4")
set(nlohmann_json_DIR "D:/vcpkg/installed/x64-windows/share/nlohmann_json")
set(pugixml_DIR "D:/vcpkg/installed/x64-windows/share/pugixml")
set(VTK_DIR "D:/vcpkg/installed/x64-windows/share/vtk")

# 强制vcpkg的头文件和库文件路径
list(APPEND CMAKE_PREFIX_PATH "D:/vcpkg/installed/x64-windows")

# 查找必要的包
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets)

# 打印Qt路径信息以便调试
message(STATUS "Qt6_DIR: ${Qt6_DIR}")
message(STATUS "Qt6Core_DIR: ${Qt6Core_DIR}")
message(STATUS "Qt6 VERSION: ${Qt6_VERSION}")
message(STATUS "QTDIR: $ENV{QTDIR}")

# 首先查找Eigen3，因为PCL依赖它
find_package(Eigen3 REQUIRED)

# 查找其他依赖库
find_package(LZ4 QUIET)
find_package(nlohmann_json QUIET)
find_package(pugixml QUIET)
find_package(VTK QUIET)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# 然后查找PCL - 明确指定需要的组件
find_package(PCL REQUIRED COMPONENTS common io filters visualization)

# 添加SDK路径 - 确保使用正确的路径
set(SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SKD2")
set(FAST_LIO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../FAST_LIO_SLAM/FAST-LIO")

# 包含SDK头文件
include_directories(
    ${SDK_DIR}/include
    ${SDK_DIR}/sdk_core
    ${FAST_LIO_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fast_lio
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fastlio2
    ${Qt6Core_INCLUDE_DIRS}
    ${Qt6Gui_INCLUDE_DIRS}
    ${Qt6Widgets_INCLUDE_DIRS}
    ${Qt6OpenGLWidgets_INCLUDE_DIRS}
    "D:/vcpkg/installed/x64-windows/include"
)

# 链接SDK库
link_directories(
    ${SDK_DIR}/build/sdk_core/Release
    ${PCL_LIBRARY_DIRS}
    "D:/vcpkg/installed/x64-windows/lib"
)

# 添加资源文件
set(RESOURCE_FILES
    resources/resources.qrc
)

# 添加源文件
file(GLOB_RECURSE PROJECT_SOURCES
    "src/*.cpp"
    "src/*.c"
    "include/*.h"
)

# 打印PCL库的详细信息
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")

# 添加PCL的定义
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
    ${RESOURCE_FILES}
)

# 手动指定所有需要的PCL库
set(PCL_LIBS_MANUAL
    "D:/vcpkg/installed/x64-windows/lib/pcl_common.lib"
    "D:/vcpkg/installed/x64-windows/lib/pcl_io.lib"
    "D:/vcpkg/installed/x64-windows/lib/pcl_filters.lib"
    "D:/vcpkg/installed/x64-windows/lib/pcl_visualization.lib"
    "D:/vcpkg/installed/x64-windows/lib/pcl_io_ply.lib"
)

# 链接库 - 使用手动指定的PCL库
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGLWidgets
    ${PCL_LIBS_MANUAL}
    ${PCL_LIBRARIES}
    livox_lidar_sdk_static
    ${Boost_LIBRARIES}
)

# 生成qt.conf文件
file(WRITE "${CMAKE_BINARY_DIR}/qt.conf" "[Paths]\nPlugins = ./plugins\n")

# 直接复制所有必要的DLL文件
if(WIN32)
  # 创建一个函数来复制目录中的所有DLL文件
  function(copy_dlls_from_dir SOURCE_DIR TARGET_DIR)
    file(GLOB DLL_FILES "${SOURCE_DIR}/*.dll")
    foreach(DLL_FILE ${DLL_FILES})
      add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${DLL_FILE}"
        "${TARGET_DIR}")
    endforeach()
  endfunction()

  # 创建一个函数来复制目录及其子目录到目标位置
  function(copy_directory_recursively SOURCE_DIR TARGET_DIR)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${SOURCE_DIR}"
      "${TARGET_DIR}")
  endfunction()

  # 复制PCL和依赖库的DLL
  copy_dlls_from_dir("D:/vcpkg/installed/x64-windows/bin" $<TARGET_FILE_DIR:${PROJECT_NAME}>)
  
  # 复制Qt DLL和插件
  if(DEFINED ENV{QTDIR})
    # 获取Qt安装路径
    set(QT_INSTALL_PATH $ENV{QTDIR})
    
    # 复制Qt核心DLL
    copy_dlls_from_dir("${QT_INSTALL_PATH}/bin" $<TARGET_FILE_DIR:${PROJECT_NAME}>)
    
    # 复制qt.conf文件到构建目录
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${CMAKE_BINARY_DIR}/qt.conf"
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>/qt.conf")
    
    # 创建插件目录
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>/plugins/platforms")
      
    # 直接复制平台插件文件
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${QT_INSTALL_PATH}/plugins/platforms/qwindows.dll"
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>/plugins/platforms/qwindows.dll")
    
    # 打印调试信息
    message(STATUS "Qt Platform Plugins Path: ${QT_INSTALL_PATH}/plugins/platforms")
    
    # 复制其他Qt插件目录
    copy_directory_recursively("${QT_INSTALL_PATH}/plugins/styles" 
                             "$<TARGET_FILE_DIR:${PROJECT_NAME}>/plugins/styles")
    copy_directory_recursively("${QT_INSTALL_PATH}/plugins/imageformats" 
                             "$<TARGET_FILE_DIR:${PROJECT_NAME}>/plugins/imageformats")
                             
    # 添加调试信息
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E echo "Copying Qt plugins from ${QT_INSTALL_PATH}/plugins")
  endif()
endif()

# 拷贝配置文件到输出目录
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/config/)

include_directories(
    ${CMAKE_SOURCE_DIR}/../FAST_LIO_SLAM/FAST-LIO/include
    ${CMAKE_SOURCE_DIR}/../FAST_LIO_SLAM/FAST-LIO/src
) 