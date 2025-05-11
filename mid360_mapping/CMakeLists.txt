cmake_minimum_required(VERSION 3.14)

# 打开调试模式，显示更多信息
set(CMAKE_VERBOSE_MAKEFILE ON)
message(STATUS "开始配置MID360 3D建图系统...")

# 添加vcpkg工具链文件设置
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake")
endif()
message(STATUS "使用工具链文件: ${CMAKE_TOOLCHAIN_FILE}")

project(MID360_3D_Mapping VERSION 0.1 LANGUAGES CXX)
message(STATUS "项目名称: ${PROJECT_NAME}, 版本: ${PROJECT_VERSION}")

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
message(STATUS "Qt路径: $ENV{QTDIR}")

set(CMAKE_PREFIX_PATH $ENV{QTDIR})

# 为MSVC添加必要的编译选项
if(MSVC)
  add_compile_options(/Zc:__cplusplus)
  # 添加Unicode支持，解决中文路径问题
  add_compile_options(/utf-8)
  message(STATUS "为MSVC编译器添加了必要的编译选项")
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
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# 查找必要的包
message(STATUS "正在查找Qt6...")
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets)
message(STATUS "找到Qt6: ${Qt6_FOUND}")

# 查找VTK，指定组件并REQUIRED
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    FiltersSources
    RenderingCore
    RenderingOpenGL2
    GUISupportQt
)
include_directories(${VTK_INCLUDE_DIRS})
message(STATUS "找到VTK: ${VTK_FOUND}")

# 打印Qt路径信息以便调试
message(STATUS "Qt6_DIR: ${Qt6_DIR}")
message(STATUS "Qt6Core_DIR: ${Qt6Core_DIR}")
message(STATUS "Qt6 VERSION: ${Qt6_VERSION}")
message(STATUS "QTDIR: $ENV{QTDIR}")

# 首先查找Eigen3，因为PCL依赖它
message(STATUS "正在查找Eigen3...")
find_package(Eigen3 REQUIRED)
message(STATUS "找到Eigen3: ${EIGEN3_FOUND}, 版本: ${EIGEN3_VERSION}")

# 查找其他依赖库
message(STATUS "正在查找其他依赖库...")
find_package(LZ4 QUIET)
find_package(nlohmann_json QUIET)
find_package(pugixml QUIET)
find_package(VTK QUIET)
message(STATUS "VTK: ${VTK_FOUND}")

message(STATUS "正在查找Boost...")
find_package(Boost REQUIRED COMPONENTS system filesystem)
message(STATUS "找到Boost: ${Boost_FOUND}, 版本: ${Boost_VERSION}")

# 然后查找PCL - 明确指定需要的组件
message(STATUS "正在查找PCL...")
find_package(PCL REQUIRED COMPONENTS
    common
    io
    filters
    visualization
    registration
    kdtree
    search
    sample_consensus
    features
)
message(STATUS "找到PCL: ${PCL_FOUND}, 版本: ${PCL_VERSION}")

# 添加SDK路径 - 使用新的目录结构
set(SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Livox-SDK2")
set(FAST_LIO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/FAST-LIO")
set(OH_MY_LOAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/oh_my_loam")
message(STATUS "SDK_DIR: ${SDK_DIR}")
message(STATUS "FAST_LIO_DIR: ${FAST_LIO_DIR}")
message(STATUS "OH_MY_LOAM_DIR: ${OH_MY_LOAM_DIR}")

# 检查SDK库文件是否存在
set(SDK_LIB_PATH "${SDK_DIR}/build/sdk_core/Release/livox_lidar_sdk_static.lib")
if(EXISTS "${SDK_LIB_PATH}")
    message(STATUS "找到SDK静态库: ${SDK_LIB_PATH}")
else()
    message(WARNING "未找到SDK静态库: ${SDK_LIB_PATH}")
endif()

# 包含SDK头文件
include_directories(
    ${SDK_DIR}/include
    ${SDK_DIR}/sdk_core
    ${FAST_LIO_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fast_lio
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fastlio2
    ${Qt6Core_INCLUDE_DIRS}
    ${Qt6Gui_INCLUDE_DIRS}
    ${Qt6Widgets_INCLUDE_DIRS}
    ${Qt6OpenGLWidgets_INCLUDE_DIRS}
    "D:/vcpkg/installed/x64-windows/include"
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/lidar
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/slam
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/oh_my_loam
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/oh_my_loam/common
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/oh_my_loam/common/pcl
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/oh_my_loam/oh_my_loam
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/oh_my_loam/oh_my_loam/base
)
message(STATUS "已配置include目录")

# 链接SDK库目录
link_directories(
    ${SDK_DIR}/build/sdk_core/Release
    ${PCL_LIBRARY_DIRS}
    "D:/vcpkg/installed/x64-windows/lib"
)
message(STATUS "已配置链接目录")

# 确保资源文件路径正确
set(RESOURCE_FILES "")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc")
    list(APPEND RESOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc")
    message(STATUS "找到资源文件: ${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc")
else()
    message(WARNING "资源文件不存在: ${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/resources.qrc")
    list(APPEND RESOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/resources.qrc")
    message(STATUS "找到UI资源文件: ${CMAKE_CURRENT_SOURCE_DIR}/src/ui/resources.qrc")
else()
    message(WARNING "UI资源文件不存在: ${CMAKE_CURRENT_SOURCE_DIR}/src/ui/resources.qrc")
endif()

message(STATUS "使用的资源文件: ${RESOURCE_FILES}")

# 添加源文件 - 使用新的目录结构
message(STATUS "正在查找源文件...")
file(GLOB_RECURSE APP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/app/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/app/*.h")
file(GLOB_RECURSE LIDAR_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/core/lidar/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/core/lidar/*.h")
file(GLOB_RECURSE SLAM_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/core/slam/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/core/slam/*.h")
file(GLOB_RECURSE UTILS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/core/utils/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/core/utils/*.h")
file(GLOB_RECURSE UI_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/*.h")

set(PROJECT_SOURCES
    ${APP_SOURCES}
    ${LIDAR_SOURCES}
    ${SLAM_SOURCES}
    ${UTILS_SOURCES}
    ${UI_SOURCES}
)

# 打印源文件列表，用于调试
message(STATUS "APP_SOURCES: ${APP_SOURCES}")
message(STATUS "LIDAR_SOURCES: ${LIDAR_SOURCES}")
message(STATUS "SLAM_SOURCES: ${SLAM_SOURCES}")
message(STATUS "UTILS_SOURCES: ${UTILS_SOURCES}")
message(STATUS "UI_SOURCES: ${UI_SOURCES}")

# 打印PCL库的详细信息
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")

# 添加PCL的定义
add_definitions(${PCL_DEFINITIONS})

# 在 add_executable 之前添加 oh_my_loam 子目录
message(STATUS "正在添加 oh_my_loam 子目录...")
add_subdirectory(third_party/oh_my_loam)

# 添加可执行文件
add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
    ${RESOURCE_FILES}
)
message(STATUS "已配置可执行文件: ${PROJECT_NAME}")

# 开发阶段自动复制 config 目录到输出目录
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/config"
    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/config"
)

# 手动指定所有需要的PCL库路径
set(PCL_LIBS_MANUAL "")
set(PCL_LIB_NAMES
    pcl_common
    pcl_io
    pcl_filters
    pcl_visualization
    pcl_io_ply
    pcl_registration
    pcl_kdtree
    pcl_search
    pcl_sample_consensus
    pcl_features
)

foreach(LIB_NAME ${PCL_LIB_NAMES})
    set(LIB_PATH "D:/vcpkg/installed/x64-windows/lib/${LIB_NAME}.lib")
    if(EXISTS "${LIB_PATH}")
        list(APPEND PCL_LIBS_MANUAL "${LIB_PATH}")
        message(STATUS "找到PCL库: ${LIB_PATH}")
    else()
        message(WARNING "未找到PCL库: ${LIB_PATH}")
    endif()
endforeach()

# 检查SDK静态库
if(EXISTS "${SDK_LIB_PATH}")
    message(STATUS "正在链接SDK静态库: ${SDK_LIB_PATH}")
    target_link_libraries(${PROJECT_NAME} PRIVATE "${SDK_LIB_PATH}")
else()
    message(WARNING "SDK静态库不存在，尝试使用动态库")
    set(SDK_SHARED_LIB_PATH "${SDK_DIR}/build/sdk_core/Release/livox_lidar_sdk_shared.lib")
    if(EXISTS "${SDK_SHARED_LIB_PATH}")
        message(STATUS "正在链接SDK动态库: ${SDK_SHARED_LIB_PATH}")
        target_link_libraries(${PROJECT_NAME} PRIVATE "${SDK_SHARED_LIB_PATH}")
    else()
        message(FATAL_ERROR "无法找到SDK库文件，请先构建SDK")
    endif()
endif()

# 链接Qt库
message(STATUS "正在链接Qt库...")
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGLWidgets
)

# 链接PCL库
message(STATUS "正在链接PCL库...")
if(PCL_LIBS_MANUAL)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBS_MANUAL})
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
endif()

# 链接Boost库
message(STATUS "正在链接Boost库...")
target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})

# 链接VTK库
message(STATUS "正在链接VTK库...")
target_link_libraries(${PROJECT_NAME} PRIVATE ${VTK_LIBRARIES})

# 链接oh_my_loam库
message(STATUS "正在链接oh_my_loam库...")
target_link_libraries(${PROJECT_NAME} PRIVATE oh_my_loam_lib)

# 生成qt.conf文件
file(WRITE "${CMAKE_BINARY_DIR}/qt.conf" "[Paths]\nPlugins = ./plugins\n")
message(STATUS "已生成qt.conf")

# 直接复制所有必要的DLL文件
if(WIN32)
  message(STATUS "配置Windows特定设置...")
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
    message(STATUS "Qt安装路径: ${QT_INSTALL_PATH}")
    
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
  endif()
endif()

# 安装目标
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)

# 确认配置目录是否存在
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    # 安装配置文件
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config/"
            DESTINATION "bin/config"
    )
    message(STATUS "找到配置目录: ${CMAKE_CURRENT_SOURCE_DIR}/config")
else()
    message(WARNING "配置目录不存在: ${CMAKE_CURRENT_SOURCE_DIR}/config")
    # 创建空的配置目录
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

# 设置输出目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# 显示编译提示信息
message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message(STATUS "PROJECT_NAME: ${PROJECT_NAME}")
message(STATUS "SDK_DIR: ${SDK_DIR}")
message(STATUS "FAST_LIO_DIR: ${FAST_LIO_DIR}")

message(STATUS "配置完成！") 