# MID360 3D 建图系统

基于Livox MID360激光雷达和Fast-LIO2算法的实时三维建图系统

## 项目介绍

MID360 3D建图系统是一个基于C++和Qt6开发的应用程序，用于连接Livox MID360激光雷达设备，实时采集点云数据，并使用Fast-LIO2算法进行SLAM（同步定位与地图构建）处理。本系统提供了友好的用户界面，支持实时点云可视化、地图保存和加载等功能。

主要特点：
- 支持Livox MID360激光雷达数据采集
- 基于Fast-LIO2算法进行实时SLAM处理
- 使用PCL库进行点云处理和可视化
- 直观的Qt6用户界面
- 支持地图保存/加载功能

## 系统需求

- Windows 10/11操作系统
- Visual Studio 2022
- CMake 3.14+
- Qt 6.9.0+
- PCL 1.13.0+
- Eigen3
- Boost
- Livox SDK2

## 项目结构

项目的主要目录结构如下:

```
MID360_3D_Mapping/
├── build/                  # 构建目录
├── src/                    # 源代码
│   ├── main.cpp            # 程序入口
│   ├── MainWindow.cpp      # 主窗口
│   ├── LidarManager.cpp    # 激光雷达管理
│   ├── FastLioProcessor.cpp # Fast-LIO2处理
│   └── Logger.cpp          # 日志系统
├── include/                # 头文件
│   ├── fast_lio/           # Fast-LIO相关头文件
│   │   └── Pose6D.h        # 位姿表示类
│   ├── fastlio2/           # Fast-LIO2接口
│   │   └── FastLio2.h      # 算法接口
│   └── MainWindow.h        # 主窗口头文件
├── resources/              # 资源文件
│   └── resources.qrc       # Qt资源配置
├── config/                 # 配置文件目录
│   └── mid360_config.json  # MID360配置文件
├── CMakeLists.txt          # CMake项目配置
└── README.md               # 项目说明
```

## 依赖项

本项目使用vcpkg管理依赖，主要依赖项如下：

- Qt6 (Core, Gui, Widgets, OpenGLWidgets)
- PCL 1.13.0+ (含VTK依赖)
- Eigen3
- Boost
- Livox SDK2

## 编译与运行

### 使用vcpkg安装依赖项

```
vcpkg install pcl:x64-windows
vcpkg install eigen3:x64-windows
vcpkg install boost:x64-windows
vcpkg install qt6:x64-windows
```

### 编译Livox SDK2

1. 从GitHub克隆Livox SDK2:
   ```
   git clone https://github.com/Livox-SDK/Livox-SDK2.git
   ```
2. 编译Livox SDK2:
   ```
   cd Livox-SDK2
   mkdir build && cd build
   cmake ..
   cmake --build . --config Release
   ```

### 编译项目

1. 创建构建目录:
   ```
   mkdir build
   cd build
   ```

2. 配置CMake:
   ```
   cmake -DCMAKE_PREFIX_PATH=D:/vcpkg/installed/x64-windows ..
   ```

3. 构建项目:
   ```
   cmake --build . --config Release
   ```

如果遇到编译问题，可以使用提供的清理脚本:
```
clean_cmake.bat
```

### 运行程序

1. 确保MID360激光雷达已连接到计算机
2. 确保配置文件`mid360_config.json`已放置在正确位置
3. 运行编译后生成的可执行文件

## 程序功能

### 主要功能

1. **激光雷达连接与控制**
   - 连接MID360设备
   - 启动/停止扫描
   - 实时显示设备状态

2. **点云处理与可视化**
   - 实时显示点云数据
   - 点云降采样和滤波
   - 基于Fast-LIO2的点云配准
   - 全局地图构建与显示

3. **地图管理**
   - 保存当前构建的地图
   - 加载已存在的地图
   - 重置地图

### 配置文件

配置文件`mid360_config.json`可以调整以下参数:

- **LivoxLidar**: 激光雷达设备参数
  - broadcast_code: 设备广播码
  - enable_fan: 风扇启用状态
  - return_mode: 回波模式
  - coordinate: 坐标系设置

- **FastLio**: SLAM算法参数
  - map_resolution: 地图分辨率
  - use_imu_data: 是否使用IMU数据
  - thread_num: 处理线程数
  - filter_size: 滤波器大小

- **Visualization**: 可视化参数
  - point_size: 点大小
  - background_color: 背景颜色
  - update_rate: 更新频率

## 已完成的工作

- 支持Livox MID360激光雷达的数据采集与连接控制
- 实现了SDK初始化并设置雷达工作模式为`kLivoxLidarNormal`以正常接收点云数据
- 实现基于时间间隔和点数的合帧逻辑，解决单帧点数少（96点）且帧号都为0的问题，使点云能够累积成完整地图
- 实现点云的实时可视化，采用高度(z值)作为颜色映射，实现彩色可视化效果
- 添加"自动相机跟随"开关功能，解决鼠标滚轮缩放被重置问题，使用户可在自动跟随和手动交互模式间切换
- 支持地图的保存、加载与重置
- 支持点云体素滤波降采样，提升大规模点云处理效率
- 支持多线程点云处理，提升系统响应速度
- 提供直观的Qt6图形界面，支持常用操作与参数配置
- 增加详细的调试输出与日志，便于问题定位

## 常见问题解决

1. **无法连接激光雷达**
   - 检查设备是否已通电
   - 检查网络连接是否正常
   - 确认配置文件中的broadcast_code是否正确
   - 确保雷达工作模式设置为`kLivoxLidarNormal`

2. **点云显示异常或无法累积**
   - 检查点云合帧逻辑是否正确
   - 确认帧号处理机制工作正常
   - 调整合帧时间间隔和最小点数阈值

3. **程序崩溃或性能问题**
   - 降低点云分辨率
   - 增大体素滤波分辨率
   - 降低可视化刷新频率
   - 减少处理线程数
   - 确保电脑满足系统需求

4. **编译错误**
   - 运行clean_cmake.bat清理构建目录
   - 确认所有依赖项安装正确
   - 检查CMake配置和路径设置

## 许可证

本项目基于MIT许可证开源。

## 贡献

欢迎提交问题和改进建议!

## 未完成的工作

- 点云地图渲染性能优化（随点云数量增加出现卡顿，需进一步优化）
- 支持多种点云可视化模式（如只显示当前帧/最近N帧/全局地图）
- 自动相机跟随与手动交互的更多自定义选项
- 更丰富的点云颜色映射方案（除高度外，支持按强度、距离等）
- 地图保存/加载的效率和兼容性提升
- 支持多雷达/多传感器数据融合
- 更完善的异常处理和日志系统
- UI界面美化与交互体验提升

## 未来开发方向

- 引入多线程/异步机制，提升点云处理与渲染效率
- 实现基于时间窗口的动态点云管理，避免全局点云过大造成性能下降
- 支持点云地图的分块管理与动态加载，适应超大场景
- 增加点云地图的编辑、标注与测量功能
- 优化体素滤波参数，在保持细节和性能间取得平衡
- 支持SLAM/定位算法的插件化扩展
- 融合IMU、GNSS等多源数据，提升建图精度
- 支持Web端/远程点云可视化与协作
- 增加自动化测试与持续集成，提升项目稳定性
- 完善文档与开发者指南，方便二次开发 