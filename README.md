# MID360 3D建图系统

## 项目概述

本项目旨在使用MID360激光雷达和内置IMU实现高精度3D地图构建系统。项目基于Qt和PCL框架，无需依赖ROS，可直接部署到Windows/Linux平台。系统集成了FAST-LIO和Oh-My-LOAM两种SLAM算法，提供灵活的点云处理和地图构建方案。

## 整理后的项目结构

```
mid360_mapping/
├── src/                      # 应用程序源代码
│   ├── app/                  # 主应用程序
│   │   ├── main.cpp          # 程序入口
│   │   ├── MainWindow.cpp    # 主窗口实现
│   │   └── MainWindow.h      # 主窗口头文件
│   ├── core/                 # 核心功能模块
│   │   ├── lidar/            # 激光雷达相关实现
│   │   │   ├── LidarManager.cpp
│   │   │   └── LidarManager.h
│   │   ├── slam/             # SLAM算法实现
│   │   │   ├── FastLioProcessor.cpp
│   │   │   ├── FastLioProcessor.h
│   │   │   ├── OhMyLoamProcessor.cpp
│   │   │   └── OhMyLoamProcessor.h
│   │   └── utils/            # 工具类
│   │       ├── Logger.cpp
│   │       └── Logger.h
│   └── ui/                   # 界面相关实现
│       └── resources.qrc     # Qt资源文件
├── include/                  # 第三方库头文件
│   ├── fast_lio/             # FAST-LIO算法头文件
│   └── livox_sdk/            # Livox SDK头文件
├── lib/                      # 预编译库文件
│   ├── win64/                # Windows 64位库
│   └── linux64/              # Linux 64位库
├── config/                   # 配置文件目录
│   └── mid360_config.json    # MID360配置
├── resources/                # 资源文件(图片、图标等)
├── docs/                     # 文档
│   ├── images/               # 文档图片
│   └── api/                  # API文档
├── scripts/                  # 脚本文件
│   ├── build.sh              # 构建脚本
│   └── install_deps.sh       # 安装依赖脚本
├── samples/                  # 示例数据
│   └── sample_scan/          # 示例扫描数据
├── tests/                    # 测试代码
│   ├── unit/                 # 单元测试
│   └── integration/          # 集成测试
├── third_party/              # 第三方库源码
│   ├── FAST-LIO/             # FAST-LIO库
│   ├── Livox-SDK2/           # Livox SDK 2.0
│   └── oh_my_loam/           # Oh-My-LOAM库
├── build/                    # 构建输出目录(git忽略)
├── .gitignore                # Git忽略文件
├── CMakeLists.txt            # 主CMake构建文件
├── LICENSE                   # 许可证文件
└── README.md                 # 项目说明文件
```

## 主要组件说明

### 核心组件
- **LidarManager**: 负责与MID360激光雷达通信，处理点云数据采集
- **FastLioProcessor**: 实现FAST-LIO算法，处理点云配准与地图构建
- **OhMyLoamProcessor**: 实现Oh-My-LOAM算法，提供纯激光SLAM功能
- **MainWindow**: 主窗口实现，提供用户交互界面

### 工具组件
- **Logger**: 日志记录系统，提供统一的日志记录接口

## SLAM算法选择
系统集成了两种SLAM算法，可以根据需求灵活选择：

1. **FAST-LIO**：紧耦合激光-惯性里程计，利用IMU信息实现高精度定位和建图
   - 优势：高精度、抗抖动、适应高速场景
   - 适用场景：需要高定位精度的场景，如无人驾驶、轨迹跟踪等

2. **Oh-My-LOAM**：无ROS依赖的纯激光SLAM算法，基于经典LOAM优化
   - 优势：无需IMU数据、实现简洁、计算量较小
   - 适用场景：室内建模、小型机器人导航等

### 算法切换界面
系统在界面上添加了算法选择控件，方便用户直接切换SLAM算法：
- 算法下拉选择框：位于控制面板顶部，可选择使用的SLAM算法
- 算法状态标签：以蓝色粗体显示当前激活的算法
- 状态栏提示：动态显示当前使用的算法名称
- 配置文件支持：算法选择会保存到配置文件，下次启动时自动加载

## 构建说明

### 依赖项

- Qt (推荐6.x+)
- PCL (推荐1.12+)
- Eigen3
- VTK
- Boost
- Livox SDK 2.0+
- Ceres Solver (用于Oh-My-LOAM优化)
- GLOG (用于Oh-My-LOAM日志系统)
- Yaml-cpp (用于Oh-My-LOAM配置)

### Windows下安装依赖（vcpkg）

```bash
# 使用vcpkg安装依赖
vcpkg install eigen3:x64-windows
vcpkg install pcl:x64-windows
vcpkg install vtk:x64-windows
vcpkg install boost:x64-windows
vcpkg install ceres:x64-windows
vcpkg install yaml-cpp:x64-windows
# glog会作为ceres的依赖自动安装
```

### 编译步骤

```bash
# 克隆仓库
git clone https://github.com/yourusername/mid360_mapping.git
cd mid360_mapping

# 克隆子模块
git submodule update --init --recursive

# 创建构建目录
mkdir build && cd build

# 配置
cmake ..

# 构建
cmake --build . --config Release
```

## 使用方法

1. 连接MID360激光雷达
2. 启动应用程序
3. 点击"配置参数"按钮，选择或编辑配置文件
4. 在界面顶部的下拉框中选择SLAM算法（FAST-LIO或Oh-My-LOAM）
5. 点击"开始扫描"按钮开始数据采集和建图
6. 扫描完成后，点击"保存地图"按钮导出点云地图

## 配置文件说明

系统使用JSON格式配置文件，主要包含以下配置项：

```json
{
  "Lidar": {
    "ip": "192.168.1.1",
    "port": 60001
  },
  "current_slam_algorithm": "FastLio",
  "FastLio": {
    "max_iterations": 10,
    "map_resolution": 0.1,
    "use_imu_data": true
  },
  "OhMyLoam": {
    "map_resolution": 0.2,
    "use_imu_data": false
  }
}
```

## 当前进度

- [x] 基础框架搭建（Qt + PCL）
- [x] 激光雷达数据采集
- [x] 基础点云可视化
- [x] FAST-LIO算法集成
- [x] Oh-My-LOAM算法集成
- [x] SLAM算法切换功能
- [x] 界面优化与用户体验改进
- [ ] 多线程优化
- [ ] 地图管理与优化
- [ ] 最终系统集成与优化

## 近期更新

### 2023.11.10 - 界面优化与扫描功能完善

1. **算法切换界面**：添加了明显的算法选择控件和状态显示
2. **配置文件集成**：实现算法选择保存到配置文件并自动加载
3. **扫描控制优化**：
   - 修复了开始/停止扫描功能，确保正确处理不同算法
   - 优化了信号连接和断开逻辑，避免信号重复连接
   - 增强了带时间戳点云处理，提高数据同步精度
4. **IMU数据处理**：增强了IMU配置读取和处理，根据算法特性调整
5. **状态反馈**：通过状态栏和日志系统提供清晰的当前状态反馈
6. **错误处理**：添加了更多的异常捕获和错误提示，提高系统健壮性

### 2023.11 - Oh-My-LOAM集成

1. **系统集成**：将Oh-My-LOAM作为静态库集成到项目中，与现有的Fast-LIO算法形成互补
2. **跨平台适配**：解决Windows平台下编译问题，包括CMake配置、依赖库安装及MSVC兼容
3. **接口设计**：设计统一的处理器接口，便于算法切换和处理流程标准化
4. **日志系统**：从g3log迁移到glog，确保日志系统在不同平台正常工作
5. **点云处理**：实现点云类型转换、降采样、坐标变换等功能
6. **地图管理**：支持增量建图、地图保存/加载、地图降采样等功能
7. **错误处理**：添加全面的错误检查、异常处理和状态验证
8. **线程安全**：引入互斥锁机制，确保多线程环境下的数据一致性

## 下一步计划

1. 优化OhMyLoamProcessor性能，减少内存占用
2. 实现点云后处理功能，如颜色映射、孔洞填充等
3. 开发地图分块管理功能，支持大场景建图
4. 添加实时轨迹可视化和地图质量评估工具

## 常见问题

### Oh-My-LOAM编译错误

- **问题**：编译时出现"PCL requires 201703L or above"错误
- **解决**：确保在CMake中设置 `set(CMAKE_CXX_STANDARD 17)`

### glog相关错误

- **问题**：出现"glog/logging.h was not included correctly"
- **解决**：在CMake中添加 `add_definitions(-DGOOGLE_GLOG_DLL_DECL=)`

### 点云类型转换错误

- **问题**：`oh_my_loam::PointXYZT` 与 `pcl::PointXYZI` 类型不匹配
- **解决**：在处理点云时进行正确的类型转换，详见 `OhMyLoamProcessor.cpp`

### 算法不显示或切换无效

- **问题**：界面上看不到当前使用的算法，或切换算法后没有效果
- **解决**：检查`MainWindow.cpp`中的`disconnectLidarSignals()`和`onAlgorithmChanged()`方法是否正确实现

## 贡献指南

欢迎参与本项目的开发！如果您有兴趣，请遵循以下步骤：

1. Fork本仓库
2. 创建您的特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交您的更改 (`git commit -m 'Add some amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建Pull Request

## 参考资料

- [LIO-Livox: 一种用于Livox激光雷达的鲁棒激光-惯性里程计](https://github.com/Livox-SDK/LIO-Livox)
- [oh_my_loam: 无ROS依赖的LOAM实现](https://github.com/feixyz10/oh_my_loam)
- [LIO-SAM: 紧耦合激光-惯性里程计与地图构建](https://github.com/TixiaoShan/LIO-SAM)
- [Livox SDK 2.0](https://github.com/Livox-SDK/Livox-SDK2)
- [PCL: 点云库](https://pointclouds.org/)
