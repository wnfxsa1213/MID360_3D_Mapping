# MID360 3D建图系统

## 项目概述

本项目旨在使用MID360激光雷达和内置IMU实现高精度3D地图构建系统。项目基于Qt和PCL框架，无需依赖ROS，可直接部署到Windows/Linux平台。

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
│   │   │   └── FastLioProcessor.h
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
│   └── Livox-SDK2/           # Livox SDK 2.0
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
- **MainWindow**: 主窗口实现，提供用户交互界面

### 工具组件
- **Logger**: 日志记录系统，提供统一的日志记录接口

## 构建说明

### 依赖项

- Qt (推荐6.x+)
- PCL (推荐1.12+)
- Eigen3
- VTK
- Livox SDK 2.0+
- Ceres Solver (可选，用于优化)

### 编译步骤

```bash
# 克隆仓库
git clone https://github.com/yourusername/mid360_mapping.git
cd mid360_mapping

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
4. 点击"开始扫描"按钮开始数据采集和建图
5. 扫描完成后，点击"保存地图"按钮导出点云地图

## 当前进度

- [x] 基础框架搭建（Qt + PCL）
- [x] 激光雷达数据采集
- [x] 基础点云可视化
- [x] IMU与点云同步和去畸变
- [ ] 特征提取与配准
- [ ] 滑窗优化
- [ ] 地图管理与优化
- [ ] 最终系统集成与优化

## 下一步计划

1. 实现IMU与点云的严格同步和去畸变补偿
2. 参考LIO-Livox实现特征提取和点云配准算法
3. 集成滑窗优化与地图管理功能
4. 优化系统性能和可视化效果



## 参考资料

- [LIO-Livox: 一种用于Livox激光雷达的鲁棒激光-惯性里程计](https://github.com/Livox-SDK/LIO-Livox)
- [oh_my_loam: 无ROS依赖的LOAM实现](https://github.com/feixyz10/oh_my_loam)
- [LIO-SAM: 紧耦合激光-惯性里程计与地图构建](https://github.com/TixiaoShan/LIO-SAM)
- [Livox SDK 2.0](https://github.com/Livox-SDK/Livox-SDK2)
- [PCL: 点云库](https://pointclouds.org/)
