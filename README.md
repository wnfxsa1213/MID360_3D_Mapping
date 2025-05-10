# MID360 3D建图系统实现指南

## 项目概述

本项目旨在使用MID360激光雷达和内置IMU实现高精度3D地图构建系统。项目基于Qt和PCL框架，无需依赖ROS，可直接部署到Windows/Linux平台。

## 目标效果

如下图所示，我们的目标是实现高精度、高密度、细节丰富的3D点云地图，能够清晰展示建筑物内外部结构、道路、植被等环境细节。

![目标效果图1](高精度点云地图示例1.jpg)
![目标效果图2](高精度点云地图示例2.jpg)

## 技术路线

经过分析比较，我们有两条可选的技术路线：

### 方案一：移植LIO-Livox核心算法（推荐）

- **优点**：效果最接近目标图，工业级精度，对MID360有专门优化
- **方法**：只使用LIO-Livox的核心C++库，不用ROS，自己实现数据输入输出和可视化接口
- **难点**：需要理解LIO-Livox源码，适配数据流和接口
- **参考**：[LIO-Livox官方仓库](https://github.com/Livox-SDK/LIO-Livox)

### 方案二：使用oh_my_loam等轻量级算法

- **优点**：易于集成，代码结构清晰，适合自定义开发
- **方法**：先跑通基本LIO/LOAM流程，再逐步优化
- **难点**：最终效果和鲁棒性略逊于LIO-Livox
- **参考**：[oh_my_loam仓库](https://github.com/feixyz10/oh_my_loam)

## 核心功能模块

无论选择哪种方案，以下核心功能模块是必须实现的：

### 1. IMU与点云的严格同步和去畸变补偿

- IMU数据缓存与管理
- 点云中每个点的时间戳处理
- IMU预积分与插值
- 点云运动畸变补偿

### 2. 特征提取与点云配准

- 角点/线特征提取
- 平面/面特征提取
- 特征匹配与位姿估计
- 全局特征地图管理

### 3. 滑窗优化与闭环检测

- 滑动窗口优化
- 位姿图优化
- 闭环检测与校正
- 全局一致性优化

### 4. 地图管理与可视化

- 体素滤波与降采样
- 动态物体识别与剔除
- 高效地图数据结构
- 美观的3D可视化效果

## 项目结构

```
MID360_3D_Mapping/
├── config/                  # 配置文件目录
│   └── mid360_config.json   # MID360配置
├── include/                 # 头文件
│   ├── LidarManager.h       # 激光雷达管理
│   ├── FastLioProcessor.h   # Fast-LIO算法处理器
│   └── MainWindow.h         # 主窗口
├── src/                     # 源代码
│   ├── LidarManager.cpp     # 激光雷达管理实现
│   ├── FastLioProcessor.cpp # Fast-LIO算法实现
│   ├── MainWindow.cpp       # 主窗口实现
│   ├── Logger.cpp           # 日志系统
│   └── main.cpp             # 程序入口
├── resources/               # 资源文件
│   └── resources.qrc        # Qt资源文件
└── CMakeLists.txt           # CMake构建文件
```

## 安装与构建

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
git clone https://github.com/yourusername/MID360_3D_Mapping.git
cd MID360_3D_Mapping

# 创建构建目录
mkdir build && cd build

# 配置
cmake ..

# 构建
cmake --build . --config Release
```

### Windows构建特别说明

在Windows平台上，推荐使用Visual Studio 2019/2022或Qt Creator进行编译。确保所有依赖库的DLL文件在系统PATH中或应用程序目录中。

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
- [ ] IMU与点云同步和去畸变
- [ ] 特征提取与配准
- [ ] 滑窗优化
- [ ] 地图管理与优化
- [ ] 最终系统集成与优化

## 下一步计划

1. 实现IMU与点云的严格同步和去畸变补偿
2. 参考LIO-Livox实现特征提取和点云配准算法
3. 集成滑窗优化与地图管理功能
4. 优化系统性能和可视化效果

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
