#include "MainWindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QMenuBar>
#include <QStatusBar>
#include <QSplitter>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QMessageBox>
#include <QFileDialog>
#include <QMutexLocker>
#include <QDebug>
#include <QCheckBox>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include "Logger.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <QComboBox>
#include <QBoxLayout>
#include <QLayout>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(nullptr), isScanning(false), autoFollowCamera(true), 
      currentAlgorithm(SlamAlgorithm::FastLio), // 初始化为FastLio
      algorithmSelector(nullptr), algorithmStatusLabel(nullptr)
{
    try {
        // 初始化点云指针
        currentCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        globalMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
        
        // 设置窗口标题和大小
        setWindowTitle("MID360 3D Mapping");
        resize(1200, 800);
        
        // 设置UI
        setupUI();
        
        // 初始化设置
        initializeSettings();
        
        try {
            // 初始化PCL可视化
            LOG_INFO("正在初始化PCL可视化...");
            initializePclVisualizer();
            LOG_INFO("PCL可视化初始化完成");
        } catch (const std::exception& e) {
            LOG_ERROR(QString("PCL可视化初始化失败: %1").arg(e.what()));
            QMessageBox::warning(this, "警告", QString("PCL可视化初始化失败: %1\n程序将尝试继续运行。").arg(e.what()));
        } catch (...) {
            LOG_ERROR("PCL可视化初始化失败，发生未知错误");
            QMessageBox::warning(this, "警告", "PCL可视化初始化失败，发生未知错误\n程序将尝试继续运行。");
        }
        
        try {
            // 创建激光雷达管理器和SLAM处理器
            LOG_INFO("正在创建激光雷达管理器...");
            lidarManager = new LidarManager();
            LOG_INFO("正在创建Fast-LIO处理器...");
            fastLioProcessor = new FastLioProcessor();
            LOG_INFO("正在创建Oh-My-LOAM处理器...");
            ohMyLoamProcessor = new OhMyLoamProcessor();
            LOG_INFO("组件创建完成");
            
            // 将管理器和处理器移动到线程中
            lidarManager->moveToThread(&lidarThread);
            fastLioProcessor->moveToThread(&processorThread);
            ohMyLoamProcessor->moveToThread(&processorThread);
            
            // 连接信号和槽
            LOG_INFO("正在连接信号和槽...");
            connectSignals();
            LOG_INFO("信号和槽连接完成");
            
            // 加载配置文件
            LOG_INFO("准备加载配置文件...");
            try {
                loadConfigFile();
                LOG_INFO("配置文件加载完成");
            } catch (const std::exception& e) {
                LOG_FATAL(QString("加载配置文件时发生异常: %1").arg(e.what()));
                QMessageBox::critical(this, "致命错误", QString("加载配置文件时发生异常: %1").arg(e.what()));
                throw; // 重新抛出异常，让main函数处理
            } catch (...) {
                LOG_FATAL("加载配置文件时发生未知异常");
                QMessageBox::critical(this, "致命错误", "加载配置文件时发生未知异常");
                throw; // 重新抛出异常，让main函数处理
            }
            
            // 启动线程
            LOG_INFO("正在启动线程...");
            try {
                lidarThread.start();
                processorThread.start();
                LOG_INFO("线程启动完成");
            } catch (const std::exception& e) {
                LOG_FATAL(QString("启动线程时发生异常: %1").arg(e.what()));
                QMessageBox::critical(this, "致命错误", QString("启动线程时发生异常: %1").arg(e.what()));
                throw;
            } catch (...) {
                LOG_FATAL("启动线程时发生未知异常");
                QMessageBox::critical(this, "致命错误", "启动线程时发生未知异常");
                throw;
            }
            
            // 创建并启动可视化定时器
            LOG_INFO("正在创建可视化定时器...");
            try {
                visualizerTimer = new QTimer(this);
                LOG_INFO("可视化定时器创建完成");
                
                // 在这里连接定时器信号，使用新式连接语法
                LOG_INFO("连接定时器timeout信号...");
                connect(visualizerTimer, &QTimer::timeout, this, &MainWindow::onUpdatePointCloud);
                LOG_INFO("定时器信号连接完成");
                
                // 创建辅助定时器，用于性能调整
                adaptiveTimerCounter = 0;
            } catch (const std::exception& e) {
                LOG_FATAL(QString("创建可视化定时器时发生异常: %1").arg(e.what()));
                QMessageBox::critical(this, "致命错误", QString("创建可视化定时器时发生异常: %1").arg(e.what()));
                throw;
            } catch (...) {
                LOG_FATAL("创建可视化定时器时发生未知异常");
                QMessageBox::critical(this, "致命错误", "创建可视化定时器时发生未知异常");
                throw;
            }
            
            LOG_INFO("正在启动可视化定时器...");
            try {
                if (!visualizerTimer) {
                    LOG_ERROR("可视化定时器未创建");
                    throw std::runtime_error("可视化定时器未创建");
                }
                
                // 使用低频率启动定时器，降低初始化阶段资源消耗
                visualizerTimer->start(100); // 先使用较低频率(100ms)，之后可以调整为更高频率
                LOG_INFO("可视化定时器启动完成, 初始刷新率: 10fps");
                
                // 稍后自动提高刷新率
                QTimer::singleShot(5000, this, [this]() {
                    if (visualizerTimer) {
                        visualizerTimer->setInterval(50); // 5秒后提高到20fps
                        LOG_INFO("可视化定时器刷新率提升至: 20fps");
                    }
                });
                
                // 再稍后进一步提高刷新率
                QTimer::singleShot(10000, this, [this]() {
                    if (visualizerTimer) {
                        visualizerTimer->setInterval(33); // 10秒后提高到30fps
                        LOG_INFO("可视化定时器刷新率提升至: 30fps");
                    }
                });
            } catch (const std::exception& e) {
                LOG_FATAL(QString("启动可视化定时器时发生异常: %1").arg(e.what()));
                QMessageBox::critical(this, "致命错误", QString("启动可视化定时器时发生异常: %1").arg(e.what()));
                throw;
            } catch (...) {
                LOG_FATAL("启动可视化定时器时发生未知异常");
                QMessageBox::critical(this, "致命错误", "启动可视化定时器时发生未知异常");
                throw;
            }
        } catch (const std::exception& e) {
            LOG_FATAL(QString("组件初始化失败: %1").arg(e.what()));
            QMessageBox::critical(this, "致命错误", QString("组件初始化失败: %1").arg(e.what()));
            throw; // 重新抛出异常，让main函数处理
        } catch (...) {
            LOG_FATAL("组件初始化失败，发生未知错误");
            QMessageBox::critical(this, "致命错误", "组件初始化失败，发生未知错误");
            throw; // 重新抛出异常，让main函数处理
        }
    } catch (const std::exception& e) {
        LOG_FATAL(QString("MainWindow构造函数异常: %1").arg(e.what()));
        throw; // 重新抛出异常，让main函数处理
    } catch (...) {
        LOG_FATAL("MainWindow构造函数发生未知异常");
        throw; // 重新抛出异常，让main函数处理
    }
}

MainWindow::~MainWindow()
{
    // 停止扫描
    if (isScanning) {
        onStopScan();
    }
    
    // 停止线程
    lidarThread.quit();
    processorThread.quit();
    
    // 等待线程结束
    lidarThread.wait();
    processorThread.wait();
    
    // 释放资源
    delete lidarManager;
    delete fastLioProcessor;
    delete ohMyLoamProcessor;
    delete visualizerTimer;
    
    // 删除UI
    delete ui;
}

void MainWindow::setupUI()
{
    // 创建中央部件
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    
    // 创建主布局
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);
    
    // 创建左侧布局
    QVBoxLayout *leftLayout = new QVBoxLayout();
    
    // 创建分割器
    QSplitter *splitter = new QSplitter(Qt::Horizontal, centralWidget);
    mainLayout->addWidget(splitter);
    
    // 创建控制面板
    QWidget *controlPanel = new QWidget(splitter);
    QVBoxLayout *controlLayout = new QVBoxLayout(controlPanel);
    
    // 创建算法选择组
    QGroupBox *algorithmGroupBox = new QGroupBox("SLAM算法选择", controlPanel);
    QVBoxLayout *algorithmLayout = new QVBoxLayout(algorithmGroupBox);
    
    algorithmSelector = new QComboBox(algorithmGroupBox);
    algorithmSelector->addItem("Fast-LIO (默认)");
    algorithmSelector->addItem("Oh-My-LOAM");
    algorithmSelector->setCurrentIndex(static_cast<int>(currentAlgorithm));
    
    // 添加算法状态标签
    algorithmStatusLabel = new QLabel(getCurrentAlgorithmName(), algorithmGroupBox);
    algorithmStatusLabel->setStyleSheet("font-weight: bold; color: blue;");
    
    algorithmLayout->addWidget(algorithmSelector);
    algorithmLayout->addWidget(algorithmStatusLabel);
    
    // 添加算法选择组到控制布局
    controlLayout->addWidget(algorithmGroupBox);
    
    // 创建扫描控制组
    QGroupBox *scanGroupBox = new QGroupBox("扫描控制", controlPanel);
    QVBoxLayout *scanLayout = new QVBoxLayout(scanGroupBox);
    
    QPushButton *startButton = new QPushButton("开始扫描", scanGroupBox);
    QPushButton *stopButton = new QPushButton("停止扫描", scanGroupBox);
    scanLayout->addWidget(startButton);
    scanLayout->addWidget(stopButton);
    
    // 创建地图操作组
    QGroupBox *mapGroupBox = new QGroupBox("地图操作", controlPanel);
    QVBoxLayout *mapLayout = new QVBoxLayout(mapGroupBox);
    
    QPushButton *saveMapButton = new QPushButton("保存地图", mapGroupBox);
    QPushButton *loadMapButton = new QPushButton("加载地图", mapGroupBox);
    QPushButton *clearMapButton = new QPushButton("清除地图", mapGroupBox);
    mapLayout->addWidget(saveMapButton);
    mapLayout->addWidget(loadMapButton);
    mapLayout->addWidget(clearMapButton);
    
    // 创建设置组
    QGroupBox *settingsGroupBox = new QGroupBox("设置", controlPanel);
    QVBoxLayout *settingsLayout = new QVBoxLayout(settingsGroupBox);
    
    QPushButton *configButton = new QPushButton("配置参数", settingsGroupBox);
    settingsLayout->addWidget(configButton);
    
    // 添加自动相机跟随复选框
    QCheckBox *autoFollowCheckBox = new QCheckBox("自动相机跟随", settingsGroupBox);
    autoFollowCheckBox->setChecked(true);
    settingsLayout->addWidget(autoFollowCheckBox);
    
    // 连接复选框信号
    connect(autoFollowCheckBox, &QCheckBox::toggled, this, [this](bool checked){
        autoFollowCamera = checked;
        statusBar()->showMessage(checked ? "自动相机跟随已开启" : "自动相机跟随已关闭");
    });
    
    // 添加到控制面板
    controlLayout->addWidget(scanGroupBox);
    controlLayout->addWidget(mapGroupBox);
    controlLayout->addWidget(settingsGroupBox);
    controlLayout->addStretch();
    
    // 创建可视化面板
    QWidget *visualPanel = new QWidget(splitter);
    QVBoxLayout *visualLayout = new QVBoxLayout(visualPanel);
    
    // 创建QVTK部件
    qvtkWidget = new QVTKOpenGLNativeWidget(visualPanel);
    visualLayout->addWidget(qvtkWidget);
    
    // 设置分割器尺寸
    splitter->setSizes(QList<int>() << 200 << 1000);
    
    // 设置菜单
    QMenuBar *menuBar = new QMenuBar(this);
    setMenuBar(menuBar);
    
    QMenu *fileMenu = menuBar->addMenu("文件");
    fileMenu->addAction("退出", this, &QWidget::close);
    
    QMenu *helpMenu = menuBar->addMenu("帮助");
    helpMenu->addAction("关于", [this]() {
        QMessageBox::about(this, "关于", "MID360 3D 建图系统\n基于Livox-SDK2和Fast-LIO2");
    });
    
    // 设置状态栏
    QStatusBar *statusBar = new QStatusBar(this);
    setStatusBar(statusBar);
    statusBar->showMessage(QString("当前SLAM算法: %1").arg(getCurrentAlgorithmName()));
    
    // 连接按钮信号
    connect(startButton, &QPushButton::clicked, this, &MainWindow::onStartScan);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::onStopScan);
    connect(saveMapButton, &QPushButton::clicked, this, &MainWindow::onSaveMap);
    connect(loadMapButton, &QPushButton::clicked, this, &MainWindow::onLoadMap);
    connect(clearMapButton, &QPushButton::clicked, this, &MainWindow::onClearMap);
    connect(configButton, &QPushButton::clicked, this, &MainWindow::onConfigSettings);
    
    // 连接算法选择信号
    connect(algorithmSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &MainWindow::onAlgorithmChanged);
}

void MainWindow::connectSignals()
{
    try {
        LOG_INFO("开始连接激光雷达信号...");
        // 先检查对象是否有效
        if (!lidarManager || !fastLioProcessor || !ohMyLoamProcessor) {
            LOG_ERROR("组件对象为空，无法连接信号");
            throw std::runtime_error("组件对象为空，无法连接信号");
        }
        
        // 连接点云与IMU同步信号 - 用于高精度SLAM/建图
        connect(lidarManager, &LidarManager::pointCloudWithTimestamp,
                fastLioProcessor, &FastLioProcessor::processPointCloudWithTimestamp);
        LOG_INFO("点云与IMU同步信号连接完成 - 用于SLAM/建图");
        
        // 连接点云信号 - 用于实时可视化
        connect(lidarManager, &LidarManager::pointCloudReceived,
                this, &MainWindow::onPointCloudReceived);
        LOG_INFO("点云数据信号连接完成 - 用于可视化");
        
        // 连接IMU数据信号
        LOG_INFO("开始连接IMU数据信号...");
        connect(lidarManager, &LidarManager::imuDataReceived,
                fastLioProcessor, &FastLioProcessor::processImuData);
        LOG_INFO("IMU数据信号连接完成");
        
        connect(lidarManager, &LidarManager::lidarStatusChanged,
                this, &MainWindow::onLidarStatusChanged);
        
        connect(lidarManager, &LidarManager::lidarError,
                this, &MainWindow::onLidarError);
        
        // 连接处理器信号
        LOG_INFO("开始连接处理器信号...");
        try {
            // 逐一连接处理器信号，并在每个信号后添加日志
            LOG_INFO("连接processFinished信号...");
            QObject::connect(fastLioProcessor, SIGNAL(processFinished(pcl::PointCloud<pcl::PointXYZI>::Ptr)),
                         this, SLOT(onProcessFinished(pcl::PointCloud<pcl::PointXYZI>::Ptr)));
            LOG_INFO("processFinished信号连接完成");
            
            LOG_INFO("连接globalMapUpdated信号...");
            QObject::connect(fastLioProcessor, SIGNAL(globalMapUpdated(pcl::PointCloud<pcl::PointXYZI>::Ptr)),
                         this, SLOT(onGlobalMapUpdated(pcl::PointCloud<pcl::PointXYZI>::Ptr)));
            LOG_INFO("globalMapUpdated信号连接完成");
            
            LOG_INFO("连接poseUpdated信号...");
            QObject::connect(fastLioProcessor, SIGNAL(poseUpdated(const Eigen::Matrix4f&)),
                         this, SLOT(onPoseUpdated(const Eigen::Matrix4f&)));
            LOG_INFO("poseUpdated信号连接完成");
            
            LOG_INFO("连接processorError信号...");
            QObject::connect(fastLioProcessor, SIGNAL(processorError(const QString&)),
                         this, SLOT(onProcessorError(const QString&)));
            LOG_INFO("processorError信号连接完成");
        } catch (const std::exception& e) {
            LOG_FATAL(QString("连接处理器信号时发生异常: %1").arg(e.what()));
            QMessageBox::critical(this, "致命错误", QString("连接处理器信号时发生异常: %1").arg(e.what()));
        } catch (...) {
            LOG_FATAL("连接处理器信号时发生未知异常");
            QMessageBox::critical(this, "致命错误", "连接处理器信号时发生未知异常");
        }
        
        // 连接FastLioProcessor信号
        connect(fastLioProcessor, &FastLioProcessor::processFinished, this, &MainWindow::onProcessFinished);
        connect(fastLioProcessor, &FastLioProcessor::globalMapUpdated, this, &MainWindow::onGlobalMapUpdated);
        connect(fastLioProcessor, &FastLioProcessor::poseUpdated, this, &MainWindow::onPoseUpdated);
        connect(fastLioProcessor, &FastLioProcessor::processorError, this, &MainWindow::onProcessorError);
        
        // 连接OhMyLoamProcessor信号
        connect(ohMyLoamProcessor, &OhMyLoamProcessor::processFinished, this, &MainWindow::onProcessFinished);
        connect(ohMyLoamProcessor, &OhMyLoamProcessor::globalMapUpdated, this, &MainWindow::onGlobalMapUpdated);
        connect(ohMyLoamProcessor, &OhMyLoamProcessor::poseUpdated, this, &MainWindow::onPoseUpdated);
        connect(ohMyLoamProcessor, &OhMyLoamProcessor::processorError, this, &MainWindow::onProcessorError);
        
        // 注意：我们在这里不连接定时器信号，而是在创建定时器后再连接
        LOG_INFO("信号和槽连接部分完成，定时器信号将在创建定时器后连接");
                
    } catch (const std::exception& e) {
        LOG_FATAL(QString("连接信号和槽时发生异常: %1").arg(e.what()));
        QMessageBox::critical(this, "致命错误", QString("连接信号和槽时发生异常: %1").arg(e.what()));
    } catch (...) {
        LOG_FATAL("连接信号和槽时发生未知异常");
        QMessageBox::critical(this, "致命错误", "连接信号和槽时发生未知异常");
    }
}

void MainWindow::initializeSettings()
{
    // 设置默认配置文件路径
    configFilePath = qApp->applicationDirPath() + "/config/mid360_config.json";
}

void MainWindow::initializePclVisualizer()
{
    // 创建一个vtkGenericOpenGLRenderWindow实例
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    qvtkWidget->setRenderWindow(renderWindow);
    
    // 为PCLVisualizer创建一个渲染器
    vtkNew<vtkRenderer> renderer;
    qvtkWidget->renderWindow()->AddRenderer(renderer);
    
    // 创建PCL可视化器，使用正确的构造函数
    visualizer.reset(new pcl::visualization::PCLVisualizer(renderer, qvtkWidget->renderWindow(), "MID360 3D Mapping", false));
    
    // 设置背景颜色
    visualizer->setBackgroundColor(0.1, 0.1, 0.1);
    
    // 设置相机参数
    visualizer->initCameraParameters();
    visualizer->setCameraPosition(0, 0, -5, 0, -1, 0);
    
    // 添加坐标系
    visualizer->addCoordinateSystem(1.0);
    
    // 启用自动相机重置
    visualizer->registerKeyboardCallback(
        [this](const pcl::visualization::KeyboardEvent& event) {
            if (event.getKeySym() == "r" && event.keyDown()) {
                visualizer->resetCamera();
            }
        }
    );
    
    // 确保交互器正确设置
    visualizer->setupInteractor(qvtkWidget->interactor(), qvtkWidget->renderWindow());
    
    // 设置交互模式 - 允许旋转和滚轮缩放
    visualizer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    
    // 更新VTK部件
    qvtkWidget->update();
    
    // 添加状态栏提示
    statusBar()->showMessage("按住左键旋转，按住Shift+左键平移，滚轮缩放，按R键重置视图");
}

void MainWindow::loadConfigFile()
{
    LOG_INFO("开始加载配置文件: " + configFilePath);
    
    try {
        LOG_INFO("检查配置文件是否存在...");
        if (!QFile::exists(configFilePath)) {
            LOG_ERROR("配置文件不存在: " + configFilePath);
            statusBar()->showMessage("配置文件不存在: " + configFilePath);
            QMessageBox::warning(this, "警告", "配置文件不存在: " + configFilePath + "\n请先配置参数。");
            return;
        }
        LOG_INFO("配置文件存在");
        
        // 从配置文件中读取设置
        QFile file(configFilePath);
        if (file.open(QIODevice::ReadOnly)) {
            QByteArray fileData = file.readAll();
            file.close();
            
            QJsonDocument jsonDoc = QJsonDocument::fromJson(fileData);
            if (!jsonDoc.isNull() && jsonDoc.isObject()) {
                QJsonObject rootObj = jsonDoc.object();
                
                // 读取当前算法设置
                if (rootObj.contains("current_slam_algorithm")) {
                    QString algStr = rootObj["current_slam_algorithm"].toString();
                    if (algStr == "OhMyLoam") {
                        currentAlgorithm = SlamAlgorithm::OhMyLoam;
                    } else {
                        currentAlgorithm = SlamAlgorithm::FastLio;
                    }
                    
                    // 更新算法选择器
                    if (algorithmSelector) {
                        algorithmSelector->setCurrentIndex(static_cast<int>(currentAlgorithm));
                    }
                    
                    // 更新算法状态标签
                    if (algorithmStatusLabel) {
                        algorithmStatusLabel->setText(getCurrentAlgorithmName());
                    }
                }
                
                // 读取IMU设置
                if (rootObj.contains("FastLio") && rootObj["FastLio"].isObject()) {
                    QJsonObject fastLioObj = rootObj["FastLio"].toObject();
                    if (fastLioObj.contains("use_imu_data")) {
                        useImuData = fastLioObj["use_imu_data"].toBool();
                        LOG_INFO(QString("FastLio IMU配置: %1").arg(useImuData ? "启用" : "禁用"));
                    }
                }
                
                if (rootObj.contains("OhMyLoam") && rootObj["OhMyLoam"].isObject()) {
                    QJsonObject loamObj = rootObj["OhMyLoam"].toObject();
                    if (loamObj.contains("use_imu_data")) {
                        // 如果Oh-My-LOAM有特定的IMU配置，读取它
                        if (currentAlgorithm == SlamAlgorithm::OhMyLoam) {
                            useImuData = loamObj["use_imu_data"].toBool();
                            LOG_INFO(QString("OhMyLoam IMU配置: %1").arg(useImuData ? "启用" : "禁用"));
                        }
                    }
                }
            }
        }
        
        LOG_INFO("检查组件对象...");
        // 确保组件对象已创建
        if (!lidarManager) {
            LOG_ERROR("激光雷达管理器对象未创建");
            QMessageBox::critical(this, "错误", "激光雷达管理器对象未创建，无法初始化");
            return;
        }
        LOG_INFO("激光雷达管理器对象有效");
        
        if (!fastLioProcessor) {
            LOG_ERROR("Fast-LIO处理器对象未创建");
            QMessageBox::critical(this, "错误", "Fast-LIO处理器对象未创建，无法初始化");
            return;
        }
        LOG_INFO("Fast-LIO处理器对象有效");
        
        LOG_INFO("配置文件存在，准备初始化组件");
        
        // 初始化激光雷达
        LOG_INFO("开始初始化激光雷达管理器...");
        bool lidarInitResult = false;
        try {
            LOG_INFO("调用lidarManager->initialize...");
            lidarInitResult = lidarManager->initialize(configFilePath);
            LOG_INFO("lidarManager->initialize调用完成，结果: " + (lidarInitResult ? QString("成功") : QString("失败")));
        } catch (const std::exception& e) {
            LOG_ERROR(QString("激光雷达初始化异常: %1").arg(e.what()));
            QMessageBox::critical(this, "错误", QString("激光雷达初始化异常: %1").arg(e.what()));
            return;
        }
        
        if (!lidarInitResult) {
            LOG_ERROR("激光雷达初始化失败");
            statusBar()->showMessage("激光雷达初始化失败");
            QMessageBox::critical(this, "错误", "激光雷达初始化失败，请检查配置文件和设备连接");
            return;
        }
        LOG_INFO("激光雷达管理器初始化成功");
        
        // 检查是否需要启用IMU
        LOG_INFO("开始检查IMU配置...");
        try {
            LOG_INFO("打开配置文件读取IMU设置...");
            QFile file(configFilePath);
            if (file.open(QIODevice::ReadOnly)) {
                LOG_INFO("配置文件打开成功");
                QByteArray fileData = file.readAll();
                file.close();
                LOG_INFO("配置文件读取完成");
                
                LOG_INFO("解析JSON数据...");
                QJsonDocument jsonDoc = QJsonDocument::fromJson(fileData);
                LOG_INFO("JSON数据解析完成");
                
                if (!jsonDoc.isNull() && jsonDoc.isObject()) {
                    LOG_INFO("检查FastLio节点...");
                    QJsonObject rootObj = jsonDoc.object();
                    
                    if (rootObj.contains("FastLio") && rootObj["FastLio"].isObject()) {
                        LOG_INFO("找到FastLio节点，检查use_imu_data设置...");
                        QJsonObject fastLioObj = rootObj["FastLio"].toObject();
                        if (fastLioObj.contains("use_imu_data")) {
                            bool useImu = fastLioObj["use_imu_data"].toBool();
                            LOG_INFO(QString("IMU配置: %1").arg(useImu ? "启用" : "禁用"));
                            
                            LOG_INFO("设置IMU状态...");
                            if (useImu) {
                                LOG_INFO("正在启用IMU...");
                                lidarManager->enableImu(true);
                                LOG_INFO("IMU已启用");
                            } else {
                                LOG_INFO("IMU配置为禁用，不执行操作");
                            }
                        } else {
                            LOG_INFO("未找到use_imu_data设置，使用默认配置");
                        }
                    } else {
                        LOG_INFO("未找到FastLio节点，使用默认配置");
                    }
                } else {
                    LOG_ERROR("JSON数据无效或为空");
                }
            } else {
                LOG_ERROR("无法打开配置文件读取IMU设置: " + file.errorString());
            }
        } catch (const std::exception& e) {
            LOG_ERROR(QString("读取IMU配置时异常: %1").arg(e.what()));
        }
        LOG_INFO("IMU配置检查完成");
        
        // 初始化Fast-LIO处理器
        LOG_INFO("开始初始化Fast-LIO处理器...");
        bool processorInitResult = false;
        try {
            LOG_INFO("调用fastLioProcessor->initialize...");
            processorInitResult = fastLioProcessor->initialize(configFilePath);
            LOG_INFO("fastLioProcessor->initialize调用完成，结果: " + (processorInitResult ? QString("成功") : QString("失败")));
        } catch (const std::exception& e) {
            LOG_ERROR(QString("Fast-LIO初始化异常: %1").arg(e.what()));
            QMessageBox::critical(this, "错误", QString("Fast-LIO初始化异常: %1").arg(e.what()));
            return;
        }
        
        if (!processorInitResult) {
            LOG_ERROR("Fast-LIO初始化失败");
            statusBar()->showMessage("Fast-LIO初始化失败");
            QMessageBox::critical(this, "错误", "Fast-LIO初始化失败，请检查配置文件");
            return;
        }
        LOG_INFO("Fast-LIO处理器初始化成功");
        
        // 初始化Oh-My-LOAM处理器
        if (!ohMyLoamProcessor->initialize(configFilePath)) {
            LOG_ERROR("初始化Oh-My-LOAM处理器失败");
            LOG_WARNING("Oh-My-LOAM初始化失败，但不影响Fast-LIO使用");
            // 不抛出异常，允许Oh-My-LOAM初始化失败
        }
        
        LOG_INFO("配置文件加载和组件初始化完成");
    } catch (const std::exception& e) {
        LOG_ERROR(QString("加载配置文件时发生异常: %1").arg(e.what()));
        statusBar()->showMessage("配置加载异常");
        QMessageBox::critical(this, "错误", QString("加载配置文件时发生异常: %1").arg(e.what()));
    } catch (...) {
        LOG_ERROR("加载配置文件时发生未知异常");
        statusBar()->showMessage("配置加载异常");
        QMessageBox::critical(this, "错误", "加载配置文件时发生未知异常");
    }
}

void MainWindow::saveConfigFile()
{
    // 保存配置文件
    try {
        QFile file(configFilePath);
        if (file.open(QIODevice::ReadOnly)) {
            QByteArray fileData = file.readAll();
            file.close();
            
            QJsonDocument jsonDoc = QJsonDocument::fromJson(fileData);
            if (!jsonDoc.isNull() && jsonDoc.isObject()) {
                QJsonObject rootObj = jsonDoc.object();
                
                // 添加或更新当前算法设置
                rootObj["current_slam_algorithm"] = (currentAlgorithm == SlamAlgorithm::OhMyLoam) ? 
                    "OhMyLoam" : "FastLio";
                
                // 写回文件
                if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                    file.write(QJsonDocument(rootObj).toJson());
                    file.close();
                    LOG_INFO("配置文件已更新");
                } else {
                    LOG_ERROR("无法写入配置文件: " + file.errorString());
                }
            }
        }
    } catch (const std::exception& e) {
        LOG_ERROR(QString("保存配置文件时发生异常: %1").arg(e.what()));
    } catch (...) {
        LOG_ERROR("保存配置文件时发生未知异常");
    }
}

void MainWindow::onStartScan()
{
    if (isScanning) {
        // 已经在扫描中，不需要重复操作
        return;
    }

    // 检查激光雷达连接状态
    if (!lidarManager->isConnected()) {
        // 重新尝试连接激光雷达
        if (!lidarManager->initialize(configFilePath)) {
            statusBar()->showMessage("激光雷达连接失败");
            QMessageBox::critical(this, "错误", "无法连接激光雷达，请检查配置");
            return;
        }
    }
    
    // 先断开之前可能的连接，确保信号不会重复连接
    lidarManager->disconnect(fastLioProcessor);
    lidarManager->disconnect(ohMyLoamProcessor);
    
    // 根据当前选择的算法，连接点云信号
    switch(currentAlgorithm) {
    case SlamAlgorithm::FastLio:
        LOG_INFO("使用Fast-LIO算法开始扫描");
        statusBar()->showMessage("使用Fast-LIO算法开始扫描");
        
        // 连接点云数据信号
        connect(lidarManager, &LidarManager::pointCloudReceived, 
                fastLioProcessor, &FastLioProcessor::processPointCloud);
                
        // 连接带时间戳的点云信号
        connect(lidarManager, &LidarManager::pointCloudWithTimestamp,
                fastLioProcessor, &FastLioProcessor::processPointCloudWithTimestamp);
        
        // Fast-LIO通常需要IMU数据
        if (useImuData) {
            connect(lidarManager, &LidarManager::imuDataReceived,
                    fastLioProcessor, &FastLioProcessor::processImuData);
        }
        break;
        
    case SlamAlgorithm::OhMyLoam:
        LOG_INFO("使用Oh-My-LOAM算法开始扫描");
        statusBar()->showMessage("使用Oh-My-LOAM算法开始扫描");
        
        // 连接点云数据信号
        connect(lidarManager, &LidarManager::pointCloudReceived, 
                ohMyLoamProcessor, &OhMyLoamProcessor::processPointCloud);
                
        // 连接带时间戳的点云信号
        connect(lidarManager, &LidarManager::pointCloudWithTimestamp,
                ohMyLoamProcessor, &OhMyLoamProcessor::processPointCloudWithTimestamp);
        
        // Oh-My-LOAM通常不需要IMU数据，但如果配置了使用，仍然连接
        if (useImuData) {
            connect(lidarManager, &LidarManager::imuDataReceived,
                    ohMyLoamProcessor, &OhMyLoamProcessor::processImuData);
        }
        break;
    }
    
    // 无论使用哪种算法，都连接可视化需要的信号
    connect(lidarManager, &LidarManager::pointCloudReceived, 
            this, &MainWindow::onPointCloudReceived);
    
    // 开始扫描 - 修复：移除if条件判断，因为startScan()返回void
    try {
        lidarManager->startScan();
        isScanning = true;
        LOG_INFO("激光雷达扫描已开始");
    } catch (const std::exception& e) {
        statusBar()->showMessage(QString("激光雷达扫描启动失败: %1").arg(e.what()));
        QMessageBox::warning(this, "警告", QString("启动扫描失败: %1").arg(e.what()));
        
        // 断开刚刚连接的信号
        disconnectLidarSignals();
    } catch (...) {
        statusBar()->showMessage("激光雷达扫描启动失败");
        QMessageBox::warning(this, "警告", "启动扫描失败，发生未知错误");
        
        // 断开刚刚连接的信号
        disconnectLidarSignals();
    }
}

void MainWindow::onStopScan()
{
    if (!isScanning) {
        return; // 未在扫描，不需要操作
    }
    
    // 停止扫描 - 修复：移除if条件判断，因为stopScan()返回void
    try {
        lidarManager->stopScan();
        LOG_INFO("激光雷达扫描已停止");
    } catch (const std::exception& e) {
        LOG_WARNING(QString("停止激光雷达扫描时发生异常: %1").arg(e.what()));
    } catch (...) {
        LOG_WARNING("停止激光雷达扫描时发生未知异常");
    }
    
    // 断开所有点云和IMU信号连接
    disconnectLidarSignals();
    
    isScanning = false;
    statusBar()->showMessage(QString("停止扫描 - 当前算法: %1").arg(getCurrentAlgorithmName()));
}

// 添加辅助方法，用于断开所有激光雷达信号连接
void MainWindow::disconnectLidarSignals()
{
    // 断开与FastLioProcessor的连接
    lidarManager->disconnect(SIGNAL(pointCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr)),
                          fastLioProcessor, SLOT(processPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr)));
    lidarManager->disconnect(SIGNAL(pointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr, uint64_t)),
                          fastLioProcessor, SLOT(processPointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr, uint64_t)));
    lidarManager->disconnect(SIGNAL(imuDataReceived(const ImuData&)),
                          fastLioProcessor, SLOT(processImuData(const ImuData&)));
    
    // 断开与OhMyLoamProcessor的连接
    lidarManager->disconnect(SIGNAL(pointCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr)),
                          ohMyLoamProcessor, SLOT(processPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr)));
    lidarManager->disconnect(SIGNAL(pointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr, uint64_t)),
                          ohMyLoamProcessor, SLOT(processPointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr, uint64_t)));
    lidarManager->disconnect(SIGNAL(imuDataReceived(const ImuData&)),
                          ohMyLoamProcessor, SLOT(processImuData(const ImuData&)));
    
    // 断开与MainWindow的连接
    lidarManager->disconnect(SIGNAL(pointCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr)),
                          this, SLOT(onPointCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr)));
}

void MainWindow::onSaveMap()
{
    if (globalMap->empty()) {
        QMessageBox::warning(this, "警告", "地图为空，无法保存");
        return;
    }
    
    QString fileName = QFileDialog::getSaveFileName(this, "保存地图", 
                                                  qApp->applicationDirPath(), 
                                                  "PCD文件 (*.pcd);;所有文件 (*)");
    if (!fileName.isEmpty()) {
        bool result = false;
        
        // 根据当前算法选择相应的处理器保存地图
        switch(currentAlgorithm) {
        case SlamAlgorithm::FastLio:
            result = fastLioProcessor->saveMap(fileName);
            break;
        case SlamAlgorithm::OhMyLoam:
            result = ohMyLoamProcessor->saveMap(fileName);
            break;
        }
        
        if (result) {
            statusBar()->showMessage("地图已保存: " + fileName);
        } else {
            statusBar()->showMessage("地图保存失败");
            QMessageBox::critical(this, "错误", "无法保存地图到: " + fileName);
        }
    }
}

void MainWindow::onLoadMap()
{
    QString fileName = QFileDialog::getOpenFileName(this, "加载地图", 
                                                  qApp->applicationDirPath(), 
                                                  "PCD文件 (*.pcd);;所有文件 (*)");
    if (!fileName.isEmpty()) {
        bool result = false;
        
        // 根据当前算法选择相应的处理器加载地图
        switch(currentAlgorithm) {
        case SlamAlgorithm::FastLio:
            result = fastLioProcessor->loadMap(fileName);
            break;
        case SlamAlgorithm::OhMyLoam:
            result = ohMyLoamProcessor->loadMap(fileName);
            break;
        }
        
        if (result) {
            statusBar()->showMessage("地图已加载: " + fileName);
        } else {
            statusBar()->showMessage("地图加载失败");
            QMessageBox::critical(this, "错误", "无法加载地图: " + fileName);
        }
    }
}

void MainWindow::onClearMap()
{
    if (!globalMap->empty()) {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "确认", 
                                                               "确定要清除当前地图吗？", 
                                                               QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            // 根据当前算法重置对应的处理器
            switch(currentAlgorithm) {
            case SlamAlgorithm::FastLio:
                fastLioProcessor->reset();
                break;
            case SlamAlgorithm::OhMyLoam:
                ohMyLoamProcessor->reset();
                break;
            }
            
            // 清空全局地图
            QMutexLocker locker(&cloudMutex);
            globalMap->clear();
            
            statusBar()->showMessage("地图已清除");
        }
    }
}

void MainWindow::onUpdatePointCloud()
{
    try {
        LOG_INFO("开始更新点云显示...");
        
        // 首次运行标记，用于延迟初始化
        static bool firstRun = true;
        // 全局地图更新计数器，减少更新频率
        static int globalMapUpdateCounter = 0;
        
        if (firstRun) {
            // 首次运行时只进行基本检查，不执行复杂操作
            LOG_INFO("首次执行onUpdatePointCloud，只进行基本检查");
            
            if (!visualizer) {
                LOG_ERROR("可视化器对象为空，跳过点云更新");
                return;
            }
            
            // 确认窗口组件正常
            if (!qvtkWidget || !qvtkWidget->isValid()) {
                LOG_ERROR("QVTK部件无效，跳过点云更新");
                return;
            }
            
            firstRun = false;
            LOG_INFO("首次执行onUpdatePointCloud检查通过");
            
            // 首次运行只做简单更新，不处理点云
            qvtkWidget->update();
            return;
        }
        
        // 记录刷新帧率
        adaptiveTimerCounter++;
        
        // 非首次运行，执行完整的点云更新流程
        if (!visualizer) {
            LOG_ERROR("可视化器对象为空，无法更新点云");
            return;
        }
        
        // 使用锁保护访问点云数据
        QMutexLocker locker(&cloudMutex);
        LOG_INFO("获取点云互斥锁成功");
        
        // 更新当前点云 - 每次都更新
        if (currentCloud && !currentCloud->empty()) {
            try {
                // 使用强度作为颜色映射，增强障碍物可见性
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(currentCloud, "intensity");
                
                if (!visualizer->updatePointCloud(currentCloud, color_handler, "current_cloud")) {
                    visualizer->addPointCloud(currentCloud, color_handler, "current_cloud");
                }
                // 增大点云尺寸，便于观察
                visualizer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "current_cloud");
                    
                // 确保当前帧点云显示在全局地图之上
                visualizer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "current_cloud");
                
                // 若存在全局地图，将其透明度降低，防止遮挡当前帧
                if (globalMap && !globalMap->empty()) {
                    visualizer->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "global_map");
                }
            } catch (const std::exception& e) {
                LOG_ERROR(QString("更新当前点云异常: %1").arg(e.what()));
            }
        } else {
            LOG_INFO("当前点云为空或不存在，跳过更新");
        }
        
        // 更新全局地图 - 减少更新频率
        globalMapUpdateCounter++;
        if (globalMapUpdateCounter >= 5) { // 每5帧更新一次全局地图
            globalMapUpdateCounter = 0;
            
            LOG_INFO("开始更新全局地图...");
            if (globalMap && !globalMap->empty()) {
                try {
                    LOG_INFO("创建全局地图颜色处理器...");
                    // 使用高度(z)作为颜色映射
                    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(globalMap, "z");
                    LOG_INFO("全局地图颜色处理器创建成功");
                    
                    LOG_INFO("更新/添加全局地图到可视化器...");
                    if (!visualizer->updatePointCloud(globalMap, color_handler, "global_map")) {
                        LOG_INFO("全局地图首次添加");
                        visualizer->addPointCloud(globalMap, color_handler, "global_map");
                    }
                    LOG_INFO("设置全局地图渲染属性...");
                    visualizer->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "global_map");
                    LOG_INFO("全局地图渲染属性设置完成");
                } catch (const std::exception& e) {
                    LOG_ERROR(QString("更新全局地图异常: %1").arg(e.what()));
                }
            } else {
                LOG_INFO("全局地图为空或不存在，跳过更新");
            }
        }
        
        // 更新显示
        LOG_INFO("更新VTK部件...");
        qvtkWidget->update();
        LOG_INFO("VTK部件更新完成");
        
        // 添加调试输出 - 减少日志频率
        if (adaptiveTimerCounter % 30 == 0) {
            LOG_INFO(QString("点云更新完成 - 当前点云点数: %1, 全局地图点数: %2")
                    .arg(currentCloud ? currentCloud->size() : 0)
                    .arg(globalMap ? globalMap->size() : 0));
        }
    } catch (const std::exception& e) {
        LOG_ERROR(QString("更新点云过程中发生异常: %1").arg(e.what()));
    } catch (...) {
        LOG_ERROR("更新点云过程中发生未知异常");
    }
}

void MainWindow::onConfigSettings()
{
    // 这里可以打开一个配置对话框，用于编辑激光雷达和Fast-LIO的参数
    // 现在先简化处理，只打开配置文件
    QString fileName = QFileDialog::getOpenFileName(this, "选择配置文件", 
                                                  qApp->applicationDirPath(), 
                                                  "JSON文件 (*.json);;所有文件 (*)");
    if (!fileName.isEmpty()) {
        configFilePath = fileName;
        
        // 如果正在扫描，先停止
        if (isScanning) {
            onStopScan();
        }
        
        // 重新加载配置
        loadConfigFile();
    }
}

void MainWindow::onLidarStatusChanged(bool connected) {
    if (connected) {
        statusBar()->showMessage("激光雷达已连接");
    } else {
        statusBar()->showMessage("激光雷达已断开");
    }
}

void MainWindow::onLidarError(const QString &errorMessage) {
    statusBar()->showMessage("激光雷达错误: " + errorMessage);
    QMessageBox::critical(this, "激光雷达错误", errorMessage);
}

void MainWindow::onProcessFinished(pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud)
{
    // 更新当前点云
    currentCloud = processedCloud;
}

void MainWindow::onGlobalMapUpdated(pcl::PointCloud<pcl::PointXYZI>::Ptr newGlobalMap)
{
    // 更新全局地图
    globalMap = newGlobalMap;
}

void MainWindow::onPoseUpdated(const Eigen::Matrix4f& pose)
{
    // 记录轨迹点 - 用于可视化
    static pcl::PointCloud<pcl::PointXYZ>::Ptr trajectoryCloud(new pcl::PointCloud<pcl::PointXYZ>());
    static int trajectorySkipCount = 0;
    
    // 每5个位姿更新才添加一个轨迹点，避免过密
    trajectorySkipCount++;
    if (trajectorySkipCount >= 5) {
        trajectorySkipCount = 0;
        
        // 添加当前位置到轨迹
        pcl::PointXYZ trajectoryPoint;
        trajectoryPoint.x = pose(0, 3);
        trajectoryPoint.y = pose(1, 3);
        trajectoryPoint.z = pose(2, 3);
        trajectoryCloud->push_back(trajectoryPoint);
        
        // 限制轨迹点数量
        const size_t MAX_TRAJECTORY_POINTS = 1000;
        if (trajectoryCloud->size() > MAX_TRAJECTORY_POINTS) {
            trajectoryCloud->erase(trajectoryCloud->begin(), 
                                  trajectoryCloud->begin() + (trajectoryCloud->size() - MAX_TRAJECTORY_POINTS));
        }
        
        // 更新轨迹可视化 - 修复：使用逐段线条替代不存在的addPolyline方法
        if (visualizer) {
            // 移除旧的轨迹线段
            visualizer->removeAllShapes();
            
            // 如果轨迹中有至少两个点，开始绘制线段
            if (trajectoryCloud->size() >= 2) {
                for (size_t i = 1; i < trajectoryCloud->size(); ++i) {
                    // 每段线条使用唯一ID
                    std::string line_id = "trajectory_line_" + std::to_string(i-1);
                    
                    // 添加线段连接相邻两点
                    visualizer->addLine(
                        trajectoryCloud->points[i-1], 
                        trajectoryCloud->points[i], 
                        0, 0, 255,  // 蓝色 RGB
                        line_id);
                    
                    // 设置线条宽度
                    visualizer->setShapeRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
                        2, line_id);
                }
            }
        }
    }
    
    // 可视化当前位姿 - 坐标系
    if (visualizer) {
        // 提取旋转矩阵和平移向量
        Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
        Eigen::Vector3f position(pose(0, 3), pose(1, 3), pose(2, 3));
        
        // 移除旧的坐标系
        visualizer->removeCoordinateSystem("current_pose");
        
        // 添加新的坐标系表示当前位姿
        Eigen::Affine3f poseAffine;
        poseAffine.linear() = rotation;
        poseAffine.translation() = position;
        visualizer->addCoordinateSystem(0.5, poseAffine, "current_pose");
    }
    
    // 自动相机跟随功能
    if (visualizer && autoFollowCamera) {
        static int update_count = 0;
        if (update_count++ % 10 == 0) {
            Eigen::Vector3f position(pose(0, 3), pose(1, 3), pose(2, 3));
            Eigen::Vector3f view_direction = pose.block<3,3>(0,0) * Eigen::Vector3f(1, 0, 0);
            Eigen::Vector3f camera_pos = position + Eigen::Vector3f(0, 0, 2);
            Eigen::Vector3f focal_point = position + 3.0f * view_direction.normalized();
            Eigen::Vector3f up_vector(0, 0, 1);
            visualizer->setCameraPosition(
                camera_pos.x(), camera_pos.y(), camera_pos.z(),
                focal_point.x(), focal_point.y(), focal_point.z(),
                up_vector.x(), up_vector.y(), up_vector.z()
            );
            qvtkWidget->update();
        }
    }
}

void MainWindow::onProcessorError(const QString &errorMessage) {
    statusBar()->showMessage("处理错误: " + errorMessage);
    QMessageBox::critical(this, "处理错误", errorMessage);
}

// 添加新的槽函数用于处理可视化点云
void MainWindow::onPointCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // 更新当前点云用于可视化
    currentCloud = cloud;
}

// 添加算法切换槽函数
void MainWindow::onAlgorithmChanged(int index)
{
    // 如果正在扫描，先停止
    bool wasScanning = isScanning;
    if (wasScanning) {
        onStopScan();
    }
    
    // 更新算法类型
    currentAlgorithm = static_cast<SlamAlgorithm>(index);
    
    // 更新算法状态标签
    if (algorithmStatusLabel) {
        algorithmStatusLabel->setText(getCurrentAlgorithmName());
    }
    
    // 更新状态栏
    QString algorithmName = getCurrentAlgorithmName();
    statusBar()->showMessage(QString("当前SLAM算法: %1").arg(algorithmName));
    
    // 更新UI以反映当前算法
    updateUIForCurrentAlgorithm();
    
    // 保存当前选择到配置文件
    saveConfigFile();
    
    LOG_INFO(QString("SLAM算法已切换为: %1").arg(algorithmName));
    QMessageBox::information(this, "算法切换", QString("SLAM算法已切换为: %1").arg(algorithmName));
    
    // 如果之前正在扫描，重新开始
    if (wasScanning) {
        onStartScan();
    }
}

// 获取当前算法名称
QString MainWindow::getCurrentAlgorithmName() const
{
    switch(currentAlgorithm) {
    case SlamAlgorithm::FastLio:
        return "Fast-LIO";
    case SlamAlgorithm::OhMyLoam:
        return "Oh-My-LOAM";
    default:
        return "未知算法";
    }
}

// 更新UI以反映当前算法
void MainWindow::updateUIForCurrentAlgorithm()
{
    switch(currentAlgorithm) {
    case SlamAlgorithm::FastLio:
        // Fast-LIO相关UI调整
        // 例如：启用IMU相关设置
        break;
        
    case SlamAlgorithm::OhMyLoam:
        // Oh-My-LOAM相关UI调整
        // 例如：禁用IMU相关设置
        break;
    }
} 