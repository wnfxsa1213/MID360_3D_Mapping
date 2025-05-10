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

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkRenderer.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(nullptr), isScanning(false), autoFollowCamera(true)
{
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
    
    // 初始化PCL可视化
    initializePclVisualizer();
    
    // 创建激光雷达管理器和Fast-LIO处理器
    lidarManager = new LidarManager();
    fastLioProcessor = new FastLioProcessor();
    
    // 将管理器和处理器移动到线程中
    lidarManager->moveToThread(&lidarThread);
    fastLioProcessor->moveToThread(&processorThread);
    
    // 连接信号和槽
    connectSignals();
    
    // 加载配置文件
    loadConfigFile();
    
    // 启动线程
    lidarThread.start();
    processorThread.start();
    
    // 启动可视化定时器
    visualizerTimer = new QTimer(this);
    connect(visualizerTimer, &QTimer::timeout, this, &MainWindow::onUpdatePointCloud);
    visualizerTimer->start(33); // 约30 FPS
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
    
    // 创建分割器
    QSplitter *splitter = new QSplitter(Qt::Horizontal, centralWidget);
    mainLayout->addWidget(splitter);
    
    // 创建控制面板
    QWidget *controlPanel = new QWidget(splitter);
    QVBoxLayout *controlLayout = new QVBoxLayout(controlPanel);
    
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
    statusBar->showMessage("就绪");
    
    // 连接按钮信号
    connect(startButton, &QPushButton::clicked, this, &MainWindow::onStartScan);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::onStopScan);
    connect(saveMapButton, &QPushButton::clicked, this, &MainWindow::onSaveMap);
    connect(loadMapButton, &QPushButton::clicked, this, &MainWindow::onLoadMap);
    connect(clearMapButton, &QPushButton::clicked, this, &MainWindow::onClearMap);
    connect(configButton, &QPushButton::clicked, this, &MainWindow::onConfigSettings);
}

void MainWindow::connectSignals()
{
    // 连接激光雷达信号 - 使用新式连接语法
    connect(lidarManager, &LidarManager::pointCloudReceived, 
            fastLioProcessor, &FastLioProcessor::processPointCloud,
            Qt::QueuedConnection);
    
    connect(lidarManager, &LidarManager::lidarStatusChanged, 
            this, &MainWindow::onLidarStatusChanged);
    
    connect(lidarManager, &LidarManager::lidarError, 
            this, &MainWindow::onLidarError);
    
    // 连接Fast-LIO处理器信号 - 使用新式连接语法
    connect(fastLioProcessor, &FastLioProcessor::processFinished, 
            this, &MainWindow::onProcessFinished);
    
    connect(fastLioProcessor, &FastLioProcessor::globalMapUpdated, 
            this, &MainWindow::onGlobalMapUpdated);
    
    connect(fastLioProcessor, &FastLioProcessor::poseUpdated, 
            this, &MainWindow::onPoseUpdated);
    
    connect(fastLioProcessor, &FastLioProcessor::processorError, 
            this, &MainWindow::onProcessorError);
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
    if (QFile::exists(configFilePath)) {
        // 初始化激光雷达和处理器
        if (!lidarManager->initialize(configFilePath)) {
            statusBar()->showMessage("激光雷达初始化失败");
            QMessageBox::critical(this, "错误", "激光雷达初始化失败，请检查配置文件");
        }
        
        if (!fastLioProcessor->initialize(configFilePath)) {
            statusBar()->showMessage("Fast-LIO初始化失败");
            QMessageBox::critical(this, "错误", "Fast-LIO初始化失败，请检查配置文件");
        }
    } else {
        statusBar()->showMessage("配置文件不存在: " + configFilePath);
        QMessageBox::warning(this, "警告", "配置文件不存在: " + configFilePath + "\n请先配置参数。");
    }
}

void MainWindow::saveConfigFile()
{
    // 保存配置文件的实现
    // 这里需要根据实际情况保存配置参数
}

void MainWindow::onStartScan()
{
    if (!isScanning) {
        if (!lidarManager->isConnected()) {
            // 重新尝试连接激光雷达
            if (!lidarManager->initialize(configFilePath)) {
                statusBar()->showMessage("激光雷达连接失败");
                QMessageBox::critical(this, "错误", "无法连接激光雷达，请检查配置");
                return;
            }
        }
        
        // 开始扫描
        lidarManager->startScan();
        isScanning = true;
        statusBar()->showMessage("开始扫描");
    }
}

void MainWindow::onStopScan()
{
    if (isScanning) {
        // 停止扫描
        lidarManager->stopScan();
        isScanning = false;
        statusBar()->showMessage("停止扫描");
    }
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
        if (fastLioProcessor->saveMap(fileName)) {
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
        if (fastLioProcessor->loadMap(fileName)) {
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
            // 重置处理器
            fastLioProcessor->reset();
            
            // 清空全局地图
            QMutexLocker locker(&cloudMutex);
            globalMap->clear();
            
            statusBar()->showMessage("地图已清除");
        }
    }
}

void MainWindow::onUpdatePointCloud()
{
    if (!visualizer) {
        return;
    }
    
    QMutexLocker locker(&cloudMutex);
    
    // 更新当前点云
    if (currentCloud && !currentCloud->empty()) {
        // 使用高度(z)作为颜色映射
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(currentCloud, "z");
        if (!visualizer->updatePointCloud(currentCloud, color_handler, "current_cloud")) {
            visualizer->addPointCloud(currentCloud, color_handler, "current_cloud");
        }
        visualizer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "current_cloud");
    }
    
    // 更新全局地图
    if (globalMap && !globalMap->empty()) {
        // 使用高度(z)作为颜色映射
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(globalMap, "z");
        if (!visualizer->updatePointCloud(globalMap, color_handler, "global_map")) {
            visualizer->addPointCloud(globalMap, color_handler, "global_map");
        }
        visualizer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "global_map");
    }
    
    // 更新显示
    qvtkWidget->update();
    
    // 添加调试输出
    qDebug() << "更新点云显示 - 当前点云点数:" << (currentCloud ? currentCloud->size() : 0)
             << "全局地图点数:" << (globalMap ? globalMap->size() : 0);
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