#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <QSettings>
#include <QFileDialog>
#include <QMessageBox>
#include <QComboBox>
#include <Eigen/Dense>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>

#include "LidarManager.h"
#include "FastLioProcessor.h"
#include "OhMyLoamProcessor.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// 添加SLAM算法枚举
enum class SlamAlgorithm {
    FastLio = 0,
    OhMyLoam = 1
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onStartScan();
    void onStopScan();
    void onSaveMap();
    void onLoadMap();
    void onClearMap();
    void onUpdatePointCloud();
    void onConfigSettings();
    
    // 新增的信号处理槽
    void onLidarStatusChanged(bool connected);
    void onLidarError(const QString &errorMessage);
    void onProcessFinished(pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud);
    void onGlobalMapUpdated(pcl::PointCloud<pcl::PointXYZI>::Ptr newGlobalMap);
    void onPoseUpdated(const Eigen::Matrix4f &pose);
    void onProcessorError(const QString &errorMessage);
    void onPointCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);  // 新增槽函数声明
    void onAlgorithmChanged(int index); // 添加算法切换槽函数

private:
    void setupUI();
    void connectSignals();
    void initializeSettings();
    void initializePclVisualizer();
    void loadConfigFile();
    void saveConfigFile();
    void disconnectLidarSignals(); // 辅助方法：断开激光雷达信号连接

    // 获取当前算法名称（用于显示）
    QString getCurrentAlgorithmName() const;
    // 更新UI以反映当前算法
    void updateUIForCurrentAlgorithm();

    Ui::MainWindow *ui;
    
    // PCL可视化组件
    pcl::visualization::PCLVisualizer::Ptr visualizer;
    QVTKOpenGLNativeWidget *qvtkWidget;
    QTimer *visualizerTimer;
    int adaptiveTimerCounter; // 用于自适应性能调整的计数器
    
    // 线程与互斥锁
    QThread lidarThread;
    QThread processorThread;
    QMutex cloudMutex;
    
    // 激光雷达管理器
    LidarManager *lidarManager;
    
    // 处理算法
    FastLioProcessor *fastLioProcessor;
    OhMyLoamProcessor *ohMyLoamProcessor; // 添加OhMyLoamProcessor成员
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap;
    
    // 设置
    QString configFilePath;
    bool useImuData = false;  // 是否使用IMU数据
    bool isScanning = false;
    bool autoFollowCamera = true;
    
    // SLAM算法相关
    SlamAlgorithm currentAlgorithm;

    QComboBox *algorithmSelector; // 算法选择下拉框
    QLabel *algorithmStatusLabel; // 当前算法状态标签
};
#endif // MAINWINDOW_H 