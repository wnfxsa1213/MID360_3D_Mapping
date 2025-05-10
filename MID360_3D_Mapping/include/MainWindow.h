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

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>

#include "LidarManager.h"
#include "FastLioProcessor.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

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

private:
    void setupUI();
    void connectSignals();
    void initializeSettings();
    void initializePclVisualizer();
    void loadConfigFile();
    void saveConfigFile();

    Ui::MainWindow *ui;
    
    // PCL可视化组件
    pcl::visualization::PCLVisualizer::Ptr visualizer;
    QVTKOpenGLNativeWidget *qvtkWidget;
    QTimer *visualizerTimer;
    
    // 线程与互斥锁
    QThread lidarThread;
    QThread processorThread;
    QMutex cloudMutex;
    
    // 激光雷达管理器
    LidarManager *lidarManager;
    
    // 处理算法
    FastLioProcessor *fastLioProcessor;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap;
    
    // 设置
    QString configFilePath;
    bool isScanning;
};
#endif // MAINWINDOW_H 