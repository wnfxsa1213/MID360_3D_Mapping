#ifndef LIDARMANAGER_H
#define LIDARMANAGER_H

#include <QObject>
#include <QMutex>
#include <QString>
#include <QThread>
#include <QDebug>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

// 定义我们需要的状态类型
typedef enum {
    kLivoxLidarStateNormal = 0,
    kLivoxLidarStateDisconnect = 1,
    kLivoxLidarStateError = 2,
    kLivoxLidarStateSampling = 3
} LivoxLidarState;

class LidarManager : public QObject
{
    Q_OBJECT
public:
    explicit LidarManager(QObject *parent = nullptr);
    ~LidarManager();

    bool initialize(const QString &configFilePath);
    void startScan();
    void stopScan();
    bool isConnected() const;
    bool isScanning() const;

signals:
    void pointCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void lidarStatusChanged(bool connected);
    void lidarError(const QString &errorMessage);

private slots:
    void processPointCloud();

private:
    // 静态回调函数
    static void onLidarDataCallback(uint8_t handle, const uint8_t *data, uint32_t data_num, void *client_data);
    static void onLidarInfoCallback(const uint8_t handle, const LivoxLidarInfo* info, void* client_data);
    static void onLidarPushCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);

    // 实际处理函数
    void handleLidarData(uint32_t handle, const uint8_t *data, uint32_t data_num);
    void handleLidarInfo(uint32_t handle, const LivoxLidarInfo* info);
    void handleLidarStatus(uint32_t handle, LivoxLidarState status);

    // 转换点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr convertToPointCloud(const uint8_t *data, uint32_t data_num);

    // 成员变量
    QString configFile;
    bool connected;
    bool scanning;
    QMutex mutex;
    
    // 最新的点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr latestCloud;
};

#endif // LIDARMANAGER_H 