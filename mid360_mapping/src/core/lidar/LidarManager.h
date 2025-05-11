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
#include "PointCloudBuffer.h"

// 定义我们需要的状态类型
typedef enum {
    kLivoxLidarStateNormal = 0,
    kLivoxLidarStateDisconnect = 1,
    kLivoxLidarStateError = 2,
    kLivoxLidarStateSampling = 3
} LivoxLidarState;

// 定义IMU数据结构
struct ImuData {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    uint64_t timestamp;  // 纳秒级时间戳
};

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
    bool isImuEnabled() const;
    void enableImu(bool enable);

    // 点云缓存相关方法
    void setPointCloudBufferConfig(const PointCloudBuffer::BufferConfig& config);
    PointCloudBuffer::BufferConfig getPointCloudBufferConfig() const;
    size_t getPointCloudBufferSize() const;

signals:
    void pointCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void pointCloudWithTimestamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp);
    void imuDataReceived(const ImuData& imuData);
    void lidarStatusChanged(bool connected);
    void lidarError(const QString &errorMessage);
    void bufferSizeChanged(size_t size);
    void compressionCompleted();
    void flushCompleted();

private slots:
    void processPointCloud();
    void onBufferSizeChanged(size_t size);
    void onCompressionCompleted();
    void onFlushCompleted();

private:
    // 静态回调函数，签名与SDK一致
    static void onLidarDataCallback(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
    static void onLidarInfoCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);
    static void onLidarPushCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);

    // 实际处理函数
    void handleLidarData(uint32_t handle, const uint8_t *data, uint32_t data_num);
    void handleImuData(uint32_t handle, const uint8_t *data, uint32_t data_num);
    void handleLidarInfo(uint32_t handle, const LivoxLidarInfo* info);
    void handleLidarStatus(uint32_t handle, LivoxLidarState status);

    // 转换点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr convertToPointCloud(const uint8_t *data, uint32_t data_num);

    // 成员变量
    QString configFile;
    bool connected;
    bool scanning;
    bool imuEnabled;
    QMutex mutex;
    
    // 点云缓存
    std::unique_ptr<PointCloudBuffer> point_cloud_buffer_;
    
    // 最新的点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr latestCloud;
    
    // 最新的IMU数据
    ImuData latestImuData;
};

#endif // LIDARMANAGER_H 