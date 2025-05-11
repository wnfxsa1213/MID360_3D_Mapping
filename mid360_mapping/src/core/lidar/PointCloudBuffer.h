#ifndef POINTCLOUDBUFFER_H
#define POINTCLOUDBUFFER_H

#include <QObject>
#include <QMutex>
#include <QVector>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudBuffer : public QObject
{
    Q_OBJECT
public:
    // 缓存配置
    struct BufferConfig {
        size_t max_size;          // 最大缓存大小
        size_t flush_interval;    // 刷新间隔(ms)
        bool enable_compression;  // 是否启用压缩
        float time_window;        // 时间窗口(s)
        float voxel_size;         // 体素大小(m)
    };

    // 点云数据包
    struct PointCloudPacket {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        uint64_t timestamp;
        bool is_processed;
    };

    explicit PointCloudBuffer(QObject *parent = nullptr);
    ~PointCloudBuffer();

    // 初始化配置
    bool initialize(const BufferConfig& config);
    
    // 添加新的点云数据
    void addPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, uint64_t timestamp);
    
    // 获取时间窗口内的点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloudInTimeWindow(uint64_t current_time);
    
    // 数据压缩
    void compress();
    
    // 定期刷新缓冲区
    void flush();
    
    // 获取当前缓冲区大小
    size_t getBufferSize() const;
    
    // 获取配置
    BufferConfig getConfig() const;
    
    // 更新配置
    void updateConfig(const BufferConfig& config);

signals:
    void bufferSizeChanged(size_t size);
    void compressionCompleted();
    void flushCompleted();

private:
    // 刷新循环
    void flushLoop();
    
    QVector<PointCloudPacket> buffer_;
    BufferConfig config_;
    mutable QMutex mutex_;
    std::atomic<bool> is_running_{false};
    std::thread flush_thread_;
};

#endif // POINTCLOUDBUFFER_H 