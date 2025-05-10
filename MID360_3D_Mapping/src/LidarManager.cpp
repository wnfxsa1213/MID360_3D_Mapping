#include "LidarManager.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <vector>

static std::vector<LivoxLidarCartesianHighRawPoint> frame_buffer;
static uint8_t last_frame_cnt = 0;
static bool first_packet = true;
static const size_t MAX_POINTS_PER_FRAME = 30000; // 限制每帧最大点数
static size_t total_packets_count = 0; // 计数收到的数据包数量
static uint64_t last_publish_time = 0; // 上次发布点云的时间

LidarManager::LidarManager(QObject *parent)
    : QObject(parent), connected(false), scanning(false)
{
    // 初始化点云指针
    latestCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

LidarManager::~LidarManager()
{
    // 如果正在扫描，则先停止
    if (scanning) {
        stopScan();
    }
    
    // 断开连接并清理资源
    if (connected) {
        LivoxLidarSdkUninit();
    }
}

bool LidarManager::initialize(const QString &configFilePath)
{
    configFile = configFilePath;
    
    // 检查配置文件是否存在
    QFile file(configFilePath);
    if (!file.exists()) {
        emit lidarError("配置文件不存在: " + configFilePath);
        return false;
    }
    
    // 初始化Livox SDK - 修正函数调用，传递正确的参数
    if (!LivoxLidarSdkInit(configFilePath.toStdString().c_str())) {
        emit lidarError("Livox SDK初始化失败");
        return false;
    }
    
    // 启动SDK
    if (!LivoxLidarSdkStart()) {
        emit lidarError("Livox SDK启动失败");
        LivoxLidarSdkUninit();
        return false;
    }
    
    // 设置点云数据回调
    SetLivoxLidarPointCloudCallBack(
        [](const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
            // 增加调试输出，确认回调是否被触发
            qDebug() << "收到点云数据包: handle=" << handle 
                     << "类型=" << (int)dev_type 
                     << "点数=" << (data ? data->dot_num : 0) 
                     << "帧号=" << (data ? (int)data->frame_cnt : -1);
            
            // 将SDK回调转换为我们的回调格式
            LidarManager* manager = static_cast<LidarManager*>(client_data);
            if (manager && data) {
                // 计算完整的数据包大小
                uint32_t total_size = sizeof(LivoxLidarEthernetPacket) + 
                                    data->dot_num * sizeof(LivoxLidarCartesianHighRawPoint);
                manager->handleLidarData(handle, reinterpret_cast<const uint8_t*>(data), total_size);
            } else {
                qDebug() << "点云数据回调参数无效: manager=" << (manager != nullptr) << " data=" << (data != nullptr);
            }
        }, 
        this
    );
    
    // 设置连接状态回调
    SetLivoxLidarInfoCallback(
        [](const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
            // 将SDK回调转换为我们的回调格式
            LidarManager* manager = static_cast<LidarManager*>(client_data);
            if (manager && info) {
                // 这里我们创建一个临时的LivoxLidarInfo结构体
                LivoxLidarInfo lidarInfo;
                lidarInfo.dev_type = dev_type;
                manager->handleLidarInfo(handle, &lidarInfo);
            }
        }, 
        this
    );
    
    // 设置状态推送回调
    SetLivoxLidarInfoChangeCallback(
        [](const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
            // 将SDK回调转换为我们的回调格式
            LidarManager* manager = static_cast<LidarManager*>(client_data);
            if (manager && info) {
                qDebug() << "设备连接状态变化: " << handle << " SN: " << info->sn;
                
                // 设置工作模式为Normal，这是接收点云数据的必要步骤
                SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, 
                    [](livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
                        qDebug() << "设置工作模式结果: " << status << " handle: " << handle;
                    }, 
                    nullptr);
                
                // 调用状态变化处理函数
                manager->handleLidarStatus(handle, kLivoxLidarStateNormal);
            }
        }, 
        this
    );
    
    // 标记为已连接
    connected = true;
    emit lidarStatusChanged(connected);
    
    return true;
}

void LidarManager::startScan()
{
    if (!connected) {
        emit lidarError("激光雷达未连接，无法开始扫描");
        return;
    }
    
    if (scanning) {
        // 已经在扫描中
        return;
    }
    
    // 重置帧缓存状态
    frame_buffer.clear();
    total_packets_count = 0;
    first_packet = true;
    
    // 启动所有设备
    uint32_t handle = 50440384; // 使用从日志中看到的实际handle值
    
    // 首先确保雷达处于normal工作模式
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, 
        [](livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
            qDebug() << "设置工作模式结果: " << status << " handle: " << handle;
            
            if (status == kLivoxLidarStatusSuccess) {
                // 在回调内部无法直接使用this，改为通过client_data传递
                LidarManager* manager = static_cast<LidarManager*>(client_data);
                if (manager) {
                    // 启用点云发送
                    livox_status send_status = EnableLivoxLidarPointSend(handle, nullptr, nullptr);
                    if (send_status == kLivoxLidarStatusSuccess) {
                        manager->scanning = true;
                        qDebug() << "激光雷达扫描已启动";
                        emit manager->lidarStatusChanged(true);
                    } else {
                        emit manager->lidarError("启动激光雷达扫描失败，状态码: " + QString::number(send_status));
                    }
                }
            } else {
                // 这里无法使用emit，因为在静态上下文中
                qDebug() << "设置工作模式失败，无法开始扫描，状态码: " << status;
            }
        }, 
        this);  // 通过client_data传递this指针
    
    qDebug() << "正在设置工作模式并启动扫描...";
}

void LidarManager::stopScan()
{
    if (!connected || !scanning) {
        return;
    }
    
    // 停止所有设备
    uint32_t handle = 50440384; // 使用相同的handle值
    livox_status status = DisableLivoxLidarPointSend(handle, nullptr, nullptr);
    if (status != 0) { // 0表示成功
        emit lidarError("停止激光雷达扫描失败");
        return;
    }
    
    scanning = false;
    qDebug() << "激光雷达扫描已停止";
}

bool LidarManager::isConnected() const
{
    return connected;
}

bool LidarManager::isScanning() const
{
    return scanning;
}

void LidarManager::processPointCloud()
{
    // 处理最新的点云数据
    QMutexLocker locker(&mutex);
    if (!latestCloud->empty()) {
        emit pointCloudReceived(latestCloud);
    }
}

// 静态回调函数 - 现在这些函数不再使用，我们使用了lambda替代
void LidarManager::onLidarDataCallback(uint8_t handle, const uint8_t *data, uint32_t data_num, void *client_data)
{
    LidarManager *manager = static_cast<LidarManager*>(client_data);
    if (manager) {
        manager->handleLidarData(handle, data, data_num);
    }
}

void LidarManager::onLidarInfoCallback(const uint8_t handle, const LivoxLidarInfo* info, void* client_data)
{
    LidarManager *manager = static_cast<LidarManager*>(client_data);
    if (manager) {
        manager->handleLidarInfo(handle, info);
    }
}

void LidarManager::onLidarPushCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
    LidarManager *manager = static_cast<LidarManager*>(client_data);
    if (manager) {
        if (info) {
            manager->handleLidarStatus(handle, kLivoxLidarStateNormal);
        }
    }
}

// 实际处理函数
void LidarManager::handleLidarData(uint32_t handle, const uint8_t *data, uint32_t data_num)
{
    // 解析数据包
    if (data_num < sizeof(LivoxLidarEthernetPacket)) return;
    const LivoxLidarEthernetPacket* packet = reinterpret_cast<const LivoxLidarEthernetPacket*>(data);

    if (packet->data_type != kLivoxLidarCartesianCoordinateHighData) return;

    const LivoxLidarCartesianHighRawPoint* points = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(packet->data);

    // 累加当前包的所有点
    for (uint32_t i = 0; i < packet->dot_num; ++i) {
        frame_buffer.push_back(points[i]);
    }
    
    // 增加包计数
    total_packets_count++;
    
    // 获取当前时间（毫秒）
    uint64_t current_time = QDateTime::currentMSecsSinceEpoch();
    
    // 合帧逻辑：满足以下任一条件则发布点云
    // 1. 帧号变化（原有逻辑）
    // 2. 累积点数超过阈值
    // 3. 距离上次发布点云已超过时间阈值
    bool should_publish = false;
    
    if (first_packet) {
        last_frame_cnt = packet->frame_cnt;
        last_publish_time = current_time;
        first_packet = false;
        return; // 第一个包不发布
    }
    
    if (packet->frame_cnt != last_frame_cnt && !frame_buffer.empty()) {
        should_publish = true;
        qDebug() << "帧号变化触发发布:" << (int)last_frame_cnt << "->" << (int)packet->frame_cnt;
    } else if (frame_buffer.size() >= MAX_POINTS_PER_FRAME) {
        should_publish = true;
        qDebug() << "点数达到阈值触发发布:" << frame_buffer.size() << "点";
    } else if ((current_time - last_publish_time) > 50) { // 减少到50ms，提高帧率
        should_publish = true;
        qDebug() << "时间间隔触发发布:" << (current_time - last_publish_time) << "毫秒";
    }
    
    if (should_publish && !frame_buffer.empty()) {
        // 合成点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& pt : frame_buffer) {
            pcl::PointXYZI pcl_pt;
            pcl_pt.x = pt.x / 1000.0f;
            pcl_pt.y = pt.y / 1000.0f;
            pcl_pt.z = pt.z / 1000.0f;
            pcl_pt.intensity = pt.reflectivity;
            cloud->push_back(pcl_pt);
        }
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = true;

        // 更新最新点云
        {
            QMutexLocker locker(&mutex);
            *latestCloud = *cloud;
        }
        emit pointCloudReceived(cloud);
        qDebug() << "合成一帧点云，点数:" << cloud->size() 
                 << "，包含" << total_packets_count << "个数据包"
                 << "，耗时:" << (current_time - last_publish_time) << "毫秒";
        
        // 重置状态
        frame_buffer.clear();
        total_packets_count = 0;
        last_publish_time = current_time;
    }
    
    last_frame_cnt = packet->frame_cnt;
}

void LidarManager::handleLidarInfo(uint32_t handle, const LivoxLidarInfo* info)
{
    qDebug() << "激光雷达信息:" << "handle=" << handle 
             << "dev_type=" << info->dev_type;
    
    // 可以根据需要处理更多的激光雷达信息
}

void LidarManager::handleLidarStatus(uint32_t handle, LivoxLidarState status)
{
    switch (status) {
    case kLivoxLidarStateSampling:
        qDebug() << "激光雷达状态：正在采样";
        scanning = true;
        break;
    case kLivoxLidarStateDisconnect:
        qDebug() << "激光雷达状态：已断开";
        connected = false;
        scanning = false;
        emit lidarError("激光雷达连接已断开");
        emit lidarStatusChanged(false);
        break;
    case kLivoxLidarStateError:
        qDebug() << "激光雷达状态：错误";
        emit lidarError("激光雷达报告错误");
        break;
    case kLivoxLidarStateNormal:
        qDebug() << "激光雷达状态：正常";
        break;
    default:
        qDebug() << "激光雷达状态：未知状态" << static_cast<int>(status);
        break;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarManager::convertToPointCloud(const uint8_t *data, uint32_t data_num)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    // 添加调试输出
    qDebug() << "开始转换点云数据，数据大小:" << data_num;
    
    // 检查数据包大小
    if (data_num < sizeof(LivoxLidarEthernetPacket)) {
        qWarning() << "数据包大小太小:" << data_num << "小于最小包大小:" << sizeof(LivoxLidarEthernetPacket);
        return cloud;
    }
    
    // 解析Livox数据包
    LivoxLidarEthernetPacket* packet = reinterpret_cast<LivoxLidarEthernetPacket*>(const_cast<uint8_t*>(data));
    
    // 检查数据包类型
    qDebug() << "数据包类型:" << packet->data_type 
             << "点数量:" << packet->dot_num
             << "数据包长度:" << packet->length
             << "时间间隔:" << packet->time_interval
             << "帧计数:" << packet->frame_cnt;
    
    // 检查数据包类型
    if (packet->data_type != kLivoxLidarCartesianCoordinateHighData) {
        qWarning() << "不支持的数据类型:" << packet->data_type;
        return cloud;
    }
    
    // 检查数据包大小是否足够
    uint32_t expected_size = sizeof(LivoxLidarEthernetPacket) + 
                           packet->dot_num * sizeof(LivoxLidarCartesianHighRawPoint);
    if (data_num < expected_size) {
        qWarning() << "数据包大小不足，需要:" << expected_size << "实际:" << data_num;
        return cloud;
    }
    
    // 获取点云数据
    LivoxLidarCartesianHighRawPoint* points = 
        reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(packet->data);
    
    // 使用数据包中的点数量
    uint32_t point_num = packet->dot_num;
    
    qDebug() << "使用数据包中的点数量:" << point_num;
    
    // 预分配点云空间
    cloud->reserve(point_num);
    
    // 转换每个点
    for (uint32_t i = 0; i < point_num; ++i) {
        pcl::PointXYZI point;
        
        // 转换坐标（从毫米到米）
        point.x = points[i].x / 1000.0f;
        point.y = points[i].y / 1000.0f;
        point.z = points[i].z / 1000.0f;
        
        // 转换强度
        point.intensity = points[i].reflectivity;
        
        // 添加到点云
        cloud->push_back(point);
    }
    
    // 设置点云参数
    cloud->width = point_num;
    cloud->height = 1;
    cloud->is_dense = true;
    
    qDebug() << "点云转换完成，实际点数:" << cloud->size();
    
    return cloud;
} 