#ifndef POSE6D_H
#define POSE6D_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace fast_lio
{
    // 简化版的Pose6D结构，用于替代原始ROS的Pose6D
    class Pose6D
    {
    public:
        Pose6D() : time(0), x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
        
        Pose6D(double t, double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
            : time(t), x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw) {}
        
        // 从Eigen矩阵构造
        Pose6D(double t, const Eigen::Matrix4d &pose)
            : time(t)
        {
            x = pose(0, 3);
            y = pose(1, 3);
            z = pose(2, 3);
            
            Eigen::Matrix3d rot = pose.block<3, 3>(0, 0);
            Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0); // ZYX顺序，返回yaw,pitch,roll
            
            yaw = euler[0];
            pitch = euler[1];
            roll = euler[2];
        }
        
        // 转换为Eigen变换矩阵
        Eigen::Matrix4d to_matrix() const
        {
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            
            Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
            Eigen::Matrix3d rot = q.matrix();
            
            Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
            res.block<3, 3>(0, 0) = rot;
            res(0, 3) = x;
            res(1, 3) = y;
            res(2, 3) = z;
            
            return res;
        }
        
        double time;  // 时间戳
        double x, y, z;  // 位置
        double roll, pitch, yaw;  // 姿态角
    };
}

#endif // POSE6D_H 