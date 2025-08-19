#pragma once
#include "transform6D.hpp"

using PoseNode = Transform6D; //!< 位姿节点类型


//! 坐标系类型
struct CoordFrame
{
    static std::string WORLD;
    static std::string CAMERA;
    static std::string JOINT;
    static std::string GYRO;
};

//! 世界坐标系
inline std::string CoordFrame::WORLD = "world";
//! 相机坐标系
inline std::string CoordFrame::CAMERA = "camera";
//! 转轴坐标系
inline std::string CoordFrame::JOINT = "joint";
//! 陀螺仪坐标系
inline std::string CoordFrame::GYRO = "gyro";