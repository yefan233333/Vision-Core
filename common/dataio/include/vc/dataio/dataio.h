#pragma once

#include <opencv2/core/types.hpp>

#include <string>
#include <vector>

//! 陀螺仪数据
struct GyroData
{
    //! 转动姿态信息
    struct Rotation
    {
        float yaw = 0.f;         //!< 偏转角（向右运动为正）
        float pitch = 0.f;       //!< 俯仰角（向下运动为正）
        float roll = 0.f;        //!< 滚转角（顺时针运动为正）
        float yaw_speed = 0.f;   //!< 偏转角速度（向右运动为正）
        float pitch_speed = 0.f; //!< 俯仰角速度（向下运动为正）
        float roll_speed = 0.f;  //!< 滚转角速度（顺时针运动为正）
    } rotation;
};
