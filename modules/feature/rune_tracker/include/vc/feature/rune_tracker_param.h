#pragma once

#include "vc/core/yml_manager.hpp"


//! RuneTrackerParam 参数模块
struct RuneTrackerParam
{
    //! 追踪帧数
    int TRACK_FRAMES = 4;
    //! 采样时间
    float SAMPLE_INTERVAL = 10.f;
    //! 角度滤波器过程噪声协方差矩阵
    cv::Matx22f ROTATE_Q = cv::Matx22f::eye();
    //! 角度滤波器测量噪声协方差矩阵
    // float ROTATE_R = 0.1;
    cv::Matx22f ROTATE_R = cv::Matx22f::eye();
    //! 运动滤波器过程噪声协方差矩阵
    cv::Matx44f MOTION_Q = cv::Matx44f::eye();
    //! 运动滤波器测量噪声协方差矩阵
    cv::Matx44f MOTION_R = cv::Matx44f::diag({1, 1, 4, 4});
    //! 组合体队列的最大长度
    int MAX_DEQUE_SIZE = 32;

    YML_INIT(
        RuneTrackerParam,
        YML_ADD_PARAM(TRACK_FRAMES);
        YML_ADD_PARAM(SAMPLE_INTERVAL);
        YML_ADD_PARAM(ROTATE_Q);
        YML_ADD_PARAM(ROTATE_R);
        YML_ADD_PARAM(MOTION_Q);
        YML_ADD_PARAM(MOTION_R);
        YML_ADD_PARAM(MAX_DEQUE_SIZE);
    );
};
inline RuneTrackerParam rune_tracker_param;
