#pragma once

#include "vc/core/yml_manager.hpp"
#include <opencv2/core.hpp>

/**
 * @brief 扩展卡尔曼滤波器参数
 *        抽取自 RuneFilterEKF_CV，实现与 RuneTargetParam 一致的配置风格
 */
struct RuneEKFParam
{
    //--------------------[初始协方差矩阵 P]--------------------
    //! 初始状态协方差
    double INIT_POS_VAR = 0.1; // 位置和角度初始方差
    double INIT_VEL_VAR = 0.1; // 速度初始方差

    //--------------------[过程噪声 Q]--------------------
    //! 位置噪声
    double Q_POS = 1e-4;
    //! 姿态角噪声
    double Q_ANGLE = 1e-4;
    //! 速度噪声
    double Q_VEL = 1e-3;
    //! 角速度噪声
    double Q_ANGLE_VEL = 1e-3;

    //--------------------[观测噪声 R]--------------------
    //! 观测位置噪声
    double R_POS = 5e-4;
    //! 观测角度噪声
    double R_ANGLE = 1e-3;

    //--------------------[其他参数]--------------------
    //! 最小时间步（秒）
    double MIN_DT = 1e-3;

    YML_INIT(
        RuneEKFParam,
        YML_ADD_PARAM(INIT_POS_VAR);
        YML_ADD_PARAM(INIT_VEL_VAR);
        YML_ADD_PARAM(Q_POS);
        YML_ADD_PARAM(Q_ANGLE);
        YML_ADD_PARAM(Q_VEL);
        YML_ADD_PARAM(Q_ANGLE_VEL);
        YML_ADD_PARAM(R_POS);
        YML_ADD_PARAM(R_ANGLE);
        YML_ADD_PARAM(MIN_DT););
};

// 全局参数实例
inline RuneEKFParam rune_ekf_param;
