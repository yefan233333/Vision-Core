#pragma once

#include <opencv2/core/types.hpp>
#include "vc/core/yml_manager.hpp"

struct RuneFanHumpParam
{
    //=============顶部突起==========================

    //! 滤波器长度的与轮廓点数的比例
    float TOP_HUMP_FILTER_LEN_RATIO = 0.040;
    //! 滤波器标准差
    float TOP_HUMP_FILTER_SIGMA = 5.0;
    //! 不同突起点的角点的最小间隔
    int TOP_HUMP_MIN_INTERVAL = 10;
    //! 判断共线时的最大容许角度偏差（角度）
    float TOP_HUMP_MAX_COLLINEAR_DELTA = 25;
    //! 判断同向时的最大容许角度偏差（角度）
    // alignment_delta
    float TOP_HUMP_MAX_ALIGNMENT_DELTA = 20;
    //! 扇叶中心点指向突起点的向量 与 突起点自身方向的最大偏差(角度)
    float TOP_HUMP_MAX_DIRECTION_DELTA = 15;
    //! 中心点到侧边点的距离的最大比例
    float TOP_HUMP_MAX_DISTANCE_RATIO = 1.5;
    //! 最大垂直距离与轮廓长度的比例
    float TOP_HUMP_MAX_VERTICAL_DISTANCE_RATIO = 0.025;
    //! 突起点连线与其方向的夹角(角度)
    float TOP_HUMP_MIN_LINE_DIRECTION_DELTA = 45;

    //=============底部中心突起==========================
    //! 不同突起点的中心点的最小间隔
    int BOTTOM_CENTER_HUMP_MIN_INTERVAL = 10;
    //! 突起方向与扇叶底边的最小角度（角度）
    int BOTTOM_CENTER_HUMP_MIN_ANGLE = 30;
    //! 突起方向最大偏差角（角度）
    int BOTTOM_CENTER_HUMP_MAX_DELTA_ANGLE = 20;

    //=============底部侧边突起==========================

    float SIDE_HUMP_MAX_ANGLE_DELTA = 90; // 侧边突起点与顶部突起点的最大角度差

    YML_INIT(
        RuneFanHumpParam,
        YML_ADD_PARAM(TOP_HUMP_FILTER_LEN_RATIO);
        YML_ADD_PARAM(TOP_HUMP_FILTER_SIGMA);
        YML_ADD_PARAM(TOP_HUMP_MIN_INTERVAL);
        YML_ADD_PARAM(TOP_HUMP_MAX_COLLINEAR_DELTA);
        YML_ADD_PARAM(TOP_HUMP_MAX_ALIGNMENT_DELTA);
        YML_ADD_PARAM(TOP_HUMP_MAX_DIRECTION_DELTA);
        YML_ADD_PARAM(TOP_HUMP_MAX_DISTANCE_RATIO);
        YML_ADD_PARAM(TOP_HUMP_MAX_VERTICAL_DISTANCE_RATIO);
        YML_ADD_PARAM(TOP_HUMP_MIN_LINE_DIRECTION_DELTA);
        YML_ADD_PARAM(BOTTOM_CENTER_HUMP_MIN_INTERVAL);
        YML_ADD_PARAM(BOTTOM_CENTER_HUMP_MIN_ANGLE);
        YML_ADD_PARAM(BOTTOM_CENTER_HUMP_MAX_DELTA_ANGLE);
        YML_ADD_PARAM(SIDE_HUMP_MAX_ANGLE_DELTA););

};
inline RuneFanHumpParam rune_fan_hump_param;