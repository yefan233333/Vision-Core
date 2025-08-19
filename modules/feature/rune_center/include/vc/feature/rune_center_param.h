#pragma once

#include <opencv2/core/types.hpp>
#include "vc/core/yml_manager.hpp"

//! RuneCenterParam 参数模块
struct RuneCenterParam
{
    //! 最大面积
    float MAX_AREA = 1000.f;
    //! 最小面积
    float MIN_AREA = 10.f;
    //! 最大边长比率
    float MAX_SIDE_RATIO = 2.5f;
    //! 最小边长比率
    float MIN_SIDE_RATIO = 0.3f;
    //! 子轮廓最大面积占比
    float MAX_SUB_AREA_RATIO = 0.2f;
    //! 与凸包轮廓的最小面积占比
    float MIN_CONVEX_AREA_RATIO = 0.9f;
    //! 最大缺陷的面积占比的最大值
    float MAX_DEFECT_AREA_RATIO = 0.3f;
    //! 最小圆形度
    float MIN_ROUNDNESS = 0.2f;
    //! 最大圆形度
    float MAX_ROUNDNESS = 0.9f;
    //! 启用面积比例判断的最小面积
    float MIN_AREA_FOR_RATIO = 20.f;
    //! 默认边长(用于强制构造时)
    float DEFAULT_SIDE = 20.f;

    //! 神符中心的父子轮廓同心度与最大轮廓的比值
    double CENTER_CONCENTRICITY_RATIO = 0.08;
    //! 神符中心坐标系相对于旋转中心坐标系的平移矩阵
    cv::Matx31f TRANSLATION = cv::Matx31f(0, 0, -165.44);
    //! 神符中心坐标系相对于旋转中心坐标系的旋转矩阵
    cv::Matx33f ROTATION = cv::Matx33f(1, 0, 0,
                                       0, 1, 0,
                                       0, 0, 1);

    YML_INIT(
        RuneCenterParam,
        YML_ADD_PARAM(MAX_AREA);
        YML_ADD_PARAM(MIN_AREA);
        YML_ADD_PARAM(MAX_SIDE_RATIO);
        YML_ADD_PARAM(MIN_SIDE_RATIO);
        YML_ADD_PARAM(MAX_SUB_AREA_RATIO);
        YML_ADD_PARAM(MIN_CONVEX_AREA_RATIO);
        YML_ADD_PARAM(MIN_ROUNDNESS);
        YML_ADD_PARAM(MIN_AREA_FOR_RATIO);
        YML_ADD_PARAM(DEFAULT_SIDE);
        YML_ADD_PARAM(CENTER_CONCENTRICITY_RATIO);
        YML_ADD_PARAM(TRANSLATION);
        YML_ADD_PARAM(ROTATION););
};
inline RuneCenterParam rune_center_param; //!< 神符中心参数实例