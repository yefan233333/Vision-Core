#pragma once

#include "vc/core/yml_manager.hpp"
#include <opencv2/core/types.hpp>

//! RuneTargetParam 参数模块
struct RuneTargetParam
{
    //--------------------[通用]------------------------
    //! 靶心的半径 /mm (仅用于可视化，不用作PNP解算)
    float RADIUS = 150.f;

    //--------------------[已激活靶心]--------------------

    //! 最小面积——已激活靶心
    float ACTIVE_MIN_AREA = 60.f;
    //! 最大面积——已激活靶心
    float ACTIVE_MAX_AREA = 6000.f;
    //! 最小边长比率——已激活靶心
    float ACTIVE_MIN_SIDE_RATIO = 0.99f;
    //! 最大边长比率——已激活靶心
    float ACTIVE_MAX_SIDE_RATIO = 1.55f;
    //! 最小面积比率——已激活靶心
    float ACTIVE_MIN_AREA_RATIO = 0.8f;
    //! 最大面积比率——已激活靶心
    float ACTIVE_MAX_AREA_RATIO = 1.20f;
    //! 最小周长比率——已激活靶心
    float ACTIVE_MIN_PERI_RATIO = 0.35f;
    //! 最大周长比率——已激活靶心
    float ACTIVE_MAX_PERI_RATIO = 0.80f;
    //! 最大凸包面积比率——已激活靶心
    float ACTIVE_MAX_CONVEX_AREA_RATIO = 0.9f;
    //! 最大凸包周长比例——已激活靶心
    float ACTIVE_MAX_CONVEX_PERI_RATIO = 0.11f;
    //! 子轮廓与父轮廓的最小面积比值
    float ACTIVE_MIN_AREA_RATIO_SUB = 0.70f;
    //! 十环时，子轮廓面积之和与父轮廓面积的最大比值
    float ACTIVE_MAX_AREA_RATIO_SUB_TEN_RING = 0.30f;
    //! 默认边长(用于强制构造时)
    float ACTIVE_DEFAULT_SIDE = 60.f;

    //! 中心点作PNP解算时的3D坐标——已激活靶心
    std::vector<cv::Point3f> ACTIVE_3D = {cv::Point3f(0, 0, 0)};

    //----------------------【未激活靶心】----------------------
    //! 未激活靶心的最小面积
    float INACTIVE_MIN_AREA = 60.f;
    //! 未激活靶心的最大面积
    float INACTIVE_MAX_AREA = 6000.f;
    //! 未激活靶心的最小边长比率
    float INACTIVE_MIN_SIDE_RATIO = 0.99f;
    //! 未激活靶心的最大边长比率
    float INACTIVE_MAX_SIDE_RATIO = 1.55f;
    //! 未激活靶心的最小面积比率
    float INACTIVE_MIN_AREA_RATIO = 0.8f;
    //! 未激活靶心的最大面积比率
    float INACTIVE_MAX_AREA_RATIO = 1.20f;
    //! 未激活靶心的最小周长比率
    float INACTIVE_MIN_PERI_RATIO = 0.35f;
    //! 未激活靶心的最大周长比率
    float INACTIVE_MAX_PERI_RATIO = 0.80f;
    //! 中心点作PNP解算时的3D坐标
    std::vector<cv::Point3f> INACTIVE_3D = {cv::Point3f(0, 0, 0)};

    //----------------------【缺口检测参数】-------------------------

    //! 缺陷面积比率最小值
    float GAP_MIN_AREA_RATIO = 0.025f;
    //! 缺陷面积比率最大值
    float GAP_MAX_AREA_RATIO = 0.20f;
    //! 缺陷的长宽比最小值
    float GAP_MIN_SIDE_RATIO = 1.55f;
    //! 缺陷的长宽比最大值
    float GAP_MAX_SIDE_RATIO = 8.0f;
    //! 缺陷所在圆和最外层圆的半径比值
    float GAP_CIRCLE_RADIUS_RATIO = 0.7037f;
    //! 缺陷中心到缺陷圆的距离的与最外层圆的半径比值
    float GAP_MAX_DISTANCE_RATIO = 0.80f;
    //! 缺陷中心到缺陷圆的距离的与最外层圆的半径比值
    float GAP_MIN_DISTANCE_RATIO = 0.50f;
    //! 缺陷角点的PNP结算用的3D坐标
    std::vector<cv::Point3f> GAP_3D = {cv::Point3f(-67.175, -67.175, 0), cv::Point3f(0, -95, 0),
                                       cv::Point3f(67.175, -67.175, 0), cv::Point3f(95, 0, 0),
                                       cv::Point3f(67.175, 67.175, 0), cv::Point3f(0, 95, 0),
                                       cv::Point3f(-67.175, 67.175, 0), cv::Point3f(-95, 0, 0)};

    //! 靶心坐标系相对于神符中心的旋转矩阵
    cv::Matx33f ROTATION = (cv::Matx33f(1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1));
    //! 靶心坐标系相对于神符中心的平移矩阵
    cv::Matx31f TRANSLATION = (cv::Matx31f(0, -700, 0));

    YML_INIT(
        RuneTargetParam,
        YML_ADD_PARAM(RADIUS);
        YML_ADD_PARAM(ACTIVE_MIN_AREA);
        YML_ADD_PARAM(ACTIVE_MAX_AREA);
        YML_ADD_PARAM(ACTIVE_MIN_SIDE_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_SIDE_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_PERI_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_PERI_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_CONVEX_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_CONVEX_PERI_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_RATIO_SUB);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_RATIO_SUB_TEN_RING);
        YML_ADD_PARAM(ACTIVE_DEFAULT_SIDE);
        YML_ADD_PARAM(ACTIVE_3D);
        YML_ADD_PARAM(INACTIVE_MIN_AREA);
        YML_ADD_PARAM(INACTIVE_MAX_AREA);
        YML_ADD_PARAM(INACTIVE_MIN_SIDE_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_SIDE_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_AREA_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_AREA_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_PERI_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_PERI_RATIO);
        YML_ADD_PARAM(INACTIVE_3D);
        YML_ADD_PARAM(GAP_MIN_AREA_RATIO);
        YML_ADD_PARAM(GAP_MAX_AREA_RATIO);
        YML_ADD_PARAM(GAP_MIN_SIDE_RATIO);
        YML_ADD_PARAM(GAP_MAX_SIDE_RATIO);
        YML_ADD_PARAM(GAP_CIRCLE_RADIUS_RATIO);
        YML_ADD_PARAM(GAP_MAX_DISTANCE_RATIO);
        YML_ADD_PARAM(GAP_MIN_DISTANCE_RATIO);
        YML_ADD_PARAM(GAP_3D);
        YML_ADD_PARAM(ROTATION);
        YML_ADD_PARAM(TRANSLATION);
    );
};

//! RuneTargetParam 参数模块
inline RuneTargetParam rune_target_param;