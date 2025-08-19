#pragma once

#include "vc/core/yml_manager.hpp"

//! RuneFanParam 参数模块
struct RuneFanParam
{
    //------------------【已激活】-------------------
    //! 已激活扇叶的最大长边短边比
    float ACTIVE_MAX_SIDE_RATIO = 2.0;
    //! 已激活扇叶的最小面积
    float ACTIVE_MIN_AREA = 100.0;
    //! 已激活扇叶的最大面积
    float ACTIVE_MAX_AREA = 4000.0;
    //! 已激活扇叶的与其最小外接矩形的最小面积比例
    float ACTIVE_MIN_AREA_RATIO = 0.05;
    //! 已激活扇叶与其最小外接矩形的最大面积比例
    float ACTIVE_MAX_AREA_RATIO = 0.60;
    //! 已激活扇叶的最大面积周长比
    float ACTIVE_MAX_AREA_PERIMETER_RATIO = 0.030;
    //! 已激活扇叶的最小面积周长比
    float ACTIVE_MIN_AREA_PERIMETER_RATIO = 0.0002;

    //! 顶部角点的PNP解算用的3D坐标。（坐标系的中心点是的扇叶顶部中点，此时扇叶转到神符的最高处）
    std::vector<cv::Point3f> ACTIVE_TOP_3D = {cv::Point3f(-174, -32, 0), cv::Point3f(0, 0, 0), cv::Point3f(174, -32, 0)};
    //! 底部中心角点的PNP解算用的3D坐标
    std::vector<cv::Point3f> ACTIVE_BOTTOM_CENTER_3D = {cv::Point3f(0, 350, 0)};
    //! 侧面角点的PNP解算用的3D坐标
    std::vector<cv::Point3f> ACTIVE_SIDE_3D = {cv::Point3f(-186, 173, 0), cv::Point3f(186, 173, 0)};
    //! 底部侧面角点的PNP解算用的3D坐标
    std::vector<cv::Point3f> ACTIVE_BOTTOM_SIDE_3D = {cv::Point3f(-57, 350, 0), cv::Point3f(57, 350, 0)};
    //! 扇叶坐标系相对于神符中心的平移矩阵
    cv::Matx31f ACTIVE_TRANSLATION = cv::Matx31f(0, -505, 0);
    //! 扇叶坐标系相对于神符中心的旋转矩阵
    cv::Matx33f ACTIVE_ROTATION = cv::Matx33f(1, 0, 0,
                                              0, 1, 0,
                                              0, 0, 1);

    //! 残缺扇叶的最小面积
    float ACTIVE_MIN_AREA_INCOMPLETE = 30.0;
    //! 残缺扇叶的最大面积周长比
    float ACTIVE_MAX_AREA_PERIMETER_RATIO_INCOMPLETE = 0.07;
    //! 残缺扇叶的方向的最大偏移角度
    float ACTIVE_MAX_DIRECTION_DELTA_INCOMPLETE = 45.0;

    //-----------------【未激活】--------------------
    //! 迭代匹配箭头时的最大面积增长比例
    float INACTIVE_MAX_AREA_GROWTH_RATIO = 1.5f;
    //! 箭头匹配时，两矩形的投影比值
    float INACTIVE_MAX_RECT_PROJECTION_RATIO = 1.5f;

    //! 灯臂的凸包轮廓与其最小外接矩形的最小面积比例
    float INACTIVE_MIN_AREA_RATIO = 0.70;
    //! 进行轮廓合并时的距离阈值比例
    float INACTIVE_MERGE_DISTANCE_RATIO = 0.5;
    //! 灯臂的长边与短边的最大比例
    float INACTIVE_MAX_SIDE_RATIO = 6.0;
    //! 灯臂的长边与短边的最小比例
    float INACTIVE_MIN_SIDE_RATIO = 2.0;
    //! 灯臂的最小面积
    float INACTIVE_MIN_AREA = 100.0;
    //! 灯臂的最大面积
    float INACTIVE_MAX_AREA = 4000.0;
    //! 灯臂距离其它特征的最大距离比例
    float INACTIVE_MAX_DISTANCE_RATIO = 6.0;
    //! 灯臂角点的PNP解算用的3D坐标.(坐标系的中心点是的灯臂顶边中点，此时灯臂转到神符的最高处)(顺序：左上，右上，右下，左下)
    std::vector<cv::Point3f> INACTIVE_3D = {cv::Point3f(-30, 0, 0), cv::Point3f(30, 0, 0), cv::Point3f(30, 330, 0), cv::Point3f(-30, 330, 0)};
    //! 灯臂坐标系相对于神符中心的平移矩阵
    cv::Matx31f INACTIVE_TRANSLATION = cv::Matx31f(0, -505, 0);
    //! 灯臂坐标系相对于神符中心的旋转矩阵
    cv::Matx33f INACTIVE_ROTATION = cv::Matx33f(1, 0, 0,
                                                0, 1, 0,
                                                0, 0, 1);

    YML_INIT(
        RuneFanParam,
        YML_ADD_PARAM(ACTIVE_MAX_SIDE_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_AREA);
        YML_ADD_PARAM(ACTIVE_MAX_AREA);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_RATIO);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_PERIMETER_RATIO);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_PERIMETER_RATIO);
        YML_ADD_PARAM(ACTIVE_TOP_3D);
        YML_ADD_PARAM(ACTIVE_BOTTOM_CENTER_3D);
        YML_ADD_PARAM(ACTIVE_SIDE_3D);
        YML_ADD_PARAM(ACTIVE_BOTTOM_SIDE_3D);
        YML_ADD_PARAM(ACTIVE_TRANSLATION);
        YML_ADD_PARAM(ACTIVE_ROTATION);
        YML_ADD_PARAM(ACTIVE_MIN_AREA_INCOMPLETE);
        YML_ADD_PARAM(ACTIVE_MAX_AREA_PERIMETER_RATIO_INCOMPLETE);
        YML_ADD_PARAM(ACTIVE_MAX_DIRECTION_DELTA_INCOMPLETE);
        YML_ADD_PARAM(INACTIVE_MAX_AREA_GROWTH_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_RECT_PROJECTION_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_AREA_RATIO);
        YML_ADD_PARAM(INACTIVE_MERGE_DISTANCE_RATIO);
        YML_ADD_PARAM(INACTIVE_MAX_SIDE_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_SIDE_RATIO);
        YML_ADD_PARAM(INACTIVE_MIN_AREA);
        YML_ADD_PARAM(INACTIVE_MAX_AREA);
        YML_ADD_PARAM(INACTIVE_MAX_DISTANCE_RATIO);
        YML_ADD_PARAM(INACTIVE_3D);
        YML_ADD_PARAM(INACTIVE_TRANSLATION);
        YML_ADD_PARAM(INACTIVE_ROTATION););
};

//! RuneFanParam 参数模块
inline RuneFanParam rune_fan_param;

//! 神符扇叶的绘制参数
struct RuneFanDrawParam
{
    //! 已激活扇叶
    struct Active
    {
        cv::Scalar color = cv::Scalar(0, 255, 0);          // 绿色
        int thickness = 2;                                 // 线条粗细
        int point_radius = 3;                              // 点的半径
        double font_scale = 0.5;                           //!< 文字大小
        int font_thickness = 1;                            //!< 文字粗细
        cv::Scalar font_color = cv::Scalar(255, 255, 255); //!< 文字颜色
        int arrow_thickness = 2;                           //!< 箭头粗细
        double arrow_length = 50.0;                        //!< 箭头长度
        cv::Scalar arrow_color = cv::Scalar(255, 255, 255); //!< 箭头颜色

        YML_INIT(
            Active,
            YML_ADD_PARAM(color);
            YML_ADD_PARAM(thickness);
            YML_ADD_PARAM(point_radius);
            YML_ADD_PARAM(font_scale);
            YML_ADD_PARAM(font_thickness);
            YML_ADD_PARAM(font_color);
            YML_ADD_PARAM(arrow_thickness);
            YML_ADD_PARAM(arrow_length);
            YML_ADD_PARAM(arrow_color);
        );
            
    } active;

    //! 未激活扇叶
    struct Inactive
    {
        cv::Scalar color = cv::Scalar(0, 255, 0);          // 绿色
        int thickness = 2;                                 // 线条粗细
        int point_radius = 3;                              // 点的半径
        double font_scale = 0.5;                           //!< 文字大小
        int font_thickness = 1;                            //!< 文字粗细
        cv::Scalar font_color = cv::Scalar(255, 255, 255); //!< 文字颜色
        int arrow_thickness = 2;                           //!< 箭头粗细
        double arrow_length = 50.0;                         //!< 箭头长度
        cv::Scalar arrow_color = cv::Scalar(255, 255, 255); //!< 箭头颜色

        YML_INIT(
            Inactive,
            YML_ADD_PARAM(color);
            YML_ADD_PARAM(thickness);
            YML_ADD_PARAM(point_radius);
            YML_ADD_PARAM(font_scale);
            YML_ADD_PARAM(font_thickness);
            YML_ADD_PARAM(font_color);
            YML_ADD_PARAM(arrow_thickness);
            YML_ADD_PARAM(arrow_length);
            YML_ADD_PARAM(arrow_color);
        );
    } inactive;

};

inline RuneFanDrawParam rune_fan_draw_param;