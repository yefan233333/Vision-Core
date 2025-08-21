#pragma once

#include "feature_node.h"

//! 绘制配置结构体
struct FeatureNode::DrawConfig
{
    cv::Scalar color = cv::Scalar(100, 255, 0); //!< 绘制颜色
    int thickness = 2;                          //!< 绘制线条粗细
    DrawMask type = 0;                          //!< 绘制类型掩码
    bool draw_contours = false;                 //!< 是否绘制轮廓
    bool draw_corners = false;                  //!< 是否绘制角点
    bool draw_center = false;                   //!< 是否绘制中心点
    bool draw_pose_nodes = false;               //!< 是否绘制位姿节点
};
