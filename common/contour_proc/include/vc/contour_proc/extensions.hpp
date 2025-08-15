/**
 * @file extensions.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 适配contourWrapper的OpenCV处理函数
 * @version 1.0
 * @date 2025-4-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/core/logging.h"
#include <iostream>
#include <opencv2/core.hpp>

/**
 * @brief 增强版轮廓检测函数，返回智能轮廓对象集合
 *
 * @param image 输入图像(二值图，建议使用clone保留原始数据)
 * @param contours 输出轮廓集合(Contour_ptr对象)
 * @param hierarchy 输出轮廓层级信息
 * @param mode 轮廓检索模式
 * @param method 轮廓近似方法
 * @param offset 轮廓点坐标偏移量
 *
 * @note 自动执行抗锯齿处理(根据ENABLE_SMOOTH_CONTOUR_CALC配置)
 */
inline void findContours(cv::InputArray image,
                         std::vector<Contour_ptr> &contours,
                         cv::OutputArray hierarchy,
                         int mode = cv::RETR_TREE,
                         int method = cv::CHAIN_APPROX_NONE,
                         const cv::Point &offset = cv::Point(0, 0))
{
    std::vector<std::vector<cv::Point>> raw_contours;
    cv::findContours(image, raw_contours, hierarchy, mode, method, offset);
    contours.reserve(raw_contours.size());
    for (auto &&contour : raw_contours)
    {
        contours.emplace_back(ContourWrapper<int>::make_contour(std::move(contour)));
    }
}

/**
 * @brief 增强版轮廓检测函数，返回智能轮廓对象集合
 *
 * @param image 输入图像(二值图，建议使用clone保留原始数据)
 * @param contours 输出轮廓集合(ContourWrapper对象)
 * @param hierarchy 输出轮廓层级信息 (用 unordered_map 代替)
 * @param mode 轮廓检索模式
 * @param method 轮廓近似方法
 * @param offset 轮廓点坐标偏移量
 *
 * @note 自动执行抗锯齿处理(根据ENABLE_SMOOTH_CONTOUR_CALC配置)
 *
 *        - hierarchy : std::unordered_map<Contour_ptr, std::tuple<Contour_ptr, Contour_ptr, Contour_ptr, Contour_ptr>>
 *
            unordered_map < 当前轮廓, std::tuple<后一个轮廓,前一个轮廓,内嵌轮廓,父轮廓>>
 */
inline void findContours(cv::InputArray image,
                         std::vector<Contour_ptr> &contours,
                         std::unordered_map<Contour_ptr, std::tuple<Contour_ptr, Contour_ptr, Contour_ptr, Contour_ptr>> &hierarchy,
                         int mode = cv::RETR_TREE,
                         int method = cv::CHAIN_APPROX_NONE,
                         const cv::Point &offset = cv::Point(0, 0))
{
    std::vector<std::vector<cv::Point>> raw_contours;
    std::vector<cv::Vec4i> hierarchy_vec;
    cv::findContours(image, raw_contours, hierarchy_vec, mode, method, offset);
    contours.reserve(raw_contours.size());
    for (auto &&contour : raw_contours)
    {
        contours.emplace_back(ContourWrapper<int>::make_contour(std::move(contour)));
    }
    hierarchy.reserve(raw_contours.size());
    for (size_t i = 0; i < raw_contours.size(); ++i)
    {
        auto &contour = contours[i];
        auto &h = hierarchy_vec[i];
        hierarchy[contour] = std::make_tuple(
            (h[0] != -1) ? contours[h[0]] : nullptr,
            (h[1] != -1) ? contours[h[1]] : nullptr,
            (h[2] != -1) ? contours[h[2]] : nullptr,
            (h[3] != -1) ? contours[h[3]] : nullptr);
    }
}

/**
 * @brief 绘制ContourWrapper轮廓到图像
 * @tparam T 轮廓数据类型 (int/float/double)
 * @param image 目标图像 (BGR格式)
 * @param contour 轮廓智能指针
 * @param color 绘制颜色 (默认绿色)
 * @param thickness 线宽 (默认2)
 * @param lineType 线型 (默认抗锯齿线型)
 * @param use_smoothed 是否使用平滑轮廓 (默认true)
 * @param hierarchy 轮廓层级信息 (默认无)
 * @param max_level 最大绘制层级 (默认全部)
 *
 * @note 自动处理坐标类型转换（float/double转int）
 */
template <ContourWrapperBaseType T>
void drawContour(
    cv::Mat &image,
    const std::shared_ptr<ContourWrapper<T>> &contour,
    const cv::Scalar &color = cv::Scalar(0, 255, 0),
    int thickness = 2,
    int lineType = cv::LINE_AA)
{
    // 空轮廓检查
    if (contour == nullptr || contour->empty())
    {
        return;
    }

    // 获取轮廓数据
    const auto &points = contour->points();

    // 类型转换处理器
    std::vector<cv::Point> int_points;
    int_points.reserve(points.size());

    // 坐标转换（支持int/float/double）
    if constexpr (std::is_same_v<T, int>)
    {
        // 直接复制整数坐标
        std::transform(
            points.begin(), points.end(),
            std::back_inserter(int_points),
            [](const cv::Point_<T> &p)
            {
                return cv::Point(p.x, p.y);
            });
    }
    else
    {
        // 浮点坐标四舍五入
        std::transform(
            points.begin(), points.end(),
            std::back_inserter(int_points),
            [](const cv::Point_<T> &p)
            {
                return cv::Point(
                    cvRound(p.x),
                    cvRound(p.y));
            });
    }

    // 包装为OpenCV需要的轮廓结构
    std::vector<std::vector<cv::Point>> contours{int_points};

    // 执行绘制
    cv::drawContours(
        image, contours, 0, color,
        thickness, lineType);
}

/**
 * @brief 绘制轮廓到图像
 *
 * @param[in] image 输入输出图像
 * @param[in] contours 轮廓集合
 * @param[in] contourIdx 绘制的轮廓索引，-1表示绘制所有轮廓
 * @param[in] color 绘制颜色
 * @param[in] thickness 绘制线条的粗细
 * @param[in] lineType 绘制线条的类型
 */
inline void drawContours(cv::InputOutputArray image,
                         const std::vector<Contour_ptr> &contours,
                         int contourIdx,
                         const cv::Scalar &color,
                         int thickness = 1,
                         int lineType = cv::LINE_8)
{
    if (contourIdx < -1 || contourIdx >= static_cast<int>(contours.size()))
    {
        throw std::out_of_range("Invalid contour index");
    }

    for (size_t i = 0; i < contours.size(); ++i)
    {
        if (contourIdx == -1 || contourIdx == static_cast<int>(i))
        {
            const auto &contour = contours[i];
            if (contour)
            {
                drawContour(image.getMatRef(), contour, color, thickness, lineType);
            }
            else
            {
                VC_WARNING_INFO("Contour at index %li is null.", i);
            }
        }
    }
}
