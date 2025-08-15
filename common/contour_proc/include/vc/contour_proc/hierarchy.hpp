/**
 * @file hierarchy.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 轮廓的层级结构处理函数
 * @version 0.1
 * @date 2025-1-2
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <vector>
#include <unordered_set>
#include <opencv2/opencv.hpp>

#include "vc/contour_proc/contour_wrapper.hpp"

// 获取所有子轮廓的下标——实现函数
template <typename T>
void getAllSubContoursIdxImpl(const std::vector<cv::Vec4i> &hierarchy, int idx, T &sub_contours_idx)
{
    using namespace std;
    using namespace cv;

    // 如果当前轮廓没有子轮廓（下标为-1），直接返回
    if (hierarchy[idx][2] == -1)
        return;

    // 获取前子轮廓的索引（孩子在链表中的顺序）
    int front_child_idx = hierarchy[idx][2];
    while (front_child_idx != -1)
    {
        sub_contours_idx.insert(sub_contours_idx.end(), static_cast<typename T::value_type>(front_child_idx)); // 插入索引
        getAllSubContoursIdxImpl(hierarchy, front_child_idx, sub_contours_idx);                                // 递归获取子轮廓
        front_child_idx = hierarchy[front_child_idx][1];                                                       // 获取下一个兄弟轮廓
    }

    // 如果没有后子轮廓，直接返回
    if (hierarchy[hierarchy[idx][2]][0] == -1)
        return;

    // 获取后子轮廓的索引
    int back_child_idx = hierarchy[hierarchy[idx][2]][0];
    while (back_child_idx != -1)
    {
        sub_contours_idx.insert(sub_contours_idx.end(), static_cast<typename T::value_type>(back_child_idx)); // 插入索引
        getAllSubContoursIdxImpl(hierarchy, back_child_idx, sub_contours_idx);                                // 递归获取子轮廓
        back_child_idx = hierarchy[back_child_idx][0];                                                        // 获取下一个兄弟轮廓
    }
}

/**
 * @brief 获取指定轮廓的所有子轮廓的下标——接口函数
 *
 * @param hierarchy 所有轮廓的层级结构
 * @param idx 指定轮廓的下标
 *
 * @return 所有子轮廓的下标
 */
template <typename T>
void getAllSubContoursIdx(const std::vector<cv::Vec4i> &hierarchy, int idx, T &sub_contours_idx)
{
    getAllSubContoursIdxImpl<T>(hierarchy, idx, sub_contours_idx);
}
