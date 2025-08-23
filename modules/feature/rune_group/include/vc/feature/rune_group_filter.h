/**
 * @file rune_group_filter.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 神符序列组滤波策略头文件
 * @version 1.0
 * @date 2025-03-22
 * @copyright Copyright (c) 2025
 */

#pragma once

#include <deque>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

#include "vc/math/pose_node.hpp"
#include "rune_filter_type.h"

/**
 * @brief 神符位姿滤波策略基类
 *
 * 提供滤波流程的基础接口，支持派生类实现不同的滤波方法。
 */
class RuneFilterStrategy
{
protected:
    size_t __filter_count = 0;   //!< 已执行的滤波次数
    cv::Matx61d __latest_value;  //!< 最近一次滤波结果

public:
    virtual ~RuneFilterStrategy() = default;

    /**
     * @brief 滤波输入参数
     *
     * @note
     * 1. yaw、pitch、roll 单位均为角度
     * 2. pitch 角范围限制为 [-85, 85]，以避免万向节锁死
     * 3. yaw 和 roll 角需提前处理连续性
     */
    struct FilterInput
    {
        cv::Matx61d raw_pos;     //!< 原始位姿 [x y z | yaw pitch roll]
        int64 tick;              //!< 时间戳（单位：毫秒或纳秒）
        PoseNode cam_to_gyro;    //!< 相机到陀螺仪的位姿转换结果
        bool is_observation = false; //!< 是否为有效观测数据
    };

    /**
     * @brief 滤波输出参数
     */
    struct FilterOutput
    {
        cv::Matx61d filtered_pos; //!< 滤波后的位姿 [x y z | yaw pitch roll]
    };

    /**
     * @brief 滤波核心函数
     * @param input 滤波输入参数
     * @return FilterOutput 滤波输出结果
     */
    virtual FilterOutput filter(FilterInput input) = 0;

    /**
     * @brief 获取最新预测结果
     * @return 预测位姿 [x y z | yaw pitch roll]
     */
    virtual cv::Matx61d getPredict() = 0;

    /**
     * @brief 获取滤波器当前最新值
     * @return 最新滤波结果
     */
    inline cv::Matx61d getLatest() { return cv::Matx61d::zeros(); }

    /**
     * @brief 判断当前滤波器是否有效
     * @return true 有效，false 无效
     */
    virtual bool isValid() = 0;

    /**
     * @brief 获取滤波数据类型
     * @return RuneFilterDataType 枚举值
     */
    virtual RuneFilterDataType getDataType() = 0;

    /**
     * @brief 获取已执行的滤波次数
     * @return 滤波次数
     */
    size_t getFilterCount() const { return __filter_count; }

    /**
     * @brief 获取滤波器最新结果
     * @return 最近一次滤波结果
     */
    cv::Matx61d getLatestValue() const { return __latest_value; }
};

using RuneFilter_ptr = std::shared_ptr<RuneFilterStrategy>;
