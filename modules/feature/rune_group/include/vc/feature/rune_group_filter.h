/**
 * @file rune_group_filter.h
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 神符序列组的滤波策略头文件
 * @version 1.0
 * @date 2025-3-22
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <deque>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

#include "vc/math/pose_node.hpp"

/**
 * @brief 神符位姿的滤波策略
 */
class RuneFilterStrategy
{
protected:
    size_t __filter_count = 0;  //!< 滤波次数
    cv::Matx61d __latest_value; //!< 滤波器的最新值
public:
    virtual ~RuneFilterStrategy() = default;

    /**
     * @brief 滤波输入结构体
     *
     * @note 1. yaw pitch roll 单位为角度
     *      2. pitch 角限制在 [-85, 85]，避免万向节锁死
     *      3. yaw 角和 roll 角都需要提前做好连续性处理
     */
    struct FilterInput
    {
        cv::Matx61d raw_pos;          //!< 原始位姿
        int64 tick;                   //!< 时间戳
        PoseNode cam_to_gyro; //!< 相机到陀螺仪的转换结果
        bool is_observation = false;  //!< 是否为有效观测数据
    };

    /**
     * @brief 滤波输出结构体
     */
    struct FilterOutput
    {
        cv::Matx61d filtered_pos; //!< 滤波后的位姿 滤波后的位姿 [x y z | yaw pitch roll]
    };

    /**
     * @brief 滤波主函数
     * @param input 滤波输入
     * @return FilterOutput 滤波输出
     */
    virtual FilterOutput filter(FilterInput input) = 0;

    /**
     * @brief 获取最新帧的预测值
     *
     * @return cv::Matx61d 预测值
     */
    virtual cv::Matx61d getPredict() = 0;

    /**
     * @brief 获取滤波器的最新值
     *
     * @return cv::Matx61d 滤波器的最新值
     */
    inline cv::Matx61d getLatest() {}

    //! 是否有效
    virtual bool isValid() = 0;
    //! 获取滤波数据类型
    //! 获取滤波次数
    size_t getFilterCount() const { return __filter_count; }
    //! 获取滤波器的最新值
    cv::Matx61d getLatestValue() const { return __latest_value; }
};

using RuneFilter_ptr = std::shared_ptr<RuneFilterStrategy>;
