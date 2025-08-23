/**
 * @file rune_filter_ekf_cv.h
 * @brief 基于 OpenCV 的扩展卡尔曼滤波器实现
 */

#pragma once

#include "rune_group_filter.h"
#include <opencv2/core.hpp>
#include <optional>

class RuneFilterEKF_CV : public RuneFilterStrategy
{
public:
    RuneFilterEKF_CV(RuneFilterDataType type);
    virtual ~RuneFilterEKF_CV() {}

    static std::shared_ptr<RuneFilterEKF_CV> make_filter(RuneFilterDataType type)
    {
        return std::make_shared<RuneFilterEKF_CV>(type);
    }
    
    FilterOutput filter(FilterInput input) override;
    cv::Matx61d getPredict() override;
    bool isValid() override;
    RuneFilterDataType getDataType() override { return _data_type; }

private:
    RuneFilterDataType _data_type = RuneFilterDataType::XYZ;
    bool initialized_ = false;
    int64 last_tick_ = 0;
    double dt_ = 0.0;

    cv::Mat x_; ///< 状态向量 (12x1)
    cv::Mat P_; ///< 协方差矩阵 (12x12)
    cv::Mat Q_; ///< 过程噪声 (12x12)
    cv::Mat R_; ///< 观测噪声 (6x6)
    cv::Mat H_; ///< 观测矩阵 (6x12)

    void predict(double dt);
    void update(const cv::Mat &z);
};
