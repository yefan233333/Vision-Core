#pragma once
#include "rune_group_filter.h"
#include "rune_filter_type.h"

class RuneFilterFusion : public RuneFilterStrategy
{
public:
    RuneFilterFusion() = default;

    static std::shared_ptr<RuneFilterStrategy> make_filter();

    /**
     * @brief 滤波主函数
     * @param input 滤波输入
     * @return FilterOutput 滤波输出
     */
    virtual FilterOutput filter(FilterInput input) override;

    /**
     * @brief 获取最新帧的预测值
     *
     * @return cv::Matx61d 预测值
     *
     */
    cv::Matx61d getPredict() override;

    /**
     * @brief 是否有效
     */
    bool isValid() override;

    /**
     * @brief 获取滤波数据类型
     */
    RuneFilterDataType getDataType() override { return RuneFilterDataType::XYZ | RuneFilterDataType::YAW_PITCH | RuneFilterDataType::ROLL; }

protected:
    /**
     * @brief 初始化滤波器
     *
     * @param[in] first_pos 第一次滤波的位置
     * @param[in] tick 时间戳
     */
    void initFilter(const cv::Matx61d &first_pos, const int64 tick);

protected:
    bool _is_init_filter = false;         //!< 是否初始化滤波器
    std::vector<RuneFilter_ptr> _filters; //!< 子滤波器
};

using RuneFilterFusion_ptr = std::shared_ptr<RuneFilterFusion>;