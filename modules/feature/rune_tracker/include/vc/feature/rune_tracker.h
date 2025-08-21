#pragma once

#include "vc/feature/tracking_feature_node.h"

//! 神符时间序列
class RuneTracker : public TrackingFeatureNode
{
    using Ptr = std::shared_ptr<RuneTracker>;

    //! 掉帧数
    DEFINE_PROPERTY_WITH_INIT(DropFrameCount, public, protected, (int), 0);
public:
    RuneTracker() = default;
    RuneTracker(const RuneTracker &) = delete;
    RuneTracker(RuneTracker &&) = delete;
    virtual ~RuneTracker() = default;

    /**
     * @brief 构建 RuneTracker
     *
     * @param[in] p_rune 第一帧神符模块组合特征（不允许为空）
     */
    static Ptr make_feature(){return std::make_shared<RuneTracker>();};

    /**
     * @brief 动态类型转换
     *
     * @param[in] p_tracker tracker_ptr 抽象指针
     * @return 派生对象指针
     */
    static inline Ptr cast(FeatureNode_ptr p_tracker)
    {
        return std::dynamic_pointer_cast<RuneTracker>(p_tracker);
    }

    /**
     * @brief 更新时间序列
     *
     * @param[in] p_rune 神符共享指针
     * @param[in] tick 时间戳
     * @param[in] gyro_data 云台数据
     */
    void update(FeatureNode_ptr p_rune, int64 tick, const GyroData &gyro_data);

    /**
     * @brief 可见性更新
     *
     * @param[in] is_visible 是否可见
     */
    void updateVisible(bool is_visible);

private:
    /**
     * @brief 从 rune 中更新数据
     *
     * @param[in] p_combo 神符组合体
     */
    void updateFromRune(FeatureNode_ptr p_combo);
};
//! 神符追踪器智能指针类型定义
using RuneTracker_ptr = std::shared_ptr<RuneTracker>;
using RuneTracker_cptr = std::shared_ptr<const RuneTracker>;


