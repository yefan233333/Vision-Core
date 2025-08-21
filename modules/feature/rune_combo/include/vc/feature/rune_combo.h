#pragma once

#include "vc/feature/feature_node.h"

//! 单片神符的状态类型
enum class RuneType : uint8_t
{
    UNKNOWN,        //!< 未知
    STRUCK,         //!< 已击打
    UNSTRUCK,       //!< 未击打
    PENDING_STRUCK, //!< 待击打
};

//! 神符组合体
class RuneCombo : public FeatureNode
{
    using Ptr = std::shared_ptr<RuneCombo>;
    DEFINE_PROPERTY(RuneType, public, protected, (RuneType)); //!< 神符状态类型
public:
    RuneCombo() = delete;

    /**
     * @brief Rune 构造接口
     *
     * @param[in] p_target 神符靶心
     * @param[in] p_center 神符中心
     * @param[in] p_fan 神符扇叶
     * @param[in] rune_to_cam PnP 数据
     * @param[in] type 神符类型
     * @param[in] gyro_data 陀螺仪数据
     * @param[in] tick 捕获特征的时间戳
     */
    RuneCombo(const FeatureNode_ptr &p_target,
              const FeatureNode_ptr &p_center,
              const FeatureNode_ptr &p_fan,
              const PoseNode &rune_to_cam,
              const RuneType &type,
              const GyroData &gyro_data,
              int64 tick);

    RuneCombo(const RuneCombo &) = delete;
    RuneCombo(RuneCombo &&) = delete;
    /**
     * @brief 通过 PnP 数据构造 Rune
     *
     * @param[in] rune_to_cam PnP 数据
     * @param[in] type 神符类型
     *
     * @return 构造成功返回 Rune 指针，否则返回 nullptr
     */
    static Ptr make_feature(const PoseNode &pnp_data, const RuneType &type, const GyroData &gyro_data, int64 tick);

    /**
     * @brief 动态类型转换
     *
     * @param[in] p_combo combo_ptr 抽象指针
     * @return 派生对象指针
     */
    static inline Ptr cast(FeatureNode_ptr p_combo)
    {
        return std::dynamic_pointer_cast<RuneCombo>(p_combo);
    }

private:
};

//! 神符组合体智能指针类型
using RuneCombo_ptr = std::shared_ptr<RuneCombo>;
using RuneCombo_cptr = std::shared_ptr<const RuneCombo>;
