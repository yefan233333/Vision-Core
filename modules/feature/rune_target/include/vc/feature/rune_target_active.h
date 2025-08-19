#pragma once

#include "rune_target.h"

//! 神符已激活靶心类
class RuneTargetActive : public RuneTarget
{
    using Ptr = std::shared_ptr<RuneTargetActive>;
public:
    RuneTargetActive() = default;
    RuneTargetActive(const RuneTargetActive &) = delete;
    RuneTargetActive(RuneTargetActive &&) = delete;
    virtual ~RuneTargetActive() = default;

    /**
     * @brief 动态类型转换
     * @param p_feature feature_ptr 抽象指针
     * @return 派生对象指针
     */
    static inline Ptr cast(FeatureNode_ptr p_feature)
    {
        return std::dynamic_pointer_cast<RuneTargetActive>(p_feature);
    }

    /**
     * @brief 找到所有靶心
     *
     * @param[out] targets 返回找到的靶心
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find(std::vector<FeatureNode_ptr> &targets,
                     const std::vector<Contour_ptr> &contours,
                     const std::vector<cv::Vec4i> &hierarchy,
                     const std::unordered_set<size_t> &mask,
                     std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs);

private:
    /**
     * @brief 未激活 RuneTarget 的构造接口
     *
     * @param[in] contours 所有轮廓
     * @param[in] hierarchy 所有的等级向量
     * @param[in] idx 最外层轮廓的下标
     * @param[out] used_contour_idxs 使用了的轮廓下标
     */
    static Ptr make_feature(const std::vector<Contour_ptr> &contours,
                            const std::vector<cv::Vec4i> &hierarchy,
                            size_t idx,
                            std::unordered_set<size_t> &used_contour_idxs);
    /**
     * @brief 获取所有PNP角点
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> override;
};
using RuneTargetActive_ptr = std::shared_ptr<RuneTargetActive>;