#pragma once

#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/feature/feature_node.h"
#include "vc/math/transform6D.hpp"
#include <unordered_map>
#include <unordered_set>

//! 神符靶心特征
class RuneTarget : public FeatureNode
{
    using Ptr = std::shared_ptr<RuneTarget>;
    //! 激活标志位
    DEFINE_PROPERTY(ActiveFlag, public, protected, (bool));

public:
    RuneTarget() = default;
    RuneTarget(const RuneTarget &) = delete;
    RuneTarget(RuneTarget &&) = delete;
    virtual ~RuneTarget() = default;

    static inline std::shared_ptr<RuneTarget> cast(FeatureNode_ptr p_feature)
    {
        return std::dynamic_pointer_cast<RuneTarget>(p_feature);
    }

    /**
     * @brief 找到所有未激活靶心
     *
     * @param[out] targets 返回找到的未激活靶心
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find_inactive_targets(std::vector<FeatureNode_ptr> &targets,
                                      const std::vector<Contour_cptr> &contours,
                                      const std::vector<cv::Vec4i> &hierarchy,
                                      const std::unordered_set<size_t> &mask,
                                      std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 找到所有激活靶心
     *
     * @param[out] targets 返回找到的激活靶心
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find_active_targets(std::vector<FeatureNode_ptr> &targets,
                                    const std::vector<Contour_cptr> &contours,
                                    const std::vector<cv::Vec4i> &hierarchy,
                                    const std::unordered_set<size_t> &mask,
                                    std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 获取角点在图像坐标系和特征坐标系下的坐标
     *
     * @return [0] 图像坐标系 [1] 特征坐标系 [2] 各个点的权重
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>;

    /**
     * @brief 获取角点在图像坐标系和旋转中心坐标系下的坐标
     *
     * @return [0] 图像坐标系 [1] 旋转中心坐标系 [2] 各个点的权重
     */
    auto getRelativePnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>;
};

//! 神符靶心特征共享指针
using RuneTarget_ptr = std::shared_ptr<RuneTarget>;
using RuneTarget_cptr = std::shared_ptr<const RuneTarget>;
