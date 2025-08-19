#pragma once

#include "vc/feature/feature_node.h"

//! 神符扇叶类
class RuneFan : public FeatureNode
{
    using Ptr = std::shared_ptr<RuneFan>;
    //! 激活标志位
    DEFINE_PROPERTY(ActiveFlag, public, protected, (bool));
    DEFINE_PROPERTY(Direction, public, protected, (cv::Point2f)); //!< 方向
    DEFINE_PROPERTY(RotatedRect, public, protected, (cv::RotatedRect)); //!< 外接矩形

public:
    RuneFan() = default;
    RuneFan(const RuneFan &) = delete;
    RuneFan(RuneFan &&) = delete;
    virtual ~RuneFan() = default;

    static inline std::shared_ptr<RuneFan> cast(FeatureNode_ptr p_feature)
    {
        return std::dynamic_pointer_cast<RuneFan>(p_feature);
    }

    /**
     * @brief 找到所有未激活扇叶
     *
     * @param[out] fans 返回找到的未激活扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find_inactive_fans(std::vector<FeatureNode_ptr> &fans,
                                   const std::vector<Contour_ptr> &contours,
                                   const std::vector<cv::Vec4i> &hierarchy,
                                   const std::unordered_set<size_t> &mask,
                                   std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 找到所有激活扇叶
     *
     * @param[out] fans 返回找到的激活扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     * @param[in] inactive_targets 未激活的靶心特征
     */
    static void find_active_fans(std::vector<FeatureNode_ptr> &fans,
                                 const std::vector<Contour_ptr> &contours,
                                 const std::vector<cv::Vec4i> &hierarchy,
                                 const std::unordered_set<size_t> &mask,
                                 std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs,
                                 const std::vector<FeatureNode_ptr> &inactive_targets);

    /**
     * @brief 找到所有残缺的已激活扇叶
     * 
     * @param[out] fans 返回找到的残缺的已激活扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[in] rotate_center 旋转中心
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find_incomplete_active_fans(std::vector<FeatureNode_ptr> &fans,
                                            const std::vector<Contour_ptr> &contours,
                                            const std::vector<cv::Vec4i> &hierarchy,
                                            const std::unordered_set<size_t> &mask,
                                            const cv::Point2f &rotate_center,
                                            std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs);

    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> {};
};
//! 神符扇叶特征共享指针
using RuneFan_ptr = std::shared_ptr<RuneFan>;
