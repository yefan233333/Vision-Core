#pragma once

#include "rune_target.h"

//! 神符靶心特征的缺口
struct RuneTargetGap
{
    cv::Point2f left_corner;  //!< 左角点
    cv::Point2f right_corner; //!< 右角点
    cv::Point2f center;       //!< 中心点
    bool is_valid = true;     //!< 是否有效
};

//! 神符未激活靶心
class RuneTargetInactive : public RuneTarget
{
    using Ptr = std::shared_ptr<RuneTargetInactive>;

public:
    RuneTargetInactive() = default;
    RuneTargetInactive(const RuneTargetInactive &) = delete;
    RuneTargetInactive(RuneTargetInactive &&) = delete;
    virtual ~RuneTargetInactive() = default;

    /**
     * @brief 动态类型转换
     * @param p_feature feature_ptr 抽象指针
     * @return 派生对象指针
     */
    static inline Ptr cast(FeatureNode_ptr p_feature)
    {
        return std::dynamic_pointer_cast<RuneTargetInactive>(p_feature);
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
                     const std::vector<Contour_cptr> &contours,
                     const std::vector<cv::Vec4i> &hierarchy,
                     const std::unordered_set<size_t> &mask,
                     std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs);

    /**
     * @brief 未激活靶心的矫正
     *
     * @param[in] rotate_center 旋转中心
     */
    bool correct(const cv::Point2f &rotate_center);

protected:
    /**
     * @brief 未激活 RuneTarget 的构造接口
     *
     * @param[in] contours 所有轮廓
     * @param[in] hierarchy 所有的等级向量
     * @param[in] idx 最外层轮廓的下标
     * @param[out] used_contour_idxs 使用了的轮廓下标
     */
    static Ptr make_feature(const std::vector<Contour_cptr> &contours,
                            const std::vector<cv::Vec4i> &hierarchy,
                            size_t idx,
                            std::unordered_set<size_t> &used_contour_idxs);

    /**
     * @brief 通过神符位姿PNP解算结果构造 RuneTarget
     *
     * @param[in] target_to_cam 靶心相对于相机的位姿解算结果
     * @param[in] is_active 是否激活
     */
    static Ptr make_feature(Transform6D &target_to_cam, bool is_active);

    /**
     * @brief 未激活靶心的方向修正
     *
     */
    bool correctDirection();

    /**
     * @brief 未激活扇叶的角点修正
     */
    bool correctCorners(const cv::Point2f &rune_center);

    /**
     * @brief 未激活靶心的缺口排序
     *
     * @param[in] rune_center 神符中心
     */
    bool sortedGap(const cv::Point2f &rune_center);

    /**
     * @brief 设置缺口角点
     *
     * @param[in] rune_center 神符中心
     */
    bool calcGapCorners(const cv::Point2f &rune_center);

    /**
     * @brief 获取所有PNP角点
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> override;

protected:

    DEFINE_PROPERTY(Direction, public, protected, (cv::Point2f));                       //!< 方向
    DEFINE_PROPERTY(Gaps, public, protected, (std::vector<RuneTargetGap>)); //!< 缺口.(未排序的缺口)，排序后的顺序为：左上、右上、右下、左下
    DEFINE_PROPERTY(LeftTopGap, public, protected, (RuneTargetGap*)); //!< 左上缺口
    DEFINE_PROPERTY(RightTopGap, public, protected, (RuneTargetGap*)); //!< 右上缺口
    DEFINE_PROPERTY(RightBottomGap, public, protected, (RuneTargetGap*)); //!< 右下缺口
    DEFINE_PROPERTY(LeftBottomGap, public, protected, (RuneTargetGap*)); //!< 左下缺口
    DEFINE_PROPERTY_WITH_INIT(GapSortedFlag, public, protected, (bool), false); //!< 缺口是否已排序
    DEFINE_PROPERTY_WITH_INIT(GapCorners, public, protected, (std::vector<cv::Point2f>),{}); //!< 缺口角点占位点

    inline static const cv::Point2f VACANCY_POINT = cv::Point2f(-1, -1); //!< 默认占位点
};
using RuneTargetInactive_ptr = std::shared_ptr<RuneTargetInactive>;
using RuneTargetInactive_cptr = std::shared_ptr<const RuneTargetInactive>;
