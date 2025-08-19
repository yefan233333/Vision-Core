#pragma once

#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_fan_hump.h"

class RuneFanInactive : public RuneFan
{
    using Ptr = std::shared_ptr<RuneFanInactive>;

    //! 箭头轮廓组
    DEFINE_PROPERTY(ArrowContours, public, protected, (std::vector<Contour_cptr>));

public:
    RuneFanInactive() = default;
    RuneFanInactive(const RuneFanInactive &) = delete;
    RuneFanInactive(RuneFanInactive &&) = delete;
    virtual ~RuneFanInactive() = default;

    static inline Ptr cast(FeatureNode_ptr p_feature)
    {
        return std::dynamic_pointer_cast<RuneFanInactive>(p_feature);
    }

    /**
     * @brief 使用轮廓和旋转矩形构造 RuneFan(未激活)
     *
     * @param[in] hull_contour 凸包轮廓
     * @param[in] arrow_contours 箭头轮廓
     * @param[in] rotated_rect 最小面积矩形
     */
    RuneFanInactive(const Contour_cptr hull_contour, const std::vector<Contour_cptr> &arrow_contours, const cv::RotatedRect &rotated_rect);

    /**
     * @brief 使用四个顶点构造 RuneFan(未激活)
     *
     * @param[in] top_left 左上顶点
     * @param[in] top_right 右上顶点
     * @param[in] bottom_right 右下顶点
     * @param[in] bottom_left 左下顶点
     */
    RuneFanInactive(const cv::Point2f &top_left,
                                 const cv::Point2f &top_right,
                                 const cv::Point2f &bottom_right,
                                 const cv::Point2f &bottom_left);

    /**
     * @brief 找出所有未激活扇叶
     *
     * @param[out] fans 输出的扇叶数组
     * @param[in] contours 所有轮廓
     * @param[in] hierarchy 所有的等级向量
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标
     * @param[in] inactive_targets 未激活的靶心
     *
     */
    static void find(std::vector<FeatureNode_ptr> &fans,
                        const std::vector<Contour_cptr> &contours,
                        const std::vector<cv::Vec4i> &hierarchy,
                        const std::unordered_set<size_t> &mask,
                        std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs,
                        const std::vector<FeatureNode_cptr> &inactive_targets);


    /**
     * @brief 未激活扇叶的方向矫正
     * @param[in] fan 待矫正的扇叶
     * @param[in] correct_center 矫正中心
     * @return 如果矫正成功，返回 true；否则返回 false
     */
    static bool correctDirection(FeatureNode_ptr &fan, const cv::Point2f &correct_center);

    /**
     * @brief 未激活扇叶的角点矫正
     *
     * @param[in,out] fan 未激活扇叶
     * @return 是否矫正成功
     *
     * @note 矫正后的 向量（__corners[2] - __corners[1]) 、指向神符中心
     */
    static bool correctCorners(FeatureNode_ptr &fan);

    /**
     * @brief 提取末端箭头轮廓
     *
     * @param[in] fan 待提取的未激活扇叶
     * @param[in] target_inactive
     * @return 末端箭头轮廓,若返回失败则返回nullptr
     *
     * @note 1. 返回灯臂中距离靶心最远的轮廓，用于强制构造神符中心
     *        2. 此函数会重新构造 fan ,所以需要在 fan 的 correct 函数之前调用
     */
    static Contour_cptr getEndArrowContour(FeatureNode_ptr &fan, const std::vector<FeatureNode_cptr> &target_inactve);

protected:
    /**
     * @brief 未激活 RuneFan 的构造接口
     *
     * @param[in] contours 轮廓组
     * @return 如果成功，返回 RuneFanInactive 对象指针；否则返回 nullptr
     */
    static Ptr make_feature(const std::vector<Contour_cptr> &contours);
};
using RuneFanInactive_ptr = std::shared_ptr<RuneFanInactive>;
using RuneFanInactive_cptr = std::shared_ptr<const RuneFanInactive>;