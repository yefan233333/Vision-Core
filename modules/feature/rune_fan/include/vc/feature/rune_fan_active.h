#pragma once

#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_fan_hump.h"

//! 已激活的神符扇叶特征
class RuneFanActive : public RuneFan
{
    using Ptr = std::shared_ptr<RuneFanActive>;

    DEFINE_PROPERTY(RotatedRect, public, public, (cv::RotatedRect)); //!< 外接矩形
    DEFINE_PROPERTY(TopHumpCorners, protected, protected, (std::vector<cv::Point2f>));              //!< 顶部突起角点
    DEFINE_PROPERTY(BottomCenterHumpCorners, protected, protected, (std::vector<cv::Point2f>));        //!< 底部中心突起角点
    DEFINE_PROPERTY(SideHumpCorners, protected, protected, (std::vector<cv::Point2f>));                //!< 侧面突起角点
    DEFINE_PROPERTY(BottomSideHumpCorners, protected, protected, (std::vector<cv::Point2f>));   //!< 底部侧面突起角点
    DEFINE_PROPERTY(Error, protected, protected, (float));                                     //!< 误差量
public:
    RuneFanActive() = default;
    RuneFanActive(const RuneFanActive &) = delete;
    RuneFanActive(RuneFanActive &&) = delete;
    virtual ~RuneFanActive() = default;

    /**
     * @brief 已激活 RuneFan 的构造函数
     *
     * @param contour 扇叶轮廓
     * @param rotated_rect 扇叶的最小外接矩形
     * @param top_hump_corners 顶部突起角点
     * @param bottom_center_hump_corners 底部中心突起角点
     * @param side_hump_corners 侧面突起角点
     * @param bottom_side_hump_corners 底部侧面突起角点
     */
    RuneFanActive(const Contour_ptr &contour,
                  const cv::RotatedRect &rotated_rect,
                  const std::vector<cv::Point2f> &top_hump_corners,
                  const std::vector<cv::Point2f> &bottom_center_hump_corners,
                  const std::vector<cv::Point2f> &side_hump_corners,
                  const std::vector<cv::Point2f> &bottom_side_hump_corners);
    /**
     * @brief 已激活 RuneFan 的构造函数
     *
     * @param contours 扇叶轮廓点集
     * @param top_hump_corners 顶部突起角点
     * @param direction 扇叶方向
     */
    RuneFanActive(const std::vector<Contour_ptr> &contours,
                  const std::vector<cv::Point2f> &top_hump_corners,
                  const cv::Point2f &direction);

    static inline std::shared_ptr<RuneFanActive> cast(FeatureNode_ptr p_feature)
    {
        return std::dynamic_pointer_cast<RuneFanActive>(p_feature);
    }

    /**
     * @brief 找到所有已激活扇叶
     * 
     * @param[out] fans 返回找到的神符扇叶
     * @param[in] contours 输入的轮廓点集
     * @param[in] hierarchy 输入的层级结构
     * @param[in] mask 可以跳过构造的轮廓下标集合
     * @param[out] used_contour_idxs 使用了的轮廓下标集合
     */
    static void find(std::vector<FeatureNode_ptr> &fans,
                     const std::vector<Contour_ptr> &contours,
                     const std::vector<cv::Vec4i> &hierarchy,
                     const std::unordered_set<size_t> &mask,
                     std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs);

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
    static bool find_incomplete(std::vector<FeatureNode_ptr> &fans,
                                const std::vector<Contour_ptr> &contours,
                                const std::vector<cv::Vec4i> &hierarchy,
                                const std::unordered_set<size_t> &mask,
                                const cv::Point2f &rotate_center,
                                std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs);

protected:
    /**
     * @brief 已激活 RuneFan 的构造接口
     * 
     * @param[in] contour 轮廓
     * @return 若构造成功则返回指针，否则返回 nullptr
     */
    static Ptr make_feature(const Contour_ptr &contour);

    /**
     * @brief 已激活 RuneFan 的强制构造接口
     *
     * @param[in] top_corners 顶部突起角点
     * @param[in] bottom_center_corners 底部中心突起角点
     * @param[in] side_corners 侧面突起角点
     * @param[in] bottom_side_corners 底部侧面突起角点
     *
     * @return 如果成功，返回 RuneFan 的共享指针，否则返回 nullptr
     * @note 利用从PNP解算获取到的角点构造，输入的角点有顺序要求
     */
    static Ptr make_feature(const std::vector<cv::Point2f> &top_corners,
                                                 const std::vector<cv::Point2f> &bottom_center_corners,
                                                 const std::vector<cv::Point2f> &side_corners,
                                                 const std::vector<cv::Point2f> &bottom_side_corners);

    /**
     * @brief 已激活 RuneFan 的缺陷构造接口
     *
     * @param[in] hump_1 第一个突起点
     * @param[in] hump_2 第二个突起点
     * @param[in] hump_3 第三个突起点
     * @return 如果成功，返回 RuneFan 的共享指针，否则返回 nullptr
     *
     */
    static Ptr make_feature(const std::tuple<TopHump, Contour_ptr> &hump_1,
                                                 const std::tuple<TopHump, Contour_ptr> &hump_2,
                                                 const std::tuple<TopHump, Contour_ptr> &hump_3);

    /**
     * @brief 通过扇叶位姿PNP解算结果构造 RuneFan
     *
     * @param[in] fan_to_cam 扇叶相对于相机的位姿解算结果
     * @param[in] is_active 是否激活？
     */
    static Ptr make_feature(const Transform6D &fan_to_cam, bool is_active);

public:
    /**
     * @brief 获取轮廓角度数组
     *
     * @param[in] contour_plus 加长后的轮廓
     * @return 轮廓角度数组
     */
    static cv::Mat getAngles(const std::vector<cv::Point> &contour_plus);

    /**
     * @brief 获取角度矩阵
     *
     * @param[in] angles_mat 角度数组
     * @return 梯度数组
     */
    static cv::Mat getGradient(const cv::Mat &angles_mat);

    /**
     * @brief 获取所有的直线对
     *
     * @param[in] contour_plus 加长后的轮廓
     * @param[in] angles_mat 角度矩阵
     * @param[in] gradient_mat 梯度矩阵
     * @param[out] line_pairs 输出的直线对
     */
    static bool getLinePairs(const std::vector<cv::Point> &contour_plus,
                             const cv::Mat &angles_mat,
                             const cv::Mat &gradient_mat,
                             std::vector<std::tuple<Line, Line>> &line_pairs);

    /**
     * @brief 获取角点在图像坐标系和特征坐标系下的坐标
     *
     * @return [0] 图像坐标系 [1] 特征坐标系 [2] 各个点的权重
     */
    virtual auto getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> override;

    /**
     * @brief 获取已激活扇叶的所有角点
     *
     * @param[in] contour 轮廓
     * @param[out] top_hump_corners 输出的扇叶顶部角点
     * @param[out] bottom_center_hump_corners   输出的扇叶的底部中心角点
     * @param[out] side_hump_corners 输出的扇叶的侧面角点
     * @param[out] bottom_side_hump_corners 输出的扇叶的底部侧面角点
     * @param[out] direction 输出的扇叶方向单位向量，指向神符中心
     * @return 是否获取成功
     */
    static bool getActiveFunCorners(const Contour_ptr &contour,
                                    std::vector<cv::Point2f> &top_hump_corners,
                                    std::vector<cv::Point2f> &bottom_center_hump_corners,
                                    std::vector<cv::Point2f> &side_hump_corners,
                                    std::vector<cv::Point2f> &bottom_side_hump_corners);

};
using RuneFanActive_ptr = std::shared_ptr<RuneFanActive>;
