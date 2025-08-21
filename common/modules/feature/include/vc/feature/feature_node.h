#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <memory>
#include <unordered_map>

#include "vc/core/property_wrapper.hpp"
#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/math/pose_node.hpp"
#include "vc/core/type_utils.h"
#include "vc/dataio/dataio.h"

/**
 * @brief 特征节点
 *
 * @note 用于承载视觉识别的特征节点基类
 */
class FeatureNode : public std::enable_shared_from_this<FeatureNode>
{
    using Ptr = std::shared_ptr<FeatureNode>;                    //! 指针类型
    using FeatureNodeMap = std::unordered_map<std::string, Ptr>; //! 特征节点映射表类型
public:
    using DrawMask = std::uint64_t;

public:
    /**
     * @brief 图像信息缓存块
     */
    struct ImageCache
    {
        //! 轮廓组
        DEFINE_PROPERTY(Contours, public, public, (std::vector<Contour_cptr>));
        //! 角点集
        DEFINE_PROPERTY(Corners, public, public, (std::vector<cv::Point2f>));
        //! 中心位置
        DEFINE_PROPERTY(Center, public, public, (cv::Point2f));
        //! 方向
        DEFINE_PROPERTY(Direction, public, public, (cv::Point2f));
        //! 宽(图像中)
        DEFINE_PROPERTY(Width, public, public, (float));
        //! 高(图像中)
        DEFINE_PROPERTY(Height, public, public, (float));
    };

    /**
     * @brief 位姿信息缓存块
     */
    struct PoseCache
    {
        //! 位姿节点映射表类型
        using PoseNodeMap = std::unordered_map<std::string, PoseNode>;
        //! 位姿节点映射
        DEFINE_PROPERTY_WITH_INIT(PoseNodes, public, public, (PoseNodeMap), PoseNodeMap{});
        //! 陀螺仪位姿信息
        DEFINE_PROPERTY(GyroPose, public, public, (GyroData));
    };

    //! 绘制配置结构体
    struct DrawConfig;
    using DrawConfig_ptr = std::shared_ptr<DrawConfig>;
    using DrawConfig_cptr = std::shared_ptr<const DrawConfig>;

    //! 子特征节点映射表类型
    struct ChildFeatureType;
private:
    //! 图像信息缓存
    DEFINE_PROPERTY_WITH_INIT(ImageCache, public, protected, (ImageCache), ImageCache());
    //! 位姿信息缓存
    DEFINE_PROPERTY_WITH_INIT(PoseCache, public, protected, (PoseCache), PoseCache());
    //! 子特征节点映射表
    DEFINE_PROPERTY_WITH_INIT(ChildFeatures, public, protected, (FeatureNodeMap), FeatureNodeMap());
    //! 构建时间戳
    DEFINE_PROPERTY(Tick, public, protected, (int64_t));

public:
    /**
     * @brief 特征节点构造函数
     */
    FeatureNode() = default;

    /**
     * @brief 特征节点析构函数
     */
    virtual ~FeatureNode() = default;

    /**
     * @brief 访问图像信息缓存
     */
    virtual const ImageCache &imageCache() const noexcept { return this->getImageCache(); }

    /**
     * @brief 访问位姿节点缓存
     */
    virtual const PoseCache &poseCache() const noexcept { return this->getPoseCache(); }

    /**
     * @brief 获取子特征节点映射
     */
    virtual const FeatureNodeMap &childFeatures() const noexcept { return this->getChildFeatures(); }

    /**
     * @brief 绘制特征
     *
     * @param image 要绘制的图像
     * @param config 绘制配置
     *
     * @note - 该方法用于在图像上绘制特征节点的可视化表示
     *
     *       - 默认实现为空，子类可以重载此方法以实现具体的绘制逻辑
     */
    virtual void drawFeature(cv::Mat &image, const DrawConfig_cptr &config = nullptr) const
    {
        // 默认实现为空，子类可以重载此方法以实现具体的绘制逻辑
        (void)image; // 避免未使用参数警告
        (void)config; // 避免未使用参数警告
    }

protected:
};

using FeatureNode_ptr = std::shared_ptr<FeatureNode>;
using FeatureNode_cptr = std::shared_ptr<const FeatureNode>;

#include "feature_node_draw_config.h" // 包含绘制配置定义
#include "feature_node_child_feature_type.h" // 包含子特征节点类型定义