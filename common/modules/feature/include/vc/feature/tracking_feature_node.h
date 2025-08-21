#pragma once

#include "vc/feature/feature_node.h"

//! 追踪特征节点
class TrackingFeatureNode : public FeatureNode
{
    using Ptr = std::shared_ptr<TrackingFeatureNode>;

    //! 历史追踪节点
    DEFINE_PROPERTY_WITH_INIT(HistoryNodes, public, protected, (std::deque<FeatureNode_cptr>), {}); //!< 历史追踪节点
    //! 历史追踪时间戳
    DEFINE_PROPERTY_WITH_INIT(HistoryTicks, public, protected, (std::deque<int64_t>), {}); //!< 历史追踪时间戳
public:
    TrackingFeatureNode() = default;
    TrackingFeatureNode(const TrackingFeatureNode &) = delete;
    TrackingFeatureNode(TrackingFeatureNode &&) = delete;
    virtual ~TrackingFeatureNode() = default;

    static inline std::shared_ptr<TrackingFeatureNode> cast(FeatureNode_ptr p_feature)
    {
        return std::dynamic_pointer_cast<TrackingFeatureNode>(p_feature);
    }

};
//! 追踪特征节点智能指针类型定义
using TrackingFeatureNode_ptr = std::shared_ptr<TrackingFeatureNode>;
using TrackingFeatureNode_cptr = std::shared_ptr<const TrackingFeatureNode>;