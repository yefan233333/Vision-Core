#pragma once

#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/feature/feature_node.h"
#include "vc/math/transform6D.hpp"
#include <unordered_map>
#include <unordered_set>


//! 神符靶心特征
class RuneTarget : public FeatureNode
{
    //! 激活标志位
    DEFINE_PROPERTY(ActiveFlag, public, protected,(bool));

public:
    RuneTarget() = default;
    RuneTarget(const RuneTarget &) = delete;
    RuneTarget(RuneTarget &&) = delete;

    static inline std::shared_ptr<RuneTarget> cast(FeatureNode_ptr p_feature)
    {
        return std::dynamic_pointer_cast<RuneTarget>(p_feature);
    }
};

//! 神符靶心特征共享指针
using RuneTarget_ptr = std::shared_ptr<RuneTarget>;
