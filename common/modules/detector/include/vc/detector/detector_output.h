#pragma once

#include "vc/core/yml_manager.hpp"
#include "vc/feature/feature_node.h"
#include "vc/dataio/dataio.h"

//! 识别器的输出参数
struct DetectorOutput
{
    //! 识别结果
    DEFINE_PROPERTY(FeatureNodes, public, public, (std::vector<FeatureNode_ptr>));
    //! 识别有效性
    DEFINE_PROPERTY(Valid, public, public, (bool));
};
