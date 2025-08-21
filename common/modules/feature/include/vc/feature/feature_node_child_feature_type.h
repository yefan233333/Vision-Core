#pragma once

#include <string>
#include "feature_node.h"


//! 子特征类型枚举
struct FeatureNode::ChildFeatureType
{
    static std::string RUNE_TARGET; //!< 神符靶心
    static std::string RUNE_CENTER; //!< 神符中心
    static std::string RUNE_FAN;    //!< 神符扇叶
};

inline std::string FeatureNode::ChildFeatureType::RUNE_TARGET = "rune_target";
inline std::string FeatureNode::ChildFeatureType::RUNE_CENTER = "rune_center";
inline std::string FeatureNode::ChildFeatureType::RUNE_FAN = "rune_fan";
