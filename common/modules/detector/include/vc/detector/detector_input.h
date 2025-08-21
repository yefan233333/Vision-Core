#pragma once

#include "vc/core/yml_manager.hpp"
#include "vc/feature/feature_node.h"
#include "vc/dataio/dataio.h"

//! 识别器的输入参数
struct DetectorInput
{
    //! 输入图像
    DEFINE_PROPERTY(Image, public, public, (cv::Mat));
    //! 时间戳
    DEFINE_PROPERTY(Tick, public, public, (int64_t));
    //! 陀螺仪信息
    DEFINE_PROPERTY(GyroData, public, public, (GyroData));
    //! 特征节点数组
    DEFINE_PROPERTY(FeatureNodes, public, public, (std::vector<FeatureNode>));
};
