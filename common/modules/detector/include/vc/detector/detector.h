#pragma once

#include "vc/core/yml_manager.hpp"
#include "vc/feature/feature_node.h"

struct DetectorInput;
struct DetectorOutput;

//! 识别器基类
class Detector
{
public:
    virtual ~Detector() = default;

    /**
     * @brief 识别器的主函数
     * @param input 识别器输入
     * @param output 识别器输出
     */
    virtual void detect(DetectorInput& input, DetectorOutput& output) = 0;

};
using Detector_ptr = std::shared_ptr<Detector>;


#include "detector_input.h"     // 存放 DetectorInput 结构体
#include "detector_output.h"    // 存放 DetectorOutput 结构体