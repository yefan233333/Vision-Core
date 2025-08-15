#pragma once

#include <opencv2/core.hpp>

#include "vc/core/type_expansion.hpp"

//----------------------【OpenCV 类型拓展】-----------------------
namespace cv
{
    //! 矩阵
    using Matx51f = cv::Matx<float, 5, 1>; //!< 5行1列的矩阵
    using Matx52f = cv::Matx<float, 5, 2>; //!< 5行2列的矩阵
    using Matx53f = cv::Matx<float, 5, 3>; //!< 5行3列的矩阵
    using Matx54f = cv::Matx<float, 5, 4>; //!< 5行4列的矩阵
    using Matx55f = cv::Matx<float, 5, 5>; //!< 5行5列的矩阵
}
