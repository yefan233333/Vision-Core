#pragma once

#include <opencv2/core.hpp>

#include "vc/math/type_utils.hpp"
#include "vc/core/logging.h"

//! 角度制
enum AngleMode : bool
{
    RAD = true,
    DEG = false
};

//! 欧拉角转轴枚举
enum EulerAxis : int
{
    X = 0, //!< X 轴
    Y = 1, //!< Y 轴
    Z = 2  //!< Z 轴
};

// -------------------- 角度规范化函数 (取代宏) --------------------
template <typename T>
inline T NormalizeDegree(T degrees)
{
    while (degrees > 180)
        degrees -= 360;
    while (degrees <= -180)
        degrees += 360;
    return degrees;
}

template <typename T>
inline T NormalizeRadian(T radians)
{
    while (radians > CV_PI)
        radians -= 2 * CV_PI;
    while (radians <= -CV_PI)
        radians += 2 * CV_PI;
    return radians;
}

// ------------------------【常用变换公式】------------------------

/**
 * @brief 角度转换为弧度
 *
 * @tparam Tp 变量类型
 * @param[in] deg 角度
 * @return 弧度
 */
template <typename Tp>
inline Tp deg2rad(Tp deg)
{
    return deg * static_cast<Tp>(CV_PI) / static_cast<Tp>(180);
}

/**
 * @brief 弧度转换为角度
 *
 * @tparam Tp 变量类型
 * @param[in] rad 弧度
 * @return 角度
 */
template <typename Tp>
inline Tp rad2deg(Tp rad) { return rad * static_cast<Tp>(180) / static_cast<Tp>(CV_PI); }

/**
 * @brief Point类型转换为Matx类型
 *
 * @tparam Tp 数据类型
 * @param[in] point Point类型变量
 * @return Matx类型变量
 */
template <typename Tp>
inline cv::Matx<Tp, 3, 1> point2matx(cv::Point3_<Tp> point) { return cv::Matx<Tp, 3, 1>(point.x, point.y, point.z); }

/**
 * @brief Matx类型转换为Point类型
 *
 * @tparam Tp 数据类型
 * @param[in] matx Matx类型变量
 * @return Point类型变量
 */
template <typename Tp>
inline cv::Point3_<Tp> matx2point(cv::Matx<Tp, 3, 1> matx) { return cv::Point3_<Tp>(matx(0), matx(1), matx(2)); }

/**
 * @brief Matx类型转换为Vec类型
 *
 * @tparam Tp 数据类型
 * @param[in] matx Matx类型变量
 * @return Vec类型变量
 */
template <typename Tp>
inline cv::Vec<Tp, 3> matx2vec(cv::Matx<Tp, 3, 1> matx) { return cv::Vec<Tp, 3>(matx(0), matx(1), matx(2)); }

// ------------------------【几何距离计算】------------------------

/**
 * @brief 计算二维向量之间的欧氏距离
 *
 * @param v1 第一个向量
 * @param v2 第二个向量
 * @return 两个向量之间的距离
 */
template <geom_utils_concepts::vector_2_type Vec1,
          geom_utils_concepts::vector_2_type Vec2>
inline auto getDist(const Vec1 &v1, const Vec2 &v2) noexcept
{
    using namespace geom_utils_concepts;
    const auto dx = get_x(v1) - get_x(v2);
    const auto dy = get_y(v1) - get_y(v2);
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 计算三维向量之间的欧氏距离
 *
 * @param v1 第一个三维向量
 * @param v2 第二个三维向量
 * @return 两个三维向量之间的距离
 */
template <geom_utils_concepts::vector_3_type Vec1,
          geom_utils_concepts::vector_3_type Vec2>
inline auto getDist(const Vec1 &v1, const Vec2 &v2) noexcept
{
    using namespace geom_utils_concepts;
    const auto dx = get_x(v1) - get_x(v2);
    const auto dy = get_y(v1) - get_y(v2);
    const auto dz = get_z(v1) - get_z(v2);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 平面向量外积计算
 *
 * @param v1 第一个向量
 * @param v2 第二个向量
 * @return 外积结果 , 若 retval = 0,则共线
 */
template <geom_utils_concepts::vector_2_type Vec1,
          geom_utils_concepts::vector_2_type Vec2>
inline auto getCross(const Vec1 &v1, const Vec2 &v2) noexcept
{
    using namespace geom_utils_concepts;
    const auto v1_x = get_x(v1);
    const auto v1_y = get_y(v1);
    const auto v2_x = get_x(v2);
    const auto v2_y = get_y(v2);
    return v1_x * v2_y - v1_y * v2_x; // 返回外积结果
}

/**
 * @brief 欧拉角转换为旋转矩阵
 *
 * @tparam _Tp 数据类型
 * @param[in] val 角度数值（弧度制）
 * @param[in] axis 转轴
 * @return Matx 格式的旋转矩阵
 */
template <typename _Tp>
inline cv::Matx<_Tp, 3, 3> euler2Mat(_Tp val, EulerAxis axis)
{
    _Tp s = std::sin(val), c = std::cos(val);
    switch (axis)
    {
    case X:
        return {1, 0, 0, 0, c, -s, 0, s, c};
    case Y:
        return {c, 0, s, 0, 1, 0, -s, 0, c};
    case Z:
        return {c, -s, 0, s, c, 0, 0, 0, 1};
    default:
        VC_THROW_ERROR("Bad argument of the \"axis\": %d", static_cast<int>(axis));
        return cv::Matx<_Tp, 3, 3>::eye();
    }
}
