#pragma once

#include <opencv2/core.hpp>
#include <cmath>
#include <utility>
#include <limits>
#include <stdexcept>

/**
 * @brief 平面直线类
 */
template <typename _Tp = double>
class Line2_
{
public:
    using value_type = _Tp; //!< 直线的数值类型
    using point_type = cv::Point_<value_type>; //!< 直线上的点类型

    /**
     * @brief 默认构造函数，创建一条水平直线
     * @note 默认直线方程为 y = 0
     */
    Line2_();

    /**
     * @brief 构造函数，使用角度和点创建直线
     * 
     * @param[in] angle 直线的角度（弧度）
     * @param[in] point 直线上的一个点
     * @note 角度为0时，直线方向为水平方向，角度增大时，直线逆时针旋转
     */
    Line2_(value_type angle, const point_type &point);

    /**
     * @brief 构造函数，使用两个点创建直线
     * 
     * @param[in] p1 直线上的第一个点
     * @param[in] p2 直线上的第二个点
     * @note 如果两个点相同，将抛出异常
     */
    Line2_(const point_type &p1, const point_type &p2);

    /**
     * @brief 拷贝构造函数
     */
    Line2_(const Line2_ &other);

    /**
     * @brief 移动构造函数
     */
    Line2_(Line2_ &&other) noexcept;

    /**
     * @brief 赋值运算符重载
     */
    Line2_ &operator=(const Line2_ &other);

    /**
     * @brief 移动赋值运算符重载
     */
    Line2_ &operator=(Line2_ &&other) noexcept;

    /**
     * @brief 获取直线的角度（弧度）
     */
    value_type angle() const;

    /**
     * @brief 获取直线的角度（角度）
     */
    value_type angleDegrees() const;

    /**
     * @brief 获取直线上的顶点
     */
    point_type point() const;

    /**
     * @brief 获取直线的方向向量
     */
    point_type direction() const;

    /**
     * @brief 获取直线的法向量
     */
    point_type normal() const;

    /**
     * @brief 计算点到直线的距离
     */
    value_type distanceTo(const point_type &p) const;

    /**
     * @brief 检查点是否在直线上
     * 
     * @param p 要检查的点
     * @param tolerance 容差值，默认值为1e-5
     * @return true 如果点在直线上（在容差范围内）
     */
    bool contains(const point_type &p, value_type tolerance = 1e-5) const;

    /**
     * @brief 计算两条直线的交点
     * 
     * @param other 另一条直线
     * @return std::vector<point_type> 交点的集合
     */
    std::vector<point_type> intersect(const Line2_ &other) const;


    /**
     * @brief 计算点在直线上的投影点
     * 
     * @param p 要投影的点
     * @return point_type 投影点
     */
    point_type project(const point_type &p) const;

    

    /**
     * @brief 获取直线的一般方程
     * 
     * @param A 一般方程的系数 A
     * @param B 一般方程的系数 B
     * @param C 一般方程的系数 C
     */
    void getGeneralEquation(value_type &A, value_type &B, value_type &C) const;

private:

    /**
     * @brief 规范化角度，使其在 [0, 2π) 范围内
     */
    void normalizeAngle();

    /**
     * @brief 根据参数 t 计算直线上对应的点
     */
    point_type pointAt(value_type t) const;

    value_type __angle;
    point_type __point;
    point_type __direction;
};

// 类外成员函数实现
template <typename _Tp>
Line2_<_Tp>::Line2_()
    : __angle(0), __point(0, 0), __direction(1, 0) {}

template <typename _Tp>
Line2_<_Tp>::Line2_(value_type angle, const point_type &point)
    : __angle(angle), __point(point),
      __direction(std::cos(angle), std::sin(angle))
{
    normalizeAngle();
}

template <typename _Tp>
Line2_<_Tp>::Line2_(const point_type &p1, const point_type &p2)
    : __point(p1)
{
    if (p1 == p2)
        throw std::invalid_argument("直线的两个点不能相同");

    cv::Point2d dir = p2 - p1;
    double len = cv::norm(dir);
    __direction = dir / len;
    __angle = std::atan2(__direction.y, __direction.x);

    normalizeAngle();
}

template <typename _Tp>
Line2_<_Tp>::Line2_(const Line2_ &other)
    : __angle(other.__angle), __point(other.__point), __direction(other.__direction) {}

template <typename _Tp>
Line2_<_Tp>::Line2_(Line2_ &&other) noexcept
    : __angle(std::move(other.__angle)), __point(std::move(other.__point)),
      __direction(std::move(other.__direction)) {}

template <typename _Tp>
Line2_<_Tp> &Line2_<_Tp>::operator=(const Line2_ &other)
{
    if (this != &other)
    {
        __angle = other.__angle;
        __point = other.__point;
        __direction = other.__direction;
    }
    return *this;
}

template <typename _Tp>
Line2_<_Tp> &Line2_<_Tp>::operator=(Line2_ &&other) noexcept
{
    if (this != &other)
    {
        __angle = std::move(other.__angle);
        __point = std::move(other.__point);
        __direction = std::move(other.__direction);
    }
    return *this;
}

template <typename _Tp>
void Line2_<_Tp>::normalizeAngle()
{
    constexpr value_type twoPi = 2 * static_cast<value_type>(CV_PI);
    __angle = std::fmod(__angle, twoPi);
    if (__angle < 0)
        __angle += twoPi;
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::pointAt(value_type t) const
{
    return __point + t * __direction;
}

template <typename _Tp>
typename Line2_<_Tp>::value_type Line2_<_Tp>::angle() const
{
    return __angle;
}

template <typename _Tp>
typename Line2_<_Tp>::value_type Line2_<_Tp>::angleDegrees() const
{
    return __angle * 180.0 / CV_PI;
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::point() const
{
    return __point;
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::direction() const
{
    return __direction;
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::normal() const
{
    return point_type(-__direction.y, __direction.x);
}

template <typename _Tp>
typename Line2_<_Tp>::value_type Line2_<_Tp>::distanceTo(const point_type &p) const
{
    cv::Point2d diff = p - __point;
    return std::abs(diff.dot(normal()));
}

template <typename _Tp>
bool Line2_<_Tp>::contains(const point_type &p, value_type tolerance) const
{
    return std::abs(distanceTo(p)) < tolerance;
}

template <typename _Tp>
std::vector<typename Line2_<_Tp>::point_type> Line2_<_Tp>::intersect(const Line2_ &other) const
{
    const double tolerance = std::numeric_limits<value_type>::epsilon();
    const double det = __direction.x * other.__direction.y -
                       __direction.y * other.__direction.x;

    if (std::abs(det) < tolerance)
        return {};

    cv::Point2d diff = other.__point - __point;
    const double t = (diff.x * other.__direction.y - diff.y * other.__direction.x) / det;
    return {pointAt(t)};
}

template <typename _Tp>
typename Line2_<_Tp>::point_type Line2_<_Tp>::project(const point_type &p) const
{
    // 计算点到直线上的向量
    cv::Point2d vec = p - __point;
    value_type t = vec.dot(__direction);
    return __point + t * __direction;
}

template <typename _Tp>
void Line2_<_Tp>::getGeneralEquation(value_type &A, value_type &B, value_type &C) const
{
    A = -__direction.y;
    B = __direction.x;
    C = __direction.y * __point.x - __direction.x * __point.y;
}

using Line2i = Line2_<int>; //!< 整型直线类
using Line2f = Line2_<float>; //!< 单精度浮点数直线类
using Line2d = Line2_<double>; //!< 双精度浮点数直线类

using Line = Line2f;    //!< 默认直线类型，使用单精度浮点数