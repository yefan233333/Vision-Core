#pragma once

#include <opencv2/core.hpp>
#include <cmath>
#include <utility>
#include <limits>
#include <stdexcept>

#include "geom_line.hpp"

/**
 * @brief 二维圆类
 *
 * 该类用于表示和操作二维圆的几何属性。
 */
template <typename _Tp = double>
class Circle2_
{
public:
    using value_type = _Tp;                    //!< 圆的数值类型
    using point_type = cv::Point_<value_type>; //!< 圆心点类型

    /**
     * @brief 默认构造函数，创建单位圆
     * @note 圆心位于原点 (0,0)，半径为 1.0
     */
    Circle2_();

    /**
     * @brief 构造函数，通过圆心和半径创建圆
     *
     * @param[in] center 圆心坐标
     * @param[in] radius 圆的半径
     * @note 半径值必须大于0，否则抛出异常
     */
    Circle2_(const point_type &center, value_type radius);

    /**
     * @brief 构造函数，通过三个点创建圆
     *
     * @param[in] p1 圆上的第一个点
     * @param[in] p2 圆上的第二个点
     * @param[in] p3 圆上的第三个点
     * @throws std::invalid_argument 如果点共线或无法确定唯一的圆
     */
    Circle2_(const point_type &p1, const point_type &p2, const point_type &p3);

    /**
     * @brief 拷贝构造函数
     */
    Circle2_(const Circle2_ &other);

    /**
     * @brief 移动构造函数
     */
    Circle2_(Circle2_ &&other) noexcept;

    /**
     * @brief 赋值运算符重载
     */
    Circle2_ &operator=(const Circle2_ &other);

    /**
     * @brief 移动赋值运算符重载
     */
    Circle2_ &operator=(Circle2_ &&other) noexcept;

    /**
     * @brief 获取圆心坐标
     */
    point_type center() const;

    /**
     * @brief 获取圆的半径
     */
    value_type radius() const;

    /**
     * @brief 计算圆的周长
     */
    value_type circumference() const;

    /**
     * @brief 计算圆的面积
     */
    value_type area() const;

    /**
     * @brief 计算点到圆的距离
     *
     * @param p 外部点
     * @return 点到圆周的最短距离（正为外点，负为内点）
     */
    value_type distanceTo(const point_type &p) const;

    /**
     * @brief 检查点是否在圆上或圆内
     *
     * @param p 要检查的点
     * @param tolerance 容差值，默认值为1e-5
     * @return true 如果点在圆上或圆内（在容差范围内）
     */
    bool contains(const point_type &p, value_type tolerance = 1e-5) const;

    /**
     * @brief 检查点是否在圆周上
     *
     * @param p 要检查的点
     * @param tolerance 容差值，默认值为1e-5
     * @return true 如果点在圆周上（在容差范围内）
     */
    bool onCircumference(const point_type &p, value_type tolerance = 1e-5) const;

    /**
     * @brief 计算圆与直线的交点
     *
     * @param line 要检查的直线
     * @return std::vector<point_type> 交点的集合（0, 1或2个点）
     */
    std::vector<point_type> intersect(const Line2_<value_type> &line) const;

    /**
     * @brief 计算两个圆的交点
     *
     * @param other 另一个圆
     * @return std::vector<point_type> 交点的集合（0, 1或2个点）
     */
    std::vector<point_type> intersect(const Circle2_ &other) const;

private:
    point_type __center; //!< 圆心坐标
    value_type __radius; //!< 圆的半径
};

// 类外成员函数实现
template <typename _Tp>
Circle2_<_Tp>::Circle2_()
    : __center(0, 0), __radius(1) {}

template <typename _Tp>
Circle2_<_Tp>::Circle2_(const point_type &center, value_type radius)
    : __center(center), __radius(radius)
{
    if (radius <= 0)
        throw std::invalid_argument("圆的半径必须大于0");
}

template <typename _Tp>
Circle2_<_Tp>::Circle2_(const point_type &p1, const point_type &p2, const point_type &p3)
{
    // 检查点是否互异
    if (p1 == p2 || p1 == p3 || p2 == p3)
    {
        throw std::invalid_argument("创建圆需要三个不同的点");
    }

    // 避免点共线的情况
    cv::Point3d v1(p2.x - p1.x, p2.y - p1.y, 0);
    cv::Point3d v2(p3.x - p1.x, p3.y - p1.y, 0);
    if (std::abs(v1.cross(v2).z) < std::numeric_limits<value_type>::epsilon())
    {
        throw std::invalid_argument("三点共线，无法确定圆");
    }

    // 计算三点形成圆的圆心和半径
    value_type A = p2.x - p1.x;
    value_type B = p2.y - p1.y;
    value_type C = p3.x - p1.x;
    value_type D = p3.y - p1.y;

    value_type E = A * (p1.x + p2.x) + B * (p1.y + p2.y);
    value_type F = C * (p1.x + p3.x) + D * (p1.y + p3.y);

    value_type G = 2 * (A * (p3.y - p2.y) - B * (p3.x - p2.x));

    if (std::abs(G) < std::numeric_limits<value_type>::epsilon())
    {
        throw std::invalid_argument("三点共线，无法确定圆");
    }

    // 计算圆心坐标
    value_type center_x = (D * E - B * F) / G;
    value_type center_y = (A * F - C * E) / G;
    __center = point_type(center_x, center_y);

    // 计算半径
    __radius = std::sqrt(
        (p1.x - center_x) * (p1.x - center_x) +
        (p1.y - center_y) * (p1.y - center_y));
}

template <typename _Tp>
Circle2_<_Tp>::Circle2_(const Circle2_ &other)
    : __center(other.__center), __radius(other.__radius) {}

template <typename _Tp>
Circle2_<_Tp>::Circle2_(Circle2_ &&other) noexcept
    : __center(std::move(other.__center)), __radius(std::move(other.__radius)) {}

template <typename _Tp>
Circle2_<_Tp> &Circle2_<_Tp>::operator=(const Circle2_ &other)
{
    if (this != &other)
    {
        __center = other.__center;
        __radius = other.__radius;
    }
    return *this;
}

template <typename _Tp>
Circle2_<_Tp> &Circle2_<_Tp>::operator=(Circle2_ &&other) noexcept
{
    if (this != &other)
    {
        __center = std::move(other.__center);
        __radius = std::move(other.__radius);
    }
    return *this;
}

template <typename _Tp>
typename Circle2_<_Tp>::point_type Circle2_<_Tp>::center() const
{
    return __center;
}

template <typename _Tp>
typename Circle2_<_Tp>::value_type Circle2_<_Tp>::radius() const
{
    return __radius;
}

template <typename _Tp>
typename Circle2_<_Tp>::value_type Circle2_<_Tp>::circumference() const
{
    return 2 * static_cast<value_type>(CV_PI) * __radius;
}

template <typename _Tp>
typename Circle2_<_Tp>::value_type Circle2_<_Tp>::area() const
{
    return static_cast<value_type>(CV_PI) * __radius * __radius;
}

template <typename _Tp>
typename Circle2_<_Tp>::value_type Circle2_<_Tp>::distanceTo(const point_type &p) const
{
    value_type d = std::sqrt(
        (p.x - __center.x) * (p.x - __center.x) +
        (p.y - __center.y) * (p.y - __center.y));
    return d - __radius;
}

template <typename _Tp>
bool Circle2_<_Tp>::contains(const point_type &p, value_type tolerance) const
{
    value_type d_sq =
        (p.x - __center.x) * (p.x - __center.x) +
        (p.y - __center.y) * (p.y - __center.y);
    return d_sq <= (__radius + tolerance) * (__radius + tolerance);
}

template <typename _Tp>
bool Circle2_<_Tp>::onCircumference(const point_type &p, value_type tolerance) const
{
    value_type d_sq =
        (p.x - __center.x) * (p.x - __center.x) +
        (p.y - __center.y) * (p.y - __center.y);
    value_type r_sq = __radius * __radius;
    return std::abs(d_sq - r_sq) < tolerance * (2 * __radius + tolerance);
}

template <typename _Tp>
std::vector<typename Circle2_<_Tp>::point_type>
Circle2_<_Tp>::intersect(const Line2_<value_type> &line) const
{
    // 计算圆心到直线的垂直距离
    value_type dist = line.distanceTo(__center);

    std::vector<point_type> intersections;
    const value_type tolerance = std::numeric_limits<value_type>::epsilon();

    // 根据距离和半径的关系判断交点情况
    if (dist > __radius + tolerance)
    {
        // 无交点
        return intersections;
    }

    if (std::abs(dist - __radius) < tolerance)
    {
        // 一个交点（相切）
        // 找到垂足位置
        point_type foot = line.project(__center);
        intersections.push_back(foot);
        return intersections;
    }

    // 两个交点情况
    // 直线方向向量和法向量
    auto dir = line.direction();
    auto norm = cv::Point_<value_type>(-dir.y, dir.x);

    // 计算圆心到直线垂足的距离平方
    value_type h_sq = __radius * __radius - dist * dist;

    if (h_sq < 0)
        h_sq = 0; // 避免浮点误差
    value_type h = std::sqrt(h_sq);

    // 垂足位置
    point_type foot = line.project(__center);

    // 两个交点位置
    intersections.push_back(foot + h * dir);
    intersections.push_back(foot - h * dir);

    return intersections;
}

template <typename _Tp>
std::vector<typename Circle2_<_Tp>::point_type>
Circle2_<_Tp>::intersect(const Circle2_ &other) const
{
    const value_type tolerance = std::numeric_limits<value_type>::epsilon();
    std::vector<point_type> intersections;

    // 计算两个圆心之间的距离
    value_type d_x = other.__center.x - __center.x;
    value_type d_y = other.__center.y - __center.y;
    value_type d_sq = d_x * d_x + d_y * d_y;
    value_type d = std::sqrt(d_sq);

    // 处理同心圆情况
    if (d < tolerance)
    {
        if (std::abs(__radius - other.__radius) < tolerance)
        {
            // 完全相同圆 - 返回空表示无限交点
        }
        else
        {
            // 同心不同半径 - 无交点
        }
        return intersections;
    }

    // 检查是否无交点
    if (d > __radius + other.__radius + tolerance)
    {
        return intersections; // 相离
    }

    // 检查是否内切或内含
    if (d + tolerance < std::abs(__radius - other.__radius))
    {
        return intersections; // 内含
    }

    // 检查是否外切
    if (std::abs(d - (__radius + other.__radius)) < tolerance)
    {
        // 外切：一个交点
        point_type p = __center + (__radius / d) * (other.__center - __center);
        intersections.push_back(p);
        return intersections;
    }

    // 检查是否内切
    if (std::abs(d - std::abs(__radius - other.__radius)) < tolerance)
    {
        // 内切：一个交点
        if (__radius > other.__radius)
        {
            point_type p = other.__center + (other.__radius / d) * (__center - other.__center);
            intersections.push_back(p);
        }
        else
        {
            point_type p = __center + (__radius / d) * (other.__center - __center);
            intersections.push_back(p);
        }
        return intersections;
    }

    // 一般情况：两个交点
    value_type a = (__radius * __radius - other.__radius * other.__radius + d_sq) / (2 * d);
    value_type h_sq = __radius * __radius - a * a;

    if (h_sq < 0)
        h_sq = 0; // 避免浮点误差
    value_type h = std::sqrt(h_sq);

    // 计算交点坐标
    point_type p0(
        __center.x + a * d_x / d,
        __center.y + a * d_y / d);

    value_type rx = -d_y * h / d;
    value_type ry = d_x * h / d;

    intersections.push_back(point_type(p0.x + rx, p0.y + ry));
    intersections.push_back(point_type(p0.x - rx, p0.y - ry));

    return intersections;
}

using Circle2i = Circle2_<int>;    //!< 整型圆类
using Circle2f = Circle2_<float>;  //!< 单精度浮点数圆类
using Circle2d = Circle2_<double>; //!< 双精度浮点数圆类

using Circle = Circle2f; //!< 默认圆类型，使用单精度浮点数