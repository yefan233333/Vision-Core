#pragma once

#include <opencv2/core.hpp>
#include <cmath>
#include <utility>
#include <limits>
#include <stdexcept>

/**
 * @brief 二维线段类
 *
 * 该类用于表示二维空间中的线段，提供基本的几何操作。
 * 包括计算长度、判断点是否在线段上、点投影等功能。
 */
template <typename _Tp = double>
class Segment2_
{
public:
    using value_type = _Tp;                    //!< 线段的数值类型
    using point_type = cv::Point_<value_type>; //!< 点类型

    /**
     * @brief 默认构造函数，创建单位线段
     * @note 起点在原点 (0,0)，终点在 (1,0)
     */
    Segment2_();

    /**
     * @brief 构造函数，通过两个点创建线段
     *
     * @param[in] pt1 线段的起点
     * @param[in] pt2 线段的终点
     * @note 如果两点相同，线段长度为零
     */
    Segment2_(const point_type &pt1, const point_type &pt2);

    /**
     * @brief 拷贝构造函数
     */
    Segment2_(const Segment2_ &other);

    /**
     * @brief 移动构造函数
     */
    Segment2_(Segment2_ &&other) noexcept;

    /**
     * @brief 赋值运算符重载
     */
    Segment2_ &operator=(const Segment2_ &other);

    /**
     * @brief 移动赋值运算符重载
     */
    Segment2_ &operator=(Segment2_ &&other) noexcept;

    /**
     * @brief 获取线段的起点
     */
    point_type start() const;

    /**
     * @brief 获取线段的终点
     */
    point_type end() const;

    /**
     * @brief 获取线段的向量表示（终点减起点）
     */
    point_type vector() const;

    /**
     * @brief 计算线段长度
     */
    value_type length() const;

    /**
     * @brief 计算点在线段上的投影点
     *
     * @param[in] p 外部点
     * @return 点在线段上的投影坐标
     * @note 投影点可能在线段起点之前或终点之后
     */
    point_type project(const point_type &p) const;

    /**
     * @brief 计算点到线段的最近点
     *
     * @param[in] p 外部点
     * @return 线段上距离该点最近的点
     * @note 最近点始终在线段上
     */
    point_type closestPoint(const point_type &p) const;

    /**
     * @brief 计算点到线段的距离
     *
     * @param[in] p 外部点
     * @return 点到线段的最近距离
     */
    value_type distanceTo(const point_type &p) const;

    /**
     * @brief 检查点是否在线段上
     *
     * @param p 要检查的点
     * @param tolerance 容差值，默认值为1e-5
     * @return true 如果点在线段上（在容差范围内）
     */
    bool contains(const point_type &p, value_type tolerance = 1e-5) const;

    /**
     * @brief 判断两条线段是否相交
     *
     * @param other 另一条线段
     * @return true 如果线段相交（包括端点接触）
     */
    bool intersects(const Segment2_ &other) const;

    /**
     * @brief 计算两条线段的交点
     *
     * @param other 另一条线段
     * @return std::vector<point_type> 交点的集合（0或1个点）
     */
    std::vector<point_type> intersect(const Segment2_ &other) const;

private:
    point_type __pt1; //!< 线段起点
    point_type __pt2; //!< 线段终点
};

// 类外成员函数实现
template <typename _Tp>
Segment2_<_Tp>::Segment2_()
    : __pt1(0, 0), __pt2(1, 0) {}

template <typename _Tp>
Segment2_<_Tp>::Segment2_(const point_type &pt1, const point_type &pt2)
    : __pt1(pt1), __pt2(pt2) {}

template <typename _Tp>
Segment2_<_Tp>::Segment2_(const Segment2_ &other)
    : __pt1(other.__pt1), __pt2(other.__pt2) {}

template <typename _Tp>
Segment2_<_Tp>::Segment2_(Segment2_ &&other) noexcept
    : __pt1(std::move(other.__pt1)), __pt2(std::move(other.__pt2)) {}

template <typename _Tp>
Segment2_<_Tp> &Segment2_<_Tp>::operator=(const Segment2_ &other)
{
    if (this != &other)
    {
        __pt1 = other.__pt1;
        __pt2 = other.__pt2;
    }
    return *this;
}

template <typename _Tp>
Segment2_<_Tp> &Segment2_<_Tp>::operator=(Segment2_ &&other) noexcept
{
    if (this != &other)
    {
        __pt1 = std::move(other.__pt1);
        __pt2 = std::move(other.__pt2);
    }
    return *this;
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::start() const
{
    return __pt1;
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::end() const
{
    return __pt2;
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::vector() const
{
    return __pt2 - __pt1;
}

template <typename _Tp>
typename Segment2_<_Tp>::value_type Segment2_<_Tp>::length() const
{
    return std::sqrt(
        (__pt2.x - __pt1.x) * (__pt2.x - __pt1.x) +
        (__pt2.y - __pt1.y) * (__pt2.y - __pt1.y));
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::project(const point_type &p) const
{
    point_type vec = vector();
    point_type diff = p - __pt1;

    // 如果线段长度为零，直接返回起点
    value_type len_sq = vec.x * vec.x + vec.y * vec.y;
    if (len_sq < std::numeric_limits<value_type>::epsilon())
    {
        return __pt1;
    }

    // 计算投影比例：t = (p - pt1) · vec / |vec|²
    value_type t = diff.dot(vec) / len_sq;

    // 投影点坐标
    return __pt1 + t * vec;
}

template <typename _Tp>
typename Segment2_<_Tp>::point_type Segment2_<_Tp>::closestPoint(const point_type &p) const
{
    point_type proj = project(p);
    point_type vec = vector();

    // 计算投影比例
    value_type t;
    if (std::abs(vec.x) > std::abs(vec.y))
    {
        t = (proj.x - __pt1.x) / vec.x;
    }
    else if (vec.y != 0)
    {
        t = (proj.y - __pt1.y) / vec.y;
    }
    else
    {
        t = 0; // 线段长度为0
    }

    // 钳制 t 在 [0,1] 范围内
    if (t < 0)
        return __pt1;
    if (t > 1)
        return __pt2;
    return proj;
}

template <typename _Tp>
typename Segment2_<_Tp>::value_type Segment2_<_Tp>::distanceTo(const point_type &p) const
{
    point_type closest = closestPoint(p);
    value_type dx = p.x - closest.x;
    value_type dy = p.y - closest.y;
    return std::sqrt(dx * dx + dy * dy);
}

template <typename _Tp>
bool Segment2_<_Tp>::contains(const point_type &p, value_type tolerance) const
{
    // 先检查点是否在无限延长线上
    point_type proj = project(p);
    value_type dx = p.x - proj.x;
    value_type dy = p.y - proj.y;
    if (dx * dx + dy * dy > tolerance * tolerance)
    {
        return false;
    }

    // 再检查投影点是否在线段上
    point_type closest = closestPoint(p);
    if (closest == __pt1 || closest == __pt2)
    {
        return true;
    }

    // 检查点是否在起点和终点之间
    point_type v1 = closest - __pt1;
    point_type v2 = closest - __pt2;
    return v1.dot(v2) < tolerance;
}

template <typename _Tp>
bool Segment2_<_Tp>::intersects(const Segment2_ &other) const
{
    point_type p1 = __pt1;
    point_type q1 = __pt2;
    point_type p2 = other.__pt1;
    point_type q2 = other.__pt2;

    // 计算方向向量
    point_type r = q1 - p1;
    point_type s = q2 - p2;
    point_type pq = p2 - p1;

    // 计算叉积行列式
    value_type rxs = r.cross(s);
    value_type pqxs = pq.cross(s);
    value_type pqxr = pq.cross(r);

    const value_type tolerance = std::numeric_limits<value_type>::epsilon();

    // 处理共线情况
    if (std::abs(rxs) < tolerance)
    {
        // 平行线段
        if (std::abs(pqxr) > tolerance)
        {
            return false; // 平行但不相交
        }

        // 共线，检查重叠
        value_type t0 = pq.dot(r) / r.dot(r);
        value_type t1 = t0 + s.dot(r) / r.dot(r);

        if (s.dot(r) < 0)
        {
            std::swap(t0, t1);
        }

        return !(t1 < 0 || t0 > 1);
    }

    // 计算交点参数
    value_type t = pqxs / rxs;
    value_type u = pqxr / rxs;

    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}

template <typename _Tp>
std::vector<typename Segment2_<_Tp>::point_type>
Segment2_<_Tp>::intersect(const Segment2_ &other) const
{
    point_type p1 = __pt1;
    point_type q1 = __pt2;
    point_type p2 = other.__pt1;
    point_type q2 = other.__pt2;

    // 计算方向向量
    point_type r = q1 - p1;
    point_type s = q2 - p2;
    point_type pq = p2 - p1;

    // 计算叉积行列式
    value_type rxs = r.cross(s);
    value_type pqxs = pq.cross(s);
    value_type pqxr = pq.cross(r);

    const value_type tolerance = std::numeric_limits<value_type>::epsilon();
    std::vector<point_type> intersections;

    // 处理共线情况
    if (std::abs(rxs) < tolerance && std::abs(pqxr) < tolerance)
    {
        // 共线线段，计算重叠区域
        value_type t0 = pq.dot(r) / r.dot(r);
        value_type t1 = t0 + s.dot(r) / r.dot(r);

        value_type start_t = std::max<value_type>(0, std::min(t0, t1));
        value_type end_t = std::min<value_type>(1, std::max(t0, t1));

        if (start_t <= end_t)
        {
            // 添加重叠端点
            point_type start_pt = p1 + start_t * r;
            point_type end_pt = p1 + end_t * r;

            intersections.push_back(start_pt);
            if (std::abs(end_t - start_t) > tolerance)
            {
                intersections.push_back(end_pt);
            }
        }
        return intersections;
    }

    // 检查交点是否在线段上
    if (std::abs(rxs) > tolerance)
    {
        value_type t = pqxs / rxs;
        value_type u = pqxr / rxs;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        {
            intersections.push_back(p1 + t * r);
        }
    }

    return intersections;
}

using Segment2i = Segment2_<int>;    //!< 整型线段类
using Segment2f = Segment2_<float>;  //!< 单精度浮点数线段类
using Segment2d = Segment2_<double>; //!< 双精度浮点数线段类

using Segment = Segment2f; //!< 默认线段类型，使用单精度浮点数