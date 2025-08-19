/**
 * @file contour_wrapper.hpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief 轮廓分析器头文件
 * @version 1.0
 * @date 2025-4-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once
#include <opencv2/core.hpp>

#include <bitset>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include "vc/core/property_wrapper.hpp"

/**
 * @brief 可以用于ContourWrapper的基本算术类型 int 、float 和 double
 */
template <typename T>
concept ContourWrapperBaseType =
    std::is_same_v<std::remove_cv_t<T>, int> ||
    std::is_same_v<std::remove_cv_t<T>, float> ||
    std::is_same_v<std::remove_cv_t<T>, double>;

/**
 * @class ContourWrapper
 * @brief 高性能轮廓分析器，实现计算结果的延迟加载和智能缓存
 *
 * 1. 基于写时复制(copy-on-write)和延迟加载(lazy initialization)优化内存使用
 * 2. 采用抗锯齿预处理提升几何特征计算精度
 * 3. 自动缓存计算结果，避免重复运算
 * 4. 支持线程安全：const方法可并发调用，非const方法需外部同步
 *
 * @note 构造时会自动进行轮廓抗锯齿处理(approxPolyDP)，后续所有计算基于处理后的轮廓
 */
template <ContourWrapperBaseType _Tp = int>
class ContourWrapper
{
public:
    using ValueType = _Tp;
    /**
     * @brief 关键点的精度
     *
     * @note 当轮廓点集精度为int时，将关键点精度设置为float，其余情况下和轮廓点类型相同
     */
    using KeyType = std::conditional_t<std::is_same_v<std::remove_cv_t<ValueType>, int>, float, ValueType>;
    friend class std::allocator<ContourWrapper<ValueType>>;

private:
    using PointType = cv::Point_<ValueType>;  //!< 点类型
    using KeyPointType = cv::Point_<KeyType>; //!< 关键点类型

    // 标志位缓存
    mutable std::bitset<16> cache_flags;

    //----------------【直接存储】---------------------
    std::vector<PointType> __points;
    mutable double __area = 0;
    mutable double __perimeter_close = 0;
    mutable double __perimeter_open = 0;
    mutable KeyPointType __center = KeyPointType(0, 0);
    mutable double __convex_area = 0; //!< 凸包面积

    //----------------【缓存存储】---------------------
    mutable std::unique_ptr<cv::Rect> __bounding_rect;
    mutable std::unique_ptr<cv::RotatedRect> __min_area_rect;
    mutable std::unique_ptr<std::tuple<cv::Point2f, float>> __fitted_circle;
    mutable std::unique_ptr<cv::RotatedRect> __fitted_ellipse;
    mutable std::unique_ptr<std::vector<PointType>> __convex_hull;
    mutable std::unique_ptr<std::vector<int>> __convex_hull_idx;
    // 标志位定义
    enum CacheFlags
    {
        AREA_CALC = 0,             //!< 面积计算
        PERIMETER_CLOSE_CALC = 1,  //!< 周长计算
        PERIMETER_OPEN_CALC = 2,   //!< 开放式周长计算
        CENTER_CALC = 3,           //!< 中心点计算
        BOUNDING_RECT_CALC = 4,    //!< 外接矩形计算
        MIN_AREA_RECT_CALC = 5,    //!< 最小外接矩形计算
        FITTED_CIRCLE_CALC = 6,    //!< 拟合圆计算
        FITTED_ELLIPSE_CALC = 7,   //!< 拟合椭圆计算
        SMOOTHED_CONTOUR_CALC = 8, //!< 平滑轮廓计算
        CONVEX_HULL_CALC = 9,      //!< 凸包轮廓计算
        CONVEX_HULL_IDX_CALC = 10, //!< 凸包索引计算
        CONVEX_HULL_AREA_CALC = 11 //!< 凸包面积计算
    };

public:
    /**
     * @brief 构造函数
     *
     * @param contour 轮廓点集
     */
    ContourWrapper(const std::vector<PointType> &contour)
        : __points(contour)
    {
        // 初始化标志位
        cache_flags.reset();
    }
    /**
     * @brief 移动构造函数
     *
     * @param contour 轮廓点集
     */
    ContourWrapper(std::vector<PointType> &&contour)
        : __points(std::move(contour))
    {
        // 初始化标志位
        cache_flags.reset();
    }

public:
    // 禁用拷贝构造函数
    ContourWrapper(const ContourWrapper &) = delete;
    ContourWrapper &operator=(const ContourWrapper &) = delete;
    // 允许移动
    ContourWrapper(ContourWrapper &&) = default;
    ContourWrapper &operator=(ContourWrapper &&) = default;

    /**
     * @brief 创建轮廓对象
     *
     * @param contour 轮廓点集
     */
    inline static std::shared_ptr<ContourWrapper<ValueType>> make_contour(const std::vector<PointType> &contour)
    {
        return std::make_shared<ContourWrapper<ValueType>>(contour);
    }
    /**
     * @brief 创建轮廓对象
     *
     * @param contour 轮廓点集
     */
    inline static std::shared_ptr<ContourWrapper<ValueType>> make_contour(std::vector<PointType> &&contour)
    {
        return std::make_shared<ContourWrapper<ValueType>>(std::move(contour));
    }

    /**
     * @brief 获取轮廓点集
     */
    const std::vector<PointType> &points() const
    {
        return __points;
    }

    /**
     * @brief 获取轮廓面积
     */
    double area() const
    {
        if (!cache_flags.test(AREA_CALC))
        {
            __area = std::abs(cv::contourArea(__points));
            cache_flags.set(AREA_CALC);
        }
        return __area;
    }

    /**
     * @brief 获取轮廓周长
     */
    double perimeter(bool close = true) const
    {
        if (close)
        {
            if (!cache_flags.test(PERIMETER_CLOSE_CALC))
            {
                // 自动应用平滑处理
                // const auto &smooth_contour = smoothedContour();
                const auto &contour = this->points();
                __perimeter_close = cv::arcLength(contour, true);
                cache_flags.set(PERIMETER_CLOSE_CALC);
            }
            return __perimeter_close;
        }
        else
        {
            if (!cache_flags.test(PERIMETER_OPEN_CALC))
            {
                // 自动应用平滑处理
                // const auto &smooth_contour = smoothedContour();
                const auto &contour = this->points();
                __perimeter_open = cv::arcLength(contour, false);
                cache_flags.set(PERIMETER_OPEN_CALC);
            }
            return __perimeter_open;
        }
    }

    /**
     * @brief 获取轮廓中心点
     */
    KeyPointType center() const
    {
        if (!cache_flags.test(CENTER_CALC))
        {
            // 自动应用平滑处理
            // const auto &smooth_contour = smoothedContour();
            const auto &contour = this->points();
            cv::Scalar mean_val = cv::mean(contour);
            __center = KeyPointType(mean_val[0], mean_val[1]);
            cache_flags.set(CENTER_CALC);
        }
        return __center;
    }

    /**
     * @brief 获取轮廓的正外接矩形
     */
    cv::Rect boundingRect() const
    {
        if (!cache_flags.test(BOUNDING_RECT_CALC))
        {
            // const auto &smooth_contour = smoothedContour();
            const auto &contour = this->points();
            __bounding_rect = std::make_unique<cv::Rect>(
                cv::boundingRect(contour));
            cache_flags.set(BOUNDING_RECT_CALC);
        }
        return *__bounding_rect;
    }

    /**
     * @brief 获取轮廓的最小外接矩形
     */
    cv::RotatedRect minAreaRect() const
    {
        if (!cache_flags.test(MIN_AREA_RECT_CALC))
        {
            // const auto &smooth_contour = smoothedContour();
            const auto &contour = this->points();
            __min_area_rect = std::make_unique<cv::RotatedRect>(
                cv::minAreaRect(contour));
            cache_flags.set(MIN_AREA_RECT_CALC);
        }
        return *__min_area_rect;
    }

    /**
     * @brief 获取轮廓的拟合圆
     */
    std::tuple<cv::Point2f, float> fittedCircle() const
    {
        if (!cache_flags.test(FITTED_CIRCLE_CALC))
        {
            // const auto &smooth_contour = smoothedContour();
            const auto &contour = this->points();
            if (contour.size() >= 3)
            {
                cv::Point2f center_temp;
                float radius_temp = 0;
                cv::minEnclosingCircle(contour, center_temp, radius_temp);
                __fitted_circle = std::make_unique<std::tuple<cv::Point2f, float>>(center_temp, radius_temp);
                cache_flags.set(FITTED_CIRCLE_CALC);
            }
            else
            {
                // 点数不足时触发异常
                VC_THROW_ERROR("Insufficient points for circle fitting");
            }
        }
        return *__fitted_circle;
    }

    /**
     * @brief 获取轮廓的拟合椭圆
     */
    cv::RotatedRect fittedEllipse() const
    {
        if (!cache_flags.test(FITTED_ELLIPSE_CALC))
        {
            // const auto &smooth_contour = smoothedContour();
            const auto &contour = this->points();
            if (contour.size() >= 5)
            {
                __fitted_ellipse = std::make_unique<cv::RotatedRect>(cv::fitEllipse(contour));
                cache_flags.set(FITTED_ELLIPSE_CALC);
            }
            else
            {
                // 点数不足时触发异常
                VC_THROW_ERROR("Insufficient points for ellipse fitting");
            }
        }
        return *__fitted_ellipse;
    }

    /**
     * @brief 获取轮廓的凸包
     */
    const std::vector<PointType> &convexHull() const
    {
        if (!cache_flags.test(CONVEX_HULL_CALC))
        {
            // const auto &smooth_contour = smoothedContour();
            const auto &contour = this->points();
            __convex_hull = std::make_unique<std::vector<PointType>>();
            cv::convexHull(contour, *__convex_hull);
            cache_flags.set(CONVEX_HULL_CALC);
        }
        return *__convex_hull;
    }

    /**
     * @brief 获取凸包的索引
     */
    const std::vector<int> &convexHullIdx() const
    {
        if (!cache_flags.test(CONVEX_HULL_IDX_CALC))
        {
            // const auto &smooth_contour = smoothedContour();
            const auto &contour = this->points();
            __convex_hull_idx = std::make_unique<std::vector<int>>();
            cv::convexHull(contour, *__convex_hull_idx);
            cache_flags.set(CONVEX_HULL_IDX_CALC);
        }
        return *__convex_hull_idx;
    }

    /**
     * @brief 获取凸包的面积
     */
    float convexArea() const
    {
        if (!cache_flags.test(CONVEX_HULL_AREA_CALC))
        {
            const auto &convex_contour = this->convexHull();
            __convex_area = std::abs(cv::contourArea(convex_contour));
            cache_flags.set(CONVEX_HULL_AREA_CALC);
        }
        return __convex_area;
    }

    /**
     * @brief 生成信息字符串
     */
    std::string infoString() const
    {
        std::ostringstream oss;
        oss << "  Area: " << area() << "\n";
        oss << "  Perimeter (Closed): " << perimeter(true) << "\n";
        oss << "  Center: (" << center().x << ", " << center().y << ")\n";
        return oss.str();
    }

    /**
     * @brief 隐式转换
     */
    operator const std::vector<PointType> &() const
    {
        return __points;
    }

    /**
     * @brief 获取轮廓的凸包轮廓——接口
     */
    static std::shared_ptr<ContourWrapper<ValueType>> getConvexHull(const std::vector<std::shared_ptr<const ContourWrapper<ValueType>>> &contours)
    {
        // 检查所有轮廓是否有效
        for (const auto &contour : contours)
        {
            if (contour == nullptr || contour->points().empty())
            {
                VC_THROW_ERROR("Invalid contour");
            }
        }
        return getConvexHullImpl(contours);
    }


    // /**
    //  * @brief 加入调试信息码
    //  */
    // void addMsgCode(const MsgCode_ptr &msg_code)
    // {
    //     // 若调试信息码为空，则不进行处理
    //     if (msg_code == nullptr)
    //     {
    //         return;
    //     }
    //     // 若调试信息码节点为空，则创建一个新的节点
    //     if(isSetMsgCodeNodePtr() == false)
    //     {
    //         setMsgCodeNodePtr(MsgCodeNode::make_node());
    //     }
    //     // 将调试信息码添加到节点中
    //     getMsgCodeNodePtr()->addMsgCode(msg_code);
    // }

    // /**
    //  * @brief 获取调试信息节点
    //  */
    // MsgCodeNode_ptr getMsgCodeNode()
    // {
    //     if (isSetMsgCodeNodePtr() == false)
    //     {
    //         setMsgCodeNodePtr(MsgCodeNode::make_node());
    //     }
    //     return getMsgCodeNodePtr();
    // }

public:
    //--------------------【迭代器访问----------------------】
    using const_iterator = typename std::vector<PointType>::const_iterator;                 // 常量迭代器
    using const_reverse_iterator = typename std::vector<PointType>::const_reverse_iterator; // 常量反向迭代器

    const_iterator begin() const noexcept { return __points.begin(); }
    const_iterator end() const noexcept { return __points.end(); }
    const_iterator cbegin() const noexcept { return __points.cbegin(); }
    const_iterator cend() const noexcept { return __points.cend(); }
    const_reverse_iterator rbegin() const noexcept { return __points.rbegin(); }
    const_reverse_iterator rend() const noexcept { return __points.rend(); }
    const_reverse_iterator crbegin() const noexcept { return __points.crbegin(); }
    const_reverse_iterator crend() const noexcept { return __points.crend(); }

    // 常用stl接口
    size_t size() const noexcept { return __points.size(); }
    bool empty() const noexcept { return __points.empty(); }
    const PointType &operator[](size_t idx) const noexcept { return __points[idx]; }
    const PointType &at(size_t idx) const noexcept
    {
        if (idx >= __points.size())
        {
            VC_THROW_ERROR("Index out of range");
        }
        return __points.at(idx);
    }
    const PointType &front() const noexcept { return __points.front(); }
    const PointType &back() const noexcept { return __points.back(); }

private:
    //     /**
    //      * @brief 获取平滑轮廓
    //      */
    //     const std::vector<PointType> &smoothedContour() const
    //     {
    // #if ENABLE_SMOOTH_CONTOUR_CALC
    //         if (!cache_flags.test(SMOOTH_CONTOUR_CALC))
    //         {
    //             __smoothed_contour = std::make_unique<std::vector<cv::Point>>();

    //             // 抗锯齿处理流程
    //             if (__points.size() >= 4)
    //             {
    //                 const double epsilon = 0.01 * cv::arcLength(__points, true);
    //                 cv::approxPolyDP(__points, *__smoothed_contour, epsilon, true);
    //             }
    //             else
    //             {
    //                 *__smoothed_contour = __points;
    //             }

    //             cache_flags.set(SMOOTH_CONTOUR_CALC);
    //         }
    //         return *__smoothed_contour;
    // #else
    //         return __points;
    // #endif
    //     }

    /**
     * @brief 获取多轮廓的凸包轮廓
     */
    static std::shared_ptr<ContourWrapper<ValueType>> getConvexHullImpl(const std::vector<std::shared_ptr<const ContourWrapper<ValueType>>> &contours)
    {
        size_t total_point_size = 0;
        for (const auto &contour : contours)
            total_point_size += contour->points().size();
        std::vector<PointType> all_points;
        all_points.reserve(total_point_size);
        for (const auto &contour : contours)
        {
            const auto &points = contour->points();
            all_points.insert(all_points.end(), points.begin(), points.end());
        }
        std::vector<PointType> convex_hull;
        cv::convexHull(all_points, convex_hull);
        return ContourWrapper<ValueType>::make_contour(std::move(convex_hull));
    }
};

using Contour_ptr = std::shared_ptr<ContourWrapper<int>>;
using ContourF_ptr = std::shared_ptr<ContourWrapper<float>>;
using ContourD_ptr = std::shared_ptr<ContourWrapper<double>>;

using Contour_cptr = std::shared_ptr<const ContourWrapper<int>>;
using ContourF_cptr = std::shared_ptr<const ContourWrapper<float>>;
using ContourD_cptr = std::shared_ptr<const ContourWrapper<double>>;

/**
 * @brief 类型检验，是否为 ContourWrapper<T>，T符合ContourWrapperBaseType
 */
template <typename T>
struct is_contour_wrapper : std::false_type
{
};
template <ContourWrapperBaseType T>
struct is_contour_wrapper<ContourWrapper<T>> : std::true_type
{
};
template <typename T>
concept ContourWrapperType = is_contour_wrapper<std::remove_cvref_t<T>>::value;

// template <typename T>
// concept ContourWrapperType = requires {
//     typename T::value_type; // SFINAE 触发点
//     requires std::is_base_of_v<
//         ContourWrapper<std::remove_cvref_t<typename T::value_type>>,
//         std::remove_cvref_t<T>>;
//     requires ContourWrapperBaseType<typename T::value_type>;
// };

/**
 * @brief 类型检验，检验是否为 std::shared_ptr<ContourWrapper<T>>，T符合ContourWrapperBaseType
 */
template <typename T>
concept ContourWrapperPtrType = requires {
    requires std::is_same_v<
        std::shared_ptr<typename std::remove_cvref_t<T>::element_type>,
        std::remove_cvref_t<T>>;
    requires ContourWrapperType<typename std::remove_cvref_t<T>::element_type>;
};

// namespace ContourWrapper
// {
//     // /**
//     //  * @brief 精度转换
//     //  */
//     // template <ContourWrapperBaseType OutPutType, ContourWrapperBaseType InputType>
//     // static std::shared_ptr<ContourWrapper<OutPutType>> getConvert(const std::shared_ptr<ContourWrapper<InputType>> &contour)
//     // {
//     //     // 若类型相同，则直接返回
//     //     if constexpr (std::is_same_v<OutPutType, ValueType>)
//     //     {
//     //         return contour;
//     //     }

//     //     using OutPutPointType = cv::Point_<OutPutType>;
//     //     std::vector<OutPutPointType> converted_points;
//     //     converted_points.reserve(contour->points().size());
//     //     for (const auto &point : contour->points())
//     //     {
//     //         converted_points.emplace_back(static_cast<OutPutType>(point.x), static_cast<OutPutType>(point.y));
//     //     }
//     //     return std::make_shared<ContourWrapper<OutPutType>>(std::move(converted_points));
//     // }

// /**
//  * @brief 精度转换
//  */
// template <ContourWrapperBaseType OutPutType, ContourWrapperBaseType InputType>
// inline std::shared_ptr<ContourWrapper<OutPutType>> getConverted(const std::shared_ptr<ContourWrapper<InputType>> &contour)
// {
//     // 若类型相同，则直接返回
//     if constexpr (std::is_same_v<OutPutType, InputType>)
//     {
//         return contour;
//     }

//     using OutPutPointType = cv::Point_<OutPutType>;
//     std::vector<OutPutPointType> converted_points;
//     converted_points.reserve(contour->points().size());
//     for (const auto &point : contour->points())
//     {
//         converted_points.emplace_back(static_cast<OutPutType>(point.x), static_cast<OutPutType>(point.y));
//     }
//     return std::make_shared<ContourWrapper<OutPutType>>(std::move(converted_points));

// }

/**
 * @brief 轮廓精度转换函数（支持 int/float/double 互转）
 * @tparam OutputType 目标精度类型 (int/float/double)
 * @tparam ContourPtr  输入轮廓指针类型 (自动推导)
 *
 * @param contour 输入轮廓的智能指针
 * @return std::shared_ptr<ContourWrapper<OutputType>> 转换后的轮廓智能指针
 *
 * @note 类型相同时直接返回原指针（无额外开销）
 *       类型不同时生成新轮廓（触发写时复制）
 */
template <ContourWrapperBaseType OutputType, ContourWrapperPtrType ContourPtr>
inline auto convert(const ContourPtr &contour) -> std::shared_ptr<ContourWrapper<OutputType>>
{
    using InputType = typename ContourPtr::element_type::value_type;

    if constexpr (std::is_same_v<OutputType, InputType>)
    {
        // 类型相同直接返回原始指针
        return std::static_pointer_cast<ContourWrapper<OutputType>>(contour);
    }
    else
    {
        // 类型转换生成新轮廓
        using OutputPoint = cv::Point_<OutputType>;
        std::vector<OutputPoint> converted;
        converted.reserve(contour->points().size());

        // 坐标转换（使用完美转发避免拷贝）
        for (const auto &p : contour->points())
        {
            converted.emplace_back(
                static_cast<OutputType>(p.x),
                static_cast<OutputType>(p.y));
        }

        // 构造新轮廓（触发移动语义）
        return ContourWrapper<OutputType>::make_contour(std::move(converted));
    }

    return nullptr;
}

#include "extensions.hpp"
#include "hierarchy.hpp"