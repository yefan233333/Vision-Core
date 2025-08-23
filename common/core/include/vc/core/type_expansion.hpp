#pragma once
#include <iostream>
#include <type_traits>


/**
 * @class Range
 */
template <typename T>
requires std::is_arithmetic_v<T> // 确保 T 是一个算术类型
class ArithmeticRange
{
public:
    T low;
    T high;

    // 默认构造函数
    ArithmeticRange() : low(), high() {}

    // 带参数的构造函数
    ArithmeticRange(T low, T high) : low(low), high(high)
    {
        if (low > high)
        {
            std::swap(this->low, this->high); // 保证 low <= high
        }
    }

    // 获取区间长度
    T length() const
    {
        return high - low;
    }

    // 判断一个值是否在区间内
    bool contains(T value) const
    {
        return value >= low && value <= high;
    }

    // 判断当前区间与另一个区间是否有交集
    bool intersects(const ArithmeticRange<T> &other) const
    {
        return !(other.high < low || other.low > high);
    }

    // 打印区间信息
    void print() const
    {
        std::cout << "[" << low << ", " << high << "]" << std::endl;
    }

    // 运算符重载：比较两个区间是否相等
    bool operator==(const ArithmeticRange<T> &other) const
    {
        return low == other.low && high == other.high;
    }

    // 运算符重载：获取区间的宽度（高 - 低）
    T operator-(const ArithmeticRange<T> &other) const
    {
        return this->low - other.high;
    }
};

using  RangeI = ArithmeticRange<int>;
using  RangeF = ArithmeticRange<float>;
using  RangeD = ArithmeticRange<double>;
using  RangeSize = ArithmeticRange<std::size_t>;

//! 像素通道枚举
enum PixChannel : uint8_t
{
    BLUE = 0U,  //!< 蓝色通道
    GREEN = 1U, //!< 绿色通道
    RED = 2U    //!< 红色通道
};