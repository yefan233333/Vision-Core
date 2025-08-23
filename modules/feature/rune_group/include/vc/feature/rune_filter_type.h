#pragma once
#include <type_traits>

/**
 * @brief 进行滤波的目标数据的类型枚举
 */
enum class RuneFilterDataType : unsigned int
{
    XYZ = 1 << 0,       //!< x y z
    YAW_PITCH = 1 << 1, //!< yaw pitch
    ROLL = 1 << 2,      //!< roll
};

// 重载运算符 | (组合标志位)
inline RuneFilterDataType operator|(RuneFilterDataType a, RuneFilterDataType b)
{
    using underlying = std::underlying_type_t<RuneFilterDataType>;
    return static_cast<RuneFilterDataType>(static_cast<underlying>(a) | static_cast<underlying>(b));
}

// 重载运算符 & (检查标志位)
inline RuneFilterDataType operator&(RuneFilterDataType a, RuneFilterDataType b)
{
    using underlying = std::underlying_type_t<RuneFilterDataType>;
    return static_cast<RuneFilterDataType>(static_cast<underlying>(a) & static_cast<underlying>(b));
}

// 重载运算符 ~ (取反)
inline RuneFilterDataType operator~(RuneFilterDataType a)
{
    using underlying = std::underlying_type_t<RuneFilterDataType>;
    return static_cast<RuneFilterDataType>(~static_cast<underlying>(a));
}