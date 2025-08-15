#pragma once

#include <memory>
#include <string>
#include "vc/core/logging.h"

/**
 * @brief 参数展开辅助宏
 *
 * @note 用于展开被括号包裹的参数
 */
#define PROPERTY_WRAPPER_EXPAND_PARAMS(...) __VA_ARGS__ // 展开被括号包裹的参数

/**
 * @brief 核心属性包装类
 * @tparam WrapperType 被包装的属性类型
 *
 * @note 提供属性值的存在状态跟踪、安全访问、类型转发等功能
 */
template <typename WrapperType>
class PropertyWrapper
{
    WrapperType _value;
    bool _has_value = false;

public:
    PropertyWrapper() = default;
    PropertyWrapper(WrapperType &&value) : _value(std::move(value)), _has_value(true) {}
    PropertyWrapper(const WrapperType &value) : _value(value), _has_value(true) {}

    [[nodiscard]] const WrapperType &getValue() const
    {
        if (!_has_value)
            VC_THROW_ERROR("属性未设置");
        return _value;
    }

    WrapperType &getValue()
    {
        if (!_has_value)
            VC_THROW_ERROR("属性未设置");
        return _value;
    }

    [[nodiscard]] bool hasValue() const noexcept { return _has_value; }

    template <typename U>
    void setValue(U &&value)
    {
        _value = std::forward<U>(value);
        _has_value = true;
    }

    void clearValue() noexcept
    {
        _has_value = false;
    }
};

#define DEFINE_PROPERTY(PROPERTY_NAME, READ_SCOPE, WRITE_SCOPE, TYPE)    \
private:                                                                 \
    using _##PROPERTY_NAME##_type = PROPERTY_WRAPPER_EXPAND_PARAMS TYPE; \
    PropertyWrapper<_##PROPERTY_NAME##_type>                             \
        _##PROPERTY_NAME##_prop;                                         \
    READ_SCOPE:                                                          \
    const _##PROPERTY_NAME##_type &get##PROPERTY_NAME() const            \
    {                                                                    \
        return _##PROPERTY_NAME##_prop.getValue();                       \
    }                                                                    \
    _##PROPERTY_NAME##_type &get##PROPERTY_NAME()                        \
    {                                                                    \
        return _##PROPERTY_NAME##_prop.getValue();                       \
    }                                                                    \
    bool isSet##PROPERTY_NAME() const noexcept                           \
    {                                                                    \
        return _##PROPERTY_NAME##_prop.hasValue();                       \
    }                                                                    \
                                                                         \
    WRITE_SCOPE:                                                         \
    template <typename U>                                                \
    void set##PROPERTY_NAME(U &&value)                                   \
    {                                                                    \
        _##PROPERTY_NAME##_prop.setValue(std::forward<U>(value));        \
    }                                                                    \
    void clear##PROPERTY_NAME() noexcept                                 \
    {                                                                    \
        _##PROPERTY_NAME##_prop.clearValue();                            \
    }

#define DEFINE_PROPERTY_WITH_INIT(PROPERTY_NAME, READ_SCOPE, WRITE_SCOPE, TYPE, VALUE...) \
private:                                                                                  \
    using _##PROPERTY_NAME##_type = PROPERTY_WRAPPER_EXPAND_PARAMS TYPE;                  \
    PropertyWrapper<_##PROPERTY_NAME##_type>                                              \
        _##PROPERTY_NAME##_prop{VALUE};                                                   \
    READ_SCOPE:                                                                           \
    const _##PROPERTY_NAME##_type &get##PROPERTY_NAME() const                             \
    {                                                                                     \
        return _##PROPERTY_NAME##_prop.getValue();                                        \
    }                                                                                     \
    _##PROPERTY_NAME##_type &get##PROPERTY_NAME()                                         \
    {                                                                                     \
        return _##PROPERTY_NAME##_prop.getValue();                                        \
    }                                                                                     \
    bool isSet##PROPERTY_NAME() const noexcept                                            \
    {                                                                                     \
        return _##PROPERTY_NAME##_prop.hasValue();                                        \
    }                                                                                     \
                                                                                          \
    WRITE_SCOPE:                                                                          \
    template <typename U>                                                                 \
    void set##PROPERTY_NAME(U &&value)                                                    \
    {                                                                                     \
        _##PROPERTY_NAME##_prop.setValue(std::forward<U>(value));                         \
    }                                                                                     \
    void clear##PROPERTY_NAME() noexcept                                                  \
    {                                                                                     \
        _##PROPERTY_NAME##_prop.clearValue();                                             \
    }
