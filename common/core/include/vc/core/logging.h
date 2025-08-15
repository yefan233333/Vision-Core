#pragma once

#include <cstdio>
#include <stdexcept>
#include <string>

/**
 * @brief 信息打印管理
 */
#define VC_HIGHLIGHT_INFO(msg...)            \
    do                                       \
    {                                        \
        printf("\033[35m[   INFO   ] " msg); \
        printf("\033[0m\n");                 \
    } while (false)

#define VC_WARNING_INFO(msg...)              \
    do                                       \
    {                                        \
        printf("\033[33m[   WARN   ] " msg); \
        printf("\033[0m\n");                 \
    } while (false)

#define VC_PASS_INFO(msg...)                 \
    do                                       \
    {                                        \
        printf("\033[32m[   PASS   ] " msg); \
        printf("\033[0m\n");                 \
    } while (false)

#define VC_ERROR_INFO(msg...)                \
    do                                       \
    {                                        \
        printf("\033[31m[   ERR    ] " msg); \
        printf("\033[0m\n");                 \
    } while (false)

#define VC_NORMAL_INFO(msg...)       \
    do                               \
    {                                \
        printf("[   INFO   ] " msg); \
        printf("\n");                \
    } while (false)

/**
 * @brief 带详细信息的异常抛出宏
 * @param ... printf格式的错误信息
 *
 * 使用示例：
 * VC_THROW_ERROR("Invalid value: %d", 42);
 *
 * 输出效果：
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * [  ERROR  ] main.cpp:42 (functionName)
 * Invalid value: 42
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 */
#define VC_THROW_ERROR(msg...)                                                      \
    do                                                                              \
    {                                                                               \
        constexpr const char *_file_ = __FILE__;                                    \
        constexpr int _line_ = __LINE__;                                            \
        constexpr const char *_func_ = __func__;                                    \
                                                                                    \
        fprintf(stderr, "\033[31m");                                                \
        fprintf(stderr, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");        \
        fprintf(stderr, "[  ERROR  ] %s:%d (%s)\n", _file_, _line_, _func_);        \
        fprintf(stderr, "\033[33m"); /* 转为黄色显示用户信息 */                     \
        fprintf(stderr, msg);                                                       \
        fprintf(stderr, "\033[0m\n"); /* 结束颜色 */                                \
        fprintf(stderr, "\033[31m");  /* 恢复红色 */                                \
        fprintf(stderr, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m\n"); \
                                                                                    \
        char _msg_[1024];                                                           \
        snprintf(_msg_, sizeof(_msg_), "%s:%d (%s) | ", _file_, _line_, _func_);    \
        snprintf(_msg_ + strlen(_msg_), sizeof(_msg_) - strlen(_msg_), msg);        \
        throw std::runtime_error(_msg_);                                            \
    } while (false)
