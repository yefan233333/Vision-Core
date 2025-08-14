# 禁止根目录构建
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "禁止在源码目录中进行构建,防止编译生成文件污染源代码")
endif()


cmake_minimum_required(VERSION 3.15)

project(RuneVision
        VERSION 0.1.0)

# cmake 函数/宏
include(cmake/RuneVisionModule.cmake)

# cmake 编译选项
include(cmake/RuneVisionCompilerOptions.cmake)


# RuneVision 通用基础模块
add_subdirectory(common)

# RuneVision 具体实现模块
add_subdirectory(modules)

# RuneVision 测试模块
add_subdirectory(examples)