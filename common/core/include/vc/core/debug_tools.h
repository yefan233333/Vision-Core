#pragma once

#include <opencv2/opencv.hpp>

#pragma once
#include <opencv2/opencv.hpp>
#include <mutex>
#include <string>

class DebugTools
{
public:
    // 单例模式，方便全局调用
    static DebugTools &get();

    // 设置缓存图像（通常在处理开始时调用）
    void setImage(const cv::Mat &img);

    // 获取缓存图像引用（用于绘制调试信息）
    cv::Mat getImage();

    // 在固定窗口显示（例如 "Debug"）
    void show(const std::string &winName = "Debug");

    // 禁止复制和赋值
    DebugTools(const DebugTools &) = delete;
    DebugTools &operator=(const DebugTools &) = delete;

private:
    DebugTools() = default;

    // 初始化窗口
    void initWindow(const std::string &winName = "Debug");



    cv::Mat cache_;
    std::mutex mtx_;
    bool window_initialized_ = false;
};
