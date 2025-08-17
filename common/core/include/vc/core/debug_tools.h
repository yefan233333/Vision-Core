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
    static DebugTools &get()
    {
        static DebugTools instance;
        return instance;
    }

    // 设置缓存图像（通常在处理开始时调用）
    void setImage(const cv::Mat &img)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        img.copyTo(cache_);
    }

    // 获取缓存图像引用（用于绘制调试信息）
    cv::Mat getImage()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (cache_.empty())
            return cv::Mat();
        return cache_;
    }


    // 在固定窗口显示（例如 "Debug"）
    void show(const std::string &winName = "Debug")
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!window_initialized_)
        {
            initWindow(winName);
            window_initialized_ = true;
        }

        if (!cache_.empty())
        {
            cv::imshow(winName, cache_);
        }
    }

    // 禁止复制和赋值
    DebugTools(const DebugTools &) = delete;
    DebugTools &operator=(const DebugTools &) = delete;

private:
    DebugTools() = default;

    // 初始化窗口
    void initWindow(const std::string &winName = "Debug")
    {
        cv::namedWindow(winName, cv::WINDOW_NORMAL);
        cv::resizeWindow(winName, 640, 480);
    }



    cv::Mat cache_;
    std::mutex mtx_;
    bool window_initialized_ = false;
};
