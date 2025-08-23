#include "vc/core/debug_tools.h"

using namespace std;
using namespace cv;

// 单例模式，方便全局调用
DebugTools &DebugTools::get()
{
    static DebugTools instance;
    return instance;
}

// 设置缓存图像（通常在处理开始时调用）
void DebugTools::setImage(const cv::Mat &img)
{
    std::lock_guard<std::mutex> lock(mtx_);
    img.copyTo(cache_);
}

// 获取缓存图像引用（用于绘制调试信息）
cv::Mat DebugTools::getImage()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (cache_.empty())
        return cv::Mat();
    return cache_;
}

// 在固定窗口显示（例如 "Debug"）
void DebugTools::show(const std::string &winName)
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


// 初始化窗口
void DebugTools::initWindow(const std::string &winName)
{
    cv::namedWindow(winName, cv::WINDOW_NORMAL);
    cv::resizeWindow(winName, 640, 480);
}
