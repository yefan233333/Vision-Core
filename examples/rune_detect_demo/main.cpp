#include <opencv2/opencv.hpp>
#include "rune_detect_demo/rune_detect_demo.h"
#include "vc/contour_proc/contour_wrapper.hpp"
// #include "vc/feature/rune_target.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_active.h"
#include "vc/core/debug_tools.h"
#include "vc/feature/rune_fan.h"
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

//! 像素通道枚举
enum PixChannel : uint8_t
{
    BLUE = 0U,  //!< 蓝色通道
    GREEN = 1U, //!< 绿色通道
    RED = 2U    //!< 红色通道
};

inline cv::Mat binary(cv::Mat src, PixChannel ch1, PixChannel ch2, uint8_t thresh)
{
    if (src.type() != CV_8UC3)
    {
        VC_THROW_ERROR("The image type of \"src\" is incorrect");
    }
    Mat bin = Mat::zeros(Size(src.cols, src.rows), CV_8UC1);
    // Image process
    parallel_for_(Range(0, src.rows),
                  [&](const Range &range)
                  {
                      uchar *data_src = nullptr;
                      uchar *data_bin = nullptr;
                      for (int row = range.start; row < range.end; ++row)
                      {
                          data_src = src.ptr<uchar>(row);
                          data_bin = bin.ptr<uchar>(row);
                          for (int col = 0; col < src.cols; ++col)
                              if (data_src[3 * col + ch1] - data_src[3 * col + ch2] > thresh)
                                  data_bin[col] = 255;
                      }
                  });
    return bin;
}

int main(int argc, char **argv)
{
    // 将第一个参数作为视频路径，读取视频
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <video_path>" << endl;
        return -1;
    }

    VideoCapture cap(argv[1]); // 打开指定视频文件
    if (!cap.isOpened())
    {
        cerr << "Error: Could not open video file." << endl;
        return -1;
    }
    Mat frame;

    // 获取视频总帧数
    int total_frames = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    int current_frame = 0;

    // 创建进度条窗口
    const string trackbar_window = "Camera Feed";
    namedWindow(trackbar_window, WINDOW_NORMAL);

    // 回调函数用于拖动进度条时跳转到指定帧
    auto on_trackbar = [](int pos, void* userdata) {
        VideoCapture* cap_ptr = static_cast<VideoCapture*>(userdata);
        cap_ptr->set(CAP_PROP_POS_FRAMES, pos);
    };
    createTrackbar("Frame", trackbar_window, &current_frame, max(1, total_frames - 1), on_trackbar, &cap);

    // 读取第一帧
    cap.set(CAP_PROP_POS_FRAMES, current_frame);
    cap >> frame;

    while (true)
    {
        cap >> frame; // 从摄像头捕获一帧
        if (frame.empty())
        {
            cerr << "Error: Could not read frame." << endl;
            break;
        }
        DebugTools::get().setImage(frame);

        // 二值化
        Mat bin = binary(frame, PixChannel::RED, PixChannel::BLUE, 160);
        Mat contourImage = frame.clone();

        // 提取轮廓
        vector<Contour_ptr> contours;
        vector<Vec4i> hierarchy;
        findContours(bin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

        // 搜索扇叶轮廓
        vector<RuneFanActive_ptr> activeFans;
        unordered_map<FeatureNode_ptr, unordered_set<size_t>> used_contour_idxs;
        unordered_set<size_t> mask; // 可以跳过的轮廓下标集合
        RuneFanActive::find(activeFans, contours, hierarchy, mask, used_contour_idxs);

        for (const auto &fan : activeFans)
        {
            const auto& corners = fan->getImageCache().getCorners();
            if (corners.empty())
                continue;
            // 连线
            for (size_t i = 0; i < corners.size(); ++i)
            {
                line(contourImage, corners[i], corners[(i + 1) % corners.size()], Scalar(0, 255, 0), 2);
            }
        }
        // 绘制轮廓
        // 在这里添加图像处理代码

        namedWindow("Camera Feed", WINDOW_NORMAL);
        namedWindow("Contours", WINDOW_NORMAL);
        namedWindow("Binary Image", WINDOW_NORMAL);

        resizeWindow("Camera Feed", 640, 480);
        resizeWindow("Contours", 640, 480);
        resizeWindow("Binary Image", 640, 480);

        // 分别设置到屏幕的三个角(左上、右上、右下)
        moveWindow("Camera Feed", 0, 0);
        moveWindow("Contours", 640, 0);
        moveWindow("Binary Image", 0, 480);

        imshow("Camera Feed", frame);
        imshow("Contours", contourImage);
        imshow("Binary Image", bin);

        DebugTools::get().show();

        int key = waitKey(16);
        if(key == 27) // 按 'ESC' 键退出
        {
            break;
        }
        else if (key == ' ') // 按空格键暂停/继续
        {
            while (true)
            {
                int key2 = waitKey(0);
                if (key2 == ' ') // 再次按空格键继续
                {
                    break;
                }
                else if (key2 == 27) // 按 'ESC' 键退出
                {
                    return 0;
                }
            }
        }
    }
}