#include <opencv2/opencv.hpp>
#include "rune_detect_demo/rune_detect_demo.h"
#include "vc/contour_proc/contour_wrapper.hpp"
// #include "vc/feature/rune_target.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_active.h"
#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_center.h"
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
        vector<Contour_cptr> contours;
        vector<Vec4i> hierarchy;
        findContours(bin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        //! 删除一些面积过小的轮廓
        for(size_t i = 0;i < contours.size(); ++i){
            auto area = contours[i]->area();
            if(area < 20)
            {
                deleteContour(contours, hierarchy, i);
            }
        }

        // 搜索靶心轮廓
        vector<FeatureNode_ptr> inactiveTargets;
        unordered_map<FeatureNode_cptr, unordered_set<size_t>> used_contour_idxs;
        unordered_set<size_t> mask; // 可以跳过的轮廓下标集合
        RuneTarget::find_inactive_targets(inactiveTargets, contours, hierarchy, mask, used_contour_idxs);
        for(const auto &target : inactiveTargets)
        {
            mask.insert(used_contour_idxs[target].begin(), used_contour_idxs[target].end());
        }

        // 搜索扇叶轮廓
        vector<FeatureNode_ptr> activeFans;
        RuneFan::find_active_fans(activeFans, contours, hierarchy, mask, used_contour_idxs);
        for(const auto &fan : activeFans)
        {
            mask.insert(used_contour_idxs[fan].begin(), used_contour_idxs[fan].end());
        }

        // 搜索已激活靶心
        vector<FeatureNode_ptr> activeTargets;
        RuneTarget::find_active_targets(activeTargets, contours, hierarchy, mask, used_contour_idxs);
        for(const auto &target : activeTargets)
        {
            mask.insert(used_contour_idxs[target].begin(), used_contour_idxs[target].end());
        }

        // 搜索未激活扇叶
        vector<FeatureNode_ptr> inactiveFans;
        RuneFan::find_inactive_fans(inactiveFans, contours, hierarchy, mask, used_contour_idxs, to_const_ptr(inactiveTargets));
        for(const auto &fan : inactiveFans)
        {
            mask.insert(used_contour_idxs[fan].begin(), used_contour_idxs[fan].end());  
        }

        // 搜索神符中心
        vector<FeatureNode_ptr> runeCenters;
        RuneCenter::find(runeCenters, contours, hierarchy, mask, used_contour_idxs);    
        for(const auto &center : runeCenters)
        {
            mask.insert(used_contour_idxs[center].begin(), used_contour_idxs[center].end());
        }

        vector<FeatureNode_ptr> features;
        for(const auto &target : inactiveTargets)
        {
            features.push_back(target);
        }
        for(const auto &fan : activeFans)
        {
            features.push_back(fan);
        }
        for(const auto &target : activeTargets)
        {
            features.push_back(target);
        }
        for(const auto &fan : inactiveFans)
        {
            features.push_back(fan);
        }
        for(const auto &center : runeCenters)
        {
            features.push_back(center);
        }


        for (const auto &feature : features)
        {
            // 绘制角点
            const auto& corners = feature->imageCache().getCorners();
            const Scalar color = Scalar(0, 255, 0); // 绿色
            for(int i = 0 ;i < static_cast<int>(corners.size()); ++i)
            {
                line(contourImage, corners[i], corners[(i + 1) % corners.size()], color, 2);
            }
            // 绘制方向
            do
            {
                const auto fan_ptr = RuneFan::cast(feature);
                if(!fan_ptr)
                    break;
                const auto &direction = fan_ptr->getDirection();
                if (norm(direction) < 1e-6)
                    break;
                const auto &center = fan_ptr->imageCache().getCenter();
                const auto end_point = center + direction * 50; // 方向线长度为50
                cv::arrowedLine(contourImage, center, end_point, color, 2, LINE_AA, 0, 0.1);
            }while(0);
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