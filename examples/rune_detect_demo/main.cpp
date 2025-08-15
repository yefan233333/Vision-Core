#include <opencv2/opencv.hpp>
#include "rune_detect_demo/rune_detect_demo.h"
#include "vc/contour_proc/contour_wrapper.hpp"
// #include "vc/feature/rune_target.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_inactive.h"
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
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
    while (true)
    {
        cap >> frame; // 从摄像头捕获一帧
        if (frame.empty())
        {
            cerr << "Error: Could not read frame." << endl;
            break;
        }

        // 二值化
        Mat gray, binary;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 70, 255, THRESH_BINARY);
        Mat contourImage = frame.clone();

        // 提取轮廓
        vector<Contour_ptr> contours;
        vector<Vec4i> hierarchy;
        findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // 搜索靶心轮廓
        vector<RuneTargetInactive_ptr> targets;
        unordered_map<RuneTargetInactive_ptr, unordered_set<size_t>> used_contour_idxs;
        unordered_set<size_t> all_sub_idx;
        RuneTargetInactive::find(targets, contours, hierarchy, all_sub_idx, used_contour_idxs);
        if(targets.size() > 0)
        {
            // cout << "Found " << targets.size() << " inactive rune targets." << endl;
            cout << "target_size = " << targets.size() << endl;

            // 绘制靶心的位置和四个缺口的位置
            for (const auto& target : targets)
            {
                circle(contourImage, target->getImageCache().getCenter(), 5, Scalar(255, 0, 0), -1);
                for (const auto& gap : target->getGaps())
                {
                    circle(contourImage, gap.center, 5, Scalar(0, 0, 255), -1);
                }
            }
        }
        else
        {
            // cout << "No inactive rune targets found." << endl;
        }



        // 绘制轮廓

        // drawContours(contourImage, contours, -1, Scalar(0, 255, 0), 2);
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
        imshow("Binary Image", binary);
        if (waitKey(30) >= 0)
            break; // 按任意键退出
    }
}