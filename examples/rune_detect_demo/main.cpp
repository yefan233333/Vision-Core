#include <opencv2/opencv.hpp>
#include "rune_detect_demo/rune_detect_demo.h"
#include "rv/contour_proc/contour_wrapper.hpp"
#include <vector>
#include <iostream>
#include <cmath>



using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap(0); // 打开默认摄像头
    if (!cap.isOpened())
    {
        cerr << "Error: Could not open camera." << endl;
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
        threshold(gray, binary, 128, 255, THRESH_BINARY);

        // 提取轮廓
        vector<Contour_ptr> contours;
        vector<Vec4i> hierarchy;
        findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        
        // 绘制轮廓
        Mat contourImage = frame.clone();
        drawContours(contourImage, contours, -1, Scalar(0, 255, 0), 2);
        // 在这里添加图像处理代码

        imshow("Camera Feed", frame);
        imshow("Contours", contourImage);
        imshow("Binary Image", binary);
        if (waitKey(30) >= 0) break; // 按任意键退出
    }
}