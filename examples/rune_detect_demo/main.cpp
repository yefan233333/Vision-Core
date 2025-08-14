#include <opencv2/opencv.hpp>
#include "rune_detect_demo/rune_detect_demo.h"
#include <vector>
#include <iostream>
#include <cmath>

int angleX_slider = 0;
int angleY_slider = 0;
int angleZ_slider = 0;
int tx_slider = 300;
int ty_slider = 300;
int tz_slider = 300;

using namespace std;
using namespace cv;

double sliderToRad(int slider_val) { return slider_val * CV_PI / 180.0; }
double sliderToTrans(int slider_val) { return slider_val / 1.0; }

int main()
{
    cv::Mat img(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));

    // 立方体顶点
    std::vector<cv::Vec3d> cubeVertices = {
        {-50, -50, -50}, {50, -50, -50}, {50, 50, -50}, {-50, 50, -50}, {-50, -50, 50}, {50, -50, 50}, {50, 50, 50}, {-50, 50, 50}};

    std::vector<std::pair<int, int>> edges = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    cv::namedWindow("Cube");
    cv::createTrackbar("Angle X", "Cube", &angleX_slider, 360);
    cv::createTrackbar("Angle Y", "Cube", &angleY_slider, 360);
    cv::createTrackbar("Angle Z", "Cube", &angleZ_slider, 360);
    cv::createTrackbar("TX", "Cube", &tx_slider, 600);
    cv::createTrackbar("TY", "Cube", &ty_slider, 600);
    cv::createTrackbar("TZ", "Cube", &tz_slider, 600);

    Transform6D tf;

    while (true)
    {
        img.setTo(cv::Scalar(255, 255, 255));

        // 重置旋转矩阵
        tf = Transform6D(); // 默认单位旋转 + 零平移

        // 使用新增的旋转函数
        tf.rotate_x(angleX_slider, DEG);
        cout << "angleX_slider = " << angleX_slider << endl;
        tf.rotate_y(angleY_slider, DEG);
        tf.rotate_z(angleZ_slider, DEG);

        // 设置平移
        tf.tvec(cv::Vec3d(sliderToTrans(tx_slider),
                          sliderToTrans(ty_slider),
                          sliderToTrans(tz_slider) + 3.0));

        // 投影立方体顶点
        std::vector<cv::Point> projected;
        for (auto &v : cubeVertices)
        {
            cv::Vec3d p = tf.rmat() * v + tf.tvec();
            double f = 300.0;
            int x = int(p[0] * f / img.cols + img.cols / 2);
            int y = int(-p[1] * f / img.rows + img.rows / 2);
            projected.emplace_back(x, y);
        }

        cout << "tf.tvec() = \n" << tf.tvec() << endl;
        cout << "tf.rvec() = \n" << tf.rvec() << endl;

        // 绘制立方体
        for (auto &e : edges)
        {
            cv::line(img, projected[e.first], projected[e.second], cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("Cube", img);
        char key = cv::waitKey(30);
        if (key == 27)
            break; // ESC退出
    }

    return 0;
}