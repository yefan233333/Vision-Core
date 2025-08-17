#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_hump_param.h"

using namespace std;
using namespace cv;

HumpDetector::HumpDetector(const std::vector<cv::Point> &contour, int _start_idx, int _end_idx) : points(contour.begin() + _start_idx, contour.begin() + _end_idx),
                                                                                              start_idx(_start_idx),
                                                                                              end_idx(_end_idx)
{
    direction = getPointsDirectionVector(points);
    for (auto &&point : points)
    {
        avePoint += static_cast<Point2f>(point);
    }
    avePoint /= static_cast<float>(points.size()); // 平均点
}

// 计算以顶点a为中心的角度
inline double calculateVertexAngle(const Point2f &a, const Point2f &b, const Point2f &c)
{
    Point2f vec1 = b - a;
    Point2f vec2 = c - a;

    // 处理零向量
    double norm1 = norm(vec1);
    double norm2 = norm(vec2);
    if (norm1 < 1e-6 || norm2 < 1e-6)
        return 180.0; // 存在重合点，视为共线

    double dot = vec1.dot(vec2);
    double cosTheta = dot / (norm1 * norm2);
    cosTheta = max(-1.0, min(1.0, cosTheta)); // 约束范围避免数值误差

    return acos(cosTheta) * 180.0 / CV_PI;
}

double HumpDetector::CheckCollinearity(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p3)
{
    // double angle_1 = atan2(p2.y - p1.y, p2.x - p1.x);
    // double angle_2 = atan2(p3.y - p1.y, p3.x - p1.x);
    // double angle_3 = atan2(p3.y - p2.y, p3.x - p2.x);

    // // 向 angle_1 对齐
    // double aligned_angles[3] = {0, angle_2 - angle_1, angle_3 - angle_1};
    // for (int i = 0; i < 3; ++i)
    // {
    //     while(aligned_angles[i] < -M_PI)
    //         aligned_angles[i] += 2 * M_PI;
    //     while(aligned_angles[i] > M_PI)
    //         aligned_angles[i] -= 2 * M_PI;
    // }
    // // 计算角度差

    double angleAtP1 = calculateVertexAngle(p1, p2, p3);
    double angleAtP2 = calculateVertexAngle(p2, p1, p3);
    double angleAtP3 = calculateVertexAngle(p3, p1, p2);

    // 计算最大角度
    double maxAngle = max(angleAtP1, max(angleAtP2, angleAtP3));
    return 180.0 - maxAngle; // 返回偏差角度

    // Point2f v_12 = p2 - p1;
    // Point2f v_13 = p3 - p1;
    // float delta_angle_213 = getVectorMinAngle(v_12,v_13,DEG);

    // Point2f v_21 = p1 - p2;
    // Point2f v_23 = p3 - p2;
    // float delta_angle_321 = getVectorMinAngle(v_21,v_23,DEG);

    // Point2f v_31 = p1 - p3;
    // Point2f v_32 = p2 - p3;
    // float delta_angle_132 = getVectorMinAngle(v_31,v_32,DEG);
    // float delta_angle = max(delta_angle_213,max(delta_angle_321,delta_angle_132));

    // if(delta_angle > 90) delta_angle = 180 - delta_angle;
    // return delta_angle;

    // 计算角度偏差的标准差
}

double HumpDetector::CheckAlignment(const cv::Point2f &direction_1, const cv::Point2f &direction_2, const cv::Point2f &direction_3)
{
    // Point2f ave_direction = (direction_1 + direction_2 + direction_3)/3.0;
    // float max_delta_angle;
    // for(auto&& direction:{direction_1,direction_2,direction_3})
    // {
    //     float delta_angle = getVectorMinAngle(direction,ave_direction,DEG);
    //     max_delta_angle = max(max_delta_angle,delta_angle);
    // }
    // return max_delta_angle;

    // 计算标准差
    double angle_1 = atan2(direction_1.y, direction_1.x);
    double angle_2 = atan2(direction_2.y, direction_2.x);
    double angle_3 = atan2(direction_3.y, direction_3.x);

    // 对齐第一个角度
    double aligned_angles[3] = {0, angle_2 - angle_1, angle_3 - angle_1};
    for (int i = 0; i < 3; ++i)
    {
        while (aligned_angles[i] < -M_PI)
            aligned_angles[i] += 2 * M_PI;
        while (aligned_angles[i] > M_PI)
            aligned_angles[i] -= 2 * M_PI;
    }
    // 计算标准差
    double mean = (aligned_angles[0] + aligned_angles[1] + aligned_angles[2]) / 3.0;
    double variance = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        variance += (aligned_angles[i] - mean) * (aligned_angles[i] - mean);
    }
    variance /= 3.0;
    double stddev = sqrt(variance);
    // #if 1
    //     // 打印各个角度
    //     cout << "aligned angles: \n:";
    //     for (int i = 0; i < 3; ++i)
    //     {
    //         cout << aligned_angles[i] * 180.0 / M_PI << " ";
    //     }
    //     cout << "\n";
    //     cout << "mean: " << mean * 180.0 / M_PI << "\n";
    //     cout << "variance: " << variance << "\n";
    //     cout << endl;
    // #endif

    return stddev * 180.0 / M_PI; // 转换为角度
}
