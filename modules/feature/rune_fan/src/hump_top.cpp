/**
 * @file hump_finder.cpp
 * @author 张峰玮 (3480409161@qq.com)
 * @brief   神符扇叶的顶部突起点类的源文件
 * @version 0.1
 * @date 2024-11-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_hump_param.h"
#include "vc/core/debug_tools.h"
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define RUNE_FAN_DEBUG 1

struct TopHumpDebugParam
{
    // 是否打印调试信息
    bool ENABLE_DEBUG = false;

    YML_INIT(TopHumpDebugParam,
             YML_ADD_PARAM(ENABLE_DEBUG);)
};
inline TopHumpDebugParam top_hump_debug_param;


#define rune_fan_debug_0
// #define use_center_direction

#define rune_top_hump_debug 1

#ifdef rune_fan_debug_0
cv::Mat rune_fan_debug_0_img;
std::string winname = "rune fan debug img";
bool debugWinname_flag = true;
#endif

TopHump::TopHump(int _up_start_idx, int _up_end_idx,
                 int _down_start_idx, int _down_end_idx,
                 int _line_pair_idx,
                 const cv::Point2f &_direction, const cv::Point2f &_center)
{
    up_start_idx = _up_start_idx;
    up_end_idx = _up_end_idx;

    down_start_idx = _down_start_idx;
    down_end_idx = _down_end_idx;

    line_pair_idx = _line_pair_idx;

    direction = _direction;
    center = _center;
    up_itertaion_num = _up_end_idx - _up_start_idx;
    down_itertaion_num = _down_end_idx - _down_start_idx;
    end_iteration_up_idx = _up_end_idx;
    current_state = DOWN;
}

TopHump::TopHump(int up_idx, int down_idx, state find_state, const cv::Point2f &_direction)
{
    current_state = find_state;
    // 在前向查找的过程中，需要更新的数据
    if (find_state == TopHump::UP)
    {
        up_start_idx = up_idx;
        down_start_idx = down_idx;
        up_end_idx = 0;
        down_end_idx = 0;
    }
    else if (find_state == TopHump::DOWN)
    {
        up_start_idx = 0;
        down_start_idx = 0;
        up_end_idx = up_idx;
        down_end_idx = down_idx;
    }

    up_itertaion_num = 0;
    down_itertaion_num = 0;
    end_iteration_up_idx = up_idx;
    direction = _direction;
}

std::vector<TopHump> TopHump::getTopHumps(const std::vector<cv::Point> &contour_plus, const cv::Point2f &contour_center, const std::vector<std::tuple<Line, Line>> &line_pairs)
{
    // 轮廓太小，无法进行查找
    if (contour_plus.size() < 20)
    {
        VC_WARNING_INFO("fan active get_top_humps : 轮廓长度小于20,无法进行查找,轮廓数量：%zu", contour_plus.size());
        return {};
    }

    // 收集所有突起点
    vector<TopHump> humps{};
    TopHump::getAllHumps2(contour_plus, line_pairs, humps);
    // TopHump::getAllHumps(contour_plus,humps);
    if (humps.size() < 3)
    {
        VC_WARNING_INFO("fan active get_top_humps : 收集到的突起点数不足3个,无法进行查找,轮廓数量：%zu", contour_plus.size());
        return {}; // 获取到的角点数不到3个，放弃构造
    }

    // 过滤收集到的突起点
    TopHump::filter(contour_plus, humps);
    if (humps.size() < 3)
    {
        VC_WARNING_INFO("fan active get_top_humps : 过滤后的突起点数不足3个,无法进行查找,轮廓数量：%zu", contour_plus.size());
        return {}; // 过滤后的角点数不到3个，放弃构造
    }
    // 存储成功构造的突起点组
    vector<tuple<TopHumpCombo, double>> hump_combos{};

// #if RUNE_FAN_DEBUG
//     do
//     {
//         // 绘制所有突起点
//         Mat img_show = DebugTools::get().getImage();
//         for (const auto &hump : humps)
//         {
//             auto center = hump.getCenter();
//             auto direction = hump.getDirection();
//             cv::arrowedLine(img_show, center, center + direction * 50, cv::Scalar(0, 255, 0), 2); 
//         }
//     } while (0);
    

// #endif

    // 尝试构造突起点
    for (size_t i = 0; i < humps.size() - 2; ++i)
    {
        for (size_t j = i + 1; j < humps.size() - 1; ++j)
        {
            for (size_t k = j + 1; k < humps.size(); ++k)
            {
                TopHumpCombo hump_combo = {humps[i], humps[j], humps[k]}; // 用于分类的突起点
                double delta = 0;
                if (TopHump::make_TopHumps(hump_combo, contour_center, delta))
                {
                    hump_combos.emplace_back(hump_combo, delta);
                }
            }
        }
    }
    if (hump_combos.empty())
    {
        return {};
    }
    // 选择误差最小的突起点组
    auto [best_combo, delta] = *min_element(hump_combos.begin(), hump_combos.end(), [](const auto &a, const auto &b)
                                            { return get<1>(a) < get<1>(b); });

    return best_combo.getHumpsVector();
}

// 计算高斯函数值
float gaussian(float x, float sigma)
{
    return (1.0 / (std::sqrt(2 * M_PI) * sigma)) * std::exp(-x * x / (2 * sigma * sigma));
}
// 归一化
void normalize(std::vector<float> &kernel)
{
    float sum = 0.0;
    for (float k : kernel)
    {
        sum += k;
    }
    for (float &k : kernel)
    {
        k /= sum;
    }
}
// 生成一维高斯滤波器核
std::vector<float> createGaussianKernel(int size, float sigma)
{
    std::vector<float> kernel(size);
    int center = size / 2;

    for (int i = 0; i < size; ++i)
    {
        float x = i - center;
        kernel[i] = gaussian(x, sigma);
    }

    // 归一化
    normalize(kernel);

    return kernel;
}
/**
 * @brief 更新突起点数据
 * @param[in] hump 突起点
 */
void TopHump::update(const TopHump &hump)
{
    current_state = hump.current_state;
    if (current_state == TopHump::UP)
    {
        up_start_idx = min(up_start_idx, hump.up_start_idx);
        down_start_idx = min(down_start_idx, hump.down_start_idx);
        up_itertaion_num++;
    }
    else if (current_state == TopHump::DOWN)
    {
        up_end_idx = max(up_end_idx, hump.up_end_idx);
        down_end_idx = max(down_end_idx, hump.down_end_idx);
        down_itertaion_num++;
    }
    end_iteration_up_idx = hump.end_iteration_up_idx;

    // 更新方向
    int iteration_num = up_itertaion_num + down_itertaion_num;
    float ratio_old = static_cast<float>(iteration_num - 1) / static_cast<float>(iteration_num);
    float ratio_new = static_cast<float>(1) / static_cast<float>(iteration_num);
    direction = direction * ratio_old + hump.direction * ratio_new;
}

/**
 * @brief 获取所有突起点
 * @param[in] contour_plus 加长后的轮廓
 * @param[in] contour_center 轮廓中心
 * @param[out] humps 输出的突起点集。
 * @return 是否查找成功(找到三个以上的突起点算作成功)
 */
inline bool TopHump::getAllHumps2(const std::vector<cv::Point> &contours_plus, const std::vector<std::tuple<Line, Line>> &line_pairs, std::vector<TopHump> &humps)
{
    // 获取所有的突起点
    humps.clear();

    for (int i = 0; i < line_pairs.size(); i++)
    {
        auto &[up_line, down_line] = line_pairs[i];
        int up_start_idx = up_line.start_idx;
        int up_end_idx = up_line.end_idx;
        int down_start_idx = down_line.start_idx;
        int down_end_idx = down_line.end_idx;
        Point2f direction = Point2f(cos(deg2rad(up_line.angle)), sin(deg2rad(up_line.angle))); // 以上升线段的角度为方向
        Point2f center = (contours_plus[up_start_idx] + contours_plus[down_start_idx]) / 2.0;

        humps.emplace_back(up_start_idx, up_end_idx, down_start_idx, down_end_idx, i, direction, center);
    }

    for (auto &hump : humps)
    {
        setVertex(hump, contours_plus);
    }

    if (humps.size() < 3)
    {
        return false;
    }
    return true;
}

bool TopHump::filter(const std::vector<cv::Point> &contour_plus, std::vector<TopHump> &humps)
{
    if (humps.empty())
        return false;

    // 过滤几乎重合的角点
    for (auto it_1 = humps.begin(); it_1 != humps.end(); it_1++)
    {
        for (auto it_2 = it_1 + 1; it_2 != humps.end();)
        {
            // 若两角点的方向相同，且距离小于最小间隔，则使用正常阈值
            if (getVectorMinAngle(it_1->getDirection(), it_2->getDirection(), DEG) < rune_fan_hump_param.TOP_HUMP_MAX_ALIGNMENT_DELTA)
            {
                if (getDist(it_1->getVertex(), it_2->getVertex()) < rune_fan_hump_param.TOP_HUMP_MIN_INTERVAL)
                    it_2 = humps.erase(it_2);
                else
                    it_2++;
            }
            else // 若两个角点的方向不同，且距离小于最小间隔，则阈值翻倍
            {
                if (getDist(it_1->getVertex(), it_2->getVertex()) < 2 * rune_fan_hump_param.TOP_HUMP_MIN_INTERVAL)
                    it_2 = humps.erase(it_2);
                else
                    it_2++;
            }
        }
    }

    return true;
}


/**
 * @brief 尝试构造突起点
 * @param[in] humps 用于构造的hump数组，长度为3，构造成功则输出排序后的humps
 * @param[in] contour_center 轮廓中心,用于确定左右突起点
 * @param[out] delta 平均偏差角度
 * @return 是否构造成功
 */
bool TopHump::make_TopHumps(TopHumpCombo &hump_combo, const cv::Point2f &contour_center, double &delta)
{
    auto &humps = hump_combo.humps();

    vector<TopHump> humps_to_classify(humps.begin(), humps.end()); // 用于分类的突起点
    // 当扇叶在神符中心上方时，
    TopHump left_h;   // 左边突起点
    TopHump right_h;  // 右边突起点
    TopHump center_h; // 中心突起点

    // 获取中心突起点
    Point2f aveVertex{};
    for (auto &hump : humps)
    {
        aveVertex += hump.getVertex();
    }
    aveVertex /= static_cast<float>(humps.size());
    auto center_hump_it = min_element(humps_to_classify.begin(), humps_to_classify.end(),
                                      [&](const TopHump &h1, const TopHump &h2)
                                      {
                                          return getDist(h1.getVertex(), aveVertex) < getDist(h2.getVertex(), aveVertex);
                                      });
    center_h = *center_hump_it;
    humps_to_classify.erase(center_hump_it);

    if (humps_to_classify.size() != 2)
    {
        VC_THROW_ERROR("Size of the \" humps_to_classify \" are not equal to 2. (size = %zu)", humps_to_classify.size());
        return 0;
    }
    // 获取左、右突起点
    Point2f center2hump_0 = humps_to_classify[0].getVertex() - contour_center;
    Point2f center2hump_1 = humps_to_classify[1].getVertex() - contour_center;
    if (center2hump_0.x * center2hump_1.y - center2hump_0.y * center2hump_1.x > 0)
    {
        left_h = humps_to_classify[0];
        right_h = humps_to_classify[1];
    }
    else
    {
        left_h = humps_to_classify[1];
        right_h = humps_to_classify[0];
    }
    // 判断是否三点共线
    double collinearity_delta = HumpDetector::CheckCollinearity(left_h.getVertex(), center_h.getVertex(), right_h.getVertex());
    if (collinearity_delta > rune_fan_hump_param.TOP_HUMP_MAX_COLLINEAR_DELTA)
    {
        return false;
    }

    // 判断中心突起点的方向是否正确
    float direction_delta = getVectorMinAngle(center_h.getDirection(), center_h.getVertex() - contour_center, DEG);
    // 计算距离
    float vertex_to_center_distance = getDist(center_h.getVertex(), contour_center);
    //  计算各自的角度
    float angle_1 = rad2deg(atan2(center_h.getVertex().y - contour_center.y, center_h.getVertex().x - contour_center.x));
    float angle_2 = rad2deg(atan2(center_h.getDirection().y, center_h.getDirection().x));
    if (direction_delta > rune_fan_hump_param.TOP_HUMP_MAX_DIRECTION_DELTA)
    {
        return false;
    }
    // 判断是否同向
    double alignment_delta = HumpDetector::CheckAlignment(left_h.getDirection(), center_h.getDirection(), right_h.getDirection());
    if (alignment_delta > rune_fan_hump_param.TOP_HUMP_MAX_ALIGNMENT_DELTA)
    {
        return false;
    }
    // 判断距离比例是否正常
    double distance_ratio = getDist(left_h.getVertex(), center_h.getVertex()) / getDist(center_h.getVertex(), right_h.getVertex());
    if (distance_ratio < 1.0)
        distance_ratio = 1.0 / distance_ratio;
    if (distance_ratio > rune_fan_hump_param.TOP_HUMP_MAX_DISTANCE_RATIO)
    {
        return false;
    }
    // 判断突起点连线与其方向是否接近垂直
    // 直线拟合
    cv::Vec4f line;
    cv::fitLine(vector<Point2f>{left_h.getVertex(), center_h.getVertex(), right_h.getVertex()}, line, cv::DIST_L2, 0, 0.01, 0.01);
    Point2f ave_direction = (left_h.getDirection() + center_h.getDirection() + right_h.getDirection()) / 3.0;
    double line_direction_delta = getVectorMinAngle(Point2f(line[0], line[1]), ave_direction, DEG);
    line_direction_delta = line_direction_delta > 90 ? 180 - line_direction_delta : line_direction_delta;
    if (line_direction_delta < rune_fan_hump_param.TOP_HUMP_MIN_LINE_DIRECTION_DELTA)
    {
        return false;
    }
    // 设置误差权重
    float collinearity_weight = 0;
    float direction_weight = 0;
    float alignment_weight = 1;
    float distance_weight = 0;
    // 总权重
    float total_weight = collinearity_weight + direction_weight + alignment_weight + distance_weight;

    // 计算加权平均值
    delta = collinearity_weight * pow(collinearity_delta, 2) + direction_weight * pow(direction_delta, 2) + alignment_weight * pow(alignment_delta, 2) + distance_weight * pow(distance_ratio, 2);

    humps = {left_h, center_h, right_h};
    return true;
}

/**
 * @brief 设置突起点的角点
 * @param[in] hump 突起点
 * @param[in] contour_plus 加长后的轮廓
 * @return 是否设置成功
 */
bool TopHump::setVertex(TopHump &hump, const std::vector<cv::Point> &contour_plus)
{
    if (contour_plus.size() < 20) // 轮廓太小，无法设置角点
    {
        return false;
    }

    // 获取在突起方向上、距离突起中心最远的轮廓点
    auto farthest_point = max_element(contour_plus.begin() + hump.up_start_idx, contour_plus.begin() + hump.down_end_idx,
                                      [&](const Point &p1, const Point &p2)
                                      {
                                          return getProjection(static_cast<Point2f>(p1) - hump.getCenter(), hump.getDirection()) < getProjection(static_cast<Point2f>(p2) - hump.getCenter(), hump.getDirection());
                                      });
    hump.vertex = hump.getCenter() + getProjectionVector(static_cast<Point2f>(*farthest_point) - hump.getCenter(), hump.getDirection());
    // hump.vertex = *farthest_point;
    return true;
}