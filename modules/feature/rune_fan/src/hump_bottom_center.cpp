#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_hump_param.h"
#include "vc/math/geom_utils.hpp"

using namespace std;
using namespace cv;

BottomCenterHump::BottomCenterHump(const cv::Point2f &_center, const cv::Point2f &direction, const int _idx)
{
    center = _center;
    this->direction = direction;
    idx = _idx;
}

std::vector<BottomCenterHump> BottomCenterHump::getBottomCenterHump(const std::vector<cv::Point> &contour, const cv::Point2f &contour_center, const std::vector<TopHump> &top_humps)
{
    if (contour.size() < 30)
        return {};

    if (top_humps.size() != 3)
    {
        VC_THROW_ERROR("Size of the \" top_humps \" are not equal to 3. (size = %zu)", top_humps.size());
    }

    vector<BottomCenterHump> humps;
    // 获取所有底部中心突起点
    if (getAllHumps(contour, top_humps, humps) == false) // 未找到底部中心突起点
    {
        return {};
    }

    // 过滤所有突起点，只保留最优的突起点
    if (filter(contour, top_humps, humps) == false) // 过滤后的突起点数不足 1 个，判断为查找失败
    {
        return {};
    }

    if (humps.size() != 1)
    {
        VC_THROW_ERROR(" humps \" are not equal to 1. (size = %zu)", humps.size());
        return {};
    }
    // 设置底部突起点的角点（即为突起中心点）
    humps[0].vertex = (humps[0].getCenter());

    return humps;
}

bool BottomCenterHump::getAllHumps(const std::vector<cv::Point> &contour,
                                   const std::vector<TopHump> &top_humps,
                                   std::vector<BottomCenterHump> &humps)
{
    if (contour.size() < 30)
        return false;
    if (top_humps.size() != 3)
    {
        VC_THROW_ERROR("Size of the \" top_humps \" are not equal to 3. (size = %zu)", top_humps.size());
    }
    // 利用扇叶顶部中心突起点的信息，获取扇叶的中轴线
    Point2f line_center = top_humps[1].getCenter();
    Point2f line_direction = -1 * top_humps[1].getDirection();

    vector<BottomCenterHump> found_humps{};

    Point last_Point = contour.back();
    int idx = 0;
    for (auto &point : contour)
    {
        Point2f center2point = static_cast<Point2f>(point) - line_center;
        Point2f center2last_point = static_cast<Point2f>(last_Point) - line_center;
        if ((center2point.x * line_direction.y - center2point.y * line_direction.x) * (center2last_point.x * line_direction.y - center2last_point.y * line_direction.x) < 0) // 判断为交点
        {
            Point2f avePoint = (static_cast<Point2f>(point) + static_cast<Point2f>(last_Point)) / 2;
            found_humps.emplace_back(avePoint, line_direction, idx);
        }
        last_Point = point;
        idx++;
    }
    if (found_humps.size() < 1)
        return false; // 未找到底部中心突起点

    humps = move(found_humps);
    return true;
}

bool BottomCenterHump::filter(const std::vector<cv::Point> &contour, const std::vector<TopHump> &top_humps, std::vector<BottomCenterHump> &humps)
{
    if (contour.size() < 30)
    {
        humps = {};
        return false;
    }

    if (top_humps.size() != 3)
    {
        VC_THROW_ERROR("Size of the \" top_humps \" are not equal to 3. (size = %zu)", top_humps.size());
    }
    if (humps.size() < 1)
    {
        humps = {};
        return false;
    }

    // 过滤几乎重合的角点
    for (auto it_1 = humps.begin(); it_1 != humps.end(); it_1++)
    {
        for (auto it_2 = it_1 + 1; it_2 != humps.end();)
        {
            if (getDist(it_1->getCenter(), it_2->getCenter()) < rune_fan_hump_param.BOTTOM_CENTER_HUMP_MIN_INTERVAL)
                it_2 = humps.erase(it_2);
            else
                it_2++;
        }
    }

    if (humps.size() < 1) // 若过滤后的突起点数不足 1 个，判断为查找失败
    {
        humps = {};
        return false;
    }

    // 获取在中轴线方向上离顶部中心突起点最远的点 作为底部中心突起点
    Point2f line_center = top_humps[1].getCenter();
    Point2f line_direction = -1 * top_humps[1].getDirection();
    auto bottom_center_hump_it = max_element(humps.begin(), humps.end(),
                                             [&](const BottomCenterHump &h1, const BottomCenterHump &h2)
                                             {
                                                 Point2f v1 = h1.getCenter() - line_center;
                                                 Point2f v2 = h2.getCenter() - line_center;
                                                 return v1.dot(line_direction) < v2.dot(line_direction);
                                             });
    humps = {*bottom_center_hump_it};

    // 判定是否为有效的底部中心突起点
    float Delta_angle = getVectorMinAngle(line_direction, humps[0].getCenter() - line_center, DEG);

    if (Delta_angle > rune_fan_hump_param.BOTTOM_CENTER_HUMP_MAX_DELTA_ANGLE)
    {
        humps = {};
        return false;
    }

    return true;
}
