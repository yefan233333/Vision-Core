
#include <numeric>
#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_hump_param.h"
#include "vc/math/geom_utils.hpp"

using namespace std;
using namespace cv;

#define rune_fan_debug_0

SideHump::SideHump(const cv::Point2f &_direction, const cv::Point2f &_center)
{
    direction = _direction;
    center = _center;
}

std::vector<SideHump> SideHump::getSideHumps(const std::vector<cv::Point> &contour_plus,
                                             const cv::Point2f &contour_center,
                                             const std::vector<TopHump> &top_humps,
                                             const std::vector<BottomCenterHump> &bottom_center_humps,
                                             std::vector<std::tuple<Line, Line>> &line_pairs)
{
    if (line_pairs.size() < 2)
        return {};
    vector<SideHump> humps;
    getAllHumps(top_humps, line_pairs, humps);
    if (humps.size() < 2)
        return {};

    for (auto &hump : humps)
    {
        SideHump::setVertex(hump, contour_plus);
    }

    filter(humps);
    if (humps.size() < 2)
        return {};

    // 过滤收集到的突起点
    vector<tuple<vector<SideHump>, double>> hump_groups{};

    // 尝试构造突起点
    for (size_t i = 0; i < humps.size() - 1; ++i)
    {
        for (size_t j = i + 1; j < humps.size(); ++j)
        {
            vector<SideHump> temp_humps = {humps[i], humps[j]}; // 用于分类的突起点
            double delta = 0;
            if (SideHump::make_SideHumps(temp_humps, top_humps, contour_center, delta))
            {
                hump_groups.emplace_back(temp_humps, delta);
            }
        }
    }
    if (hump_groups.empty())
    {
        return {};
    }
    // 选择误差最小的突起点组
    auto [best_humps, delta] = *min_element(hump_groups.begin(), hump_groups.end(), [](const auto &a, const auto &b)
                                            { return get<1>(a) < get<1>(b); });

    return best_humps;
}

bool SideHump::getAllHumps(const std::vector<TopHump> &top_humps, const std::vector<std::tuple<Line, Line>> &line_pairs, std::vector<SideHump> &humps)
{
    humps.clear();
    if (top_humps.size() != 3)
    {
        VC_THROW_ERROR("Size of the \" top_humps \" are not equal to 3. (size = %zu)", top_humps.size());
        return false;
    }

    // 设置待查找的线段对的下标
    unordered_set<int> line_pair_idx_set;
    for (int i = 0; i < line_pairs.size(); i++)
    {
        line_pair_idx_set.insert(i);
    }

    // 删除已经被使用的线段对
    for (auto &top_hump : top_humps)
    {
        line_pair_idx_set.erase(top_hump.getLinePairIdx());
    }

    Point2f l_center = top_humps[0].getCenter();
    Point2f l_direction = top_humps[0].getDirection();
    Point2f r_center = top_humps[2].getCenter();
    Point2f r_direction = top_humps[2].getDirection();

    auto isSideHump = [&](const Point2f &top_center, const Point2f &top_dir, const Point2f &center, const Point2f &dir)
    {
        if (getVectorMinAngle(top_dir, dir, DEG) > rune_fan_hump_param.SIDE_HUMP_MAX_ANGLE_DELTA)
            return false;
        Point2f v1 = top_dir;
        Point2f v2 = top_center - center;
        Point2f v3 = dir;
        if (v1.cross(v2) * v2.cross(v3) < 0) // 三个向量不同向
            return false;
        // cout << endl;
        if (getVectorMinAngle(v1, v2, DEG) > rune_fan_hump_param.SIDE_HUMP_MAX_ANGLE_DELTA / 2.0 || getVectorMinAngle(v2, v3, DEG) > rune_fan_hump_param.SIDE_HUMP_MAX_ANGLE_DELTA / 2.0)
        {
            return false;
        }
        return true;
    };

    for (auto &idx : line_pair_idx_set)
    {
        auto &[up_line, down_line] = line_pairs[idx];
        Point2f center = (up_line.center + down_line.center) / 2.0;
        Point2f direction(cos(deg2rad(up_line.angle)), sin(deg2rad(up_line.angle))); // 以上升线段的角度为方向

        // 选择距离最近的顶部突起点
        float to_l_distance = getDist(center, l_center);
        float to_r_distance = getDist(center, r_center);
        if (to_l_distance < to_r_distance)
        {
            if (isSideHump(l_center, l_direction, center, direction))
            {
                Point2f hump_center = getLineIntersection(l_center, l_center + l_direction, center, center + direction);
                humps.emplace_back(direction, hump_center);
            }
        }
        else
        {
            if (isSideHump(r_center, r_direction, center, direction))
            {
                Point2f hump_center = getLineIntersection(r_center, r_center + r_direction, center, center + direction);
                humps.emplace_back(direction, hump_center);
            }
        }
    }
    if (humps.size() < 2)
        return false;

    return true;
}

bool SideHump::setVertex(SideHump &hump, const std::vector<cv::Point> &contour_plus)
{
    hump.vertex = hump.center;
    return true;
}

bool SideHump::filter(std::vector<SideHump> &humps)
{
    // 过滤几乎重合的突起点
    for (auto it_1 = humps.begin(); it_1 != humps.end(); it_1++)
    {
        for (auto it_2 = it_1 + 1; it_2 != humps.end();)
        {
            if (getDist(it_1->getCenter(), it_2->getCenter()) < 10)
                it_2 = humps.erase(it_2);
            else
                it_2++;
        }
    }
    return true;
}
bool SideHump::make_SideHumps(std::vector<SideHump> &humps, const std::vector<TopHump> &top_humps, const cv::Point2f &contour_center, double &delta)
{
    if (humps.size() != 2)
    {
        VC_THROW_ERROR("Size of the \" humps \" are not equal to 2. (size = %zu)", humps.size());
        return false;
    }

    Point2f fan_direcion = top_humps[1].getDirection();
    Point2f fan_center = top_humps[1].getCenter();

    double cross_0 = (humps[0].getCenter() - fan_center).cross(fan_direcion);
    double cross_1 = (humps[1].getCenter() - fan_center).cross(fan_direcion);

    if (cross_0 * cross_1 > 0)
    {
        return false; // 两个突起点不在扇叶的两侧
    }
    // 确定左右
    SideHump left_hump = humps[0];
    SideHump right_hump = humps[1];
    if (cross_0 < 0)
    {
        left_hump = humps[1];
        right_hump = humps[0];
    }
    else
    {
        left_hump = humps[0];
        right_hump = humps[1];
    }
    humps = {left_hump, right_hump};
    delta = 0;

    return true;
}
