#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_fan_param.h"



using namespace std;
using namespace cv;

#define rune_fan_debug_0

#define rune_fan_inactive

#define rune_fan_show

/**
 * @brief 未激活神符扇叶等级向量判断
 *
 * @param[in] contours 所有轮廓
 * @param[in] hierarchy 所有的等级向量
 * @param[in] idx 指定的等级向量的下标
 * @return 等级结构是否满足要求
 *
 */
inline bool isHierarchyInactiveFan(const vector<Contour_ptr> &contours, const vector<Vec4i> &hierarchy, size_t idx)
{
    if (hierarchy[idx][3] != -1) // 无父轮廓
        return false;

    return true;
}

/**
 * @brief 计算小矩形在大矩形边上的投影比值
 */
inline double calculateProjectionRatio(const RotatedRect &rect1, const RotatedRect &rect2)
{
    // 确定大矩形和小矩形
    bool rect1Larger = rect1.size.area() > rect2.size.area();
    const RotatedRect &largeRect = rect1Larger ? rect1 : rect2;
    const RotatedRect &smallRect = rect1Larger ? rect2 : rect1;

    vector<Point2f> largePoints(4);
    largeRect.points(largePoints.data());
    vector<Point2f> smallPoints(4);
    smallRect.points(smallPoints.data());

    // 获取大矩形的两条边的方向
    struct EdgeInfo
    {
        Point2f start;
        Point2f end;
        Point2f direction;
        double length;
    };
    vector<EdgeInfo> largeEdges(2);
    for (int i = 0; i < 2; ++i)
    {
        largeEdges[i].start = largePoints[i];
        largeEdges[i].end = largePoints[(i + 1) % 4];
        largeEdges[i].direction = largeEdges[i].end - largeEdges[i].start;
        largeEdges[i].length = norm(largeEdges[i].direction);
        largeEdges[i].direction /= largeEdges[i].length; // 归一化
    }

    // 将小矩形的两条边分别投影到大矩形的两条边上
    vector<double> projections_x{};
    vector<double> projections_y{};
    for (int i = 0; i < 4; ++i)
    {
        Point2f v_x = smallPoints[i] - largeEdges[0].start;
        Point2f v_y = smallPoints[i] - largeEdges[1].start;
        double projection_x = v_x.dot(largeEdges[0].direction);
        double projection_y = v_y.dot(largeEdges[1].direction);

        // double projection_x = (smallPoints[i] - largeEdges[0].start).dot(largeEdges[0].direction);
        // double projection_y = (smallPoints[i] - largeEdges[1].start).dot(largeEdges[1].direction);
        projections_x.push_back(projection_x);
        projections_y.push_back(projection_y);
        // projections_x.push_back((smallPoints[i] - largeEdges[0].start).dot(largeEdges[0].direction));
        // projections_y.push_back((smallPoints[i] - largeEdges[1].start).dot(largeEdges[1].direction));
    }
    struct RangeDouble
    {
        double min;
        double max;
        double size() const
        {
            return max - min;
        }
    };
    RangeDouble x_range = {*min_element(projections_x.begin(), projections_x.end()), *max_element(projections_x.begin(), projections_x.end())};
    RangeDouble y_range = {*min_element(projections_y.begin(), projections_y.end()), *max_element(projections_y.begin(), projections_y.end())};
    RangeDouble x_raw_range = {0, largeEdges[0].length};
    RangeDouble y_raw_range = {0, largeEdges[1].length};
    // 取区间交集，并将交集最大的方向的交集比例作为投影比值
    RangeDouble x_intersection = {max(x_range.min, x_raw_range.min), min(x_range.max, x_raw_range.max)};
    RangeDouble y_intersection = {max(y_range.min, y_raw_range.min), min(y_range.max, y_raw_range.max)};
    float x_intersection_ratio = x_intersection.size() / x_raw_range.size();
    float y_intersection_ratio = y_intersection.size() / y_raw_range.size();

    return x_intersection_ratio > y_intersection_ratio ? x_range.size() / x_raw_range.size() : y_intersection.size() / y_raw_range.size();

    // Range x_range = Range(*min_element(projections_x.begin(), projections_x.end()), *max_element(projections_x.begin(), projections_x.end()));
    // Range y_range = Range(*min_element(projections_y.begin(), projections_y.end()), *max_element(projections_y.begin(), projections_y.end()));
    // Range x_raw_range = Range(0, largeEdges[0].length);
    // Range y_raw_range = Range(0, largeEdges[1].length);

    // // 取区间交集，并将交集最大的方向的交集比例作为投影比值
    // Range x_intersection = x_range & x_raw_range;
    // Range y_intersection = y_range & y_raw_range;
    // return x_intersection.size() > y_intersection.size() ? x_intersection.size() / x_raw_range.size() : y_intersection.size() / y_raw_range.size();
}

// inline double calculateProjectionRatio(const RotatedRect &rect1, const RotatedRect &rect2)
// {
//     // 确定大矩形和小矩形
//     bool rect1Larger = rect1.size.area() > rect2.size.area();
//     const RotatedRect &largeRect = rect1Larger ? rect1 : rect2;
//     const RotatedRect &smallRect = rect1Larger ? rect2 : rect1;

//     // 边结构
//     struct EdgeInfo
//     {
//         Point2f start;
//         Point2f end;
//         Point2f direction;
//         double length;
//     };

//     // 获取大矩形的四条边
//     vector<EdgeInfo> largeEdges(4);
//     vector<Point2f> largePoints(4);
//     largeRect.points(largePoints.data());
//     for (int i = 0; i < 4; ++i)
//     {
//         largeEdges[i].start = largePoints[i];
//         largeEdges[i].end = largePoints[(i + 1) % 4];
//     }

//     // 计算大矩形的边的方向和长度
//     for (int i = 0; i < 4; ++i)
//     {
//         largeEdges[i].direction = largeEdges[i].end - largeEdges[i].start;
//         largeEdges[i].length = norm(largeEdges[i].direction);
//         largeEdges[i].direction /= largeEdges[i].length; // 归一化
//     }

//     // 找出面向小矩形的边
//     vector<EdgeInfo> candidateEdges{};
//     Point2f to_small = smallRect.center - largeRect.center;
//     for (int i = 0; i < 4; i++)
//     {
//         Point2f last_direction = largeEdges[(i + 3) % 4].direction;
//         Point2f current_direction = largeEdges[i].direction;
//         if (last_direction.cross(to_small) * current_direction.cross(to_small) < 0)
//         {
//             candidateEdges.push_back(largeEdges[i]);
//         }
//     }
// }

/**
 * @brief 轮廓过滤
 *
 * @note 尝试删除远离未激活靶心那端的轮廓，防止神符中心轮廓被误识别成未激活靶心的一部分
 */
inline void filterFanContours(const std::vector<Contour_ptr> &in_contours, std::vector<Contour_ptr> &out_contours, vector<FeatureNode_ptr> &inactive_targets)
{
    // 0. 判空
    if (in_contours.size() < 2 || inactive_targets.empty()) // 当轮廓数量为1时，不进行过滤
        return;

    // 1. 用于参考的未激活靶心
    FeatureNode_ptr ref_target = nullptr;
    if (inactive_targets.size() > 1) // 多个靶心，选择最近的那个
    {
        // 1. 设置所有轮廓的权重。
        unordered_map<Contour_ptr, double> contour_weights{};
        double area_sum = 0;
        for (auto &contour : in_contours)
            area_sum += contour->area();

        if (area_sum == 0)
            return;

        for (auto &contour : in_contours)
        {
            contour_weights[contour] = contour->area() / area_sum;
        }

        // 2. 计算所有轮廓的重心
        Point2f all_contours_center{0, 0};
        for (auto &contour : in_contours)
            all_contours_center += static_cast<Point2f>(contour->center()) * contour_weights[contour];

        // 3. 获取距离重心最近的未激活靶心
        auto near_target = *min_element(inactive_targets.begin(), inactive_targets.end(), [&](const FeatureNode_ptr &a, const FeatureNode_ptr &b)
                                        { return norm(a->getImageCache().getCenter() - all_contours_center) < norm(b->getImageCache().getCenter() - all_contours_center); });
    }
    else
    {
        ref_target = inactive_targets.front();
    }

    // 2. 获取最远的轮廓
    auto far_contour = *max_element(in_contours.begin(), in_contours.end(), [&](const Contour_ptr &a, const Contour_ptr &b)
                                    { return norm(static_cast<Point2f>(a->center()) - ref_target->getImageCache().getCenter()) < norm(static_cast<Point2f>(b->center()) - ref_target->getImageCache().getCenter()); });

    // 3. 删除远离靶心的轮廓
    out_contours = in_contours;
    out_contours.erase(remove_if(out_contours.begin(), out_contours.end(), [&](const Contour_ptr &contour)
                                 { return contour == far_contour; }),
                       out_contours.end());
}

void RuneFanInactive::find(std::vector<FeatureNode_ptr> &fans,
                 const std::vector<Contour_ptr> &contours,
                 const std::vector<cv::Vec4i> &hierarchy,
                 const std::unordered_set<size_t> &mask,
                 std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs,
                 const std::vector<FeatureNode_ptr> &inactive_targets)
{
    // SRVL_Error(SRVL_StsBadArg, "The number of 1 in \"order\" must be less than or equal to the number of nodes.");
    if (contours.empty() || hierarchy.empty())
    {
        VC_THROW_ERROR("The contours or hierarchy is empty. to find inactive fans.");
    }
    if (contours.size() != hierarchy.size())
    {
        VC_THROW_ERROR("The contours size is not equal to hierarchy size. to find inactive fans.");
    }
    fans.clear();

    // 获取查找范围集合
    vector<size_t> find_idxs{};
    for (auto i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (isHierarchyInactiveFan(contours, hierarchy, i) == false)
            continue;
        find_idxs.push_back(i);
    }

    vector<vector<size_t>> contours_group{};
    for (const auto &idx : find_idxs)
    {
        contours_group.emplace_back(vector<size_t>(1, idx)); // 初始化每个轮廓为一个组
    }

    // matchFanContours(contours, find_idxs, contours_group); // 获取匹配好的轮廓组

    for (auto &group : contours_group)
    {
        vector<Contour_ptr> temp_contours{};
        for (auto &idx : group)
            temp_contours.push_back(contours[idx]);
        auto p_fan = RuneFanInactive::make_feature(temp_contours);
        if (p_fan)
        {
            fans.push_back(p_fan);
            used_contour_idxs.insert({p_fan, {group.begin(), group.end()}});
        }
    }
}

// ------------------------【未激活扇叶】------------------------
RuneFanInactive_ptr RuneFanInactive::make_feature(const vector<Contour_ptr> &contours)
{
    if (contours.empty())
        return nullptr;

    // 获取凸包轮廓
    vector<Point> temp_contour{};
    for (auto &contour : contours)
        temp_contour.insert(temp_contour.end(), contour->points().begin(), contour->points().end());
    vector<Point> hull_contour_temp{};
    convexHull(temp_contour, hull_contour_temp);
    Contour_ptr hull_contour = ContourWrapper<int>::make_contour(hull_contour_temp);
    double hull_area = hull_contour->area();
    RotatedRect rotated_rect = hull_contour->minAreaRect();

    // 面积判断
    double rect_area = rotated_rect.size.area();
    if (hull_area < rune_fan_param.INACTIVE_MIN_AREA)
    {
        return nullptr;
    }
    if (hull_area > rune_fan_param.INACTIVE_MAX_AREA)
    {
        return nullptr;
    }

    // 矩形度判断
    double area_ratio = hull_area / rotated_rect.size.area();
    if (area_ratio < rune_fan_param.INACTIVE_MIN_AREA_RATIO)
    {
        return nullptr;
    }

    // 边长比例判断
    double width = max(rotated_rect.size.width, rotated_rect.size.height);
    double height = min(rotated_rect.size.width, rotated_rect.size.height);
    double side_ratio = width / height;
    if (side_ratio < rune_fan_param.INACTIVE_MIN_SIDE_RATIO)
    {
        return nullptr;
    }
    if (side_ratio > rune_fan_param.INACTIVE_MAX_SIDE_RATIO)
    {
        return nullptr;
    }

    return make_shared<RuneFanInactive>(hull_contour, contours, rotated_rect);
}

// RuneFanInactive_ptr RuneFanInactive::make_feature(const Point2f &top_left,
//                                               const Point2f &top_right,
//                                               const Point2f &bottom_right,
//                                               const Point2f &bottom_left)
// {
//     // 若点发生重合，返回空指针
//     if (top_left == top_right || top_left == bottom_right || top_left == bottom_left || top_right == bottom_right || top_right == bottom_left || bottom_right == bottom_left)
//         return nullptr;

//     return make_shared<RuneFan>(top_left, top_right, bottom_right, bottom_left);
// }

// ------------------------【构造函数】------------------------
RuneFanInactive::RuneFanInactive(const Contour_ptr hull_contour, const vector<Contour_ptr> &arrow_contours, const RotatedRect &rotated_rect)
{
    auto width = min(rotated_rect.size.width, rotated_rect.size.height);
    auto height = max(rotated_rect.size.width, rotated_rect.size.height);
    auto center = rotated_rect.center;
    vector<Point2f> corners(4);
    rotated_rect.points(corners.data());
    // 取最小外接矩形的长边作为方向
    Point2f direction_temp{};
    if (getDist(corners[0], corners[1]) > getDist(corners[0], corners[3]))
        direction_temp = corners[0] - corners[1];
    else
        direction_temp = corners[0] - corners[3];

    // 设置基本属性
    setArrowContours(arrow_contours);
    setActiveFlag(false);
    setDirection(getUnitVector(direction_temp)); // 扇叶的方向指向神符中心
    setRotatedRect(rotated_rect);

    // 设置图像属性
    auto& image_info = getImageCache();
    image_info.setContours(vector<Contour_ptr>{hull_contour});
    image_info.setCorners(corners);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCenter(center);
}

RuneFanInactive::RuneFanInactive(const Point2f &top_left,
                                 const Point2f &top_right,
                                 const Point2f &bottom_right,
                                 const Point2f &bottom_left)
{
    Point2f top_center = (top_left + top_right) / 2;
    Point2f bottom_center = (bottom_left + bottom_right) / 2;
    Point2f left_center = (top_left + bottom_left) / 2;
    Point2f right_center = (top_right + bottom_right) / 2;

    auto center = (top_center + bottom_center) / 2;
    auto contour = ContourWrapper<int>::make_contour({static_cast<Point>(top_left), static_cast<Point>(top_right), static_cast<Point>(bottom_right), static_cast<Point>(bottom_left)});
    auto corners = {top_left, top_right, bottom_right, bottom_left};
    // __rotated_rect = getContour()->minAreaRect();
    // __angle = __rotated_rect.angle;
    auto width = getDist(left_center, right_center);
    auto height = getDist(top_center, bottom_center);
    auto is_active = false;

    setDirection(getUnitVector(bottom_center - top_center)); // 扇叶的方向指向神符中心
    setRotatedRect(contour->minAreaRect());

    auto& image_info = getImageCache();
    image_info.setContours(vector<Contour_ptr>{contour});
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCenter(center);
    image_info.setCorners(corners);
}

// ------------------------【未激活扇叶的方向矫正】------------------------
bool RuneFanInactive::correctDirection(FeatureNode_ptr &fan, const cv::Point2f &correct_center)
{
    auto rune_fan = RuneFanInactive::cast(fan);
    if (rune_fan == nullptr)
        return false;
    if (rune_fan->getActiveFlag())
        return false;
    if (rune_fan->getDirection() == cv::Point2f(0, 0))
        return false;
    if (correct_center == cv::Point2f(0, 0))
        return false;
    vector<Point2f> rect_points(4);
    rune_fan->getRotatedRect().points(rect_points.data());
    Point2f direction{};
    if (getDist(rect_points[0], rect_points[1]) > getDist(rect_points[0], rect_points[3]))
        direction = rect_points[0] - rect_points[1];
    else
        direction = rect_points[0] - rect_points[3];

    // 参考方向
    Point2f reference_direction = getUnitVector(correct_center - rune_fan->getImageCache().getCenter());
    if (direction.dot(reference_direction) < 0)
        direction = -direction;
    rune_fan->setDirection(getUnitVector(direction));

    return true;
}

// ------------------------【未激活扇叶的角点矫正】------------------------
bool RuneFanInactive::correctCorners(FeatureNode_ptr &fan)
{
    auto rune_fan = RuneFanInactive::cast(fan);
    if (rune_fan == nullptr)
        return false;
    if (rune_fan->getActiveFlag())
        return false;
    if (rune_fan->getDirection() == cv::Point2f(0, 0))
        return false;

    vector<Point2f> rect_points(4);
    rune_fan->getRotatedRect().points(rect_points.data());
    vector<Point2f> temp_corners(rect_points.begin(), rect_points.end());
    Point2f dirction = rune_fan->getDirection();
    Point2f center = rune_fan->getRotatedRect().center;
    // 按照在方向上的投影排序
    sort(temp_corners.begin(), temp_corners.end(), [&](Point2f p1, Point2f p2)
         {
        Point2f v1 = p1 - center;
        Point2f v2 = p2 - center;
        return v1.dot(dirction) < v2.dot(dirction); });
    // 根据叉积判断顺序
    Point2f v0 = temp_corners[0] - center;
    Point2f v1 = temp_corners[1] - center;
    Point2f v2 = temp_corners[2] - center;
    Point2f v3 = temp_corners[3] - center;
    if (v0.cross(v1) < 0)
        swap(temp_corners[0], temp_corners[1]);
    if (v2.cross(v3) < 0)
        swap(temp_corners[2], temp_corners[3]);

    rune_fan->getImageCache().setCorners(temp_corners);
    return true;
}

Contour_ptr RuneFanInactive::getEndArrowContour(FeatureNode_ptr &inactive_fan, const vector<FeatureNode_ptr> &inactive_targets)
{
    // 0. 判空
    if (inactive_targets.empty())
        return nullptr;
    if (inactive_fan == nullptr)
        return nullptr;
    auto fan = RuneFanInactive::cast(inactive_fan);
    if (fan->getArrowContours().size() < 2) // 轮廓数量为1时，不进行过滤
        return nullptr;

    // 1. 用于参考的未激活靶心
    FeatureNode_ptr ref_target = nullptr;
    vector<Contour_ptr> arrow_contours = fan->getArrowContours();
    if (inactive_targets.size() == 1)
    {
        ref_target = inactive_targets.front();
    }
    else
    {
        // 1. 设置所有轮廓的权重。
        unordered_map<Contour_ptr, double> contour_weights{};
        double area_sum = 0;
        for (auto &contour : arrow_contours)
            area_sum += contour->area();

        if (area_sum == 0)
            return nullptr;

        for (auto &contour : arrow_contours)
        {
            contour_weights[contour] = contour->area() / area_sum;
        }

        // 2. 计算所有轮廓的重心
        Point2f all_contours_center{0, 0};
        for (auto &contour : arrow_contours)
            all_contours_center += static_cast<Point2f>(contour->center()) * contour_weights[contour];

        // 3. 获取距离重心最近的未激活靶心
        auto near_target = *min_element(inactive_targets.begin(), inactive_targets.end(), [&](const FeatureNode_ptr &a, const FeatureNode_ptr &b)
                                        { return norm(a->getImageCache().getCenter() - all_contours_center) < norm(b->getImageCache().getCenter() - all_contours_center); });
        ref_target = near_target;
    }

    // 2. 获取最远的轮廓
    auto far_contour = *max_element(arrow_contours.begin(), arrow_contours.end(), [&](const Contour_ptr &a, const Contour_ptr &b)
                                    { return norm(static_cast<Point2f>(a->center()) - ref_target->getImageCache().getCenter()) < norm(static_cast<Point2f>(b->center()) - ref_target->getImageCache().getCenter()); });

    // 3. 获取剩余轮廓
    vector<Contour_ptr> rest_contours = arrow_contours;
    rest_contours.erase(remove_if(rest_contours.begin(), rest_contours.end(), [&](const Contour_ptr &contour)
                                  { return contour == far_contour; }),
                        rest_contours.end());

    // 4. 判断剩余轮廓能否构造灯臂特征
    auto try_make_fan = RuneFanInactive::make_feature(rest_contours);
    if (try_make_fan == nullptr)
    {
        return nullptr;
    }

    // 5. 替换
    fan = try_make_fan;
    inactive_fan = static_cast<FeatureNode_ptr>(fan);

    // 6. 返回
    return far_contour;
}