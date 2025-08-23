#include <opencv2/imgproc.hpp>
#include <unordered_set>

#include "vc/contour_proc/contour_wrapper.hpp"
#include "vc/feature/rune_target_param.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/math/geom_utils.hpp"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;

RuneTargetInactive::RuneTargetInactive(const Point2f center, const std::vector<cv::Point2f> corners)
{
    vector<Point2f> temp_contour{};
    float width = rune_target_param.ACTIVE_DEFAULT_SIDE;
    float height = rune_target_param.ACTIVE_DEFAULT_SIDE;
    Contour_cptr contour = nullptr;
    temp_contour.insert(temp_contour.end(), corners.begin(), corners.end());
    if (temp_contour.size() < 3)
    {
        vector<Point> contours_point(temp_contour.begin(), temp_contour.end());
        contour = ContourWrapper<int>::make_contour(contours_point);
    }
    else
    {
        vector<Point2f> hull;
        convexHull(temp_contour, hull);
        vector<Point> contours_point(hull.begin(), hull.end());
        contour = ContourWrapper<int>::make_contour(contours_point);
    }

    setActiveFlag(false);
    auto &image_info = getImageCache();
    image_info.setCenter(center);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCorners(corners);
    image_info.setContours(vector<Contour_cptr>{contour});
}

/**
 * @brief 未激活神符靶心等级向量判断
 *
 * @param[in] hierarchy 所有的等级向量
 * @param[in] idx 指定的等级向量的下标
 * @return 等级结构是否满足要求
 */
inline bool isHierarchyInactiveTarget(const vector<Vec4i> &hierarchy, size_t idx)
{
    // 没有父轮廓
    if (hierarchy[idx][3] != -1)
        return false;

    // 有多个子轮廓
    if (hierarchy[idx][2] == -1)
        return false;
    int child_num = 1;
    int front_child_idx = hierarchy[hierarchy[idx][2]][1];
    while (front_child_idx != -1)
    {
        child_num += 1;
        front_child_idx = hierarchy[front_child_idx][1];
    }
    int back_child_idx = hierarchy[hierarchy[idx][2]][0];
    while (back_child_idx != -1)
    {
        child_num += 1;
        back_child_idx = hierarchy[back_child_idx][0];
    }
    if (child_num < 4) // 子轮廓数量小于4
        return false;

    return true;
}

/**
 * @brief 通过轮廓特征判断是否为未激活靶心
 *
 * @param[in] contours 所有轮廓
 * @param[in] current_idx 当前轮廓下标
 * @param[in] sub_idxs 所有子轮廓的下标
 */
inline bool isContourInactiveTarget(const vector<Contour_cptr> &contours, const int &outer_idx, const unordered_set<size_t> &sub_idxs)
{
    if (contours[outer_idx]->points().size() < 6)
        return false;

    // -----------------------绝对面积判断-------------------------
    float contour_area = contours[outer_idx]->area();

    if (contour_area < rune_target_param.INACTIVE_MIN_AREA)
    {
        return false;
    }

    // -----------------------边长比例判断-------------------------
    auto fit_ellipse = contours[outer_idx]->fittedEllipse();
    float width = max(fit_ellipse.size.width, fit_ellipse.size.height);
    float height = min(fit_ellipse.size.width, fit_ellipse.size.height);
    float side_ratio = width / height;

    if (side_ratio > rune_target_param.INACTIVE_MAX_SIDE_RATIO)
    {
        return false;
    }
    if (side_ratio < rune_target_param.INACTIVE_MIN_SIDE_RATIO)
    {
        return false;
    }

    // ------------------------面积比例判断----------------------------
    float fit_ellipse_area = width * height * CV_PI / 4;
    float area_ratio = contour_area / fit_ellipse_area;
    if (area_ratio > rune_target_param.INACTIVE_MAX_AREA_RATIO)
    {
        return false;
    }
    if (area_ratio < rune_target_param.INACTIVE_MIN_AREA_RATIO)
    {
        return false;
    }

    // ----------------------周长比例判断-------------------------
    float perimeter = contours[outer_idx]->perimeter();
    float fit_ellipse_perimeter = CV_PI * (3 * (width + height) - sqrt((3 * width + height) * (width + 3 * height)));
    float perimeter_ratio = perimeter / fit_ellipse_perimeter;

    if (perimeter_ratio > rune_target_param.INACTIVE_MAX_PERI_RATIO)
    {
        return false;
    }
    if (perimeter_ratio < rune_target_param.INACTIVE_MIN_PERI_RATIO)
    {
        return false;
    }

    return true;
}

/**
 * @brief 获取等比例变换后的椭圆
 *
 * @param ellipse 椭圆
 * @param ratio 比例
 * @return 等比例变换后的椭圆
 */
inline RotatedRect getRatioEllipse(const RotatedRect &ellipse, float ratio)
{
    float width = ellipse.size.width;
    float height = ellipse.size.height;
    float new_width = width * ratio;
    float new_height = height * ratio;
    return RotatedRect(ellipse.center, Size2f(new_width, new_height), ellipse.angle);
};

/**
 * @brief 判断一个点是否在旋转椭圆内
 *
 * @param point 待检测的点
 * @param ellipse 旋转椭圆的参数，包括中心点、长短轴和旋转角度
 * @return true 如果点在椭圆内
 * @return false 如果点不在椭圆内
 *
 * 该函数首先将点平移到椭圆中心，然后将点旋转到与椭圆对齐的坐标系中，
 * 最后通过椭圆方程判断点是否在椭圆内。
 */
inline bool isPointInEllipse(const Point2f &point, const RotatedRect &ellipse)
{
    Point2f center = ellipse.center;
    Size2f axes = ellipse.size;
    float angle = ellipse.angle;
    // 将点平移到椭圆中心
    Point2f translated_point = point - center;

    // 将点旋转回去
    float cos_angle = cos(-angle * CV_PI / 180.0);
    float sin_angle = sin(-angle * CV_PI / 180.0);
    Point2f rotated_point(
        translated_point.x * cos_angle - translated_point.y * sin_angle,
        translated_point.x * sin_angle + translated_point.y * cos_angle);

    // 检查点是否在椭圆内
    float x_radius = axes.width / 2.0;
    float y_radius = axes.height / 2.0;
    float value = (rotated_point.x * rotated_point.x) / (x_radius * x_radius) +
                  (rotated_point.y * rotated_point.y) / (y_radius * y_radius);

    return value <= 1.0;
}

/**
 * @brief 获取椭圆的矫正矩阵
 *
 * @param[in] ellipse 椭圆
 * @param[out] radius 矫正后的圆的半径的半径
 *
 * @return 矫正矩阵
 */
inline Matx33f getEllipseCorrectionMat(const RotatedRect &ellipse, float &radius)
{
    // 获取旋转矩阵，用于和椭圆对齐
    Mat temp_R = getRotationMatrix2D(ellipse.center, ellipse.angle, 1.0);
    Matx33f R = Matx33f(temp_R.at<double>(0, 0), temp_R.at<double>(0, 1), temp_R.at<double>(0, 2),
                        temp_R.at<double>(1, 0), temp_R.at<double>(1, 1), temp_R.at<double>(1, 2),
                        0, 0, 1);
    Matx33f R_inv = R.inv();

    // 获取椭圆坐标系下的点
    Matx33f T = Matx33f(1, 0, -ellipse.center.x,
                        0, 1, -ellipse.center.y,
                        0, 0, 1);

    // 获取放缩矩阵
    // float len = (ellipse.size.width + ellipse.size.height) / 2.0;
    float len = (ellipse.size.width + ellipse.size.height) * 2.0;
    Matx33f S = Matx33f(len / ellipse.size.width, 0, 0,
                        0, len / ellipse.size.height, 0,
                        0, 0, 1);

    radius = len / 2.0;

    // 平移回原坐标系
    Matx33f T_inv = T.inv();

    // 返回变化矩阵
    return R_inv * T_inv * S * T * R;
}

inline unordered_set<size_t> getAllGapsIdx(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, const int outer_idx, const unordered_set<size_t> &all_sub_idx)
{
    unordered_set<size_t> pending_idx{}; // 待查找的轮廓下标
    pending_idx.insert(all_sub_idx.begin(), all_sub_idx.end());

    // 轮廓层级关系筛选
    for (auto &sub_idx : all_sub_idx)
    {
        if (hierarchy[sub_idx][3] == -1) // 无父轮廓，直接删除
            pending_idx.erase(sub_idx);
        if (hierarchy[sub_idx][3] != outer_idx) // 父轮廓不是当前轮廓，直接删除
            pending_idx.erase(sub_idx);
    }

    // 轮廓点数筛选
    for (auto &sub_idx : all_sub_idx)
    {
        if (contours[sub_idx]->points().size() < 10)
            pending_idx.erase(sub_idx);
    }

    // 位置筛选
    auto outer_fit_ellipse = contours[outer_idx]->fittedEllipse();
    Point2f outer_center = outer_fit_ellipse.center;
    for (auto &sub_idx : all_sub_idx)
    {
        if (pending_idx.find(sub_idx) == pending_idx.end())
            continue;
        if (pointPolygonTest(contours[sub_idx]->points(), outer_center, false) > 0) // 靶心的中心在缺口内，直接删除
            pending_idx.erase(sub_idx);
    }

    // 面积比例筛选
    unordered_map<size_t, float> area_map;
    float current_area = contours[outer_idx]->area();
    for (auto &sub_idx : pending_idx)
    {
        float area = contours[sub_idx]->area();
        area_map[sub_idx] = area;
    }
    for (auto &[sub_idx, area] : area_map)
    {
        float area_ratio = area / current_area;
        if (area_ratio > rune_target_param.GAP_MAX_AREA_RATIO || area_ratio < rune_target_param.GAP_MIN_AREA_RATIO)
            pending_idx.erase(sub_idx);
    }

    // 长宽比例筛选
    for (auto &sub_idx : all_sub_idx)
    {
        if (pending_idx.find(sub_idx) == pending_idx.end())
            continue;
        auto fit_ellipse = contours[sub_idx]->fittedEllipse();
        float width = max(fit_ellipse.size.width, fit_ellipse.size.height);
        float height = min(fit_ellipse.size.width, fit_ellipse.size.height);
        float side_ratio = width / height;
        if (side_ratio > rune_target_param.GAP_MAX_SIDE_RATIO || side_ratio < rune_target_param.GAP_MIN_SIDE_RATIO)
            pending_idx.erase(sub_idx);
    }
    return pending_idx;
}

inline tuple<int, int> getLeftRightIdx(const vector<Point2f> &contour, const Point2f &center)
{
    size_t left_idx = 0;
    size_t right_idx = 0;
    Point2f left_v = static_cast<Point2f>(contour[left_idx]) - center;
    Point2f right_v = static_cast<Point2f>(contour[right_idx]) - center;
    for (size_t i = 0; i < contour.size(); i++)
    {
        Point2f current_v = static_cast<Point2f>(contour[i]) - center;
        if (left_v.cross(current_v) < 0)
        {
            left_v = current_v;
            left_idx = i;
        }
        if (right_v.cross(current_v) > 0)
        {
            right_v = current_v;
            right_idx = i;
        }
    }
    return make_tuple(left_idx, right_idx);
}

inline bool make_gap(const Contour_cptr &contour, const RotatedRect &outer_ellipse, RuneTargetGap &result_gap)
{
    //----------------------【通过矫正后的轮廓的属性判断】----------------------
    // 获取矫正矩阵
    float correction_radius = 0; // 对外轮廓椭圆矫正后得到的正圆的半径。
    Matx33f correction_mat = getEllipseCorrectionMat(outer_ellipse, correction_radius);

    // 获取缺口的矫正轮廓
    Mat contour_mat(3, contour->points().size(), CV_32F);
    float *x_ptr = contour_mat.ptr<float>(0);
    float *y_ptr = contour_mat.ptr<float>(1);
    float *one_ptr = contour_mat.ptr<float>(2);
    const Point *gap_points = contour->points().data();
    for (size_t i = 0; i < static_cast<size_t>(contour_mat.cols); i++)
    {
        *x_ptr++ = gap_points->x;
        *y_ptr++ = gap_points->y;
        *one_ptr++ = 1;
        gap_points++;
    }
    Mat correction_contour_mat = correction_mat * contour_mat;

    vector<Point2f> correction_contour(contour_mat.cols);
    float *correction_x_ptr = correction_contour_mat.ptr<float>(0);
    float *correction_y_ptr = correction_contour_mat.ptr<float>(1);
    Point2f *correction_points = correction_contour.data();
    for (size_t i = 0; i < static_cast<size_t>(correction_contour_mat.cols); i++)
    {
        correction_points->x = *correction_x_ptr++;
        correction_points->y = *correction_y_ptr++;
        correction_points++;
    }

    // 获取矫正轮廓的椭圆
    auto correction_ellipse = fitEllipse(correction_contour);

    // 中心点筛选
    float center_distance = getDist(correction_ellipse.center, outer_ellipse.center); // 中心点距离
    if (center_distance > correction_radius * rune_target_param.GAP_MAX_DISTANCE_RATIO || center_distance < correction_radius * rune_target_param.GAP_MIN_DISTANCE_RATIO)
        return false; // 中心点距离不符合要求

    // 获取顺时针上最靠左的点和最靠右的点的下标
    auto [left_idx, right_idx] = getLeftRightIdx(correction_contour, outer_ellipse.center);

    Point2f left_corner = getUnitVector(static_cast<Point2f>(correction_contour[left_idx]) - outer_ellipse.center) * rune_target_param.GAP_CIRCLE_RADIUS_RATIO * correction_radius + outer_ellipse.center;
    Point2f right_corner = getUnitVector(static_cast<Point2f>(correction_contour[right_idx]) - outer_ellipse.center) * rune_target_param.GAP_CIRCLE_RADIUS_RATIO * correction_radius + outer_ellipse.center;
    Point2f gap_center = getUnitVector(correction_ellipse.center - outer_ellipse.center) * rune_target_param.GAP_CIRCLE_RADIUS_RATIO * correction_radius + outer_ellipse.center;

    // 张角筛选
    float delta_angle = getVectorMinAngle(left_corner - outer_ellipse.center, right_corner - outer_ellipse.center, DEG);
    if (delta_angle > 100 || delta_angle < 60) // 张角不符合要求
        return false;

    // 方向筛选
    Point2f gap_direction = getUnitVector(right_corner - left_corner);
    Point2f center_to_gap = getUnitVector(gap_center - outer_ellipse.center);
    float direction_angle = getVectorMinAngle(gap_direction, center_to_gap, DEG);
    static const float direction_angle_threshold_low = 70;
    static const float direction_angle_threshold_high = 110;
    if (direction_angle < direction_angle_threshold_low || direction_angle > direction_angle_threshold_high) // 方向不符合要求
        return false;

    Matx33f correction_mat_inv = correction_mat.inv();
    auto get_inv_point = [&correction_mat_inv](const Point2f &point) -> Point2f
    {
        Matx31f point_mat(point.x, point.y, 1);
        Matx31f result_mat = correction_mat_inv * point_mat;
        return Point2f(result_mat(0), result_mat(1));
    };

    result_gap.left_corner = get_inv_point(left_corner);
    result_gap.right_corner = get_inv_point(right_corner);
    result_gap.center = get_inv_point(gap_center);

    // 获取缺口中心点

    return true;
}

/**
 * @brief 筛选缺口
 *
 * @param[out] gaps 筛选后的缺口
 * @param[in] all_gaps 所有缺口
 */
inline bool filterGaps(vector<RuneTargetGap> &filter_gaps, const vector<RuneTargetGap> &all_gaps)
{
    filter_gaps = all_gaps;
    return true;
}

/**
 * @brief 获取缺口
 *
 * @param contours 所有轮廓
 * @param hierarchy 等级向量
 * @param outer_idx 最外层轮廓下标
 * @param all_sub_idx 所有子轮廓下标
 *
 * @return true 如果成功获取缺口
 */
inline bool findGaps(vector<RuneTargetGap> &gaps, const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t outer_idx, const unordered_set<size_t> &all_sub_idx)
{
    unordered_set<size_t> pending_idx = getAllGapsIdx(contours, hierarchy, outer_idx, all_sub_idx); // 待查找的轮廓下标

    if (pending_idx.empty())
        return false;

    // 获取椭圆
    auto outer_ellipse = contours[outer_idx]->fittedEllipse();
    Point2f outer_center = outer_ellipse.center;

    // 构造所有缺口
    vector<RuneTargetGap> all_gaps;
    for (auto &gap_idx : pending_idx)
    {
        RuneTargetGap gap;
        if (make_gap(contours[gap_idx], outer_ellipse, gap) == false)
            continue;
        all_gaps.push_back(gap);
    }

    // 筛选缺口
    vector<RuneTargetGap> filter_gaps;
    if (filterGaps(filter_gaps, all_gaps) == false)
        return false;

    if (filter_gaps.empty() || filter_gaps.size() > 4) // 缺口数量不符合要求
        return false;

    gaps = filter_gaps;
    return true;
}

auto RuneTargetInactive::make_feature(const std::vector<Contour_cptr> &contours,
                                      const std::vector<cv::Vec4i> &hierarchy,
                                      size_t idx,
                                      std::unordered_set<size_t> &used_contour_idxs) -> RuneTargetInactive_ptr
{
    // 递归获取所有子轮廓的下标
    unordered_set<size_t> sub_contours_idx{};
    getAllSubContoursIdx(hierarchy, idx, sub_contours_idx);

    // 将当前轮廓下标和其子轮廓下标添加入已使用轮廓下标集合
    used_contour_idxs.insert(idx);
    used_contour_idxs.insert(sub_contours_idx.begin(), sub_contours_idx.end());

    // 通过面积判断是否为未激活靶心
    if (isContourInactiveTarget(contours, idx, sub_contours_idx) == false)
        return nullptr;

    // 获取缺口
    vector<RuneTargetGap> gaps;
    if (findGaps(gaps, contours, hierarchy, idx, sub_contours_idx) == false)
        return nullptr;

    auto &outer_contour = contours[idx];
    vector<Point2f> corners{};
    auto fit_ellipse = outer_contour->fittedEllipse();
    corners.push_back(fit_ellipse.center); // 将中心点作为第一个角点
    // 构建未激活靶心对象，并赋予属性
    auto rune_target = make_shared<RuneTargetInactive>(outer_contour, corners, gaps);
    return rune_target;
}

// 方案二：找出所有未激活靶心
void RuneTargetInactive::find(vector<FeatureNode_ptr> &targets,
                              const vector<Contour_cptr> &contours,
                              const vector<Vec4i> &hierarchy,
                              const unordered_set<size_t> &mask,
                              unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (isHierarchyInactiveTarget(hierarchy, i))
        {
            // DEBUG_RUNE_INFO_("--------------------------------------");
            // DEBUG_RUNE_INFO_("target unactive");

            unordered_set<size_t> temp_used_contour_idxs{};
            auto p_target = make_feature(contours, hierarchy, i, temp_used_contour_idxs);
            if (p_target != nullptr)
            {
                // DEBUG_RUNE_PASS_("target pass");
                // RUNE_TARGET_INACTIVE_CONTOUR_PASS(contours[i], "target inactive : pass");
                targets.push_back(p_target);
                used_contour_idxs[p_target] = temp_used_contour_idxs;
            }
        }
    }
}

bool RuneTargetInactive::correct(const cv::Point2f &rotate_center)
{
    // 判断 rotate_center 是否 为有效点
    if (isnan(rotate_center.x) || isnan(rotate_center.y))
    {
        return false;
    }

    if (getActiveFlag())
    {
    }
    else
    {
        if (correctCorners(rotate_center) == false)
            return false;
        if (correctDirection() == false)
            return false;
    }

    return true;
}

bool RuneTargetInactive::sortedGap(const cv::Point2f &rune_center)
{

    if (getActiveFlag())
    {
        VC_THROW_ERROR("The target is active.");
        return false;
    }
    if (getGapSortedFlag())
    {
        VC_THROW_ERROR("The gap corners have been set.");
        return false;
    }

    auto &gaps = getGaps();
    if (gaps.empty() || gaps.size() > 4) // 缺口数量不符合要求
    {
        return false;
    }

    // 获取由靶心指向神符中心的大致方向
    auto center = getImageCache().getCenter();
    Point2f center_to_RuneCenter = getUnitVector(rune_center - center);
    // 为缺口匹配上对应的位置

    // 以指向神符中心的方向为向下。
    RuneTargetGap *left_top_gap = nullptr;     // 左上缺口
    RuneTargetGap *right_top_gap = nullptr;    // 右上缺口
    RuneTargetGap *right_bottom_gap = nullptr; // 右下缺口
    RuneTargetGap *left_bottom_gap = nullptr;  // 左下缺口

    for (auto &gap : gaps)
    {
        Point2f center_to_gap = getUnitVector(gap.center - center);
        float dot_product = center_to_RuneCenter.x * center_to_gap.x + center_to_RuneCenter.y * center_to_gap.y;
        float cross_product = center_to_RuneCenter.cross(center_to_gap);

        if (dot_product < 0) // 上方缺口
        {
            if (cross_product > 0 && left_top_gap == nullptr)
                left_top_gap = &gap;
            else if (cross_product <= 0 && right_top_gap == nullptr)
                right_top_gap = &gap;
        }
        else // 下方缺口
        {
            if (cross_product > 0 && left_bottom_gap == nullptr)
                left_bottom_gap = &gap;
            else if (cross_product <= 0 && right_bottom_gap == nullptr)
                right_bottom_gap = &gap;
        }
    }

    if (left_top_gap == nullptr && right_top_gap == nullptr && right_bottom_gap == nullptr && left_bottom_gap == nullptr)
    {
        return false;
    }
    // 占位Gap
    const static RuneTargetGap vacancy_gap = {VACANCY_POINT, VACANCY_POINT, VACANCY_POINT, false};

    vector<RuneTargetGap> sorted_gaps(4);
    sorted_gaps[0] = left_top_gap ? *left_top_gap : vacancy_gap;
    sorted_gaps[1] = right_top_gap ? *right_top_gap : vacancy_gap;
    sorted_gaps[2] = right_bottom_gap ? *right_bottom_gap : vacancy_gap;
    sorted_gaps[3] = left_bottom_gap ? *left_bottom_gap : vacancy_gap;

    setLeftTopGap(&sorted_gaps[0]);
    setRightTopGap(&sorted_gaps[1]);
    setRightBottomGap(&sorted_gaps[2]);
    setLeftBottomGap(&sorted_gaps[3]);

    setGaps(sorted_gaps);   // 设置缺口
    setGapSortedFlag(true); // 设置缺口已排序标志

    return true;
}

bool RuneTargetInactive::calcGapCorners(const cv::Point2f &rune_center)
{
    if (getActiveFlag())
    {
        VC_THROW_ERROR("The target is active.");
        return false;
    }
    if (getGapSortedFlag() == false)
    {
        VC_THROW_ERROR("The gap is not sorted.");
    }
    if (getGaps().size() != 4)
    {
        VC_THROW_ERROR("The gap size is not 4.");
        return false;
    }

    // 按照九点模型,设置角点。角点包括：靶心中心点、缺口中心点、相邻缺口角点的中心点。（例如左上缺口的右侧角点和右上缺口的左侧角点的中点）

    auto &gap = getGaps();
    auto &gap_corners = getGapCorners();
    auto &target_corners = getImageCache().getCorners();
    gap_corners.clear();

    // 左上角的缺口的中心点为第一个点
    for (size_t i = 0; i < gap.size(); i++)
    {
        RuneTargetGap *current_gap = &gap[i];
        RuneTargetGap *next_gap = &gap[(i + 1) % 4];
        if (current_gap->is_valid)
        {
            gap_corners.push_back(current_gap->center);
        }
        else
        {
            gap_corners.push_back(VACANCY_POINT);
        }

        if (current_gap->is_valid && next_gap->is_valid)
        {
            gap_corners.push_back((current_gap->right_corner + next_gap->left_corner) / 2.0);
        }
        else
        {
            gap_corners.push_back(VACANCY_POINT);
        }

        // gap_corners.push_back(current_gap ? current_gap->center : VACANCY_POINT);
        // gap_corners.push_back((current_gap && next_gap) ? (current_gap->right_corner + next_gap->left_corner) / 2.0 : VACANCY_POINT);
    }
    if (gap_corners.size() != 8) // 角点数量有问题
    {
        VC_THROW_ERROR("The gap corners size is not 8.");
        return false;
    }

    for (auto &gap_corner : gap_corners)
    {
        if (gap_corner == VACANCY_POINT)
            continue;
        target_corners.push_back(gap_corner);
    }

    return true;
}

/**
 * @brief 未激活靶心的角点修正
 */
bool RuneTargetInactive::correctCorners(const cv::Point2f &rune_center)
{
    if (getActiveFlag())
    {
        VC_THROW_ERROR("The target is active.");
        return false;
    }

    // 设置初始方向
    if (isSetDirection() == false)
    {
        setDirection(getUnitVector(rune_center - getImageCache().getCenter()));
    }

    if (getGapSortedFlag() == false)
    {
        if (this->sortedGap(rune_center) == false) // 对缺口进行排序
            return false;
    }

    // 设置角点
    if (this->calcGapCorners(rune_center) == false)
        return false;

    return true;
}

/**
 * @brief 未激活靶心的方向修正
 *
 * @param[in] rune_center 神符中心
 *
 */
bool RuneTargetInactive::correctDirection()
{
    if (getActiveFlag())
    {
        VC_THROW_ERROR("The target is active.");
        return false;
    }

    if (getGapSortedFlag() == false)
    {
        return false;
    }

    Point2f direction_left{};
    Point2f direction_right{};
    Point2f direction_total{};

    if (getLeftTopGap() && getLeftBottomGap())
        direction_left = getUnitVector(getLeftBottomGap()->center - getLeftTopGap()->center);
    if (getRightTopGap() && getRightBottomGap())
        direction_right = getUnitVector(getRightBottomGap()->center - getRightTopGap()->center);

    if (direction_left == Point2f(0, 0) && direction_right == Point2f(0, 0))
        return false;
    direction_total = (direction_left == Point2f(0, 0)) ? direction_right : (direction_right == Point2f(0, 0)) ? direction_left
                                                                                                               : (direction_left + direction_right) / 2.0;

    setDirection(direction_total); // 设置方向
    return true;
}

auto RuneTargetInactive::getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    vector<Point2f> pnp_points_2d{};
    vector<Point3f> pnp_points_3d{};
    vector<float> pnp_points_weight{};

    // 添加中心点
    pnp_points_2d.push_back(getImageCache().getCenter());
    pnp_points_3d.push_back(Point3f(0, 0, 0));
    pnp_points_weight.push_back(1.0);

    if (getGapSortedFlag() == false)
    {
        return make_tuple(pnp_points_2d, pnp_points_3d, pnp_points_weight);
    }

    auto &gap_corners = getGapCorners();
    if (gap_corners.size() != 8)
    {
        VC_THROW_ERROR("The gap corners size is not 8.");
    }

    // 添加不为空的缺口角点
    for (size_t i = 0; i < gap_corners.size(); i++)
    {
        if (gap_corners[i] != VACANCY_POINT)
        {
            pnp_points_2d.push_back(gap_corners[i]);
            pnp_points_3d.push_back(rune_target_param.GAP_3D[i]);
            pnp_points_weight.push_back(1.0);
        }
    }

    return make_tuple(pnp_points_2d, pnp_points_3d, pnp_points_weight);
}

void RuneTargetInactive::drawFeature(cv::Mat &image, const DrawConfig_cptr &config) const
{
    // 利用半径绘制正圆
    auto draw_circle = [&]() -> bool
    {
        const auto &image_info = this->getImageCache();
        if (!image_info.isSetCorners())
            return false;
        const auto &center = image_info.getCenter();
        auto default_radius = rune_target_draw_param.inactive.point_radius;
        auto circle_color = rune_target_draw_param.inactive.color;
        auto circle_thickness = rune_target_draw_param.inactive.thickness;
        float radius = image_info.isSetHeight() && image_info.isSetWidth() ? std::min(image_info.getHeight(), image_info.getWidth()) / 2.0f : default_radius;
        cv::circle(image, center, radius, circle_color, circle_thickness);
        if (radius > 30)
        {
            cv::circle(image, center, radius * 0.5f, circle_color, circle_thickness);
        }
        if (radius > 50)
        {
            cv::circle(image, center, radius * 0.25f, circle_color, circle_thickness);
        }
        if (radius > 100)
        {
            cv::circle(image, center, radius * 0.125f, circle_color, circle_thickness);
        }

        return true;
    };

    // 绘制椭圆
    auto draw_ellipse = [&]() -> bool
    {
        const auto &image_info = this->getImageCache();
        if (!image_info.isSetContours())
            return false;
        const auto &contours = image_info.getContours();
        if (contours.empty())
            return false;
        if (contours.front()->points().size() < 6)
        {
            return false;
        }
        auto fit_ellipse = contours.front()->fittedEllipse();
        auto circle_color = rune_target_draw_param.inactive.color;
        auto circle_thickness = rune_target_draw_param.inactive.thickness;
        cv::ellipse(image, fit_ellipse, circle_color, circle_thickness);
        cv::circle(image, fit_ellipse.center, 2, circle_color, -1); // 绘制中心点
        return true;
    };

    // 绘制角点
    auto draw_corners = [&]() -> bool
    {
        const auto &image_info = this->getImageCache();
        if (!image_info.isSetCorners())
            return false;
        const auto &corners = image_info.getCorners();
        for (int i = 0; i < static_cast<int>(corners.size()); i++)
        {
            auto color = rune_target_draw_param.inactive.color;
            auto thickness = rune_target_draw_param.inactive.thickness;
            line(image, corners[i], corners[(i + 1) % corners.size()], color, thickness, LINE_AA);
            auto point_radius = rune_target_draw_param.inactive.point_radius;
            circle(image, corners[i], point_radius, color, thickness, LINE_AA);
        }
        for (int i = 0; i < static_cast<int>(corners.size()); i++)
        {
            auto font_scale = rune_target_draw_param.inactive.font_scale;
            auto font_thickness = rune_target_draw_param.inactive.font_thickness;
            auto font_color = rune_target_draw_param.inactive.font_color;
            putText(image, to_string(i), corners[i], FONT_HERSHEY_SIMPLEX, font_scale, font_color, font_thickness, LINE_AA);
        }
        return true;
    };

    do
    {
        // 尝试绘制椭圆，若成功
        if (draw_ellipse())
        {
            break;
        }
        else if (draw_corners())
        {
            break;
        }
        else
        {
            draw_circle();
        }

    } while (0);
}

RuneTargetInactive_ptr RuneTargetInactive::make_feature(const PoseNode &target_to_cam)
{
    vector<Point3f> corners_3d{};
    for (const auto &corner : rune_target_param.GAP_3D)
    {
        corners_3d.emplace_back(corner);
    }

    // 重投影
    vector<Point2f> corners_2d{};
    projectPoints(corners_3d, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, corners_2d);

    Point2f rune_center{};
    vector<Point2f> temp_rune_center{};
    projectPoints(vector<Point3f>{Point3f(0, 0, 0)}, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, temp_rune_center);

    auto result_ptr = make_shared<RuneTargetInactive>(temp_rune_center[0], corners_2d);

    if (result_ptr)
    {
        auto &pose_info = result_ptr->getPoseCache();
        pose_info.getPoseNodes()[CoordFrame::CAMERA] = target_to_cam;
    }
    return result_ptr;
}