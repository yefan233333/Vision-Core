#include <numeric>
#include "vc/feature/rune_fan_active.h"
#include "vc/feature/rune_fan_param.h"
#include "vc/feature/rune_fan_hump_param.h"
#include "vc/core/debug_tools.h"

using namespace std;
using namespace cv;

#define RUNE_FAN_DEBUG 1


/**
 * @brief 已激活神符扇叶等级向量判断
 *
 * @param[in] contours 所有轮廓
 * @param[in] hierarchy 所有的等级向量
 * @param[in] idx 指定的等级向量的下标
 * @return 等级结构是否满足要求
 */
inline bool isHierarchyActiveFan(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t idx)
{
    if (hierarchy[idx][3] != -1) // 无父轮廓
        return false;

    return true;
}

void RuneFanActive::find(std::vector<FeatureNode_ptr> &fans,
                         const std::vector<Contour_cptr> &contours,
                         const std::vector<cv::Vec4i> &hierarchy,
                         const std::unordered_set<size_t> &mask,
                         std::unordered_map<FeatureNode_ptr, unordered_set<size_t>> &used_contour_idxs)
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (contours[i]->points().size() < 6)
            continue;
        if (isHierarchyActiveFan(contours, hierarchy, i))
        {
            auto p_fan = RuneFanActive::make_feature(contours[i]);
            if (p_fan != nullptr)
            {
                fans.push_back(p_fan);
                used_contour_idxs[p_fan] = {i};
            }
        }
    }
}

// ------------------------【已激活扇叶】------------------------
shared_ptr<RuneFanActive> RuneFanActive::make_feature(const Contour_cptr &contour)
{

    // 获取扇叶的最小面积矩形
    RotatedRect rotated_rect = contour->minAreaRect();
    // 轮廓面积
    float contour_area = contour->area();
    // 轮廓周长
    float perimeter = contour->perimeter();

    // ------------------------ 边长比例判断 ------------------------
    double width = max(rotated_rect.size.width, rotated_rect.size.height);
    double height = min(rotated_rect.size.width, rotated_rect.size.height);
    double side_ratio = width / height;
    if (side_ratio > rune_fan_param.ACTIVE_MAX_SIDE_RATIO)
    {
        return nullptr;
    }

    // ---------------------- 面积比例判断 ----------------------
    double rect_area = rotated_rect.size.area();
    double area_ratio = contour_area / rect_area;
    if (area_ratio > rune_fan_param.ACTIVE_MAX_AREA_RATIO)
    {
        return nullptr;
    }
    if (area_ratio < rune_fan_param.ACTIVE_MIN_AREA_RATIO)
    {
        return nullptr;
    }

    // -------------------- 面积周长比例判断 --------------------
    double area_perimeter_ratio = contour_area / (perimeter * perimeter);
    if (area_perimeter_ratio > rune_fan_param.ACTIVE_MAX_AREA_PERIMETER_RATIO)
    {
        return nullptr;
    }
    if (area_perimeter_ratio < rune_fan_param.ACTIVE_MIN_AREA_PERIMETER_RATIO)
    {
        return nullptr;
    }

    // ---------------------- 绝对面积判断 ----------------------
    if (contour_area < rune_fan_param.ACTIVE_MIN_AREA)
    {
        return nullptr;
    }
    if (contour_area > rune_fan_param.ACTIVE_MAX_AREA)
    {
        return nullptr;
    }

    vector<Point2f> top_hump_corners;
    vector<Point2f> bottom_center_hump_corners;
    vector<Point2f> side_hump_corners;
    vector<Point2f> bottom_side_hump_corners;
    if (getActiveFunCorners(contour,
                            top_hump_corners,
                            bottom_center_hump_corners,
                            side_hump_corners,
                            bottom_side_hump_corners) == false)
    {
        return nullptr;
    }

    return make_shared<RuneFanActive>(contour,
                                      rotated_rect,
                                      top_hump_corners,
                                      bottom_center_hump_corners,
                                      side_hump_corners,
                                      bottom_side_hump_corners);
    return nullptr;
}

RuneFanActive_ptr RuneFanActive::make_feature(const vector<Point2f> &top_corners,
                                              const vector<Point2f> &bottom_center_corners,
                                              const vector<Point2f> &side_corners,
                                              const vector<Point2f> &bottom_side_corners)
{
    vector<Point2f> points;
    points.insert(points.end(), top_corners.begin(), top_corners.end());
    points.insert(points.end(), bottom_center_corners.begin(), bottom_center_corners.end());
    points.insert(points.end(), side_corners.begin(), side_corners.end());
    points.insert(points.end(), bottom_side_corners.begin(), bottom_side_corners.end());
    vector<Point2f> hull;
    convexHull(points, hull);
    auto contour = ContourWrapper<int>::make_contour(vector<Point>(hull.begin(), hull.end()));
    RotatedRect rotated_rect = contour->minAreaRect();

    return make_shared<RuneFanActive>(contour, rotated_rect, top_corners, bottom_center_corners, side_corners, bottom_side_corners);
}

/**
 * @brief 轮廓滤波
 * @param[in] contour_plus 加长后的轮廓
 * @return 轮廓角度数组
 */
cv::Mat RuneFanActive::getAngles(const vector<Point> &contour_plus)
{
    // 将轮廓点转换为 存入 Mat 中
    Mat contours_mat(2, contour_plus.size(), CV_32F);
    // 通过指针填充数据
    float *mat_p_x = contours_mat.ptr<float>(0);
    float *mat_p_y = contours_mat.ptr<float>(1);
    const Point *p = contour_plus.data();
    for (size_t i = 0; i < static_cast<size_t>(contour_plus.size()); i++)
    {
        *mat_p_x++ = static_cast<float>(p->x);
        *mat_p_y++ = static_cast<float>(p->y);
        p++;
    }
    // 使用滤波器获取方向数组
    Mat directions_mat(contours_mat.size(), CV_32F);
    Mat direction_kernel = (Mat_<float>(1, 2) << -1, 1);
    filter2D(contours_mat, directions_mat, -1, direction_kernel, Point(0, 0), 0, BORDER_DEFAULT);

    // 计算滤波器长度，确保其为奇数且在指定范围内
    int filter_len = static_cast<int>(rune_fan_hump_param.TOP_HUMP_FILTER_LEN_RATIO * contour_plus.size());
    filter_len = max(17, min(filter_len | 1, 101)); // 确保滤波器长度至少为7，最大为101，且为奇数

    Mat kernel = getGaussianKernel(filter_len, rune_fan_hump_param.TOP_HUMP_FILTER_SIGMA, CV_32F).t();

    normalize(kernel, kernel, 1, 0, NORM_L1);
    filter2D(directions_mat, directions_mat, -1, kernel, Point(-1, -1), 0, BORDER_DEFAULT);

    auto calculate_angles = [](const Mat &directions_mat, Mat &angle_array)
    {
        auto p_x = directions_mat.ptr<float>(0);
        auto p_y = directions_mat.ptr<float>(1);
        auto angle_p = angle_array.ptr<float>(0);
        int n = 0;
        int last_angle = 0;
        for (size_t i = 0; i < static_cast<size_t>(directions_mat.cols); i++)
        {
            float angle = rad2deg(atan2(*p_y++, *p_x++));
            if (angle - last_angle < -180)
                n++;
            else if (angle - last_angle > 180)
                n--;
            last_angle = angle;
            *angle_p++ = angle + n * 360;
        }
    };
    Mat angles(1, directions_mat.cols, CV_32FC1);
    calculate_angles(directions_mat, angles);
    return angles;
}

Mat RuneFanActive::getGradient(const Mat &angles_mat)
{
    Mat gradient_mat(1, angles_mat.cols, CV_32F);
    Mat kernel = (Mat_<float>(1, 3) << -1, 0, 1);
    filter2D(angles_mat, gradient_mat, -1, kernel, Point(-1, -1), 0, BORDER_DEFAULT);
    return gradient_mat;
}

// 获取所有的线段
inline bool getAllLine(const vector<Point> &contour_plus, const Mat &angles_mat, const Mat &gradient, std::vector<Line> &lines)
{
    if (angles_mat.cols != gradient.cols)
    {
        VC_THROW_ERROR("The cols of angles and gradient must be equal");
    }
    // 获取所有的线段
    vector<tuple<size_t, float, size_t, float>> temp_lines{}; // 线段的 起始索引，起始点角度，结束索引，结束点角度
    const auto *p_angles = angles_mat.ptr<float>(0);
    const auto *p_gradient = gradient.ptr<float>(0);
    bool is_in_line = false;
    for (size_t i = 0; i < angles_mat.cols; i++)
    {
        const float &angle = *p_angles;
        const float &gradent = *p_gradient;
        // 状态判断切换
        if (abs(gradent) <= 3)
        {
            if (!is_in_line)
            {
                is_in_line = true;
                temp_lines.emplace_back(make_tuple(i, angle, i, angle));
            }
            else
            {
                auto &[start_idx, start_angle, end_idx, end_angle] = temp_lines.back();
                end_idx = i;
                end_angle = angle;
            }
        }
        else
        {
            if (is_in_line)
            {
                is_in_line = false;
            }
        }

        p_angles++;
        p_gradient++;
    }
    // 线段合并
    for (size_t i = 1; i < temp_lines.size(); i++)
    {
        auto &[prev_start_idx, prev_start_angle, prev_end_idx, prev_end_angle] = temp_lines[i - 1];
        auto &[next_start_idx, next_start_angle, next_end_idx, next_end_angle] = temp_lines[i];
        if (next_start_idx - prev_end_idx < 20 && abs(next_end_angle - prev_start_angle) < 5)
        {
            prev_end_idx = next_end_idx;
            prev_end_angle = next_end_angle;
            temp_lines.erase(temp_lines.begin() + i);
            i--;
        }
    }
    lines.clear();
    for (auto &[start_idx, start_angle, end_idx, end_angle] : temp_lines)
    {
        float angle = accumulate(angles_mat.ptr<float>(0) + start_idx, angles_mat.ptr<float>(0) + end_idx + 1, 0.0) / (end_idx - start_idx + 1);
        Point2f center = static_cast<Point2f>(accumulate(contour_plus.begin() + start_idx, contour_plus.begin() + end_idx + 1, Point(0, 0))) / static_cast<float>((end_idx - start_idx + 1));
        lines.emplace_back(start_idx, end_idx, angle, center);
    }

    return true;
}

// 正反线段匹配
/**
 * @brief 匹配正反线段
 * @param[in] contour_plus 加长后的轮廓
 * @param[in] angles_mat 轮廓角度数组
 * @param[in,out] lines 所有的线段
 * @param[out] matched 匹配的正反线段
 * @return 是否匹配成功
 *
 * @note 1. 直线的表示方式：Vec4f(第一个点的索引，第一个点的角度，第二个点的索引，第二个点的角度)
 *      2. 匹配的条件：1. 角度差值大于阈值 2. 中心点的垂直距离小于阈值
 *     3. 匹配结果 matched: tuple(正线段，反线段，正线段的角度（滤波后）)
 */
inline bool matchLine(const vector<Point> contour_plus, const Mat &angles_mat, std::vector<Line> &lines, std::vector<tuple<Line, Line>> &matched_lines)
{

    // 设置最小外接矩形
    RotatedRect rect = minAreaRect(contour_plus);
    // 设置最大垂直距离
    float max_vertical_distance = max(rect.size.height, rect.size.width) / 2.0;
    // 最大角度偏差
    float max_angle_delta = 20; // 正线段的方向 和 反线段的反方向的 最大角度差值

    // 获取两条直线中心点的垂直距离
    auto getLineVerticalDistance = [](const Line &l1, const Line &l2) -> float
    {
        float len_1 = l1.end_idx - l1.start_idx + 1;
        float len_2 = l2.end_idx - l2.start_idx + 1;
        float ave_angle = (l1.angle * len_1 + l2.angle * len_2) / (len_1 + len_2);
        float vertical_distance = getDist(l1.center, l2.center) * abs(cos(deg2rad(abs(ave_angle - l1.angle))));
        return vertical_distance;
    };

    // 判断两条正反直线能否匹配
    auto isMatchedLine = [&](const size_t &line1_idx, const size_t &line2_idx, float &vertical_distance) -> bool // vertical_distance 为 正向线段在上一次匹配中，和反向线段的垂直距离。
    {
        const float &angle1 = lines[line1_idx].angle;
        const float &angle2 = lines[line2_idx].angle;
        const Point2f &center1 = lines[line1_idx].center;
        const Point2f &center2 = lines[line2_idx].center;
        float delta_angle = abs(angle2 - angle1);
        if (delta_angle > 360)
            return false;
        if (abs(delta_angle - 180) > max_angle_delta)
            return false; // 角度差值小于阈值，不可能是正反线段
        float new_vertical_distance = getLineVerticalDistance(lines[line1_idx], lines[line2_idx]);
        if (new_vertical_distance > max_vertical_distance)
            return false; // 垂直距离大于阈值，不可能是正反线段
        if (new_vertical_distance > vertical_distance)
            return false; // 垂直距离 大于上一个匹配线段，不进行更新。
        vertical_distance = new_vertical_distance;
        return true;
    };

    vector<tuple<size_t, size_t>> matched_idxs{};
    // 匹配正反线段
    for (size_t i = 0; i < lines.size(); i++)
    {
        float last_vertical_distance = 1e6;
        bool is_make = false;
        size_t matched_j = -1;
        for (size_t j = i + 1; j < lines.size(); j++)
        {
            // 判断是否为正反线段
            if (abs(lines[i].angle - lines[j].angle) > 360)
                break; // 超过360度，不可能是相邻的正反线段,直接退出，因为后面线段的角度差值只会更大
            if (abs(lines[i].end_idx - lines[j].start_idx) > contour_plus.size() / 3.0)
                break;

            const float &angle1 = lines[i].angle;
            const float &angle2 = lines[j].angle;
            float delta_angle = abs(angle2 - angle1);
            if (delta_angle > 360)
                continue;
            ;
            if (abs(delta_angle - 180) > max_angle_delta)
                continue;
            ; // 角度差值小于阈值，不可能是正反线段
            float new_vertical_distance = getLineVerticalDistance(lines[i], lines[j]);
            if (new_vertical_distance > max_vertical_distance)
                break; // 垂直距离大于阈值，不可能是正反线段
            if (new_vertical_distance > last_vertical_distance)
                continue;
            ; // 垂直距离 大于上一个匹配线段，不进行更新。
            last_vertical_distance = new_vertical_distance;
            // 判断为
            is_make = true;
            matched_j = j;
        }
        if (is_make)
        {
            matched_idxs.emplace_back(i, matched_j);
        }
    }

    // 尝试合并 共用了同一个反线段 的正线段
    for (size_t i = 0; i < matched_idxs.size(); i++)
    {
        auto &[up_line_idx_1, down_line_idx_1] = matched_idxs[i];
        for (size_t j = i + 1; j < matched_idxs.size(); j++)
        {
            auto &[up_line_idx_2, down_line_idx_2] = matched_idxs[j];
            if (down_line_idx_1 == down_line_idx_2)
            {
                Line &l1 = lines[up_line_idx_1];
                Line &l2 = lines[up_line_idx_2];
                matched_idxs.erase(matched_idxs.begin() + j); // 删除被合并的一方的匹配关系
                float distance = getLineVerticalDistance(l1, l2);
                if (distance < 5) // 两条正线段比较贴近，可以进行合并。
                {
                    // 更新第一个正向匹配线段
                    l1.angle = (l1.angle * (l1.end_idx - l1.start_idx + 1) + l2.angle * (l2.end_idx - l2.start_idx + 1)) / (l1.end_idx - l1.start_idx + 1 + l2.end_idx - l2.start_idx + 1);
                    l1.start_idx = min(l1.start_idx, l2.start_idx);
                    l1.end_idx = max(l1.end_idx, l2.end_idx);
                }
            }
        }
    }

    // 删除中心在轮廓外部的线段组
    matched_idxs.erase(remove_if(matched_idxs.begin(), matched_idxs.end(), [&](const tuple<size_t, size_t> &matched_idx)
                                 {
                           const Line &up_line = lines[get<0>(matched_idx)];
                           const Line &down_line = lines[get<1>(matched_idx)];
                           Point2f up_line_direction = Point2f(cos(deg2rad(up_line.angle)), sin(deg2rad(up_line.angle)));
                           Point2f up2down_vector = down_line.center - up_line.center;
                           double cross = up_line_direction.cross(up2down_vector);
                           return cross > 0; }),
                       matched_idxs.end());

    // 生成匹配的线段
    for (auto &[up_line_idx, down_line_idx] : matched_idxs)
    {
        matched_lines.emplace_back(lines[up_line_idx], lines[down_line_idx]);
    }

    return true;
}

// 线段角度矫正
/**
 * @brief 正反线段对角度矫正
 * @param[in,out] line_pairs 匹配的正反线段对
 * @note 将正反线段的角度归为 -180 ~ 180，且利用两者的角度互相矫正
 */
inline void correctLineAngle(std::vector<std::tuple<Line, Line>> &line_pairs)
{
    for (auto &[up_line, down_line] : line_pairs)
    {
        auto normalizeAngle = [](float &angle)
        {
            while (angle > 180)
                angle -= 360;
            while (angle < -180)
                angle += 360;
        };
        normalizeAngle(up_line.angle);
        normalizeAngle(down_line.angle);
        float ave_angle = (up_line.angle + down_line.angle) / 2.0;
        up_line.angle = up_line.angle > ave_angle ? ave_angle + 90 : ave_angle - 90;
        down_line.angle = down_line.angle > ave_angle ? ave_angle + 90 : ave_angle - 90;
        normalizeAngle(up_line.angle);
        normalizeAngle(down_line.angle);
    }
}

// 重合线段对合并
/**
 */
inline bool mergeLinePairs(const vector<Point> &contour_plus, std::vector<std::tuple<Line, Line>> &line_pairs)
{
    static const float max_distance = 20;
    static const float max_angle_delta = 10;
    static const float max_len_ratio = 3.0;

    auto getDeltaAngle = [](const Line &l1, const Line &l2) -> float
    {
        float delta_angle = abs(l1.angle - l2.angle);
        if (delta_angle > 360)
            delta_angle -= 360;
        if (delta_angle > 180)
            delta_angle = 360 - delta_angle;
        return delta_angle;
    };
    for (int i = 0; i < static_cast<int>(line_pairs.size()); i++)
    {
        auto &[up_line_1, down_line_1] = line_pairs[i];
        Point2f center_1 = (up_line_1.center + down_line_1.center) / 2.0;
        for (int j = i + 1; j < static_cast<int>(line_pairs.size()); j++)
        {
            auto &[up_line_2, down_line_2] = line_pairs[j];
            Point2f center_2 = (up_line_2.center + down_line_2.center) / 2.0;
            if (getDist(center_1, center_2) > max_distance) // 中心点距离过远，不可能是同一对线段
            {
                continue;
            }
            float up_delta_angle = getDeltaAngle(up_line_1, up_line_2);
            float down_delta_angle = getDeltaAngle(down_line_1, down_line_2);
            if (up_delta_angle > max_angle_delta || down_delta_angle > max_angle_delta) // 角度差距过大，不可能是同一对线段
            {
                continue;
            }
            float up_len_1 = up_line_1.end_idx - up_line_1.start_idx + 1;
            float up_len_2 = up_line_2.end_idx - up_line_2.start_idx + 1;
            float down_len_1 = down_line_1.end_idx - down_line_1.start_idx + 1;
            float down_len_2 = down_line_2.end_idx - down_line_2.start_idx + 1;
            if (max(up_len_1, up_len_2) / min(up_len_1, up_len_2) < max_len_ratio) // 若线段长度差距不大，可以合并
            {
                Point2f direction_1(cos(deg2rad(up_line_1.angle)), sin(deg2rad(up_line_1.angle)));
                Point2f direction_2(cos(deg2rad(up_line_2.angle)), sin(deg2rad(up_line_2.angle)));
                Point2f new_direction = (direction_1 * up_len_1 + direction_2 * up_len_2) / (up_len_1 + up_len_2);
                up_line_1.angle = rad2deg(atan2(new_direction.y, new_direction.x));
            }
            else
            {
                up_line_1.angle = up_len_1 > up_len_2 ? up_line_1.angle : up_line_2.angle;
            }
            if (max(down_len_1, down_len_2) / min(down_len_1, down_len_2) < max_len_ratio) // 若线段长度差距不大，可以合并
            {
                Point2f direction_1(cos(deg2rad(down_line_1.angle)), sin(deg2rad(down_line_1.angle)));
                Point2f direction_2(cos(deg2rad(down_line_2.angle)), sin(deg2rad(down_line_2.angle)));
                Point2f new_direction = (direction_1 * down_len_1 + direction_2 * down_len_2) / (down_len_1 + down_len_2);
                down_line_1.angle = rad2deg(atan2(new_direction.y, new_direction.x));
            }
            else
            {
                down_line_1.angle = down_len_1 > down_len_2 ? down_line_1.angle : down_line_2.angle;
            }
            line_pairs.erase(line_pairs.begin() + j);
            j--;
        }
    }

    return true;
}

auto RuneFanActive::getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    vector<Point2f> points_2d{};
    vector<Point3f> points_3d{};
    vector<float> weights{};

    if (isSetTopHumpCorners())
    {
        auto &corners = getTopHumpCorners();
        points_2d.insert(points_2d.end(), corners.begin(), corners.end());
        points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_TOP_3D.begin(), rune_fan_param.ACTIVE_TOP_3D.end());
    }
    if (isSetBottomCenterHumpCorners())
    {
        auto &corners = getBottomCenterHumpCorners();
        points_2d.insert(points_2d.end(), corners.begin(), corners.end());
        points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_BOTTOM_CENTER_3D.begin(), rune_fan_param.ACTIVE_BOTTOM_CENTER_3D.end());
    }
    if (isSetSideHumpCorners())
    {
        auto &corners = getSideHumpCorners();
        points_2d.insert(points_2d.end(), corners.begin(), corners.end());
        points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_SIDE_3D.begin(), rune_fan_param.ACTIVE_SIDE_3D.end());
    }
    if (isSetBottomSideHumpCorners())
    {
        auto &corners = getBottomSideHumpCorners();
        points_2d.insert(points_2d.end(), corners.begin(), corners.end());
        points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_BOTTOM_SIDE_3D.begin(), rune_fan_param.ACTIVE_BOTTOM_SIDE_3D.end());
    }
    
    if (points_2d.size() != points_3d.size())
    {
        VC_THROW_ERROR("The size of points_2d and points_3d must be equal");
    }
    weights.resize(points_2d.size(), 1.0);

    return make_tuple(points_2d, points_3d, weights);
}

inline bool extendLines(const vector<Point> &contour_plus, const std::vector<Line> &lines, std::vector<tuple<Line, Line>> &matched_idxs)
{
    return true;
}

bool RuneFanActive::getLinePairs(const vector<Point> &contour_plus, const Mat &angles_mat, const Mat &gradient_mat, std::vector<std::tuple<Line, Line>> &line_pairs)
{
    // 获取所有线段
    vector<Line> lines;
    getAllLine(contour_plus, angles_mat, gradient_mat, lines);

#if RUNE_FAN_DEBUG
    // 绘制所有线段
    do
    {
        Mat img_show = DebugTools::get().getImage();
        for (const auto &line : lines)
        {
            Point2f start_point = contour_plus[line.start_idx];
            Point2f end_point = contour_plus[line.end_idx];
            cv::line(img_show, start_point, end_point, Scalar(0, 255, 0), 1);
            Point2f center = (start_point + end_point) / 2.0;
            circle(img_show, center, 2, Scalar(255, 0, 0), -1);
        }
    } while (0);
#endif
 
    // 匹配正反线段
    vector<tuple<Line, Line>> matched_lines;
    matchLine(contour_plus, angles_mat, lines, matched_lines);
    // 合并明显重合的线段对
    mergeLinePairs(contour_plus, matched_lines);
    // 线段角度矫正
    correctLineAngle(matched_lines);

    // 线段延长
    extendLines(contour_plus, lines, matched_lines);

    line_pairs = matched_lines;
    return true;
}

// ------------------------【已激活扇叶】------------------------
bool RuneFanActive::getActiveFunCorners(const Contour_cptr &contour,
                                        std::vector<cv::Point2f> &top_hump_corners,
                                        std::vector<cv::Point2f> &bottom_center_hump_corners,
                                        std::vector<cv::Point2f> &side_hump_corners,
                                        std::vector<cv::Point2f> &bottom_side_hump_corners)
{
    // 获取轮廓的平均点坐标
    Point2f contour_center = contour->fittedEllipse().center;

    // 对轮廓进行加长
    const auto &raw_contour_points = contour->points();
    vector<Point> contour_plus(raw_contour_points.begin(), raw_contour_points.end());
    contour_plus.insert(contour_plus.end(), raw_contour_points.begin(), raw_contour_points.begin() + static_cast<size_t>(raw_contour_points.size() / 3));
    Mat angles_mat = getAngles(contour_plus);   // 角度数组
    Mat gradient_mat = getGradient(angles_mat); // 角度梯度数组

    vector<tuple<Line, Line>> line_pairs;
    getLinePairs(contour_plus, angles_mat, gradient_mat, line_pairs);

    if (line_pairs.size() < 4) // 线段匹配不足，放弃构造
    {
        return false;
    }

    //========获取 顶部 角点===========
    // DEBUG_RUNE_INFO_("fan active 5.get_top_humps");
    vector<TopHump> top_humps = TopHump::getTopHumps(contour_plus, contour_center, line_pairs);
    if (top_humps.empty())
    {
        return false; // 顶部角点没有获取到，放弃构造
    }

    //=========获取底部中心角点====================
    vector<BottomCenterHump> bottom_center_humps = BottomCenterHump::getBottomCenterHump(contour_plus, contour_center, top_humps);
    if (bottom_center_humps.empty())
    {
        return false; // 底部中心角点没有获取到，放弃构造
    }

    //=========获取侧面角点===================
    vector<SideHump> side_humps = SideHump::getSideHumps(contour_plus, contour_center, top_humps, bottom_center_humps, line_pairs);
    if (side_humps.empty())
    {
    }

    // 设置角点
    if (!top_humps.empty())
        top_hump_corners = {top_humps[0].getVertex(), top_humps[1].getVertex(), top_humps[2].getVertex()};
    else
        top_hump_corners = {};

    if (!bottom_center_humps.empty())
        bottom_center_hump_corners = {bottom_center_humps[0].getVertex()};
    else
        bottom_center_hump_corners = {};

    if (!side_humps.empty())
        side_hump_corners = {side_humps[0].getVertex(), side_humps[1].getVertex()};
    else
        side_hump_corners = {};

    bottom_side_hump_corners = {};

    return true;
}

// ------------------------【已激活扇叶】------------------------
RuneFanActive::RuneFanActive(const Contour_cptr &contour,
                             const cv::RotatedRect &rotated_rect,
                             const std::vector<cv::Point2f> &top_hump_corners,
                             const std::vector<cv::Point2f> &bottom_center_hump_corners,
                             const std::vector<cv::Point2f> &side_hump_corners,
                             const std::vector<cv::Point2f> &bottom_side_hump_corners)
{

    if (!top_hump_corners.empty()) // 判断是否获取到顶部突起
    {
        if (top_hump_corners.size() != 3)
        {
            VC_THROW_ERROR(" top_hump_corners \" are not equal to 3. (size = %zu)", top_hump_corners.size());
        }
        setTopHumpCorners(top_hump_corners); // 设置顶部突起角点
    }
    if (!bottom_center_hump_corners.empty()) // 判断是否获取到底部中心突起
    {
        if (bottom_center_hump_corners.size() != 1)
        {
            VC_THROW_ERROR(" bottom_center_hump_corners \" are not equal to 1. (size = %zu)", bottom_center_hump_corners.size());
        }
        setBottomCenterHumpCorners(bottom_center_hump_corners); // 设置底部中心突起角点
    }
    if (!side_hump_corners.empty()) // 判断是否获取到侧面突起
    {
        if (side_hump_corners.size() != 2)
        {
            VC_THROW_ERROR(" side_hump_corners \" are not equal to 2. (size = %zu)", side_hump_corners.size());
        }
        setSideHumpCorners(side_hump_corners); // 设置侧面突起角点
    }
    if (!bottom_side_hump_corners.empty()) // 判断是否获取到底部侧面突起
    {
        if (bottom_side_hump_corners.size() != 2)
        {
            VC_THROW_ERROR(" bottom_side_hump_corners \" are not equal to 2. (size = %zu)", bottom_side_hump_corners.size());
        }
        setBottomSideHumpCorners(bottom_side_hump_corners); // 设置底部侧面突起角点
    }

    if (isSetTopHumpCorners() == false || isSetBottomCenterHumpCorners() == false) // 如果没有获取到顶部角点和底部中心角点，判定为参数传入错误
    {
        VC_THROW_ERROR("RuneFan::RuneFan : top_hump_corners or bottom_center_hump_corners is empty");
    }

    // 根据不同的获取情况，设置角点
    vector<Point2f> corners;
    if (isSetTopHumpCorners() && isSetBottomCenterHumpCorners() && !isSetSideHumpCorners() && !isSetBottomSideHumpCorners()) // 只获取到顶部角点和底部中心角点
    {
        corners = {top_hump_corners[0], top_hump_corners[1], top_hump_corners[2], bottom_center_hump_corners[0]};
    }

    if (isSetTopHumpCorners() && isSetBottomCenterHumpCorners() && !isSetSideHumpCorners() && isSetBottomSideHumpCorners()) // 获取到顶部角点、底部中心角点和底部侧面角点
    {
        corners = {top_hump_corners[0], top_hump_corners[1], top_hump_corners[2], bottom_side_hump_corners[1], bottom_center_hump_corners[0], bottom_side_hump_corners[0]};
    }

    if (isSetTopHumpCorners() && isSetBottomCenterHumpCorners() && isSetSideHumpCorners() && !isSetBottomSideHumpCorners()) // 获取到顶部角点、底部中心角点和侧面角点
    {
        corners = {top_hump_corners[0], top_hump_corners[1], top_hump_corners[2], side_hump_corners[1], bottom_center_hump_corners[0], side_hump_corners[0]};
    }
    if (isSetTopHumpCorners() && isSetBottomCenterHumpCorners() && isSetSideHumpCorners() && isSetBottomSideHumpCorners()) // 获取到所有角点
    {
        corners = {top_hump_corners[0], top_hump_corners[1], top_hump_corners[2], side_hump_corners[1], bottom_side_hump_corners[1], bottom_center_hump_corners[0], bottom_side_hump_corners[0], side_hump_corners[0]};
    }

    float width = getDist(top_hump_corners[0], top_hump_corners[2]);                  // 将顶部角点的左右两个角点之间的距离作为宽度
    float height = getDist(top_hump_corners[1], bottom_center_hump_corners[0]);       // 将顶部中心角点和底部中心角点之间的距离作为高度
    Point2f center = (top_hump_corners[1] + bottom_center_hump_corners[0]) / 2;       // 将顶部中心角点和底部中心角点的中点作为扇叶的中心点
    setDirection(getUnitVector(bottom_center_hump_corners[0] - top_hump_corners[1])); // 将顶部中心角点指向底部中心角点的向量作为扇叶的方向
    setRotatedRect(rotated_rect);                                                     // 设置最小外接矩形
    // 图像属性
    auto &image_info = this->getImageCache();
    image_info.setContours(vector<Contour_cptr>{contour}); // 设置轮廓
    image_info.setCorners(corners);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCenter(center);
    setActiveFlag(true); // 激活标志位
}
