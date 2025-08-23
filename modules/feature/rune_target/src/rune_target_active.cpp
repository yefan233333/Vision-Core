#include <opencv2/imgproc.hpp>
#include <string>

#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_target_param.h"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;


RuneTargetActive::RuneTargetActive(const Contour_cptr contour, const std::vector<cv::Point2f> corners)
    : RuneTarget(contour, corners)
{
    setActiveFlag(true);
}

RuneTargetActive::RuneTargetActive(const Point2f center, const std::vector<cv::Point2f> corners)
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

    setActiveFlag(true);
    auto &image_info = getImageCache();
    image_info.setCenter(center);
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCorners(corners);
    image_info.setContours(vector<Contour_cptr>{contour});
}

/**
 * @brief 已激活神符靶心等级向量判断
 *
 * @param[in] contours 所有轮廓
 * @param[in] hierarchy 所有的等级向量
 * @param[in] idx 指定的等级向量的下标
 * @return 等级结构是否满足要求
 */
inline bool isHierarchyActiveTarget(const vector<Contour_cptr> &contours, const vector<Vec4i> &hierarchy, size_t idx)
{
    if (hierarchy[idx][3] != -1) // 该轮廓有父轮廓，退出
        return false;
    if (hierarchy[idx][2] == -1) // 该轮廓没有子轮廓，退出
        return false;

    return true;
}

void RuneTargetActive::find(std::vector<FeatureNode_ptr> &targets,
                            const std::vector<Contour_cptr> &contours,
                            const std::vector<cv::Vec4i> &hierarchy,
                            const std::unordered_set<size_t> &mask,
                            std::unordered_map<FeatureNode_cptr, unordered_set<size_t>> &used_contour_idxs)
{

    for (size_t i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (isHierarchyActiveTarget(contours, hierarchy, i))
        {
            unordered_set<size_t> temp_used_contour_idxs{};
            auto p_target = RuneTargetActive::make_feature(contours, hierarchy, i, temp_used_contour_idxs);
            if (p_target != nullptr)
            {
                targets.push_back(p_target);
                used_contour_idxs[p_target] = temp_used_contour_idxs;
            }
        }
    }
}

auto RuneTargetActive::getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    vector<Point2f> points_2d = {getImageCache().getCenter()};
    vector<Point3f> points_3d = {Point3f(0, 0, 0)};
    vector<float> weights = {1.0};

    return make_tuple(points_2d, points_3d, weights);
}

/**
 * @brief 椭圆度检测
 *
 * @param contours 轮廓
 */
inline bool checkEllipse(const Contour_cptr &contour)
{
    if (contour->points().size() < 6)
    {
        return false;
    }
    float contour_area = contour->area();
    // -----------------------绝对面积判断-------------------------
    if (contour_area < rune_target_param.ACTIVE_MIN_AREA)
    {
        return false;
    }
    if (contour_area > rune_target_param.ACTIVE_MAX_AREA)
    {
        return false;
    }

    // -----------------------边长比例判断-------------------------
    auto fit_ellipse = contour->fittedEllipse();
    float width = max(fit_ellipse.size.width, fit_ellipse.size.height);
    float height = min(fit_ellipse.size.width, fit_ellipse.size.height);
    float side_ratio = width / height;

    if (side_ratio > rune_target_param.ACTIVE_MAX_SIDE_RATIO)
    {
        return false;
    }
    if (side_ratio < rune_target_param.ACTIVE_MIN_SIDE_RATIO)
    {
        return false;
    }

    // ------------------------面积比例判断----------------------------
    float fit_ellipse_area = width * height * CV_PI / 4;
    float area_ratio = contour_area / fit_ellipse_area;
    // {
    //     return false;
    // }
    if (area_ratio > rune_target_param.ACTIVE_MAX_AREA_RATIO)
    {
        return false;
    }
    if (area_ratio < rune_target_param.ACTIVE_MIN_AREA_RATIO)
    {
        return false;
    }

    // ----------------------周长比例判断-------------------------
    float perimeter = contour->perimeter();
    float fit_ellipse_perimeter = CV_PI * (3 * (width + height) - sqrt((3 * width + height) * (width + 3 * height)));
    float perimeter_ratio = perimeter / fit_ellipse_perimeter;
    if (perimeter_ratio > rune_target_param.ACTIVE_MAX_PERI_RATIO)
    {
        return false;
    }
    if (perimeter_ratio < rune_target_param.ACTIVE_MIN_PERI_RATIO)
    {
        return false;
    }
    const auto &hull_contour = contour->convexHull();
    float hull_area = contourArea(hull_contour);
    float hull_area_ratio = hull_area / contour_area;
    if (hull_area_ratio > rune_target_param.ACTIVE_MAX_CONVEX_AREA_RATIO)
    {
        return false;
    }
    float hull_perimeter = arcLength(hull_contour, true);
    float hull_perimeter_ratio = hull_perimeter / perimeter;
    hull_perimeter_ratio = hull_perimeter_ratio > 1 ? hull_perimeter_ratio : 1 / hull_perimeter_ratio;
    if (hull_perimeter_ratio > rune_target_param.ACTIVE_MAX_CONVEX_PERI_RATIO)
    {
        return false;
    }

    return true;
}

/**
 * @brief 常规环数的靶心构造检验
 *
 * @param[in] contours 所有轮廓
 * @param[in] hierarchy 等级向量
 * @param[in] idx 轮廓下标
 * @param[in] all_sub_idx 所有子轮廓下标
 *
 * @return 是否符合要求
 */
inline bool checkConcentricity(const std::vector<Contour_cptr> &contours,
                               const std::vector<cv::Vec4i> &hierarchy,
                               const std::vector<int> &all_sub_idx,
                               size_t idx,
                               double contour_area)
{
    if (all_sub_idx.empty()) // 无子轮廓
    {
        return false;
    }
    unordered_map<size_t, float> area_map;
    for (auto sub_idx : all_sub_idx)
    {
        float area = contours[sub_idx]->area();
        area_map[sub_idx] = area;
    }
    auto [max_area_idx, max_sub_area] = *max_element(area_map.begin(), area_map.end(), [](const auto &lhs, const auto &rhs)
                                                     { return lhs.second < rhs.second; });
    if (max_sub_area > contour_area)
    {
        VC_THROW_ERROR("sub_contour_area > outer_area"); // 子轮廓面积大于当前轮廓面积
    }
    float sub_area_ratio = max_sub_area / contour_area;
    if (sub_area_ratio < rune_target_param.ACTIVE_MIN_AREA_RATIO_SUB) // 子轮廓面积过小
    {
        return false;
    }

    // 对最大子轮廓进行椭圆度检测6
    if (checkEllipse(contours[max_area_idx]) == false)
        return false;

    return true;
}

/**
 * @brief 针对十环靶心的构造检验
 *
 * @param contours 轮廓
 * @param hierarchy 等级向量
 * @param idx 轮廓下标
 * @param all_sub_idx 所有子轮廓下标
 * @param contour_area 轮廓面积
 *
 */
inline bool checkTenRing(const std::vector<Contour_cptr> &contours,
                         const std::vector<cv::Vec4i> &hierarchy,
                         const std::vector<int> &all_sub_idx,
                         size_t idx,
                         double contour_area)
{
    // 计算所有子轮廓的面积之和
    float total_area = 0;
    for (auto sub_idx : all_sub_idx)
    {
        float area = contours[sub_idx]->area();
        total_area += area;
    }
    // 获取比例
    float area_ratio = total_area / contour_area;
    if (area_ratio > rune_target_param.ACTIVE_MAX_AREA_RATIO_SUB_TEN_RING)
    {
        return false;
    }
    return true;
}

RuneTargetActive_ptr RuneTargetActive::make_feature(const std::vector<Contour_cptr> &contours,
                                                    const std::vector<cv::Vec4i> &hierarchy,
                                                    size_t idx,
                                                    std::unordered_set<size_t> &used_contour_idxs)
{

    const auto &contour_outer = contours[idx];

    // 轮廓点数判断
    if (contour_outer->points().size() < 6)
        return nullptr;

    float contour_area = contour_outer->area();

    // 椭圆度检测
    if (checkEllipse(contour_outer) == false)
        return nullptr;

    // 当面积足够时，利用子轮廓进行判断
    vector<int> all_sub_idx{};
    getAllSubContoursIdx(hierarchy, idx, all_sub_idx);
    if (contour_area < 100)
        return nullptr;

    if (checkConcentricity(contours, hierarchy, all_sub_idx, idx, contour_area) == false)
    {
        if (checkTenRing(contours, hierarchy, all_sub_idx, idx, contour_area) == false)
            return nullptr;
    }

    // 将当前轮廓和子轮廓加入到使用轮廓集合中
    used_contour_idxs.insert(idx);
    used_contour_idxs.insert(all_sub_idx.begin(), all_sub_idx.end());

    auto fit_ellipse = contour_outer->fittedEllipse();

    // 构建未激活靶心对象，并赋予属性
    vector<Point2f> corners{};
    corners.push_back(fit_ellipse.center); // 将中心点作为第一个角点
    auto rune_target = make_shared<RuneTargetActive>(contour_outer, corners);
    return rune_target;
}

void RuneTargetActive::drawFeature(cv::Mat &image, const DrawConfig_cptr &config) const
{
    // 利用半径绘制正圆
    auto draw_circle = [&]() -> bool
    {
        const auto &image_info = this->getImageCache();
        if (!image_info.isSetCorners())
            return false;
        if (!image_info.isSetCenter())
        {
            return false;
        }
        const auto &center = image_info.getCenter();
        auto default_radius = rune_target_draw_param.active.point_radius;
        auto circle_color = rune_target_draw_param.active.color;
        auto circle_thickness = rune_target_draw_param.active.thickness;
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
        auto circle_color = rune_target_draw_param.active.color;
        auto circle_thickness = rune_target_draw_param.active.thickness;
        cv::ellipse(image, fit_ellipse, circle_color, circle_thickness);
        cv::circle(image, fit_ellipse.center, 2, circle_color, -1); // 绘制中心点
        return true;
    };

    do
    {
        // 尝试绘制椭圆，若成功
        if (draw_ellipse())
        {
            break;
        }
        else
        {
            draw_circle();
        }
    } while (0);
}

RuneTargetActive_ptr RuneTargetActive::make_feature(const PoseNode &target_to_cam)
{
    vector<Point3f> corners_3d{};

    float radius = rune_target_param.RADIUS;
    corners_3d.emplace_back(0, -radius, 0);
    corners_3d.emplace_back(radius, 0, 0);
    corners_3d.emplace_back(0, radius, 0);
    corners_3d.emplace_back(-radius, 0, 0);

    // 重投影
    vector<Point2f> corners_2d{};
    projectPoints(corners_3d, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, corners_2d);

    Point2f rune_center{};
    vector<Point2f> temp_rune_center{};
    projectPoints(vector<Point3f>{Point3f(0, 0, 0)}, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, temp_rune_center);

    auto result_ptr = make_shared<RuneTargetActive>(temp_rune_center[0], corners_2d);

    if (result_ptr)
    {
        auto &pose_info = result_ptr->getPoseCache();
        pose_info.getPoseNodes()[CoordFrame::CAMERA] = target_to_cam;
    }
    return result_ptr;
}