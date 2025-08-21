#include "vc/feature/rune_target.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_target_param.h"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;

/**
 * @brief 构造函数
 * @param contour 轮廓
 * @param corners 角点
 */
RuneTarget::RuneTarget(const Contour_cptr contour, const std::vector<cv::Point2f> corners)
{
    if(contour->points().size() < 3)
    {
        VC_THROW_ERROR("Contour points are less than 3, cannot construct RuneTarget.");
    }
    auto rotate_rect = contour->minAreaRect();
    auto width = std::max(rotate_rect.size.width,rotate_rect.size.height);
    auto height = std::min(rotate_rect.size.width, rotate_rect.size.height);
    Point2f center{};
    if(contour->points().size() > 6)
    {
        center = contour->fittedEllipse().center;
    }
    else
    {
        center = contour->center();
    }
    
    auto& image_info = getImageCache();
    image_info.setContours(vector<Contour_cptr>{contour});
    image_info.setCorners(corners);
    image_info.setCenter(center);
    image_info.setWidth(width);
    image_info.setHeight(height);
}

void RuneTarget::find_active_targets(std::vector<FeatureNode_ptr> &targets,
                                     const std::vector<Contour_cptr> &contours,
                                     const std::vector<cv::Vec4i> &hierarchy,
                                     const std::unordered_set<size_t> &mask,
                                     std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs)
{
    RuneTargetActive::find(targets, contours, hierarchy, mask, used_contour_idxs);
}

void RuneTarget::find_inactive_targets(std::vector<FeatureNode_ptr> &targets,
                                       const std::vector<Contour_cptr> &contours,
                                       const std::vector<cv::Vec4i> &hierarchy,
                                       const std::unordered_set<size_t> &mask,
                                       std::unordered_map<FeatureNode_cptr, std::unordered_set<size_t>> &used_contour_idxs)
{
    RuneTargetInactive::find(targets, contours, hierarchy, mask, used_contour_idxs);
}

auto RuneTarget::getPnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    // 默认实现返回空
    return std::make_tuple(std::vector<cv::Point2f>{}, std::vector<cv::Point3f>{}, std::vector<float>{});
}

auto RuneTarget::getRelativePnpPoints() const -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>>
{
    auto [points_2d, points_3d, weights] = this->getPnpPoints();
    vector<Point3f> relative_points_3d(points_3d.size());
    for (size_t i = 0; i < points_3d.size(); i++)
    {
        Matx31d points_3d_mat(points_3d[i].x, points_3d[i].y, points_3d[i].z);
        Matx31d relative_points_3d_mat = rune_target_param.ROTATION * points_3d_mat + rune_target_param.TRANSLATION;
        relative_points_3d[i] = Point3f(relative_points_3d_mat(0), relative_points_3d_mat(1), relative_points_3d_mat(2));
    }
    return make_tuple(points_2d, relative_points_3d, weights);
}

// std::shared_ptr<RuneTarget> RuneTarget::make_feature(const PoseNode &target_to_cam, bool is_active)
// {
//     vector<Point3f> corners_3d{};
//     if (is_active)
//     {
//         float radius = rune_target_param.RADIUS;
//         corners_3d.emplace_back(0, -radius, 0);
//         corners_3d.emplace_back(radius, 0, 0);
//         corners_3d.emplace_back(0, radius, 0);
//         corners_3d.emplace_back(-radius, 0, 0);
//     }
//     else
//     {
//         for (const auto &corner : rune_target_param.GAP_3D)
//         {
//             corners_3d.emplace_back(corner);
//         }
//     }

//     // 重投影
//     vector<Point2f> corners_2d{};
//     projectPoints(corners_3d, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, corners_2d);

//     Point2f rune_center{};
//     vector<Point2f> temp_rune_center{};
//     projectPoints(vector<Point3f>{Point3f(0, 0, 0)}, target_to_cam.rvec(), target_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, temp_rune_center);
//     auto result_ptr = make_shared<RuneTarget>(temp_rune_center[0], corners_2d);

//     if (result_ptr)
//     {
//         auto &pose_info = result_ptr->getPoseCache();
//         pose_info.getPoseNodes()[CoordFrame::CAMERA] = target_to_cam;
//     }
//     return result_ptr;
// }

