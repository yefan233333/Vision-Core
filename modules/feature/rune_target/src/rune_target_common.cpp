#include "vc/feature/rune_target.h"
#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_active.h"
#include "vc/feature/rune_target_param.h"

using namespace std;
using namespace cv;

void RuneTarget::find_active_targets(std::vector<FeatureNode_ptr> &targets,
                                     const std::vector<Contour_cptr> &contours,
                                     const std::vector<cv::Vec4i> &hierarchy,
                                     const std::unordered_set<size_t> &mask,
                                     std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs)
{
    RuneTargetActive::find(targets, contours, hierarchy, mask, used_contour_idxs);
}

void RuneTarget::find_inactive_targets(std::vector<FeatureNode_ptr> &targets,
                                       const std::vector<Contour_cptr> &contours,
                                       const std::vector<cv::Vec4i> &hierarchy,
                                       const std::unordered_set<size_t> &mask,
                                       std::unordered_map<FeatureNode_ptr, std::unordered_set<size_t>> &used_contour_idxs)
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
        Matx31f points_3d_mat(points_3d[i].x, points_3d[i].y, points_3d[i].z);
        Matx31f relative_points_3d_mat = rune_target_param.ROTATION * points_3d_mat + rune_target_param.TRANSLATION;
        relative_points_3d[i] = Point3f(relative_points_3d_mat(0), relative_points_3d_mat(1), relative_points_3d_mat(2));
    }
    return make_tuple(points_2d, relative_points_3d, weights);
}

