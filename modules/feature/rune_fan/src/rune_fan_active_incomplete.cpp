
#include <numeric>
#include "vc/feature/rune_fan_active.h"
#include "vc/feature/rune_fan_param.h"
#include "vc/feature/rune_fan_hump_param.h"

using namespace std;
using namespace cv;

inline bool getHumps(const vector<cv::Point> &contour, vector<TopHump> &top_humps)
{
    if (contour.size() < 30)
        return false;

    // 轮廓加长
    vector<cv::Point> contour_plus{};
    contour_plus.insert(contour_plus.end(), contour.begin(), contour.end());
    contour_plus.insert(contour_plus.end(), contour.begin(), contour.begin() + contour.size() / 3.0);

    // 获取角度数组
    Mat angles_mat = RuneFanActive::getAngles(contour_plus);
    // 获取梯度数组
    Mat gradient_mat = RuneFanActive::getGradient(angles_mat);
    // 获取线段对
    vector<tuple<Line, Line>> line_pairs{};
    RuneFanActive::getLinePairs(contour_plus, angles_mat, gradient_mat, line_pairs);

    // 获取所有的突起点
    vector<TopHump> humps{};

    for (int i = 0; i < line_pairs.size(); i++)
    {
        auto &[up_line, down_line] = line_pairs[i];
        int up_start_idx = up_line.start_idx;
        int up_end_idx = up_line.end_idx;
        int down_start_idx = down_line.start_idx;
        int down_end_idx = down_line.end_idx;
        Point2f direction = Point2f(cos(deg2rad(up_line.angle)), sin(deg2rad(up_line.angle))); // 以上升线段的角度为方向
        Point2f center = (contour_plus[up_start_idx] + contour_plus[down_start_idx]) / 2.0;

        humps.push_back(TopHump(up_start_idx, up_end_idx, down_start_idx, down_end_idx, i, direction, center));
    }

    for (auto &hump : humps)
    {
        TopHump::setVertex(hump, contour_plus);
    }

    // 过滤突起点
    TopHump::filter(contour_plus, humps);
    top_humps = humps;
    return true;
}

inline bool isOverlap(const RuneFanActive_ptr &fan1, const RuneFanActive_ptr &fan2)
{
    // 获取外接矩形
    RotatedRect rrect1 = fan1->getRotatedRect();
    RotatedRect rrect2 = fan2->getRotatedRect();
    // 判断两个矩形是否相交
    return (rrect1.boundingRect() & rrect2.boundingRect()).area() > 0;
}

inline bool filterFan(const vector<RuneFanActive_ptr> &fans,
                      vector<RuneFanActive_ptr> &filtered_fans,
                      const Point2f &rotate_center)
{
    unordered_set<RuneFanActive_ptr> used_fans{};
    used_fans.insert(fans.begin(), fans.end());

    // 删除方向明显偏移旋转中心的扇叶
    for (auto &fan : fans)
    {
        if (used_fans.find(fan) == used_fans.end())
            continue;
        Point2f direction = fan->getDirection();
        Point2f center = fan->getRotatedRect().center;
        Point2f fan_to_center = rotate_center - fan->getImageCache().getCenter();
        double angle = getVectorMinAngle(direction, fan_to_center, DEG);
        if (abs(angle) > rune_fan_param.ACTIVE_MAX_DIRECTION_DELTA_INCOMPLETE)
            used_fans.erase(fan);
    }

    if (used_fans.empty())
        return false;

    filtered_fans.insert(filtered_fans.end(), used_fans.begin(), used_fans.end());
    return true;
}

bool RuneFanActive::find_incomplete(std::vector<RuneFanActive_ptr> &fans,
                                    const std::vector<Contour_ptr> &contours,
                                    const std::vector<cv::Vec4i> &hierarchy,
                                    const std::unordered_set<size_t> &mask,
                                    const cv::Point2f &rotate_center,
                                    std::unordered_map<FeatureNode_ptr, unordered_set<size_t>> &used_contour_idxs)
{

    // 待匹配的轮廓下标
    std::unordered_set<size_t> pending_idxs{};

    for (size_t i = 0; i < contours.size(); i++)
    {
        if (mask.find(i) != mask.end())
            continue;
        if (contours[i]->points().size() < 6)
            continue;
        if (hierarchy[i][3] != -1)
            continue;

        pending_idxs.insert(i);
    }

    // 面积筛选
    auto temp_idxs = pending_idxs;
    for (auto idx : temp_idxs)
    {
        if (contours[idx]->area() < rune_fan_param.ACTIVE_MIN_AREA_INCOMPLETE)
        {
            pending_idxs.erase(idx);
        }
    }

    if (pending_idxs.empty())
    {
        return false;
    }

    // 面积周长比例筛选
    // 待删除的轮廓下标
    unordered_set<size_t> erase_idxs{};
    for (auto idx : pending_idxs)
    {
        double area = contours[idx]->area();
        double perimeter = contours[idx]->perimeter();
        if (area / pow(perimeter, 2) > rune_fan_param.ACTIVE_MAX_AREA_PERIMETER_RATIO_INCOMPLETE)
        {
            erase_idxs.insert(idx);
            continue;
        }
    }
    for (const auto &erase_idx : erase_idxs)
    {
        pending_idxs.erase(erase_idx);
    }

    // 计算轮廓下标
    static auto getContourIdx = [](const Contour_ptr &contour, const std::vector<Contour_ptr> &contours) -> size_t
    {
        auto it = std::find(contours.begin(), contours.end(), contour);
        if (it != contours.end())
        {
            return std::distance(contours.begin(), it);
        }
        else
        {
            VC_THROW_ERROR("Contour not found in the vector");
            return -1; // 或者抛出异常
        }
    };
    vector<tuple<TopHump, Contour_ptr>> all_humps{}; // 突起点和对应的轮廓下标
    {
        for (auto idx : pending_idxs)
        {
            vector<TopHump> temp_humps{};
            getHumps(contours[idx]->points(), temp_humps);
            for (auto &hump : temp_humps)
            {
                all_humps.push_back({hump, contours[idx]});
            }
        }
    }

    // 若突起点数量过少，直接返回
    if (all_humps.size() < 3)
    {
        return false;
    }

    // 准备二维点集
    std::vector<Point2f> hump_centers(all_humps.size());
    for (size_t i = 0; i < all_humps.size(); ++i)
    {
        hump_centers[i] = get<0>(all_humps[i]).getCenter();
    }

    cv::Mat hump_centers_mat(hump_centers.size(), 2, CV_32F);
    for (size_t i = 0; i < hump_centers.size(); ++i)
    {
        hump_centers_mat.at<float>(i, 0) = hump_centers[i].x;
        hump_centers_mat.at<float>(i, 1) = hump_centers[i].y;
    }

    // 创建 FLANN 的KD树索引
    cv::flann::Index kd_tree(hump_centers_mat, cv::flann::KDTreeIndexParams(1));

    // 尝试使用零散的突起点构造残缺扇叶
    unordered_map<RuneFanActive_ptr, unordered_set<size_t>> used_contour_idxs_temp{};
    for (size_t i = 0; i < all_humps.size(); ++i)
    {
        Mat query = (Mat_<float>(1, 2) << hump_centers[i].x, hump_centers[i].y);
        float search_radius = 500;
        int max_results = 50;
        Mat indices, dists;
        indices.create(1, max_results, CV_32S);
        dists.create(1, max_results, CV_32F);
        // kd_tree.radiusSearch(query, indices, dists, pow(search_radius,2),cv::flann::SearchParams(32));
        int found = kd_tree.radiusSearch(query, indices, dists, search_radius * search_radius, max_results, cv::flann::SearchParams(32));

        for (size_t n = 0; n < found; ++n)
        {
            size_t j = indices.at<int>(0, n);
            float dist = sqrt(dists.at<float>(0, n));
            if (j <= i)
                continue; // 避免重复计算

            // cout << i << " 和 " << j << " 的距离为 " << dist << endl;

            // 若距离过大，直接跳过
            constexpr double max_distance_threshold = 300.0;
            if (dist > max_distance_threshold)
                break;

            // 获取两个突起点
            auto &hump_1 = get<0>(all_humps[i]);
            auto &hump_2 = get<0>(all_humps[j]);

            // 判断方向是否差异过大
            double max_delta_angle_threshold = 30.0; // 最大允许的角度差异
            double angle_delta = getVectorMinAngle(hump_1.getDirection(), hump_2.getDirection(), DEG);
            if (angle_delta > max_delta_angle_threshold)
                continue;

            for (size_t n2 = 0; n2 < found; ++n2)
            {
                size_t k = indices.at<int>(0, n2);
                float dist2 = sqrt(dists.at<float>(0, n2));
                if (k <= i || k == j)
                {
                    continue; // 避免重复计算
                }

                // 若距离过大，直接跳过
                if (dist2 > max_distance_threshold)
                    break;

                // 获取第三个突起点
                auto &hump_3 = get<0>(all_humps[k]);

                // 判断方向差异是否过大
                double angle_delta_2 = getVectorMinAngle(hump_1.getDirection(), hump_3.getDirection(), DEG);
                double angle_delta_3 = getVectorMinAngle(hump_2.getDirection(), hump_3.getDirection(), DEG);
                if (angle_delta_2 > max_delta_angle_threshold || angle_delta_3 > max_delta_angle_threshold)
                {
                    continue;
                }

                // 构造残缺扇叶
                auto p_fan = make_feature(all_humps[i], all_humps[j], all_humps[k]);
                if (p_fan != nullptr)
                {
                    fans.push_back(p_fan);
                    // 在 contours 中查找对应的轮廓下标
                    auto &contour_1 = get<1>(all_humps[i]);
                    auto &contour_2 = get<1>(all_humps[j]);
                    auto &contour_3 = get<1>(all_humps[k]);
                    used_contour_idxs_temp[p_fan].insert(getContourIdx(contour_1, contours));
                    used_contour_idxs_temp[p_fan].insert(getContourIdx(contour_2, contours));
                    used_contour_idxs_temp[p_fan].insert(getContourIdx(contour_3, contours));

                }
            }
        }
    }

    // 过滤获取到的扇叶
    vector<RuneFanActive_ptr> filtered_fans{};
    filterFan(fans, filtered_fans, rotate_center);

    return true;
}

RuneFanActive::RuneFanActive(const std::vector<Contour_ptr> &contours,
                 const std::vector<cv::Point2f> &top_hump_corners,
                 const cv::Point2f &direction)
{

    // 获取凸包轮廓
    vector<Point> contour_temp{};
    for (auto &contour : contours)
        contour_temp.insert(contour_temp.end(), contour->begin(), contour->end());
    vector<Point> hull_contour_temp{};
    convexHull(contour_temp, hull_contour_temp);

    auto contour = ContourWrapper<int>::make_contour(hull_contour_temp);
    RotatedRect fit_ellipse = contour->fittedEllipse();
    float width = max(fit_ellipse.size.width, fit_ellipse.size.height);
    float height = min(fit_ellipse.size.width, fit_ellipse.size.height);
    Point2f center = fit_ellipse.center;


    vector<Point2f> corners{};
    // 设置缺陷扇叶的角点（仅用于展示，实际的PNP解算不会直接使用这里的角点）
    corners.insert(corners.end(), top_hump_corners.begin(), top_hump_corners.end());
    // 获取扇叶的中垂线
    Point2f top_center = top_hump_corners[1];
    float max_projection = 0;
    for (const auto &point : contour->points())
    {
        Point2f v1 = static_cast<Point2f>(point) - top_center;
        Point2f v2 = direction;
        float projection = getProjection(v1, v2);
        if (projection > max_projection)
        {
            max_projection = projection;
        }
    }
    Point2f bottom_center = top_center + max_projection * direction;
    corners.emplace_back(bottom_center);

    // 设置基本属性
    setActiveFlag(true);
    setTopHumpCorners(top_hump_corners);
    setDirection(direction);
    setRotatedRect(fit_ellipse);

    // 设置图像属性
    auto image_info = this->getImageCache();
    image_info.setContours(vector<Contour_ptr>{contour});
    image_info.setWidth(width);
    image_info.setHeight(height);
    image_info.setCenter(center);
    image_info.setCorners(corners);
}

// 获取突起点组的中心点
inline Point2f getHumpCenter(const std::array<std::tuple<TopHump, Contour_ptr>, 3> &humps)
{
    // 计算凸包
    unordered_set<Contour_ptr> contours{};
    for (const auto &[hump, contour] : humps)
    {
        contours.insert(contour);
    }
    if (contours.size() == 3)
    {
        // 计算距离中心最近的突起点
        Point2f ave_point = (get<0>(humps[0]).getVertex() + get<0>(humps[1]).getVertex() + get<0>(humps[2]).getVertex()) / 3.0;
        std::array<float, 3> distances{};
        for (int i = 0; i < 3; i++)
        {
            distances[i] = getDist(ave_point, get<0>(humps[i]).getVertex());
        }
        // 获取距离中心最近的突起点
        int center_hump_idx = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
        return get<1>(humps[center_hump_idx])->center();
    }
    else if (contours.size() == 2)
    {
        // 计算两个轮廓中的凸包
        auto hull_contour = ContourWrapper<int>::getConvexHull({contours.begin(), contours.end()});
        // 获取轮廓的中心点
        Point2f center = hull_contour->fittedEllipse().center;
        return center;
    }
    else if (contours.size() == 1)
    {
        return contours.begin()->get()->fittedEllipse().center;
    }
    else
    {
        VC_THROW_ERROR("humps size is not equal to 3");
        return Point2f(0, 0);
    }
    return Point2f(0, 0);
}

std::shared_ptr<RuneFanActive> RuneFanActive::make_feature(const std::tuple<TopHump, Contour_ptr> &hump_1,
                                                           const std::tuple<TopHump, Contour_ptr> &hump_2,
                                                           const std::tuple<TopHump, Contour_ptr> &hump_3)
{
    const auto &[hump_1_obj, contour_1] = hump_1;
    const auto &[hump_2_obj, contour_2] = hump_2;
    const auto &[hump_3_obj, contour_3] = hump_3;

    Point2f contours_center = getHumpCenter({hump_1, hump_2, hump_3});

    // 获取需要用到的突起点数组
    TopHumpCombo hump_combo(hump_1_obj, hump_2_obj, hump_3_obj);

    // ------------------判断能否构造----------------------------
    // 利用突起点之间的距离判定(排除明显不相邻的情况)
    double max_distance = 0; // 获取特征之间的最大距离
    double distance_1 = getDist(hump_1_obj.getVertex(), hump_2_obj.getVertex());
    double distance_2 = getDist(hump_1_obj.getVertex(), hump_3_obj.getVertex());
    double distance_3 = getDist(hump_2_obj.getVertex(), hump_3_obj.getVertex());
    max_distance = max(max_distance, max(distance_1, max(distance_2, distance_3)));

    if (max_distance == 0) // 传入数据有问题
    {
        return nullptr;
    }

    double max_side_length = 0; // 获取特征的最大边长
    double side_length_1 = max(contour_1->fittedEllipse().size.width, contour_1->fittedEllipse().size.height);
    double side_length_2 = max(contour_2->fittedEllipse().size.width, contour_2->fittedEllipse().size.height);
    double side_length_3 = max(contour_3->fittedEllipse().size.width, contour_3->fittedEllipse().size.height);
    max_side_length = max(max_side_length, max(side_length_1, max(side_length_2, side_length_3)));

    if (max_side_length == 0) // 传入数据有问题
    {
        return nullptr;
    }

    if (max_distance > 3.0 * max_side_length) // 两个特征之间的距离大于两个特征的最大边长的三倍
    {
        return nullptr;
    }

    // 利用顶部突起点的构造判定，并对收集的突起点排序。
    double angle_delta = 1e5;
    bool is_make = TopHump::make_TopHumps(hump_combo, contours_center, angle_delta);
    if (is_make == false)
    {
        return nullptr;
    }

    // 复制需要的轮廓
    vector<Contour_ptr> fan_contours{contour_1, contour_2, contour_3};
    // 获取扇叶的方向
    auto &top_humps = hump_combo.humps();

    Point2f direction = -1 * top_humps[1].getDirection(); // 将顶部中心突起点的方向的反方向作为扇叶的方向,指向神符中心
    // 获取扇叶的顶部突起点
    vector<Point2f> top_humps_corners{};
    for (auto &hump : top_humps)
    {
        top_humps_corners.push_back(hump.getVertex());
    }

    RuneFanActive_ptr fan = make_shared<RuneFanActive>(fan_contours, top_humps_corners, direction);
    if (fan && !fan->isSetError())
    {
        fan->setError(angle_delta);
    }

    return fan;
}
