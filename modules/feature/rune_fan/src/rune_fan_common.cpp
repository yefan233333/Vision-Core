// #include <numeric>
// #include "vc/feature/rune_fan.h"
// #include "vc/feature/rune_fan_active.h"
// #include "vc/feature/rune_fan_inactive.h"
// #include "vc/feature/rune_fan_param.h"

// using namespace std;
// using namespace cv;

// #define rune_fan_debug_0

// #define rune_fan_show

// std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> RuneFan::getPnpPoints() const
// {
//     vector<Point2f> points_2d{};
//     vector<Point3f> points_3d{};
//     vector<float> weights{};
//     if (this->getActiveFlag())
//     {
//         auto this_ptr = shared_from_this();
//         const auto fan_ptr = RuneFanActive::cast(this_ptr);

//         if (isSetTopHumpCorners())
//         {
//             auto &corners = getTopHumpCorners();
//             points_2d.insert(points_2d.end(), corners.begin(), corners.end());
//             points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_TOP_3D.begin(), rune_fan_param.ACTIVE_TOP_3D.end());
//         }
//         if (isSetBottomCenterHumpCorners())
//         {
//             auto &corners = getBottomCenterHumpCorners();
//             points_2d.insert(points_2d.end(), corners.begin(), corners.end());
//             points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_BOTTOM_CENTER_3D.begin(), rune_fan_param.ACTIVE_BOTTOM_CENTER_3D.end());
//         }
//         if (isSetSideHumpCorners())
//         {
//             auto &corners = getSideHumpCorners();
//             points_2d.insert(points_2d.end(), corners.begin(), corners.end());
//             points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_SIDE_3D.begin(), rune_fan_param.ACTIVE_SIDE_3D.end());
//         }
//         if (isSetBottomSideHumpCorners())
//         {
//             auto &corners = getBottomSideHumpCorners();
//             points_2d.insert(points_2d.end(), corners.begin(), corners.end());
//             points_3d.insert(points_3d.end(), rune_fan_param.ACTIVE_BOTTOM_SIDE_3D.begin(), rune_fan_param.ACTIVE_BOTTOM_SIDE_3D.end());
//         }
//     }
//     else
//     {
//         // points_2d.insert(points_2d.end(), __corners.begin(), __corners.end());
//         // points_3d.insert(points_3d.end(), rune_fan_param.INACTIVE_3D.begin(), rune_fan_param.INACTIVE_3D.end());
//         points_2d.push_back(__corners[0]);
//         points_2d.push_back(__corners[1]);
//         points_3d.push_back(rune_fan_param.INACTIVE_3D[0]);
//         points_3d.push_back(rune_fan_param.INACTIVE_3D[1]);
//     }
//     if (points_2d.size() != points_3d.size())
//     {
//         SRVL_Error(SRVL_StsBadSize, "The size of points_2d and points_3d must be equal");
//     }
//     weights.resize(points_2d.size(), 1.0);

//     return make_tuple(points_2d, points_3d, weights);
// }

// std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point3f>, std::vector<float>> RuneFan::getRelativePnpPoints() const
// {
//     auto [points_2d, points_3d, weights] = this->getPnpPoints();
//     vector<Point3f> relative_points_3d(points_3d.size());
//     for (int i = 0; i < points_3d.size(); i++)
//     {
//         Matx31f points_3d_mat(points_3d[i].x, points_3d[i].y, points_3d[i].z);
//         Matx31f relative_points_3d_mat{};
//         if (this->isActive())
//         {
//             relative_points_3d_mat = rune_fan_param.ACTIVE_ROTATION * points_3d_mat + rune_fan_param.ACTIVE_TRANSLATION;
//         }
//         else
//         {
//             relative_points_3d_mat = rune_fan_param.INACTIVE_ROTATION * points_3d_mat + rune_fan_param.INACTIVE_TRANSLATION;
//         }
//         relative_points_3d[i] = Point3f(relative_points_3d_mat(0), relative_points_3d_mat(1), relative_points_3d_mat(2));
//     }
//     return make_tuple(points_2d, relative_points_3d, weights);
// }

// // std::shared_ptr<RuneFan> RuneFan::make_feature(const Point2f& top_left,
// //                                                 const Point2f& top_right,
// //                                                 const Point2f& bottom_right,
// //                                                 const Point2f& bottom_left)

// /**
//  * @brief 检查坐标的范围是否正常
//  *
//  * @param points 待检查的坐标
//  * @return true 坐标范围正常
//  */
// bool checkPoints(const vector<Point2f> &points)
// {
//     static const float MAX_X = 5000;
//     static const float MAX_Y = 5000;

//     for (const auto &point : points)
//     {
//         if (abs(point.x) > MAX_X || abs(point.y) > MAX_Y)
//         {
//             return false;
//         }
//     }
//     return true;
// }

// std::shared_ptr<RuneFan> RuneFan::make_feature(const ResultPnP<float> &fan_to_cam, bool is_active)
// {
//     RuneF_ptr result_ptr;
//     if (is_active)
//     {
//         vector<Point3f> top_hump_corners{};
//         vector<Point3f> bottom_center_hump_corners{};
//         vector<Point3f> side_hump_corners{};
//         vector<Point3f> bottom_side_hump_corners{};

//         top_hump_corners = rune_fan_param.ACTIVE_TOP_3D;
//         bottom_center_hump_corners = rune_fan_param.ACTIVE_BOTTOM_CENTER_3D;
//         side_hump_corners = rune_fan_param.ACTIVE_SIDE_3D;
//         bottom_side_hump_corners = rune_fan_param.ACTIVE_BOTTOM_SIDE_3D;

//         // 重投影
//         vector<Point2f> top_hump_corners_2d{};
//         vector<Point2f> bottom_center_hump_corners_2d{};
//         vector<Point2f> side_hump_corners_2d{};
//         vector<Point2f> bottom_side_hump_corners_2d{};

//         projectPoints(top_hump_corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, top_hump_corners_2d);
//         projectPoints(bottom_center_hump_corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, bottom_center_hump_corners_2d);
//         projectPoints(side_hump_corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, side_hump_corners_2d);
//         projectPoints(bottom_side_hump_corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, bottom_side_hump_corners_2d);

//         // 检查坐标范围
//         if (checkPoints(top_hump_corners_2d) == false || checkPoints(bottom_center_hump_corners_2d) == false || checkPoints(side_hump_corners_2d) == false || checkPoints(bottom_side_hump_corners_2d) == false)
//         {
//             return nullptr;
//         }

//         result_ptr = RuneFan::make_feature(top_hump_corners_2d, bottom_center_hump_corners_2d, side_hump_corners_2d, bottom_side_hump_corners_2d);
//     }
//     else
//     {
//         vector<Point3f> corners = rune_fan_param.INACTIVE_3D;
//         vector<Point2f> corners_2d{};
//         // 重投影
//         projectPoints(corners, fan_to_cam.rvec(), fan_to_cam.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, corners_2d);

//         // 检查坐标范围
//         if (checkPoints(corners_2d) == false)
//         {
//             return nullptr;
//         }

//         result_ptr = RuneFan::make_feature(corners_2d[0], corners_2d[1], corners_2d[2], corners_2d[3]);
//     }
//     if (result_ptr)
//     {
//         const auto &temp_ptr = result_ptr;
//         temp_ptr->setCamPnpData(fan_to_cam);
//         // result_ptr->setCamPnpData(fan_to_cam);
//     }
//     return result_ptr;
// }

// bool RuneFan::correct(std::shared_ptr<RuneFan> &fan, const cv::Point2f &rune_center)
// {
//     if (fan == nullptr)
//     {
//         return false;
//     }
//     if (fan->isActive())
//     {
//         if (correctDirection(fan, rune_center) == false)
//         {
//             return false;
//         }
//         if (correctCorners(fan) == false)
//         {
//             return false;
//         }
//     }

//     return true;
// }