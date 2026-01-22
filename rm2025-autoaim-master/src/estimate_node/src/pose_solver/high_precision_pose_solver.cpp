//
// Created by wpie on 23-11-22.
//

#include "estimate_node/pose_solver/high_precision_pose_solver.h"

using namespace FP;

std::optional<double> HighPrecisionPoseSolver::Run(const cv::Point3d& armor_pose,
                                                   const double input_gimbal_yaw,
                                                   const double input_gimbal_pitch,
                                                   const double input_gimbal_roll,
                                                   const Contour& image_armor_points,
                                                   const RM::ArmorSize armor_size,
                                                   const double estimated_armor_yaw,
                                                   const bool is_outpost_spin,
                                                   bool use_sencondary_camera,
                                                   const double inclined) {
    return Trichotomy(input_gimbal_yaw, input_gimbal_pitch, input_gimbal_roll, armor_pose, armor_size,
                      image_armor_points, estimated_armor_yaw, is_outpost_spin, use_sencondary_camera, inclined);
}

[[maybe_unused]] double HighPrecisionPoseSolver::ComputeIOU(const HighPrecisionPoseSolver::Contour& cont_1,
                                                            const HighPrecisionPoseSolver::Contour& cont_2,
                                                            const cv::Size& roi) {
    cv::Mat cont_both_image = cv::Mat::zeros(roi, CV_8UC1);

    cv::fillPoly(cont_both_image, std::vector<Contour>{cont_1}, 255);
    cv::fillPoly(cont_both_image, std::vector<Contour>{cont_2}, 255);

    cv::Mat test = cv::Mat::zeros(roi, CV_8UC3);

    //    cv::fillPoly(test, std::vector<Contour>{cont_1}, cv::Scalar(0, 0 ,255));
    //    cv::fillPoly(test, std::vector<Contour>{cont_2}, cv::Scalar(0, 255 ,0));
    cv::drawContours(test, std::vector<Contour>{cont_1}, -1, cv::Scalar(0, 0, 255));
    cv::drawContours(test, std::vector<Contour>{cont_2}, -1, cv::Scalar(0, 255, 0));
    //    cv::imshow("5", test);

    std::vector<Contour> both_contours;
    double cont_1_area = cv::contourArea(cont_1);
    double cont_2_area = cv::contourArea(cont_2);
    cv::findContours(cont_both_image, both_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (both_contours.empty())
        std::terminate();
    double union_area = cv::contourArea(both_contours.at(0));
    double overlap_area = cont_1_area + cont_2_area - union_area;

    return overlap_area / union_area;
}

std::vector<cv::Point2f> HighPrecisionPoseSolver::Reprojection(const double armor_yaw,
                                                               const double gimbal_yaw,
                                                               const double gimbal_pitch,
                                                               const double gimbal_roll,
                                                               const cv::Point3d& t,
                                                               const RM::ArmorSize armor_size,
                                                               const bool is_outpost_spin,
                                                               bool use_sencondary_camera) const {
    cv::Mat T_yaw_to_geographic = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_pitch_to_yaw = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_camera_to_pitch = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_camera_to_geographic = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat T_model_to_camera = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_model_to_geographic = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat T_rotate = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat R_rotate_yaw =
        (cv::Mat_<double>(3, 3) << cos(-armor_yaw), sin(-armor_yaw), 0, -sin(-armor_yaw), cos(-armor_yaw), 0, 0, 0, 1);

    cv::Mat R_rotate_pitch;
    if (is_outpost_spin) {
        R_rotate_pitch = (cv::Mat_<double>(3, 3) << cos(param_.armor_pitch), 0, -sin(param_.armor_pitch), 0, 1, 0,
                          sin(param_.armor_pitch), 0, cos(param_.armor_pitch));
    } else {
        R_rotate_pitch = (cv::Mat_<double>(3, 3) << cos(-param_.armor_pitch), 0, -sin(-param_.armor_pitch), 0, 1, 0,
                          sin(-param_.armor_pitch), 0, cos(-param_.armor_pitch));
    }

    //    cv::Mat R_rotate = R_rotate_pitch * R_rotate_yaw;
    cv::Mat R_rotate = R_rotate_yaw * R_rotate_pitch;
    R_rotate.copyTo(T_rotate(cv::Rect(0, 0, 3, 3)));
    T_model_to_geographic = T_rotate;
    T_rotate.at<double>(0, 3) = t.x;
    T_rotate.at<double>(1, 3) = t.y;
    T_rotate.at<double>(2, 3) = t.z;

    cv::Mat t_yaw_to_geographic =
        (cv::Mat_<double>(3, 1) << param_.yaw_to_geographic_x, param_.yaw_to_geographic_y, param_.yaw_to_geographic_z);
    cv::Mat R_yaw_to_geographic = (cv::Mat_<double>(3, 3) << cos(-gimbal_yaw), sin(-gimbal_yaw), 0, -sin(-gimbal_yaw),
                                   cos(-gimbal_yaw), 0, 0, 0, 1);

    R_yaw_to_geographic.copyTo(T_yaw_to_geographic(cv::Rect(0, 0, 3, 3)));
    t_yaw_to_geographic.copyTo(T_yaw_to_geographic(cv::Rect(3, 0, 1, 3)));

    cv::Mat t_pitch_to_yaw =
        (cv::Mat_<double>(3, 1) << param_.pitch_to_yaw_x, param_.pitch_to_yaw_y, param_.pitch_to_yaw_z);
    cv::Mat R_pitch_to_yaw = (cv::Mat_<double>(3, 3) << cos(-gimbal_pitch), 0, -sin(-gimbal_pitch), 0, 1, 0,
                              sin(-gimbal_pitch), 0, cos(-gimbal_pitch));
    R_pitch_to_yaw.copyTo(T_pitch_to_yaw(cv::Rect(0, 0, 3, 3)));
    t_pitch_to_yaw.copyTo(T_pitch_to_yaw(cv::Rect(3, 0, 1, 3)));

    cv::Mat t_camera_to_pitch;
    if (use_sencondary_camera) {
        t_camera_to_pitch = (cv::Mat_<double>(3, 1) << param_.secondary_camera_to_gimbal_pitch_x,
                             param_.secondary_camera_to_gimbal_pitch_y, param_.secondary_camera_to_gimbal_pitch_z);
    } else {
        t_camera_to_pitch =
            (cv::Mat_<double>(3, 1) << param_.camera_to_pitch_x, param_.camera_to_pitch_y, param_.camera_to_pitch_z);
    }

    cv::Mat R_camera_to_pitch = (cv::Mat_<double>(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0);

    cv::Mat R_camera_to_pitch_install_pitch_offset;
    if (use_sencondary_camera) {
        R_camera_to_pitch_install_pitch_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0,
             -sin(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0, 1, 0,
             sin(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0,
             cos(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset));
    } else {
        R_camera_to_pitch_install_pitch_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0,
             -sin(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0, 1, 0,
             sin(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0,
             cos(-param_.camera_to_gimbal_pitch_install_pitch_offset));
    }

    cv::Mat R_camera_to_pitch_install_yaw_offset;
    if (use_sencondary_camera) {
        R_camera_to_pitch_install_yaw_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset),
             sin(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset), 0,
             -sin(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset),
             cos(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset), 0, 0, 0, 1);
    } else {
        R_camera_to_pitch_install_yaw_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.camera_to_gimbal_pitch_install_yaw_offset),
             sin(-param_.camera_to_gimbal_pitch_install_yaw_offset), 0,
             -sin(-param_.camera_to_gimbal_pitch_install_yaw_offset),
             cos(-param_.camera_to_gimbal_pitch_install_yaw_offset), 0, 0, 0, 1);
    }

    cv::Mat R_roll_offset = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(-gimbal_roll), -sin(-gimbal_roll), 0,
                             sin(-gimbal_roll), cos(-gimbal_roll));

    R_camera_to_pitch = R_roll_offset * R_camera_to_pitch_install_yaw_offset * R_camera_to_pitch_install_pitch_offset *
                        R_camera_to_pitch;
    R_camera_to_pitch.copyTo(T_camera_to_pitch(cv::Rect(0, 0, 3, 3)));
    t_camera_to_pitch.copyTo(T_camera_to_pitch(cv::Rect(3, 0, 1, 3)));

    T_camera_to_geographic = T_yaw_to_geographic * T_pitch_to_yaw * T_camera_to_pitch;

    cv::Point3d armor_l_top{};
    cv::Point3d armor_r_top{};
    cv::Point3d armor_l_bottom{};
    cv::Point3d armor_r_bottom{};

    if (armor_size == RM::ArmorSize::LARGE) {
        armor_l_top = cv::Point3f(0, (float)param_.large_armor_width / 2, (float)param_.large_armor_height / 2);
        armor_r_top = cv::Point3f(0, (float)-param_.large_armor_width / 2, (float)param_.large_armor_height / 2);
        armor_l_bottom = cv::Point3f(0, (float)param_.large_armor_width / 2, (float)-param_.large_armor_height / 2);
        armor_r_bottom = cv::Point3f(0, (float)-param_.large_armor_width / 2, (float)-param_.large_armor_height / 2);
    } else if (armor_size == RM::ArmorSize::SMALL) {
        armor_l_top = cv::Point3f(0, (float)param_.small_armor_width / 2, (float)param_.small_armor_height / 2);
        armor_r_top = cv::Point3f(0, (float)-param_.small_armor_width / 2, (float)param_.small_armor_height / 2);
        armor_l_bottom = cv::Point3f(0, (float)param_.small_armor_width / 2, (float)-param_.small_armor_height / 2);
        armor_r_bottom = cv::Point3f(0, (float)-param_.small_armor_width / 2, (float)-param_.small_armor_height / 2);
    } else {
        std::cout<< "HighPrecisionPoseSolver: invalid armor size" << std::endl;
        std::terminate();
    }

    auto armor_point_vec = std::vector<cv::Point3f>{armor_l_bottom, armor_l_top, armor_r_top, armor_r_bottom};
    std::for_each(armor_point_vec.begin(), armor_point_vec.end(),
                  [&T_model_to_geographic, &T_camera_to_geographic](cv::Point3f& p) {
                      cv::Mat point_mat = (cv::Mat_<double>(4, 1) << p.x, p.y, p.z,
                                           1.0);  // �?�?为齐次左边坐标进行坐标转�?

                      //                      std::cout << t << std::endl;

                      point_mat = T_model_to_geographic * point_mat;

                      //                      std::cout << T_model_to_geographic << std::endl;

                      //                      std::cout << point_mat << std::endl;
                      point_mat = T_camera_to_geographic.inv() * point_mat;

                      //                      std::cout << point_mat << std::endl;
                      //                      std::cout << "______" << std::endl;
                      //                      assert(0);

                      // �?�?为mm
                      p.x = static_cast<float>(point_mat.at<double>(0) * 1000);
                      p.y = static_cast<float>(point_mat.at<double>(1) * 1000);
                      p.z = static_cast<float>(point_mat.at<double>(2) * 1000);
                      //                      int i = 1;
                      //                      p.x = static_cast<float>(point_mat.at<double>(0) * 1);
                      //                      p.y = static_cast<float>(point_mat.at<double>(1) * 1);
                      //                      p.z = static_cast<float>(point_mat.at<double>(2) * 1);
                  });

    Contour output;
    std::vector<cv::Point2f> image_p;

    cv::projectPoints(
        armor_point_vec, cv::Mat::zeros(3, 1, CV_32F), cv::Mat::zeros(3, 1, CV_32F),
        use_sencondary_camera ? param_.secondary_camera_intrinsics_mm : param_.camera_intrinsics_mm,
        use_sencondary_camera ? param_.secondary_camera_distortion_coefficient : param_.distortion_coefficient,
        image_p);
    return image_p;
}

std::optional<double> HighPrecisionPoseSolver::Trichotomy(double gimbal_yaw,
                                                          double gimbal_pitch,
                                                          double gimbal_roll,
                                                          const cv::Point3d& t,
                                                          RM::ArmorSize armor_size,
                                                          const Contour& image_cont,
                                                          double estimated_armor_yaw,
                                                          const bool is_outpost_spin,
                                                          bool use_sencondary_camera,
                                                          const double inclined) const {
    double left = angles::normalize_angle(estimated_armor_yaw + param_.trichotomy_left_offset);
    double right = angles::normalize_angle(estimated_armor_yaw + param_.trichotomy_right_offset);

    //    std::cout << "left " << angles::to_degrees(left) << " right " << angles::to_degrees(right) << std::endl;

    int count = 0;
    //    int count_1 = 0;
    //    int count_2 = 0;
    while (count <= param_.trichotomy_max_iterations) {
        double mid_1 = angles::normalize_angle(left + angles::shortest_angular_distance(left, right) / 3);
        double mid_2 = angles::normalize_angle(right - angles::shortest_angular_distance(left, right) / 3);

        std::vector<cv::Point2f> image_cont_f{image_cont.at(0), image_cont.at(1), image_cont.at(2), image_cont.at(3)};
        auto re_mid1 = Reprojection(mid_1, gimbal_yaw, gimbal_pitch, gimbal_roll, t, armor_size, is_outpost_spin,
                                    use_sencondary_camera);
        auto re_mid2 = Reprojection(mid_2, gimbal_yaw, gimbal_pitch, gimbal_roll, t, armor_size, is_outpost_spin,
                                    use_sencondary_camera);
        auto a = std::vector<cv::Point>{re_mid1.at(0), re_mid1.at(1), re_mid1.at(2), re_mid1.at(3)};
        auto b = std::vector<cv::Point>{re_mid2.at(0), re_mid2.at(1), re_mid2.at(2), re_mid2.at(3)};

        //        ComputeIOU(a , image_cont, cv::Size (1280, 1024));
        //        std::unique_ptr<int> p = std::make_unique<int>(0);
        //        getchar();
        //        ComputeIOU(b , image_cont, cv::Size (1280, 1024));
        //        getchar();

        auto sort_1 = SJTU_get_pts_cost(re_mid1, image_cont_f, inclined);
        auto sort_2 = SJTU_get_pts_cost(re_mid2, image_cont_f, inclined);

        //        std::cout << "this time" << angles::to_degrees(mid_1) << " " << angles::to_degrees(mid_2) <<
        //        std::endl; std::cout << "value" << sort_1 << " " << sort_2 << std::endl;

        if (sort_1 > sort_2) {
            left = mid_1;
            //            count_1++;
        } else {
            right = mid_2;
            //            count_2++;
        }

        if (abs(angles::shortest_angular_distance(left, right)) < param_.trichotomy_termination_accuracy)
            break;
        count++;
    }
    //    std::cout << count_1 << " " << count_2 << std::endl;

    if (count <= param_.trichotomy_max_iterations) {
        return angles::normalize_angle((left + right) / 2);
    } else {
        return std::nullopt;
    }
}

double HighPrecisionPoseSolver::SJTU_get_pts_cost(const std::vector<cv::Point2f>& cv_refs,
                                                  const std::vector<cv::Point2f>& cv_pts,
                                                  const double& inclined) {
    auto get_abs_angle = [](Eigen::Vector2d a, Eigen::Vector2d b) {
        double a_angle = atan2(a.y(), a.x());
        double b_angle = atan2(b.y(), b.x());
        return std::abs(angles::shortest_angular_distance(a_angle, b_angle));
    };

    std::size_t size = cv_refs.size();
    std::vector<Eigen::Vector2d> refs;
    std::vector<Eigen::Vector2d> pts;
    for (std::size_t i = 0u; i < size; ++i) {
        refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
        pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
    }
    double cost = 0.;
    for (std::size_t i = 0u; i < size; ++i) {
        std::size_t p = (i + 1u) % size;
        // i - p 构成线�?�。过程：先移动起点，再补长度，再旋转
        Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
        Eigen::Vector2d pt_d = pts[p] - pts[i];
        // 长度�?代价 + 起点�?代价(1 / 2)�?0 度左右应该抛�?)
        double pixel_dis =  // dis �?指方�?平面内到原点的距�?
            (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) + std::fabs(ref_d.norm() - pt_d.norm())) /
            ref_d.norm();
        double angular_dis = ref_d.norm() * get_abs_angle(ref_d, pt_d) / ref_d.norm();
        // 平方�?能是为了配合 sin �? cos
        // 弧度�?代价�?0 度左右占比应该大�?
        double cost_i = pow(pixel_dis * std::sin(inclined), 2) + pow(angular_dis * std::cos(inclined), 2) * 2.;
        // 重投影像素�??�?越大，越相信斜率
        cost += std::sqrt(cost_i);
    }
    return cost;
}
