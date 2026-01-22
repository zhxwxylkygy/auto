// Copyright 2022 Chen Jun

#pragma once
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <string>
#include "ceres/ceres.h"
#include "estimate_node/interface/interface.h"
#include "param_loader.hpp"
#include "calculate/extended_kalman_filter.hpp"

namespace MT::IRVT {

//    enum class ArmorsNum {
//        NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3
//    };

class IRVFilter {
   public:
    IRVFilter() = delete;

    IRVFilter(bool is_hero);

    void Init(const std::vector<std::shared_ptr<FP::ArmorInfo>>& input_armors,
              RM::TargetArmorNum input_target_armor_num);

    void Update(const std::vector<std::shared_ptr<FP::ArmorInfo>>& input_armors, double d_t);

    ExtendedKalmanFilter ekf_{};

    enum State {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } tracker_state_ = LOST;

    RM::ArmorId tracked_id_ = RM::ArmorId::BASE;
    std::shared_ptr<FP::ArmorInfo> matched_armor_{};
    RM::TargetArmorNum target_armor_num_{};

    Eigen::VectorXd target_state = Eigen::VectorXd::Zero(9);

    double d_z_{}, another_r_{};

    int kalman_score_ = 0;

   private:
    static int GetKalmanScore(Eigen::MatrixXd p_post);

    void InitEKF(const std::shared_ptr<FP::ArmorInfo>& input_armor);

    void UpdateArmorsNum(RM::TargetArmorNum input_target_armor_num);

    void HandleArmorJump(const std::shared_ptr<FP::ArmorInfo>& input_armor);

    double SerializeArmorYawAngle(double yaw);

    double SerializePhiAngle(double phi);

    static Eigen::Vector3d GetArmorPositionFromState(const Eigen::VectorXd& x);

    struct Param {
        const double q_xyz = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "Q_XYZ_FACTOR");
        const double q_yaw = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "Q_YAW");
        const double q_r = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "Q_R");

        const double r_distance_factor = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "R_DISTANCE_FACTOR");
        const double r_theta = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "R_THETA");
        const double r_phi = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "R_PHI");
        const double r_yaw = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "R_YAW");

        const double hero_q_xyz = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "HERO_Q_XYZ_FACTOR");
        const double hero_q_yaw = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "HERO_Q_YAW");
        const double hero_q_r = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "HERO_Q_R");

        const double hero_r_distance_factor =
            ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "HERO_R_DISTANCE_FACTOR");
        const double hero_r_theta = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "HERO_R_THETA");
        const double hero_r_phi = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "HERO_R_PHI");
        const double hero_r_yaw = ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "HERO_R_YAW");

        const double max_match_distance_diff =
            ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "MAX_MATCH_DISTANCE_DIFF");
        const double max_match_yaw_diff =
            ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "MAX_MATCH_YAW_DIFF");

        const int tracking_count_threshold =
            ParamLoader::GetInstance().GetParam<int>("MT", "IRVT", "TRACKING_COUNT_THRESHOLD");
        const double lost_duration_threshold =
            ParamLoader::GetInstance().GetParam<double>("MT", "IRVT", "LOST_DURATION_THRESHOLD");

    } param_;

    // struct EKFHCostFunction {
    //     bool operator()(const double* const x, double* residual) const {
    //         double xc = x[0], yc = x[2], zc = x[4], yaw = x[6], r = x[8];
    //         double xa = xc - r * cos(yaw);
    //         double ya = yc - r * sin(yaw);
    //         double za = zc;
    //         double distance = std::sqrt(pow(xa, 2) + pow(ya, 2) + pow(za, 2));
    //         residual[0] = distance;                  // a_distance
    //         residual[1] = std::acos(za / distance);  // a_theta
    //         residual[2] = atan2(ya, xa);             // a_phi
    //         residual[3] = yaw;                       // yaw
    //         return true;
    //     }
    // };

    struct EKFHCostFunctionA {
        template <typename T>
        bool operator()(const T* x, T* residual) const {
            const T xc = x[0], yc = x[2], zc = x[4], yaw = x[6], r = x[8];
            const T xa = xc - r * ceres::cos(yaw);
            const T ya = yc - r * ceres::sin(yaw);
            const T za = zc;
            const T distance = ceres::sqrt(ceres::pow(xa, 2) + ceres::pow(ya, 2) + ceres::pow(za, 2));
            residual[0] = distance;                    // a_distance
            residual[1] = ceres::acos(za / distance);  // a_theta
            residual[2] = ceres::atan2(ya, xa);        // a_phi
            residual[3] = yaw;                         // yaw
            return true;
        }
    } ekf_h_cost_function_a;

    // ceres::CostFunction* h_cost_function =
    //     new ceres::NumericDiffCostFunction<EKFHCostFunction, ceres::RIDDERS, 4, 9>(new EKFHCostFunction);

    ceres::CostFunction* h_cost_function_a =
        new ceres::AutoDiffCostFunction<EKFHCostFunctionA, 4, 9>(new EKFHCostFunctionA);

    struct {
        double r_yaw_2;
        double r_distance_factor_2;
        double r_theta_2;
        double r_phi_2;
        double q_xyz_2;
        double q_yaw_2;
        double q_r_2;
        double d_t = 0;
    } ekf_formula_server_{};

    int lost_count_threshold_{};

    int detect_count_ = 0;
    int lost_count_ = 0;

    double last_armor_yaw_ = 0;
    double last_phi_ = 0;

    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h_func_;


    //sjtu magic

    inline double reduced_angle(const double& x) { return std::atan2(std::sin(x), std::cos(x)); }

    inline double reduced(const double& x, const double& range) {
        double times = range / (2. * M_PI);
        return times * (this->reduced_angle(x / times - M_PI) + M_PI);
    }

    inline double get_closest(const double& cur, const double& tar, const double& period) {
        double reduced = this->reduced(cur, period);
        double possibles[3] = {reduced - period, reduced, reduced + period};
        double closest = possibles[0];
        for (double possible : possibles) {
            if (std::fabs(tar - possible) < std::fabs(tar - closest)) {
                closest = possible;
            }
        }
        return closest;
    }
};
}  // namespace MT::IRVT
