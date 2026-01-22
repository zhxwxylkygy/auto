//
// Created by wpie on 24-1-21.
//

#pragma once

#include <eigen3/Eigen/Eigen>
#include <memory>
#include <string>
#include "ceres/ceres.h"

#include "angles/angles.h"
#include "calculate/basic_calculate.h"
#include "calculate/extended_kalman_filter.hpp"
#include "estimate_node/interface/interface.h"
#include "param_loader.hpp"


namespace MT::PT {

class SSPRFilter {
   public:
    enum class FilterState {
        LOST,
        DIR_JUDGE,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    };

    SSPRFilter();

    void Init(const std::shared_ptr<FP::FanInfo>& input_fan);

    std::tuple<Eigen::VectorXd, bool> Update(const std::vector<std::shared_ptr<FP::FanInfo>>& input_fans, double d_t);

    void DirJudge(const std::vector<std::shared_ptr<FP::FanInfo>>& input_fans, double input_d_t);

    FilterState GetFilterState();

    void SetFilterState(FilterState input_state);

   private:
    void InitEKF(const std::shared_ptr<FP::FanInfo>& input_armor);

    void HandleFanJump(const std::shared_ptr<FP::FanInfo>& input_fan, Eigen::VectorXd& target_pri_state);

    static double SerializeAngle(double input_now_angle, double& last_angle);

    // void EKFStateLimit(Eigen::VectorXd& target_non_finite_state);

    static std::tuple<double, double> GetRollAndRadiusFromPose(const cv::Point3d& bull_pose,
                                                               const cv::Point3d& r_symbol_pose);

    static cv::Point3d GetBullFromRollAndRadius(const double& input_roll,
                                                const double& input_radius,
                                                const cv::Point3d& r_symbol_pose);

    struct Param {
        double q_roll = ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "Q_ROLL_FACTOR");
        double q_centre_xyz = ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "Q_CENTRE_XYZ");
        double r_roll = ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "R_ROLL");
        double r_centre_distance = ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "R_CENTRE_DISTANCE");
        double r_centre_theta = ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "R_CENTRE_THETA");
        double r_centre_phi = ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "R_CENTRE_PHI");

        const double max_match_roll_dev = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "MAX_MATCH_ROLL_DEV_DEGREE"));

        const double lost_duration_threshold =
            ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "LOST_DURATION_THRESHOLD");
        const double tracking_count_threshold =
            ParamLoader::GetInstance().GetParam<int>("MT", "PT", "SSPR", "TRACKING_COUNT_THRESHOLD");

        const double dir_judge_min_v_roll_degree = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("MT", "PT", "SSPR", "DIR_JUDGE_MIN_V_ROLL_DEGREE"));
        const int dir_judge_deque_size =
            ParamLoader::GetInstance().GetParam<int>("MT", "PT", "SSPR", "DIR_JUDGE_DEQUE_SIZE");
        const int dir_judge_valid_value_count_threshold =
            ParamLoader::GetInstance().GetParam<int>("MT", "PT", "SSPR", "DIR_JUDGE_VALID_VALUE_COUNT_THRESHOLD");
    } param_;

    struct EKFHCostFunction {
        explicit EKFHCostFunction(SSPRFilter* input_ef) : input_ef_(input_ef) {}

        bool operator()(const double* x, double* residual) const;

       private:
        SSPRFilter* input_ef_;
    };

    ceres::CostFunction* h_cost_function =
        new ceres::NumericDiffCostFunction<EKFHCostFunction, ceres::RIDDERS, 4, 5>(new EKFHCostFunction(this));

    std::shared_ptr<FP::FanInfo> tracked_fan_;

    struct {
        double r_energy_roll_2;
        double r_centre_distance_factor_2;
        double r_centre_theta_2;
        double r_centre_phi_2;
        double q_energy_roll_2;
        double q_centre_xyz_2;

        double d_t = 0;
    } ekf_formula_server_{};

    enum class DirModeUnit { CLOCKWISE, ANTICLOCKWISE, UNCERTAIN };

    ExtendedKalmanFilter ekf_{};

    FilterState filter_state_ = FilterState::LOST;

    std::deque<DirModeUnit> dir_judge_deque_;
    int lost_count_threshold_{};
    int detect_count_ = 0;
    int lost_count_ = 0;

    double last_dir_judge_roll_ = 0;
    double last_measure_power_rune_roll_ = 0;
    double last_measure_r_symbol_phi_ = 0;
    bool is_clockwise_ = false;
};
}  // namespace MT::PT
