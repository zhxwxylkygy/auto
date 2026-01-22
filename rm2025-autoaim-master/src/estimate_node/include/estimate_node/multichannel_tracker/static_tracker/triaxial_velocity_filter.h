#pragma once

#include <ceres/ceres.h>
#include <ceres/types.h>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <string>
#include "estimate_node/interface/interface.h"
#include "param_loader.hpp"
#include "calculate/extended_kalman_filter.hpp"

namespace MT::ST {

class TriaxialVelocityFilter {
   public:
    TriaxialVelocityFilter();

    void Init(const std::vector<FP::ArmorInfo>& armors);

    void Update(const std::vector<FP::ArmorInfo>& armors, double d_t);

    ExtendedKalmanFilter ekf_{};

    enum State {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } tracker_state_ = LOST;

    RM::ArmorId tracked_id_ = RM::ArmorId::BASE;
    FP::ArmorInfo tracked_armor_{};

    Eigen::VectorXd target_state_ = Eigen::VectorXd::Zero(6);

   private:
    struct Param {
        const double max_match_distance_diff =
            ParamLoader::GetInstance().GetParam<double>("MT","ST","MAX_MATCH_DISTANCE_DIFF");
        const int tracking_count_threshold = ParamLoader::GetInstance().GetParam<int>("MT","ST","TRACKING_COUNT_THRESHOLD");
        const double lost_time_threshold = ParamLoader::GetInstance().GetParam<double>("MT","ST","LOST_DURATION_THRESHOLD");

        const double q_xyz_factor = ParamLoader::GetInstance().GetParam<double>("MT","ST","Q_XYZ_FACTOR");

        const double r_distance_factor = ParamLoader::GetInstance().GetParam<double>("MT","ST","R_DISTANCE_FACTOR");
        const double r_theta = ParamLoader::GetInstance().GetParam<double>("MT","ST","R_THETA");
        const double r_phi = ParamLoader::GetInstance().GetParam<double>("MT","ST","R_PHI");

    } param_;

    struct EKFHCostFunction {
        bool operator()(const double* const x, double* residual) const {
            double xa = x[0], ya = x[2], za = x[4];

            double distance = std::sqrt(pow(xa, 2) + pow(ya, 2) + pow(za, 2));
            residual[0] = distance;                  // a_distance
            residual[1] = std::acos(za / distance);  // a_theta
            residual[2] = atan2(ya, xa);             // a_phi

            return true;
        }
    };

    ceres::CostFunction* h_cost_function =
        new ceres::NumericDiffCostFunction<EKFHCostFunction, ceres::RIDDERS, 3, 6>(new EKFHCostFunction);

    void initEKF(const FP::ArmorInfo& a);

    double SerializeThetaAngle(double theta);

    static Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x);

    struct {
        double q_xyz_factor_2;
        double r_distance_factor_2;
        double r_theta_2;
        double r_phi_2;
        double d_t = 0;
    } ekf_formula_server_{};

    int lost_count_threshold_{};

    int detect_count_ = 0;
    int lost_count_ = 0;

    double last_theta_ = 0;
};
}  // namespace MT::ST
