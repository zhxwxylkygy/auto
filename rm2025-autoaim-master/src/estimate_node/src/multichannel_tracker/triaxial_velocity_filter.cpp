#include "estimate_node/multichannel_tracker/static_tracker/triaxial_velocity_filter.h"
#include <cfloat>
#include <memory>
#include <string>
#include "angles/angles.h"
#include "calculate/basic_calculate.h"


namespace MT::ST {
TriaxialVelocityFilter::TriaxialVelocityFilter() {
    auto f = [this](const Eigen::VectorXd& x) {
        auto d_t = ekf_formula_server_.d_t;
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * d_t;
        x_new(2) += x(3) * d_t;
        x_new(4) += x(5) * d_t;
        return x_new;
    };

    auto jacobian_f = [this](const Eigen::VectorXd&) {
        auto d_t = ekf_formula_server_.d_t;
        Eigen::MatrixXd f(6, 6);

        f << 1, d_t, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, d_t, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, d_t, 0, 0, 0, 0,
            0, 1;

        return f;
    };

    auto h = [](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(3);
        double xa = x(0), ya = x(2), za = x(4);

        double distance = std::sqrt(pow(xa, 2) + pow(ya, 2) + pow(za, 2));
        z(0) = distance;                  // dis
        z(1) = std::acos(za / distance);  // theta
        z(2) = atan2(ya, xa);             // phi

        return z;
    };

    auto jacobian_h = [this](const Eigen::VectorXd& x_vec) {
        Eigen::MatrixXd h(3, 6);  // Jacobian matrix

        double x[6] = {x_vec(0), x_vec(1), x_vec(2), x_vec(3), x_vec(4), x_vec(5)};
        double* x_blocks[1] = {x};
        double residuals[3];
        double jacobians[18] = {};
        double* jacobians_blocks[1] = {jacobians};
        h_cost_function->Evaluate(x_blocks, residuals, jacobians_blocks);
        h << jacobians[0], jacobians[1], jacobians[2], jacobians[3], jacobians[4], jacobians[5], jacobians[6],
            jacobians[7], jacobians[8], jacobians[9], jacobians[10], jacobians[11], jacobians[12], jacobians[13],
            jacobians[14], jacobians[15], jacobians[16], jacobians[17];

        //            std::cout << h << std::endl;
        //            assert(0);
        return h;
    };

    ekf_formula_server_.q_xyz_factor_2 = pow(param_.q_xyz_factor, 2);
    auto update_q = [this]() {
        Eigen::MatrixXd q(6, 6);
        auto d_t = ekf_formula_server_.d_t;
        double t = d_t, x = ekf_formula_server_.q_xyz_factor_2;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;

        //      xc          v_xc        yc      v_yc        za      v_za        yaw     v_yaw       r
        q << q_x_x, q_x_vx, 0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, q_x_vx, q_vx_vx,
            0, 0, 0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, q_x_vx, q_vx_vx;

        return q;
    };

    ekf_formula_server_.r_distance_factor_2 = pow(param_.r_distance_factor, 2);
    ekf_formula_server_.r_theta_2 = pow(param_.r_theta, 2);
    ekf_formula_server_.r_phi_2 = pow(param_.r_phi, 2);
    auto update_r = [this](const Eigen::VectorXd& z) {
        Eigen::DiagonalMatrix<double, 3> r;
        r.diagonal() << ekf_formula_server_.r_distance_factor_2 * z[0] * z[0], ekf_formula_server_.r_theta_2,
            ekf_formula_server_.r_phi_2;
        return r;
    };

    Eigen::DiagonalMatrix<double, 6> p_0;
    p_0.setIdentity();
    ekf_.LoadFormula(f, h, jacobian_f, jacobian_h, update_q, update_r, p_0);
}

void TriaxialVelocityFilter::Init(const std::vector<FP::ArmorInfo>& armors) {
    if (armors.empty()) {
        return;
    }

    tracked_armor_ = armors[0];

    initEKF(tracked_armor_);
    //        std::cout << "[armor_tracker]" << "Init EKF!" << std::endl;

    tracked_id_ = tracked_armor_.armor_id;
    tracker_state_ = DETECTING;
}

void TriaxialVelocityFilter::Update(const std::vector<FP::ArmorInfo>& armors, double d_t) {
    lost_count_threshold_ = static_cast<int>(param_.lost_time_threshold / d_t);
    ekf_formula_server_.d_t = d_t;

    // KF predict
    Eigen::VectorXd ekf_prediction = ekf_.Predict();
    //        Eigen::Vector3d dtp = {ekf_prediction(0), ekf_prediction(1), ekf_prediction(2)};
    //        Eigen::Vector3d xyz = DTP2XYZ(dtp);
    //        Eigen::Vector4d xyz_yaw = {xyz(0), xyz(1), xyz(2), ekf_prediction(3)};

    //        Eigen::Vector3d ekf_prediction_xyz =
    //        ekf_prediction = DTP2XYZ(ekf_prediction);

    bool matched = false;
    // Use KF prediction as default target_classify state if no matched armor is found
    target_state_ = ekf_prediction;

    if (!armors.empty()) {
        // Find the closest armor with the same id
        auto predicted_position = getArmorPositionFromState(ekf_prediction);
        auto min_position_diff = DBL_MAX;

        for (const auto& armor : armors) {
            // Only consider armors with the same id
            if (armor.armor_id == tracked_id_) {
                // Calculate the difference between the predicted position and the current armor position
                auto p = armor.geographic_coord_pose;

                Eigen::Vector3d position_vec(p.x, p.y, p.z);
                double position_diff = (predicted_position - position_vec).norm();

                if (position_diff < min_position_diff) {
                    // Find the closest armor
                    min_position_diff = position_diff;
                    tracked_armor_ = armor;
                }
            }
        }

        if (min_position_diff < param_.max_match_distance_diff) {
            matched = true;
            auto p = tracked_armor_.geographic_coord_pose;
            Eigen::Vector3d p_vec = {p.x, p.y, p.z};

            auto dtp = XYZ2DTP(p_vec);
            dtp.y() = SerializeThetaAngle(dtp.y());
            auto measurement = Eigen::Vector3d(dtp.x(), dtp.y(), dtp.z());

            target_state_ = ekf_.Update(measurement);

        } else {
            // No matched armor found
            std::cout << "[armor_tracker]"
                      << "No matched armor found!" << std::endl;
        }
    }

    // Tracking state machine
    if (tracker_state_ == DETECTING) {
        if (matched) {
            detect_count_++;
            if (detect_count_ > param_.tracking_count_threshold) {
                detect_count_ = 0;
                tracker_state_ = TRACKING;
            }
        } else {
            detect_count_ = 0;
            tracker_state_ = LOST;
        }
    } else if (tracker_state_ == TRACKING) {
        if (!matched) {
            tracker_state_ = TEMP_LOST;
            lost_count_++;
        }
    } else if (tracker_state_ == TEMP_LOST) {
        if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_count_threshold_) {
                lost_count_ = 0;
                tracker_state_ = LOST;
            }
        } else {
            tracker_state_ = TRACKING;
            lost_count_ = 0;
        }
    }
}

void TriaxialVelocityFilter::initEKF(const FP::ArmorInfo& a) {
    double xa = a.geographic_coord_pose.x;
    double ya = a.geographic_coord_pose.y;
    double za = a.geographic_coord_pose.z;
    last_theta_ = 0;
    target_state_ = Eigen::VectorXd::Zero(6);
    target_state_ << xa, 0, ya, 0, za, 0;
    ekf_.SetState(target_state_);
}

double TriaxialVelocityFilter::SerializeThetaAngle(double theta) {
    theta = last_theta_ + angles::shortest_angular_distance(last_theta_, theta);
    last_theta_ = theta;
    return theta;
}

Eigen::Vector3d TriaxialVelocityFilter::getArmorPositionFromState(const Eigen::VectorXd& x) {
    // Calculate predicted position of the current armor
    double xa = x(0), ya = x(2), za = x(4);
    return {xa, ya, za};
}

}  // namespace MT::ST
