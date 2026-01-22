//
// Created by wpie on 23-10-25.
//

#include "trajectory_calculate.h"
#include <cmath>
#include <iostream>

void Trajectory::Init(Trajectory::Mode mode, double bullet_speed) {
    has_init = true;
    if (mode == Mode::BULLET_17MM_AVER_K) {
        param_variant_.emplace<AverKParam>();
        auto& p = std::get<AverKParam>(param_variant_);
        p.bullet_speed = bullet_speed;

    } else if (mode == Mode::BULLET_42MM_SECOND_ORDER_APPROX) {
        param_variant_.emplace<SecondOrderIterationParam>();
        auto& p = std::get<SecondOrderIterationParam>(param_variant_);
        p.rongo_kuta_param.bullet_speed = bullet_speed;
        p.rongo_kuta_param.m = 0.0445;
        p.rongo_kuta_param.f_coefficient = 0.00043;
        p.rongo_kuta_param.t_h = 0.003;
        iterator_variant_.emplace<SecondOrderApproxIterator>();
    } else if (mode == Mode::BULLET_42MM_AVER_K) {
        param_variant_.emplace<AverKParam>();
        auto& p = std::get<AverKParam>(param_variant_);
        p.bullet_speed = bullet_speed;
        p.k_a = 0.114514;  // 需要拟合更改！
        p.k_b = 0.1919810;
        std::cerr << "trajectory calculate: you are using a mode with uncalibrated param" << std::endl;
    } else if (mode == Mode::BULLET_17MM_SECOND_ORDER_APPROX) {
        param_variant_.emplace<SecondOrderIterationParam>();
        auto& p = std::get<SecondOrderIterationParam>(param_variant_);
        p.rongo_kuta_param.bullet_speed = bullet_speed;
        p.rongo_kuta_param.m = 0.0032;
        p.rongo_kuta_param.f_coefficient = 0.00001903;
        p.rongo_kuta_param.t_h = 0.001;
        iterator_variant_.emplace<SecondOrderApproxIterator>();
    }
}
std::optional<Trajectory::TrajectorySolution> Trajectory::GetSolution(const double x, const double y) {
    if (!has_init) {
        std::cerr << "trajectory calculate: need Init before GetSolution" << std::endl;
    }
    if (param_variant_.index() == 1) {
        return AverK(x, y);

    } else {  // param_variant_.index() == 2
        return Dichotomy(x, y);
    }
}

std::optional<Trajectory::TrajectorySolution> Trajectory::GetSolution(cv::Point3d p) {
    if (!has_init) {
        std::cerr << "trajectory calculate: need Init before GetSolution" << std::endl;
    }

    double hori = std::sqrt(p.x * p.x + p.y * p.y);
    if (param_variant_.index() == 1) {
        return AverK(hori, p.z);

    } else {  // param_variant_.index() == 2
        return Dichotomy(hori, p.z);
    }
}

std::optional<Trajectory::RongoKutaSolution> Trajectory::RongoKuta(const double theta, const double stop_iterate_x) {
    using uBullet = SecondOrderApproxIterator::RongoKutaIterator::Bullet;
    using uArray = std::array<SecondOrderApproxIterator::RongoKutaIterator::Bullet, 4>;
    auto& p = std::get<SecondOrderIterationParam>(param_variant_).rongo_kuta_param;
    auto& i = std::get<SecondOrderApproxIterator>(iterator_variant_).rongo_kuta_iterator;

    uArray k_array;

    // 赋初始�?
    i.last_bullet.theta = theta;
    i.last_bullet.x = 0;
    i.last_bullet.y = 0;
    i.last_bullet.v = p.bullet_speed;

    auto f = [&p]([[maybe_unused]] double x, [[maybe_unused]] double y, double v, double theta) -> uBullet {
        uBullet bullet;
        double D = p.f_coefficient * v * v;

        bullet.x = v * cos(theta);
        bullet.y = v * sin(theta);
        bullet.v = (-D) / p.m - p.g * sin(theta);
        bullet.theta = -(p.g * cos(theta)) / v;

        return bullet;
    };

    double t_sum = 0;
    int iterations = 0;
    while (i.last_bullet.x <= stop_iterate_x && iterations <= p.rongo_kuta_max_iterations) {
        k_array.at(0) = f(i.last_bullet.x, i.last_bullet.y, i.last_bullet.v, i.last_bullet.theta);
        k_array.at(1) =
            f(i.last_bullet.x + 0.5 * p.t_h * k_array[0].x, i.last_bullet.y + 0.5 * p.t_h * k_array[0].y,
              i.last_bullet.v + 0.5 * p.t_h * k_array[0].v, i.last_bullet.theta + 0.5 * p.t_h * k_array[0].theta);
        k_array.at(2) =
            f(i.last_bullet.x + 0.5 * p.t_h * k_array[1].x, i.last_bullet.y + 0.5 * p.t_h * k_array[1].y,
              i.last_bullet.v + 0.5 * p.t_h * k_array[1].v, i.last_bullet.theta + 0.5 * p.t_h * k_array[1].theta);
        k_array.at(3) = f(i.last_bullet.x + p.t_h * k_array[2].x, i.last_bullet.y + p.t_h * k_array[2].y,
                          i.last_bullet.v + p.t_h * k_array[2].v, i.last_bullet.theta + p.t_h * k_array[2].theta);

        i.last_bullet.x =
            i.last_bullet.x + p.t_h * (k_array[0].x + 2 * k_array[1].x + 2 * k_array[2].x + k_array[3].x) / 6;
        i.last_bullet.y =
            i.last_bullet.y + p.t_h * (k_array[0].y + 2 * k_array[1].y + 2 * k_array[2].y + k_array[3].y) / 6;
        i.last_bullet.v =
            i.last_bullet.v + p.t_h * (k_array[0].v + 2 * k_array[1].v + 2 * k_array[2].v + k_array[3].v) / 6;
        i.last_bullet.theta =
            i.last_bullet.theta +
            p.t_h * (k_array[0].theta + 2 * k_array[1].theta + 2 * k_array[2].theta + k_array[3].theta) / 6;

        t_sum += p.t_h;
        iterations++;
        //        v_5.Send((float)i.last_bullet.x, (float)i.last_bullet.y);
    }
    if (iterations <= p.rongo_kuta_max_iterations) {
        return RongoKutaSolution{i.last_bullet.y, t_sum};
    } else {
        std::cout << "trajectory: rongo kuta error" << std::endl;
        return std::nullopt;
    }
}
std::optional<Trajectory::TrajectorySolution> Trajectory::Dichotomy(const double target_x, const double target_y) {
    auto& p = std::get<SecondOrderIterationParam>(param_variant_).dichotomy_param;
    auto& i = std::get<SecondOrderApproxIterator>(iterator_variant_).dichotomy_iterator;

    int iterations = 0;
    i.last_max = p.max_theta;
    i.last_min = p.min_theta;
    while (iterations <= p.dichotomy_max_iterations) {
        i.mid_theta = (i.last_max + i.last_min) / 2;

        auto o = RongoKuta(i.mid_theta, target_x);
        if (o.has_value()) {
            i.compute_y = o.value().y;
            i.t_sum = o.value().t_sum;
        } else {
            std::cout << "trajectory: dichotomy error" << std::endl;
            return std::nullopt;
        }

        if (abs(i.compute_y - target_y) <= p.y_termination_accuracy) {
            break;
        } else {
            if (i.compute_y > target_y) {
                i.last_max = i.mid_theta;
            } else {
                i.last_min = i.mid_theta;
            }
        }
        iterations++;
    }

    if (iterations <= p.dichotomy_max_iterations) {
        //        std::cout << iterations << std::endl;
        return TrajectorySolution{i.mid_theta, i.t_sum};
    } else {
        std::cout << "trajectory: dichotomy error" << std::endl;
        return std::nullopt;
    }
}
Trajectory::TrajectorySolution Trajectory::AverK(const double x, const double y) {
    auto& p = std::get<AverKParam>(param_variant_);
    double distance = sqrt(x * x + y * y);
    double k;
    k = p.k_a * distance + p.k_b;
    double y1;
    y1 = y + (0.5 * 9.8 * distance * distance) / (k * k * p.bullet_speed * p.bullet_speed);

    return TrajectorySolution{atan2(-y1, x), distance / (k * p.bullet_speed)};
}
double Trajectory::DegreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}
double Trajectory::RadiansToDegrees(double radians) {
    return radians * 180 / M_PI;
}