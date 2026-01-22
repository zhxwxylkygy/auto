//
// Created by wpie on 23-10-25.
//

#ifndef INC_0601_TRAJECTORY_CALCULATE_H
#define INC_0601_TRAJECTORY_CALCULATE_H
#include <opencv2/core/types.hpp>
#include <variant>
#include <optional>
#include <array>

//单位 m s kg 弧度

class Trajectory{
public:
    struct TrajectorySolution{
        double angle;
        double fly_time;
    };
public:

    enum class Mode{
        BULLET_17MM_AVER_K,
        BULLET_17MM_SECOND_ORDER_APPROX,
        BULLET_42MM_AVER_K,
        BULLET_42MM_SECOND_ORDER_APPROX
    };

    void Init(Mode mode,double bullet_speed);

    [[nodiscard]] std::optional<TrajectorySolution> GetSolution(double x, double y);

    std::optional<TrajectorySolution> GetSolution(cv::Point3d p);
private:
    struct CommonParam{
        double bullet_speed;
        double g = 9.8;

    };
    struct AverKParam : CommonParam{
        double k_a = 0.000218834080717; //线性速度系数k的常数a
        double k_b = 0.662825112107624; //线性速度系数k的常数b
    };
    struct SecondOrderIterationParam {

        struct RongoKutaParam : CommonParam{
            double t_h; //Rongo Kuta法的�?变量单位步长 这里为时间t
            // 停�?�迭代条件为 超过�?标的x坐标
            int rongo_kuta_max_iterations = 800; //保护作用 防�?�程序在特殊数值下跑�??
            double f_coefficient; //空气阻力系数
            double m; //弹丸质量


        }rongo_kuta_param;

        struct DichotomyParam{
            double max_theta = DegreesToRadians(45); //�?代的最大�?�度
            double min_theta = DegreesToRadians(-30); //�?代的最小�?�度
            double y_termination_accuracy = 0.007; //�?标y坐标的迭代终止精�?
            int dichotomy_max_iterations = 150;//保护作用 防�?�程序在特殊数值下跑�??
        }dichotomy_param;

    };
    struct SecondOrderApproxIterator{
        struct RongoKutaIterator{
            struct Bullet{
                double x; // x 坐标
                double y; // y 坐标
                double v; // v_3 飞�?�速度绝�?��?
                double theta; // theta 飞�?��?�度 水平�?0�? 垂直�?90�?
            }last_bullet;
            std::array<Bullet, 4> kn;// rongo kuta �? k1 k2 k3 k4
        }rongo_kuta_iterator;

        struct DichotomyIterator{
            double last_max;
            double last_min;

            double compute_y;
            double t_sum;
            double mid_theta;
        }dichotomy_iterator;
    };

    struct RongoKutaSolution{
        double y;
        double t_sum;
    };

    std::variant<std::monostate, AverKParam, SecondOrderIterationParam> param_variant_; // 计算参数
    std::variant<std::monostate, SecondOrderApproxIterator> iterator_variant_; // �?代算�?

    bool has_init = false;

    [[nodiscard]] std::optional<RongoKutaSolution> RongoKuta(double theta, double stop_iterate_x);

    [[nodiscard]] std::optional<TrajectorySolution> Dichotomy(double target_x, double target_y);

    [[nodiscard]] Trajectory::TrajectorySolution AverK(double x, double y);

    [[nodiscard]] static double DegreesToRadians(double degrees);

    [[nodiscard]] static double RadiansToDegrees(double radians);
};
#endif //INC_0601_TRAJECTORY_CALCULATE_H
