//
// Created by wpie on 24-1-12.
//

#ifndef INC_1021_LINE_INTERSECT_SPHERE_SOLVER_H
#define INC_1021_LINE_INTERSECT_SPHERE_SOLVER_H

#include <cmath>
#include <vector>
#include "opencv4/opencv2/opencv.hpp"

class LineInterSectSphereSolver {
public:
    [[nodiscard]] static std::vector<cv::Point3d>
    Run(const cv::Point3d &line_o, const cv::Point3d &line_e, const cv::Point3d &sphere_centre,
        double sphere_radius);

private:
    static const double EPSILON;

// 3D vector
    struct Vector3d {
    public:
        Vector3d(double dx, double dy, double dz);

        // 矢量赋值
        void set(double dx, double dy, double dz);

        // 矢量相加
        Vector3d operator+(const Vector3d &v) const;

        // 矢量相减
        Vector3d operator-(const Vector3d &v) const;

        //矢量数乘
        [[nodiscard]] Vector3d Scalar(double c) const;

        // 矢量点积
        [[nodiscard]] double Dot(const Vector3d &v) const;

        // 矢量叉积
        [[nodiscard]] Vector3d Cross(const Vector3d &v) const;

        bool operator==(const Vector3d &v) const;

        double x, y, z;
    };

    //求解一元二次方程组ax*x + b*x + c = 0
    static void SolvingQuadratics(double a, double b, double c, std::vector<double> &t);

    static void
    LineIntersectSphere(Vector3d &O, Vector3d &E, Vector3d &Center, double R, std::vector<Vector3d> &points);

};

#endif //INC_1021_LINE_INTERSECT_SPHERE_SOLVER_H
