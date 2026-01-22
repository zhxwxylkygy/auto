//
// Created by wpie on 24-1-12.
//

#include "line_intersect_sphere_solver.h"

const double LineInterSectSphereSolver::EPSILON = 0.0000000001;

LineInterSectSphereSolver::Vector3d::Vector3d(double dx, double dy, double dz) {
    x = dx;
    y = dy;
    z = dz;
}

void LineInterSectSphereSolver::Vector3d::set(double dx, double dy, double dz) {
    x = dx;
    y = dy;
    z = dz;
}

LineInterSectSphereSolver::Vector3d
LineInterSectSphereSolver::Vector3d::operator+(const LineInterSectSphereSolver::Vector3d &v) const {
    return {x + v.x, y + v.y, z + v.z};
}

LineInterSectSphereSolver::Vector3d
LineInterSectSphereSolver::Vector3d::operator-(const LineInterSectSphereSolver::Vector3d &v) const {
    return {x - v.x, y - v.y, z - v.z};
}

LineInterSectSphereSolver::Vector3d LineInterSectSphereSolver::Vector3d::Scalar(double c) const {
    return {c * x, c * y, c * z};
}

double LineInterSectSphereSolver::Vector3d::Dot(const LineInterSectSphereSolver::Vector3d &v) const {
    return x * v.x + y * v.y + z * v.z;
}

LineInterSectSphereSolver::Vector3d
LineInterSectSphereSolver::Vector3d::Cross(const LineInterSectSphereSolver::Vector3d &v) const {
    return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
}

bool LineInterSectSphereSolver::Vector3d::operator==(const LineInterSectSphereSolver::Vector3d &v) const {
    if (std::abs(x - v.x) < EPSILON && std::abs(y - v.y) < EPSILON && std::abs(z - v.z) < EPSILON) {
        return true;
    }
    return false;
}

void LineInterSectSphereSolver::SolvingQuadratics(double a, double b, double c, std::vector<double> &t) {
    double delta = b * b - 4 * a * c;
    if (delta < 0) {
        return;
    }

    if (std::abs(delta) < EPSILON) {
        t.push_back(-b / (2 * a));
    } else {
        t.push_back((-b + sqrt(delta)) / (2 * a));
        t.push_back((-b - sqrt(delta)) / (2 * a));
    }
}

void LineInterSectSphereSolver::LineIntersectSphere(LineInterSectSphereSolver::Vector3d &O,
                                                    LineInterSectSphereSolver::Vector3d &E,
                                                    LineInterSectSphereSolver::Vector3d &Center, double R,
                                                    std::vector<Vector3d> &points) {

    Vector3d D = E - O;            //线段方向向量

    double a = (D.x * D.x) + (D.y * D.y) + (D.z * D.z);
    double b = (2 * D.x * (O.x - Center.x) + 2 * D.y * (O.y - Center.y) + 2 * D.z * (O.z - Center.z));
    double c = ((O.x - Center.x) * (O.x - Center.x) + (O.y - Center.y) * (O.y - Center.y) +
                (O.z - Center.z) * (O.z - Center.z)) - R * R;

    std::vector<double> t;
    SolvingQuadratics(a, b, c, t);

    for (auto it: t) {
        if (it >= 0 && it <= 1) {
            points.push_back(O + D.Scalar(it));
        }
    }
}

std::vector<cv::Point3d>
LineInterSectSphereSolver::Run(const cv::Point3d &line_o, const cv::Point3d &line_e, const cv::Point3d &sphere_centre,
                               const double sphere_radius) {
    Vector3d O(line_o.x, line_o.y, line_o.z);
    Vector3d E(line_e.x, line_e.y, line_e.z);
    Vector3d Center(sphere_centre.x, sphere_centre.y, sphere_centre.z);
    double R = sphere_radius;

    std::vector<Vector3d> points;
    LineIntersectSphere(O, E, Center, R, points);

    std::vector<cv::Point3d> output;
    for (auto point: points) {
        cv::Point3d p(point.x, point.y, point.z);
        output.push_back(p);
    }
    return output;
}