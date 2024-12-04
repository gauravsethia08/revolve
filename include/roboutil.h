#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// Function to compute the homogeneous transformation matrix from RPY and XYZ
Matrix4d rpyxyz2H(const Vector3d &rpy, const Vector3d &xyz) {
    Matrix4d Ht = Matrix4d::Identity();
    Ht(0, 3) = xyz[0];
    Ht(1, 3) = xyz[1];
    Ht(2, 3) = xyz[2];

    Matrix4d Hx = Matrix4d::Identity();
    Hx(1, 1) = cos(rpy[0]);
    Hx(1, 2) = -sin(rpy[0]);
    Hx(2, 1) = sin(rpy[0]);
    Hx(2, 2) = cos(rpy[0]);

    Matrix4d Hy = Matrix4d::Identity();
    Hy(0, 0) = cos(rpy[1]);
    Hy(0, 2) = sin(rpy[1]);
    Hy(2, 0) = -sin(rpy[1]);
    Hy(2, 2) = cos(rpy[1]);

    Matrix4d Hz = Matrix4d::Identity();
    Hz(0, 0) = cos(rpy[2]);
    Hz(0, 1) = -sin(rpy[2]);
    Hz(1, 0) = sin(rpy[2]);
    Hz(1, 1) = cos(rpy[2]);

    return Ht * Hz * Hy * Hx;
}

// Function to compute the axis and angle from a rotation matrix
pair<Vector3d, double> R2axisang(const Matrix3d &R) {
    double ang = acos((R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2.0);
    double Z = sqrt(pow(R(2, 1) - R(1, 2), 2) +
                    pow(R(0, 2) - R(2, 0), 2) +
                    pow(R(1, 0) - R(0, 1), 2));
    if (Z == 0) {
        return {Vector3d(1, 0, 0), 0.0};
    }
    Vector3d axis;
    axis[0] = (R(2, 1) - R(1, 2)) / Z;
    axis[1] = (R(0, 2) - R(2, 0)) / Z;
    axis[2] = (R(1, 0) - R(0, 1)) / Z;
    return {axis, ang};
}

// Function to compute the skew-symmetric matrix of a vector
Matrix3d so3(const Vector3d &axis) {
    Matrix3d so3_axis;
    so3_axis << 0, -axis[2], axis[1],
                axis[2], 0, -axis[0],
                -axis[1], axis[0], 0;
    return so3_axis;
}

// Function to compute the matrix exponential of a rotation matrix
Matrix4d MatrixExp(const Vector3d &axis, double theta) {
    Matrix3d so3_axis = so3(axis);
    Matrix3d R = Matrix3d::Identity() + sin(theta) * so3_axis +
                 (1 - cos(theta)) * (so3_axis * so3_axis);
    Matrix4d H_r = Matrix4d::Identity();
    H_r.block<3, 3>(0, 0) = R;
    return H_r;
}