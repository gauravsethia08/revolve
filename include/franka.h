#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "roboutil.h" // Contains rpyxyz2H, R2axisang, so3, and MatrixExp

using namespace std;
using namespace Eigen;

class FrankaArm {
public:
    vector<Matrix4d> Tlink, Tjoint, Tcurr;
    Matrix4d Tbase;
    vector<Vector3d> axis;
    vector<vector<double>> Rdesc;
    Matrix4d Tlinkzero;
    Matrix<double, 6, 7> J;
    VectorXd q;

    FrankaArm() {
        // Robot descriptor taken from URDF file
        Rdesc = {
            {0, 0, 0, 0., 0, 0.333},
            {-M_PI / 2, 0, 0, 0, 0, 0},
            {M_PI / 2, 0, 0, 0, -0.316, 0},
            {M_PI / 2, 0, 0, 0.0825, 0, 0},
            {-M_PI / 2, 0, 0, -0.0825, 0.384, 0},
            {M_PI / 2, 0, 0, 0, 0, 0},
            {M_PI / 2, 0, 0, 0.088, 0, 0},
            {0, 0, 0, 0, 0, 0.107}
        };

        axis = vector<Vector3d>(8, Vector3d(0, 0, 1));

        Tbase = Matrix4d::Identity();
        Tlink.reserve(Rdesc.size());
        Tjoint.resize(Rdesc.size(), Matrix4d::Identity());
        Tcurr.resize(Rdesc.size(), Matrix4d::Identity());

        for (const auto &desc : Rdesc) {
            Vector3d rpy(desc[0], desc[1], desc[2]);
            Vector3d xyz(desc[3], desc[4], desc[5]);
            Tlink.push_back(rpyxyz2H(rpy, xyz));
        }

        Tlinkzero = rpyxyz2H(
            Vector3d(Rdesc[0][0], Rdesc[0][1], Rdesc[0][2]),
            Vector3d(Rdesc[0][3], Rdesc[0][4], Rdesc[0][5])
        );

        Tlink[0] = Tbase * Tlink[0];
        J = Matrix<double, 6, 7>::Zero();
        q = VectorXd::Zero(7);

        ForwardKin(VectorXd::Zero(7));
    }

    std::pair<std::vector<Matrix4d>, Matrix<double, 6, 7>> ForwardKin(const VectorXd &ang) {
        // Resize q to be the same size as ang (this ensures q has the same size before assignment)
        q.resize(Rdesc.size());  
        q.head(ang.size()) = ang;

        for (size_t i = 0; i < Rdesc.size(); i++) {
            // cout<<"im in the stupid for loop"<<endl;
            Vector3d rpy = axis[i] * q[i];
            Tjoint[i] = rpyxyz2H(rpy, Vector3d(0, 0, 0));

            if (i == 0) {
                Tcurr[i] = Tlink[i] * Tjoint[i];
            } else {
                Tcurr[i] = Tcurr[i - 1] * Tlink[i] * Tjoint[i];
            }

        }

        for (size_t i = 0; i < Rdesc.size() - 1; ++i) {
            Vector3d z = Tcurr[i].block<3, 1>(0, 2);
            Vector3d p = Tcurr[i].block<3, 1>(0, 3);
            Vector3d pe = Tcurr.back().block<3, 1>(0, 3);

            Vector3d cross = z.cross(pe - p);
            J.block<3, 1>(0, i) = cross;
            J.block<3, 1>(3, i) = z;
        }

        return {Tcurr, J};
    }


    std::pair<VectorXd, VectorXd> IterInvKin(VectorXd ang, const Matrix4d &TGoal, double x_eps = 1e-3, double r_eps = 1e-3) {
        Matrix<double, 7, 7> W = Matrix<double, 7, 7>::Identity();
        Matrix<double, 6, 6> C = Matrix<double, 6, 6>::Identity();
        C(0, 0) = C(1, 1) = C(2, 2) = 1e6;
        C(3, 3) = C(4, 4) = C(5, 5) = 1e3;

        ForwardKin(ang); // Initialize Tcurr and J
        double rotate_thr = INFINITY, translation_thr = INFINITY;

        VectorXd Err(6);
        while (rotate_thr > r_eps || translation_thr > x_eps) {
            Matrix3d R_goal = TGoal.block<3, 3>(0, 0);
            Vector3d T_goal = TGoal.block<3, 1>(0, 3);

            Matrix3d R_ecur = Tcurr.back().block<3, 3>(0, 0);
            Vector3d T_ecur = Tcurr.back().block<3, 1>(0, 3);

            Matrix3d R_err_mat = R_goal * R_ecur.transpose();
            auto [axis, angle] = R2axisang(R_err_mat);
            rotate_thr = axis.norm() * angle;

            if (angle > 0.1) angle = 0.1;
            else if (angle < -0.1) angle = -0.1;

            Vector3d R_err = axis * angle;
            Vector3d T_err = T_goal - T_ecur;
            translation_thr = T_err.norm();

            if (translation_thr > 0.01) {
                T_err *= 0.01 / translation_thr;
            }

            Err << T_err, R_err;

            MatrixXd inner_term = J * W.inverse() * J.transpose() + C.inverse();
            MatrixXd jacobian_pseudoinv = W.inverse() * J.transpose() * inner_term.inverse();

            q.head(7) += jacobian_pseudoinv * Err;

            ForwardKin(q.head(7));
        }

        return {q.head(7), Err};
    }
};