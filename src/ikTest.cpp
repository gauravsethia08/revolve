#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "franka.h"   // FrankaArm class

using namespace std;
using namespace Eigen;

int main() {
    // cout<<"i entered stupid"<<endl;

    // // Initialize robot object
    FrankaArm mybot;

    // cout<<"i failed stupid"<<endl;

    // Compute forward kinematics
    double deg_to_rad = M_PI / 180.0;

    vector<VectorXd> joint_targets = {
        VectorXd::Zero(7),
        (VectorXd(7) << 0, 0, -45.0 * deg_to_rad, -15.0 * deg_to_rad,
                        20.0 * deg_to_rad, 15.0 * deg_to_rad, -75.0 * deg_to_rad).finished(),
        (VectorXd(7) << 0, 0, 30.0 * deg_to_rad, -60.0 * deg_to_rad,
                        -65.0 * deg_to_rad, 45.0 * deg_to_rad, 0.0 * deg_to_rad).finished(),
        (VectorXd(7) << 0.0, -M_PI / 4, 0.0, -M_PI / 2, 0.0, M_PI / 2, 0.0).finished()
    };

    for (const auto &joint_target : joint_targets) {
        cout << "\nJoints:\n" << joint_target.transpose() << endl;

        auto [Hcurr, J] = mybot.ForwardKin(joint_target);
        Matrix4d ee_pose = Hcurr.back();
        Matrix3d rot_ee = ee_pose.block<3, 3>(0, 0);
        Vector3d pos_ee = ee_pose.block<3, 1>(0, 3);

        cout << "Computed FK EE Position:\n" << pos_ee.transpose() << endl;
        cout << "Computed FK EE Rotation:\n" << rot_ee << endl;
    }

    // Compute inverse kinematics
    VectorXd qInit(7);
    qInit << 0, 0, 0, -2.11, 0, 3.65, -0.785;

    Matrix4d HGoal;
    HGoal << 0., 0., 1., 0.6,
             0., 1., 0., 0.,
            -1., 0., 0., 0.5,
             0., 0., 0., 1.;

    auto [q, Err] = mybot.IterInvKin(qInit, HGoal);

    cout << "\nError:\n"
         << "Translation: " << Err.head<3>().norm()
         << "\nRotation: " << Err.tail<3>().norm() << endl;
    cout << "Computed IK Angles:\n" << q.transpose() << endl;

    cout<<"\n\nVerification of IK values: "<<endl;
    
    cout << "\nqinit Joint \n" << qInit.transpose() << endl;

    auto [Hcurr, J] = mybot.ForwardKin(qInit);
    Matrix4d ee_pose = Hcurr.back();
    Matrix3d rot_ee = ee_pose.block<3, 3>(0, 0);
    Vector3d pos_ee = ee_pose.block<3, 1>(0, 3);

    cout << "Computed FK EE Position:\n" << pos_ee.transpose() << endl;
    cout << "Computed FK EE Rotation:\n" << rot_ee << endl;

    Matrix4d CheckHGoal;

    CheckHGoal << 0.0217815, 0.0217642,  0.999526, 0.564036,
                  0.706825,  -0.707388,  -6.31178e-17, -2.84551e-17,
                  0.707053,    0.70649,  -0.0307915, 0.607292,
                  0., 0., 0., 1.;

    auto [q1, Err1] = mybot.IterInvKin(qInit, CheckHGoal);

    cout << "\nError:\n"
         << "Translation: " << Err1.head<3>().norm()
         << "\nRotation: " << Err1.tail<3>().norm() << endl;
    cout << "Computed IK Angles:\n" << q1.transpose() << endl;




    // VectorXd verify_targets = (VectorXd(7) << 0.298799, 0.266241, -0.347551, -2.08222, -0.0537502, 3.90809, -3.01306).finished();

    // cout << "\nVerify Joints:\n" << verify_targets.transpose() << endl;

    // auto [Hcurr, J] = mybot.ForwardKin(verify_targets);
    // Matrix4d ee_pose = Hcurr.back();
    // Matrix3d rot_ee = ee_pose.block<3, 3>(0, 0);
    // Vector3d pos_ee = ee_pose.block<3, 1>(0, 3);

    // cout << "Computed FK EE Position:\n" << pos_ee.transpose() << endl;
    // cout << "Computed FK EE Rotation:\n" << rot_ee << endl;

    return 0;
}
