//----------------------------------------------------------
// Function of computation of jacobian matrix
//----------------------------------------------------------
// Description: computation of jacobian matrix
// Copyright: Yihan Liu 2024
//----------------------------------------------------------

#include "Jacobian.h"
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


//----------------------------------------------------------
// Computation of jacobian matrix
//----------------------------------------------------------
// 1 input:
// q: joint angular positions
//
// 1 output:
// jaco: jacobian matrix
//----------------------------------------------------------
MatrixXd Jacobian::jaco_m(const VectorXd& q) {
    Matrix4d T_B1, T_12, T_23, T_34, T_45, T_56, T_67, T_7Tool;

    Eigen::Vector3d z0, z1, z2, z3, z4, z5, z6, z7;
    Eigen::Vector4d p0, p1, p2, p3, p4, p5, p6, p7, pE;
    Eigen::Vector3d z1_p1, z2_p2, z3_p3, z4_p4, z5_p5, z6_p6, z7_p7;

    Eigen::MatrixXd jaco(6,7);

    // DH parameters from YAML file
    // All a = 0, most alpha = pi/2, last alpha = pi
    // theta_offset applied to theta values

    // Joint 1: alpha=pi/2, d=-0.2848, theta_offset=0
    double theta1 = q(0) + 0.0;
    T_B1 << cos(theta1), 0, sin(theta1), 0,
            sin(theta1), 0, -cos(theta1), 0,
            0, 1, 0, -0.2848,
            0, 0, 0, 1;

    // Joint 2: alpha=pi/2, d=-0.0118, theta_offset=pi
    double theta2 = q(1) + M_PI;
    T_12 << cos(theta2), 0, sin(theta2), 0,
            sin(theta2), 0, -cos(theta2), 0,
            0, 1, 0, -0.0118,
            0, 0, 0, 1;

    // Joint 3: alpha=pi/2, d=-0.4208, theta_offset=pi
    double theta3 = q(2) + M_PI;
    T_23 << cos(theta3), 0, sin(theta3), 0,
            sin(theta3), 0, -cos(theta3), 0,
            0, 1, 0, -0.4208,
            0, 0, 0, 1;

    // Joint 4: alpha=pi/2, d=-0.0128, theta_offset=pi
    double theta4 = q(3) + M_PI;
    T_34 << cos(theta4), 0, sin(theta4), 0,
            sin(theta4), 0, -cos(theta4), 0,
            0, 1, 0, -0.0128,
            0, 0, 0, 1;

    // Joint 5: alpha=pi/2, d=-0.3143, theta_offset=pi
    double theta5 = q(4) + M_PI;
    T_45 << cos(theta5), 0, sin(theta5), 0,
            sin(theta5), 0, -cos(theta5), 0,
            0, 1, 0, -0.3143,
            0, 0, 0, 1;

    // Joint 6: alpha=pi/2, d=0, theta_offset=pi
    double theta6 = q(5) + M_PI;
    T_56 << cos(theta6), 0, sin(theta6), 0,
            sin(theta6), 0, -cos(theta6), 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

    // Joint 7: alpha=pi, d=-0.2474, theta_offset=pi
    double theta7 = q(6) + M_PI;
    T_67 << cos(theta7), -sin(theta7), 0, 0,
            sin(theta7), cos(theta7), 0, 0,
            0, 0, 1, -0.1674,
            0, 0, 0, 1;

    // Tool frame (if needed, otherwise identity)
    T_7Tool << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.08,
            0, 0, 0, 1;

    z0 << 0, 0, 1;
    z1 = T_B1.block<3,3>(0,0)*z0;
    z2 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*z0;
    z3 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*z0;
    z4 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*z0;
    z5 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*z0;
    z6 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*T_56.block<3,3>(0,0)*z0;
    z7 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*T_56.block<3,3>(0,0)*T_67.block<3,3>(0,0)*z0;

    p0 << 0, 0, 0, 1;
    p1 = T_B1*p0;
    p2 = T_B1*T_12*p0;
    p3 = T_B1*T_12*T_23*p0;
    p4 = T_B1*T_12*T_23*T_34*p0;
    p5 = T_B1*T_12*T_23*T_34*T_45*p0;
    p6 = T_B1*T_12*T_23*T_34*T_45*T_56*p0;
    p7 = T_B1*T_12*T_23*T_34*T_45*T_56*T_67*p0;
    pE = T_B1*T_12*T_23*T_34*T_45*T_56*T_67*T_7Tool*p0;

    z1_p1 = z1.cross(pE.head<3>()-p1.head<3>());
    z2_p2 = z2.cross(pE.head<3>()-p2.head<3>());
    z3_p3 = z3.cross(pE.head<3>()-p3.head<3>());
    z4_p4 = z4.cross(pE.head<3>()-p4.head<3>());
    z5_p5 = z5.cross(pE.head<3>()-p5.head<3>());
    z6_p6 = z6.cross(pE.head<3>()-p6.head<3>());
    z7_p7 = z7.cross(pE.head<3>()-p7.head<3>());

    // Correct Jacobian assembly:
    // Top 3 rows are linear velocity components
    // Bottom 3 rows are angular velocity components
    jaco.topRows(3) << z1_p1, z2_p2, z3_p3, z4_p4, z5_p5, z6_p6, z7_p7;
    jaco.bottomRows(3) << z1, z2, z3, z4, z5, z6, z7;

    return jaco;
}