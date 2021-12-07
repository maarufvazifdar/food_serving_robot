#!/usr/bin/python2
import rospy
from std_msgs.msg import Float64
import time
import numpy as np
import sympy as sp
from math import cos, sin, pi


J = sp.Matrix([])
J_inv = sp.Matrix([])
O7 = sp.Matrix([])
th1 = th2 = th3 = th4 = th5 = th6 = th7 = 0

dt = 0.1  # sampling time
x_vel = sp.Matrix([])
y_vel = sp.Matrix([])
z_vel = sp.Matrix([])
roll_vel = sp.Matrix([])
pitch_vel = sp.Matrix([])
yaw_vel = sp.Matrix([])
th_vel = sp.Matrix([[]])
cartesian_vels = sp.Matrix([])

r_intial_pos = [0, 0, 0, 0, 0, 0, 0]
r_pos_1 = [1.55, 2.6, 0.9, -2, -1.7, 1.7, -0.4]
r_pos_2 = [1.55, 2.725, 0.9, -2, -1.7, 1.7, -0.4]
r_pos_3 = [1.575, 2.725, 0.9, -2, -1.7, 1.7, 0]
r_pos_4 = [1.6, 2.725, 0.9, -2, -1.7, 1.7, 0.2]

l_intial_pos = [0, 0, 0, 0, 0, 0, 0]
l_pos_1 = [-1.56, -2.6, -0.9, -1.14, 1.7, -1.7, -0.4]
l_pos_2 = [-1.56, -2.725, -0.9, -1.14, 1.7, -1.7, -0.4]
l_pos_3 = [-1.585, -2.725, -0.9, -1.14, 1.7, -1.7, 0]
l_pos_4 = [-1.6, -2.725, -0.9, -1.14, 1.7, -1.7, 0.2]


def calculateJ_J_inv(th1, th2, th4, th5, th6, th7):
    global J, J_inv, O7

    # Manipulator's constant parameters.
    d1 = 0.1273
    d2 = 0
    d3 = 0
    d4 = 0.163941
    d5 = 0.1157
    d6 = 0.0922
    d7 = 0.12

    # Transformation Matrix for frame{0}
    _0T0 = sp.Matrix([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    # Transformation Matrix for frame{1} w.r.t frame{0}.
    _0T1 = sp.Matrix([[cos(th1), 0, -sin(th1), 0],
                      [sin(th1), 0, cos(th1), 0],
                      [0, -1, 0, d1],
                      [0, 0, 0, 1]])

    # Transformation Matrix for frame{2} w.r.t frame{1}.
    _1T2 = sp.Matrix([[cos(th2), 0, sin(th2), 0],
                      [sin(th2), 0, -cos(th2), 0],
                      [0, 1, 0, d2],
                      [0, 0, 0, 1]])

    # Transformation Matrix for frame{3} w.r.t frame{2}.
    _2T3 = sp.Matrix([[cos(th3), 0, sin(th3), 0],
                      [sin(th3), 0, -cos(th3), 0],
                      [0, 1, 0, d3],
                      [0, 0, 0, 1]])

    # Transformation Matrix for frame{4} w.r.t frame{3}.
    _3T4 = sp.Matrix([[cos(th4), 0, -sin(th4), 0],
                      [sin(th4), 0, cos(th4), 0],
                      [0, -1, 0, d4],
                      [0, 0, 0, 1]])

    # Transformation Matrix for frame{5} w.r.t frame{4}.
    _4T5 = sp.Matrix([[cos(th5), 0, -sin(th5), 0],
                      [sin(th5), 0, cos(th5), 0],
                      [0, -1, 0, d5],
                      [0, 0, 0, 1]])

    # Transformation Matrix for frame{6} w.r.t frame{5}.
    _5T6 = sp.Matrix([[cos(th6), 0, sin(th6), 0],
                      [sin(th6), 0, -cos(th6), 0],
                      [0, 1, 0, d6],
                      [0, 0, 0, 1]])

    # Transformation Matrix for frame{7} w.r.t frame{6}.
    _6T7 = sp.Matrix([[cos(th7), -sin(th7), 0, 0],
                      [sin(th7), cos(th7), 0, 0],
                      [0, 0, 1, d7],
                      [0, 0, 0, 1]])

    # Intermediate transformation matrices for frame{i} w.r.t. frame{0}
    _0T2 = _0T1 * _1T2
    _0T3 = _0T2 * _2T3
    _0T4 = _0T3 * _3T4
    _0T5 = _0T4 * _4T5
    _0T6 = _0T5 * _5T6
    # Final Transformation Matrix for frame{7} w.r.t frame{0}.
    _0T7 = _0T6 * _6T7

    # Z matrices (3x1)
    Z0 = _0T0[0:3, 2]
    Z1 = _0T1[0:3, 2]
    Z2 = _0T2[0:3, 2]
    Z3 = _0T3[0:3, 2]
    Z4 = _0T4[0:3, 2]
    Z5 = _0T5[0:3, 2]
    Z6 = _0T6[0:3, 2]

    # O matrices (3x1)
    O0 = _0T0[0:3, 3]
    O1 = _0T1[0:3, 3]
    O2 = _0T2[0:3, 3]
    O3 = _0T3[0:3, 3]
    O4 = _0T4[0:3, 3]
    O5 = _0T5[0:3, 3]
    O6 = _0T6[0:3, 3]
    O7 = _0T7[0:3, 3]

    # Calculating individual columns (6x1) of Jacobian  matrix.
    J1 = Z0.cross(O7 - O0)
    J1 = J1.row_insert(3, Z0)

    J2 = Z1.cross(O7 - O1)
    J2 = J2.row_insert(3, Z1)

    J3 = Z2.cross(O7 - O2)
    J3 = J3.row_insert(3, Z2)

    J4 = Z3.cross(O7 - O3)
    J4 = J4.row_insert(3, Z3)

    J5 = Z4.cross(O7 - O4)
    J5 = J5.row_insert(3, Z4)

    J6 = Z5.cross(O7 - O5)
    J6 = J6.row_insert(3, Z5)

    J7 = Z6.cross(O7 - O6)
    J7 = J7.row_insert(3, Z6)

    # 6x6 Jacobian Matrix
    J = J1
    J = J.col_insert(1, J2)
    J = J.col_insert(2, J4)
    J = J.col_insert(3, J5)
    J = J.col_insert(4, J6)
    J = J.col_insert(5, J7)

    # Calculating Inverse Jacobian Matrix
    J_inv = J.inv(method="LU")


def pub_angle(pose0, pose1, pose2, pose3):
    t1 = np.linspace(pose0[0], pose1[0], 10)
    t2 = np.linspace(pose0[1], pose1[1], 10)
    t3 = np.linspace(pose0[2], pose1[2], 10)
    t4 = np.linspace(pose0[3], pose1[3], 10)
    t5 = np.linspace(pose0[4], pose1[4], 10)
    t6 = np.linspace(pose0[5], pose1[5], 10)
    t7 = np.linspace(pose0[6], pose1[6], 10)

    l1 = np.linspace(pose2[0], pose3[0], 10)
    l2 = np.linspace(pose2[1], pose3[1], 10)
    l3 = np.linspace(pose2[2], pose3[2], 10)
    l4 = np.linspace(pose2[3], pose3[3], 10)
    l5 = np.linspace(pose2[4], pose3[4], 10)
    l6 = np.linspace(pose2[5], pose3[5], 10)
    l7 = np.linspace(pose2[6], pose3[6], 10)

    for i in range(10):
        pub_r_shoulder_pan_joint_controller.publish(t1[i])
        pub_r_shoulder_lift_joint_controller.publish(t2[i])
        pub_r_elbow_joint_controller.publish(t3[i])
        pub_r_wrist_1_joint_controller.publish(t4[i])
        pub_r_wrist_2_joint_controller.publish(t5[i])
        pub_r_wrist_3_joint_controller.publish(t6[i])
        pub_r_finger_joint_controller.publish(t7[i])

        pub_l_shoulder_pan_joint_controller.publish(l1[i])
        pub_l_shoulder_lift_joint_controller.publish(l2[i])
        pub_l_elbow_joint_controller.publish(l3[i])
        pub_l_wrist_1_joint_controller.publish(l4[i])
        pub_l_wrist_2_joint_controller.publish(l5[i])
        pub_l_wrist_3_joint_controller.publish(l6[i])
        pub_l_finger_joint_controller.publish(l7[i])
        time.sleep(1)


def move():
    global pub_r_shoulder_pan_joint_controller, pub_r_shoulder_lift_joint_controller, pub_r_elbow_joint_controller, pub_r_wrist_1_joint_controller, pub_r_wrist_2_joint_controller, pub_r_wrist_3_joint_controller, pub_r_finger_joint_controller

    global pub_l_shoulder_pan_joint_controller, pub_l_shoulder_lift_joint_controller, pub_l_elbow_joint_controller, pub_l_wrist_1_joint_controller, pub_l_wrist_2_joint_controller, pub_l_wrist_3_joint_controller, pub_l_finger_joint_controller

    rospy.init_node('move', anonymous=True)

    pub_r_shoulder_pan_joint_controller = rospy.Publisher(
        '/r_shoulder_pan_joint_controller/command', Float64, queue_size=10)
    pub_r_shoulder_lift_joint_controller = rospy.Publisher(
        '/r_shoulder_lift_joint_controller/command', Float64, queue_size=10)
    pub_r_elbow_joint_controller = rospy.Publisher(
        '/r_elbow_joint_controller/command', Float64, queue_size=10)
    pub_r_wrist_1_joint_controller = rospy.Publisher(
        '/r_wrist_1_joint_controller/command', Float64, queue_size=10)
    pub_r_wrist_2_joint_controller = rospy.Publisher(
        '/r_wrist_2_joint_controller/command', Float64, queue_size=10)
    pub_r_wrist_3_joint_controller = rospy.Publisher(
        '/r_wrist_3_joint_controller/command', Float64, queue_size=10)
    pub_r_finger_joint_controller = rospy.Publisher(
        '/r_finger_joint_controller/command', Float64, queue_size=10)

    pub_l_shoulder_pan_joint_controller = rospy.Publisher(
        '/l_shoulder_pan_joint_controller/command', Float64, queue_size=10)
    pub_l_shoulder_lift_joint_controller = rospy.Publisher(
        '/l_shoulder_lift_joint_controller/command', Float64, queue_size=10)
    pub_l_elbow_joint_controller = rospy.Publisher(
        '/l_elbow_joint_controller/command', Float64, queue_size=10)
    pub_l_wrist_1_joint_controller = rospy.Publisher(
        '/l_wrist_1_joint_controller/command', Float64, queue_size=10)
    pub_l_wrist_2_joint_controller = rospy.Publisher(
        '/l_wrist_2_joint_controller/command', Float64, queue_size=10)
    pub_l_wrist_3_joint_controller = rospy.Publisher(
        '/l_wrist_3_joint_controller/command', Float64, queue_size=10)
    pub_l_finger_joint_controller = rospy.Publisher(
        '/l_finger_joint_controller/command', Float64, queue_size=10)

    time.sleep(1)
    pub_angle(r_intial_pos, r_pos_1, l_intial_pos, l_pos_1)
    time.sleep(5)
    pub_angle(r_pos_1, r_pos_2, l_pos_1, l_pos_2)
    time.sleep(1)
    pub_angle(r_pos_2, r_pos_3, l_pos_2, l_pos_3)
    time.sleep(1)
    pub_angle(r_pos_3, r_pos_4, l_pos_3, l_pos_4)
    time.sleep(1)


def inverse():
    calculateJ_J_inv(th1, th2, th4, th5, th6, th7)
    Y = O7[1]
    Z = O7[2]

    # Joint velocity martix (6X1)
    th_vel = J_inv * sp.Matrix([[x_vel],
                                [y_vel],
                                [z_vel],
                                [roll_vel],
                                [pitch_vel],
                                [yaw_vel]])

    # Updating joint angles
    th1 += th1 * (th_vel[0] * dt)
    th2 += th2 * (th_vel[1] * dt)
    th4 += th4 * (th_vel[2] * dt)
    th5 += th5 * (th_vel[3] * dt)
    th6 += th6 * (th_vel[4] * dt)
    th7 += th7 * (th_vel[5] * dt)

    # Cartesian velocity martix (6X1)
    cartesian_vels = J * th_vel

    # Integrating Cartesian velocities for plotting on graph
    Y = Y + (cartesian_vels[1] * dt)
    Z = Z + (cartesian_vels[2] * dt)


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
