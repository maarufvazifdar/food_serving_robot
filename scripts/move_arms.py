#!/usr/bin/python2
import rospy
from std_msgs.msg import Float64
import time
import numpy as np

# right arm pose
r_intial_pos = [0, 0, 0, 0, 0, 0, 0]
r_pos_1 = [1.55, 2.6, 0.9, -2, -1.7, 1.7, -0.4]
r_pos_2 = [1.55, 2.725, 0.9, -2, -1.7, 1.7, -0.4]
r_pos_3 = [1.575, 2.725, 0.9, -2, -1.7, 1.7, 0]
r_pos_4 = [1.6, 2.725, 0.9, -2, -1.7, 1.7, 0.2]

# left arm pose
l_intial_pos = [0, 0, 0, 0, 0, 0, 0]
l_pos_1 = [-1.56, -2.6, -0.9, -1.14, 1.7, -1.7, -0.4]
l_pos_2 = [-1.56, -2.725, -0.9, -1.14, 1.7, -1.7, -0.4]
l_pos_3 = [-1.585, -2.725, -0.9, -1.14, 1.7, -1.7, 0]
l_pos_4 = [-1.6, -2.725, -0.9, -1.14, 1.7, -1.7, 0.2]


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

    # while not rospy.is_shutdown():
    time.sleep(1)
    pub_angle(r_intial_pos, r_pos_1, l_intial_pos, l_pos_1)
    time.sleep(5)
    pub_angle(r_pos_1, r_pos_2, l_pos_1, l_pos_2)
    time.sleep(1)
    pub_angle(r_pos_2, r_pos_3, l_pos_2, l_pos_3)
    time.sleep(1)
    pub_angle(r_pos_3, r_pos_4, l_pos_3, l_pos_4)
    time.sleep(1)


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
