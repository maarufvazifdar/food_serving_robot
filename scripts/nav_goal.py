#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import time


def nav_goal_py(x, y, yaw):
    # Create an action client called "move_base" with action definition file
    # "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Waits until the action server has started up and started listening for
    # goals.
    client.wait_for_server()
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    # add goal pose in terms of x,y (in metrers) and yaw (in raidans)
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # to send orientation with a yaw we need quaternion transform
    X, Y, Z, W = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = X
    goal.target_pose.pose.orientation.y = Y
    goal.target_pose.pose.orientation.z = Z
    goal.target_pose.pose.orientation.w = W

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    # wait = client.wait_for_result()
    result = client.get_result
    if result:
        print("Goal Reached")
    # If the result doesn't arrive, assume the Server is not available
    # if not wait:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:
    # # Result of executing the action


if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and
        # subscribe
        rospy.init_node('nav_goal_py')
        nav_goal_py(3, 4.25, 3.14)
        time.sleep(5)
        nav_goal_py(2.5, -4.5, -1.57)

        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
