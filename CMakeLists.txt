cmake_minimum_required(VERSION 2.8.3)

project(food_serving_robot)

find_package(catkin REQUIRED COMPONENTS
	gazebo
	rospy
	xacro
	gazebo_ros
	robot_state_publisher
	joint_state_publisher
	gazebo_plugins
	global_planner
	teb_local_planner
	navigation
	ros_control
	ros_controllers
	map_server
)

catkin_package()

find_package(roslaunch)

install(DIRECTORY models
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})