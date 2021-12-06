cmake_minimum_required(VERSION 2.8.3)

project(food_serving_robot)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

install(DIRECTORY models
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})