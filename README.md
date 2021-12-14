# food_serving_robot
Simulation of a robot waiter in cafe environment

### [Demonstration Video](https://drive.google.com/file/d/11MnvjgTBu9isVaDKphagNQhBASHZXndW/view?usp=sharing)

## Steps to run the package

1. In a new terminal:
```bash
cd <your_ws/src>
git clone https://github.com/maarufvazifdar/food_serving_robot
cd <your_ws>
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash

roslaunch food_serving_robot gazebo.launch 
```
This will launch gazebo with cafe world, spawn robot and start joint contollers.
 
2. In a new terminal:
 ```bash
cd <your_ws>
source devel/setup.bash
roslaunch food_serving_robot navigation.launch
```
This will launch RViz, start autonomous navigation nodes.

3. In a new terminal:
 ```bash
cd <your_ws>
source devel/setup.bash
rosrun food_serving_robot move_arms.py
```
This will command the robot to pick-up the food tray.


4. After robot picks-up the food tray, in a new terminal:
 ```bash
cd <your_ws>
source devel/setup.bash
rosrun food_serving_robot nav_goals.py
```
This will send goal to the robot for delivering the food tray to the table. 
