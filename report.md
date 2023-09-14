# How to compile
To compile the node, first create a workspace folder:
```sh
cd ~
mkdir catkin_ws
cd catkin_ws
mkdir project
cd project
mkdir src
cd src
catkin_init_workspace
cd ..
```

Now create a softlink from the node folder to the `catkin_ws/project/src` folder:
```sh
ln -s /home/$USER/<path to node folder> /home/$USER/catkin_ws/project/src/icp_localization
```

Finally you can compile the package using _catkin_:
```sh
catkin build
```

Then you can source the workspace:
```sh
source devel/setup.bash
```

# How to run
To run the code, you're going to need five terminals. In each one the first thing to do is source ROS:
```sh
source /opt/ros/noetic/setup.bash
```

In the first terminal, startup the roscore:
```sh
roscore
```

In the second terminal, go to the node folder and launch the `map_server` node:
```sh
rosrun map_server map_server test_data/cappero_map.yaml    
```

In the third terminal, go to the node folder and launch the `stage_ros` node:
```sh
rosrun stage_ros stageros test_data/cappero.world
```

In the fourth terminal, go to the `catkin_ws/project` folder and launch the node:
```sh
rosrun icp_localization localizer_node ~scan:=/robot_0/base_laser_link
```

In the fifth terminal, go to the node folder and launch RViz:
```sh
rviz -d test_data/rviz.rviz
```

In RViz you will be able set the _initialpose_ of the robot with the corresponding button, and you will see the localizer trying to match the scan with the map.

# How to test
To test the code you will need another terminal and the `teleop_twist_keyboard` package.
Run the command
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

And you will be able to change position and orientation of the robot in the simulation.
By doing that, the scan will change, so the localizer will change its estimate of the position, which can be seen in RViz.

