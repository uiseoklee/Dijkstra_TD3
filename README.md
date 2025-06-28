# Robot-Navigation-using-ROS2-ReinforcementL-Algorithm
![Demo](media/dijkstra_td3.gif)

# Installation
## Prerequisite
- Ubuntu 20.04
- ROS2 foxy
- Turtlebot3 packages (refer to turtlebot3 emanual)

### Terminal 1
```
cd
git clone https://github.com/uiseoklee/Dijkstra_TD3.git
cd ~/Dijkstra_TD3
colcon build --symlink-install
```
<If you have "dart" error during build, you should clone NAVIGATION/dart>
<If you have error during build, you just need to run "source opt/ros/foxy/setup.bash">
```
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Terminal 2
```
ros2 run turtlebot3_drl environment
```

### Terminal 3
```
ros2 run turtlebot3_drl test_agent td3 'examples/td3_0_stage9' 7400
```

### Terminal 4
```
ros2 run turtlebot3_drl gazebo_goals
```
<If you have error in Terminal 3, you just need to run that command again>


Robot Navigation was implemented as follows. 
1. Global Navigation Find the shortest path using Dijkstra's algorithm.
2. Local Navigation Avoid obstacles using Deep Reinforcement Learning.
3. All development environments are ROS2 based.
