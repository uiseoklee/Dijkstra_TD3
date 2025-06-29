# Robot Navigation using ROS2 and Reinforcement Learning
<img src="media/dijkstra_td3_f.gif" width="400"/>  

This project integrates **classical path planning** with **deep reinforcement learning (DRL)** to achieve intelligent robot navigation in dynamic environments. It is built on ROS2 and Gazebo, and demonstrated with a TurtleBot3 robot.

### 🧭 System Overview

1. **Global Navigation**  
   Uses **Dijkstra’s algorithm** to compute the shortest path from the robot's current location to a user-defined goal on the map.

2. **Local Navigation**  
   Employs a **deep reinforcement learning agent** (TD3) trained to avoid dynamic obstacles and follow the global path safely.

The result is a hybrid navigation system that combines the strengths of deterministic planning and learned behavior to enable autonomous, robust mobility.


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
If you have "dart" error during build, you should clone my **dart** repository.
If you have error during build, you just need to run "source opt/ros/foxy/setup.bash".
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
ros2 run turtlebot3_drl dijkstra
```
And press the **play** button in Gazebo.
A map will appear, and you can click on your desired destination using the mouse.  
The system will automatically generate a path, and the agent will perform autonomous navigation to reach the selected goal.
