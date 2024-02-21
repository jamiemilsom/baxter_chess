## Baxter Chess ♟️ +  🤖

A coursework project for the Robotics Foundations course at the University of Glasgow.
This project involves using ROS, Gazebo and RViz to allow a Baxter robot to play chess.

Clone the repo, then remeber to catkin_make and source your workspace.


**Run the following in separate terminals**
-

Terminal 1:

```roslaunch baxter_gazebo baxter_world.launch```

Terminal 2:

```rosrun baxter_tools enable_robot.py -e```


```rosrun baxter_interface joint_trajectory_action_server.py```

Terminal 3:

```roslaunch baxter_moveit_config baxter_grippers.launch```

Terminal 4:

```rosrun chess_baxter spawn_chessboard.py```
