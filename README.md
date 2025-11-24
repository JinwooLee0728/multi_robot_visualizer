# multi_robot_visualizer
Repository for real-time ground robots control and state visualizer.

A. How to use the real-time visualizer:
1. Open "visualizer_script_v2.m" in MATLAB (Make sure DataLogger.m is in the same folder)
2. Run script code
3. Run external nodes that publish commands and odometry

B. How to use the planner node (for testing)
1. Open "traj_gen_v4.m" in MATLAB
2. Run script code
3. Select the type of trajectory you want to publish (Input numbers 1-7)
4. If you are ready to publish, input 'y'.

Published topics
- /robot_1/planned_path, /robot_2/planned_path, etc. ... (Mimics PLANNER)
- /cmd_vel_list ... (Mimics PLANNER)
- /robot1/cmd_vel, /robot2/cmd_vel, etc. ... (Mimics GROUND STATION) 
