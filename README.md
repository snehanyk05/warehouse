# warehouse
Warehouse Planning

#1 
On terminal 1 - roscore

#2
On terminal 2 - rosrun warehouse_manager warehouse_master

#3
On terminal 3 - roslaunch warehouse_orca turtlebot_stage.launch

#4
On terminal 4 - cd into warehouse_orca/src and run
python multi_process_agents.py

There are 4 files in the src folder other than multi_process_agents.py
1. turtle_instance.py (without laser)
2. turtle_instance_laser.py 
3. turtle_instance_astar.py (global planning)
4. turtle_instance_astar_laser.py( with astar and lidar)

You can switch these file in multi_process_agents.py, by default it is turtle_instance_laser.py 
