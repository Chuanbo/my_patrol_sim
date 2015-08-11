# my_patrol_sim
Multi-robot patrol simulation

Required </br>
  ROS Hydro and package: [stage_ros](http://wiki.ros.org/stage_ros) <br />
  LCM(Lightweight Communications and Marshalling) [v1.0.0](https://github.com/lcm-proj/lcm) <br />
  Python 2.7 and packages: [Scanf](https://hkn.eecs.berkeley.edu/~dyoo/python/scanf/), [PIL(Python Imaging Library)](http://pythonware.com/products/pil/)

To run the simulation <br />
  1 use 'catkin_make' to build the package; <br />
  2 run 'roslaunch my_patrol_sim navigation_multi_robot.launch' in a terminal; <br />
  3 run 'roslaunch my_patrol_sim patrol_multi_robot.launch' in a new terminal; <br />
  4 start the Python script 'my_monitor.py' (in 'scripts/my_monitor' folder) to control the simulation.

To configure parameters, use the script 'initialize_sim.py'.

The results are stored in 'scripts/my_monitor/results' folder.
