# Drake

Model-Based Design and Verification for Robotics.

Please see the [Drake Documentation](https://drake.mit.edu) for more
information.

In order to run the experiments associated with "Real-Time Safety Compliant Control Framework with Operational and Joint Space Constraints for Robotic Manipulators",
Install the following:

1) QPMAD 
2) ROS (The code is not written for ROS 2).

QPMAD is available at https://github.com/asherikov/qpmad . The instruction to install them in default setting are as follow
a) Clone https://github.com/asherikov/qpmad
b) cd qmpad
c) mkdir build && cd build
d) cmake .. && make -j
e) sudo make install

For ROS 1 installation, follow the instruction on https://www.ros.org/

The experiment does have an absolute path where we store variable. You may want to change the absolute path at line 694 to your appropriate path

In order to run experiments, do the following:

1) Open terminal I, and type the following
bazel run //tools:drake_visualizer

2) Open terminal II, and the type the following
bazel run //examples/kuka_iiwa_arm:kuka_simulation

3) Open terminal III, and type the following
bazel run //examples/kuka_iiwa_arm:kuka_task_control_final

This will start the application but it will stilll not run. This is because the program is expecting the ros command(which were used to perform experiments with vision).
Do the following

4) Open Terminal IV, run roscore by the typing the following command
roscore

5) Open Terminal V, and do the following:
cd /path-to-drake/catkin_ws/
source devel/setup.bash



