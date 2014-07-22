hrp2_moveit
===========

Workspace for MoveIt! manipulation integration with HRP2NT. Note that the HRP2 model (URDF) is private, so this repo is not useful unless you have access to hrp2jsknt_description.

## Install

To use this repo:

* Install wstool package if you have not already:
    ```
    sudo apt-get install python-wstool
    ```

* Create a catkin workspace

    ```
    mkdir -p ~/ros/ws_hrp2/src
    cd ~/ros/ws_hrp2/src/
    catkin_init_workspace
    ```

* Use the rosinstall file to pull in all necessary repositories.
  This includes various repos as needed to build at the time of this writing (based on what is available as debians)

    ```
    wstool init .
    wstool merge https://raw.githubusercontent.com/davetcoleman/moveit_hrp2/master/moveit_hrp2.rosinstall
    wstool update
    ```

* Add the private package hrp2jstnt_description if you have access (sorry).

* Install dependencies
    ```
    rosdep install --from-paths src --ignore-src --rosdistro hydro -y -r
    ```
    *This may not work well if you have rosbuild/pre-Hydro packages in your repository

* Build the workspace
  
    ```
    cd ~/ros/ws_hrp2/
    catkin_make
    ```

* Build necessary ROS messages into LISP version
    ```
    rosrun roseus generate-all-msg-srv.sh moveit_msgs sensor_msgs geometry_msgs trajectory_msgs std_msgs actionlib_msgs visual_msgs std_srvs hrp2_moveit_msgs jsk_ik_server cmake_modules
    ```

* Only if you have issues - try installing these other Ubuntu dependencies (required by jsk-ros-pkg, not this repo)
    ```	  
    sudo apt-get install -y libomniorb4-dev omniidl libomniorb4* omni* libblas* liblapack* f2c omniidl-python omniorb-nameserver libomniorb4-1-dbg libomniorb4-dev omniorb omniorb-idl python-omniorb-dbg python-omniorb-doc
    sudo apt-get install -y gsfonts-x11 texlive-fonts-extra xfonts-100dpi xfonts-75dpi xfonts-100dpi-transcoded xfonts-75dpi-transcoded
    sudo apt-get install -y openjdk-7-jre openjdk-7-jdk 
    sudo apt-get install -y ros-hydro-ps3joy ros-hydro-rosjava-build-tools drcsim-hydro ros-hydro-moveit-full ros-hydro-actionlib-tutorials    
    ```
  
## Instructions

There are two ways to use the HRP2 features:

### Rviz Motion Planning Plugin Interation

Start the ik server:
```
roslaunch jsk_ik_server hrp2jsknt-ik-server.launch 
```

Start the walking server:
```
roslaunch hrp2jsknt_moveit_demos calc_walking_pattern_server.launch 
```

Launch move_group separate:
```
roslaunch hrp2jsknt_moveit_config move_group_context.launch 
```

Launch Rviz w/Motion Planning Plugin:
```
roslaunch hrp2jsknt_moveit_config moveit_rviz.launch 
```

#### Modification

To change how many end effectors are loaded, edit the file
```
hrp2jsknt_moveit_config/config/kinematics.yaml
```
And [un]comment out the links you want the kinematics solver to use.

To change if MoveIt uses a walking motion generator, edit the file 
```
hrp2jsknt_moveit_config/launch/ompl_planning_pipeline.launch.xml
```
And add/remove the line
```
default_planner_request_adapters/AddWalkingSteps
```
from the ``planning_adapters`` arg.

### HRP2 MoveIt Demos

A script has been created for testing various functionalities

Launch Rviz first:
```
roslaunch hrp2jsknt_moveit_demos moveit_rviz.launch
```

You need the walking server running for mode #2
```
roslaunch hrp2jsknt_moveit_demos calc_walking_pattern_server.launch 
```

Now run demo in different modes (change the number at the end):
```
roslaunch hrp2jsknt_moveit_demos hrp2_demos.launch mode:=1
```

Change this mode to different numbers:

* 0 - Loop through all these modes continously
* 1 - Plan to a pre-defined crouching position, fixed feet
* 2 - Generate random walking positions and generate footsteps using ROS Service call to eulisp
* 3 - Plan with MoveIt + Lightning for different arm positions
* 4 - Show the experience database visually in Rviz
* 5 - Solve for different fixed leg positions using KDL IK (proof of concept for sampler)
* 6 - Generate random walking positions and plan to them with MoveIt (no walking)
* 7 - Generate completely random poses of robot, then transform robot to foot on ground
* 8 - Test single arm planning on HRP2 using MoveIt Whole Body IK solver
* 9 - Exit
