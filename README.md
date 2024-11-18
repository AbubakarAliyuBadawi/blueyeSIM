# Mundus Mir Gazebo Workspace

This workspace should contain all the necessary packages to run the Mundus MIR Simulator. 

## Modules
**mundus_mir_simulator:** Module containing all the packages directly related to the simulator. This includes the gazebo enviroment, gazebo models, gazebo-ros bridge and gazebo sensors.

**mundus_mir_simulator_launch:** Module containing all the launch files for this workspace. The idea behind this module is to have a single place to set configuration parameters and launch the simulator.

**mundus_mir_navigation:** Module containing all the packages related to navigation. This includes the navigation stack, localization, mapping and path planning.

**mundus_mir_controllers:** Module containing all the packages related to control. This includes the control stack, motion planning and trajectory generation.

**mundus_mir_perception:** Module containing all the packages related to perception. This includes the perception stack, object detection, object tracking and object recognition.

**mundus_mir_mission_planner:** Module containing all the packages related to mission planning. This includes the mission planner, task allocation and task execution.

**utility_toolbox:** Package containing some utilities such as UDP server and noise generation. 


## Prerequisite

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Garden](https://gazebosim.org/docs/garden/getstarted/)
- [Eigen3](https://eigen.tuxfamily.org/dox/GettingStarted.html)
- [GIT LFS](https://git-lfs.github.com/)


The ROS2 brigde appears to have added some new dependencies. By running the script in the root of this project your system will install them.

```
./dep_install.sh
```

If you are still encuntering issues building the workspace complaining about missing packages the issue will often be that your ROS2 instalation did not come with the necessary packages. You can check if this is the case and install them manually be running `sudo apt install ros-humble-<start of package name> TAB TAB` in your terminal to see if the package exist. If if does simply install it using apt and attempt to rebuild your workspace.

```
sudo apt install ros-humble-<package name>
```

## Cloning the workspace and updating submodules

This repository is a collection of git submodules. To clone the repository and all its submodules. This means that you have to clone it with the recursive flag in order for the submodules to be cloned as well. Generaiting a ssh key for the gitlab server is recommended as it will make it easier to work with the repository.

**ssh:**
```
git clone --recursive git@gitlab.com:aurlab/mundus-mir-project/mundus_mir_gazebo_ws.git
```

**https:**
```
git clone --recursive https://gitlab.com/aurlab/mundus-mir-project/mundus_mir_gazebo_ws.git
```

While we should attempt to keep the submodules up to date on the main commit it is possible that they are not. To check for new updates to submodules run the following command in the root of the workspace.

```
git submodule update --remote
```
If there are updates you can consider adding them to the main commit by running the following command in the root of the workspace.

```
git add <path_to_submodule>
git commit -m "Update submodule <submodule_name> to latest version"
```

If you are working from this workspace you will do changes to the submodules. If you want to push these changes to their repository navigate to the submodule and push the changes there. Afterwards you can update the submodule list in the mundur_mir_gazebo_ws repository and push the changes there as well.

## Building the workspace and lauching the simulator

### Source
Before building your workspace you have to source your ROS2 instalation. This can be done by running the following command in your terminal.

```
source /opt/ros/humble/setup.bash
```

You can add this line to your `.bashrc` file to have it sourced automatically when you open a new terminal. Alternativley what I like to do is set up an alias in my bashrc file that sources the ROS2 instalation making it easy to do with one single command by adding the following to the bashrc:
```
alias src_ros="source /opt/ros/humble/setup.bash"
```

### Build
To build the workspace go to the root of the workspace and do a colcon build. 
```
colcon build
```

In case you get the following error:
```
mir/src/mundus_mir_simulator/mundus_mir_vehicle_interfaces/include/mundus_mir_vehicle_interfaces/blueye_simulator_interface.hpp:8:10: fatal error: Eigen/Dense: No such file or directory
    8 | #include <Eigen/Dense>
      |          ^~~~~~~~~~~~~
compilation terminated.
```

You can fix it by running:

```
cd /usr/local/include/
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```

The default behaviour of colcon is to build in debug mode. While this is fine for development on a laptop it is recommended to build in release mode for performance reasons when working on systems with less compute. This can be done by adding the `--cmake-args -DCMAKE_BUILD_TYPE=Release` flag to the build command. Instead of typing the entire command I prefer to make the release build an alias in my bashrc file aswell.
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If you are working in python your code is interpreted during execution which means that they do not need to be compiled. You can take advantage of this by using the `--symlink-install` flag when building your workspace. This will create symlinks to the source files instead of copying them to the install directory. This means that you can make changes to the source files and run the code without having to rebuild the workspace. If you are working with python this will speed up your developtment. However, if you add new files or change the CMakeLists.txt file you will have to rebuild the workspace once for the new symlinks to take effect. 

```
colcon build --symlink-install
```

If the package is able to build without any errors you should source the workspace. 
```
source install/setup.bash
```

The next time you are working with the workspace you don't have to rebuild but can simply source the setup file and start working. 

### Launching the simulation

Before launching the simulation you have to tell your system where it can find the Gazebo files. This is done by setting the *GZ_SIM_RESOURCE_PATH* variable to the correct path. If you are in the root directory of mundus_mir_gazebo_ws you can run:
```
export GZ_SIM_RESOURCE_PATH=$(pwd)/src/mundus_mir_simulator/gz_models
export GZ_VERSION=garden
```

If this is not done your launch files will complain about missing plugins. There is a file in the root directory called *set_env* that can be sourced giving the same effect.

```
source set_env
```

All the launchfiles can be found in *mundus_mir_simulator_launch*. Here you can also find all the configuration files. See [mundur_mir_simulator_launch](https://gitlab.com/aurlab/mundus-mir-project/packages/mundus_mir_simulator_launch). 

Too launch the mundus_mir_pipeline_world run the following command:
```
ros2 launch mundus_mir_simulator_launch mundus_mir_pipeline_world.launch.py
```

There is also a simpler world
```
ros2 launch mundus_mir_simulator_launch mundus_mir_simple_world.launch.py
```

If you have a joystick connected to your computer you should be able to control the Blueye. For some reason the mapping between the buttons of the joystick and fields in the ros2 joystick msgs seems to vary from system to system. Therefore you may need to change some paramters in the *joystick.yaml* file. Follow the guide in the README of *mundus_mir_simulator_launch*. 

