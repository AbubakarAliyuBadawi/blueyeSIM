# This is a refactored code from blueye-ros2-interface

This only works for simulation.
I removed image_processing and other packages and just moved everything under ONE package which is the mundus_mir_docking_controller pkg. 

## How to launch

You will need 3 terminals. 

Terminal 1:
```
cd mundus_mir_simulator
colcon build
source install/setup.bash 
ros2 launch mundus_mir_simulator_launch generated_mundus_mir_pipeline_world.launch.py 

```

Terminal 2:
```
cd mundus_mir_simulator
source install/setup.bash 
ros2 launch mundus_mir_docking_controller docking.launch.py 

The launch file above starts the aruco tag detector. The Blueye needs to up to 10 seconds to start correctly estimating its pose.

```

Terminal 3:


```
- To control the drone with arrow keys: 

python3 keyboard_controller.py

- To initiate autonomous waypoint following:

source install/setup.bash 
ros2 run mundus_mir_docking_controller docking_sequence


```



