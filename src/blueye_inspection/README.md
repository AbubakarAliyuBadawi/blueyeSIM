# blueye_inspection

ArUco-triggered pipeline inspection for the Blueye Stonefish scenario.

The package follows the same launch pattern as `mundus_mir_stonefish_docking_controller`:
camera marker detection gates an inspection controller, which publishes `/blueye/ref_vel`
for the existing velocity controller and Stonefish thruster allocator.

Launch after the Blueye Stonefish simulation is running:

```bash
ros2 launch blueye_inspection blueye_inspection.launch.py auto_start:=true
```

Show the annotated inspection camera feed:

```bash
ros2 launch blueye_inspection blueye_inspection.launch.py auto_start:=true debug_view:=true
```

The same annotated image is also published on:

```text
/blueye/inspection/debug_image
```

Important: the current `pipe.scn` uses the same `Aruco19` look on all six
pipeline marker plates. That means camera detection can answer "a pipeline
marker is visible", but it cannot distinguish marker 1 from marker 6 by ID.
This package therefore uses ArUco visibility as a safety/inspection gate while
following the known marker sequence from the simulated pipe geometry.

To make the inspection fully ID-driven like the docking board, give each pipe
marker a unique detectable ArUco look, for example `Aruco5x5_0` through
`Aruco5x5_5`, then update `accepted_marker_ids` and the controller mapping.
