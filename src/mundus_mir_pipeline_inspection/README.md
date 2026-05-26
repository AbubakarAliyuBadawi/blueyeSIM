# mundus_mir_pipeline_inspection

Simple marker-based pipeline inspection mission launcher for Blueye in Stonefish.

The first version does not run camera detection. It uses the known marker layout
from `stonefish_sim/objects/pipe.scn`, converts marker positions into inspection
waypoints, and sends them to the existing waypoint controller services.

