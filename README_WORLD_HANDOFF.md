# Webots maze world
This folder contains the 5 × 5 Webots world for the CSCK505 group project. It was built and tested in Webots R2025a using a 32 ms basic time step.
## Files
- `worlds/maze_world.wbt` — Webots world
- `maze.py` — maze matrix and coordinate helpers
- `controllers/sensor_smoke_test/sensor_smoke_test.py` — temporary sensor test
- `evidence/` — screenshots and smoke-test output
The smoke test is diagnostic only. It does not contain the assessed navigation, mapping, finish detection or competition timing.
## Maze
```text
S...#
###.#
....#
.##..
...#F
```
Cells use zero-based `(row, column)` coordinates from the top left. Each cell is 0.25 m square.
```text
x = -0.625 + (column + 0.5) × 0.25
y =  0.625 - (row + 0.5) × 0.25
```
| Position   |    Cell | World coordinates |
| ---------- | ------: | ----------------: |
| Start `S`  | `(0,0)` |     `(-0.5, 0.5)` |
| Finish `F` | `(4,4)` |     `(0.5, -0.5)` |
The matrix constant is `MAZE`. `maze.py` also provides `cellToWorld`, `worldToCell`, `symbolAt`, `isBlocked`, `findSymbol`, `startCell` and `finishCell`.
## World setup
The arena floor is 1.25 m square. Boundary walls are 0.02 m thick and 0.10 m high, with their inner faces at `x,y = ±0.625`. Internal wall blocks occupy complete 0.25 m cells and are 0.10 m high.
The e-puck starts on `S`, facing east:
```text
translation -0.5 0.5 0
rotation 0 0 1 0
```
Reset or reload the world before every run. Do not save the world after the robot has moved, as Webots may save the resulting pose and physics state.
## Devices
Wheel motors:
```text
left wheel motor
right wheel motor
```
Infrared proximity sensors:
* front: `ps0`, `ps7`
* right: `ps1`, `ps2`
* rear: `ps3`, `ps4`
* left: `ps5`, `ps6`
The proximity sensors have a short useful range. In the smoke test they returned approximately 61–71 with the robot centred in the corridor, rising to roughly 510–550 at front-wall contact. Thresholds and wall-following offsets should therefore be calibrated in this world.
LiDAR:
* device name: `lidar`
* 180 rays
* 360° field of view
* one layer
* range: 0.02–2.0 m
* turret-slot translation: `0 0 0.02`
The e-puck turret slot places the LiDAR approximately 9.5 mm behind the robot origin. Account for this if scans are projected into world coordinates. The raised mount prevents the robot body producing self-returns and should not be lowered.
The smoke test observed ray 0 to the rear, ray 90 forward, ray 45 left and ray 135 right.
## Running and integration
Open `worlds/maze_world.wbt` in Webots R2025a and run the simulation. The smoke test drives forward briefly, reports both sensor types and stops.
To use the main controller, place its directory under `controllers/` and change the e-puck `controller` field from `sensor_smoke_test` to the directory name.
For the assessed comparison:
* Experiment 1 uses `ps0`–`ps7` only.
* Experiment 2 uses `lidar` only.
* Do not fuse the sensor types.
* Keep the maze, start pose, navigation policy and timing method unchanged.
Test evidence is in:
```text
evidence/01_overhead_maze.png
evidence/02_epuck_lidar_perspective.png
evidence/03_console_smoke_test.png
evidence/03_console_transcript.md
```
## Known limitation
The blocks at `(3,1)` and `(3,2)` connect to the boundary structure only at a corner. A wall follower may transfer between them and the boundary wall, so the controller must be tested for looping.
A valid route is:
```text
(0,0) → (0,1) → (0,2) → (0,3) → (1,3) → (2,3)
→ (3,3) → (3,4) → (4,4)
```
