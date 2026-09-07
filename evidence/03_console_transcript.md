# Smoke-test console transcript

Webots R2025a, `worlds/maze_world.wbt`, controller `sensor_smoke_test`, 6 September 2026.

These are transcriptions of the Webots console, not screenshots. Evidence items 3 and 4 in the task brief ask for console **screenshots**; those are attached seperately.

---

## Run A — delivered smoke test (2 s, 2.0 rad/s), robot at the start cell

```text
INFO: sensor_smoke_test: Starting controller: "C:\Users\Owner\CSCK505 Robotics\.venv\Scripts\python.exe" -u sensor_smoke_test.py
SMOKE TEST: temporary validation code, not assessed.
SMOKE TEST: sampling period 32 ms.
SMOKE TEST: lidar 180 rays x 1 layer(s), fov 6.2832 rad.
SMOKE TEST: lidar range 0.020 m to 2.000 m.
t=0.00 s  PROX  ps0=  64  ps1=  70  ps2=  64  ps3=  68  ps4=  61  ps5=  67  ps6=  71  ps7=  70
t=0.00 s  LIDAR 180/180 finite  min=0.1151 m at ray 0  max=0.902 m  mean=0.215 m  r45=0.125  r90=0.885  r135=0.125  selfHits=0
t=0.51 s  PROX  ps0=  61  ps1=  66  ps2=  69  ps3=  67  ps4=  66  ps5=  70  ps6=  71  ps7=  67
t=1.02 s  PROX  ps0=  61  ps1=  65  ps2=  69  ps3=  68  ps4=  71  ps5=  71  ps6=  71  ps7=  63
t=1.02 s  LIDAR 180/180 finite  min=0.1250 m at ray 134  max=0.858 m  mean=0.224 m  r45=0.125  r90=0.842  r135=0.125  selfHits=0
t=1.50 s  PROX  ps0=  62  ps1=  68  ps2=  69  ps3=  70  ps4=  67  ps5=  64  ps6=  71  ps7=  67
t=1.50 s  LIDAR 180/180 finite  min=0.1250 m at ray 44  max=0.839 m  mean=0.227 m  r45=0.125  r90=0.823  r135=0.125  selfHits=0
t=2.02 s  PROX  ps0=  67  ps1=  65  ps2=  67  ps3=  66  ps4=  67  ps5=  65  ps6=  61  ps7=  61
t=2.02 s  LIDAR 180/180 finite  min=0.1250 m at ray 134  max=0.823 m  mean=0.232 m  r45=0.125  r90=0.802  r135=0.125  selfHits=0
SMOKE TEST: complete, robot stopped, no faults raised.
INFO: 'sensor_smoke_test' controller exited successfully.
```

**What this establishes.** All eight proximity sensors and the LiDAR resolve by name and return data. Sampling period is 32 ms. In open space the proximity baseline is 61–71. The LiDAR returns 180 finite ranges of 180, with 0.125 m to the north and south walls and a value decreasing from 0.885 m to 0.802 m along the open corridor east as the robot advances. No self-detection.

The forward and rear readings at t=0.00 (0.885 m and 0.1151 m, summing to 1.000 m across a 1.25 m arena from a robot 0.125 m off the west wall) are what establish the LiDAR's 0.010 m rearward mounting offset.

---

## Run B — temporary wall-contact test (12 s, 6.28 rad/s)

Run only to prove collision geometry and proximity response. The drive speed and duration were restored to the delivered values immediately afterwards.

```text
t=9.02 s  PROX  ps0= 525  ps1= 128  ps2=  64  ps3=  68  ps4=  67  ps5=  63  ps6= 134  ps7= 547
t=9.02 s  LIDAR 180/180 finite  min=0.0466 m at ray 89  max=0.961 m  mean=0.267 m  r45=0.125  r90=0.047  r135=0.875  selfHits=22
t=10.53 s  PROX  ps0= 507  ps1= 120  ps2=  65  ps3=  66  ps4=  64  ps5=  63  ps6= 128  ps7= 541
t=12.00 s  PROX  ps0= 513  ps1= 127  ps2=  71  ps3=  71  ps4=  68  ps5=  63  ps6= 119  ps7= 502
t=12.00 s  LIDAR 180/180 finite  min=0.0466 m at ray 89  max=0.961 m  mean=0.267 m  r45=0.125  r90=0.047  r135=0.875  selfHits=22
SMOKE TEST: complete, robot stopped, no faults raised.
INFO: 'sensor_smoke_test' controller exited successfully.
```

**What this establishes.** The robot travelled the length of the row-0 corridor and was physically stopped by `wallBlockR0C4`, so the internal blocks have working collision geometry. The front proximity pair rose from a ~65 baseline to 525 and 547, the front diagonals to 128 and 134, while the rear and side sensors stayed at baseline. Forward LiDAR read 0.047 m and the rearward bearing opened to 0.875 m.

`selfHits=22` here is not self-detection: with the robot 0.047 m from a flat wall, a forward cone of genuine wall returns falls under the 0.05 m diagnostic threshold.

---

## Rejected configuration, recorded so it is not repeated

With the LiDAR at the bare turret-slot origin — that is, without the `translation 0 0 0.02` it now carries — the rays around the rear bearing struck the robot's own body:

```text
LIDAR 180/180 finite  min=0.0203 m at ray 0  max=0.858 m  mean=0.212 m  r0=0.020  r1=0.021  r45=0.125  r90=0.842  r135=0.125  r178=0.021  r179=0.020
```

Rays 0, 1, 178 and 179 all returned about 0.020 m against a `minRange` of 0.020 m, while the side and forward bearings were correct. Raising the sensor 0.02 m within the slot clears the body and returns `selfHits` to 0.
