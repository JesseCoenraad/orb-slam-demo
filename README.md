# ORB-SLAM Demo — Duckietown

A ROS-based implementation of a monocular ORB-SLAM pipeline for Duckietown.
The system reads a compressed camera stream, builds a sparse 3-D point cloud
of the environment in real time, and displays it in a live viewer window.

---

## Table of Contents

1. [What is ORB-SLAM?](#what-is-orb-slam)
2. [Repository Structure](#repository-structure)
3. [ROS Architecture](#ros-architecture)
4. [Prerequisites](#prerequisites)
5. [Running in the Duckiematrix (simulation)](#running-in-the-duckiematrix-simulation)
6. [Running on a Real Duckiebot](#running-on-a-real-duckiebot)
7. [Configuration Parameters](#configuration-parameters)
8. [Known Limitations](#known-limitations)

---

## What is ORB-SLAM?

**SLAM** (Simultaneous Localization and Mapping) is the problem of building a
map of an unknown environment while at the same time tracking a robot's
position within that map — using only sensor data.

**ORB-SLAM** solves this with a monocular camera by chaining four steps on
every pair of consecutive frames:

### 1. Feature Extraction

ORB (**O**riented FAST and **R**otated **B**RIEF) detectors locate visually
distinctive corner-like regions in each greyscale frame.  For each region the
algorithm computes a compact binary descriptor that is invariant to rotation
and robust to noise.  Up to `n_features` keypoints are extracted per frame
(default 5 000).

### 2. Feature Matching

The binary descriptors from the current frame are compared to those from the
previous frame using a **Brute-Force Hamming-distance** matcher with
cross-check enabled.  Matches are sorted by distance so that the most
confident correspondences are used downstream.

### 3. Camera Pose Estimation

From the matched pixel pairs, the **Fundamental matrix** F is estimated with
**RANSAC** to reject outlier matches.  F is then converted to the **Essential
matrix** E using the camera intrinsic matrix K:

```
E = K^T · F · K
```

OpenCV's `recoverPose` decomposes E into a rotation matrix **R** and a
(unit-norm) translation vector **t** — the relative motion of the camera
between the two frames.  The node maintains a running camera-to-world
transform by composing these relative poses frame by frame:

```
R_cw_new = R_cw_old · R^T
t_cw_new = t_cw_old − R_cw_new · t
```

### 4. Triangulation for 3-D Mapping

With the relative pose known, matched pixel coordinates are **undistorted**
into normalised camera coordinates and then triangulated using
`cv2.triangulatePoints`.  Points that project behind either camera are
discarded.  The surviving 3-D points are transformed from the local camera
frame into the global (world) frame and accumulated in a rolling map buffer.

The resulting point cloud is published as a `sensor_msgs/PointCloud2` message
and rendered live in a viewer window (Open3D when available, matplotlib
otherwise).

> **Scale ambiguity** — because a single camera cannot recover the absolute
> scale of the scene, all distances in the map are relative.  The shape of
> obstacles is preserved, but metric dimensions are not.

---

## Repository Structure

```
orb-slam-demo/
├── Dockerfile                  # Duckietown dt-core–based image
├── dependencies-apt.txt        # APT packages (libgl, python3-tk, …)
├── dependencies-py3.txt        # pip packages (open3d, matplotlib)
├── launchers/
│   └── default.sh              # Entrypoint — runs roslaunch
├── notebooks/
│   └── SLAM_pipeline_step_by_step.ipynb   # Step-by-step prototype
└── packages/
    └── orb_slam_node/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── orb_slam.launch # Launches both ROS nodes
        └── src/
            ├── orb_slam_node.py              # SLAM pipeline
            └── pointcloud_visualizer_node.py # Live viewer
```

---

## ROS Architecture

### Nodes

| Node | Script | Role |
|---|---|---|
| `orb_slam_node` | `orb_slam_node.py` | Runs the SLAM pipeline; publishes the point cloud |
| `pointcloud_visualizer_node` | `pointcloud_visualizer_node.py` | Opens a viewer window and renders the point cloud |

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera_node/image/compressed` | `sensor_msgs/CompressedImage` | Input to SLAM node |
| `/camera_node/camera_info` | `sensor_msgs/CameraInfo` | Input — provides camera intrinsics |
| `/orb_slam/point_cloud` | `sensor_msgs/PointCloud2` | Output of SLAM node → input to visualizer |

> On a real Duckiebot or in the Duckiematrix the camera topics are prefixed
> with the robot name, e.g. `/[ROBOT_NAME]/camera_node/image/compressed`.
> Update the `image_topic` and `camera_info_topic` parameters in
> `launch/orb_slam.launch` accordingly, or remap them at launch time.

---

## Prerequisites

- **Docker** and the **Duckietown Shell** (`dts`) installed on your laptop
- A display available for the viewer window — ensure `$DISPLAY` is set
  (on Linux this is usually `:0`; for remote sessions use X-forwarding)
- For the Duckiematrix: Duckietown Shell with the `matrix` command set
- For a real Duckiebot: the robot flashed with Duckietown OS (ente)

Install or update the Duckietown Shell:

```bash
pip install --user --upgrade duckietown-shell
dts update
```

---

## Running in the Duckiematrix (simulation)

The Duckiematrix provides a simulated Duckietown environment with virtual
cameras that publish on the same ROS topics as a real robot.

### 1. Build the image for amd64

```bash
git clone https://github.com/VikramRadhakrishnan/orb-slam-demo.git
cd orb-slam-demo
dts devel build -a amd64
```

### 2. Start the Duckiematrix

Follow the Duckiematrix setup for your configuration.  Once running, note the
simulated robot name (e.g. `duckiebot`).

### 3. Remap the camera topics

Edit `packages/orb_slam_node/launch/orb_slam.launch` and set the topic
parameters to match the simulated robot name:

```xml
<param name="image_topic"       value="/duckiebot/camera_node/image/compressed"/>
<param name="camera_info_topic" value="/duckiebot/camera_node/camera_info"/>
```

### 4. Run the container connected to the matrix

```bash
dts devel run \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --net host
```

The `--net host` flag makes the container share the host network so it can
reach the ROS master running inside the Duckiematrix.  The `DISPLAY` passthrough
lets the viewer window appear on your laptop screen.

A 1 280 × 720 Open3D window (or a matplotlib window if Open3D is unavailable)
will open and fill with green points as the simulated robot moves.

---

## Running on a Real Duckiebot

### 1. Build the image for arm64

```bash
dts devel build -a arm64v8 -H [DUCKIEBOT_HOSTNAME]
```

This cross-compiles the image and transfers it to the robot.

Alternatively, build locally and push to Docker Hub, then pull on the robot:

```bash
dts devel build -a arm64v8 --push
ssh [DUCKIEBOT_HOSTNAME].local docker pull duckietown/orb-slam-demo:v2-arm64v8
```

### 2. Remap the camera topics

Edit `launch/orb_slam.launch` to use the robot's actual topic namespace:

```xml
<param name="image_topic"       value="/[DUCKIEBOT_HOSTNAME]/camera_node/image/compressed"/>
<param name="camera_info_topic" value="/[DUCKIEBOT_HOSTNAME]/camera_node/camera_info"/>
```

### 3. Update the camera intrinsics (important)

The default camera matrix in the launch file was measured from a specific
camera and will not be accurate for your Duckiebot.  If `camera_info` is
published by your camera driver (it usually is after calibration), the node
picks it up automatically and the fallback matrix is ignored.

If your robot has not been calibrated, run the Duckietown camera calibration
procedure first:

```bash
dts duckiebot calibrate_intrinsics [DUCKIEBOT_HOSTNAME]
```

### 4. Run the container on the robot

```bash
dts devel run \
  -H [DUCKIEBOT_HOSTNAME] \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --net host
```

For the viewer window to appear on your laptop rather than on the robot,
set `DISPLAY` to your laptop's display and allow X connections before running:

```bash
xhost +local:docker
DISPLAY=:0 dts devel run -H [DUCKIEBOT_HOSTNAME] \
  -e DISPLAY=:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --net host
```

---

## Configuration Parameters

All parameters are set in `packages/orb_slam_node/launch/orb_slam.launch`.

### `orb_slam_node`

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/camera_node/image/compressed` | Input compressed image topic |
| `camera_info_topic` | `/camera_node/camera_info` | Camera info topic (for intrinsics) |
| `n_features` | `5000` | ORB keypoints extracted per frame. Higher → denser map, more CPU |
| `max_map_points` | `10000` | Maximum 3-D points kept in the rolling map buffer |
| `camera_matrix` | see launch file | Fallback K matrix (9 floats, row-major) used when `camera_info` is absent |

### `pointcloud_visualizer_node`

| Parameter | Default | Description |
|---|---|---|
| `point_cloud_topic` | `/orb_slam/point_cloud` | Input point cloud topic |
| `max_display_pts` | `8000` | Points are randomly subsampled to this cap before rendering |
| `point_size` | `2.0` | Rendered point size in the viewer |

---

## Known Limitations

- **Scale ambiguity** — monocular SLAM cannot recover metric scale.  The
  shape of the map is correct but all distances are up to an unknown scale
  factor.  Stereo or RGB-D cameras are needed for metric maps.

- **No loop closure** — the current implementation does not detect when the
  camera revisits a previously seen location, so drift accumulates over long
  trajectories.

- **Open3D on arm64** — Open3D pip wheels may not be available for all arm64
  Linux distributions.  The visualizer node automatically falls back to
  matplotlib in that case.  Alternatively, subscribe to `/orb_slam/point_cloud`
  with RViz running on a separate machine.

- **Texture-poor environments** — ORB features require visible texture.
  Plain white walls or low-light scenes will produce few matches and a sparse
  or empty map.
