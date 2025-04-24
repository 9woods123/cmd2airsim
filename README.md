

# SE3 Controller and EasySim Setup

## Overview
This guide explains how to set up and launch the SE3 controller for multicopter takeoff, control the multicopter to follow either a circular or a trajectory path, and receive image messages from Unreal Engine (UE) via EasySim and publish them to ROS.  

## 中文README
https://github.com/9woods123/cmd2airsim/blob/master/README_ch.md

---

## Setup Instructions

### 0. Clone Repositories and Build the Workspace

First, clone the necessary packages into your `catkin_ws/src` directory:

```bash
cd ~/catkin_ws/src
git clone https://github.com/9woods123/cmd2airsim.git
```

Then, go back to the workspace root and build the project:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 0.1 Adjust Network Buffer Size

Before launching the image streaming node, increase the system network buffer size to avoid dropped image messages:

```bash
sudo sysctl -w net.core.rmem_default=8388608
sudo sysctl -w net.core.rmem_max=8388608
```

---

### 1. Start Unreal Engine (UE) Simulation

Make sure the UE simulation environment is up and running before starting the SE3 controller.

---

### 2. Launch SE3 Controller for Takeoff

In a terminal, launch the SE3 controller to initiate multicopter takeoff:

```bash
roslaunch se3controller se3controller.launch
```

![Takeoff Screenshot](https://github.com/user-attachments/assets/7c34966b-2ffa-4b9d-a547-0e56d9310391)

---

### 3. Control the Multicopter (Choose One)

After takeoff, you can choose **one** of the following flight modes:

#### Option 1: Follow a Circular Path

```bash
roslaunch se3controller flying_example.launch
```

Open **RViz** and set the **Fixed Frame** to `world_enu`:

![Circular Path RViz](https://github.com/user-attachments/assets/3a0dfe4e-90ae-44d8-8690-a8cd4f2dddf2)

---

#### Option 2: Follow a Predefined Trajectory

```bash
roslaunch se3controller flying_traj_example.launch
```

We provide a default RViz configuration for this example. You can refer to it to set the **Fixed Frame**, **image topic**, and **odometry topic**:

![Trajectory RViz](https://github.com/user-attachments/assets/ab26ab64-b533-4181-ae8b-ba5233a42870)

> 💡 Only **one** control mode should be launched at a time.

---

### 4. Launch EasySim ROS Wrapper to Receive Image Messages

To receive and publish image messages from Unreal Engine, launch the EasySim ROS wrapper:

```bash
roslaunch easysim_ros_wrapper img_ros_node.launch
```

> 💡 If you want to **test image reception separately** (without the SE3 controller), this node can be launched independently to verify communication with UE.

---

## Troubleshooting

- ✅ Ensure the **UE environment** is running and configured to stream image data correctly.
- ✅ Confirm that the **network buffer size** was increased (step 0.1).
- ✅ Use `source devel/setup.bash` in every terminal before launching ROS nodes.
- ✅ If `catkin_make` fails, check for missing dependencies or incorrect package structure.

---
