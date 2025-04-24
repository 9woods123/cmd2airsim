
# SE3 Controller and EasySim Setup

## Overview
This guide explains how to set up and launch the SE3 controller for multicopter takeoff, control the multicopter to follow a circular or trajectory path (choose one), and receive image messages from Unreal Engine (UE) via EasySim and publish them to ROS.

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

Make sure the UE game environment is already running and ready to simulate the drone before launching the takeoff controller.

---

### 2. Launch SE3 Controller for Takeoff

In a terminal, launch the SE3 controller to take off the multicopter:

```bash
roslaunch se3controller se3controller.launch
```

![2025-04-24 19-18-39 的屏幕截图](https://github.com/user-attachments/assets/7c34966b-2ffa-4b9d-a547-0e56d9310391)

---

### 3. Control the Multicopter (Choose One)

After takeoff, you can choose **one** of the following control modes:

- **To follow a circular path**, run:

  ```bash
  roslaunch se3controller flying_example.launch
  ```

open rviz and set frame se "world_enu":

![2025-04-24 19-19-59 的屏幕截图](https://github.com/user-attachments/assets/3a0dfe4e-90ae-44d8-8690-a8cd4f2dddf2)


- **To follow a predefined trajectory**, run:

  ```bash
  roslaunch se3controller flying_traj_example.launch
  ```
  We give your a predefault rviz setting, so you can looking into this example for rviz setting including fixed frame, img topic, odomtry topic.

![2025-04-24 19-23-58 的屏幕截图](https://github.com/user-attachments/assets/ab26ab64-b533-4181-ae8b-ba5233a42870)

you can looking into this example for rviz setting including fixed frame, img topic, odomtry topic.

> 💡 Only one control mode should be run at a time.

---

### 4. Launch EasySim ROS Wrapper to Receive Image Messages

To receive and publish image messages from Unreal Engine, start the EasySim ROS wrapper:

```bash
roslaunch easysim_ros_wrapper img_ros_node.launch
```

> 💡 If you want to **test image reception separately** (without running the full SE3 controller), you can launch only this node to verify communication with Unreal Engine.

---

## Troubleshooting

- ✅ Ensure the **UE environment** is running and properly configured to stream image data.
- ✅ Verify the **network buffer size** was increased to avoid dropped images.
- ✅ Make sure you've run `source devel/setup.bash` in each terminal before launching ROS nodes.
- ✅ If `catkin_make` fails, check for missing ROS dependencies or incorrect package paths.

---
