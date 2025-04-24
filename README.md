

# SE3 Controller and EasySim Setup

## Overview
This guide explains how to set up and launch the SE3 controller for multicopter takeoff, follow a circular or trajectory path, and receive image messages from Unreal Engine (UE) via EasySim and publish them to ROS.

---

## Setup Instructions

### 0. Clone Repositories and Build the Workspace

Before launching any nodes, make sure you have cloned the necessary packages into your `catkin_ws/src` folder:

```bash
cd ~/catkin_ws/src
git clone [<SE3_CONTROLLER_REPO_URL>](https://github.com/9woods123/cmd2airsim.git)
```

Then go back to the workspace root and build:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Replace `<SE3_CONTROLLER_REPO_URL>` and `<EASYSIM_ROS_WRAPPER_REPO_URL>` with the actual repository URLs.

---

### 1. Launch SE3 Controller for Takeoff

In a terminal, run the following command to launch the SE3 controller and take off the multicopter:

```bash
roslaunch se3controller se3controller.launch
```

---

### 2. Control the Multicopter to Follow a Circle

Open a new terminal and run the circular path example:

```bash
roslaunch se3controller flying_example.launch
```

---

### 3. Control the Multicopter to Follow a Trajectory

In another terminal, run the trajectory-following example:

```bash
roslaunch se3controller flying_traj_example.launch
```

---

### 4. Adjust Network Buffer Size

To prevent image message loss, increase the default network buffer size:

```bash
sudo sysctl -w net.core.rmem_default=8388608
sudo sysctl -w net.core.rmem_max=8388608
```

---

### 5. Launch EasySim ROS Wrapper to Receive Image Messages

In a new terminal, start the EasySim ROS wrapper to receive image data from Unreal Engine:

```bash
roslaunch easysim_ros_wrapper img_ros_node.launch
```

---

## Troubleshooting

- Ensure the network buffer size is set properly to avoid dropped images.
- Confirm that the UE environment is correctly configured to send image feeds.
- Check that all required dependencies are installed and sourced correctly with `source devel/setup.bash`.
