

# SE3 控制器与 EasySim 使用指南

## 简介
本指南将指导你如何设置并启动 SE3 控制器以实现无人机起飞、选择性地控制其按照**圆形轨迹**或**预设轨迹**飞行，并通过 EasySim 从 Unreal Engine（UE）接收图像数据并发布到 ROS。

---

## 安装与配置步骤

### 0. 克隆仓库并编译工作空间

首先将所需的包克隆到你的 `catkin_ws/src` 目录中：

```bash
cd ~/catkin_ws/src
git clone https://github.com/9woods123/cmd2airsim.git
```

然后回到工作空间根目录，编译整个工作空间：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 0.1 增加网络缓冲区大小

在启动图像传输节点之前，先调整系统的网络缓冲区大小，以避免图像数据丢失：

```bash
sudo sysctl -w net.core.rmem_default=8388608
sudo sysctl -w net.core.rmem_max=8388608
```

---

### 1. 启动 Unreal Engine（UE）仿真环境

在启动无人机控制节点之前，请确保 Unreal Engine 仿真场景已经运行并准备就绪。

---

### 2. 启动 SE3 控制器进行起飞

在一个终端中运行以下命令，启动 SE3 控制器并控制无人机起飞：

```bash
roslaunch se3controller se3controller.launch
```

![起飞截图](https://github.com/user-attachments/assets/7c34966b-2ffa-4b9d-a547-0e56d9310391)

---

### 3. 控制无人机（两种飞行方式二选一）

起飞后，你可以从以下两种飞行方式中选择一种：

#### 方式一：沿圆形路径飞行

```bash
roslaunch se3controller flying_example.launch
```

打开 RViz，并将 **Fixed Frame** 设置为 `world_enu`：

![圆形飞行 RViz 设置](https://github.com/user-attachments/assets/3a0dfe4e-90ae-44d8-8690-a8cd4f2dddf2)

---

#### 方式二：沿预设轨迹飞行

```bash
roslaunch se3controller flying_traj_example.launch
```

我们为此示例提供了预设的 RViz 配置。你可以参考它设置：

- Fixed Frame
- 图像话题（image topic）
- 里程计话题（odometry topic）

![轨迹飞行 RViz 设置](https://github.com/user-attachments/assets/ab26ab64-b533-4181-ae8b-ba5233a42870)

> 💡 以上两种控制模式**只能选择其一运行**，请勿同时启动。

---

### 4. 启动 EasySim 图像接收节点

启动 EasySim 的 ROS 包，从 UE 接收图像并发布到 ROS：

```bash
roslaunch easysim_ros_wrapper img_ros_node.launch
```

> 💡 如果你**只想单独测试图像接收功能**（不需要运行飞行控制器），也可以仅启动此节点以验证与 UE 的通信是否正常。

---

## 故障排查

- ✅ 确保 UE 环境已正确启动，并配置为发送图像数据；
- ✅ 网络缓冲区是否按照步骤 0.1 设置；
- ✅ 每个终端都需要运行 `source devel/setup.bash`；
- ✅ 如果 `catkin_make` 编译失败，请检查依赖项是否完整、包结构是否正确。

---
