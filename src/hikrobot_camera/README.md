# 海康相机 ROS 2 驱动封装 (Hikrobot Camera ROS 2 Driver)

> 🚀 **项目亮点**：基于 Ubuntu 24.04 + ROS 2 Jazzy 封装海康 MVS SDK，实现工业相机图像发布与**动态参数配置（含严格的范围校验与错误拦截）**，并在无实物相机的环境下完成了**容错性验证**。

---

## 1. 核心功能实现 (重点验收项)

- **✅ 节点与设备连接**：实现海康 SDK 初始化、设备枚举与连接。关闭触发模式，开启连续采集。
  - *避坑容错*：在虚拟机未连接实物相机时，节点不会崩溃退出，而是进入等待状态，避免空指针段错误。
- **✅ 图像数据发布**：发布 `/image_raw` 话题，类型为 `sensor_msgs/msg/Image`。使用独立线程高速抓图，通过 `memcpy` 直接封装海康原始数据。
  - *架构决策*：直接发布标准消息，未引入 `cv_bridge` 和 OpenCV，避免 Jazzy 环境下动态库链接冲突，提升运行效率。
- **✅ 相机参数动态配置**：基于 `add_on_set_parameters_callback` 实现 `exposure_time`、`gain`、`frame_rate` 的动态设置。
  - *严格校验*：参数越界直接拒绝并返回明确原因。
  - *自动模式处理*：设置曝光/增益前，自动关闭相机自动曝光/自动增益。
  - *无硬件保护*：无相机时参数设置失败并安全报错，节点保持存活等待重连。

## 2. 环境依赖与重要避坑提醒

**操作系统**：Ubuntu 24.04 (虚拟机)
**ROS 版本**：ROS 2 Jazzy
**海康 SDK**：MVS V5.1.0 (Linux x86_64)

> ⚠️ **必看避坑说明（MVS SDK 无法通过 rosdep 自动安装）：**
> 海康 MVS SDK 未提供 rosdep 规则。请务必去官网下载 `.deb` 安装包手动安装：
> `sudo dpkg -i MVS-5.1.0_Linux_x86_64_20260909.deb`
> 若提示依赖缺失，执行：`sudo apt install -f`
> 默认路径为 `/opt/MVS`，本项目已在 `CMakeLists.txt` 中硬编码了头文件与库文件路径。

## 3. 编译与运行

编译工作空间：
`cd ~/ros2_ws`
`colcon build --packages-select hikrobot_camera`
`source install/setup.bash`

运行 (Launch 一键启动，加载默认参数)：
`ros2 launch hikrobot_camera camera.launch.py`

## 4. 测试记录与无相机调试说明

由于开发环境为虚拟机且无实物相机，无法进行实际采图。我们通过命令行参数机制，对代码的**健壮性和异常拦截能力**进行了测试。

**测试截图请查看：`.doc/param_debug.png`**

1. **参数类型校验测试**
   - 命令：`ros2 param set /hikrobot_camera_node exposure_time 10000` (整数)
   - 结果：系统拦截报错 `Wrong parameter type...`，必须输入 `10000.0`。验证了参数类型的严格性。
2. **参数范围校验测试**
   - 命令：`ros2 param set /hikrobot_camera_node exposure_time 99999999.0`
   - 结果：系统拦截报错 `曝光时间超出范围 (10 - 1000000)`。实现了任务书“校验范围”的要求。
3. **无硬件下的 SDK 设置测试**
   - 命令：`ros2 param set /hikrobot_camera_node gain 55.0`
   - 结果：终端提示 `SDK设置增益失败，请检查相机连接状态`。
   - *关键验证*：节点并未崩溃退出，依然保持后台运行。证明了异常拦截和容错机制正常工作。

## 5. 交付产物清单

- 完整的 ROS 2 功能包源码（包含 `CMakeLists.txt`, `package.xml`, `src/hik_camera_node.cpp`）
- Launch 启动文件（`launch/camera.launch.py`）
- 测试截图（`doc/param_debug.png`）
- 本 README 文档