# ROS 2 Drone Core Framework

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

本项目是一个基于 **ROS 2 Humble** 开发的轻量级无人机控制核心框架，专门针对 **PX4 飞控** 的 Offboard 模式设计。项目不仅实现了核心的飞行任务逻辑，更展示了一套完整的机器人工程化开发流。

---

## 🎯 核心功能 (Core Features)

* **模块化硬件抽象**：通过 `PX4Wrapper` 封装 MAVROS/DDS 通信协议，实现业务逻辑与底层通信的解耦。
* **有限状态机 (FSM)**：采用严谨的状态机逻辑（INIT -> ARM -> TAKEOFF -> HOVER），确保飞行任务切换的安全性与可靠性。
* **闭环高度控制**：订阅 `VehicleLocalPosition` 实时反馈，实现基于地理坐标的闭环起飞逻辑，而非简单的延时控制。
* **参数化配置系统**：集成 ROS 2 Parameter 机制，支持通过 `params.yaml` 动态调整解锁延迟、起飞高度等关键阈值。

---

## 🏗️ 架构设计 (Architecture)

项目遵循典型的机器人系统分层设计：

[Image of ROS 2 system architecture diagram with nodes, publishers, and subscribers]

1.  **应用逻辑层 (`motion_node`)**: 负责高层状态决策（FSM）与任务管理。
2.  **中间件封装层 (`px4_wrapper`)**: 负责将高层指令转换为 PX4 UORB 消息格式。
3.  **仿真/执行层 (PX4 SITL)**: 接收控制指令并执行飞行物理仿真。

---

## 🛠️ 工程化实践 (Engineering Excellence)

本项目深度集成了多项提升研发效率的工程工具：

* **容器化开发环境**：基于 **Distrobox (Fedora)** 搭建隔离开发环境，确保环境在不同机器间的 100% 可复现性。
* **IDE 深度优化**：CMake 配置自动导出 `compile_commands.json`，在 **CLion** 中实现极致的代码跳转、静态分析与补全体验。
* **自动化脚手架 (Scaffold)**：内置 `scripts/` 工具集，支持一键生成符合 Ament CMake 规范的新包。
    * 👉 **详细工具说明见：[scripts/README.md](./scripts/README.md)**

---

## 🚀 快速开始 (Quick Start)

### 1. 环境依赖
* Ubuntu 22.04 / Fedora (ROS 2 Humble)
* `px4_msgs`
* PX4-Autopilot (SITL)

### 2. 编译安装
```bash
# 进入工作空间
cd ~/ros2_drone_ws

# 编译指定包
colcon build --packages-select ros2_drone_core

# 刷新环境变量
source install/setup.zsh
明白你的意思了，你希望在 **"3. 运行控制节点"** 之后，再加上关于**代码输出（日志）以及如何验证系统运行**的说明。这样面试官看到 README 时，就能立刻知道程序运行起来是什么样子的。

以下是补充了“运行效果”和“验证步骤”的完整 Markdown 代码段。你可以直接替换掉原来 `README.md` 的后半部分：

```markdown
### 3. 运行控制节点
```bash
# 启动 Launch 文件（自动加载参数并运行节点）
ros2 launch ros2_drone_core system.launch.py

```

---

## 📊 运行预期输出 (Expected Output)

成功启动后，终端将输出如下标准化日志，展示了从参数加载到状态机启动的完整过程：

```text
[INFO] [motion_controller]: ✅ System Ready. Waiting for param config...
[INFO] [motion_controller]: ⚙️ Config: Delay=2.0s, AutoArm=1
[INFO] [motion_controller]: ⚠️ Sending ARM Command!
[INFO] [motion_controller]: 🛫 State Changed: INIT -> TAKEOFF
[INFO] [motion_controller]: 🎯 Target Reached! Altitude: 2.50m
[INFO] [motion_controller]: 🛸 State Changed: TAKEOFF -> HOVER

```

---

## 🔍 系统验证 (System Verification)

为了确保系统各层级通信正常，建议使用以下 ROS 2 指令进行验证：

### 1. 检查话题通信

验证 `PX4Wrapper` 是否成功向 FMU 发送指令：

```bash
ros2 topic echo /fmu/in/vehicle_command

```

### 2. 查看参数服务器

确认 Launch 文件是否正确注入了 YAML 配置：

```bash
ros2 param get /motion_controller arm_delay_sec

```

### 3. 可视化高度反馈

观察闭环控制中的高度变化曲线：

```bash
# 建议安装 PlotJuggler 或使用 rqt_plot
ros2 run rqt_plot rqt_plot /fmu/out/vehicle_local_position/z

```

---

## 📈 未来规划 (Roadmap)

* [ ] **TF2 集成**：实现机体坐标系与全局坐标系的实时变换展示。
* [ ] **Action 接口**：将“起飞”和“降落”改写为 Action，支持异步任务取消与进度反馈。
* [ ] **多机协同**：利用命名空间 (Namespace) 机制实现单机对多架无人机的并发控制。

---

**Author**: tuyigu

**Maintainer**: tuyigu (tuyihugy@gmail.com)

**Project**: Professional ROS 2 Drone Framework


