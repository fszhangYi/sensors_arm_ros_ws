# sensors_arm_ros_ws

ROS 2 **Foxy** + Gazebo Classic 11 工作区：把 [sensors-arm](../sensors-arm) 多 Cell 可视化/控制迁到 ROS，Cell C 腕部 **D405 视场**以 Gazebo `CameraInfo` 投影为主验收路径。

架构关系图（与 sensors-arm 的复用 / 替代）：[`docs/architecture.png`](docs/architecture.png) · [SVG](docs/architecture.svg) · [源 DOT](docs/architecture.dot)

---

## 快速开始（推荐用脚本）

```bash
cd ~/autodl-tmp/sensors_arm_ros_ws

# 首次 / 改代码后编译（必须用系统 Python 3.8，勿混 conda）
./scripts/build.sh

# 仿真全家桶：Gazebo FOV + 控制 + rosbridge + Web UI
./scripts/start_sim.sh
# 浏览器打开 http://127.0.0.1:6008/

# Gazebo + FOV 服务（默认无头；GUI=true 开 gzclient）
./scripts/start_fov_sim.sh
GUI=true ./scripts/start_fov_sim.sh

# 真机 / dry-run（无 Gazebo）
./scripts/start_hw.sh                  # 默认 dry_run:=true
DRY_RUN=false ./scripts/start_hw.sh    # 实机

# 只起 Web（依赖已有 ROS / rosbridge，或走离线 FOV）
./scripts/start_web.sh

# FOV 命令行
./scripts/fov_cli.sh                   # 离线投影 HD87
./scripts/fov_cli.sh --ros             # 调用 /fov/validate
./scripts/fov_cli.sh --mode VGA80 --zip /tmp/fov.zip

# 停掉本栈相关进程
./scripts/stop.sh
```

脚本一览：

| 脚本 | 作用 |
|------|------|
| `scripts/env.sh` | `source`：Foxy + 本工作区环境（去 conda、设 `SENSORS_SRC`） |
| `scripts/build.sh` | `colcon build --symlink-install` |
| `scripts/start_sim.sh` | `multi_cell_sim` 全仿真 + Web `:6008` |
| `scripts/start_fov_sim.sh` | Gazebo Cell C + FOV 节点（`GUI` 可选） |
| `scripts/start_hw.sh` | 无 Gazebo 的 hw/control/FOV/web |
| `scripts/start_web.sh` | 仅 Web UI |
| `scripts/fov_cli.sh` | FOV CLI 封装 |
| `scripts/stop.sh` | 结束 gzserver / bringup / rosbridge / web 等 |

环境变量（可选）：

| 变量 | 默认 | 说明 |
|------|------|------|
| `WEB_PORT` | `6008` | Web 端口 |
| `START_WEB` | `true` | 是否随 launch 起 Web |
| `GUI` | `false` | 是否启动 Gazebo `gzclient` |
| `DRY_RUN` | `true` | `start_hw.sh` 是否 dry-run |
| `ARM_ID` | `arm-c` | 默认工位 |
| `ROS_LOCALHOST_ONLY` | `1`（经 `SENSORS_ARM_ROS_LOCALHOST_ONLY`） | 仅本机 DDS |
| `ROS_DOMAIN_ID` | `31`（经 `SENSORS_ARM_ROS_DOMAIN_ID`） | 与本机其它 ROS2（如 Foxglove snap）隔离 |
| `SENSORS_SRC` | `~/autodl-tmp/sensors/src` | hik-sensors 源码路径 |

---

## 包一览

| Package | Role |
|---------|------|
| `sensors_arm_description` | EC616 / CS66 / Cell C xacro + D405 optical + 网格 symlink |
| `sensors_arm_gazebo` | Cell C world、`gazebo_ros_camera`、无头 launch |
| `sensors_arm_msgs` | `FovValidate` / `ArmCommand` / `ArmConnect` / `ArmIk` |
| `sensors_arm_fov` | 视场校验节点 + CLI + 几何对照脚本 |
| `sensors_arm_control` | Sim 关节、abs ramp、IK |
| `sensors_arm_hw` | hik-sensors 桥（dry_run / hardware） |
| `sensors_arm_web` | 同等 Three.js UI（默认 **6008**）→ rosbridge/ROS |
| `sensors_arm_bringup` | 一键 launch |

---

## 常用命令（手工）

### 环境

```bash
# 重要：Foxy 用系统 Python 3.8，不要用 conda
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:$HOME/.local/bin"
source /opt/ros/foxy/setup.bash
source ~/autodl-tmp/sensors_arm_ros_ws/install/setup.bash
# 或：
source ~/autodl-tmp/sensors_arm_ros_ws/scripts/env.sh
```

### 编译

```bash
cd ~/autodl-tmp/sensors_arm_ros_ws
colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
# 等价：./scripts/build.sh
```

### Launch

```bash
# Gazebo + FOV（默认无头）
ros2 launch sensors_arm_bringup cell_c_gazebo_fov.launch.py
ros2 launch sensors_arm_bringup cell_c_gazebo_fov.launch.py gui:=true

# 全仿真 + rosbridge + Web :6008
ros2 launch sensors_arm_bringup multi_cell_sim.launch.py
ros2 launch sensors_arm_bringup multi_cell_sim.launch.py start_web:=false
ros2 launch sensors_arm_bringup multi_cell_sim.launch.py web_port:=6010
ros2 launch sensors_arm_bringup multi_cell_sim.launch.py gui:=true

# 真机 / dry-run（无 Gazebo）
ros2 launch sensors_arm_bringup hardware.launch.py dry_run:=true
ros2 launch sensors_arm_bringup hardware.launch.py dry_run:=false arm_id:=arm-c
```

### FOV

```bash
CFG=$(ros2 pkg prefix sensors_arm_fov)/share/sensors_arm_fov/config/cell_c_fov.yaml

# 离线投影（不依赖 Gazebo）
ros2 run sensors_arm_fov fov_cli -c "$CFG" --mode HD87

# 走 ROS 服务（需先 start_fov_sim / start_sim）
ros2 run sensors_arm_fov fov_cli -c "$CFG" --mode HD87 --ros
ros2 run sensors_arm_fov fov_cli -c "$CFG" --mode HD87 --ros --zip /tmp/fov.zip

# 话题 / 服务速查
ros2 topic list | grep -E 'd405|joint_states|clock'
ros2 service list | grep fov
```

### 控制（sim）

`./scripts/start_sim.sh` 下 Web / `/arm/command` 改 `/joint_states`；Gazebo 用运动学插件每步 `SetPosition` 跟随（无动力学 PID）。

```bash
# 需已有 arm_control_node（start_sim 会起）
ros2 service call /arm/command sensors_arm_msgs/srv/ArmCommand \
  "{command: {arm_id: 'arm-c', action: 'connect', joints: [], jog_delta: [], speed_scale: 1.0, cancel_abs_ramp: false}}"

ros2 service call /arm/command sensors_arm_msgs/srv/ArmCommand \
  "{command: {arm_id: 'arm-c', action: 'home', joints: [], jog_delta: [], speed_scale: 1.0, cancel_abs_ramp: false}}"

ros2 service call /arm/command sensors_arm_msgs/srv/ArmCommand \
  "{command: {arm_id: 'arm-c', action: 'jog', joints: [], jog_delta: [0.1, 0, 0, 0, 0, 0, 0], speed_scale: 1.0, cancel_abs_ramp: false}}"
```

### 仅 Web

```bash
# 先 source 工作区，再用 venv 的 FastAPI
source ~/autodl-tmp/sensors_arm_ros_ws/scripts/env.sh
~/autodl-tmp/.venv/bin/python -m sensors_arm_web.web_server --port 6008 --mode sim
# 等价：./scripts/start_web.sh
```

### 描述 / 调试

```bash
# 展开 Cell C URDF
xacro $(ros2 pkg prefix sensors_arm_description)/share/sensors_arm_description/urdf/cell_c.urdf.xacro > /tmp/cell_c.urdf

# 仅 RSP 显示（无 Gazebo）
ros2 launch sensors_arm_description display.launch.py
```

---

## 注意

- ROS 节点跑 **Python 3.8 + Foxy**；`colcon build` 不要带 conda。
- Web（FastAPI）用 `~/autodl-tmp/.venv`（通常是 3.12），经 rosbridge 连 ROS。
- 网格是指向 `sensors-arm/.../meshes` 的 **symlink**。
- 产品视场路径是 CameraInfo 投影；几何 `fov.analyze` 仅对照，见 `src/sensors_arm_fov/scripts/compare_geom_vs_sim.py`。
