# nav2_robocon — ROBOCON 2026 全向导航

基于 ROS 2 Nav2 (MPPI) 的全向移动机器人自主导航包。

## 构建

```bash
cd ~/nav2_ws_new
colcon build --packages-select nav2_robocon
source install/setup.bash
```

## 启动

```bash
ros2 launch nav2_robocon third_area_single.launch.py team:=red   # 红队
ros2 launch nav2_robocon third_area_single.launch.py team:=blue  # 蓝队
```

## 发送导航目标

### 方式一：`/nav_goal`（推荐，直接输入角度）

类型 `std_msgs/msg/Float32MultiArray`，数据格式 `[x, y, yaw]`，yaw 单位**弧度**。

```bash
# 导航到 (7.7, 1.6)，朝向 90°
ros2 topic pub /nav_goal std_msgs/msg/Float32MultiArray \
  "{data: [7.7, 1.6, 1.57]}" -1

# 导航到 (6.0, 2.5)，朝向 -90°（-π/2）
ros2 topic pub /nav_goal std_msgs/msg/Float32MultiArray \
  "{data: [6.0, 2.5, -1.57]}" -1
```

**常用角度速查：**

| 角度 | 弧度 | 朝向 |
|------|------|------|
| 0° | 0.0 | → +x |
| 90° | 1.57 | → +y |
| 180° | 3.14 | → -x |
| -90° | -1.57 | → -y |

### 方式二：`/single_nav_goal`（PoseStamped，兼容 RViz）

```bash
ros2 topic pub /single_nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, \
    pose: {position: {x: 7.7, y: 1.6}, \
           orientation: {z: 0.707, w: 0.707}}}" -1
```

## 行为说明

1. 收到目标后，**记录目标 yaw** 到 `/desired_yaw`（待命），调用 Nav2 `NavigateToPose` 导航
2. **正常路段**：Nav2 自由控制 yaw，边走边转
3. **Ramp 区域**：`ramp_zone_manager` 自动接管，yaw 锁死为目标朝向 + 悬挂升高 + 最低限速
4. 离开 ramp 后：yaw 交还 Nav2，悬挂降低
5. 到达目标后解除 yaw 待命状态，等待下一个目标
6. 导航中收到新目标会被忽略（需等当前导航完成）

## 节点架构

```
/nav_goal [x,y,yaw]
/single_nav_goal (PoseStamped)
        │
        ▼
 third_area_single  ──► /desired_yaw (Float32)
        │
        ▼
 NavigateToPose action ──► Nav2 (MPPI controller)
        │
        ▼
    /cmd_vel ──► ramp_zone_manager ──► /cmd_vel_adjusted ──► cmd_vel_bridge ──► 底盘
                     │
        /odin1/relocation (位姿)
```

## Ramp 斜坡参数

`ramp_zone_manager` 的可配置参数，通过 launch 或 yaml 传入：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `team` | red | 队伍，影响 ramp Y 默认值 |
| `ramp_x_min` | 8.9 | 斜坡 X 起点 (m) |
| `ramp_x_max` | 10.4 | 斜坡 X 终点 (m) |
| `ramp_y_min` | 蓝 3.1 / 红 -4.6 | 斜坡 Y 下界 (m) |
| `ramp_y_max` | 蓝 4.6 / 红 -3.1 | 斜坡 Y 上界 (m) |
| `min_ramp_speed` | 0.25 | 斜坡区最低速 (m/s) |
| `suspension_ramp` | 75.0 | 斜坡悬挂高度 (mm) |
| `suspension_flat` | 30.0 | 平地悬挂高度 (mm) |
| `yaw_kp` | 2.0 | yaw 保持 P 增益 |
| `yaw_max_vel` | 2.0 | yaw 最大角速度 (rad/s) |

**覆盖参数示例：**

```bash
ros2 run nav2_robocon ramp_zone_manager --ros-args \
  -p team:=blue \
  -p ramp_x_min:=9.0 \
  -p ramp_y_min:=3.0 \
  -p ramp_y_max:=4.5
```

或在 launch 文件中：

```python
Node(
    package="nav2_robocon",
    executable="ramp_zone_manager",
    parameters=[{
        "team": "blue",
        "ramp_y_min": 3.1,
        "ramp_y_max": 4.6,
    }],
)
```

## 改地图时需同步修改

| 文件 | 改什么 |
|------|--------|
| `maps/field_red.pgm` | 新地图图片（红队） |
| `maps/field_blue.pgm` | 新地图图片（蓝队） |
| `maps/field_red.yaml` | `resolution`, `origin` |
| `maps/field_blue.yaml` | `resolution`, `origin` |
| `scripts/generate_map.py` | 场地尺寸常量（如用脚本生成） |
| ramp 参数 | `ramp_x_min/max`, `ramp_y_min/max`（见上表） |

`nav2_params.yaml`、悬挂高度、限速、到位精度等**不需要随着地图改动**。

---

## 相对原始仓库的变更

### 修改的文件

**`nav2_robocon/third_area_single.py`**
- 硬编码 4 步序列（red_sequence / blue_sequence）→ 订阅 `/nav_goal` 接收 `[x, y, yaw]`，单目标导航
- 删除机械臂发布器 `/t0x0103`、悬挂发布器 `/t0x0102_action`
- 删除 `height_400`、`armpose_4`、`mirror()`、`skip_first`
- `wait_for_server()` 从 `__init__` 阻塞调用改为 timer 非阻塞轮询

**`nav2_robocon/ramp_zone_manager.py`**
- `ramp_x_min` 6.15 → 8.9，`ramp_x_max` 8.75 → 10.4（匹配新场地斜坡位置）
- 蓝队 ramp Y [2.0, 3.0] → [3.1, 4.6]，红队 [-3.0, -2.0] → [-4.6, -3.1]
- ramp Y 边界从代码硬编码改为 `declare_parameter`，按队伍自动设默认值
- yaw 覆盖加 `and self.in_ramp` 条件：正常路段 Nav2 控制 yaw，只在斜坡区锁死

**`config/nav2_params.yaml`**
- planner plugin 名 `nav2_smac_planner/SmacPlanner2D` → `nav2_smac_planner::SmacPlanner2D`（ROS 版本兼容）

**`scripts/generate_map.py`**
- 全部场地尺寸常量重写，匹配新地图坐标（边界、高台、斜坡、斜边墙）

**`launch/third_area_single.launch.py`**
- 文档注释更新

### 删除的文件

| 文件 | 原因 |
|------|------|
| `launch/third_restart.launch.py` | 依赖的 `skip_first` 参数已删除 |

### 新增的文件

| 文件 | 说明 |
|------|------|
| `.gitignore` | 忽略 `build/` `install/` `log/` 等构建产物 |
| `README.md` | 本文档 |

### 未改动的文件

`cmd_vel_bridge.py`、`odom_to_tf_node.py`、`goal_relay_node.py`、`nav2_bringup.launch.py`、`setup.py`、`package.xml`、`maps/*`、`resource/*`、`setup.cfg`
