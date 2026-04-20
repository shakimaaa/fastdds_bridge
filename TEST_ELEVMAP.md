# Elevmap 桥接节点测试说明

## 功能说明

- **节点**: `fastdds_ros2_bridge_elevation_node`（ROS 节点名 `elevmap_forwarder_node`）
- **订阅**: ROS2 话题 `grid_map_msgs/GridMap`，默认 `/elevation_mapping/elevation_map_raw`
- **发布**: Fast DDS 话题，默认 `lr_pro_elevmap_sim`（由参数 `robot` 决定，格式为 `<robot>_elevmap_sim`）
- **配置**: 从 `config/domain_config.yaml` 读取 `global_topic_domain` 作为 DDS 域 ID

## 1. 前置条件

- 已 `source install/setup.bash`
- 存在 `config/domain_config.yaml` 且包含 `global_topic_domain`（例如 `1`）
- 有节点在发布 `grid_map_msgs/GridMap`（例如 elevation_mapping）

## 2. 启动桥接节点

**方式一：launch 文件（推荐）**

```bash
ros2 launch fastdds_ros2_bridge elevmap_bridge.launch.py
# 指定机器人类型（DDS 话题名变为 w1_elevmap_sim）
ros2 launch fastdds_ros2_bridge elevmap_bridge.launch.py robot:=w1
```

**方式二：直接运行可执行文件**

```bash
ros2 run fastdds_ros2_bridge fastdds_ros2_bridge_elevation_node
# 指定机器人类型
ros2 run fastdds_ros2_bridge fastdds_ros2_bridge_elevation_node --robot lr_pro
# 指定订阅话题与调试输出
ros2 run fastdds_ros2_bridge fastdds_ros2_bridge_elevation_node --ros-args -p ros_topic:=/elevation_mapping/elevation_map_raw -p debug_output:=true
```

**注意**: 节点通过 `argv` 解析 `robot`，格式为 `robot <类型>`，例如：  
`ros2 run fastdds_ros2_bridge fastdds_ros2_bridge_elevation_node robot w1`

## 3. 如何检查是否转发成功

### 3.1 看桥接节点终端

- 启动后应看到：  
  `INFO: ElevMap DDS topic: lr_pro_elevmap_sim, ROS topic: /elevation_mapping/elevation_map_raw`
- 当有 DDS 订阅者连上时，会打印：`Publisher matched.`
- 若开启调试，每次转发会打印：`ElevMap forwarded to FastDDS.`  
  开启方式：  
  `ros2 launch fastdds_ros2_bridge elevmap_bridge.launch.py debug_output:=true`

### 3.2 确认 ROS 端（源话题）有数据

若源话题没有数据，桥接不会往 DDS 写任何东西。

```bash
# 看是否有发布者、频率
ros2 topic info /elevation_mapping/elevation_map_raw -v
ros2 topic hz /elevation_mapping/elevation_map_raw

# 收一条看内容（可选）
ros2 topic echo /elevation_mapping/elevation_map_raw --once
```

只有先有 elevation_mapping（或其它节点）在发布该话题，桥接才会转发。

### 3.3 确认 DDS 端有数据（可选）

用 Fast DDS 自带的 subscriber 或你自己的 DDS 订阅程序，在同一 domain（见 `config/domain_config.yaml` 里 `global_topic_domain`）下订阅话题名 `lr_pro_elevmap_sim`（或 `<robot>_elevmap_sim`），类型为 `ElevMapData`，能收到即表示转发成功。

## 4. 与 elevation_mapping 联调

若本仓库中有 elevation_mapping 或 grid_map 相关 launch：

1. 先启动仿真 / 传感器 / elevation_mapping，使 `/elevation_mapping/elevation_map_raw` 有数据。
2. 再启动 elevmap 桥接（如上）。
3. 在 DDS 端订阅 `<robot>_elevmap_sim`，检查收到的栅格与 ROS 端是否一致。

## 5. 常见问题

- **找不到 domain_config.yaml**  
  节点从 `THIS_COM_CONFIG + "domain_config.yaml"` 读配置，该路径在 CMake 中设为 `PROJECT_SOURCE_DIR/../../config/`，即工作空间下的 `config/domain_config.yaml`。请在该路径放置配置文件并保证含 `global_topic_domain`。

- **没有收到 DDS 数据**  
  检查 DDS 域 ID 与 `domain_config.yaml` 中 `global_topic_domain` 一致，且 DDS 订阅的 topic 名为 `<robot>_elevmap_sim`（例如 `lr_pro_elevmap_sim`）。
