# quadruped_nav_edge_ws

从仿真仓库中抽取的“真机导航适配最小工作区”，用于与板端已具备的 `FAST-LIVO2`/`rpg_vikit` 组合部署。

## 提取范围

已提取（保留）：
- `fastlivo_nav_bridge`：FAST-LIVO2 到 `/odom`、TF、`/obstacle_points` 的适配
- `floor_mapper`：可选在线 3D 点云到 2D OccupancyGrid 投影
- `quadruped_nav_bringup`：统一启动入口、Nav2 参数、静态地图与离线 PCD 转图脚本

已忽略（不重复带入）：
- `FAST-LIVO2`
- `rpg_vikit`
- Gazebo、仿真插件、控制器等仿真侧内容

## 已做的真机默认化调整

- `navigation_main.launch.py` / `static_map.launch.py` 默认 `use_sim_time=false`
- `nav2_params.yaml` 内全部 `use_sim_time` 改为 `False/false`
- `fastlivo_nav_bridge/launch/bridge.launch.py` 改为可参数化启动（topic/frame 可覆盖）
- `fastlivo_nav_bridge/config/bridge.yaml` 默认点云话题调整为 `/cloud_registered_lidar`

## 使用方式

```bash
cd ~/ws/quadruped_nav_edge_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

基础链路（bridge + floor_mapper）：

```bash
ros2 launch quadruped_nav_bringup navigation_bringup.launch.py
```

完整导航（可选静态地图与 Nav2）：

```bash
ros2 launch quadruped_nav_bringup navigation_main.launch.py \
  enable_static_map:=true \
  map_yaml:=/absolute/path/to/your_map.yaml
```

## 上传 GitHub 建议

仅提交本工作区目录：

```bash
cd ~/ws/quadruped_nav_edge_ws
git init
git add .
git commit -m "Extract hardware navigation bridge workspace"
```
