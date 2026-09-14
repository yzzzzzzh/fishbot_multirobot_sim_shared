# RViz 结果回放 — 每机一个窗口的三维查看

一键在**桌面 RViz** 里查看跑完的 RACER + Swarm-LIO2 结果(每架无人机开一个窗口),
**无需重跑仿真**——直接读存盘的 `output_*/` 数据。

## 打开

```bash
cd /home/yunze/fishbot_multirobot_sim/src/gazebo_sim/racer_replay
./launch_rviz_replay.sh                 # 默认看 output_600s_v2/
./launch_rviz_replay.sh output_1200s    # 看别的运行:传目录名即可
```

6 个 RViz 窗口出现在桌面(`DISPLAY=:1`），标题 `rviz_bot1`…`rviz_bot6`（可能叠着，拖开）。
每个窗口左侧 Displays 面板 4 层，勾选框可单独开关：

| 层 | 内容 | 样式 |
|---|---|---|
| Environment(真实障碍) | 真实环境障碍物 | 灰色立方体 |
| SLAM Map(SLAM点云) | 该机累积 SLAM 建图 | 按高度着色 |
| GT Path(真实轨迹) | 真实飞行轨迹 | 黑线 |
| SLAM Path(SLAM轨迹) | SLAM 估计轨迹 | 蓝线 |

固定坐标系 `map`（世界系）；点云已从各机 LIO 系转到世界系，与轨迹对齐。

## 操作

左键拖=旋转 · 滚轮=缩放 · 中键拖=平移。左侧勾选框开关各层
（如只看点云、或隐藏环境对比 SLAM 与真值）。

## 关闭

```bash
docker rm -f rviz_replay          # 停发布器 + 全部 6 窗口
```

## 文件清单

| 文件 | 作用 |
|---|---|
| `launch_rviz_replay.sh` | 一键：放通 X 权限 → 生成配置 → 起发布容器 → 开 6 窗口。可传 `output_*` 目录名 |
| `rviz_replay_pub.py` | 数据发布节点：读 `map_botN.npy`+`rec_botN.csv`+`origins.json`，转世界系，latched 发布话题。目录由 `REPLAY_DIR` 环境变量指定 |
| `replay_template.rviz` | RViz 配置模板（话题里 `/BOT/` 为占位符） |
| `cfg_bot1..6.rviz` | 由模板生成的每机配置（脚本自动生成，可删） |

## 原理与前提

- 用 `fishbot_base:latest` 容器（含 rviz2 + ROS2 Humble），`--network host`，把主机
  X 套接字 `/tmp/.X11-unix` 挂进容器、`DISPLAY=:1`，脚本自动 `xhost +local:root` 放通。
- OpenGL 走软件渲染（`LIBGL_ALWAYS_SOFTWARE=1`），无需 GPU 直通。
- 话题 QoS 为 `transient_local`（latched），后开的 RViz 也能收到已发布的数据。
- 为流畅，每机点云上限 16 万点（改 `rviz_replay_pub.py` 的 `MAP_CAP`）。

## 换数据源看新运行

传目录名即可，例如新做完 `output_1200s/`：`./launch_rviz_replay.sh output_1200s`。
