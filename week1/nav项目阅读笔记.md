# 开源项目阅读笔记：Nav2（ROS 2 Navigation）

## 1. 项目简介（README）

- **项目名 / 链接**：Nav2（navigation2）— https://github.com/ros-navigation/navigation2
- **一句话说明**：Nav2 是 ROS 2 官方导航栈，让移动机器人能「自主从当前位置导航到目标点」。
- **具体做什么**：拿到一张地图和一个目标点后，自动完成 定位 → 全局规划路线 → 沿路线走并避障 → 到达目标；中途遇到障碍会绕行，卡住会触发恢复行为。
- **定位**：它是 ROS 1 中 `move_base` 的 ROS 2 继任者，是 ROS 2 移动机器人导航的事实标准。
- **技术栈**：主体 C++，行为树用 XML，参数用 YAML，另有少量 Python 工具。

## 2. 依赖安装

**系统依赖**：
- ROS 2（Humble / Jazzy / Rolling 等，具体支持哪些看 README 顶部的发行版徽标）
- Ubuntu 22.04 / 24.04
- 构建工具：`colcon`、`rosdep`

**主要第三方库**：
- BehaviorTree.CPP（行为树库，Nav2 用它编排导航流程）

**安装方式**：

① 二进制安装（最简单，只使用不改代码）：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
# 把 humble 换成你的 ROS 2 发行版名
```

② 源码安装（要读 / 改代码）：

```bash
mkdir -p ~/nav2_ws/src && cd ~/nav2_ws
git clone https://github.com/ros-navigation/navigation2.git src/navigation2
rosdep install -r --from-paths src --ignore-src -y
colcon build --symlink-install
```

## 3. 运行入口

**启动命令（TurtleBot3 仿真示例）**：

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```

**通用启动**：

```bash
ros2 launch nav2_bringup bringup_launch.py map:=<地图文件路径>
```

**入口在哪个包**：`nav2_bringup`——它集中放所有 launch 文件和默认参数（`nav2_bringup/params/nav2_params.yaml`）。

**入口 → 核心模块的粗结构**：

```text
bringup_launch.py（把下面这些节点都拉起来）
├─ map_server      加载地图
├─ amcl            定位（我在哪）
├─ bt_navigator    行为树总指挥
│   ├─ global_planner   全局规划一条路线
│   ├─ controller       沿路线走 + 局部避障
│   └─ recoveries       卡住时恢复（转圈 / 后退 / 等待）
└─ 输出速度指令 → 机器人底盘
```

## 4. 核心模块（读懂 Nav2 的关键）

Nav2 不是一个大程序，而是一堆「节点」的组合，每个节点干一件事：

| 包 | 作用 |
| --- | --- |
| nav2_bt_navigator | 行为树导航器，总指挥 |
| nav2_planner | 全局规划（NavFn、Smac） |
| nav2_controller | 局部控制（DWB、MPPI、Regulated Pure Pursuit） |
| nav2_costmap_2d | 代价地图（全局 / 局部），避障基础 |
| nav2_amcl | 定位 |
| nav2_map_server | 加载 / 提供地图 |
| nav2_recoveries | 恢复行为（spin / backup / wait） |
| nav2_velocity_smoother | 速度平滑 |
| nav2_lifecycle_manager | 管理各节点生命周期 |
| nav2_bringup | 启动文件 + 配置 |
| nav2_simple_commander | Python API（写脚本用） |

**三个关键概念**：

- **行为树（Behavior Tree）**：把「规划 → 跟随 → 失败 → 恢复 → 重规划」串成流程，XML 在 `nav2_bt_navigator/behavior_trees/`。
- **代价地图（Costmap）**：把静态地图 + 传感器障碍叠加成「哪里能走 / 不能走」的图，规划器在这上面找路。
- **插件机制**：planner、controller 都是可替换插件；新版本推荐 Smac Planner + MPPI / RPP Controller。

## 5. Issue 讨论（常见问题）

按 Issues 页「Most commented」排序，常见几类：

- **机器人卡住 / 撞墙**：多半是 costmap 参数或传感器数据没进来。
- **No transform from odom to base_link**：定位 / 里程计没发布，TF 树断链。
- **怎么自定义规划器 / 控制器**：指到官方文档的 plugin 教程。
- **行为树怎么改**：改 XML 或写自定义 BT 节点。
- **性能 / 延迟**：换 Smac Planner、调频率、用 lifecycle 关掉不用的节点。

**维护者回答风格**：bug 类会问版本号 + 复现步骤；「怎么做」类一般指到 docs.nav2.org 的教程。

## 6. 在 GitHub 上怎么快速定位（下次自己来）

1. 仓库首页往下滚 = README（简介 + 安装 + 快速开始）。
2. 「Code」标签页根目录那一排 `nav2_*` 文件夹，就是各个功能包。
3. 找启动入口 → 进 `nav2_bringup`；找算法 → 进 `nav2_planner` / `nav2_controller`；找行为树 → `nav2_bt_navigator/behavior_trees/`。
4. 文档主要在 docs.nav2.org。
5. 问题 / 讨论 → 「Issues」标签，按 Most commented 排序。
