# 无人机运动规划

## 0. 三分钟快速安装

*已在 Ubuntu 20.04 LTS + ROS Noetic 上测试通过。*

1. 安装 [ROS](http://wiki.ros.org/ROS/Installation)（推荐安装 *Desktop-Full Install*）。

2. 克隆仓库。

    ```bash
    git clone https://github.com/ClaudyFlow/uav_motion_planning.git
    ```

3. 安装依赖。

    ```bash
    # eigen
    sudo apt install libeigen3-dev

    # osqp 和 osqp-eigen
    cd uav_motion_planning
    git submodule update --init --recursive

    ## osqp
    cd 3rd/osqp
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

    ## osqp-eigen
    cd 3rd/osqp-eigen
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

4. 编译代码。

    ```bash
    catkin_make -DCMAKE_CXX_STANDARD=14
    ```

## 1. 基于搜索的方法

### 1.1. A*

- 快速启动：

  ```bash
  # 在一个终端中
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # 在另一个终端中
  source devel/setup.bash
  roslaunch test test_astar_searching.launch
  ```

- 参数：

  ```xml
  <!-- astar 参数 -->
  <param name="astar/resolution" value="0.1"/>
  <param name="astar/lambda_heu" value="1.5"/>
  <param name="astar/allocated_node_num" value="1000000"/>
  ```

  - **astar/resolution：** A* 的搜索分辨率，控制搜索精度。

  - **astar/lambda_heu：** $f = g(n) + \lambda * h(n)$

  - **astar/allocated_node_num：** 预分配的搜索节点数量上限，避免节点过多。

- 方法说明：

  - **tie_breaker：** 提升搜索速度的打破平局策略。

  - **weighted A\***：

    ![图示](https://github.com/ClaudyFlow/motion-planning/blob/main/pic/equation1.png?raw=true)

- 仿真：

  ![动画](https://github.com/ClaudyFlow/motion-planning/blob/main/pic/astar.gif?raw=true)

### 1.2. 动力学 A*（Kinodynamic A\*）

- 快速启动：

  ```bash
  # 在一个终端中
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # 在另一个终端中
  source devel/setup.bash
  roslaunch test test_kino_astar_searching.launch
  ```

- 参数：**注意：部分参数需要修改。**

  - **kino_astar/collision_check_type：** 1：**kino_astar 规划**，2：**kino_se(3) 规划**

  - **（地图类型修改）** simulator.xml 中：**map/fix_map_type：** 0：生成随机地图，1：生成固定墙体地图

  ```xml
   <!-- kino_astar 参数 -->
  <param name="kino_astar/rou_time" value="20.0"/>
  <param name="kino_astar/lambda_heu" value="3.0"/>
  <param name="kino_astar/allocated_node_num" value="100000"/>
  <param name="kino_astar/goal_tolerance" value="2.0"/>
  <param name="kino_astar/time_step_size" value="0.075"/>
  <param name="kino_astar/max_velocity" value="7.0"/>
  <param name="kino_astar/max_accelration" value="10.0"/>
  <param name="kino_astar/acc_resolution" value="4.0"/>
  <param name="kino_astar/sample_tau" value="0.3"/>
  <!-- 碰撞检测类型 1: kino_astar, 2: kino_se3 -->
  <param name="kino_astar/collision_check_type" value="2"/>
  <!-- 机器人椭球参数 -->
  <param name="kino_se3/robot_r" value="0.4"/>
  <param name="kino_se3/robot_h" value="0.1"/>
  ```

- 方法说明：

  - **参考论文：** B. Zhou, F. Gao, L. Wang, C. Liu and S. Shen, ["Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight"](https://arxiv.org/pdf/1907.01531)

- 仿真：

  ![动画](https://github.com/ClaudyFlow/motion-planning/blob/main/pic/kino_astar.gif?raw=true)

### 1.3. SE(3) 规划

- 快速启动：

  ```bash
  # 在一个终端中
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # 在另一个终端中
  source devel/setup.bash
  roslaunch test test_kino_astar_searching.launch
  ```

- 参数：**注意：部分参数需要修改。**

  - **kino_astar/collision_check_type：** 1：**kino_astar 规划**，2：**kino_se(3) 规划**

  - **（地图类型修改）** simulator.xml 中：**map/fix_map_type：** 0：生成随机地图，1：生成固定墙体地图

  ```xml
   <!-- kino_astar 参数 -->
  <param name="kino_astar/rou_time" value="20.0"/>
  <param name="kino_astar/lambda_heu" value="3.0"/>
  <param name="kino_astar/allocated_node_num" value="100000"/>
  <param name="kino_astar/goal_tolerance" value="2.0"/>
  <param name="kino_astar/time_step_size" value="0.075"/>
  <param name="kino_astar/max_velocity" value="7.0"/>
  <param name="kino_astar/max_accelration" value="10.0"/>
  <param name="kino_astar/acc_resolution" value="4.0"/>
  <param name="kino_astar/sample_tau" value="0.3"/>
  <!-- 碰撞检测类型 1: kino_astar, 2: kino_se3 -->
  <param name="kino_astar/collision_check_type" value="2"/>
  <!-- 机器人椭球参数 -->
  <param name="kino_se3/robot_r" value="0.4"/>
  <param name="kino_se3/robot_h" value="0.1"/>
  ```

- 方法说明：

  - **参考论文：** S. Liu, K. Mohta, N. Atanasov, and V. Kumar, ["Search-based motion planning for aggressive flight in SE(3)"](https://arxiv.org/pdf/1710.02748)

- 仿真：

  ![动画](https://github.com/ClaudyFlow/motion-planning/blob/main/pic/kino_se3.gif?raw=true)

## 2. 基于采样的方法

### 2.1. RRT

- 快速启动：

  ```bash
  # 在一个终端中
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # 在另一个终端中
  source devel/setup.bash
  roslaunch test test_rrt_searching.launch
  ```

- 参数：

  ```xml
  <!-- rrt 参数 -->
  <param name="rrt/max_tree_node_num" value="100000"/>
  <param name="rrt/step_length" value="0.5"/>
  <param name="rrt/max_allowed_time" value="5"/>
  <param name="rrt/search_radius" value="1.0"/>
  ```

​ - **rrt/max_tree_node_num：** RRT 树的最大节点数，避免节点过多。

​ - **rrt/step_length：** RRT 的步长 rrt.step(x1, x2, length)，控制每一步推进的长度。

​ - **rrt/max_allowed_time：** RRT 的最大允许时间，避免搜索时间过长。

​ - **rrt/search_radius：** RRT 的目标容差，控制搜索终点与真实终点的误差。

- 方法说明：

  - **kdtree-acceleration：** 使用 KD 树加速最近邻节点查询。

- 仿真：

  ![动画](https://github.com/ClaudyFlow/motion-planning/blob/main/pic/rrt.gif?raw=true)

### 2.2. RRT\*：

- 快速启动：

  ```bash
  # 在一个终端中
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # 在另一个终端中
  source devel/setup.bash
  roslaunch test test_rrt_star_searching.launch
  ```

- 参数：

  ```xml
  <param name="rrt_star/max_tree_node_num" value="100000"/>
  <param name="rrt_star/step_length" value="0.5"/>
  <param name="rrt_star/search_radius" value="1.0"/>
  ```

- 方法说明：

  - **kdtree-acceleration：** 使用 KD 树加速最近邻节点查询。

- 仿真：

  ![动画](https://github.com/ClaudyFlow/motion-planning/blob/main/pic/rrt_star.gif?raw=true)

## 3. 轨迹优化

### 3.1. RRT\* + Minimum Snap：

- 快速启动：

  ```bash
  # 在一个终端中
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # 在另一个终端中
  source devel/setup.bash
  roslaunch test test_minimum_jerk.launch
  ```

- 方法说明：

  - **前端（路径搜索）：** 使用 RRT* 或其它基于搜索的方法，本项目基于 RRT\*。

  - **后端（轨迹优化）：** 基于前端 RRT* 得到的离散路径点，构造一个二次规划（QP），使用 **OSQP 求解器** 进行求解。

  - **参考论文：** D. Mellinger and V. Kumar, ["Minimum snap trajectory generation and control for quadrotors"](https://web.archive.org/web/20120713162030id_/http://www.seas.upenn.edu/~dmel/mellingerICRA11.pdf)

- 仿真：

  ![动画](https://github.com/ClaudyFlow/motion-planning/blob/main/pic/minimum_jerk.gif?raw=true)

  - **红色线段：** RRT* 路径
  - **红色球体：** RRT* 路径点
  - **紫色线段：** 最小 Snap 优化后的轨迹

## 4. Docker 使用

*也可以使用 Docker 来搭建开发环境，而无需在主机上安装依赖。请先安装 [foxglove](https://foxglove.dev/download)。*

1. 克隆仓库。

    ```bash
    git clone https://github.com/ClaudyFlow/uav_motion_planning.git
    ```

2. 进入工作空间目录并构建代码：
   * 在 vscode 中按下 `command + shift + p`，输入并选择：`Dev Containers: Reopen in Container`，以打开一个开发容器。
   * 在 vscode 中打开一个新的终端（`control + shift + ~`），该终端位于容器内部，后续命令均在该终端中执行。
3. 构建项目

    ```bash
    catkin_make
    ```

4. 启动 demo
   * 启动 foxglove_bridge

    ```bash
    roslaunch foxglove_bridge foxglove_bridge.launch port:=9090
    ```

   * 启动仿真

    ```bash
    # 在一个终端中
    source devel/setup.zsh
    roslaunch plan_manage single_run_in_foxglove.launch

    # 在另一个终端中
    source devel/setup.zsh
    roslaunch test test_rrt_star_searching.launch
    ```

5. 启动 foxglove 进行可视化
   输入 WebSocket URL：`ws://localhost:9090`，可以选择 ROS 话题进行可视化。
