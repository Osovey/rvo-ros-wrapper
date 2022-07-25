# RVO2 ROS WRAPPER

## RVO2 LIBRARY

https://gamma.cs.unc.edu/RVO2/

## Wrapper

https://github.com/bcharrow/rvo

## 介绍 

基于**OCRA**算法，分布式的运行在每个机器人上。

### 包名

​	rvo_move

### 节点组成

- **move_client**(src/move_client.cpp)
  - **订阅话题**
    - **名称**：\goal
    - **类型**：geometry_msgs::PoseStamped
    - **介绍**：接收目标点
  - **发布话题**：-
  - **Action**：
    - Move.action(action/Move.action)
    - **介绍**：action客户端，在接收到\goal话题后，通过action向move_server发送请求，move_server会计算agent的路线，并在行进过程中进行碰撞避免。

- **move_server**(src/move_server.cpp)

  - **订阅话题**

    - **名称**：\odom

    - **类型**：nav_msgs::Odometry

    - **介绍**：每个机器人的move_sever节点都会订阅系统中所有机器人的\odom话题来获取各机器人的位置。

      ------

    - **名称**：\map

    - **类型**：-

    - **介绍**：栅格地图

  - **发布话题**：

    - **名称**：\cmd_vel
    - **类型**：geometry_msgs::Twist
    - **介绍**： 向move_server节点所属的机器人发送速度话题来控制机器人的行进速度和方向。

  - **Action**：

    - Move.action(action/Move.action)
    - **介绍**：action服务端，接受action客户端的请求，action客户端会发送一个目标位置，move_server节点会根据A*算法计算路线，并实时的进行碰撞避免。

## 配置

部分参数的解释：

- radius：机器人的半径
- maxSpeed：机器人可以行进的最大速度
- goal_tolerance：机器人距离终点的最大容忍距离（停在终点附近即可看作是导航完成）。

```yaml
#config/scarab_move.yaml
neighborDist: 10.0
maxNeighbors: 10
timeHorizon: 10.0
timeHorizonObst: 3.0
radius: 0.08
maxSpeed: 1
timestep: 0.2 # 1 / controller rate
axel_width: 0.255
los_margin: 0.1

waypoint_spacing: 0.1
path_margin: 0.6
goal_tolerance: 0.1

# http://gamma.cs.unc.edu/RVO2/documentation/2.0/params.html
# timeStep
# 	float (time)
# 	The time step of the simulation. Must be positive.
# maxNeighbors
# 	size_t
# 	The maximal number of other agents the agent takes into account in the
# 	navigation. The larger this number, the longer the running time of the
# 	simulation. If the number is too low, the simulation will not be safe.
# maxSpeed
# 	float (distance/time)
# 	The maximum speed of the agent. Must be non-negative.
# neighborDist
# 	float (distance)
# 	The maximal distance (center point to center point) to other agents the
# 	agent takes into account in the navigation. The larger this number, the
# 	longer the running time of the simulation. If the number is too low,
# 	the simulation will not be safe. Must be non-negative.
# radius
# 	float (distance)
# 	The radius of the agent. Must be non-negative.
# timeHorizon
# 	float (time)
# 	The minimal amount of time for which the agent's velocities that are
# 	computed by the simulation are safe with respect to other agents. The
# 	larger this number, the sooner this agent will respond to the presence
# 	of other agents, but the less freedom the agent has in choosing its
# 	velocities. Must be positive.
# timeHorizonObst
# 	float (time)
# 	The minimal amount of time for which the agent's velocities that are
# 	computed by the simulation are safe with respect to obstacles. The
# 	larger this number, the sooner this agent will respond to the presence
# 	of obstacles, but the less freedom the agent has in choosing its
# 	velocities. Must be positive.

```

## Example

用launch文件，启动16个机器人。

每个机器人launch文件（递归启动多个机器人）(recursive_robots.launch)

```xml
<launch>
    <arg name="model"   default="$(find rvo_move)/urdf/robot.xacro" />
    <arg name="total_num"   default="16"  />
    <arg name="robot_id"    default="0"   />

    <group ns="robot_$(arg robot_id)">
        <param  name="robot_description"    command="$(find xacro)/xacro $(arg model) ns:=robot_$(arg robot_id)" />
        <node pkg="joint_state_publisher"   type="joint_state_publisher"    name="joint_state_publisher" />
        <node pkg="robot_state_publisher"   type="robot_state_publisher"    name="robot_state_publisher" />
        <node pkg="rvo_move"    type="move_server"  name="move_server"  output="screen">
        <rosparam file="$(find rvo_move)/config/scarab_move.yaml"   command="load" />
        <param name="nbots" value="$(arg total_num)" />
        <param name="id"    value="$(arg robot_id)" />
        </node>
        <node pkg="rvo_move"    type="move_client"  name="move_client"  output="screen" />
    </group>

    <node pkg="tf"  type="static_transform_publisher"   name="map_odom_broadcaster_$(arg robot_id)" args="0 0 0 0 0 0 map robot_$(arg robot_id)/odom 100" />

    <include file="$(find rvo_move)/launch/recursive_robots.launch" if="$(eval (arg('total_num') > arg('robot_id') + 1))">
        <arg name="total_num"   value="$(arg total_num)" />
        <arg name="robot_id"    value="$(eval (arg('robot_id') + 1))" />
    </include>

</launch>
```

总的launch文件(move_launch)

```xml
<launch>
  <param name="use_sim_time"  value="true"  />
  <param name="robot_num" value="16" />

  <node pkg="stage_ros" type="stageros" name="stageros" args="$(find rvo_move)/maps/stage.world"  />

  <node pkg="map_server"  type="map_server" name="map_server" args="$(find rvo_move)/maps/map.yaml"  />
  
  <include file="$(find rvo_move)/launch/recursive_robots.launch"  >
    <arg name="total_num" value="16"    />
    <arg name="robot_id"  value="0"  />
  </include>

  <node pkg="rviz"  type="rviz" name="rviz"  args="-d $(find rvo_move)/config/rviz.rviz"  />

</launch>
```

