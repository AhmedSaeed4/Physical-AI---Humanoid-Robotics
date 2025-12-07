---
sidebar_position: 1
title: "Chapter 5: Physics in Gazebo"
---

# Chapter 5: Physics in Gazebo

## Overview

This chapter explores physics simulation using Gazebo, a powerful 3D simulation environment for robotics. We'll learn how to simulate our 12-DOF bipedal robot in realistic physics environments, configure Gazebo plugins for sensors and actuators, and create warehouse worlds with dynamic obstacles.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up Gazebo simulation for humanoid robots
- Configure physics properties and collision detection
- Integrate sensors using Gazebo plugins
- Create custom world environments
- Simulate robot behaviors in physics-based environments

## Introduction to Gazebo

### What is Gazebo?

Gazebo is a 3D simulation environment that provides:
- High-fidelity physics simulation using ODE, Bullet, or DART engines
- Realistic rendering with support for shadows, lighting, and textures
- Sensor simulation for cameras, LIDAR, IMU, and other devices
- Integration with ROS/ROS 2 for robot control and communication
- A library of pre-built models and environments

### Why Use Gazebo for Humanoid Robotics?

For humanoid robotics, Gazebo provides:
- **Physics Validation**: Test robot behaviors before deploying to hardware
- **Safety**: Experiment with complex movements without risk of damage
- **Cost-Effective**: Reduce need for physical prototypes
- **Repeatability**: Consistent testing conditions
- **Scalability**: Test multiple scenarios simultaneously

## Gazebo Integration with URDF

### Adding Gazebo-Specific Tags

To make our URDF model compatible with Gazebo, we need to add Gazebo-specific tags:

```xml
<!-- In your URDF/Xacro file -->
<gazebo reference="base_link">
  <material>Gazebo/Grey</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>

<!-- Gazebo plugin for ros_control -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/bipedal_robot</robotNamespace>
  </plugin>
</gazebo>
```

### Complete Gazebo-Ready URDF

Here's how to modify our bipedal robot URDF for Gazebo:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="bipedal_robot">

  <!-- Include materials -->
  <xacro:include filename="$(find physical_ai_description)/urdf/materials.xacro" />

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Base dimensions -->
  <xacro:property name="base_mass" value="5.0" />
  <xacro:property name="base_length" value="0.3" />
  <xacro:property name="base_width" value="0.2" />
  <xacro:property name="base_height" value="0.15" />

  <!-- Leg dimensions -->
  <xacro:property name="upper_leg_mass" value="1.5" />
  <xacro:property name="upper_leg_length" value="0.4" />
  <xacro:property name="upper_leg_radius" value="0.05" />

  <xacro:property name="lower_leg_mass" value="1.0" />
  <xacro:property name="lower_leg_length" value="0.4" />
  <xacro:property name="lower_leg_radius" value="0.045" />

  <xacro:property name="foot_mass" value="0.5" />
  <xacro:property name="foot_length" value="0.15" />
  <xacro:property name="foot_width" value="0.08" />
  <xacro:property name="foot_height" value="0.05" />

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}" />
      </geometry>
      <material name="light_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="${base_mass}" />
      <inertia
        ixx="${base_mass/12.0 * (base_width*base_width + base_height*base_height)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${base_mass/12.0 * (base_length*base_length + base_height*base_height)}"
        iyz="0.0"
        izz="${base_mass/12.0 * (base_length*base_length + base_width*base_width)}" />
    </inertial>
  </link>

  <!-- Gazebo-specific properties for base link -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <!-- Add similar gazebo tags for other links -->
  <gazebo reference="left_upper_leg">
    <material>Gazebo/DarkGrey</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="right_upper_leg">
    <material>Gazebo/DarkGrey</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <!-- Left Hip Joint -->
  <joint name="left_hip_yaw" type="revolute">
    <parent link="base_link" />
    <child link="left_hip_link" />
    <origin xyz="${base_length/4} ${base_width/2 + 0.06} ${-base_height/2}" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="3.0" />
  </joint>

  <!-- Gazebo plugin for ros_control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/bipedal_robot</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

## Gazebo Plugins for Humanoid Robots

### Differential Drive Plugin

For wheeled robots, but useful as an example:

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/bipedal_robot</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>
    <update_rate>30</update_rate>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>10.0</max_wheel_acceleration>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```

### IMU Sensor Plugin

For humanoid robots, IMU sensors are crucial for balance:

```xml
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <ros>
      <namespace>/bipedal_robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
  </sensor>
</gazebo>
```

### Joint State Publisher Plugin

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/bipedal_robot</namespace>
      <remapping>joint_states:=joint_states</remapping>
    </ros>
    <update_rate>30</update_rate>
    <joint_name>left_hip_yaw</joint_name>
    <joint_name>left_hip_roll</joint_name>
    <joint_name>left_knee</joint_name>
    <joint_name>right_hip_yaw</joint_name>
    <joint_name>right_hip_roll</joint_name>
    <joint_name>right_knee</joint_name>
  </plugin>
</gazebo>
```

## Creating Custom Worlds

### Warehouse World with Dynamic Obstacles

Create a warehouse world with obstacles:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="warehouse_world">
    <!-- Include the outdoor world with sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Warehouse floor -->
    <model name="warehouse_floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Warehouse walls -->
    <model name="wall_1">
      <static>true</static>
      <pose>0 10 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Dynamic obstacles -->
    <model name="dynamic_box_1">
      <pose>5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>1.0 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Shelves -->
    <model name="shelf_1">
      <static>true</static>
      <pose>-8 5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.5 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.5 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.6 0.2 1</ambient>
            <diffuse>0.4 0.8 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

## Launching Gazebo Simulation

### Gazebo Launch File

Create a launch file to start Gazebo with your robot:

```python
# launch/gazebo_simulation.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='warehouse_world.sdf')

    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_physical_ai_description = get_package_share_directory('physical_ai_description')
    pkg_physical_ai_simulation = get_package_share_directory('physical_ai_simulation')

    # Paths
    world_path = os.path.join(
        pkg_physical_ai_simulation,
        'worlds',
        LaunchConfiguration('world').perform({}).split('/')[-1]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_world.sdf',
            description='Choose one of the world files from `/physical_ai_simulation/worlds`'
        ),
        gazebo,
        robot_state_publisher,
        joint_state_publisher
    ])
```

## Physics Configuration

### Physics Engine Parameters

The physics engine parameters significantly affect simulation quality:

```xml
<physics name="ode" type="ode">
  <!-- Time step for physics simulation -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time update rate -->
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- Gravity -->
  <gravity>0 0 -9.8</gravity>

  <!-- ODE-specific parameters -->
  <ode>
    <!-- Solver parameters -->
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>

    <!-- Constraints parameters -->
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Joint Dynamics for Humanoid Robots

For stable humanoid simulation, configure joint dynamics carefully:

```xml
<!-- In your URDF for each joint -->
<joint name="left_knee" type="revolute">
  <parent link="left_upper_leg" />
  <child link="left_lower_leg" />
  <origin xyz="0 0 -0.4" rpy="0 0 0" />
  <axis xyz="1 0 0" />
  <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="3.0" />
  <dynamics damping="0.1" friction="0.01" />
</joint>
```

## Sensor Integration

### Camera Sensor

```xml
<gazebo reference="base_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <visualize>true</visualize>
    <ros>
      <namespace>/bipedal_robot</namespace>
      <remapping>~/image_raw:=camera/image_raw</remapping>
      <remapping>~/camera_info:=camera/camera_info</remapping>
    </ros>
  </sensor>
</gazebo>
```

### LIDAR Sensor

```xml
<gazebo reference="base_link">
  <sensor name="lidar" type="ray">
    <pose>0.1 0 0.1 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/bipedal_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Simulation Best Practices

### 1. Time Step Selection

- **Smaller time steps**: More accurate but slower simulation
- **Larger time steps**: Faster but potentially unstable
- For humanoid robots: Use 0.001s or smaller

### 2. Mass and Inertia

- Use realistic values for stable simulation
- Ensure center of mass is properly positioned
- Balance mass distribution between links

### 3. Joint Limits and Dynamics

- Set appropriate joint limits based on physical capabilities
- Configure damping and friction for realistic movement
- Use effort and velocity limits that match real actuators

### 4. Collision Geometry

- Use simplified collision geometry for performance
- Ensure collision geometry encompasses visual geometry
- Add collision padding to prevent interpenetration

## Troubleshooting Common Issues

### Robot Falls Through Ground

- Check if static flag is set for ground plane
- Verify collision geometry exists for base link
- Check if physics engine is properly configured

### Unstable Joints

- Reduce time step size
- Increase solver iterations
- Add damping to joints
- Verify mass and inertia values

### Performance Issues

- Simplify collision geometry
- Reduce update rates for sensors
- Use fewer but larger time steps
- Optimize world complexity

## Integration with ROS 2 Control

### Setting Up ros2_control

For proper integration with ROS 2 control, add the following to your URDF:

```xml
<!-- ros2_control interface -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_hip_yaw">
    <command_interface name="position">
      <param name="min">-1.57</param>
      <param name="max">1.57</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- Add similar interfaces for all joints -->
</ros2_control>
```

## Summary

In this chapter, we've explored physics simulation in Gazebo for humanoid robots. You've learned how to:
- Modify URDF files for Gazebo compatibility
- Configure physics properties and collision detection
- Add sensors using Gazebo plugins
- Create custom world environments
- Launch simulation with proper ROS 2 integration

In the next chapter, we'll explore NVIDIA Isaac Sim for photorealistic simulation environments that provide even more realistic training data for AI systems.