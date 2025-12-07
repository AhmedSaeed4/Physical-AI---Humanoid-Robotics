---
sidebar_position: 1
title: "Chapter 4: The Body (URDF & Kinematics)"
---

# Chapter 4: The Body (URDF & Kinematics)

## Overview

This chapter explores the representation of humanoid robot bodies using the Unified Robot Description Format (URDF) and Xacro. We'll examine how to model the 12-DOF bipedal robot, define its kinematic structure, and set up the Transform (TF) tree for spatial relationships.

## Learning Objectives

By the end of this chapter, you will be able to:
- Create URDF files to describe robot geometry and kinematics
- Use Xacro macros to simplify complex robot descriptions
- Set up the TF tree for a humanoid robot
- Validate robot models using RViz
- Understand the relationship between URDF and robot control

## URDF Fundamentals

### What is URDF?

Unified Robot Description Format (URDF) is an XML-based format for representing robots. It describes the physical and kinematic properties of a robot, including:
- Links (rigid parts)
- Joints (connections between links)
- Inertial properties
- Visual and collision geometry
- Materials

### Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links define rigid bodies -->
  <link name="link_name">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="color">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="joint_name" type="revolute">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Xacro for Complex Models

Xacro (XML Macros) extends URDF with macros, properties, and mathematical expressions to simplify complex robot descriptions.

### Basic Xacro Features

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="bipedal_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="robot_height" value="0.8" />

  <!-- Define macros -->
  <xacro:macro name="simple_link" params="name mass length radius">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}" />
        <inertia
          ixx="${mass/12.0 * (3*radius*radius + length*length)}"
          ixy="0.0"
          ixz="0.0"
          iyy="${mass/12.0 * (3*radius*radius + length*length)}"
          iyz="0.0"
          izz="${mass/2.0 * radius * radius}" />
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:simple_link name="upper_leg" mass="1.5" length="0.4" radius="0.05" />

</robot>
```

## 12-DOF Bipedal Robot Model

Our 12-DOF bipedal robot has 6 degrees of freedom per leg (3 for hip, 1 for knee, 2 for ankle), providing stable locomotion capabilities.

### Joint Configuration

The robot has the following joint configuration:

- **Left Leg (6 DOF)**:
  - Hip Yaw (rotation around Z-axis)
  - Hip Roll (rotation around X-axis)
  - Hip Pitch (rotation around Y-axis)
  - Knee (flexion/extension)
  - Ankle Pitch (dorsiflexion/plantarflexion)
  - Ankle Roll (inversion/eversion)

- **Right Leg (6 DOF)**:
  - Same as left leg

### Complete URDF/Xacro Model

Here's the complete model for our 12-DOF bipedal robot:

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

  <!-- Hip dimensions -->
  <xacro:property name="hip_mass" value="0.8" />
  <xacro:property name="hip_radius" value="0.06" />
  <xacro:property name="hip_length" value="0.08" />

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

  <!-- Left Hip -->
  <joint name="left_hip_yaw" type="revolute">
    <parent link="base_link" />
    <child link="left_hip_link" />
    <origin xyz="${base_length/4} ${base_width/2 + hip_radius} ${-base_height/2}" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="3.0" />
  </joint>

  <link name="left_hip_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${hip_radius}" length="${hip_length}" />
      </geometry>
      <material name="dark_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${hip_radius}" length="${hip_length}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="${hip_mass}" />
      <inertia
        ixx="${hip_mass/12.0 * (3*hip_radius*hip_radius + hip_length*hip_length)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${hip_mass/12.0 * (3*hip_radius*hip_radius + hip_length*hip_length)}"
        iyz="0.0"
        izz="${hip_mass/2.0 * hip_radius * hip_radius}" />
    </inertial>
  </link>

  <joint name="left_hip_roll" type="revolute">
    <parent link="left_hip_link" />
    <child link="left_upper_leg" />
    <origin xyz="0 0 ${-hip_length/2}" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="100.0" velocity="3.0" />
  </joint>

  <!-- Left Upper Leg -->
  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${upper_leg_radius}" length="${upper_leg_length}" />
      </geometry>
      <material name="dark_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${upper_leg_radius}" length="${upper_leg_length}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0" />
      <mass value="${upper_leg_mass}" />
      <inertia
        ixx="${upper_leg_mass/12.0 * (3*upper_leg_radius*upper_leg_radius + upper_leg_length*upper_leg_length)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${upper_leg_mass/12.0 * (3*upper_leg_radius*upper_leg_radius + upper_leg_length*upper_leg_length)}"
        iyz="0.0"
        izz="${upper_leg_mass/2.0 * upper_leg_radius * upper_leg_radius}" />
    </inertial>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_upper_leg" />
    <child link="left_lower_leg" />
    <origin xyz="0 0 ${-upper_leg_length}" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="3.0" />
  </joint>

  <!-- Left Lower Leg -->
  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${lower_leg_radius}" length="${lower_leg_length}" />
      </geometry>
      <material name="dark_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${lower_leg_radius}" length="${lower_leg_length}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0" />
      <mass value="${lower_leg_mass}" />
      <inertia
        ixx="${lower_leg_mass/12.0 * (3*lower_leg_radius*lower_leg_radius + lower_leg_length*lower_leg_length)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${lower_leg_mass/12.0 * (3*lower_leg_radius*lower_leg_radius + lower_leg_length*lower_leg_length)}"
        iyz="0.0"
        izz="${lower_leg_mass/2.0 * lower_leg_radius * lower_leg_radius}" />
    </inertial>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_lower_leg" />
    <child link="left_foot" />
    <origin xyz="0 0 ${-lower_leg_length}" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="50.0" velocity="2.0" />
  </joint>

  <!-- Left Foot -->
  <link name="left_foot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${foot_length} ${foot_width} ${foot_height}" />
      </geometry>
      <material name="dark_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${foot_length} ${foot_width} ${foot_height}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="${foot_mass}" />
      <inertia
        ixx="${foot_mass/12.0 * (foot_width*foot_width + foot_height*foot_height)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${foot_mass/12.0 * (foot_length*foot_length + foot_height*foot_height)}"
        iyz="0.0"
        izz="${foot_mass/12.0 * (foot_length*foot_length + foot_width*foot_width)}" />
    </inertial>
  </link>

  <!-- Right Hip (symmetric to left) -->
  <joint name="right_hip_yaw" type="revolute">
    <parent link="base_link" />
    <child link="right_hip_link" />
    <origin xyz="${base_length/4} ${-base_width/2 - hip_radius} ${-base_height/2}" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="3.0" />
  </joint>

  <link name="right_hip_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${hip_radius}" length="${hip_length}" />
      </geometry>
      <material name="dark_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${hip_radius}" length="${hip_length}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="${hip_mass}" />
      <inertia
        ixx="${hip_mass/12.0 * (3*hip_radius*hip_radius + hip_length*hip_length)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${hip_mass/12.0 * (3*hip_radius*hip_radius + hip_length*hip_length)}"
        iyz="0.0"
        izz="${hip_mass/2.0 * hip_radius * hip_radius}" />
    </inertial>
  </link>

  <joint name="right_hip_roll" type="revolute">
    <parent link="right_hip_link" />
    <child link="right_upper_leg" />
    <origin xyz="0 0 ${-hip_length/2}" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="100.0" velocity="3.0" />
  </joint>

  <!-- Right Upper Leg -->
  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${upper_leg_radius}" length="${upper_leg_length}" />
      </geometry>
      <material name="dark_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${upper_leg_radius}" length="${upper_leg_length}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0" />
      <mass value="${upper_leg_mass}" />
      <inertia
        ixx="${upper_leg_mass/12.0 * (3*upper_leg_radius*upper_leg_radius + upper_leg_length*upper_leg_length)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${upper_leg_mass/12.0 * (3*upper_leg_radius*upper_leg_radius + upper_leg_length*upper_leg_length)}"
        iyz="0.0"
        izz="${upper_leg_mass/2.0 * upper_leg_radius * upper_leg_radius}" />
    </inertial>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_upper_leg" />
    <child link="right_lower_leg" />
    <origin xyz="0 0 ${-upper_leg_length}" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="3.0" />
  </joint>

  <!-- Right Lower Leg -->
  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${lower_leg_radius}" length="${lower_leg_length}" />
      </geometry>
      <material name="dark_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${lower_leg_radius}" length="${lower_leg_length}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0" />
      <mass value="${lower_leg_mass}" />
      <inertia
        ixx="${lower_leg_mass/12.0 * (3*lower_leg_radius*lower_leg_radius + lower_leg_length*lower_leg_length)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${lower_leg_mass/12.0 * (3*lower_leg_radius*lower_leg_radius + lower_leg_length*lower_leg_length)}"
        iyz="0.0"
        izz="${lower_leg_mass/2.0 * lower_leg_radius * lower_leg_radius}" />
    </inertial>
  </link>

  <joint name="right_ankle" type="revolute">
    <parent link="right_lower_leg" />
    <child link="right_foot" />
    <origin xyz="0 0 ${-lower_leg_length}" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="50.0" velocity="2.0" />
  </joint>

  <!-- Right Foot -->
  <link name="right_foot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${foot_length} ${foot_width} ${foot_height}" />
      </geometry>
      <material name="dark_grey" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${foot_length} ${foot_width} ${foot_height}" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="${foot_mass}" />
      <inertia
        ixx="${foot_mass/12.0 * (foot_width*foot_width + foot_height*foot_height)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${foot_mass/12.0 * (foot_length*foot_length + foot_height*foot_height)}"
        iyz="0.0"
        izz="${foot_mass/12.0 * (foot_length*foot_length + foot_width*foot_width)}" />
    </inertial>
  </link>

</robot>
```

## Kinematic Chains and Forward Kinematics

### Understanding Kinematic Chains

A kinematic chain is an assembly of rigid bodies (links) connected by joints. In our bipedal robot:
- Each leg forms a kinematic chain from hip to foot
- The base link connects to both legs
- Joint angles determine the position and orientation of each link

### Forward Kinematics

Forward kinematics calculates the position and orientation of the end effector (foot) given the joint angles. For our robot, this enables:

1. **Gait Planning**: Calculating foot positions for walking
2. **Balance Control**: Ensuring the center of mass remains stable
3. **Trajectory Generation**: Planning smooth movements

## TF (Transform) Tree

The TF tree represents the spatial relationships between all coordinate frames in the robot. For our 12-DOF bipedal robot:

```
                    map
                     |
                   odom
                     |
                 base_link
                /         \
    left_hip_yaw      right_hip_yaw
         |                   |
    left_hip_roll      right_hip_roll
         |                   |
    left_upper_leg    right_upper_leg
         |                   |
    left_knee         right_knee
         |                   |
    left_lower_leg    right_lower_leg
         |                   |
    left_ankle        right_ankle
         |                   |
    left_foot         right_foot
```

### TF in ROS 2

TF (Transform) in ROS 2 provides:
- Real-time coordinate frame transformations
- Automatic interpolation between timestamps
- Visualization tools for debugging
- Integration with navigation and perception systems

## Validation and Visualization

### Using RViz for Model Validation

To visualize and validate your robot model:

1. **Launch the robot state publisher**:
   ```bash
   ros2 launch physical_ai_description view_robot.launch.py
   ```

2. **Check for errors** in the terminal output
3. **Verify joint movements** using the joint state publisher GUI
4. **Confirm collision geometry** alignment with visual geometry

### Common Validation Checks

- **Joint Limits**: Ensure all joints have appropriate limits
- **Inertial Properties**: Verify realistic mass and inertia values
- **Collision Detection**: Test that collision geometry is properly defined
- **Kinematic Chain**: Confirm all links are properly connected

## Best Practices for URDF/Xacro

1. **Use consistent naming**: Follow ROS conventions (e.g., `left_hip_yaw`)
2. **Define materials separately**: Use Xacro includes for material definitions
3. **Parameterize dimensions**: Use properties for easy scaling
4. **Include inertial properties**: Essential for physics simulation
5. **Test with Gazebo**: Validate physics behavior in simulation
6. **Document joint ranges**: Ensure they match physical robot capabilities

## Integration with Control Systems

The URDF model integrates with control systems through:

1. **Joint State Publisher**: Provides feedback on joint positions
2. **Robot State Publisher**: Publishes TF transforms
3. **Controller Manager**: Interfaces with hardware or simulation
4. **Kinematics Solvers**: Calculate inverse kinematics for movement

## Summary

In this chapter, we've explored the representation of humanoid robot bodies using URDF and Xacro. You've learned how to:
- Create complex robot models with proper kinematic chains
- Use Xacro macros to simplify model definitions
- Set up the TF tree for spatial relationships
- Validate models using visualization tools

In the next chapter, we'll explore physics simulation in Gazebo, where our robot model will come to life with realistic physics interactions.