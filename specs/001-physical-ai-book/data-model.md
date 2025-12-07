# Data Model: Physical AI & Humanoid Robotics Book

## Overview
This document outlines the key data models and structures for the Physical AI & Humanoid Robotics Book. Since this is primarily an educational resource with code examples, the "data models" refer to the key ROS message types, data structures, and configuration formats that students will work with.

## Robot Model Structure

### Joint Configuration
- **Entity**: JointAngles
- **Fields**:
  - joint_names: array<string> (names of each joint)
  - positions: array<float> (current joint positions in radians)
  - velocities: array<float> (current joint velocities)
  - efforts: array<float> (current joint efforts)
- **Relationships**: Links to RobotDescription for kinematic chain definition
- **Validation**: Joint values must be within physical limits defined in URDF

### Robot Description
- **Entity**: RobotDescription
- **Fields**:
  - urdf_content: string (XML content of the robot description)
  - links: array<RobotLink> (list of all links in the robot)
  - joints: array<RobotJoint> (list of all joints in the robot)
  - materials: array<Material> (visual materials)
- **Relationships**: Contains multiple RobotLink and RobotJoint entities
- **Validation**: Must conform to URDF/Xacro standards

### Robot Link
- **Entity**: RobotLink
- **Fields**:
  - name: string (unique link identifier)
  - visual_mesh: string (path to visual mesh file)
  - collision_mesh: string (path to collision mesh file)
  - inertial: InertialProperties (mass, center of mass, inertia tensor)
- **Relationships**: Belongs to a RobotDescription

### Robot Joint
- **Entity**: RobotJoint
- **Fields**:
  - name: string (unique joint identifier)
  - type: string (revolute, prismatic, fixed, etc.)
  - parent_link: string (name of parent link)
  - child_link: string (name of child link)
  - limits: JointLimits (min/max position, velocity, effort)
- **Relationships**: Links two RobotLink entities

### Joint Limits
- **Entity**: JointLimits
- **Fields**:
  - lower: float (minimum joint position)
  - upper: float (maximum joint position)
  - velocity: float (maximum joint velocity)
  - effort: float (maximum joint effort)
- **Relationships**: Used by RobotJoint entities

## Sensor Data Structures

### PointCloud
- **Entity**: PointCloud
- **Fields**:
  - header: Header (timestamp and frame ID)
  - height: int (height of the point cloud)
  - width: int (width of the point cloud)
  - fields: array<PointField> (description of point fields)
  - data: array<uint8> (binary point data)
  - is_bigendian: boolean (endianness flag)
  - point_step: int (size of each point in bytes)
  - row_step: int (size of each row in bytes)
  - is_dense: boolean (whether points contain invalid values)
- **Relationships**: Published by depth sensors and VSLAM nodes
- **Validation**: Data must match field descriptions

### CameraInfo
- **Entity**: CameraInfo
- **Fields**:
  - header: Header (timestamp and frame ID)
  - height: int (image height)
  - width: int (image width)
  - distortion_model: string (model of distortion)
  - D: array<float> (distortion coefficients)
  - K: array<float> (3x3 intrinsic camera matrix)
  - R: array<float> (3x3 rectification matrix)
  - P: array<float> (3x4 projection matrix)
- **Relationships**: Associated with image topics from cameras

### ImuData
- **Entity**: ImuData
- **Fields**:
  - header: Header (timestamp and frame ID)
  - orientation: Quaternion (orientation in space)
  - orientation_covariance: array<float> (covariance matrix for orientation)
  - angular_velocity: Vector3 (angular velocity)
  - angular_velocity_covariance: array<float> (covariance matrix for angular velocity)
  - linear_acceleration: Vector3 (linear acceleration)
  - linear_acceleration_covariance: array<float> (covariance matrix for linear acceleration)
- **Relationships**: Published by IMU sensors

## Navigation Data Structures

### PoseWithCovarianceStamped
- **Entity**: PoseWithCovarianceStamped
- **Fields**:
  - header: Header (timestamp and frame ID)
  - pose: PoseWithCovariance (pose with uncertainty)
- **Relationships**: Used for initial pose estimation in navigation

### PoseWithCovariance
- **Entity**: PoseWithCovariance
- **Fields**:
  - pose: Pose (position and orientation)
  - covariance: array<float> (6x6 covariance matrix)
- **Relationships**: Contains detailed pose information with uncertainty

### Pose
- **Entity**: Pose
- **Fields**:
  - position: Point (x, y, z coordinates)
  - orientation: Quaternion (orientation as quaternion)
- **Relationships**: Basic position and orientation data

### Point
- **Entity**: Point
- **Fields**:
  - x: float (x coordinate)
  - y: float (y coordinate)
  - z: float (z coordinate)

### Quaternion
- **Entity**: Quaternion
- **Fields**:
  - x: float (x component)
  - y: float (y component)
  - z: float (z component)
  - w: float (w component)

## AI/LLM Integration Structures

### VoiceCommand
- **Entity**: VoiceCommand
- **Fields**:
  - text: string (recognized text from speech)
  - confidence: float (confidence score of recognition)
  - timestamp: string (ISO 8601 timestamp)
  - source_device: string (device that captured the audio)
- **Relationships**: Converted to RobotAction via LLM processing
- **Validation**: Text must not be empty

### RobotAction
- **Entity**: RobotAction
- **Fields**:
  - action_type: string (navigate, detect, grasp, etc.)
  - parameters: object (action-specific parameters)
  - priority: int (execution priority level)
  - timeout: float (maximum execution time in seconds)
- **Relationships**: Generated from VoiceCommand by LLM
- **Validation**: Must have valid action_type and required parameters

### NavigationGoal
- **Entity**: NavigationGoal
- **Fields**:
  - x: float (target x coordinate)
  - y: float (target y coordinate)
  - theta: float (target orientation in radians)
  - frame_id: string (coordinate frame for the goal)
  - navigation_mode: string (planner to use)
- **Relationships**: Specialized RobotAction for navigation
- **Validation**: Coordinates must be within navigation map bounds

## Configuration Structures

### RobotConfiguration
- **Entity**: RobotConfiguration
- **Fields**:
  - robot_name: string (name of the robot platform)
  - hardware_config: HardwareConfig (physical properties)
  - software_config: SoftwareConfig (ROS node settings)
  - simulation_config: SimulationConfig (simulation-specific settings)
- **Relationships**: Top-level configuration container

### HardwareConfig
- **Entity**: HardwareConfig
- **Fields**:
  - joint_limits: map<string, JointLimits> (limits for each joint)
  - max_velocity: float (maximum joint velocity)
  - max_acceleration: float (maximum joint acceleration)
  - sensors: array<SensorConfig> (sensor configurations)
- **Relationships**: Part of RobotConfiguration

### SensorConfig
- **Entity**: SensorConfig
- **Fields**:
  - sensor_type: string (camera, lidar, imu, etc.)
  - topic_name: string (ROS topic for sensor data)
  - update_rate: float (sensor update rate in Hz)
  - frame_id: string (TF frame for sensor)
- **Relationships**: Part of HardwareConfig

### SoftwareConfig
- **Entity**: SoftwareConfig
- **Fields**:
  - node_frequency: float (main loop frequency in Hz)
  - error_handling: ErrorConfig (error handling settings)
  - api_config: ApiConfig (API connection settings)
- **Relationships**: Part of RobotConfiguration

### ErrorConfig
- **Entity**: ErrorConfig
- **Fields**:
  - timeout_threshold: float (timeout value in seconds)
  - retry_attempts: int (number of retry attempts)
  - fallback_behavior: string (behavior when retries exhausted)
- **Relationships**: Part of SoftwareConfig

### ApiConfig
- **Entity**: ApiConfig
- **Fields**:
  - api_key: string (API key, stored securely)
  - endpoint_url: string (API endpoint)
  - rate_limit: int (requests per minute)
- **Relationships**: Part of SoftwareConfig

### SimulationConfig
- **Entity**: SimulationConfig
- **Fields**:
  - physics_engine: string (ode, bullet, etc.)
  - real_time_factor: float (simulation speed multiplier)
  - world_file: string (path to world description)
- **Relationships**: Part of RobotConfiguration

## State Management

### RobotState
- **Entity**: RobotState
- **Fields**:
  - joint_states: JointAngles (current joint positions)
  - pose: Pose (current pose in map frame)
  - status: string (operational, error, charging, etc.)
  - battery_level: float (battery level percentage)
  - current_action: RobotAction (currently executing action)
- **Relationships**: Reflects the current state of the robot
- **State Transitions**:
  - idle → executing (when action starts)
  - executing → idle (when action completes)
  - executing → error (when action fails)
  - error → idle (when error is resolved)

### SystemState
- **Entity**: SystemState
- **Fields**:
  - current_phase: string (listening, planning, moving, verifying)
  - last_command: VoiceCommand (most recent voice command)
  - goal_status: string (active, succeeded, failed, canceled)
  - navigation_status: NavigationStatus (navigation-specific status)
- **Relationships**: Tracks the overall system state
- **State Transitions**: Follows the state machine described in the requirements