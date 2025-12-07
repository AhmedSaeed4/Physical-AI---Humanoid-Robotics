# ROS Interface Contracts for Physical AI & Humanoid Robotics

## Overview
This document defines the ROS 2 interface contracts (messages, services, and actions) that will be used in the Physical AI & Humanoid Robotics book. These interfaces enable communication between different robot components and support the integration of AI systems.

## Message Definitions (.msg)

### JointAngles.msg
```
# Custom message for humanoid joint control
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
---
# Joint angles for humanoid robot control
# Used for sending desired joint positions to controllers
# Also used for publishing current joint states
```

### VoiceCommand.msg
```
# Message for voice commands processed by the AI system
std_msgs/Header header
string text
float32 confidence
string source_device
---
# Represents a voice command recognized by the system
# Used to pass voice input from the voice recognition node to the AI commander
```

### RobotAction.msg
```
# Message for high-level robot actions
std_msgs/Header header
string action_type
string parameters_json
int32 priority
float32 timeout
---
# Represents a high-level action for the robot to execute
# action_type can be: 'navigate', 'detect', 'grasp', 'speak', etc.
# parameters_json contains action-specific parameters as JSON string
```

### VisionQuery.msg
```
# Message for vision-language queries
std_msgs/Header header
sensor_msgs/Image image
string query
bool include_depth
---
# Used to send image and query to VLA system
# The query asks the VLA model to analyze the image
```

### VisionResult.msg
```
# Message for vision-language results
std_msgs/Header header
bool success
string result_text
float32 confidence
geometry_msgs/Pose[] detected_objects
---
# Response from VLA system with analysis results
# Includes detected objects with their poses if applicable
```

## Service Definitions (.srv)

### ExecuteRobotAction.srv
```
# Request
RobotAction action
---
# Response
bool success
string message
string action_id
```
```
# Service to execute a high-level robot action
# Used by the LLM commander to execute actions through the action manager
```

### GetRobotPose.srv
```
# Request
string frame_id
---
# Response
bool success
geometry_msgs/Pose pose
string message
```
```
# Service to get the current robot pose in a specific frame
# Used for localization and navigation tasks
```

### SetJointPositions.srv
```
# Request
JointAngles target_positions
float32 duration
---
# Response
bool success
string message
```
```
# Service to set joint positions with a specified duration
# Used for precise control of humanoid robot posture
```

## Action Definitions (.action)

### NavigateToPose.action
```
# Goal
geometry_msgs/PoseStamped pose
string behavior_tree
---
# Result
int32 outcome
string message
---
# Feedback
int32 current_state
string current_state_description
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
```
```
# Action for navigating the robot to a specific pose
# Provides feedback on progress and final outcome
```

### ExecuteTrajectory.action
```
# Goal
trajectory_msgs/JointTrajectory trajectory
bool blocking
---
# Result
bool success
string message
---
# Feedback
float32 progress
int32 current_point
string state
```
```
# Action for executing a joint trajectory
# Used for complex humanoid movements
```

## Publisher/Subscriber Contracts

### Publisher: `/recognized_text`
- **Message Type**: `std_msgs/String`
- **Purpose**: Publishes recognized text from voice recognition
- **Frequency**: As needed when voice command is recognized
- **Reliability**: Best effort (voice commands are typically not critical path)

### Subscriber: `/recognized_text`
- **Message Type**: `std_msgs/String`
- **Purpose**: Receives recognized text for LLM processing
- **QoS**: Best effort
- **Callback**: Triggers LLM command generation

### Publisher: `/robot_action`
- **Message Type**: `physical_ai_brain/RobotAction`
- **Purpose**: Publishes high-level robot actions from LLM
- **Frequency**: As needed when LLM generates actions
- **Reliability**: Reliable (actions are important for robot behavior)

### Publisher: `/joint_commands`
- **Message Type**: `sensor_msgs/JointState`
- **Purpose**: Sends joint position commands to robot controllers
- **Frequency**: 50-100 Hz for smooth control
- **Reliability**: Reliable (control commands are time-critical)

### Publisher: `/camera/rgb/image_raw`
- **Message Type**: `sensor_msgs/Image`
- **Purpose**: Publishes RGB camera images
- **Frequency**: 15-30 Hz depending on application
- **Reliability**: Best effort (some dropped frames acceptable)

### Publisher: `/camera/depth/image_raw`
- **Message Type**: `sensor_msgs/Image`
- **Purpose**: Publishes depth images
- **Frequency**: 15-30 Hz
- **Reliability**: Best effort

### Publisher: `/imu/data`
- **Message Type**: `sensor_msgs/Imu`
- **Purpose**: Publishes IMU data for localization and control
- **Frequency**: 100-200 Hz
- **Reliability**: Reliable (critical for stability)

## Service Server Contracts

### Service Server: `/execute_robot_action`
- **Service Type**: `physical_ai_brain/ExecuteRobotAction`
- **Purpose**: Executes high-level robot actions
- **Implementation**: Action manager node
- **Availability**: Always available when robot is operational

### Service Server: `/get_robot_pose`
- **Service Type**: `physical_ai_brain/GetRobotPose`
- **Purpose**: Retrieves current robot pose
- **Implementation**: Localization node
- **Availability**: Available when localization is active

## Action Server Contracts

### Action Server: `/navigate_to_pose`
- **Action Type**: `nav2_msgs/NavigateToPose` (or custom)
- **Purpose**: Navigate robot to specified pose
- **Implementation**: Nav2 stack with custom humanoid controllers
- **Feedback**: Provides continuous feedback on navigation progress

### Action Server: `/execute_trajectory`
- **Action Type**: `control_msgs/FollowJointTrajectory`
- **Purpose**: Execute joint space trajectories
- **Implementation**: Joint trajectory controller
- **Feedback**: Provides feedback on trajectory execution progress

## LLM Integration Contracts

### Publisher: `/llm_command`
- **Message Type**: `physical_ai_brain/RobotAction`
- **Purpose**: Publishes commands generated by LLM
- **Format**: JSON with action type and parameters
- **Example**: `{"action": "navigate", "coordinates": [x, y, theta]}`

### Subscriber: `/llm_command`
- **Message Type**: `physical_ai_brain/RobotAction`
- **Purpose**: Receives LLM-generated commands
- **Processing**: Converts to ROS 2 actions or service calls
- **Validation**: Ensures commands are safe and executable