---
sidebar_position: 1
title: "Chapter 3: ROS 2 Architecture"
---

# Chapter 3: ROS 2 Architecture

## Overview

This chapter introduces the Robot Operating System (ROS 2) architecture, focusing on the communication patterns essential for humanoid robotics applications. We'll explore the publish-subscribe model, service-based communication, and action-based communication patterns that enable coordination between different robot components.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the core concepts of ROS 2 architecture
- Implement publisher-subscriber communication patterns
- Create and use custom message types for humanoid control
- Design service-based interfaces for synchronous communication
- Structure ROS 2 packages for humanoid robotics applications

## ROS 2 Fundamentals

### What is ROS 2?

Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike traditional operating systems, ROS 2 is not an actual OS but rather a middleware that provides services designed for a heterogeneous computer cluster, including:
- Hardware abstraction
- Device drivers
- Libraries
- Visualizers
- Message-passing
- Package management

### DDS (Data Distribution Service)

ROS 2 uses DDS as its underlying communication middleware. DDS provides:
- Real-time data exchange
- Quality of Service (QoS) policies
- Publish-subscribe communication pattern
- Discovery and matching of publishers and subscribers

## Communication Patterns

### Publisher-Subscriber (Topics)

The publish-subscribe pattern is the most common communication pattern in ROS 2. Publishers send messages to topics, and subscribers receive messages from topics.

#### Basic Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Basic Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Custom Message Types for Humanoid Control

### JointAngles Message

For humanoid robots, we need to define custom messages that can handle joint control. Let's look at our JointAngles message:

```python
# In msg/JointAngles.msg
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
```

#### Using Custom Messages in Code

```python
import rclpy
from rclpy.node import Node
from physical_ai_description.msg import JointAngles  # Custom message

class JointController(Node):

    def __init__(self):
        super().__init__('joint_controller')
        self.publisher_ = self.create_publisher(JointAngles, 'joint_commands', 10)

        # Timer to send joint commands periodically
        self.timer = self.create_timer(0.1, self.send_joint_commands)
        self.i = 0

    def send_joint_commands(self):
        msg = JointAngles()
        msg.joint_names = ['left_hip_yaw', 'left_hip_roll', 'left_knee',
                          'right_hip_yaw', 'right_hip_roll', 'right_knee']
        msg.positions = [0.0, 0.1, 0.2, 0.0, -0.1, -0.2]  # Example positions
        msg.velocities = [0.0] * 6  # Zero velocities
        msg.efforts = [0.0] * 6     # Zero efforts

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint angles: {msg.positions}')

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    rclpy.spin(joint_controller)
    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services provide synchronous request-response communication in ROS 2.

### Service Definition

```python
# In srv/SetJointPositions.srv
JointAngles target_positions  # Request
---
bool success                 # Response
string message              # Response
```

### Service Server

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            SetBool,
            'set_bool_service',
            self.set_bool_callback)

    def set_bool_callback(self, request, response):
        response.success = request.data
        response.message = f'Service called with: {request.data}'
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(SetBool, 'set_bool_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClient()
    response = minimal_client.send_request(True)
    minimal_client.get_logger().info(f'Result: {response}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

Actions are used for long-running tasks with feedback and goal management.

### Action Definition

```python
# In action/FollowJointTrajectory.action
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

## Quality of Service (QoS) Settings

QoS settings are crucial for robotics applications, especially for humanoid robots where timing can be critical:

```python
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

# For critical control messages
control_qos = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
)

# For sensor data where latest data is more important
sensor_qos = QoSProfile(
    depth=5,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
)
```

## Package Structure for Humanoid Robotics

A typical ROS 2 package for humanoid robotics follows this structure:

```
physical_ai_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── robot.launch.py
│   └── simulation.launch.py
├── config/
│   ├── controllers.yaml
│   └── robot_parameters.yaml
├── nodes/
│   ├── joint_controller.py
│   └── behavior_manager.py
└── params/
    └── default_params.yaml
```

## Launch Files

Launch files allow you to start multiple nodes with a single command:

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='physical_ai_bringup',
            executable='joint_controller',
            name='joint_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        Node(
            package='physical_ai_bringup',
            executable='behavior_manager',
            name='behavior_manager',
            output='screen'
        )
    ])
```

## Best Practices for Humanoid Robotics

1. **Use appropriate QoS settings** for different types of data
2. **Implement proper error handling** for all ROS communication
3. **Design clear, consistent interfaces** for your custom messages
4. **Use namespaces** to organize related nodes and topics
5. **Document your interfaces** clearly for other developers
6. **Implement graceful degradation** when services are unavailable

## Summary

In this chapter, we've covered the fundamental ROS 2 architecture concepts essential for humanoid robotics. You've learned about:
- Publisher-subscriber communication patterns
- Service-based synchronous communication
- Custom message types for humanoid control
- Quality of Service settings for different use cases
- Package structure for robotics applications

In the next chapter, we'll explore the body representation of humanoid robots using URDF and kinematics.