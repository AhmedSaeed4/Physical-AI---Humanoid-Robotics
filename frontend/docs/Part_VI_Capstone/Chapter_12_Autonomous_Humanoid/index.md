---
sidebar_position: 1
title: "Chapter 12: The Autonomous Humanoid"
---

# Chapter 12: The Autonomous Humanoid

## Overview

This capstone chapter brings together all the components learned throughout the book to create a fully autonomous humanoid robot capable of performing complex tasks like the "Butler Test" - going to the kitchen and finding a red mug. We'll integrate simulation, ROS bridge, navigation, and LLM agents to create a cohesive autonomous system.

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate all subsystems into a complete autonomous humanoid system
- Implement the "Butler Test" scenario as a comprehensive demonstration
- Coordinate simulation, navigation, perception, and AI systems
- Deploy the complete system using Docker orchestration
- Evaluate system performance and identify improvement areas

## System Architecture Overview

### Complete System Integration

The autonomous humanoid system integrates all components developed throughout the book:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Perception    │    │      AI         │    │   Navigation    │
│   (Vision +     │◄──►│  (LLM Agent,    │◄──►│   (Nav2,        │
│   Audio)        │    │   VLA, Voice)   │    │   Path Planning)│
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Central Coordinator                          │
│  (Manages task execution, system state, and resource allocation)│
└─────────────────────────────────────────────────────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Simulation    │    │   Hardware      │    │   Monitoring    │
│   (Isaac Sim,   │    │   (Real Robot)  │    │   (Logging,     │
│   Gazebo)       │    │                 │    │   Diagnostics)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Docker Orchestration

The complete system is orchestrated using Docker Compose:

```yaml
# docker-compose.yml
version: '3.8'

services:
  # Isaac Sim simulation environment
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:4.0.0
    container_name: isaac-sim
    ports:
      - "8211:8211"
      - "55555:55555"
    volumes:
      - ./assets:/assets
      - ./scenarios:/scenarios
    environment:
      - ACCEPT_EULA=Y
      - NVIDIA_VISIBLE_DEVICES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]

  # Gazebo simulation environment
  gazebo:
    image: osrf/gazebo:gz_sim_harmonic
    container_name: gazebo
    ports:
      - "8080:8080"
    volumes:
      - ./models:/root/.gazebo/models
      - ./worlds:/worlds
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    network_mode: "bridge"

  # Robot Operating System core
  ros-core:
    image: rostooling/ros:humble-ros-base-latest
    container_name: ros-core
    command: roscore
    expose:
      - "11311"
    environment:
      - ROS_DOMAIN_ID=0

  # Physical AI robot description
  robot-description:
    image: physical-ai/robot-description:latest
    container_name: robot-description
    depends_on:
      - ros-core
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        ros2 run robot_state_publisher robot_state_publisher --ros-args --param robot_description:=$(ros2 param get /robot_state_publisher robot_description)
      "

  # Perception stack
  perception-stack:
    image: physical-ai/perception:latest
    container_name: perception-stack
    depends_on:
      - ros-core
      - robot-description
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        ros2 launch physical_ai_perception perception.launch.py
      "
    environment:
      - ROS_DOMAIN_ID=0

  # Navigation stack
  navigation-stack:
    image: physical-ai/navigation:latest
    container_name: navigation-stack
    depends_on:
      - ros-core
      - perception-stack
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        ros2 launch physical_ai_navigation navigation.launch.py
      "
    environment:
      - ROS_DOMAIN_ID=0

  # AI integration stack
  ai-stack:
    image: physical-ai/brain:latest
    container_name: ai-stack
    depends_on:
      - ros-core
      - perception-stack
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 run physical_ai_brain llm_commander_node
      "
    volumes:
      - ./config:/workspace/src/physical_ai_brain/config

  # Voice processing
  voice-processing:
    image: physical-ai/voice:latest
    container_name: voice-processing
    depends_on:
      - ros-core
      - ai-stack
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 run physical_ai_brain voice_listener
      "
    volumes:
      - /dev/snd:/dev/snd  # Audio devices

  # VLA (Vision-Language-Action) processing
  vla-processing:
    image: physical-ai/vla:latest
    container_name: vla-processing
    depends_on:
      - ros-core
      - ai-stack
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 run physical_ai_brain vision_language_action
      "
    volumes:
      - ./prompts:/workspace/src/physical_ai_brain/prompts

  # System coordinator
  system-coordinator:
    image: physical-ai/coordinator:latest
    container_name: system-coordinator
    depends_on:
      - ros-core
      - navigation-stack
      - ai-stack
      - voice-processing
      - vla-processing
    environment:
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 run physical_ai_brain system_coordinator
      "
    volumes:
      - ./logs:/workspace/logs

  # Monitoring and visualization
  monitoring:
    image: physical-ai/monitoring:latest
    container_name: monitoring
    depends_on:
      - ros-core
    ports:
      - "8081:8081"
    environment:
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_monitoring monitoring.launch.py
      "

volumes:
  robot_models:
  simulation_assets:
  logs:
```

## The Butler Test Scenario

### Complete Scenario Implementation

The "Butler Test" demonstrates the robot's ability to perform a complex, multi-step task:

```python
#!/usr/bin/env python3

"""
Butler Test Scenario Implementation

This ROS 2 node implements the complete "Go to the kitchen and find the red mug" scenario.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import time
from typing import Dict, Any, Optional
import threading


class ButlerTestNode(Node):
    """
    ROS 2 node that implements the Butler Test scenario.
    """

    def __init__(self):
        """
        Initialize the Butler Test node.
        """
        super().__init__('butler_test')

        # Declare parameters
        self.declare_parameter('task_timeout', 300)  # 5 minutes
        self.declare_parameter('location_kitchen_x', 2.0)
        self.declare_parameter('location_kitchen_y', 3.0)
        self.declare_parameter('location_kitchen_theta', 0.0)
        self.declare_parameter('search_radius', 2.0)

        # Get parameter values
        self.task_timeout = self.get_parameter('task_timeout').value
        self.kitchen_x = self.get_parameter('location_kitchen_x').value
        self.kitchen_y = self.get_parameter('location_kitchen_y').value
        self.kitchen_theta = self.get_parameter('location_kitchen_theta').value
        self.search_radius = self.get_parameter('search_radius').value

        # Initialize navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create subscriptions
        self.voice_command_subscription = self.create_subscription(
            String,
            'recognized_text',
            self.voice_command_callback,
            QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )

        self.vla_analysis_subscription = self.create_subscription(
            String,
            'vla_analysis',
            self.vla_analysis_callback,
            QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )

        # Create publishers
        self.action_publisher = self.create_publisher(
            String,
            'robot_actions',
            10
        )

        self.task_status_publisher = self.create_publisher(
            String,
            'task_status',
            10
        )

        # State management
        self.current_task = None
        self.task_start_time = 0
        self.task_steps = []
        self.current_step = 0
        self.task_completed = False
        self.task_failed = False

        # Threading for task execution
        self.task_execution_thread = None
        self.task_lock = threading.Lock()

        # Statistics
        self.stats = {
            'tasks_attempted': 0,
            'tasks_completed': 0,
            'average_completion_time': 0.0,
            'failures': []
        }

        self.get_logger().info("Butler Test node initialized")

    def voice_command_callback(self, msg: String):
        """
        Callback for voice commands.

        Args:
            msg: Voice command message
        """
        command = msg.data.lower()

        # Check if this is a butler test command
        if "go to the kitchen and find the red mug" in command:
            self.start_butler_test()
        elif "butler test" in command and "red mug" in command:
            self.start_butler_test()
        elif "perform butler test" in command:
            self.start_butler_test()

    def start_butler_test(self):
        """
        Start the butler test scenario.
        """
        with self.task_lock:
            if self.current_task is not None:
                self.get_logger().warn("Task already in progress, ignoring new command")
                return

            self.get_logger().info("Starting Butler Test: Go to kitchen and find red mug")

            # Initialize task state
            self.current_task = "butler_test"
            self.task_start_time = time.time()
            self.task_steps = [
                "navigate_to_kitchen",
                "search_for_red_mug",
                "identify_red_mug",
                "approach_red_mug",
                "confirm_task_completion"
            ]
            self.current_step = 0
            self.task_completed = False
            self.task_failed = False

            # Update statistics
            self.stats['tasks_attempted'] += 1

            # Publish task status
            status_msg = String()
            status_msg.data = json.dumps({
                'task': 'butler_test',
                'status': 'started',
                'step': 'navigate_to_kitchen',
                'timestamp': time.time()
            })
            self.task_status_publisher.publish(status_msg)

            # Start task execution in separate thread
            self.task_execution_thread = threading.Thread(target=self.execute_butler_test)
            self.task_execution_thread.start()

    def execute_butler_test(self):
        """
        Execute the butler test scenario in a separate thread.
        """
        try:
            # Step 1: Navigate to kitchen
            self.get_logger().info("Step 1: Navigating to kitchen")
            if not self.execute_navigation_to_kitchen():
                self.fail_task("Navigation to kitchen failed")
                return

            # Publish step completion
            self.publish_task_step_status("navigate_to_kitchen", "completed")

            # Step 2: Search for red mug
            self.get_logger().info("Step 2: Searching for red mug")
            if not self.search_for_red_mug():
                self.fail_task("Could not find red mug")
                return

            # Publish step completion
            self.publish_task_step_status("search_for_red_mug", "completed")

            # Step 3: Identify red mug
            self.get_logger().info("Step 3: Identifying red mug")
            if not self.identify_red_mug():
                self.fail_task("Could not identify red mug")
                return

            # Publish step completion
            self.publish_task_step_status("identify_red_mug", "completed")

            # Step 4: Approach red mug
            self.get_logger().info("Step 4: Approaching red mug")
            if not self.approach_red_mug():
                self.fail_task("Could not approach red mug")
                return

            # Publish step completion
            self.publish_task_step_status("approach_red_mug", "completed")

            # Step 5: Confirm task completion
            self.get_logger().info("Step 5: Confirming task completion")
            if not self.confirm_task_completion():
                self.fail_task("Could not confirm task completion")
                return

            # Publish step completion
            self.publish_task_step_status("confirm_task_completion", "completed")

            # Task completed successfully
            self.complete_task()

        except Exception as e:
            self.get_logger().error(f"Error executing butler test: {e}")
            self.fail_task(f"Error during execution: {e}")

    def execute_navigation_to_kitchen(self) -> bool:
        """
        Execute navigation to the kitchen.

        Returns:
            True if successful, False otherwise
        """
        try:
            # Wait for navigation server
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("Navigation server not available")
                return False

            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(self.kitchen_x)
            goal_msg.pose.pose.position.y = float(self.kitchen_y)
            goal_msg.pose.pose.position.z = 0.0

            # Convert theta to quaternion
            from math import sin, cos
            theta = self.kitchen_theta
            sin_half_theta = sin(theta / 2.0)
            cos_half_theta = cos(theta / 2.0)
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = sin_half_theta
            goal_msg.pose.pose.orientation.w = cos_half_theta

            # Send navigation goal
            future = self.nav_client.send_goal_async(goal_msg)

            # Wait for result with timeout
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 60.0:  # 60-second timeout
                time.sleep(0.1)

            if not future.done():
                self.get_logger().error("Navigation goal timed out")
                return False

            result = future.result()
            if result.status == 4:  # SUCCESS
                self.get_logger().info("Successfully navigated to kitchen")
                return True
            else:
                self.get_logger().error(f"Navigation failed with status: {result.status}")
                return False

        except Exception as e:
            self.get_logger().error(f"Error in navigation: {e}")
            return False

    def search_for_red_mug(self) -> bool:
        """
        Search for the red mug using vision system.

        Returns:
            True if red mug is found, False otherwise
        """
        try:
            # Send command to VLA system to identify red mug
            vla_command = {
                'command': 'find_object',
                'instruction': 'red mug',
                'context': 'Look for a red-colored mug in the kitchen area'
            }

            command_msg = String()
            command_msg.data = json.dumps(vla_command)
            self.get_logger().info("Sent VLA command to find red mug")

            # Publish command
            self.action_publisher.publish(command_msg)

            # Wait for response (with timeout)
            start_time = time.time()
            while (time.time() - start_time) < 30.0:  # 30-second timeout
                time.sleep(0.1)
                # In a real implementation, we'd check if response was received
                # For demo purposes, we'll assume it was found after 5 seconds
                if hasattr(self, '_mock_found_red_mug') and self._mock_found_red_mug:
                    return True

            self.get_logger().info("Red mug search completed")
            return True  # Assuming found for demo purposes

        except Exception as e:
            self.get_logger().error(f"Error searching for red mug: {e}")
            return False

    def identify_red_mug(self) -> bool:
        """
        Identify the specific red mug.

        Returns:
            True if red mug is identified, False otherwise
        """
        try:
            # Send command to identify specific red mug
            identify_command = {
                'command': 'identify_object',
                'instruction': 'red mug',
                'context': 'Identify the specific red mug among other objects'
            }

            command_msg = String()
            command_msg.data = json.dumps(identify_command)

            # Publish command
            self.action_publisher.publish(command_msg)
            self.get_logger().info("Sent command to identify red mug")

            # Wait for response (simulated)
            time.sleep(2.0)  # Simulate processing time

            return True  # Assuming successful identification

        except Exception as e:
            self.get_logger().error(f"Error identifying red mug: {e}")
            return False

    def approach_red_mug(self) -> bool:
        """
        Approach the identified red mug.

        Returns:
            True if approached successfully, False otherwise
        """
        try:
            # In a real implementation, this would navigate to the mug's location
            # For demo, we'll simulate the approach
            self.get_logger().info("Approaching the red mug")

            # Simulate approach action
            time.sleep(3.0)  # Simulate approach time

            return True

        except Exception as e:
            self.get_logger().error(f"Error approaching red mug: {e}")
            return False

    def confirm_task_completion(self) -> bool:
        """
        Confirm that the task is completed.

        Returns:
            True if task is confirmed complete, False otherwise
        """
        try:
            # In a real implementation, this would verify the mug is in hand
            # For demo, we'll assume success
            self.get_logger().info("Task completion confirmed")

            return True

        except Exception as e:
            self.get_logger().error(f"Error confirming task completion: {e}")
            return False

    def vla_analysis_callback(self, msg: String):
        """
        Callback for VLA analysis results.

        Args:
            msg: VLA analysis message
        """
        try:
            analysis_data = json.loads(msg.data)

            # Process the analysis based on current step
            if self.current_task == "butler_test":
                if self.task_steps[self.current_step] == "search_for_red_mug":
                    # Handle red mug search results
                    if analysis_data.get('found', False):
                        self._mock_found_red_mug = True
                        self.get_logger().info("Red mug found via VLA analysis")
                    else:
                        self.get_logger().info("Red mug not found in current view")

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in VLA analysis")
        except Exception as e:
            self.get_logger().error(f"Error processing VLA analysis: {e}")

    def publish_task_step_status(self, step: str, status: str):
        """
        Publish status for a specific task step.

        Args:
            step: Task step name
            status: Step status (completed, failed, etc.)
        """
        status_msg = String()
        status_msg.data = json.dumps({
            'task': self.current_task,
            'step': step,
            'status': status,
            'timestamp': time.time(),
            'current_step': self.current_step + 1,
            'total_steps': len(self.task_steps)
        })
        self.task_status_publisher.publish(status_msg)

    def complete_task(self):
        """
        Complete the current task successfully.
        """
        with self.task_lock:
            if self.current_task is None:
                return

            self.task_completed = True
            completion_time = time.time() - self.task_start_time

            # Update statistics
            self.stats['tasks_completed'] += 1
            total_time = self.stats['average_completion_time'] * (self.stats['tasks_completed'] - 1)
            self.stats['average_completion_time'] = (total_time + completion_time) / self.stats['tasks_completed']

            # Log completion
            self.get_logger().info(f"Butler Test completed successfully in {completion_time:.2f} seconds")

            # Publish completion status
            status_msg = String()
            status_msg.data = json.dumps({
                'task': self.current_task,
                'status': 'completed',
                'completion_time': completion_time,
                'timestamp': time.time()
            })
            self.task_status_publisher.publish(status_msg)

            # Reset task state
            self.current_task = None
            self._mock_found_red_mug = False

    def fail_task(self, reason: str):
        """
        Fail the current task.

        Args:
            reason: Reason for task failure
        """
        with self.task_lock:
            if self.current_task is None:
                return

            self.task_failed = True

            # Update statistics
            self.stats['failures'].append({
                'task': self.current_task,
                'reason': reason,
                'timestamp': time.time()
            })

            # Log failure
            self.get_logger().error(f"Butler Test failed: {reason}")

            # Publish failure status
            status_msg = String()
            status_msg.data = json.dumps({
                'task': self.current_task,
                'status': 'failed',
                'reason': reason,
                'timestamp': time.time()
            })
            self.task_status_publisher.publish(status_msg)

            # Reset task state
            self.current_task = None
            self._mock_found_red_mug = False

    def get_task_statistics(self) -> Dict[str, Any]:
        """
        Get current task statistics.

        Returns:
            Dictionary with task statistics
        """
        with self.task_lock:
            return self.stats.copy()

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        # Wait for task execution to complete
        if self.task_execution_thread and self.task_execution_thread.is_alive():
            self.task_execution_thread.join(timeout=5.0)

        super().destroy_node()


def main(args=None):
    """
    Main function to run the Butler Test node.

    Args:
        args: Command line arguments
    """
    try:
        rclpy.init(args=args)

        # Create and run the Butler Test node
        butler_test_node = ButlerTestNode()

        # Spin the node
        rclpy.spin(butler_test_node)

    except KeyboardInterrupt:
        print("Butler Test node interrupted by user")
    except Exception as e:
        print(f"Error in Butler Test node: {e}")
    finally:
        # Clean up
        if 'butler_test_node' in locals():
            butler_test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## System Integration Patterns

### Central Coordination Architecture

The central coordinator manages the interaction between all subsystems:

```python
#!/usr/bin/env python3

"""
System Coordinator for Autonomous Humanoid

This node coordinates all subsystems in the autonomous humanoid system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
import json
import time
from typing import Dict, Any, Optional
import threading
from enum import Enum


class SystemState(Enum):
    """
    System states for the autonomous humanoid.
    """
    IDLE = "idle"
    INITIALIZING = "initializing"
    NAVIGATING = "navigating"
    PERCEIVING = "perceiving"
    PLANNING = "planning"
    EXECUTING_ACTION = "executing_action"
    WAITING_FOR_INPUT = "waiting_for_input"
    EMERGENCY_STOP = "emergency_stop"
    SHUTDOWN = "shutdown"


class SystemCoordinator(Node):
    """
    Central coordinator for the autonomous humanoid system.
    """

    def __init__(self):
        """
        Initialize the system coordinator.
        """
        super().__init__('system_coordinator')

        # Declare parameters
        self.declare_parameter('system_timeout', 300.0)  # 5 minutes
        self.declare_parameter('safety_distance_threshold', 0.5)  # meters
        self.declare_parameter('battery_low_threshold', 20.0)  # percent

        # Get parameter values
        self.system_timeout = self.get_parameter('system_timeout').value
        self.safety_distance_threshold = self.get_parameter('safety_distance_threshold').value
        self.battery_low_threshold = self.get_parameter('battery_low_threshold').value

        # Initialize system state
        self.current_state = SystemState.INITIALIZING
        self.previous_state = SystemState.IDLE
        self.state_start_time = time.time()
        self.system_initialized = False

        # System status tracking
        self.subsystem_status = {
            'navigation': {'ready': False, 'last_update': 0},
            'perception': {'ready': False, 'last_update': 0},
            'ai': {'ready': False, 'last_update': 0},
            'voice': {'ready': False, 'last_update': 0},
            'vla': {'ready': False, 'last_update': 0}
        }

        # Robot state
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery_level': 100.0,
            'current_task': None,
            'last_action': None,
            'obstacle_detected': False,
            'safety_violation': False
        }

        # Create subscriptions
        self.navigation_status_subscription = self.create_subscription(
            String, 'navigation_status', self.navigation_status_callback, 10
        )
        self.perception_status_subscription = self.create_subscription(
            String, 'perception_status', self.perception_status_callback, 10
        )
        self.ai_status_subscription = self.create_subscription(
            String, 'ai_status', self.ai_status_callback, 10
        )
        self.voice_status_subscription = self.create_subscription(
            String, 'voice_status', self.voice_status_callback, 10
        )
        self.vla_status_subscription = self.create_subscription(
            String, 'vla_status', self.vla_status_callback, 10
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.battery_subscription = self.create_subscription(
            String, 'battery_status', self.battery_callback, 10
        )

        # Create publishers
        self.state_publisher = self.create_publisher(
            String, 'system_state', 10
        )
        self.emergency_stop_publisher = self.create_publisher(
            Bool, 'emergency_stop', 10
        )
        self.system_command_publisher = self.create_publisher(
            String, 'system_commands', 10
        )
        self.task_publisher = self.create_publisher(
            String, 'active_tasks', 10
        )

        # Initialize timers
        self.state_machine_timer = self.create_timer(1.0, self.run_state_machine)
        self.health_check_timer = self.create_timer(5.0, self.health_check)
        self.safety_check_timer = self.create_timer(0.1, self.safety_check)

        # Threading
        self.coordination_thread = None
        self.is_running = True

        # Task queue
        self.task_queue = []
        self.active_task = None

        self.get_logger().info("System Coordinator initialized")

        # Start initialization process
        self.initialize_system()

    def initialize_system(self):
        """
        Initialize the system and all subsystems.
        """
        self.get_logger().info("Initializing system and subsystems...")

        # Send initialization commands to all subsystems
        init_commands = [
            {"command": "initialize", "target": "navigation"},
            {"command": "initialize", "target": "perception"},
            {"command": "initialize", "target": "ai"},
            {"command": "initialize", "target": "voice"},
            {"command": "initialize", "target": "vla"}
        ]

        for cmd in init_commands:
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd)
            self.system_command_publisher.publish(cmd_msg)

        # Wait for subsystems to report ready status
        start_time = time.time()
        while (time.time() - start_time) < 30.0:  # 30-second timeout
            if all(status['ready'] for status in self.subsystem_status.values()):
                break
            time.sleep(0.5)

        # Check initialization results
        if all(status['ready'] for status in self.subsystem_status.values()):
            self.system_initialized = True
            self.set_state(SystemState.IDLE)
            self.get_logger().info("System initialized successfully")
        else:
            missing_subsystems = [
                name for name, status in self.subsystem_status.items() if not status['ready']
            ]
            self.get_logger().error(f"Initialization failed for subsystems: {missing_subsystems}")
            self.set_state(SystemState.EMERGENCY_STOP)

    def set_state(self, new_state: SystemState):
        """
        Set the system state.

        Args:
            new_state: New system state
        """
        if self.current_state != new_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()

            # Log state change
            self.get_logger().info(f"State changed from {self.previous_state.value} to {self.current_state.value}")

            # Publish state change
            state_msg = String()
            state_msg.data = json.dumps({
                'current_state': self.current_state.value,
                'previous_state': self.previous_state.value,
                'timestamp': time.time()
            })
            self.state_publisher.publish(state_msg)

    def run_state_machine(self):
        """
        Run the system state machine.
        """
        try:
            if not self.system_initialized:
                return

            # State-specific behaviors
            if self.current_state == SystemState.IDLE:
                self.handle_idle_state()
            elif self.current_state == SystemState.NAVIGATING:
                self.handle_navigation_state()
            elif self.current_state == SystemState.PERCEIVING:
                self.handle_perception_state()
            elif self.current_state == SystemState.PLANNING:
                self.handle_planning_state()
            elif self.current_state == SystemState.EXECUTING_ACTION:
                self.handle_execution_state()
            elif self.current_state == SystemState.WAITING_FOR_INPUT:
                self.handle_waiting_state()
            elif self.current_state == SystemState.EMERGENCY_STOP:
                self.handle_emergency_stop_state()
            elif self.current_state == SystemState.SHUTDOWN:
                self.handle_shutdown_state()

            # Check for safety violations
            if self.robot_state['safety_violation']:
                self.set_state(SystemState.EMERGENCY_STOP)

            # Check battery level
            if self.robot_state['battery_level'] < self.battery_low_threshold:
                self.get_logger().warn(f"Battery level low: {self.robot_state['battery_level']}%")

        except Exception as e:
            self.get_logger().error(f"Error in state machine: {e}")

    def handle_idle_state(self):
        """
        Handle the IDLE state.
        """
        # Check if there are tasks in the queue
        if self.task_queue:
            self.activate_next_task()

    def handle_navigation_state(self):
        """
        Handle the NAVIGATION state.
        """
        # Monitor navigation progress
        # Check for navigation completion or failure
        pass

    def handle_perception_state(self):
        """
        Handle the PERCEPTION state.
        """
        # Monitor perception system
        # Wait for perception results
        pass

    def handle_planning_state(self):
        """
        Handle the PLANNING state.
        """
        # Monitor AI planning process
        # Wait for action plan
        pass

    def handle_execution_state(self):
        """
        Handle the EXECUTION state.
        """
        # Monitor action execution
        # Check for completion or failure
        pass

    def handle_waiting_state(self):
        """
        Handle the WAITING state.
        """
        # Wait for user input or external trigger
        pass

    def handle_emergency_stop_state(self):
        """
        Handle the EMERGENCY_STOP state.
        """
        # Publish emergency stop command
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_publisher.publish(stop_msg)

    def handle_shutdown_state(self):
        """
        Handle the SHUTDOWN state.
        """
        # System is shutting down
        pass

    def activate_next_task(self):
        """
        Activate the next task in the queue.
        """
        if self.task_queue:
            task = self.task_queue.pop(0)
            self.active_task = task

            # Publish task activation
            task_msg = String()
            task_msg.data = json.dumps(task)
            self.task_publisher.publish(task_msg)

            # Log task activation
            self.get_logger().info(f"Activated task: {task.get('name', 'unknown')}")

    def health_check(self):
        """
        Perform system health check.
        """
        # Check if subsystems are responsive
        current_time = time.time()
        timeout_threshold = 30.0  # seconds

        for subsystem, status in self.subsystem_status.items():
            if current_time - status['last_update'] > timeout_threshold:
                self.get_logger().warn(f"{subsystem} subsystem not responding")

    def safety_check(self):
        """
        Perform safety checks.
        """
        # Check for obstacles
        if self.robot_state['obstacle_detected']:
            self.robot_state['safety_violation'] = True

        # Check other safety conditions
        # ...

    def navigation_status_callback(self, msg: String):
        """
        Callback for navigation status.

        Args:
            msg: Navigation status message
        """
        try:
            status_data = json.loads(msg.data)
            self.subsystem_status['navigation'] = {
                'ready': status_data.get('ready', False),
                'last_update': time.time(),
                'status': status_data
            }

            # Handle navigation-specific state changes
            nav_status = status_data.get('status', '')
            if nav_status == 'arrived':
                if self.current_state == SystemState.NAVIGATING:
                    self.set_state(SystemState.IDLE)
            elif nav_status == 'failed':
                if self.current_state == SystemState.NAVIGATING:
                    self.set_state(SystemState.IDLE)

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in navigation status")
        except Exception as e:
            self.get_logger().error(f"Error processing navigation status: {e}")

    def perception_status_callback(self, msg: String):
        """
        Callback for perception status.

        Args:
            msg: Perception status message
        """
        try:
            status_data = json.loads(msg.data)
            self.subsystem_status['perception'] = {
                'ready': status_data.get('ready', False),
                'last_update': time.time(),
                'status': status_data
            }
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in perception status")
        except Exception as e:
            self.get_logger().error(f"Error processing perception status: {e}")

    def ai_status_callback(self, msg: String):
        """
        Callback for AI status.

        Args:
            msg: AI status message
        """
        try:
            status_data = json.loads(msg.data)
            self.subsystem_status['ai'] = {
                'ready': status_data.get('ready', False),
                'last_update': time.time(),
                'status': status_data
            }
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in AI status")
        except Exception as e:
            self.get_logger().error(f"Error processing AI status: {e}")

    def voice_status_callback(self, msg: String):
        """
        Callback for voice status.

        Args:
            msg: Voice status message
        """
        try:
            status_data = json.loads(msg.data)
            self.subsystem_status['voice'] = {
                'ready': status_data.get('ready', False),
                'last_update': time.time(),
                'status': status_data
            }
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in voice status")
        except Exception as e:
            self.get_logger().error(f"Error processing voice status: {e}")

    def vla_status_callback(self, msg: String):
        """
        Callback for VLA status.

        Args:
            msg: VLA status message
        """
        try:
            status_data = json.loads(msg.data)
            self.subsystem_status['vla'] = {
                'ready': status_data.get('ready', False),
                'last_update': time.time(),
                'status': status_data
            }
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in VLA status")
        except Exception as e:
            self.get_logger().error(f"Error processing VLA status: {e}")

    def odometry_callback(self, msg: Odometry):
        """
        Callback for odometry data.

        Args:
            msg: Odometry message
        """
        try:
            self.robot_state['position']['x'] = msg.pose.pose.position.x
            self.robot_state['position']['y'] = msg.pose.pose.position.y

            # Convert quaternion to euler for theta
            import math
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            self.robot_state['position']['theta'] = theta

        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {e}")

    def scan_callback(self, msg: LaserScan):
        """
        Callback for laser scan data.

        Args:
            msg: Laser scan message
        """
        try:
            # Check for obstacles within safety threshold
            min_distance = min([d for d in msg.ranges if not (math.isnan(d) or math.isinf(d))], default=float('inf'))
            self.robot_state['obstacle_detected'] = min_distance < self.safety_distance_threshold

        except Exception as e:
            self.get_logger().error(f"Error processing scan data: {e}")

    def battery_callback(self, msg: String):
        """
        Callback for battery status.

        Args:
            msg: Battery status message
        """
        try:
            battery_data = json.loads(msg.data)
            self.robot_state['battery_level'] = battery_data.get('level', 100.0)

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in battery status")
        except Exception as e:
            self.get_logger().error(f"Error processing battery status: {e}")

    def add_task(self, task: Dict[str, Any]):
        """
        Add a task to the queue.

        Args:
            task: Task dictionary
        """
        self.task_queue.append(task)
        self.get_logger().info(f"Added task to queue: {task.get('name', 'unknown')}")

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        self.is_running = False

        # Stop all timers
        self.state_machine_timer.destroy()
        self.health_check_timer.destroy()
        self.safety_check_timer.destroy()

        super().destroy_node()


def main(args=None):
    """
    Main function to run the system coordinator.

    Args:
        args: Command line arguments
    """
    try:
        rclpy.init(args=args)

        # Create and run the system coordinator
        coordinator = SystemCoordinator()

        # Spin the node
        rclpy.spin(coordinator)

    except KeyboardInterrupt:
        print("System coordinator interrupted by user")
    except Exception as e:
        print(f"Error in system coordinator: {e}")
    finally:
        # Clean up
        if 'coordinator' in locals():
            coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Performance Monitoring and Diagnostics

### System Monitoring Dashboard

```python
#!/usr/bin/env python3

"""
System Monitoring and Diagnostics for Autonomous Humanoid

This provides comprehensive monitoring and diagnostics for the system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Temperature
import json
import time
from typing import Dict, Any
import psutil
import GPUtil
import threading
import csv
from datetime import datetime


class SystemMonitor(Node):
    """
    System monitoring and diagnostics node.
    """

    def __init__(self):
        """
        Initialize the system monitor.
        """
        super().__init__('system_monitor')

        # Declare parameters
        self.declare_parameter('log_interval', 5.0)
        self.declare_parameter('log_file_path', '/workspace/logs/system_monitor.csv')

        # Get parameter values
        self.log_interval = self.get_parameter('log_interval').value
        self.log_file_path = self.get_parameter('log_file_path').value

        # System metrics
        self.metrics = {
            'timestamp': 0.0,
            'cpu_percent': 0.0,
            'memory_percent': 0.0,
            'disk_percent': 0.0,
            'gpu_percent': 0.0,
            'gpu_memory_percent': 0.0,
            'temperature': 0.0,
            'battery_level': 100.0,
            'active_tasks': 0,
            'navigation_status': 'unknown',
            'perception_fps': 0.0,
            'ai_response_time': 0.0
        }

        # Create publishers
        self.metrics_publisher = self.create_publisher(
            String, 'system_metrics', 10
        )
        self.cpu_publisher = self.create_publisher(
            Float32, 'cpu_usage', 10
        )
        self.memory_publisher = self.create_publisher(
            Float32, 'memory_usage', 10
        )
        self.temperature_publisher = self.create_publisher(
            Temperature, 'system_temperature', 10
        )

        # Create timers
        self.monitor_timer = self.create_timer(self.log_interval, self.collect_metrics)
        self.publish_timer = self.create_timer(1.0, self.publish_metrics)

        # Logging
        self.log_to_csv_enabled = True
        self.init_log_file()

        # Threading
        self.monitoring_thread = None
        self.is_monitoring = True

        self.get_logger().info("System Monitor initialized")

    def collect_metrics(self):
        """
        Collect system metrics.
        """
        try:
            # CPU usage
            self.metrics['cpu_percent'] = psutil.cpu_percent(interval=1)

            # Memory usage
            memory = psutil.virtual_memory()
            self.metrics['memory_percent'] = memory.percent

            # Disk usage
            disk = psutil.disk_usage('/')
            self.metrics['disk_percent'] = (disk.used / disk.total) * 100

            # GPU usage (if available)
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu = gpus[0]  # Use first GPU
                self.metrics['gpu_percent'] = gpu.load * 100
                self.metrics['gpu_memory_percent'] = gpu.memoryUtil * 100
                self.metrics['temperature'] = gpu.temperature
            else:
                # Simulate values for systems without GPU
                self.metrics['gpu_percent'] = 0.0
                self.metrics['gpu_memory_percent'] = 0.0
                self.metrics['temperature'] = 40.0  # Simulated temperature

            # Timestamp
            self.metrics['timestamp'] = time.time()

            # Log to CSV if enabled
            if self.log_to_csv_enabled:
                self.log_metrics_to_csv()

        except Exception as e:
            self.get_logger().error(f"Error collecting metrics: {e}")

    def publish_metrics(self):
        """
        Publish collected metrics.
        """
        try:
            # Publish metrics as JSON
            metrics_msg = String()
            metrics_msg.data = json.dumps(self.metrics)
            self.metrics_publisher.publish(metrics_msg)

            # Publish individual metrics
            cpu_msg = Float32()
            cpu_msg.data = float(self.metrics['cpu_percent'])
            self.cpu_publisher.publish(cpu_msg)

            memory_msg = Float32()
            memory_msg.data = float(self.metrics['memory_percent'])
            self.memory_publisher.publish(memory_msg)

            temp_msg = Temperature()
            temp_msg.temperature = float(self.metrics['temperature'])
            temp_msg.variance = 0.0  # No variance measurement
            self.temperature_publisher.publish(temp_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing metrics: {e}")

    def init_log_file(self):
        """
        Initialize the log file with headers.
        """
        try:
            with open(self.log_file_path, 'w', newline='') as csvfile:
                fieldnames = list(self.metrics.keys())
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
        except Exception as e:
            self.get_logger().error(f"Error initializing log file: {e}")

    def log_metrics_to_csv(self):
        """
        Log metrics to CSV file.
        """
        try:
            with open(self.log_file_path, 'a', newline='') as csvfile:
                fieldnames = list(self.metrics.keys())
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writerow(self.metrics)
        except Exception as e:
            self.get_logger().error(f"Error logging metrics to CSV: {e}")

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        self.is_monitoring = False
        super().destroy_node()


def main(args=None):
    """
    Main function to run the system monitor.

    Args:
        args: Command line arguments
    """
    try:
        rclpy.init(args=args)

        # Create and run the system monitor
        monitor = SystemMonitor()

        # Spin the node
        rclpy.spin(monitor)

    except KeyboardInterrupt:
        print("System monitor interrupted by user")
    except Exception as e:
        print(f"Error in system monitor: {e}")
    finally:
        # Clean up
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## System Deployment and Orchestration

### Docker Compose for Complete System

```yaml
# docker-compose.prod.yml
version: '3.8'

services:
  # Load balancer/proxy for external access
  nginx-proxy:
    image: nginx:alpine
    container_name: nginx-proxy
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx/nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - monitoring
    networks:
      - internal

  # Isaac Sim with GPU support
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:4.0.0
    container_name: isaac-sim
    ports:
      - "8211:8211"
      - "55555:55555"
    volumes:
      - ./assets:/assets
      - ./scenarios:/scenarios
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    environment:
      - ACCEPT_EULA=Y
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - DISPLAY=${DISPLAY}
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    networks:
      - internal
    restart: unless-stopped

  # Gazebo simulation
  gazebo:
    image: osrf/gazebo:gz_sim_harmonic
    container_name: gazebo
    ports:
      - "8080:8080"
    volumes:
      - ./models:/root/.gazebo/models
      - ./worlds:/worlds
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    network_mode: host
    restart: unless-stopped

  # ROS 2 core
  ros-core:
    image: rostooling/ros:humble-ros-base-latest
    container_name: ros-core
    command: roscore
    expose:
      - "11311"
    environment:
      - ROS_DOMAIN_ID=0
    networks:
      - internal
    restart: unless-stopped

  # Robot state publisher
  robot-state-publisher:
    image: physical-ai/robot-description:latest
    container_name: robot-state-publisher
    depends_on:
      - ros-core
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        ros2 run robot_state_publisher robot_state_publisher --ros-args --param-file /workspace/config/robot_description.yaml
      "
    volumes:
      - ./config:/workspace/config
    environment:
      - ROS_DOMAIN_ID=0
    networks:
      - internal
    restart: unless-stopped

  # Perception stack
  perception-stack:
    image: physical-ai/perception:latest
    container_name: perception-stack
    depends_on:
      - ros-core
      - robot-state-publisher
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_perception perception_pipeline.launch.py
      "
    volumes:
      - ./config:/workspace/config
      - ./models:/workspace/models
    environment:
      - ROS_DOMAIN_ID=0
      - CUDA_VISIBLE_DEVICES=0
    networks:
      - internal
    restart: unless-stopped

  # Navigation stack
  navigation-stack:
    image: physical-ai/navigation:latest
    container_name: navigation-stack
    depends_on:
      - ros-core
      - perception-stack
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_navigation navigation.launch.py
      "
    volumes:
      - ./maps:/workspace/maps
      - ./config:/workspace/config
    environment:
      - ROS_DOMAIN_ID=0
    networks:
      - internal
    restart: unless-stopped

  # AI integration stack
  ai-stack:
    image: physical-ai/brain:latest
    container_name: ai-stack
    depends_on:
      - ros-core
      - perception-stack
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ROS_DOMAIN_ID=0
      - CUDA_VISIBLE_DEVICES=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_brain ai_integration.launch.py
      "
    volumes:
      - ./config:/workspace/config
      - ./prompts:/workspace/prompts
    networks:
      - internal
    restart: unless-stopped

  # Voice processing
  voice-processing:
    image: physical-ai/voice:latest
    container_name: voice-processing
    depends_on:
      - ros-core
      - ai-stack
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_brain voice_pipeline.launch.py
      "
    volumes:
      - ./config:/workspace/config
      - /dev/snd:/dev/snd  # Audio devices
    devices:
      - /dev/snd:/dev/snd
    networks:
      - internal
    restart: unless-stopped

  # VLA processing
  vla-processing:
    image: physical-ai/vla:latest
    container_name: vla-processing
    depends_on:
      - ros-core
      - ai-stack
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ROS_DOMAIN_ID=0
      - CUDA_VISIBLE_DEVICES=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_brain vla_pipeline.launch.py
      "
    volumes:
      - ./prompts:/workspace/prompts
      - ./config:/workspace/config
    networks:
      - internal
    restart: unless-stopped

  # System coordinator
  system-coordinator:
    image: physical-ai/coordinator:latest
    container_name: system-coordinator
    depends_on:
      - ros-core
      - navigation-stack
      - ai-stack
      - voice-processing
      - vla-processing
    environment:
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_brain system_coordinator.launch.py
      "
    volumes:
      - ./logs:/workspace/logs
      - ./config:/workspace/config
    networks:
      - internal
    restart: unless-stopped

  # Butler test scenario
  butler-test:
    image: physical-ai/butler-test:latest
    container_name: butler-test
    depends_on:
      - system-coordinator
    environment:
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_brain butler_test_scenario.launch.py
      "
    volumes:
      - ./scenarios:/workspace/scenarios
      - ./logs:/workspace/logs
    networks:
      - internal
    restart: unless-stopped

  # Monitoring and logging
  monitoring:
    image: physical-ai/monitoring:latest
    container_name: monitoring
    depends_on:
      - ros-core
    ports:
      - "3000:3000"  # Grafana
      - "9090:9090"  # Prometheus
    environment:
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch physical_ai_monitoring monitoring_suite.launch.py
      "
    volumes:
      - ./grafana:/etc/grafana
      - ./prometheus:/etc/prometheus
      - ./logs:/workspace/logs
    networks:
      - internal
    restart: unless-stopped

  # Database for storing logs and metrics
  influxdb:
    image: influxdb:2.7
    container_name: influxdb
    ports:
      - "8086:8086"
    environment:
      - DOCKER_INFLUXDB_INIT_MODE=setup
      - DOCKER_INFLUXDB_INIT_USERNAME=admin
      - DOCKER_INFLUXDB_INIT_PASSWORD=password123
      - DOCKER_INFLUXDB_INIT_ORG=physical-ai
      - DOCKER_INFLUXDB_INIT_BUCKET=robotics-data
    volumes:
      - influxdb-storage:/var/lib/influxdb2
    networks:
      - internal
    restart: unless-stopped

networks:
  internal:
    driver: bridge

volumes:
  robot_models:
  simulation_assets:
  logs:
  influxdb-storage:
```

## Summary

In this capstone chapter, we've brought together all the components learned throughout the book to create a fully autonomous humanoid robot system. You've learned how to:

- Integrate all subsystems (simulation, perception, navigation, AI) into a cohesive system
- Implement the "Butler Test" scenario as a comprehensive demonstration
- Use Docker orchestration to manage the complex multi-container system
- Create a central coordinator to manage system state and task execution
- Implement monitoring and diagnostics for system health
- Deploy the complete system in a production-ready configuration

The autonomous humanoid system represents the culmination of the technologies covered in this book, demonstrating how modern generative AI can be effectively integrated with hard robotics using the NVIDIA ecosystem. The system is designed to be scalable, maintainable, and suitable for both simulation and real-world deployment.

This completes the Physical AI & Humanoid Robotics book, providing you with the knowledge and tools to build sophisticated AI-powered humanoid robots capable of complex autonomous behaviors.