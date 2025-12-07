---
sidebar_position: 1
title: "Chapter 10: The Brain (LLM Action Planning)"
---

# Chapter 10: The Brain (LLM Action Planning)

## Overview

This chapter explores the central intelligence system of the humanoid robot - the Large Language Model (LLM) brain that interprets voice commands and generates appropriate robot actions. We'll cover system prompt design, LLM integration for action planning, and connection to navigation systems.

## Learning Objectives

By the end of this chapter, you will be able to:
- Design system prompts that output structured JSON for robot actions
- Implement LLM commander nodes that subscribe to recognized text
- Connect LLM outputs to navigation action clients
- Handle LLM response parsing and error handling
- Implement fallback mechanisms for when LLM fails

## System Prompt Design for Robot Actions

### Designing Structured Output Prompts

The system prompt is crucial for ensuring the LLM outputs structured JSON that can be directly converted to robot actions:

```markdown
# System Prompt for Humanoid Robot Commander

You are an AI commander for a 12-DOF bipedal humanoid robot. Your role is to interpret natural language commands and convert them into structured JSON actions that the robot can execute.

## Guidelines:
1. Respond ONLY with valid JSON. No explanatory text outside the JSON.
2. Use the following action types: "navigate", "rotate", "dance", "wave", "sit", "stand", "stop", "greet", "follow", "look_at"
3. For navigation, include x, y coordinates and theta (rotation) in meters and radians
4. For rotation, specify angle in degrees
5. For dance, specify style (simple, complex, happy, sad)
6. For follow, specify target (person, object, lead)
7. For look_at, specify direction (left, right, up, down) or coordinates

## Examples:
- User: "Go to the kitchen" → {"action": "navigate", "x": 2.0, "y": 3.0, "theta": 0.0}
- User: "Turn left" → {"action": "rotate", "angle": 90}
- User: "Do a little dance" → {"action": "dance", "style": "simple"}
- User: "Wave hello" → {"action": "wave", "style": "friendly"}

## Constraints:
- Keep movements within safe limits
- Avoid obstacles if known
- Maintain balance at all times
- If command is unclear, ask for clarification with {"action": "ask_clarification", "question": "..."}
```

### Prompt Templates

Different prompt templates for various scenarios:

```python
# Basic command prompt
BASIC_COMMAND_PROMPT = """
You are commanding a 12-DOF bipedal humanoid robot. Convert the following command into JSON:

Command: {command}
Context: {context}

Respond ONLY with valid JSON in this format:
{{"action": "action_type", "parameters": {{...}}}}

Available actions: navigate, rotate, dance, wave, sit, stand, stop, greet, follow, look_at
"""

# Safety-aware command prompt
SAFETY_AWARE_PROMPT = """
You are commanding a 12-DOF bipedal humanoid robot. Consider safety in all actions.

Command: {command}
Current position: {current_pos}
Known obstacles: {obstacles}
Safety constraints: {safety_constraints}

Convert to JSON ensuring:
1. Safe movement patterns
2. Balance preservation
3. Obstacle avoidance

Respond ONLY with valid JSON: {{"action": "type", "params": {{}}}}
"""

# Context-aware command prompt
CONTEXT_AWARE_PROMPT = """
Humanoid robot commander. Consider current context.

Previous actions: {history}
Current state: {state}
Environment: {environment}

Command: {command}

Convert to structured JSON action considering context.
"""
```

## LLM Commander Node Implementation

### Basic LLM Commander Node

```python
#!/usr/bin/env python3

"""
LLM Commander Node for Humanoid Robotics

This ROS 2 node subscribes to recognized text and uses an LLM to generate
robot actions in JSON format.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import openai
import threading
import queue
import time
from typing import Dict, Any, Optional


class LLMCommanderNode(Node):
    """
    ROS 2 node that uses LLM to convert voice commands to robot actions.
    """

    def __init__(self):
        """
        Initialize the LLM commander node.
        """
        super().__init__('llm_commander')

        # Declare parameters
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('model', 'gpt-4-turbo-preview')
        self.declare_parameter('temperature', 0.1)
        self.declare_parameter('max_tokens', 200)
        self.declare_parameter('system_prompt', self.get_default_system_prompt())

        # Get parameter values
        self.api_key = self.get_parameter('openai_api_key').value
        self.model = self.get_parameter('model').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.system_prompt = self.get_parameter('system_prompt').value

        # Validate API key
        if not self.api_key:
            self.get_logger().error("OpenAI API key is required!")
            return

        # Initialize OpenAI client
        openai.api_key = self.api_key

        # Create subscription for recognized text
        self.text_subscription = self.create_subscription(
            String,
            'recognized_text',
            self.text_callback,
            QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )

        # Create publisher for robot actions
        self.action_publisher = self.create_publisher(
            String,  # Will publish JSON actions
            'robot_actions',
            10
        )

        # Initialize navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Processing queue for commands
        self.command_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_commands)
        self.processing_thread.start()

        self.get_logger().info("LLM Commander node initialized")

    def get_default_system_prompt(self) -> str:
        """
        Get the default system prompt for the LLM.

        Returns:
            Default system prompt string
        """
        return """
You are an AI commander for a 12-DOF bipedal humanoid robot. Your role is to interpret natural language commands and convert them into structured JSON actions that the robot can execute.

## Guidelines:
1. Respond ONLY with valid JSON. No explanatory text outside the JSON.
2. Use the following action types: "navigate", "rotate", "dance", "wave", "sit", "stand", "stop", "greet", "follow", "look_at"
3. For navigation, include x, y coordinates and theta (rotation) in meters and radians
4. For rotation, specify angle in degrees
5. For dance, specify style (simple, complex, happy, sad)
6. For follow, specify target (person, object, lead)
7. For look_at, specify direction (left, right, up, down) or coordinates

## Examples:
- User: "Go to the kitchen" → {"action": "navigate", "x": 2.0, "y": 3.0, "theta": 0.0}
- User: "Turn left" → {"action": "rotate", "angle": 90}
- User: "Do a little dance" → {"action": "dance", "style": "simple"}
- User: "Wave hello" → {"action": "wave", "style": "friendly"}

## Constraints:
- Keep movements within safe limits
- Avoid obstacles if known
- Maintain balance at all times
- If command is unclear, ask for clarification with {"action": "ask_clarification", "question": "..."}
"""

    def text_callback(self, msg: String):
        """
        Callback for recognized text messages.

        Args:
            msg: Recognized text message
        """
        self.get_logger().info(f"Received command: {msg.data}")

        # Add command to processing queue
        self.command_queue.put({
            'text': msg.data,
            'timestamp': time.time()
        })

    def process_commands(self):
        """
        Process commands from the queue in a separate thread.
        """
        while rclpy.ok():
            try:
                # Get command from queue with timeout
                command = self.command_queue.get(timeout=1.0)

                # Process the command with LLM
                action_json = self.process_command_with_llm(command['text'])

                if action_json:
                    # Publish the action
                    action_msg = String()
                    action_msg.data = action_json
                    self.action_publisher.publish(action_msg)
                    self.get_logger().info(f"Published action: {action_json}")

                    # Execute the action
                    self.execute_action(json.loads(action_json))

            except queue.Empty:
                # Queue is empty, continue loop
                continue
            except Exception as e:
                self.get_logger().error(f"Error processing command: {e}")

    def process_command_with_llm(self, command: str) -> Optional[str]:
        """
        Process a command using the LLM to generate a robot action.

        Args:
            command: Natural language command

        Returns:
            JSON string representing the robot action or None if failed
        """
        try:
            # Create the message for the LLM
            messages = [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": command}
            ]

            # Call the OpenAI API
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                response_format={"type": "json_object"}  # Ensure JSON response
            )

            # Extract the response
            llm_response = response.choices[0].message.content.strip()

            # Validate that response is valid JSON
            try:
                parsed_json = json.loads(llm_response)

                # Basic validation of required fields
                if 'action' not in parsed_json:
                    self.get_logger().error("LLM response missing 'action' field")
                    return None

                return llm_response
            except json.JSONDecodeError:
                self.get_logger().error(f"LLM response is not valid JSON: {llm_response}")
                return None

        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            return None

    def execute_action(self, action_dict: Dict[str, Any]):
        """
        Execute a parsed action.

        Args:
            action_dict: Dictionary representing the action
        """
        action_type = action_dict.get('action', 'unknown')

        if action_type == 'navigate':
            self.execute_navigation_action(action_dict)
        elif action_type == 'rotate':
            self.execute_rotation_action(action_dict)
        elif action_type == 'dance':
            self.execute_dance_action(action_dict)
        elif action_type == 'wave':
            self.execute_wave_action(action_dict)
        elif action_type == 'sit':
            self.execute_sit_action(action_dict)
        elif action_type == 'stand':
            self.execute_stand_action(action_dict)
        elif action_type == 'stop':
            self.execute_stop_action(action_dict)
        elif action_type == 'greet':
            self.execute_greet_action(action_dict)
        elif action_type == 'follow':
            self.execute_follow_action(action_dict)
        elif action_type == 'look_at':
            self.execute_look_at_action(action_dict)
        elif action_type == 'ask_clarification':
            self.execute_ask_clarification_action(action_dict)
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")

    def execute_navigation_action(self, action_dict: Dict[str, Any]):
        """
        Execute a navigation action.

        Args:
            action_dict: Navigation action dictionary
        """
        x = action_dict.get('x', 0.0)
        y = action_dict.get('y', 0.0)
        theta = action_dict.get('theta', 0.0)

        self.get_logger().info(f"Navigating to: ({x}, {y}, {theta})")

        # Wait for navigation server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Navigation server not available")
            return

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from math import sin, cos
        sin_half_theta = sin(theta / 2.0)
        cos_half_theta = cos(theta / 2.0)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sin_half_theta
        goal_msg.pose.pose.orientation.w = cos_half_theta

        # Send navigation goal
        self.nav_client.send_goal_async(goal_msg)

    def execute_rotation_action(self, action_dict: Dict[str, Any]):
        """
        Execute a rotation action.

        Args:
            action_dict: Rotation action dictionary
        """
        angle_degrees = action_dict.get('angle', 0.0)
        self.get_logger().info(f"Rotating by {angle_degrees} degrees")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_dance_action(self, action_dict: Dict[str, Any]):
        """
        Execute a dance action.

        Args:
            action_dict: Dance action dictionary
        """
        style = action_dict.get('style', 'simple')
        self.get_logger().info(f"Dancing with style: {style}")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_wave_action(self, action_dict: Dict[str, Any]):
        """
        Execute a wave action.

        Args:
            action_dict: Wave action dictionary
        """
        style = action_dict.get('style', 'friendly')
        self.get_logger().info(f"Waving with style: {style}")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_sit_action(self, action_dict: Dict[str, Any]):
        """
        Execute a sit action.

        Args:
            action_dict: Sit action dictionary
        """
        self.get_logger().info("Sitting down")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_stand_action(self, action_dict: Dict[str, Any]):
        """
        Execute a stand action.

        Args:
            action_dict: Stand action dictionary
        """
        self.get_logger().info("Standing up")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_stop_action(self, action_dict: Dict[str, Any]):
        """
        Execute a stop action.

        Args:
            action_dict: Stop action dictionary
        """
        self.get_logger().info("Stopping")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_greet_action(self, action_dict: Dict[str, Any]):
        """
        Execute a greet action.

        Args:
            action_dict: Greet action dictionary
        """
        style = action_dict.get('style', 'friendly')
        self.get_logger().info(f"Greeting with style: {style}")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_follow_action(self, action_dict: Dict[str, Any]):
        """
        Execute a follow action.

        Args:
            action_dict: Follow action dictionary
        """
        target = action_dict.get('target', 'person')
        self.get_logger().info(f"Following: {target}")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_look_at_action(self, action_dict: Dict[str, Any]):
        """
        Execute a look_at action.

        Args:
            action_dict: Look_at action dictionary
        """
        direction = action_dict.get('direction', 'forward')
        self.get_logger().info(f"Looking at: {direction}")

        # Implementation would depend on specific robot capabilities
        # This is a placeholder
        pass

    def execute_ask_clarification_action(self, action_dict: Dict[str, Any]):
        """
        Execute an ask_clarification action.

        Args:
            action_dict: Ask clarification action dictionary
        """
        question = action_dict.get('question', 'Could you repeat that?')
        self.get_logger().info(f"Asking clarification: {question}")

        # In a real implementation, this would trigger text-to-speech
        # to ask the user for clarification
        pass


def main(args=None):
    """
    Main function to run the LLM commander node.

    Args:
        args: Command line arguments
    """
    try:
        rclpy.init(args=args)

        # Create and run the LLM commander node
        llm_commander = LLMCommanderNode()

        # Spin the node
        rclpy.spin(llm_commander)

    except KeyboardInterrupt:
        print("LLM commander interrupted by user")
    except Exception as e:
        print(f"Error in LLM commander: {e}")
    finally:
        # Clean up
        if 'llm_commander' in locals():
            llm_commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Error Handling and Fallback Mechanisms

### LLM Response Validation

Proper validation of LLM responses is essential for robot safety:

```python
def validate_llm_response(self, response: str) -> tuple[bool, str, Optional[dict]]:
    """
    Validate an LLM response for proper format and safety.

    Args:
        response: Raw LLM response string

    Returns:
        Tuple of (is_valid, error_message, parsed_json)
    """
    try:
        # Parse JSON
        parsed_json = json.loads(response)
    except json.JSONDecodeError as e:
        return False, f"Invalid JSON: {e}", None

    # Check required fields
    if 'action' not in parsed_json:
        return False, "Missing 'action' field", None

    action_type = parsed_json['action']

    # Validate action-specific parameters
    if action_type == 'navigate':
        required_fields = ['x', 'y', 'theta']
        for field in required_fields:
            if field not in parsed_json:
                return False, f"Navigate action missing '{field}' field", None

        # Validate ranges
        x, y, theta = parsed_json['x'], parsed_json['y'], parsed_json['theta']
        if abs(x) > 10.0 or abs(y) > 10.0:  # Reasonable limits
            return False, f"Navigation coordinates out of bounds: ({x}, {y})", None

    elif action_type == 'rotate':
        if 'angle' not in parsed_json:
            return False, "Rotate action missing 'angle' field", None

        angle = parsed_json['angle']
        if abs(angle) > 180:  # Reasonable rotation limit
            return False, f"Rotation angle out of bounds: {angle}", None

    elif action_type in ['dance', 'wave', 'sit', 'stand', 'stop', 'greet']:
        # These actions have no required parameters beyond 'action'
        pass

    else:
        return False, f"Unknown action type: {action_type}", None

    return True, "", parsed_json


def safe_process_command_with_llm(self, command: str) -> Optional[str]:
    """
    Safely process a command with LLM, including validation and fallbacks.

    Args:
        command: Natural language command

    Returns:
        JSON string representing the robot action or None if failed
    """
    try:
        # Process with LLM
        response = self.process_command_with_llm(command)
        if not response:
            return self.get_fallback_action(command)

        # Validate response
        is_valid, error_msg, parsed_json = self.validate_llm_response(response)
        if not is_valid:
            self.get_logger().error(f"LLM response validation failed: {error_msg}")

            # Try to fix common issues
            fixed_response = self.attempt_response_fix(response)
            if fixed_response:
                is_valid, _, parsed_json = self.validate_llm_response(fixed_response)
                if is_valid:
                    return fixed_response

            # Use fallback if validation fails
            return self.get_fallback_action(command)

        return response

    except Exception as e:
        self.get_logger().error(f"Error in safe command processing: {e}")
        return self.get_fallback_action(command)


def attempt_response_fix(self, response: str) -> Optional[str]:
    """
    Attempt to fix common LLM response issues.

    Args:
        response: Raw LLM response

    Returns:
        Fixed response string or None if fix not possible
    """
    import re

    # Try to extract JSON from response that might have extra text
    json_match = re.search(r'\{.*\}', response, re.DOTALL)
    if json_match:
        potential_json = json_match.group(0)
        try:
            # Validate the extracted JSON
            json.loads(potential_json)
            return potential_json
        except json.JSONDecodeError:
            pass

    # If the response is wrapped in code blocks, try to extract it
    code_block_match = re.search(r'```(?:json)?\s*(\{.*?\})\s*```', response, re.DOTALL)
    if code_block_match:
        potential_json = code_block_match.group(1)
        try:
            json.loads(potential_json)
            return potential_json
        except json.JSONDecodeError:
            pass

    return None


def get_fallback_action(self, command: str) -> Optional[str]:
    """
    Get a fallback action when LLM processing fails.

    Args:
        command: Original command

    Returns:
        Fallback action JSON or None
    """
    # Simple rule-based fallback for common commands
    command_lower = command.lower()

    if any(word in command_lower for word in ['stop', 'halt', 'freeze']):
        return '{"action": "stop"}'
    elif any(word in command_lower for word in ['sit', 'sit down', 'take a seat']):
        return '{"action": "sit"}'
    elif any(word in command_lower for word in ['stand', 'stand up', 'get up']):
        return '{"action": "stand"}'
    elif any(word in command_lower for word in ['wave', 'wag', 'hello']):
        return '{"action": "wave", "style": "friendly"}'
    elif any(word in command_lower for word in ['dance', 'move', 'boogie']):
        return '{"action": "dance", "style": "simple"}'
    else:
        # Ask for clarification
        return f'{{"action": "ask_clarification", "question": "I didn\'t understand \'{command}\'. Could you rephrase that?"}}'
```

## Advanced LLM Integration Patterns

### Context-Aware Command Processing

Enhanced command processing with context awareness:

```python
class ContextAwareLLMCommander(LLMCommanderNode):
    """
    Enhanced LLM commander with context awareness.
    """

    def __init__(self):
        """
        Initialize context-aware commander.
        """
        super().__init__()

        # Initialize context storage
        self.command_history = []
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery_level': 100.0,
            'current_task': None,
            'last_action_time': time.time()
        }
        self.environment_map = {}  # Known locations and objects

        # Initialize with default environment map
        self.initialize_environment_map()

    def initialize_environment_map(self):
        """
        Initialize the environment map with known locations.
        """
        self.environment_map = {
            'kitchen': {'x': 2.0, 'y': 3.0, 'theta': 0.0},
            'living_room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'bedroom': {'x': -2.0, 'y': 1.0, 'theta': 1.57},
            'bathroom': {'x': -1.0, 'y': -2.0, 'theta': 3.14}
        }

    def enhance_command_with_context(self, command: str) -> str:
        """
        Enhance a command with contextual information.

        Args:
            command: Original command

        Returns:
            Enhanced command with context
        """
        # Replace location references with coordinates
        enhanced_command = command.lower()

        for location, coords in self.environment_map.items():
            if location in enhanced_command:
                enhanced_command = enhanced_command.replace(
                    location,
                    f"coordinates ({coords['x']}, {coords['y']})"
                )

        return enhanced_command

    def process_command_with_context(self, command: str) -> Optional[str]:
        """
        Process a command with full context awareness.

        Args:
            command: Natural language command

        Returns:
            JSON string representing the robot action or None if failed
        """
        # Enhance command with context
        enhanced_command = self.enhance_command_with_context(command)

        # Build context message
        context_message = self.build_context_message(command, enhanced_command)

        try:
            # Create messages with context
            messages = [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": context_message}
            ]

            # Call the OpenAI API
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                response_format={"type": "json_object"}
            )

            # Extract and validate response
            llm_response = response.choices[0].message.content.strip()

            # Validate response
            is_valid, error_msg, parsed_json = self.validate_llm_response(llm_response)
            if not is_valid:
                self.get_logger().error(f"Context-aware response validation failed: {error_msg}")
                return self.get_fallback_action(command)

            # Add to command history
            self.command_history.append({
                'command': command,
                'enhanced_command': enhanced_command,
                'response': parsed_json,
                'timestamp': time.time()
            })

            # Keep only recent history (last 20 commands)
            if len(self.command_history) > 20:
                self.command_history = self.command_history[-20:]

            return llm_response

        except Exception as e:
            self.get_logger().error(f"Error in context-aware processing: {e}")
            return self.get_fallback_action(command)

    def build_context_message(self, original_command: str, enhanced_command: str) -> str:
        """
        Build a context message for the LLM.

        Args:
            original_command: Original command
            enhanced_command: Enhanced command with context

        Returns:
            Context message string
        """
        context = {
            'current_position': self.robot_state['position'],
            'battery_level': self.robot_state['battery_level'],
            'known_locations': list(self.environment_map.keys()),
            'recent_commands': [cmd['command'] for cmd in self.command_history[-3:]],
            'current_task': self.robot_state['current_task'],
            'time_of_day': time.strftime('%H:%M', time.localtime())
        }

        context_str = f"""
Command: {original_command}

Context:
- Current position: ({context['current_position']['x']}, {context['current_position']['y']})
- Battery level: {context['battery_level']}%
- Known locations: {', '.join(context['known_locations'])}
- Recent commands: {', '.join(context['recent_commands'])}
- Current task: {context['current_task']}
- Time: {context['time_of_day']}

Enhanced command with location context: {enhanced_command}

Convert to structured JSON action considering the context.
"""

        return context_str
```

## Integration with Navigation System

### Advanced Navigation Integration

```python
class AdvancedNavIntegration:
    """
    Advanced integration with navigation system.
    """

    def __init__(self, node):
        """
        Initialize navigation integration.

        Args:
            node: ROS 2 node instance
        """
        self.node = node
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.cancel_goal_future = None

    def execute_navigation_with_obstacle_avoidance(self, x: float, y: float, theta: float):
        """
        Execute navigation with obstacle avoidance.

        Args:
            x: Target X coordinate
            y: Target Y coordinate
            theta: Target orientation
        """
        # Check if navigation server is available
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Navigation server not available")
            return

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from math import sin, cos
        sin_half_theta = sin(theta / 2.0)
        cos_half_theta = cos(theta / 2.0)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sin_half_theta
        goal_msg.pose.pose.orientation.w = cos_half_theta

        # Send navigation goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)

        # Add feedback callback
        send_goal_future.add_done_callback(self.navigation_goal_sent_callback)

    def navigation_goal_sent_callback(self, future):
        """
        Callback when navigation goal is sent.

        Args:
            future: Future object for the goal request
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error("Navigation goal was rejected")
                return

            self.node.get_logger().info("Navigation goal accepted, executing...")

            # Get result future
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.navigation_result_callback)

        except Exception as e:
            self.node.get_logger().error(f"Error sending navigation goal: {e}")

    def navigation_result_callback(self, future):
        """
        Callback when navigation result is received.

        Args:
            future: Future object for the result
        """
        try:
            result = future.result()
            status = result.status
            nav_result = result.result

            if status == 4:  # SUCCESS
                self.node.get_logger().info("Navigation completed successfully")
            else:
                self.node.get_logger().warn(f"Navigation failed with status: {status}")

        except Exception as e:
            self.node.get_logger().error(f"Error getting navigation result: {e}")
```

## Prompt Templates and Management

### Managing Multiple Prompt Templates

```python
class PromptManager:
    """
    Manage multiple system prompts for different scenarios.
    """

    def __init__(self):
        """
        Initialize prompt manager.
        """
        self.prompts = {
            'default': """
You are an AI commander for a 12-DOF bipedal humanoid robot. Your role is to interpret natural language commands and convert them into structured JSON actions that the robot can execute.

## Guidelines:
1. Respond ONLY with valid JSON. No explanatory text outside the JSON.
2. Use the following action types: "navigate", "rotate", "dance", "wave", "sit", "stand", "stop", "greet", "follow", "look_at"
""",
            'safety_first': """
You are an AI commander for a 12-DOF bipedal humanoid robot. SAFETY IS THE TOP PRIORITY in all actions.

## Safety Guidelines:
1. Respond ONLY with valid JSON.
2. Always consider safety in navigation and movement.
3. Stop if environment seems unsafe.
4. Use "ask_clarification" if command might be unsafe.
""",
            'energy_efficient': """
You are an AI commander for a 12-DOF bipedal humanoid robot. MINIMIZE ENERGY USAGE in all actions.

## Efficiency Guidelines:
1. Respond ONLY with valid JSON.
2. Choose energy-efficient paths and movements.
3. Use minimal motion when possible.
4. Prefer sitting to standing when idle.
""",
            'social_interaction': """
You are an AI commander for a 12-DOF bipedal humanoid robot focused on social interaction.

## Social Guidelines:
1. Respond ONLY with valid JSON.
2. Prioritize greeting and interacting with people.
3. Use expressive actions like waving and dancing.
4. Engage in conversations through "ask_clarification".
"""
        }

    def get_prompt(self, scenario: str = 'default') -> str:
        """
        Get a specific prompt for a scenario.

        Args:
            scenario: Scenario name

        Returns:
            Prompt string for the scenario
        """
        return self.prompts.get(scenario, self.prompts['default'])

    def register_custom_prompt(self, name: str, prompt: str):
        """
        Register a custom prompt.

        Args:
            name: Name for the prompt
            prompt: Prompt string
        """
        self.prompts[name] = prompt

    def list_available_prompts(self) -> list:
        """
        List all available prompts.

        Returns:
            List of available prompt names
        """
        return list(self.prompts.keys())
```

## Summary

In this chapter, we've explored the brain of the humanoid robot - the LLM-based action planning system. You've learned how to:

- Design system prompts that ensure structured JSON output
- Implement LLM commander nodes that process voice commands
- Connect LLM outputs to navigation and action systems
- Handle errors and implement fallback mechanisms
- Add context awareness to improve command interpretation
- Manage multiple prompt templates for different scenarios

The LLM commander acts as the central intelligence that translates natural language commands into executable robot actions, making the robot more accessible and intuitive to interact with. Proper error handling and safety considerations ensure that the robot operates reliably even when the LLM produces unexpected results.

In the next chapter, we'll explore Vision-Language-Action (VLA) models that combine visual perception with language understanding to enable more sophisticated robot behaviors.