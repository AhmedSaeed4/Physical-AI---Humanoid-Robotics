---
sidebar_position: 1
title: "Chapter 11: Vision-Language-Action (VLA)"
---

# Chapter 11: Vision-Language-Action (VLA)

## Overview

This chapter explores Vision-Language-Action (VLA) models that integrate visual perception, natural language understanding, and robotic action planning. VLA models enable robots to perceive their environment, understand complex instructions, and execute appropriate actions in a coordinated manner.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand VLA model architectures and their applications in robotics
- Integrate vision-language models like GPT-4 Vision or LLaVA with robot control
- Implement visual prompting for robot action planning
- Create perception-action loops for autonomous robot behavior
- Handle multi-modal inputs and coordinate responses

## Understanding VLA Models

### What are VLA Models?

Vision-Language-Action (VLA) models are multi-modal AI systems that can:
- Process visual inputs (images, video streams)
- Understand natural language instructions
- Generate appropriate robotic actions
- Learn from vision-language-action demonstrations

### Key Characteristics of VLA Models

1. **Multi-modal Integration**: Seamless fusion of visual, linguistic, and action modalities
2. **Embodied Learning**: Learning from real-world interactions and demonstrations
3. **Generalization**: Ability to generalize to novel tasks and environments
4. **Real-time Processing**: Fast inference for real-time robotic control

### VLA Model Architectures

```
Vision Input → Vision Encoder → Multi-modal Fusion → Language Decoder → Action Generator
     ↑                              ↓                      ↓
Language Input → Language Encoder →                       Action Output
```

## VLA Model Options

### Cloud-Based Solutions (GPT-4 Vision)

GPT-4 Vision provides powerful vision-language capabilities:

```python
import openai
import base64
import requests
from PIL import Image
import io


class GPT4VisionVLA:
    """
    VLA implementation using GPT-4 Vision API.
    """

    def __init__(self, api_key: str, model: str = "gpt-4-vision-preview"):
        """
        Initialize GPT-4 Vision VLA.

        Args:
            api_key: OpenAI API key
            model: Vision model name
        """
        openai.api_key = api_key
        self.model = model

    def encode_image(self, image_path: str) -> str:
        """
        Encode image to base64 for API transmission.

        Args:
            image_path: Path to image file

        Returns:
            Base64 encoded image string
        """
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    def process_vision_language_request(self,
                                      image_path: str,
                                      prompt: str) -> dict:
        """
        Process a vision-language request using GPT-4 Vision.

        Args:
            image_path: Path to image file
            prompt: Natural language instruction

        Returns:
            Dictionary with action and analysis results
        """
        try:
            # Encode image
            base64_image = self.encode_image(image_path)

            # Create message with image and text
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            }
                        }
                    ]
                }
            ]

            # Call the API
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=messages,
                max_tokens=300,
                response_format={"type": "json_object"}  # Ensure JSON response
            )

            # Extract and parse response
            response_text = response.choices[0].message.content
            return json.loads(response_text)

        except Exception as e:
            print(f"Error in VLA processing: {e}")
            return {"action": "error", "error": str(e)}

    def is_path_clear(self, image_path: str) -> bool:
        """
        Check if the path is clear using vision analysis.

        Args:
            image_path: Path to image showing the path

        Returns:
            True if path is clear, False otherwise
        """
        prompt = """
        Analyze the image and determine if the path ahead is clear for a humanoid robot to walk through.
        Consider obstacles, narrow passages, and surface conditions.
        Respond with JSON: {"path_clear": true/false, "obstacles": [...], "confidence": float}
        """

        result = self.process_vision_language_request(image_path, prompt)
        return result.get("path_clear", False)

    def identify_red_mug(self, image_path: str) -> dict:
        """
        Identify the red mug in the scene.

        Args:
            image_path: Path to image containing the scene

        Returns:
            Dictionary with mug location and characteristics
        """
        prompt = """
        Identify the red mug in the image and describe its location relative to the camera.
        Respond with JSON: {
            "found": true/false,
            "position": {"x": float, "y": float, "distance": float},
            "orientation": float,  # degrees from camera
            "confidence": float
        }
        """

        return self.process_vision_language_request(image_path, prompt)
```

### Local VLA Solutions (LLaVA)

For edge deployment, LLaVA provides local vision-language capabilities:

```python
from transformers import LlavaForConditionalGeneration, LlavaProcessor
import torch
from PIL import Image


class LLaVAVLA:
    """
    VLA implementation using local LLaVA model.
    """

    def __init__(self, model_name: str = "llava-hf/llava-1.5-7b-hf"):
        """
        Initialize local LLaVA VLA.

        Args:
            model_name: Name of the LLaVA model to use
        """
        # Check if CUDA is available
        device = "cuda" if torch.cuda.is_available() else "cpu"

        # Load model and processor
        self.model = LlavaForConditionalGeneration.from_pretrained(
            model_name,
            torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
            low_cpu_mem_usage=True
        ).to(device)

        self.processor = LlavaProcessor.from_pretrained(model_name)
        self.device = device

    def process_vision_language_request(self,
                                      image_path: str,
                                      prompt: str) -> str:
        """
        Process a vision-language request using local LLaVA.

        Args:
            image_path: Path to image file
            prompt: Natural language instruction

        Returns:
            Response text from the model
        """
        try:
            # Load image
            image = Image.open(image_path)

            # Format the prompt for LLaVA
            formatted_prompt = f"USER: {prompt}\nASSISTANT:"

            # Process inputs
            inputs = self.processor(formatted_prompt, image, return_tensors="pt").to(self.device)

            # Generate response
            with torch.no_grad():
                output = self.model.generate(**inputs, max_new_tokens=200)

            # Decode response
            response = self.processor.decode(output[0], skip_special_tokens=True)

            # Extract the assistant's response part
            if "ASSISTANT:" in response:
                return response.split("ASSISTANT:")[-1].strip()
            else:
                return response

        except Exception as e:
            print(f"Error in local VLA processing: {e}")
            return f"Error processing request: {e}"

    def analyze_scene(self, image_path: str) -> dict:
        """
        Analyze the scene in the image.

        Args:
            image_path: Path to image file

        Returns:
            Dictionary with scene analysis
        """
        prompt = "Describe the scene in detail, including objects, their positions, and potential actions."
        response = self.process_vision_language_request(image_path, prompt)

        # In a real implementation, you'd parse the response for structured data
        return {
            "analysis": response,
            "objects": self.extract_objects(response),
            "potential_actions": self.extract_actions(response)
        }

    def extract_objects(self, text: str) -> list:
        """
        Extract objects from the analysis text.

        Args:
            text: Text analysis from the model

        Returns:
            List of objects identified in the scene
        """
        # This is a simplified extraction - in practice, you'd use more sophisticated NLP
        import re
        # Look for common object words
        object_words = ["table", "chair", "mug", "cup", "box", "person", "robot", "door", "window"]
        found_objects = []
        for word in object_words:
            if word.lower() in text.lower():
                found_objects.append(word)
        return list(set(found_objects))  # Remove duplicates

    def extract_actions(self, text: str) -> list:
        """
        Extract potential actions from the analysis text.

        Args:
            text: Text analysis from the model

        Returns:
            List of potential actions
        """
        # This is a simplified extraction
        action_words = ["navigate", "grasp", "move", "avoid", "approach", "pick up", "put down"]
        found_actions = []
        for word in action_words:
            if word.lower() in text.lower():
                found_actions.append(word)
        return list(set(found_actions))
```

## VLA Integration with Robot Control

### VLA Commander Node

```python
#!/usr/bin/env python3

"""
VLA Commander Node for Humanoid Robotics

This ROS 2 node integrates Vision-Language-Action models with robot control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import tempfile
import os
from typing import Optional


class VLACommanderNode(Node):
    """
    ROS 2 node that integrates VLA models with robot control.
    """

    def __init__(self):
        """
        Initialize the VLA commander node.
        """
        super().__init__('vla_commander')

        # Declare parameters
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('use_local_vla', False)
        self.declare_parameter('model_name', 'gpt-4-vision-preview')
        self.declare_parameter('image_topic', '/camera/image_raw')

        # Get parameter values
        self.api_key = self.get_parameter('openai_api_key').value
        self.use_local_vla = self.get_parameter('use_local_vla').value
        self.model_name = self.get_parameter('model_name').value
        self.image_topic = self.get_parameter('image_topic').value

        # Initialize VLA engine
        if not self.use_local_vla and self.api_key:
            self.vla_engine = GPT4VisionVLA(self.api_key, self.model_name)
        elif self.use_local_vla:
            self.vla_engine = LLaVAVLA(self.model_name)
        else:
            self.get_logger().error("No VLA engine configured!")
            return

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.command_subscription = self.create_subscription(
            String,
            'vla_commands',
            self.command_callback,
            10
        )

        # Create publishers
        self.action_publisher = self.create_publisher(
            String,
            'robot_actions',
            10
        )

        self.analysis_publisher = self.create_publisher(
            String,
            'vla_analysis',
            10
        )

        # State variables
        self.latest_image: Optional[np.ndarray] = None
        self.vla_processing_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_vla_requests)
        self.processing_thread.start()

        self.get_logger().info("VLA Commander node initialized")

    def image_callback(self, msg: Image):
        """
        Callback for camera image messages.

        Args:
            msg: Image message
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image.copy()

            self.get_logger().debug(f"Received image: {cv_image.shape}")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def command_callback(self, msg: String):
        """
        Callback for VLA command messages.

        Args:
            msg: Command message
        """
        self.get_logger().info(f"Received VLA command: {msg.data}")

        # Parse command
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('command', '')
            instruction = command_data.get('instruction', '')

            if command_type == 'analyze_scene':
                self.queue_vla_request('analyze', instruction)
            elif command_type == 'identify_object':
                self.queue_vla_request('identify', instruction)
            elif command_type == 'is_path_clear':
                self.queue_vla_request('path_check', instruction)
            elif command_type == 'execute_action':
                self.queue_vla_request('action', instruction)

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON command: {msg.data}")

    def queue_vla_request(self, request_type: str, instruction: str):
        """
        Queue a VLA request for processing.

        Args:
            request_type: Type of request (analyze, identify, path_check, action)
            instruction: Natural language instruction
        """
        if self.latest_image is None:
            self.get_logger().warn("No image available for VLA processing")
            return

        # Save current image to temporary file
        temp_image_path = self.save_image_to_temp(self.latest_image)

        request = {
            'type': request_type,
            'instruction': instruction,
            'image_path': temp_image_path,
            'timestamp': time.time()
        }

        self.vla_processing_queue.put(request)

    def save_image_to_temp(self, cv_image: np.ndarray) -> str:
        """
        Save OpenCV image to temporary file.

        Args:
            cv_image: OpenCV image

        Returns:
            Path to temporary image file
        """
        # Create temporary file
        temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
        temp_file.close()

        # Save image
        cv2.imwrite(temp_file.name, cv_image)

        return temp_file.name

    def process_vla_requests(self):
        """
        Process VLA requests from the queue.
        """
        while rclpy.ok():
            try:
                # Get request from queue
                request = self.vla_processing_queue.get(timeout=1.0)

                # Process the request based on type
                if request['type'] == 'analyze':
                    result = self.vla_engine.analyze_scene(request['image_path'])
                elif request['type'] == 'identify':
                    result = self.vla_engine.process_vision_language_request(
                        request['image_path'],
                        request['instruction']
                    )
                elif request['type'] == 'path_check':
                    result = {'path_clear': self.vla_engine.is_path_clear(request['image_path'])}
                elif request['type'] == 'action':
                    result = self.vla_engine.process_vision_language_request(
                        request['image_path'],
                        request['instruction']
                    )
                else:
                    result = {'error': f'Unknown request type: {request["type"]}'}

                # Publish result
                result_msg = String()
                result_msg.data = json.dumps(result)
                self.analysis_publisher.publish(result_msg)

                # If result contains an action, publish it
                if 'action' in result:
                    action_msg = String()
                    action_msg.data = json.dumps(result)
                    self.action_publisher.publish(action_msg)

                # Clean up temporary file
                if os.path.exists(request['image_path']):
                    os.remove(request['image_path'])

                self.get_logger().info(f"Processed VLA request: {result}")

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error processing VLA request: {e}")

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        if hasattr(self, 'processing_thread'):
            # Thread will exit when rclpy.ok() returns False
            pass

        super().destroy_node()


def main(args=None):
    """
    Main function to run the VLA commander node.

    Args:
        args: Command line arguments
    """
    try:
        rclpy.init(args=args)

        # Create and run the VLA commander node
        vla_commander = VLACommanderNode()

        # Spin the node
        rclpy.spin(vla_commander)

    except KeyboardInterrupt:
        print("VLA commander interrupted by user")
    except Exception as e:
        print(f"Error in VLA commander: {e}")
    finally:
        # Clean up
        if 'vla_commander' in locals():
            vla_commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Visual Prompting for Robot Actions

### Creating Effective Visual Prompts

Visual prompting is crucial for getting accurate responses from VLA models:

```python
class VisualPrompter:
    """
    Helper class for creating effective visual prompts.
    """

    @staticmethod
    def create_path_clear_prompt(additional_context: str = "") -> str:
        """
        Create a prompt for checking if the path is clear.

        Args:
            additional_context: Additional context for the prompt

        Returns:
            Formatted prompt string
        """
        base_prompt = """
        Analyze the image to determine if the path ahead is clear for a humanoid robot to navigate.
        Consider the following factors:
        1. Obstacles in the path (furniture, objects, people)
        2. Surface conditions (holes, stairs, slippery surfaces)
        3. Passage width (narrow passages that might be difficult for a robot)
        4. Potential hazards (sharp objects, unstable items)

        Provide your analysis in JSON format:
        {
            "path_clear": true/false,
            "obstacles": [
                {
                    "type": "object type",
                    "position": "relative position",
                    "size": "approximate size",
                    "distance": "estimated distance in meters"
                }
            ],
            "surface_conditions": ["condition1", "condition2"],
            "confidence": 0.0-1.0,
            "recommendation": "suggested action"
        }
        """

        if additional_context:
            return f"{base_prompt}\n\nAdditional context: {additional_context}"
        return base_prompt

    @staticmethod
    def create_object_identification_prompt(object_description: str,
                                          additional_context: str = "") -> str:
        """
        Create a prompt for identifying specific objects.

        Args:
            object_description: Description of the object to identify
            additional_context: Additional context for the prompt

        Returns:
            Formatted prompt string
        """
        base_prompt = f"""
        Identify the following object in the image: {object_description}
        Provide details about the object's location, orientation, and accessibility.

        Respond in JSON format:
        {{
            "found": true/false,
            "object_type": "type of object identified",
            "position": {{
                "x": "horizontal position relative to center (-1 to 1)",
                "y": "vertical position relative to center (-1 to 1)",
                "distance": "estimated distance in meters"
            }},
            "orientation": "description of object orientation",
            "accessibility": "how accessible the object is",
            "confidence": 0.0-1.0,
            "bounding_box": [x_min, y_min, x_max, y_max]  # pixel coordinates
        }}
        """

        if additional_context:
            return f"{base_prompt}\n\nAdditional context: {additional_context}"
        return base_prompt

    @staticmethod
    def create_action_planning_prompt(instruction: str,
                                   environment_context: str = "",
                                   robot_capabilities: str = "") -> str:
        """
        Create a prompt for planning robot actions based on visual input.

        Args:
            instruction: Natural language instruction
            environment_context: Description of the environment
            robot_capabilities: Description of robot capabilities

        Returns:
            Formatted prompt string
        """
        base_prompt = f"""
        Given the image and the following instruction: "{instruction}"

        Plan the appropriate robot actions considering:
        1. The current environment visible in the image
        2. The robot's capabilities: {robot_capabilities}
        3. Safety considerations
        4. Most efficient path to accomplish the goal

        Provide your action plan in JSON format:
        {{
            "primary_action": "main action to take",
            "action_sequence": [
                {{
                    "action": "specific action",
                    "parameters": {{"param1": "value1"}},
                    "reasoning": "why this action is appropriate"
                }}
            ],
            "safety_considerations": ["consideration1", "consideration2"],
            "estimated_completion_time": "time in seconds",
            "confidence": 0.0-1.0
        }}
        """

        if environment_context:
            return f"{base_prompt}\n\nEnvironment context: {environment_context}"
        return base_prompt


# Example usage of visual prompting
def example_visual_prompting():
    """
    Example of using visual prompting techniques.
    """
    prompter = VisualPrompter()

    # Example 1: Path clearance check
    path_prompt = prompter.create_path_clear_prompt(
        "The robot needs to navigate to the kitchen for a person in a wheelchair."
    )
    print("Path clearance prompt:")
    print(path_prompt)
    print("\n" + "="*50 + "\n")

    # Example 2: Object identification
    object_prompt = prompter.create_object_identification_prompt(
        "red coffee mug",
        "The user mentioned their favorite red mug is on the kitchen counter."
    )
    print("Object identification prompt:")
    print(object_prompt)
    print("\n" + "="*50 + "\n")

    # Example 3: Action planning
    action_prompt = prompter.create_action_planning_prompt(
        "Bring me the red coffee mug from the kitchen counter",
        "Kitchen with standard counters, cabinets, and appliances visible",
        "Humanoid robot with 12-DOF, capable of navigation, manipulation, and basic interaction"
    )
    print("Action planning prompt:")
    print(action_prompt)
```

## Perception-Action Loops

### Closed-Loop VLA Control

Implementing closed-loop control for continuous VLA operation:

```python
class VLAClosedLoopController:
    """
    Closed-loop controller for VLA-based robot control.
    """

    def __init__(self, vla_engine, robot_controller):
        """
        Initialize the closed-loop VLA controller.

        Args:
            vla_engine: VLA model engine (GPT4VisionVLA or LLaVAVLA)
            robot_controller: Robot control interface
        """
        self.vla_engine = vla_engine
        self.robot_controller = robot_controller

        # State variables
        self.current_task = None
        self.task_progress = 0.0
        self.perception_history = []
        self.action_history = []

        # Control loop parameters
        self.control_frequency = 1.0  # Hz
        self.max_task_duration = 300  # seconds (5 minutes)
        self.safety_timeout = 10.0    # seconds

        # Threading
        self.control_thread = None
        self.is_running = False

    def start_control_loop(self, initial_task: str = ""):
        """
        Start the closed-loop VLA control.

        Args:
            initial_task: Initial task to start with
        """
        if initial_task:
            self.set_task(initial_task)

        self.is_running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.start()

        print("VLA closed-loop control started")

    def stop_control_loop(self):
        """
        Stop the closed-loop VLA control.
        """
        self.is_running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join()

        print("VLA closed-loop control stopped")

    def set_task(self, task_description: str):
        """
        Set a new task for the VLA controller.

        Args:
            task_description: Natural language description of the task
        """
        self.current_task = task_description
        self.task_progress = 0.0
        self.task_start_time = time.time()

        print(f"New task set: {task_description}")

    def _control_loop(self):
        """
        Main control loop for VLA-based robot control.
        """
        while self.is_running:
            start_time = time.time()

            try:
                # 1. Perception: Capture current state
                perception_result = self._perceive_environment()

                # 2. Planning: Use VLA to determine next action
                action_plan = self._plan_next_action(perception_result)

                # 3. Action: Execute the planned action
                action_result = self._execute_action(action_plan)

                # 4. Update state and history
                self._update_state(perception_result, action_plan, action_result)

                # 5. Check for task completion
                if self._check_task_completion():
                    print("Task completed successfully")
                    self.current_task = None

                # 6. Sleep to maintain control frequency
                loop_duration = time.time() - start_time
                sleep_time = max(0, (1.0 / self.control_frequency) - loop_duration)

                if sleep_time > 0:
                    time.sleep(sleep_time)

            except Exception as e:
                print(f"Error in VLA control loop: {e}")
                time.sleep(0.1)  # Brief pause before continuing

    def _perceive_environment(self) -> dict:
        """
        Perceive the current environment state.

        Returns:
            Dictionary with perception results
        """
        # Capture current image
        current_image_path = self.robot_controller.capture_image()

        # Analyze the scene
        scene_analysis = self.vla_engine.analyze_scene(current_image_path)

        # Get robot state
        robot_state = self.robot_controller.get_state()

        # Combine perception results
        perception_result = {
            'timestamp': time.time(),
            'scene_analysis': scene_analysis,
            'robot_state': robot_state,
            'image_path': current_image_path
        }

        # Store in history
        self.perception_history.append(perception_result)

        # Keep only recent history (last 10 perceptions)
        if len(self.perception_history) > 10:
            self.perception_history = self.perception_history[-10:]

        return perception_result

    def _plan_next_action(self, perception_result: dict) -> dict:
        """
        Plan the next action based on perception and current task.

        Args:
            perception_result: Current perception results

        Returns:
            Dictionary with action plan
        """
        if not self.current_task:
            return {"action": "wait", "reason": "No active task"}

        # Check if task duration exceeded
        if time.time() - self.task_start_time > self.max_task_duration:
            return {
                "action": "abort_task",
                "reason": "Task duration exceeded maximum allowed time",
                "task": self.current_task
            }

        # Create prompt for action planning
        prompt = f"""
        Current task: {self.current_task}
        Scene analysis: {json.dumps(perception_result['scene_analysis'])}
        Robot state: {json.dumps(perception_result['robot_state'])}

        Based on the current task, scene analysis, and robot state,
        what is the most appropriate next action for the robot to take?
        """

        # Get action plan from VLA
        action_plan = self.vla_engine.process_vision_language_request(
            perception_result['image_path'],
            prompt
        )

        return action_plan

    def _execute_action(self, action_plan: dict) -> dict:
        """
        Execute the planned action.

        Args:
            action_plan: Action plan to execute

        Returns:
            Dictionary with execution results
        """
        action_type = action_plan.get('action', 'unknown')

        try:
            # Execute the action through robot controller
            execution_result = self.robot_controller.execute_action(action_plan)

            result = {
                'success': True,
                'action_executed': action_type,
                'execution_result': execution_result,
                'timestamp': time.time()
            }

        except Exception as e:
            result = {
                'success': False,
                'action_executed': action_type,
                'error': str(e),
                'timestamp': time.time()
            }

        # Store in action history
        self.action_history.append(result)

        # Keep only recent history (last 10 actions)
        if len(self.action_history) > 10:
            self.action_history = self.action_history[-10:]

        return result

    def _update_state(self, perception_result: dict, action_plan: dict, action_result: dict):
        """
        Update the controller state based on the action cycle.

        Args:
            perception_result: Perception results
            action_plan: Action plan executed
            action_result: Action execution results
        """
        # Update task progress based on action results
        if action_result.get('success', False):
            self.task_progress += 0.1  # Arbitrary progress increment
            self.task_progress = min(self.task_progress, 1.0)  # Cap at 1.0

    def _check_task_completion(self) -> bool:
        """
        Check if the current task is completed.

        Returns:
            True if task is completed, False otherwise
        """
        if not self.current_task:
            return True

        # Simple completion check - in practice, this would be more sophisticated
        # For example, checking if the robot has reached a target location
        # or successfully manipulated an object
        if self.task_progress >= 1.0:
            return True

        # Check if action sequence is complete
        # This would involve more complex logic in a real implementation

        return False


# Example robot controller interface
class MockRobotController:
    """
    Mock robot controller for demonstration purposes.
    """

    def __init__(self):
        """
        Initialize the mock robot controller.
        """
        self.state = {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery_level': 100.0,
            'gripper_status': 'open',
            'current_task': None
        }

    def capture_image(self) -> str:
        """
        Capture an image from the robot's camera.
        In a real implementation, this would interface with the camera.

        Returns:
            Path to the captured image
        """
        # In a real implementation, this would capture from the robot's camera
        # For demo purposes, we'll return a dummy path
        import tempfile
        temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
        temp_file.close()

        # Create a dummy image for demonstration
        import cv2
        import numpy as np
        dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(dummy_img, (100, 100), (300, 300), (255, 0, 0), 2)
        cv2.putText(dummy_img, 'Demo Scene', (150, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imwrite(temp_file.name, dummy_img)

        return temp_file.name

    def get_state(self) -> dict:
        """
        Get the current robot state.

        Returns:
            Dictionary with robot state
        """
        return self.state.copy()

    def execute_action(self, action_plan: dict) -> dict:
        """
        Execute an action plan.

        Args:
            action_plan: Action plan to execute

        Returns:
            Dictionary with execution results
        """
        action_type = action_plan.get('action', 'unknown')

        # Simulate action execution
        import time
        time.sleep(0.5)  # Simulate execution time

        # Update state based on action
        if action_type == 'navigate':
            target_x = action_plan.get('x', 0.0)
            target_y = action_plan.get('y', 0.0)
            self.state['position']['x'] = target_x
            self.state['position']['y'] = target_y
        elif action_type == 'grasp':
            self.state['gripper_status'] = 'closed'
        elif action_type == 'release':
            self.state['gripper_status'] = 'open'

        # Simulate battery drain
        self.state['battery_level'] = max(0, self.state['battery_level'] - 0.1)

        return {
            'action_completed': True,
            'new_state': self.state.copy(),
            'execution_time': 0.5
        }
```

## VLA in Real-World Scenarios

### The "Butler Test" Scenario

Implementing the comprehensive "Go to the kitchen and find the red mug" scenario:

```python
class ButlerTestScenario:
    """
    Implementation of the comprehensive "Butler Test" scenario.
    """

    def __init__(self, vla_engine, robot_controller):
        """
        Initialize the Butler Test scenario.

        Args:
            vla_engine: VLA model engine
            robot_controller: Robot control interface
        """
        self.vla_engine = vla_engine
        self.robot_controller = robot_controller
        self.visual_prompter = VisualPrompter()

    def execute_butler_test(self) -> dict:
        """
        Execute the complete butler test scenario.

        Returns:
            Dictionary with test results
        """
        results = {
            'steps': [],
            'overall_success': False,
            'completion_time': 0.0,
            'errors': []
        }

        start_time = time.time()

        try:
            # Step 1: Navigate to the kitchen
            nav_result = self._navigate_to_kitchen()
            results['steps'].append({'step': 'navigation', 'result': nav_result})

            if not nav_result.get('success', False):
                results['errors'].append('Navigation to kitchen failed')
                return results

            # Step 2: Find the red mug
            find_result = self._find_red_mug()
            results['steps'].append({'step': 'object_finding', 'result': find_result})

            if not find_result.get('success', False):
                results['errors'].append('Finding red mug failed')
                return results

            # Step 3: Approach and pick up the mug (if found)
            if find_result.get('found', False):
                pickup_result = self._pickup_object(find_result)
                results['steps'].append({'step': 'object_pickup', 'result': pickup_result})

                if not pickup_result.get('success', False):
                    results['errors'].append('Picking up mug failed')
                    # Still consider the test partially successful if object was found

            # Calculate completion time
            results['completion_time'] = time.time() - start_time
            results['overall_success'] = len(results['errors']) == 0 or (
                len(results['errors']) == 1 and 'Picking up mug failed' in results['errors']
            )

        except Exception as e:
            results['errors'].append(f'Unexpected error: {str(e)}')

        return results

    def _navigate_to_kitchen(self) -> dict:
        """
        Navigate the robot to the kitchen.

        Returns:
            Dictionary with navigation results
        """
        try:
            # Capture current image to assess surroundings
            current_image = self.robot_controller.capture_image()

            # Create navigation prompt
            nav_prompt = self.visual_prompter.create_action_planning_prompt(
                "Navigate to the kitchen area",
                "Environment with multiple rooms and pathways",
                "Humanoid robot capable of navigation and obstacle avoidance"
            )

            # Get navigation plan from VLA
            nav_plan = self.vla_engine.process_vision_language_request(
                current_image,
                nav_prompt
            )

            # Execute navigation
            if 'action_sequence' in nav_plan:
                for action in nav_plan['action_sequence']:
                    if action['action'] == 'navigate':
                        execution_result = self.robot_controller.execute_action(action)

                        if not execution_result.get('action_completed', False):
                            return {
                                'success': False,
                                'error': execution_result.get('error', 'Navigation failed'),
                                'plan': nav_plan
                            }

            return {
                'success': True,
                'plan_executed': nav_plan,
                'final_position': self.robot_controller.get_state()['position']
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

    def _find_red_mug(self) -> dict:
        """
        Find the red mug in the kitchen.

        Returns:
            Dictionary with object finding results
        """
        try:
            # Capture image in the kitchen
            kitchen_image = self.robot_controller.capture_image()

            # Create object identification prompt
            object_prompt = self.visual_prompter.create_object_identification_prompt(
                "red mug",
                "Look specifically on counters, tables, or other common places for mugs"
            )

            # Identify the red mug using VLA
            identification_result = self.vla_engine.process_vision_language_request(
                kitchen_image,
                object_prompt
            )

            # Check if mug was found
            found = identification_result.get('found', False)

            return {
                'success': True,
                'found': found,
                'object_details': identification_result if found else {},
                'confidence': identification_result.get('confidence', 0.0)
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

    def _pickup_object(self, object_info: dict) -> dict:
        """
        Pick up the identified object.

        Args:
            object_info: Information about the object to pick up

        Returns:
            Dictionary with pickup results
        """
        try:
            # Approach the object
            position = object_info.get('position', {})
            approach_action = {
                'action': 'navigate',
                'x': position.get('x', 0.0),
                'y': position.get('y', 0.0),
                'theta': 0.0
            }

            approach_result = self.robot_controller.execute_action(approach_action)
            if not approach_result.get('action_completed', False):
                return {
                    'success': False,
                    'error': 'Failed to approach object',
                    'approach_result': approach_result
                }

            # Grasp the object
            grasp_action = {
                'action': 'grasp',
                'object_info': object_info
            }

            grasp_result = self.robot_controller.execute_action(grasp_action)

            return {
                'success': grasp_result.get('action_completed', False),
                'grasp_result': grasp_result
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }


def run_butler_test_demo():
    """
    Demo function to run the butler test scenario.
    """
    # Initialize components (in a real implementation, these would be real instances)
    mock_robot = MockRobotController()

    # For demo purposes, we'll use a mock VLA engine
    class MockVLAEngine:
        def process_vision_language_request(self, image_path, prompt):
            # Mock response for demo
            if "navigate to the kitchen" in prompt.lower():
                return {
                    "action_sequence": [{"action": "navigate", "x": 2.0, "y": 3.0, "theta": 0.0}]
                }
            elif "red mug" in prompt.lower():
                return {
                    "found": True,
                    "position": {"x": 0.5, "y": 0.3, "distance": 1.2},
                    "confidence": 0.85
                }
            else:
                return {"action": "unknown", "confidence": 0.5}

    vla_engine = MockVLAEngine()

    # Initialize the butler test scenario
    butler_test = ButlerTestScenario(vla_engine, mock_robot)

    # Run the test
    results = butler_test.execute_butler_test()

    print("Butler Test Results:")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    run_butler_test_demo()
```

## Performance Optimization

### Optimizing VLA for Real-Time Operation

```python
class OptimizedVLAController:
    """
    Optimized VLA controller for real-time operation.
    """

    def __init__(self, vla_engine, robot_controller):
        """
        Initialize the optimized VLA controller.

        Args:
            vla_engine: VLA model engine
            robot_controller: Robot control interface
        """
        self.vla_engine = vla_engine
        self.robot_controller = robot_controller

        # Caching for repeated operations
        self.scene_cache = {}
        self.action_cache = {}

        # Threading for non-blocking operation
        self.processing_pool = ThreadPoolExecutor(max_workers=2)

        # Performance monitoring
        self.metrics = {
            'total_requests': 0,
            'avg_response_time': 0.0,
            'cache_hits': 0,
            'errors': 0
        }

    def process_request_with_cache(self, image_path: str, prompt: str) -> dict:
        """
        Process a request with caching for improved performance.

        Args:
            image_path: Path to the input image
            prompt: Natural language prompt

        Returns:
            Dictionary with processing results
        """
        import hashlib

        # Create cache key
        cache_key = hashlib.md5(f"{image_path}_{prompt}".encode()).hexdigest()

        # Check cache first
        if cache_key in self.scene_cache:
            self.metrics['cache_hits'] += 1
            return self.scene_cache[cache_key]

        # Process with VLA engine
        start_time = time.time()
        try:
            result = self.vla_engine.process_vision_language_request(image_path, prompt)

            # Cache the result (only for certain types of queries)
            if self._is_cacheable_request(prompt):
                self.scene_cache[cache_key] = result

            # Update metrics
            response_time = time.time() - start_time
            self.metrics['total_requests'] += 1
            # Update average response time
            total_time = self.metrics['avg_response_time'] * (self.metrics['total_requests'] - 1)
            self.metrics['avg_response_time'] = (total_time + response_time) / self.metrics['total_requests']

            return result

        except Exception as e:
            self.metrics['errors'] += 1
            raise e

    def _is_cacheable_request(self, prompt: str) -> bool:
        """
        Determine if a request is suitable for caching.

        Args:
            prompt: The natural language prompt

        Returns:
            True if the request is cacheable, False otherwise
        """
        # Don't cache requests that are time-sensitive or highly variable
        non_cacheable_keywords = [
            'current', 'now', 'immediate', 'urgent', 'right now',
            'where am i', 'what time', 'locate myself'
        ]

        prompt_lower = prompt.lower()
        for keyword in non_cacheable_keywords:
            if keyword in prompt_lower:
                return False

        return True

    def process_parallel_requests(self, requests: list) -> list:
        """
        Process multiple requests in parallel.

        Args:
            requests: List of (image_path, prompt) tuples

        Returns:
            List of results
        """
        futures = []
        for image_path, prompt in requests:
            future = self.processing_pool.submit(
                self.process_request_with_cache, image_path, prompt
            )
            futures.append(future)

        # Collect results
        results = []
        for future in futures:
            try:
                result = future.result(timeout=10.0)  # 10-second timeout
                results.append(result)
            except TimeoutError:
                results.append({"error": "Request timed out"})
            except Exception as e:
                results.append({"error": str(e)})

        return results
```

## Summary

In this chapter, we've explored Vision-Language-Action (VLA) models and their integration with humanoid robotics. You've learned:

- The architecture and capabilities of VLA models
- How to integrate cloud-based (GPT-4 Vision) and local (LLaVA) VLA solutions
- Techniques for creating effective visual prompts
- Implementation of closed-loop perception-action systems
- Application to complex scenarios like the "Butler Test"

VLA models represent a significant advancement in embodied AI, allowing robots to understand and interact with their environment in more natural and flexible ways. Proper implementation of VLA systems can greatly enhance a robot's ability to perform complex tasks in unstructured environments.

The key to successful VLA integration lies in balancing the sophistication of the vision-language understanding with the practical constraints of real-time robot control, ensuring both safety and efficiency in robot behavior.