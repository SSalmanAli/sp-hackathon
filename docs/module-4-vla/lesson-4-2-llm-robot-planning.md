---
title: "Lesson 4.2: LLM-Based Robot Planning"
sidebar_position: 2
description: "Using large language models for high-level robot task planning"
---

# Lesson 4.2: LLM-Based Robot Planning

## Context
This lesson explores using Large Language Models (LLMs) for high-level robot task planning. LLMs can interpret natural language commands and generate detailed action sequences, enabling more intuitive human-robot interaction and complex task execution in Physical AI systems.

## What You'll Build
In this lesson, you'll create an LLM-integrated system that generates robot action sequences from natural language commands. You'll learn to structure prompts for effective planning and integrate LLM outputs with robot control systems.

## Required Tools
- LLM API (OpenAI GPT, Anthropic Claude, or local model)
- ROS 2 (Humble Hawksbill)
- actionlib for action execution
- transformers library (for local models)
- prompt engineering tools

## Important Concepts

### Task Decomposition
Breaking down complex tasks into smaller, executable actions that the robot can perform.

### Prompt Engineering
Crafting effective prompts that guide LLMs to produce structured, executable plans.

### Action Primitives
Basic robot capabilities that can be combined to achieve complex tasks.

### Plan Validation
Ensuring generated plans are feasible and safe before execution.

## Implementation Steps

### Step 1: Set Up LLM Integration
1. Configure LLM API access or local model
2. Create prompt templates for task planning
3. Implement safety and validation checks

### Step 2: Define Action Primitives
1. Create a library of basic robot actions
2. Define action parameters and constraints
3. Implement action execution interfaces

### Step 3: Implement Planning Interface
1. Create natural language command processing
2. Generate structured plans from LLM outputs
3. Validate plans before execution

### Step 4: Test and Refine
1. Test with various natural language commands
2. Validate plan safety and feasibility
3. Refine prompt engineering for better results

## Code Example: LLM Planning Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import openai  # or use other LLM APIs
import json
import re
from typing import List, Dict, Any

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')

        # Create subscribers and publishers
        self.command_sub = self.create_subscription(
            String,
            'natural_command',
            self.command_callback,
            10)

        self.plan_pub = self.create_publisher(String, 'generated_plan', 10)
        self.action_pub = self.create_publisher(String, 'robot_action', 10)

        # Initialize LLM client (example with OpenAI)
        # Replace with your actual API key or local model setup
        # openai.api_key = "your-api-key-here"

        # Define available actions
        self.available_actions = {
            'move_to': {
                'description': 'Move robot to a specific location',
                'parameters': ['x', 'y', 'theta']
            },
            'pick_up': {
                'description': 'Pick up an object',
                'parameters': ['object_name', 'position']
            },
            'place': {
                'description': 'Place an object at a location',
                'parameters': ['object_name', 'x', 'y', 'theta']
            },
            'grasp': {
                'description': 'Grasp an object',
                'parameters': ['object_name']
            },
            'release': {
                'description': 'Release a grasped object',
                'parameters': ['object_name']
            },
            'detect': {
                'description': 'Detect objects in the environment',
                'parameters': ['object_type']
            },
            'navigate': {
                'description': 'Navigate to a location with obstacle avoidance',
                'parameters': ['x', 'y']
            }
        }

        # Store current robot state
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'held_object': None,
            'detected_objects': []
        }

        self.get_logger().info('LLM Planning node initialized')

    def command_callback(self, msg):
        """Process natural language command"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Generate plan using LLM
        plan = self.generate_plan(command)

        if plan:
            # Publish the plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            # Execute the plan
            self.execute_plan(plan)
        else:
            self.get_logger().error('Failed to generate plan')

    def generate_plan(self, command: str) -> List[Dict[str, Any]]:
        """Generate a plan using LLM"""
        try:
            # Create a detailed prompt for the LLM
            prompt = self.create_planning_prompt(command)

            # For demonstration, using a mock response
            # In practice, you would call the LLM API here
            mock_plan = self.create_mock_plan(command)

            # Validate the plan
            if self.validate_plan(mock_plan):
                self.get_logger().info(f'Generated valid plan with {len(mock_plan)} steps')
                return mock_plan
            else:
                self.get_logger().error('Generated plan failed validation')
                return []

        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')
            return []

    def create_planning_prompt(self, command: str) -> str:
        """Create a prompt for the LLM to generate a plan"""
        actions_str = "\n".join([
            f"- {action}: {details['description']} (params: {', '.join(details['parameters'])})"
            for action, details in self.available_actions.items()
        ])

        prompt = f"""
        You are a robot task planner. Convert the following natural language command into a sequence of executable robot actions.

        Available actions:
        {actions_str}

        Current robot state:
        - Position: {self.robot_state['position']}
        - Held object: {self.robot_state['held_object']}
        - Detected objects: {self.robot_state['detected_objects']}

        Natural language command: "{command}"

        Respond with a JSON array of actions. Each action should be an object with:
        - "action": the action name
        - "parameters": an object with required parameters

        Example response:
        [
            {{"action": "detect", "parameters": {{"object_type": "red cup"}}}},
            {{"action": "navigate", "parameters": {{"x": 1.0, "y": 2.0}}}},
            {{"action": "grasp", "parameters": {{"object_name": "red cup"}}}}
        ]

        Be specific with coordinates and object names. Ensure actions are feasible and in logical order.
        """

        return prompt

    def create_mock_plan(self, command: str) -> List[Dict[str, Any]]:
        """Create a mock plan based on the command (in practice, this would come from LLM)"""
        command_lower = command.lower()

        plan = []

        if "pick up" in command_lower or "grasp" in command_lower:
            # Extract object name if possible
            import re
            obj_match = re.search(r"pick up|grasp|take (.+?)(?:\s|$)", command_lower)
            obj_name = obj_match.group(1).strip() if obj_match else "object"

            plan = [
                {"action": "detect", "parameters": {"object_type": obj_name}},
                {"action": "navigate", "parameters": {"x": 1.0, "y": 1.0}},
                {"action": "grasp", "parameters": {"object_name": obj_name}}
            ]
        elif "go to" in command_lower or "navigate to" in command_lower:
            # Extract coordinates if possible
            coord_matches = re.findall(r"(\d+\.?\d*)", command_lower)
            if len(coord_matches) >= 2:
                x, y = float(coord_matches[0]), float(coord_matches[1])
                plan = [
                    {"action": "navigate", "parameters": {"x": x, "y": y}}
                ]
            else:
                plan = [
                    {"action": "navigate", "parameters": {"x": 2.0, "y": 2.0}}
                ]
        elif "place" in command_lower or "put down" in command_lower:
            obj_match = re.search(r"place|put down (.+?)(?:\s|$)", command_lower)
            obj_name = obj_match.group(1).strip() if obj_match else "object"

            plan = [
                {"action": "navigate", "parameters": {"x": 3.0, "y": 1.0}},
                {"action": "release", "parameters": {"object_name": obj_name}}
            ]
        else:
            # Default plan for unrecognized commands
            plan = [
                {"action": "detect", "parameters": {"object_type": "any"}},
                {"action": "navigate", "parameters": {"x": 1.0, "y": 1.0}}
            ]

        return plan

    def validate_plan(self, plan: List[Dict[str, Any]]) -> bool:
        """Validate that the plan is feasible"""
        for step in plan:
            action = step.get('action')
            if not action:
                self.get_logger().error('Plan step missing action')
                return False

            if action not in self.available_actions:
                self.get_logger().error(f'Unknown action: {action}')
                return False

            required_params = set(self.available_actions[action]['parameters'])
            provided_params = set(step.get('parameters', {}).keys())

            # Check if all required parameters are provided
            if not required_params.issubset(provided_params):
                missing = required_params - provided_params
                self.get_logger().error(f'Missing parameters for {action}: {missing}')
                return False

        return True

    def execute_plan(self, plan: List[Dict[str, Any]]):
        """Execute the generated plan step by step"""
        self.get_logger().info(f'Executing plan with {len(plan)} steps')

        for i, step in enumerate(plan):
            self.get_logger().info(f'Executing step {i+1}/{len(plan)}: {step["action"]}')

            # Publish the action
            action_msg = String()
            action_msg.data = json.dumps(step)
            self.action_pub.publish(action_msg)

            # Simulate action execution time
            # In a real system, you would wait for action completion
            import time
            time.sleep(1)

        self.get_logger().info('Plan execution completed')

def main(args=None):
    rclpy.init(args=args)

    llm_planner = LLMPlanningNode()

    try:
        rclpy.spin(llm_planner)
    except KeyboardInterrupt:
        pass
    finally:
        llm_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Example: Action Execution Manager

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import time

class ActionExecutionManager(Node):
    def __init__(self):
        super().__init__('action_execution_manager')

        # Create subscribers
        self.action_sub = self.create_subscription(
            String,
            'robot_action',
            self.action_callback,
            10)

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Robot state
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.held_object = None

        self.get_logger().info('Action execution manager initialized')

    def action_callback(self, msg):
        """Process action from planner"""
        try:
            action_data = json.loads(msg.data)
            action = action_data['action']
            params = action_data['parameters']

            self.get_logger().info(f'Executing action: {action} with params: {params}')

            # Execute the action based on type
            if action == 'navigate':
                self.execute_navigate(params)
            elif action == 'move_to':
                self.execute_move_to(params)
            elif action == 'grasp':
                self.execute_grasp(params)
            elif action == 'release':
                self.execute_release(params)
            elif action == 'detect':
                self.execute_detect(params)
            else:
                self.get_logger().warn(f'Unknown action: {action}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding action JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error executing action: {e}')

    def execute_navigate(self, params):
        """Execute navigation action"""
        try:
            # Wait for navigation server
            self.nav_client.wait_for_server()

            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            goal_msg.pose.pose.position.x = float(params['x'])
            goal_msg.pose.pose.position.y = float(params['y'])
            goal_msg.pose.pose.position.z = 0.0

            # Set orientation (simplified)
            import math
            goal_msg.pose.pose.orientation.z = math.sin(float(params.get('theta', 0.0)) / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(float(params.get('theta', 0.0)) / 2.0)

            # Send goal
            self.get_logger().info(f'Sending navigation goal to ({params["x"]}, {params["y"]})')
            future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)

            # Update robot position
            self.current_position['x'] = float(params['x'])
            self.current_position['y'] = float(params['y'])
            if 'theta' in params:
                self.current_position['theta'] = float(params['theta'])

        except Exception as e:
            self.get_logger().error(f'Navigation failed: {e}')

    def execute_move_to(self, params):
        """Execute move to action (simplified version)"""
        target_x = float(params['x'])
        target_y = float(params['y'])
        target_theta = float(params.get('theta', 0.0))

        # Calculate simple movement commands
        dx = target_x - self.current_position['x']
        dy = target_y - self.current_position['y']

        # Create velocity command
        cmd = Twist()
        cmd.linear.x = min(0.5, max(-0.5, dx * 0.5))  # Proportional control
        cmd.linear.y = min(0.5, max(-0.5, dy * 0.5))
        cmd.angular.z = min(1.0, max(-1.0, (target_theta - self.current_position['theta']) * 0.5))

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Update position after movement
        self.current_position['x'] = target_x
        self.current_position['y'] = target_y
        self.current_position['theta'] = target_theta

        # Stop robot after movement
        time.sleep(2)  # Wait for movement
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        self.get_logger().info(f'Moved to position ({target_x}, {target_y})')

    def execute_grasp(self, params):
        """Execute grasp action (simulated)"""
        object_name = params.get('object_name', 'unknown')
        self.held_object = object_name
        self.get_logger().info(f'Grasped object: {object_name}')

    def execute_release(self, params):
        """Execute release action (simulated)"""
        object_name = params.get('object_name', 'unknown')
        if self.held_object == object_name:
            self.held_object = None
            self.get_logger().info(f'Released object: {object_name}')
        else:
            self.get_logger().warn(f'Tried to release {object_name}, but holding {self.held_object}')

    def execute_detect(self, params):
        """Execute detection action (simulated)"""
        object_type = params.get('object_type', 'any')
        self.get_logger().info(f'Detected objects of type: {object_type}')
        # In a real system, this would interface with perception system

def main(args=None):
    rclpy.init(args=args)

    action_manager = ActionExecutionManager()

    try:
        rclpy.spin(action_manager)
    except KeyboardInterrupt:
        pass
    finally:
        action_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Example: Prompt Engineering Framework

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re
from typing import Dict, List, Any

class PromptEngineeringNode(Node):
    def __init__(self):
        super().__init__('prompt_engineering_node')

        # Create subscribers and publishers
        self.command_sub = self.create_subscription(
            String,
            'high_level_command',
            self.command_callback,
            10)

        self.refined_command_pub = self.create_publisher(String, 'refined_command', 10)
        self.context_pub = self.create_publisher(String, 'context_update', 10)

        # Maintain context for the conversation
        self.context = {
            'environment': 'office',
            'objects': ['table', 'chair', 'cup', 'book'],
            'robot_capabilities': ['navigation', 'manipulation', 'vision'],
            'previous_commands': [],
            'current_task': None
        }

        self.get_logger().info('Prompt engineering node initialized')

    def command_callback(self, msg):
        """Process and refine natural language command"""
        original_command = msg.data
        self.get_logger().info(f'Received command: {original_command}')

        # Refine the command using context
        refined_command = self.refine_command(original_command)

        # Update context
        self.context['previous_commands'].append(original_command)
        self.context['current_task'] = refined_command

        # Publish refined command
        refined_msg = String()
        refined_msg.data = refined_command
        self.refined_command_pub.publish(refined_msg)

        # Publish context update
        context_msg = String()
        context_msg.data = json.dumps(self.context)
        self.context_pub.publish(context_msg)

    def refine_command(self, command: str) -> str:
        """Refine the command using context and prompt engineering"""
        # Apply various refinement techniques

        # 1. Clarify ambiguous references
        command = self.resolve_references(command)

        # 2. Add context where needed
        command = self.add_context(command)

        # 3. Structure the command for better processing
        command = self.structure_command(command)

        self.get_logger().info(f'Refined command: {command}')
        return command

    def resolve_references(self, command: str) -> str:
        """Resolve ambiguous references like 'it', 'that', 'the object'"""
        # This is a simplified example - in practice, you'd have more sophisticated reference resolution
        command_lower = command.lower()

        # Replace ambiguous references with specific objects from context
        if 'it' in command_lower and self.context['objects']:
            # Replace 'it' with the most recently mentioned object
            last_object = self.context['objects'][-1]
            command = re.sub(r'\bit\b', last_object, command, flags=re.IGNORECASE)

        if 'that' in command_lower and self.context['objects']:
            # Replace 'that' with the most recently mentioned object
            last_object = self.context['objects'][-1]
            command = re.sub(r'\bthat\b', last_object, command, flags=re.IGNORECASE)

        return command

    def add_context(self, command: str) -> str:
        """Add environmental and situational context to the command"""
        # Add environment context if relevant
        if 'room' in command.lower() or 'area' in command.lower():
            command += f" in the {self.context['environment']} environment"

        # Add robot capability context if relevant
        if any(cap in command.lower() for cap in self.context['robot_capabilities']):
            command += f" using available capabilities: {', '.join(self.context['robot_capabilities'])}"

        return command

    def structure_command(self, command: str) -> str:
        """Structure the command for better LLM processing"""
        # Add structure to make it clearer for LLM processing
        structured = f"""
        TASK: {command}

        ENVIRONMENT: {self.context['environment']}
        AVAILABLE OBJECTS: {', '.join(self.context['objects'])}
        ROBOT CAPABILITIES: {', '.join(self.context['robot_capabilities'])}

        Please break this task into specific, executable steps.
        """
        return structured.strip()

    def create_effective_prompt(self, command: str) -> str:
        """Create an effective prompt for LLM planning"""
        # Use chain-of-thought prompting
        cot_prompt = f"""
        Task: {command}

        Let's think step by step:
        1. What is the main goal?
        2. What objects are involved?
        3. What actions are needed?
        4. What is the sequence of actions?
        5. Are there any constraints or safety considerations?

        Now provide the plan as a JSON array of actions with parameters.
        Each action should be from this set: navigate, grasp, release, detect, move_to, pick_up, place.
        """

        # Use few-shot prompting with examples
        few_shot_prompt = f"""
        Task: {command}

        Here are examples of how to structure robot plans:

        Example 1:
        Input: "Pick up the red cup and place it on the table"
        Output: [
          {{"action": "detect", "parameters": {{"object_type": "red cup"}}}},
          {{"action": "navigate", "parameters": {{"x": 1.0, "y": 1.0}}}},
          {{"action": "grasp", "parameters": {{"object_name": "red cup"}}}},
          {{"action": "navigate", "parameters": {{"x": 2.0, "y": 2.0}}}},
          {{"action": "place", "parameters": {{"object_name": "red cup", "x": 2.0, "y": 2.0, "theta": 0.0}}}}
        ]

        Example 2:
        Input: "Go to the kitchen"
        Output: [
          {{"action": "navigate", "parameters": {{"x": 5.0, "y": 3.0}}}}
        ]

        Now process the current task:
        """

        return few_shot_prompt

def main(args=None):
    rclpy.init(args=args)

    prompt_engineer = PromptEngineeringNode()

    try:
        rclpy.spin(prompt_engineer)
    except KeyboardInterrupt:
        pass
    finally:
        prompt_engineer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. **Context-Aware Planning**: Implement a system that maintains context across multiple commands and uses it for better planning
2. **Multi-Modal Planning**: Extend the system to incorporate visual information into the planning process
3. **Plan Optimization**: Create a system that optimizes generated plans for efficiency and safety

## Troubleshooting Tips

- Ensure LLM API keys are properly configured and have sufficient quota
- Validate that generated plans are properly formatted and contain required parameters
- Check that action execution interfaces are properly connected to robot systems
- Monitor LLM response times and implement caching for frequently requested plans
- Implement fallback mechanisms when LLM fails to generate valid plans

## Learning Outcomes
By completing this lesson, you will understand:
- How to integrate LLMs with robot planning systems
- How to structure effective prompts for task planning
- How to validate and execute plans generated by LLMs
- How to maintain context and state in LLM-driven systems