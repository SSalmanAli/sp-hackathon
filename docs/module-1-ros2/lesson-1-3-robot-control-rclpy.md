---
title: "Lesson 1.3: Robot Control with rclpy"
sidebar_position: 3
description: "Developing robot control nodes using the Python client library"
---

# Lesson 1.3: Robot Control with rclpy

## Context
This lesson builds on the previous lessons by implementing real-time robot control systems using the rclpy Python client library. You'll learn how to create teleoperation nodes and implement basic robot control mechanisms that integrate with the robot model you created in Lesson 1.2.

## What You'll Build
In this lesson, you'll create a teleoperation node that allows you to control a simulated robot using keyboard commands. This system will integrate the communication patterns learned in Lesson 1.1 with the robot model from Lesson 1.2.

## Required Tools
- ROS 2 (Humble Hawksbill)
- rclpy (ROS 2 Python client library)
- Python 3.8+
- geometry_msgs (for Twist messages)

## Important Concepts

### Twist Messages
Twist messages contain linear and angular velocities for robot movement (used for differential drive robots).

### Velocity Publishers
Publishers that send velocity commands to robot controllers.

### Teleoperation
Direct control of a robot by a human operator, typically through keyboard, joystick, or other input devices.

### Control Loop
A continuous loop that reads input, processes it, and sends commands to the robot at a consistent rate.

## Implementation Steps

### Step 1: Create the Teleoperation Node
1. Create a ROS 2 node that will handle keyboard input
2. Set up the publisher for velocity commands
3. Implement the main control loop

### Step 2: Handle Keyboard Input
1. Create functions to capture keyboard events
2. Map keys to robot movement commands
3. Implement safety features (stop on key release)

### Step 3: Integrate with Robot Model
1. Connect the teleoperation node to your robot's control interface
2. Test the robot's response to commands
3. Fine-tune control parameters

### Step 4: Test the Complete System
1. Launch the robot model in simulation
2. Launch the teleoperation node
3. Verify the robot responds correctly to commands

## Code Example: Teleoperation Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i': (1, 0, 0, 0),
        'o': (1, 0, 0, -1),
        'j': (0, 0, 0, 1),
        'l': (0, 0, 0, -1),
        'u': (1, 0, 0, 1),
        ',': (-1, 0, 0, 0),
        '.': (-1, 0, 0, 1),
        'm': (-1, 0, 0, -1),
    }

speedBindings = {
        'q': (1.1, 1.1),
        'z': (.9, .9),
        'w': (1.1, 1),
        'x': (.9, 1),
        'e': (1, 1.1),
        'c': (1, .9),
    }

class TeleopRobot(Node):
    def __init__(self):
        super().__init__('teleop_robot')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0

        # Create timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = self.z * self.speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.th * self.turn
        self.pub.publish(twist)


def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    teleop_node = TeleopRobot()

    try:
        print(msg)
        print(f"Current Speed: {teleop_node.speed}, Turn: {teleop_node.turn}")

        while True:
            key = teleop_node.getKey()
            if key in moveBindings.keys():
                teleop_node.x = moveBindings[key][0]
                teleop_node.y = moveBindings[key][1]
                teleop_node.z = moveBindings[key][2]
                teleop_node.th = moveBindings[key][3]
            elif key in speedBindings.keys():
                teleop_node.speed = teleop_node.speed * speedBindings[key][0]
                teleop_node.turn = teleop_node.turn * speedBindings[key][1]

                print(f"Current Speed: {teleop_node.speed}, Turn: {teleop_node.turn}")
                if teleop_node.status == 14:
                    print(msg)
                teleop_node.status = (teleop_node.status + 1) % 15
            elif key == ' ' or key == 'k':
                teleop_node.x = 0.0
                teleop_node.y = 0.0
                teleop_node.z = 0.0
                teleop_node.th = 0.0
            else:
                if (key == '\x03'):
                    break

    except Exception as e:
        print(e)

    finally:
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        teleop_node.pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Code Example: Simple Robot Controller Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')

        # Create subscriber for velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create publisher for robot status
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Initialize robot state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Create timer for robot state updates
        self.timer = self.create_timer(0.1, self.update_robot_state)

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

        self.get_logger().info(f'Received command: linear={self.linear_velocity}, angular={self.angular_velocity}')

    def update_robot_state(self):
        # In a real robot, this would update the actual hardware
        # For simulation, we just publish status
        status_msg = String()
        status_msg.data = f'Linear: {self.linear_velocity}, Angular: {self.angular_velocity}'
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    robot_controller = SimpleRobotController()

    rclpy.spin(robot_controller)

    # Destroy the node explicitly
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Speed Control**: Add additional keyboard controls to adjust the robot's speed in real-time
2. **Safety Features**: Implement a safety feature that stops the robot if no commands are received for a certain time
3. **Alternative Control**: Create a different control scheme (e.g., using mouse or gamepad instead of keyboard)

## Troubleshooting Tips

- Ensure the robot model is loaded before starting the teleoperation node
- Check that the topic names match between publisher and subscriber
- Verify that the robot simulation is properly configured to receive velocity commands
- Use `ros2 topic echo /cmd_vel` to verify commands are being published
- Check that the robot has the correct joint controllers configured

## Learning Outcomes
By completing this lesson, you will understand:
- How to implement robot control systems using rclpy
- How to handle real-time input for robot teleoperation
- How to integrate control systems with robot models
- How to debug robot control issues