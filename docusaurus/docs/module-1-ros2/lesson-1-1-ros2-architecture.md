---
title: "Lesson 1.1: ROS 2 Architecture and Communication"
sidebar_position: 1
description: "Understanding ROS 2 nodes, topics, services, and actions"
---

# Lesson 1.1: ROS 2 Architecture and Communication

## Context
This lesson introduces the fundamental concepts of ROS 2 architecture, focusing on the communication patterns that enable distributed robotics systems. Understanding these concepts is crucial for building any Physical AI system that involves multiple components working together.

## What You'll Build
In this lesson, you'll create a simple publisher-subscriber system that demonstrates how robot sensor data flows through a ROS 2 network. This foundational system will serve as the basis for more complex robotic applications.

## Required Tools
- ROS 2 (Humble Hawksbill)
- Python 3.8+
- rclpy (ROS 2 Python client library)

## Important Concepts

### Nodes
Nodes are the fundamental building blocks of a ROS 2 system. Each node performs a specific function and communicates with other nodes through topics, services, or actions.

### Topics
Topics enable asynchronous, many-to-many communication between nodes using a publish-subscribe pattern.

### Services
Services provide synchronous, request-response communication between nodes.

### Actions
Actions enable long-running tasks with feedback, goal, and result messages.

## Implementation Steps

### Step 1: Create a Basic Publisher Node
1. Create a new Python package for your ROS 2 workspace
2. Create a publisher node that sends simple messages
3. Test the publisher to ensure it's working correctly

### Step 2: Create a Basic Subscriber Node
1. Create a subscriber node that receives messages from the publisher
2. Process the received messages and display them
3. Test the subscriber to ensure it's receiving messages

### Step 3: Test the Communication
1. Launch both nodes simultaneously
2. Verify that messages are flowing from publisher to subscriber
3. Experiment with different message rates

## Code Example: Publisher Node

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

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Code Example: Subscriber Node

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

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Modify the Publisher**: Change the message content to include a timestamp
2. **Add Message Counting**: Modify both nodes to track and display the total number of messages sent/received
3. **Custom Message Type**: Create a custom message type instead of using String and implement it

## Troubleshooting Tips

- Ensure ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`
- Check that both nodes are on the same ROS domain ID
- Verify network configuration if running nodes on different machines
- Use `ros2 topic list` to verify topics are being created
- Use `ros2 node list` to verify nodes are running

## Learning Outcomes
By completing this lesson, you will understand:
- The basic architecture of ROS 2 systems
- How nodes communicate through topics
- How to implement publisher and subscriber nodes
- How to test and debug ROS 2 communication