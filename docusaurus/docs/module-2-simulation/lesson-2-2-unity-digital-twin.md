---
title: "Lesson 2.2: Unity Digital Twin Integration"
sidebar_position: 2
description: "Creating Unity-based digital twins for robot simulation"
---

# Lesson 2.2: Unity Digital Twin Integration

## Context
This lesson introduces Unity as a high-fidelity simulation environment for creating digital twins of Physical AI systems. Unity offers advanced graphics capabilities and realistic physics simulation, making it ideal for testing complex perception systems and creating training data for AI models.

## What You'll Build
In this lesson, you'll create a Unity scene that mirrors your robot model and environment from Gazebo. You'll implement ROS communication between Unity and ROS 2, allowing you to control your robot in Unity and receive sensor data.

## Required Tools
- Unity 2022.3+ LTS
- ROS 2 (Humble Hawksbill)
- Unity ROS TCP Connector
- Robot Operating System (ROS) for Unity package

## Important Concepts

### Digital Twins
A digital twin is a virtual replica of a physical system that can be used for simulation, testing, and training.

### Unity-ROS Bridge
A communication layer that allows Unity to send and receive ROS messages, enabling integration between Unity and ROS-based systems.

### High-Fidelity Simulation
Simulation environments that accurately model real-world physics, lighting, and sensor behavior.

### Synthetic Data Generation
Creating training data for AI models using simulated environments, which can be more cost-effective than collecting real-world data.

## Implementation Steps

### Step 1: Set Up Unity Environment
1. Install Unity Hub and Unity 2022.3+ LTS
2. Install the ROS TCP Connector package
3. Create a new Unity project for your robot simulation

### Step 2: Import Robot Model
1. Convert your URDF robot model to a Unity-compatible format
2. Import the robot model into Unity
3. Set up proper physics colliders and materials

### Step 3: Create ROS Communication
1. Set up TCP connection between Unity and ROS 2
2. Implement publishers and subscribers for robot control
3. Test basic communication between Unity and ROS

### Step 4: Implement Robot Control
1. Create Unity scripts to handle velocity commands
2. Implement robot movement based on ROS messages
3. Test the complete control pipeline

## Code Example: Unity ROS Connection Script

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string robotTopic = "cmd_vel";

    // Robot movement parameters
    public float linearSpeed = 1.0f;
    public float angularSpeed = 1.0f;

    // Current velocity command
    float linearVelocity = 0.0f;
    float angularVelocity = 0.0f;

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();

        // Register the topic with the ROS connection
        ros.RegisterPublisher<TwistMsg>(robotTopic);
    }

    // Update is called once per frame
    void Update()
    {
        // Publish the velocity command to ROS
        TwistMsg velocityCommand = new TwistMsg();
        velocityCommand.linear = new Vector3Msg(linearVelocity, 0, 0);
        velocityCommand.angular = new Vector3Msg(0, 0, angularVelocity);

        ros.Publish(robotTopic, velocityCommand);

        // Update robot position based on current velocities
        UpdateRobotMovement();
    }

    void UpdateRobotMovement()
    {
        // Apply linear and angular velocity to the robot
        transform.Translate(Vector3.forward * linearVelocity * Time.deltaTime);
        transform.Rotate(Vector3.up, angularVelocity * Time.deltaTime);
    }

    // Method to set velocity from external sources
    public void SetVelocity(float linear, float angular)
    {
        linearVelocity = linear;
        angularVelocity = angular;
    }
}
```

## Code Example: Unity Sensor Publisher Script

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class SensorPublisher : MonoBehaviour
{
    ROSConnection ros;
    string sensorTopic = "sensor_data";

    // Sensor data to publish
    public float sensorValue = 0.0f;

    // Publish rate (in seconds)
    public float publishRate = 0.1f;
    private float lastPublishTime = 0.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32Msg>(sensorTopic);
    }

    void Update()
    {
        // Publish sensor data at specified rate
        if (Time.time - lastPublishTime > publishRate)
        {
            Float32Msg sensorMsg = new Float32Msg();
            sensorMsg.data = sensorValue;

            ros.Publish(sensorTopic, sensorMsg);

            lastPublishTime = Time.time;
        }
    }

    // Method to update sensor value
    public void UpdateSensorValue(float value)
    {
        sensorValue = value;
    }
}
```

## Code Example: ROS Node for Unity Communication

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import socket
import json
import threading

class UnityBridgeNode(Node):
    def __init__(self):
        super().__init__('unity_bridge_node')

        # Create subscribers for robot commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Create publishers for sensor data
        self.sensor_pub = self.create_publisher(Float32, 'sensor_data', 10)

        # Unity connection parameters
        self.unity_ip = '127.0.0.1'
        self.unity_port = 10000

        # Connect to Unity
        self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.unity_socket.connect((self.unity_ip, self.unity_port))
            self.get_logger().info(f'Connected to Unity at {self.unity_ip}:{self.unity_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Unity: {e}')

        # Start receiving thread
        self.receive_thread = threading.Thread(target=self.receive_from_unity)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def cmd_vel_callback(self, msg):
        # Send velocity command to Unity
        command = {
            'type': 'velocity_command',
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }

        try:
            self.unity_socket.send(json.dumps(command).encode() + b'\n')
        except Exception as e:
            self.get_logger().error(f'Failed to send to Unity: {e}')

    def receive_from_unity(self):
        # Receive sensor data from Unity
        buffer = ""
        while True:
            try:
                data = self.unity_socket.recv(1024).decode()
                if not data:
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.process_unity_message(line)
            except Exception as e:
                self.get_logger().error(f'Error receiving from Unity: {e}')
                break

    def process_unity_message(self, message_str):
        try:
            message = json.loads(message_str)
            if message['type'] == 'sensor_data':
                sensor_msg = Float32()
                sensor_msg.data = message['value']
                self.sensor_pub.publish(sensor_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing Unity message: {e}')

    def destroy_node(self):
        if self.unity_socket:
            self.unity_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    unity_bridge = UnityBridgeNode()

    try:
        rclpy.spin(unity_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        unity_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Enhanced Sensor Simulation**: Add more complex sensor simulation in Unity (e.g., camera, LIDAR) and publish the data to ROS
2. **Environment Complexity**: Create a more complex Unity environment with multiple obstacles and lighting conditions
3. **Perception Pipeline**: Implement a basic perception pipeline in Unity that detects objects and publishes the information to ROS

## Troubleshooting Tips

- Ensure Unity and ROS 2 are on the same network or localhost
- Check firewall settings that might block TCP connections
- Verify that the ROS TCP Connector is properly configured
- Use Unity's console to check for connection errors
- Monitor ROS topics with `ros2 topic echo` to verify data flow

## Learning Outcomes
By completing this lesson, you will understand:
- How to set up Unity for robotics simulation
- How to create a digital twin of a physical robot
- How to establish communication between Unity and ROS
- How to simulate sensors and control systems in Unity