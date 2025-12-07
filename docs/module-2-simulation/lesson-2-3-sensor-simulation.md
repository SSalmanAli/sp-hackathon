---
title: "Lesson 2.3: Sensor Simulation and Physics"
sidebar_position: 3
description: "Simulating various sensors (lidar, cameras, IMU) in virtual environments"
---

# Lesson 2.3: Sensor Simulation and Physics

## Context
This lesson focuses on simulating various sensors in virtual environments, which is crucial for developing and testing perception systems in Physical AI. You'll learn how to create realistic sensor models in both Gazebo and Unity that closely match real-world sensor behavior.

## What You'll Build
In this lesson, you'll add multiple sensor types to your robot model, including LIDAR, cameras, and IMU sensors. You'll configure these sensors in both Gazebo and Unity, ensuring they generate realistic data that can be used for perception tasks.

## Required Tools
- Gazebo Garden
- Unity 2022.3+
- ROS 2 (Humble Hawksbill)
- Unity ROS TCP Connector
- sensor_msgs package
- image_transport package

## Important Concepts

### Sensor Noise Models
Mathematical models that simulate the noise and inaccuracies present in real sensors.

### Sensor Fusion
Combining data from multiple sensors to improve perception accuracy and robustness.

### Ray Tracing
A technique used in LIDAR simulation to calculate distances by tracing rays from the sensor.

### Camera Models
Mathematical representations of how cameras capture images, including intrinsic and extrinsic parameters.

## Implementation Steps

### Step 1: Add LIDAR Sensor to Robot
1. Define a LIDAR sensor in your URDF model
2. Configure the sensor parameters (range, resolution, noise)
3. Test the LIDAR in Gazebo simulation

### Step 2: Add Camera Sensor
1. Define a camera sensor in your URDF model
2. Set up camera parameters (resolution, field of view, noise)
3. Verify camera data publishing to ROS topics

### Step 3: Add IMU Sensor
1. Define an IMU sensor in your URDF model
2. Configure IMU parameters (linear acceleration, angular velocity)
3. Test IMU data publishing and accuracy

### Step 4: Unity Sensor Implementation
1. Create Unity implementations for each sensor
2. Simulate sensor data with realistic noise models
3. Publish sensor data to ROS topics

## Code Example: URDF with Multiple Sensors

```xml
<?xml version="1.0"?>
<robot name="robot_with_sensors" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include the base robot -->
  <xacro:include filename="simple_wheeled_robot.urdf.xacro"/>

  <!-- LIDAR sensor -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
      <material name="silver">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Camera sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.05 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- IMU sensor -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins for sensors -->
  <gazebo reference="lidar_link">
    <sensor name="lidar_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/lidar</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor name="camera_sensor" type="camera">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/camera</namespace>
          <remapping>image_raw:=image</remapping>
          <remapping>camera_info:=camera_info</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <image_topic_name>image</image_topic_name>
        <camera_info_topic_name>camera_info</camera_info_topic_name>
        <frame_name>camera_link</frame_name>
        <hack_baseline>0.07</hack_baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
        <frame_name>imu_link</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Code Example: Unity Sensor Simulation Script

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using System.Linq;

public class SensorSimulator : MonoBehaviour
{
    ROSConnection ros;

    // Sensor topics
    string lidarTopic = "scan";
    string cameraTopic = "image_raw";
    string imuTopic = "imu/data";

    // LIDAR parameters
    public int lidarRays = 360;
    public float lidarMinAngle = -Mathf.PI;
    public float lidarMaxAngle = Mathf.PI;
    public float lidarMaxRange = 30.0f;
    public Transform lidarTransform;

    // Camera parameters
    public Camera sensorCamera;
    public int imageWidth = 640;
    public int imageHeight = 480;

    // IMU parameters
    public float imuNoise = 0.01f;
    private Vector3 lastAngularVelocity = Vector3.zero;
    private Vector3 lastLinearAcceleration = Vector3.zero;

    // Publish rate
    public float publishRate = 0.1f;
    private float lastPublishTime = 0.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Register publishers
        ros.RegisterPublisher<LaserScanMsg>(lidarTopic);
        ros.RegisterPublisher<ImageMsg>(cameraTopic);
        ros.RegisterPublisher<ImuMsg>(imuTopic);
    }

    void Update()
    {
        if (Time.time - lastPublishTime > publishRate)
        {
            // Publish all sensor data
            PublishLidarData();
            PublishCameraData();
            PublishImuData();

            lastPublishTime = Time.time;
        }
    }

    void PublishLidarData()
    {
        // Simulate LIDAR rays
        float[] ranges = new float[lidarRays];

        for (int i = 0; i < lidarRays; i++)
        {
            float angle = lidarMinAngle + (lidarMaxAngle - lidarMinAngle) * i / (lidarRays - 1);

            // Create ray direction
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            direction = lidarTransform.TransformDirection(direction);

            // Perform raycast
            RaycastHit hit;
            if (Physics.Raycast(lidarTransform.position, direction, out hit, lidarMaxRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = lidarMaxRange;
            }

            // Add noise to the measurement
            ranges[i] += Random.Range(-0.01f, 0.01f);
        }

        // Create LaserScan message
        LaserScanMsg scanMsg = new LaserScanMsg();
        scanMsg.header = new HeaderMsg();
        scanMsg.header.stamp = new TimeMsg(0, (uint)(Time.time * 1e9));
        scanMsg.header.frame_id = "lidar_link";

        scanMsg.angle_min = lidarMinAngle;
        scanMsg.angle_max = lidarMaxAngle;
        scanMsg.angle_increment = (lidarMaxAngle - lidarMinAngle) / (lidarRays - 1);
        scanMsg.time_increment = 0.0;
        scanMsg.scan_time = publishRate;
        scanMsg.range_min = 0.1f;
        scanMsg.range_max = lidarMaxRange;

        scanMsg.ranges = ranges.Select(r => (float)r).ToArray();
        scanMsg.intensities = new float[ranges.Length]; // Initialize intensities array

        ros.Publish(lidarTopic, scanMsg);
    }

    void PublishCameraData()
    {
        // Capture image from camera
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = sensorCamera.targetTexture;
        sensorCamera.Render();

        Texture2D imageTex = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        imageTex.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        imageTex.Apply();

        RenderTexture.active = currentRT;

        // Convert texture to bytes
        byte[] imageData = imageTex.EncodeToPNG();
        Destroy(imageTex);

        // Create Image message
        ImageMsg imageMsg = new ImageMsg();
        imageMsg.header = new HeaderMsg();
        imageMsg.header.stamp = new TimeMsg(0, (uint)(Time.time * 1e9));
        imageMsg.header.frame_id = "camera_link";

        imageMsg.height = (uint)imageHeight;
        imageMsg.width = (uint)imageWidth;
        imageMsg.encoding = "rgb8";
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(imageWidth * 3); // 3 bytes per pixel (RGB)
        imageMsg.data = imageData;

        ros.Publish(cameraTopic, imageMsg);
    }

    void PublishImuData()
    {
        // Simulate IMU data with some noise
        Vector3 angularVelocity = transform.angularVelocity + Random.insideUnitSphere * imuNoise;
        Vector3 linearAcceleration = transform.GetComponent<Rigidbody>().velocity / Time.fixedDeltaTime + Random.insideUnitSphere * imuNoise;

        // Create IMU message
        ImuMsg imuMsg = new ImuMsg();
        imuMsg.header = new HeaderMsg();
        imuMsg.header.stamp = new TimeMsg(0, (uint)(Time.time * 1e9));
        imuMsg.header.frame_id = "imu_link";

        // For simplicity, we're not simulating orientation
        imuMsg.orientation = new QuaternionMsg(0, 0, 0, 1);
        imuMsg.orientation_covariance = new double[] { -1, 0, 0, 0, 0, 0, 0, 0, 0 };

        imuMsg.angular_velocity = new Vector3Msg(angularVelocity.x, angularVelocity.y, angularVelocity.z);
        imuMsg.angular_velocity_covariance = new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        imuMsg.linear_acceleration = new Vector3Msg(linearAcceleration.x, linearAcceleration.y, linearAcceleration.z);
        imuMsg.linear_acceleration_covariance = new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        ros.Publish(imuTopic, imuMsg);
    }
}
```

## Code Example: Sensor Data Processing Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Create subscribers for sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)

        self.camera_sub = self.create_subscription(
            Image,
            'image_raw',
            self.camera_callback,
            10)

        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)

        # CV Bridge for image processing
        self.bridge = CvBridge()

        self.get_logger().info('Sensor processor node started')

    def lidar_callback(self, msg):
        # Process LIDAR data
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f'LIDAR: Closest obstacle at {min_distance:.2f}m')

        # Simple obstacle detection
        obstacle_threshold = 1.0  # meters
        obstacles = ranges[ranges < obstacle_threshold]

        if len(obstacles) > 0:
            self.get_logger().info(f'LIDAR: {len(obstacles)} obstacles detected within {obstacle_threshold}m')

    def camera_callback(self, msg):
        # Process camera data
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Simple processing: detect edges using Canny
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Count edges as a simple feature
            edge_count = np.sum(edges > 0)
            self.get_logger().info(f'Camera: {edge_count} edges detected')

            # Optional: Display the image (comment out for headless operation)
            # cv2.imshow('Camera Feed', cv_image)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def imu_callback(self, msg):
        # Process IMU data
        linear_accel = np.array([msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z])
        angular_vel = np.array([msg.angular_velocity.x,
                               msg.angular_velocity.y,
                               msg.angular_velocity.z])

        # Calculate magnitudes
        linear_mag = np.linalg.norm(linear_accel)
        angular_mag = np.linalg.norm(angular_vel)

        self.get_logger().info(f'IMU: Linear acc: {linear_mag:.2f}, Angular vel: {angular_mag:.2f}')


def main(args=None):
    rclpy.init(args=args)

    sensor_processor = SensorProcessor()

    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Multi-Sensor Fusion**: Combine data from LIDAR and camera sensors to create a more robust perception system
2. **Advanced Noise Models**: Implement more sophisticated noise models for sensors based on real sensor specifications
3. **Sensor Calibration**: Simulate the calibration process for your sensors to correct for systematic errors

## Troubleshooting Tips

- Ensure sensor topics are correctly mapped between simulation and ROS
- Check that sensor transforms are properly defined in URDF
- Verify that simulation rates match expected sensor update rates
- Use `ros2 topic echo` to verify sensor data is being published correctly
- Monitor CPU usage as sensor simulation can be computationally intensive

## Learning Outcomes
By completing this lesson, you will understand:
- How to simulate various sensor types in virtual environments
- How to configure realistic noise models for sensors
- How to process and integrate data from multiple sensors
- How to validate sensor simulation against real-world expectations