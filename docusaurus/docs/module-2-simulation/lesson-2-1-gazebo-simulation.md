---
title: "Lesson 2.1: Gazebo Simulation Fundamentals"
sidebar_position: 1
description: "Setting up Gazebo simulation with robot models and physics"
---

# Lesson 2.1: Gazebo Simulation Fundamentals

## Context
This lesson introduces Gazebo simulation, a powerful physics-based simulation environment that allows you to test robot systems in realistic virtual environments. Gazebo is essential for developing and testing Physical AI systems safely before deploying them on real hardware.

## What You'll Build
In this lesson, you'll create a Gazebo world with physics-enabled robot model from Module 1. You'll set up the simulation environment and verify that your robot can move and interact with the virtual world.

## Required Tools
- Gazebo Garden
- ROS 2 (Humble Hawksbill)
- robot_state_publisher
- joint_state_publisher
- gazebo_ros_pkgs

## Important Concepts

### Gazebo Worlds
World files define the environment, including terrain, lighting, and static objects for your simulation.

### Physics Engine
Gazebo uses physics engines (like ODE, Bullet, or DART) to simulate realistic interactions between objects.

### Robot Plugins
Gazebo plugins provide interfaces between the simulation and ROS, allowing you to control your robot and receive sensor data.

### Sensor Simulation
Gazebo can simulate various sensors like cameras, LIDAR, IMU, and more with realistic noise models.

## Implementation Steps

### Step 1: Create a Basic Gazebo World
1. Create a new world file with basic environment
2. Set up lighting and basic physics parameters
3. Test the world in Gazebo

### Step 2: Spawn Your Robot Model
1. Convert your URDF robot model for Gazebo
2. Add necessary plugins for control and sensing
3. Spawn the robot in the simulation

### Step 3: Integrate with ROS
1. Set up ROS-Gazebo bridge
2. Verify that you can control the robot via ROS topics
3. Test sensor data publishing

### Step 4: Test Robot Movement
1. Use the teleoperation node from Module 1
2. Verify the robot moves correctly in simulation
3. Check physics interactions with the environment

## Code Example: Gazebo World File

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add a simple box obstacle -->
    <model name="box">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>1 0 0 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics parameters -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

## Code Example: Robot Model with Gazebo Plugins

```xml
<?xml version="1.0"?>
<robot name="simple_wheeled_robot_gazebo" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include the base URDF -->
  <xacro:include filename="simple_wheeled_robot.urdf.xacro"/>

  <!-- Gazebo-specific configurations -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_source>1</odometry_source>
    </plugin>
  </gazebo>

  <!-- Material definitions for Gazebo -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="front_wheel">
    <material>Gazebo/Black</material>
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>

  <gazebo reference="rear_left_wheel">
    <material>Gazebo/Black</material>
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>

  <gazebo reference="rear_right_wheel">
    <material>Gazebo/Black</material>
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>
</robot>
```

## Code Example: Launch File for Gazebo Simulation

```xml
<launch>
  <!-- Arguments -->
  <arg name="world" default="simple_world"/>
  <arg name="gui" default="true"/>
  <arg name="verbose" default="false"/>
  <arg name="paused" default="false"/>

  <!-- Start Gazebo -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gz_sim.launch.py">
    <arg name="gz_args" value="-r $(find-pkg-share simple_wheeled_robot_gazebo)/worlds/$(var world).world $(var gui):=true $(var verbose):=true $(var paused):=true"/>
  </include>

  <!-- Spawn the robot -->
  <node pkg="ros_gz_sim" exec="create" args="-name simple_wheeled_robot -file $(find-pkg-share simple_wheeled_robot_gazebo)/urdf/robot.gazebo.urdf.xacro"/>

  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(command 'xacro $(find-pkg-share simple_wheeled_robot_gazebo)/urdf/robot.gazebo.urdf.xacro')"/>
  </node>

  <!-- Joint state publisher (for non-actuated joints) -->
  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher">
    <param name="rate" value="30"/>
  </node>
</launch>
```

## Exercises

1. **Add More Obstacles**: Create additional obstacles in your Gazebo world and test robot navigation around them
2. **Different Terrain**: Experiment with different ground types and surfaces to see how they affect robot movement
3. **Sensor Integration**: Add a camera or LIDAR sensor to your robot and visualize the sensor data

## Troubleshooting Tips

- Ensure Gazebo and ROS 2 versions are compatible
- Check that all required packages are installed: `sudo apt install ros-humble-gazebo-ros-pkgs`
- Verify that your robot model has proper inertial properties
- Use `gz topic -l` to list available topics in Gazebo
- Check that joint names in plugins match your URDF joint names

## Learning Outcomes
By completing this lesson, you will understand:
- How to set up Gazebo simulation environments
- How to integrate robot models with Gazebo physics
- How to connect Gazebo with ROS 2 for control and sensing
- How to test robot systems in safe virtual environments