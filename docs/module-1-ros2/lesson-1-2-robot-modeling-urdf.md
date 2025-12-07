---
title: "Lesson 1.2: Robot Modeling with URDF"
sidebar_position: 2
description: "Creating robot models using Unified Robot Description Format"
---

# Lesson 1.2: Robot Modeling with URDF

## Context
This lesson covers the fundamentals of robot modeling using URDF (Unified Robot Description Format), which is essential for defining robot geometry, kinematics, and physical properties. Understanding URDF is crucial for simulation, visualization, and control of robotic systems.

## What You'll Build
In this lesson, you'll create a complete URDF model of a simple wheeled robot with proper geometry, joints, and physical properties. This model will be used in later lessons for simulation and visualization.

## Required Tools
- ROS 2 (Humble Hawksbill)
- XML editor
- robot_state_publisher
- joint_state_publisher

## Important Concepts

### URDF (Unified Robot Description Format)
URDF is an XML format for representing a robot model. It describes the robot's physical and kinematic properties.

### Links
Links represent rigid bodies in the robot. Each link has properties like mass, inertia, visual, and collision elements.

### Joints
Joints define the relationship between two links, specifying the allowed motion between them (fixed, continuous, revolute, prismatic, etc.).

### Transmissions
Transmissions define how actuators interact with joints, including physical properties like reduction ratios.

## Implementation Steps

### Step 1: Create the Base Link
1. Define the robot's base link with proper mass and inertia properties
2. Add visual and collision elements for the base
3. Verify the base link definition is valid

### Step 2: Add Wheel Links
1. Create separate links for each wheel
2. Define appropriate geometry and material properties
3. Position wheels relative to the base link

### Step 3: Define Joints
1. Create joints connecting wheels to the base
2. Set appropriate joint types (continuous for wheels)
3. Define joint limits and safety controllers

### Step 4: Test the Model
1. Launch the robot model in RViz
2. Verify all links and joints are displayed correctly
3. Check for any errors in the URDF definition

## Code Example: Simple Wheeled Robot URDF

```xml
<?xml version="1.0"?>
<robot name="simple_wheeled_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Front wheel -->
  <link name="front_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Rear left wheel -->
  <link name="rear_left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Rear right wheel -->
  <link name="rear_right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="front_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_wheel"/>
    <origin xyz="0.15 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel"/>
    <origin xyz="-0.15 0.1 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel"/>
    <origin xyz="-0.15 -0.1 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Code Example: Launch File for Testing

```xml
<launch>
  <param name="robot_description" command="xacro $(find-pkg-share simple_wheeled_robot)/urdf/robot.urdf.xacro" />

  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" value="30"/>
  </node>

  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher">
    <param name="rate" value="30"/>
  </node>

  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share simple_wheeled_robot)/rviz/robot.rviz"/>
</launch>
```

## Exercises

1. **Add a Sensor**: Add a camera or LIDAR sensor to your robot model
2. **Modify Dimensions**: Adjust the robot dimensions and observe how it affects the visualization
3. **Create a More Complex Robot**: Add arms or additional sensors to your robot model

## Troubleshooting Tips

- Use `check_urdf` command to validate your URDF: `check_urdf /path/to/robot.urdf`
- Ensure all links have proper mass and inertia properties
- Verify joint connections and limits
- Check that all file paths in the URDF are correct
- Use RViz to visualize the robot and identify any issues

## Learning Outcomes
By completing this lesson, you will understand:
- How to define robot geometry using URDF
- How to create links and joints for a robot model
- How to set physical properties like mass and inertia
- How to visualize and test robot models in RViz