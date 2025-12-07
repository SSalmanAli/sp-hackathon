---
title: "Lesson 3.3: Nav2 Autonomous Navigation"
sidebar_position: 3
description: "Implementing autonomous navigation using the Navigation2 stack"
---

# Lesson 3.3: Nav2 Autonomous Navigation

## Context
This lesson focuses on implementing autonomous navigation systems using the Navigation2 (Nav2) stack, which provides a complete framework for robot navigation including path planning, obstacle avoidance, and localization. Nav2 is essential for creating robots that can autonomously navigate in dynamic environments.

## What You'll Build
In this lesson, you'll create a complete navigation system with path planning and obstacle avoidance. You'll configure the Navigation2 stack with appropriate planners, controllers, and safety mechanisms to enable autonomous robot navigation.

## Required Tools
- Nav2 (Navigation2 stack)
- ROS 2 (Humble Hawksbill)
- costmap_2d
- planner plugins (Navfn, GlobalPlanner, etc.)
- controller plugins (DWB, TEB, etc.)
- rviz2 with Nav2 plugins
- robot with proper odometry and sensor setup

## Important Concepts

### Costmaps
2D or 3D representations of the environment that indicate where the robot can and cannot traverse.

### Global Planner
Algorithm that computes a path from the robot's current location to the goal, considering static obstacles.

### Local Planner
Algorithm that executes the global plan while avoiding dynamic obstacles and following the path.

### Behavior Trees
Hierarchical task planning structures that control the navigation system's decision-making process.

## Implementation Steps

### Step 1: Configure Navigation System
1. Set up costmap parameters for obstacle detection
2. Configure global and local planners
3. Define robot footprint and safety parameters

### Step 2: Integrate with Robot System
1. Connect odometry and sensor data to Nav2
2. Verify proper TF tree setup
3. Test basic navigation capabilities

### Step 3: Configure Behavior Trees
1. Set up behavior tree for navigation tasks
2. Configure recovery behaviors
3. Test navigation in simulation

### Step 4: Test and Optimize
1. Run navigation tests in various scenarios
2. Tune parameters for optimal performance
3. Validate safety and reliability

## Code Example: Nav2 Configuration File (nav2_params.yaml)

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # DWB parameters
    FollowPath:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      speed_limit: 0.4
      subsample_trajectory: false
      min_vel_x: 0.0
      max_vel_x: 0.5
      min_vel_y: -0.5
      max_vel_y: 0.5
      min_vel_theta: -1.0
      max_vel_theta: 1.0
      acc_lim_x: 2.5
      acc_lim_y: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: -2.5
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true
      restore_defaults: false
      publish_cost_grid_pc: false
      global_frame: odom
      robot_base_frame: base_link
      resolution: 0.05
      inflation_radius: 0.55
      cost_scaling_factor: 10.0
      inflation_model: "kinematic"

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      w_smooth: 0.9
      w_data: 0.1

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait", "assisted_teleop", "drive_on_heading"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1.0
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
      min_vel_theta: 0.4
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
      min_duration: 0.5
      max_duration: 10.0
      min_distance: 0.1
      max_distance: 3.0
      min_angle: 0.2
      max_angle: 3.14

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

## Code Example: Navigation Control Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import time

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Create publisher for robot status
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Wait for action server
        self.nav_to_pose_client.wait_for_server()

        self.get_logger().info('Nav2 client initialized')

    def send_goal_pose(self, x, y, theta):
        """Send a navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()

        # Set the goal pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion
        import math
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send the goal
        self.get_logger().info(f'Sending navigation goal to ({x}, {y}, {theta})')
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result of the navigation"""
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status: {status}')

        # Publish status
        status_msg = String()
        if status == 3:  # SUCCEEDED
            status_msg.data = 'Navigation succeeded'
        elif status == 4:  # CANCELED
            status_msg.data = 'Navigation canceled'
        else:
            status_msg.data = f'Navigation failed with status {status}'

        self.status_publisher.publish(status_msg)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # This is called periodically during navigation
        self.get_logger().info(f'Navigating... Current pose: {feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f}')

def main(args=None):
    rclpy.init(args=args)

    nav_client = Nav2Client()

    # Example: Navigate to a specific location
    # In a real application, you might receive goals from a higher-level planner
    time.sleep(2)  # Allow time for setup
    nav_client.send_goal_pose(2.0, 2.0, 0.0)  # Navigate to (2, 2) with 0 rotation

    try:
        rclpy.spin(nav_client)
    except KeyboardInterrupt:
        pass
    finally:
        nav_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Example: Navigation Monitoring and Safety Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np

class NavigationMonitor(Node):
    def __init__(self):
        super().__init__('navigation_monitor')

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)

        # Navigation state
        self.current_pose = None
        self.current_velocity = None
        self.scan_data = None

        # Safety parameters
        self.safety_distance = 0.5  # meters
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s

        # Create timer for safety checks
        self.timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('Navigation monitor initialized')

    def odom_callback(self, msg):
        """Update robot pose and velocity"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.scan_data = msg

    def safety_check(self):
        """Perform safety checks and emergency stops if needed"""
        if self.scan_data is None:
            return

        # Check for obstacles in front of the robot
        front_scan = self.scan_data.ranges[len(self.scan_data.ranges)//2 - 30:len(self.scan_data.ranges)//2 + 30]
        min_distance = min([r for r in front_scan if not np.isnan(r) and r > 0], default=float('inf'))

        if min_distance < self.safety_distance:
            # Emergency stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            # Publish emergency stop flag
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

            self.get_logger().warn(f'EMERGENCY STOP: Obstacle at {min_distance:.2f}m')
        else:
            # Publish emergency stop reset
            emergency_msg = Bool()
            emergency_msg.data = False
            self.emergency_stop_pub.publish(emergency_msg)

    def limit_velocity(self, cmd_vel):
        """Limit velocity commands to safe values"""
        cmd_vel.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, cmd_vel.linear.x))
        cmd_vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd_vel.angular.z))
        return cmd_vel

def main(args=None):
    rclpy.init(args=args)

    monitor = NavigationMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. **Advanced Path Planning**: Implement a custom path planner that considers dynamic obstacles and changing environments
2. **Multi-Robot Navigation**: Extend the system to handle coordination between multiple robots navigating in the same space
3. **Navigation Recovery**: Implement additional recovery behaviors for challenging navigation scenarios

## Troubleshooting Tips

- Ensure proper TF tree setup between all coordinate frames
- Verify that sensor data (especially LIDAR) is correctly published and transformed
- Check that robot footprint is properly configured in costmap parameters
- Monitor CPU and memory usage as navigation can be computationally intensive
- Use RViz to visualize costmaps and path planning for debugging

## Learning Outcomes
By completing this lesson, you will understand:
- How to configure and use the Navigation2 stack for autonomous navigation
- How to set up costmaps for obstacle detection and avoidance
- How to implement safety mechanisms for reliable navigation
- How to monitor and debug navigation system performance