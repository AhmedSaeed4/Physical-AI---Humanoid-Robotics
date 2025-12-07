---
sidebar_position: 2
title: "Chapter 8: Navigation"
---

# Chapter 8: Navigation

## Overview

This chapter covers navigation systems for humanoid robots, focusing on path planning, obstacle avoidance, and autonomous navigation in complex environments. We'll explore the ROS 2 Navigation2 stack and its application to humanoid robotics, including specialized considerations for bipedal locomotion.

## Learning Objectives

After completing this chapter, you will be able to:
- Configure and tune the Navigation2 stack for humanoid robots
- Implement path planning algorithms suitable for legged robots
- Handle dynamic obstacle avoidance for walking robots
- Integrate perception data with navigation systems
- Configure navigation parameters for stable bipedal locomotion
- Implement recovery behaviors for challenging navigation scenarios

## Navigation System Architecture

### Navigation2 Stack Components

The Navigation2 stack consists of several key components:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      NAVIGATION2 STACK                                │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────┐ │
│  │   MAP SERVER    │  │ LOCALIZATION    │  │  BEHAVIOR TREE         │ │
│  │                 │  │   (AMCL)        │  │                        │ │
│  │ • Static Map    │  │ • Particle      │  │ • Task orchestration   │ │
│  │ • Occupancy     │  │   filter        │  │ • Recovery behaviors   │ │
│  │   grid          │  │ • Pose          │  │ • Dynamic reconfiguration│ │
│  │ • Semantic      │  │   estimation    │  │                        │ │
│  │   annotations   │  │                 │  │ • Plugin interface     │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────────────┘ │
│                              │                           │              │
│                              ▼                           ▼              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────┐ │
│  │ GLOBAL PLANNER  │  │  CONTROLLER     │  │  RECOVERY ACTIONS      │ │
│  │                 │  │                 │  │                        │ │
│  │ • A* / Dijkstra │  │ • DWB / MPC     │  │ • Clear costmap        │ │
│  │ • Global path   │  │ • Trajectory    │  │ • Rotate recovery      │ │
│  │   generation    │  │   optimization  │  │ • Move base recovery   │ │
│  │ • Topological   │  │ • Velocity      │  │ • Wall following       │ │
│  │   planning      │  │   control       │  │                        │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────────────┘ │
│                              │                           │              │
│                              ▼                           ▼              │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │                        NAVIGATION CLIENT                          │ │
│  │              (Interfaces with robot platform)                     │ │
│  └─────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
```

### Humanoid-Specific Navigation Considerations

Unlike wheeled robots, humanoid robots have unique navigation requirements:

1. **Footstep Planning**: Need to plan where to place each foot
2. **Balance Preservation**: Must maintain balance during locomotion
3. **Terrain Adaptation**: Handle uneven surfaces and obstacles differently
4. **Dynamic Stability**: Consider COM (Center of Mass) during movement
5. **Gait Generation**: Generate appropriate walking patterns

## Navigation Configuration for Humanoid Robots

### Costmap Configuration

Humanoid robots require specialized costmap parameters:

```yaml
# config/nav2_params_humanoid.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.5
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "behavior_trees/navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "behavior_trees/navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
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
      - nav2_globally_consistent_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
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
      - nav2_backup_cancel_bt_node
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
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller configuration
    FollowPath:
      plugin: "nav2_mppi_controller::MPPICtrl"
      debug_visualizations: false
      # Footstep planning parameters
      max_linear_speed: 0.5  # Slower for stability
      min_linear_speed: 0.1
      max_angular_speed: 0.6
      min_angular_speed: 0.1
      # Balance preservation
      k_path: 2.0
      k_att: 1.5
      lookahead_dist: 0.6
      lookahead_time: 1.5
      # Humanoid-specific parameters
      step_size: 0.3  # Typical humanoid step size
      max_step_up: 0.1  # Max step up height
      max_step_down: 0.15  # Max step down height

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # Higher resolution for precise footstep planning
      robot_radius: 0.4  # Larger for humanoid safety
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
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: "scan"
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.4
      resolution: 0.05  # Higher resolution for humanoid navigation
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: "scan"
        scan:
          topic: "/scan"
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
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 2.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      step_size: 0.025  # Finer resolution for humanoid
      min_distance_to_path: 0.3  # Maintain distance for balance
      max_iterations: 50000

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait", "assisted_teleop", "drive_on_heading"]
    spin:
      plugin: "nav2_behaviors/Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors/BackUp"
      backup_dist: 0.15  # Shorter backup for humanoid stability
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors/Wait"
      wait_duration: 1.0
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
      min_linear_speed: 0.025
      max_linear_speed: 0.3
      min_angular_speed: 0.025
      max_angular_speed: 0.3
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
      goal_dist_tolerance: 0.25
      min_linear_speed: 0.025
      max_linear_speed: 0.25
      min_angular_speed: 0.025
      max_angular_speed: 0.25

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      wait_time: 1s