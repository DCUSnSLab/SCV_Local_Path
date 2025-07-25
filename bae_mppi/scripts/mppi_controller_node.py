#!/usr/bin/env python3
"""
MPPI Controller Node for ROS2
"""
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import torch
import numpy as np

# ROS2 messages
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

# PyTorch MPPI (copied to local package)
from bae_mppi.pytorch_mppi import MPPI

# Local modules
from bae_mppi.dynamics import DifferentialDriveDynamics
from bae_mppi.cost_functions import CombinedCostFunction
from bae_mppi.laser_processor import LaserScanProcessor
from bae_mppi.visualizer import MPPIVisualizer


class MPPIControllerNode(Node):
    """ROS2 node for MPPI-based local path planning"""
    
    def __init__(self):
        super().__init__('mppi_controller')
        
        # Initialize TF buffer
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Declare parameters
        self.declare_parameter('use_gpu', False)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('horizon_steps', 30)
        self.declare_parameter('num_samples', 1000)
        self.declare_parameter('lambda_', 1.0)
        self.declare_parameter('sigma', [0.5, 1.0])  # [v_sigma, w_sigma]
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('viz_frequency', 2.0)  # Visualization frequency (Hz)
        self.declare_parameter('footprint', [])
        self.declare_parameter('footprint_padding', 0.0)

        # Get parameters
        use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        horizon_steps = self.get_parameter('horizon_steps').get_parameter_value().integer_value
        num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        lambda_ = self.get_parameter('lambda_').get_parameter_value().double_value
        sigma = self.get_parameter('sigma').get_parameter_value().double_array_value
        max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        dt = self.get_parameter('dt').get_parameter_value().double_value
        viz_frequency = self.get_parameter('viz_frequency').get_parameter_value().double_value
        footprint_list = self.get_parameter('footprint').get_parameter_value().double_array_value
        fp_padding = self.get_parameter('footprint_padding').get_parameter_value().double_value
        # Setup device
        self.device = 'cuda' if use_gpu and torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')
        
        
        # Initialize components
        self.dynamics = DifferentialDriveDynamics(dt=dt, device=self.device)
        self.cost_function = CombinedCostFunction(device=self.device)
        self.laser_processor = LaserScanProcessor(tf_buffer=self.tf_buffer, footprint=self.footprint, fp_padding=self.fp_padding)
        self.visualizer = MPPIVisualizer(frame_id='map')
        
        # Store obstacle points for visualization
        self.obstacle_points = None
        
        # State dimensions
        nx = 3  # [x, y, theta]
        nu = 2  # [v, w]
        
        # Control bounds
        u_min = torch.tensor([-max_linear_vel, -max_angular_vel], device=self.device)
        u_max = torch.tensor([max_linear_vel, max_angular_vel], device=self.device)
        
        # Initialize MPPI controller
        # Convert sigma to diagonal covariance matrix
        noise_sigma = torch.diag(torch.tensor(sigma, device=self.device))
        
        self.mppi = MPPI(
            dynamics=self.dynamics,
            running_cost=self.cost_function,
            nx=nx,
            noise_sigma=noise_sigma,
            num_samples=num_samples,
            horizon=horizon_steps,
            lambda_=lambda_,
            device=self.device,
            u_min=u_min,
            u_max=u_max
        )
        
        # Robot state
        self.current_pose = None
        self.current_velocity = None
        self.goal_pose = None
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Sensor QoS (typically BEST_EFFORT for laser scan)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/ackermann_like_controller/cmd_vel', reliable_qos)
        self.trajectory_markers_pub = self.create_publisher(MarkerArray, '/mppi_trajectories', reliable_qos)
        self.optimal_path_pub = self.create_publisher(Marker, '/mppi_optimal_path', reliable_qos)
        self.goal_marker_pub = self.create_publisher(Marker, '/mppi_goal', reliable_qos)
        self.obstacle_markers_pub = self.create_publisher(MarkerArray, '/mppi_obstacles', reliable_qos)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, reliable_qos)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, sensor_qos)  # Use sensor QoS for laser
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, reliable_qos)
        
        # Control timer (high frequency)
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_callback)
        
        # Visualization timer (lower frequency)
        self.viz_timer = self.create_timer(
            1.0 / viz_frequency, self.visualization_callback)
        
        # Visualization data storage
        self.last_trajectories = None
        self.last_costs = None
        self.last_optimal_trajectory = None
        
        self.get_logger().info('MPPI Controller Node initialized')
    
    def odom_callback(self, msg: Odometry):
        """Process odometry messages"""
        pose = msg.pose.pose
        twist = msg.twist.twist
        
        # Extract pose
        x = pose.position.x
        y = pose.position.y
        
        # Convert quaternion to yaw
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        self.current_pose = np.array([x, y, yaw])
        self.current_velocity = np.array([twist.linear.x, twist.angular.z])
    
    def laser_callback(self, msg: LaserScan):
        # scan frame -> map
        obstacle_points = self.laser_processor.process_scan(msg, target_frame='odom')
        
        # footprint 안에 점들 없게함
        if len(obstacle_points) > 0:
            obstacle_points = self.laser_processor.filter_by_robot_footprint(obstacle_points)
        
        self.obstacle_points = obstacle_points
        
        # Update cost function
        self.cost_function.update_obstacles(obstacle_points)
        obstacle_markers = self.visualizer.create_obstacle_markers(obstacle_points)
        self.obstacle_markers_pub.publish(obstacle_markers)
    
    def goal_callback(self, msg: PoseStamped):
        """Process goal pose messages"""
        pose = msg.pose
        x = pose.position.x
        y = pose.position.y
        
        # Convert quaternion to yaw
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        self.goal_pose = [x, y, yaw]
        self.cost_function.set_goal(self.goal_pose)
        
        # Publish goal visualization
        goal_marker = self.visualizer.create_goal_marker(self.goal_pose)
        self.goal_marker_pub.publish(goal_marker)
        
        self.get_logger().info(f'New goal received: {self.goal_pose}')
    
    def control_callback(self):
        """Main control loop - high frequency"""
        if self.current_pose is None or self.goal_pose is None:
            return
        try:
            # Convert state to tensor
            state = torch.tensor(self.current_pose, dtype=torch.float32, device=self.device)
            
            # Compute control command
            action = self.mppi.command(state)
            
            # Store visualization data for later processing
            self._store_visualization_data(state, action)
            
            # Convert to ROS message
            cmd_msg = Twist()
            cmd_msg.linear.x = float(action[0])
            cmd_msg.angular.z = float(action[1])
            
            # Publish command
            self.cmd_vel_pub.publish(cmd_msg)
            
            # Check if goal reached
            goal_distance = np.linalg.norm(self.current_pose[:2] - np.array(self.goal_pose[:2]))
            if goal_distance < 0.2:  # 20cm tolerance
                self.get_logger().info('Goal reached!')
                # Stop the robot
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
        except Exception as e:
            self.get_logger().error(f'Control computation failed: {str(e)}')
            # Publish stop command for safety
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
    
    def visualization_callback(self):
        """Visualization processing - low frequency"""
        if self.last_trajectories is not None and self.last_costs is not None:
            try:
                # Publish visualizations
                self._publish_visualizations(
                    self.last_trajectories, 
                    self.last_costs, 
                    self.last_optimal_trajectory
                )
            except Exception as e:
                self.get_logger().warn(f'Visualization failed: {str(e)}')
    
    def _store_visualization_data(self, state, action):
        """Store data for visualization processing"""
        try:
            # Generate trajectories (only top 30 to save computation)
            trajectories, costs = self._generate_visualization_trajectories_fast(state, num_traj=30)
            optimal_trajectory = self._create_optimal_trajectory(state, action)
            
            # Store for visualization timer
            self.last_trajectories = trajectories
            self.last_costs = costs  
            self.last_optimal_trajectory = optimal_trajectory
            
        except Exception as e:
            self.get_logger().debug(f'Failed to store visualization data: {str(e)}')
    
    def _create_optimal_trajectory(self, state, action):
        """
        Create optimal trajectory by rolling out the current solution
        
        Args:
            state (torch.Tensor): Current state
            action (torch.Tensor): First action from MPPI
            
        Returns:
            torch.Tensor: Optimal trajectory (T x 3)
        """
        try:
            # Get the current optimal action sequence from MPPI
            if hasattr(self.mppi, 'U'):
                optimal_actions = self.mppi.U  # (T x nu)
            else:
                # Fallback: repeat the first action
                optimal_actions = action.unsqueeze(0).repeat(30, 1)
            
            # Roll out the optimal actions
            current_state = state.clone()
            trajectory = [current_state.clone()]
            
            for t in range(min(len(optimal_actions), 30)):  # Limit to horizon
                next_state = self.dynamics(current_state.unsqueeze(0), 
                                         optimal_actions[t].unsqueeze(0))
                current_state = next_state.squeeze(0)
                trajectory.append(current_state.clone())
            
            # Stack to create trajectory tensor
            optimal_trajectory = torch.stack(trajectory)  # (T+1 x 3)
            return optimal_trajectory
            
        except Exception as e:
            self.get_logger().warn(f'Failed to create optimal trajectory: {str(e)}')
            return None
    
    def _publish_visualizations(self, trajectories, costs, optimal_trajectory):
        """
        Publish all visualization markers
        
        Args:
            trajectories (torch.Tensor): All sampled trajectories (K x T x 3)
            costs (torch.Tensor): Costs for each trajectory (K,)
            optimal_trajectory (torch.Tensor): Optimal trajectory (T x 3)
        """
        try:
            # Reset marker ID for each cycle
            self.visualizer.reset_marker_id()
            
            # Publish best trajectories (top 30)
            trajectory_markers = self.visualizer.create_trajectory_markers(
                trajectories, costs, num_best=30)
            self.trajectory_markers_pub.publish(trajectory_markers)
            
            # Publish optimal path
            if optimal_trajectory is not None:
                optimal_marker = self.visualizer.create_optimal_path_marker(optimal_trajectory)
                self.optimal_path_pub.publish(optimal_marker)
                
        except Exception as e:
            self.get_logger().warn(f'Visualization failed: {str(e)}')
    
    def _generate_visualization_trajectories(self, state):
        """
        Generate trajectories for visualization by rolling out perturbed actions
        
        Args:
            state (torch.Tensor): Current state
            
        Returns:
            tuple: (trajectories, costs) both as torch tensors
        """
        try:
            if not hasattr(self.mppi, 'perturbed_action') or self.mppi.perturbed_action is None:
                return None, None
            
            if not hasattr(self.mppi, 'cost_total') or self.mppi.cost_total is None:
                return None, None
            
            perturbed_actions = self.mppi.perturbed_action  # (K x T x nu)
            costs = self.mppi.cost_total  # (K,)
            
            K, T, nu = perturbed_actions.shape
            
            # Roll out each perturbed action sequence
            trajectories = []
            
            for k in range(K):
                # Start from current state
                current_state = state.clone()
                trajectory = [current_state.clone()]
                
                # Roll out the action sequence
                for t in range(T):
                    action = perturbed_actions[k, t]  # (nu,)
                    next_state = self.dynamics(current_state.unsqueeze(0), 
                                             action.unsqueeze(0))
                    current_state = next_state.squeeze(0)
                    trajectory.append(current_state.clone())
                
                # Stack trajectory points
                traj_tensor = torch.stack(trajectory)  # (T+1 x 3)
                trajectories.append(traj_tensor)
            
            # Stack all trajectories
            trajectories = torch.stack(trajectories)  # (K x T+1 x 3)
            
            self.get_logger().debug(f'Generated trajectories shape: {trajectories.shape}, costs shape: {costs.shape}')
            
            return trajectories, costs
            
        except Exception as e:
            self.get_logger().warn(f'Failed to generate visualization trajectories: {str(e)}')
            return None, None
    
    def _generate_visualization_trajectories_fast(self, state, num_traj=30):
        """
        Generate only the best trajectories for visualization (faster)
        
        Args:
            state (torch.Tensor): Current state
            num_traj (int): Number of best trajectories to generate
            
        Returns:
            tuple: (trajectories, costs) for best trajectories only
        """
        try:
            if not hasattr(self.mppi, 'perturbed_action') or self.mppi.perturbed_action is None:
                return None, None
                
            if not hasattr(self.mppi, 'cost_total') or self.mppi.cost_total is None:
                return None, None
            
            perturbed_actions = self.mppi.perturbed_action  # (K x T x nu)
            costs = self.mppi.cost_total  # (K,)
            
            # Find best trajectories first
            best_indices = torch.argsort(costs)[:num_traj]
            best_costs = costs[best_indices]
            
            # Only rollout the best trajectories
            trajectories = []
            K, T, nu = perturbed_actions.shape
            
            for idx in best_indices:
                # Start from current state
                current_state = state.clone()
                trajectory = [current_state.clone()]
                
                # Roll out the action sequence
                for t in range(T):
                    action = perturbed_actions[idx, t]  # (nu,)
                    next_state = self.dynamics(current_state.unsqueeze(0), 
                                             action.unsqueeze(0))
                    current_state = next_state.squeeze(0)
                    trajectory.append(current_state.clone())
                
                # Stack trajectory points
                traj_tensor = torch.stack(trajectory)  # (T+1 x 3)
                trajectories.append(traj_tensor)
            
            # Stack all trajectories
            trajectories = torch.stack(trajectories)  # (num_traj x T+1 x 3)
            
            return trajectories, best_costs
            
        except Exception as e:
            self.get_logger().warn(f'Failed to generate fast visualization trajectories: {str(e)}')
            return None, None


def main(args=None):
    rclpy.init(args=args)
    
    node = MPPIControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()