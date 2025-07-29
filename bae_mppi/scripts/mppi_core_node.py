#!/usr/bin/env python3
"""
Core MPPI Controller Node
Pure MPPI computation without sensor processing or visualization
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import torch
import numpy as np
import time

# ROS2 messages
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Header

# Custom messages
from bae_mppi.msg import ProcessedObstacles, MPPIState, OptimalPath, HighCostPath

# Local modules
from bae_mppi_core.pytorch_mppi import MPPI
from bae_mppi_core.dynamics import AckermannDynamics
from bae_mppi_core.cost_functions import CombinedCostFunction


class MPPICoreNode(Node):
    """Core MPPI controller - pure computation"""
    
    def __init__(self):
        super().__init__('mppi_core')
        
        # Parameters
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('horizon_steps', 50)
        self.declare_parameter('num_samples', 5000)
        self.declare_parameter('lambda_', 1.0)
        self.declare_parameter('sigma', [0.8, 0.8])
        self.declare_parameter('max_linear_vel', 1.5)
        self.declare_parameter('max_steering_angle', 0.39)
        self.declare_parameter('wheelbase', 0.65)
        self.declare_parameter('dt', 0.1)
        
        # Visualization parameters
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('enable_path_viz', True)
        self.declare_parameter('enable_best_paths', True)
        self.declare_parameter('num_best_paths', 10)
        
        # Cost function parameters
        self.declare_parameter('obstacle_cost.safety_radius', 0.8)
        self.declare_parameter('obstacle_cost.max_range', 100.0)
        self.declare_parameter('obstacle_cost.penalty_weight', 1000.0)
        self.declare_parameter('obstacle_cost.exponential_factor', 3.0)
        self.declare_parameter('goal_cost.goal_weight', 0.3)
        self.declare_parameter('goal_cost.angle_weight', 0.5)
        
        # Get parameters
        use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        self.enable_path_viz = self.get_parameter('enable_path_viz').get_parameter_value().bool_value
        self.enable_best_paths = self.get_parameter('enable_best_paths').get_parameter_value().bool_value
        self.num_best_paths = self.get_parameter('num_best_paths').get_parameter_value().integer_value
        horizon_steps = self.get_parameter('horizon_steps').get_parameter_value().integer_value
        num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        lambda_ = self.get_parameter('lambda_').get_parameter_value().double_value
        sigma = self.get_parameter('sigma').get_parameter_value().double_array_value
        max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        dt = self.get_parameter('dt').get_parameter_value().double_value
        
        # Setup device
        self.device = 'cuda' if use_gpu and torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')
        
        # Initialize MPPI components
        self.dynamics = AckermannDynamics(wheelbase=wheelbase, dt=dt, device=self.device)
        self.wheelbase = wheelbase
        
        # Cost function with parameters
        obstacle_params = {
            'safety_radius': self.get_parameter('obstacle_cost.safety_radius').get_parameter_value().double_value,
            'max_range': self.get_parameter('obstacle_cost.max_range').get_parameter_value().double_value,
            'penalty_weight': self.get_parameter('obstacle_cost.penalty_weight').get_parameter_value().double_value,
            'exponential_factor': self.get_parameter('obstacle_cost.exponential_factor').get_parameter_value().double_value,
        }
        self.cost_function = CombinedCostFunction(device=self.device, obstacle_params=obstacle_params)
        
        # Set goal cost parameters
        self.cost_function.goal_cost.goal_weight = self.get_parameter('goal_cost.goal_weight').get_parameter_value().double_value
        self.cost_function.goal_cost.angle_weight = self.get_parameter('goal_cost.angle_weight').get_parameter_value().double_value
        
        # Control bounds
        nx = 3  # [x, y, theta]
        nu = 2  # [v, delta]
        u_min = torch.tensor([-max_linear_vel, -max_steering_angle], device=self.device)
        u_max = torch.tensor([max_linear_vel, max_steering_angle], device=self.device)
        
        # Initialize MPPI
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
        
        # State variables
        self.current_state = None
        self.goal_pose = None
        self.latest_obstacles = None
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            MPPIState, 'state', self.state_callback, reliable_qos)
        self.obstacles_sub = self.create_subscription(
            ProcessedObstacles, 'obstacles', self.obstacles_callback, reliable_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, reliable_qos)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/ackermann_like_controller/cmd_vel', reliable_qos)
        self.optimal_path_pub = self.create_publisher(
            OptimalPath, 'optimal_path', reliable_qos)
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_callback)
        
        self.get_logger().info('MPPI Core Node initialized')
    
    def state_callback(self, msg: MPPIState):
        """Receive processed robot state"""
        self.current_state = torch.tensor(msg.state_vector, dtype=torch.float32, device=self.device)
    
    def obstacles_callback(self, msg: ProcessedObstacles):
        """Receive processed obstacle information"""
        self.latest_obstacles = msg
        
        # Update cost function with obstacles using proper method
        if len(msg.ranges) > 0 and self.current_state is not None:
            # Create a mock laser message for update_laser_scan
            class MockLaserMsg:
                def __init__(self, ranges, angles):
                    self.ranges = ranges
                    self.angle_min = angles[0] if len(angles) > 0 else 0.0
                    if len(angles) > 1:
                        self.angle_increment = angles[1] - angles[0]
                    else:
                        self.angle_increment = 0.1  # Default increment
            
            mock_laser = MockLaserMsg(msg.ranges, msg.angles)
            robot_pose = self.current_state.cpu().numpy()
            
            # Use proper update method
            self.cost_function.update_laser_scan(mock_laser, robot_pose)
    
    def goal_callback(self, msg: PoseStamped):
        """Receive goal pose"""
        pose = msg.pose
        x = pose.position.x
        y = pose.position.y
        
        # Convert quaternion to yaw
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        self.goal_pose = [x, y, yaw]
        self.cost_function.set_goal(self.goal_pose)
        
        self.get_logger().info(f'New goal received: {self.goal_pose}')
    
    def control_callback(self):
        """Main control computation"""
        if self.current_state is None or self.goal_pose is None:
            return
        
        start_time = time.time()
        
        try:
            # Compute MPPI control command
            action = self.mppi.command(self.current_state)
            
            # Extract HUNTER tricycle model outputs
            # action[0] = rear_wheel_velocity, action[1] = front_steering_angle
            rear_wheel_velocity = float(action[0])
            front_steering_angle = float(action[1])
            
            # Convert to cmd_vel for HUNTER ackermann_like_controller
            # cmd_vel.linear.x = rear wheel velocity (direct mapping)
            # cmd_vel.angular.z = angular velocity from tricycle kinematics
            if abs(rear_wheel_velocity) > 0.01:
                angular_velocity = (rear_wheel_velocity / self.wheelbase) * torch.tan(action[1])
            else:
                angular_velocity = 0.0
            
            # Publish cmd_vel compatible with HUNTER ackermann_like_controller
            cmd_msg = Twist()
            cmd_msg.linear.x = rear_wheel_velocity
            cmd_msg.angular.z = float(angular_velocity)
            self.cmd_vel_pub.publish(cmd_msg)
            
            # Publish optimal path for visualization (if enabled)
            if self.enable_visualization and self.enable_path_viz:
                self.publish_optimal_path(action)
            
            # Check if goal reached
            goal_distance = torch.norm(self.current_state[:2] - torch.tensor(self.goal_pose[:2], device=self.device))
            if goal_distance < 0.2:
                self.get_logger().info('Goal reached!')
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
            
        except Exception as e:
            self.get_logger().error(f'Control computation failed: {str(e)}')
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        
        end_time = time.time()
        computation_time = (end_time - start_time) * 1000
        if computation_time > 150:  # Log if too slow
            self.get_logger().warn(f'MPPI computation: {computation_time:.1f}ms')
    
    def publish_optimal_path(self, action):
        """Publish optimal path for visualization"""
        try:
            # Create optimal trajectory
            optimal_trajectory = self.create_optimal_trajectory(self.current_state, action)
            
            if optimal_trajectory is not None:
                msg = OptimalPath()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                
                # Convert trajectory to points
                path_points = []
                for i in range(optimal_trajectory.shape[0]):
                    point = Point()
                    point.x = float(optimal_trajectory[i, 0])
                    point.y = float(optimal_trajectory[i, 1])
                    point.z = 0.0
                    path_points.append(point)
                
                msg.path_points = path_points
                msg.current_velocity = float(action[0])  # rear wheel velocity
                msg.current_steering_angle = float(action[1])  # front steering angle
                
                # Add best paths if enabled
                if self.enable_best_paths:
                    msg.high_cost_paths = self.get_best_paths()
                else:
                    msg.high_cost_paths = []
                
                self.optimal_path_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().debug(f'Optimal path publication failed: {str(e)}')
    
    def get_best_paths(self):
        """Extract best paths (lowest cost) from MPPI for visualization"""
        try:
            if (not hasattr(self.mppi, 'cost_total') or self.mppi.cost_total is None or
                not hasattr(self.mppi, 'states') or self.mppi.states is None):
                return []
            
            # Get cost values and sort to find lowest cost trajectories (best paths)
            costs = self.mppi.cost_total.cpu().numpy()
            sorted_indices = torch.argsort(self.mppi.cost_total, descending=False)
            
            # Get top N lowest cost paths (best trajectories)
            num_paths = min(self.num_best_paths, len(sorted_indices))
            best_paths = []
            
            # Use already computed states from MPPI rollouts
            # self.mppi.states shape: [M, K, T, nx] where M=rollout_samples, K=num_samples, T=horizon, nx=state_dim
            if self.mppi.states is not None and len(self.mppi.states.shape) >= 3:
                # If we have rollout samples (M > 1), take the mean across rollouts
                if len(self.mppi.states.shape) == 4:  # [M, K, T, nx]
                    states = self.mppi.states.mean(dim=0)  # [K, T, nx]
                else:  # [K, T, nx]
                    states = self.mppi.states
                
                for i in range(num_paths):
                    idx = sorted_indices[i].item()
                    cost_value = float(costs[idx])
                    
                    # Get the pre-computed trajectory states
                    trajectory_states = states[idx]  # [T, nx]
                    
                    # Convert to ROS message format
                    path_points = []
                    # Add current state as starting point
                    point = Point()
                    point.x = float(self.current_state[0])
                    point.y = float(self.current_state[1])
                    point.z = 0.0
                    path_points.append(point)
                    
                    # Add trajectory points
                    for t in range(min(trajectory_states.shape[0], 30)):
                        point = Point()
                        point.x = float(trajectory_states[t, 0])
                        point.y = float(trajectory_states[t, 1])
                        point.z = 0.0
                        path_points.append(point)
                    
                    # Create HighCostPath message (reusing message name)
                    best_path_msg = HighCostPath()
                    best_path_msg.path_points = path_points
                    best_path_msg.path_cost = cost_value
                    best_paths.append(best_path_msg)
            
            return best_paths
            
        except Exception as e:
            self.get_logger().debug(f'Failed to get best paths: {str(e)}')
            return []
    
    def create_optimal_trajectory(self, state, action):
        """Create optimal trajectory by rolling out current solution"""
        try:
            if hasattr(self.mppi, 'U'):
                optimal_actions = self.mppi.U
            else:
                optimal_actions = action.unsqueeze(0).repeat(30, 1)
            
            current_state = state.clone()
            trajectory = [current_state.clone()]
            
            for t in range(min(len(optimal_actions), 30)):
                next_state = self.dynamics(current_state.unsqueeze(0), 
                                         optimal_actions[t].unsqueeze(0))
                current_state = next_state.squeeze(0)
                trajectory.append(current_state.clone())
            
            return torch.stack(trajectory)
            
        except Exception as e:
            self.get_logger().debug(f'Failed to create optimal trajectory: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    node = MPPICoreNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()