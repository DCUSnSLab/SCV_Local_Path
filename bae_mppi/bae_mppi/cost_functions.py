"""
Cost functions for MPPI controller
"""
import torch
import numpy as np
from typing import List, Tuple


class ObstacleAvoidanceCost:
    """Cost function for obstacle avoidance using laser scan data"""
    
    def __init__(self, safety_radius=0.3, max_range=5.0, device='cpu'):
        """
        Initialize obstacle avoidance cost
        
        Args:
            safety_radius (float): Minimum safe distance from obstacles (m)
            max_range (float): Maximum sensor range to consider (m)
            device (str): PyTorch device
        """
        self.safety_radius = safety_radius
        self.max_range = max_range
        self.device = device
        self.obstacle_points = None
    
    def update_obstacles(self, obstacle_points):
        """
        Update obstacle points from laser scan
        
        Args:
            obstacle_points (torch.Tensor): Obstacle points in robot frame (N x 2)
        """
        if obstacle_points is not None and len(obstacle_points) > 0:
            self.obstacle_points = torch.tensor(obstacle_points, 
                                              dtype=torch.float32, 
                                              device=self.device)
        else:
            self.obstacle_points = None
    
    def __call__(self, state, action):
        """
        Compute obstacle avoidance cost
        
        Args:
            state (torch.Tensor): Robot states [x, y, theta] (K x 3)
            action (torch.Tensor): Control actions [v, w] (K x 2)
            
        Returns:
            torch.Tensor: Obstacle costs (K,) - Note: flattened output
        """
        batch_size = state.shape[0]
        costs = torch.zeros(batch_size, device=self.device)
        
        if self.obstacle_points is None or len(self.obstacle_points) == 0:
            return costs
        
        # Extract position
        robot_pos = state[:, :2]  # (K x 2)
        
        # Compute distance to all obstacles for all states
        # robot_pos: (K x 2), obstacle_points: (N x 2)
        # distances: (K x N)
        distances = torch.cdist(robot_pos, self.obstacle_points)
        
        # Find minimum distance to any obstacle for each state
        min_distances, _ = torch.min(distances, dim=1)  # (K,)
        
        # Apply exponential penalty for close obstacles
        penalty_mask = min_distances < self.safety_radius * 2.0
        costs[penalty_mask] = torch.exp(-min_distances[penalty_mask] / self.safety_radius) * 100.0
        
        # High penalty for collision
        collision_mask = min_distances < self.safety_radius
        costs[collision_mask] = 1000.0
        
        return costs


class GoalTrackingCost:
    """Cost function for tracking a goal position"""
    
    def __init__(self, goal_weight=1.0, angle_weight=0.5, device='cpu'):
        """
        Initialize goal tracking cost
        
        Args:
            goal_weight (float): Weight for position error
            angle_weight (float): Weight for orientation error
            device (str): PyTorch device
        """
        self.goal_weight = goal_weight
        self.angle_weight = angle_weight
        self.device = device
        self.goal_pose = None
    
    def set_goal(self, goal_pose):
        """
        Set goal pose [x, y, theta]
        
        Args:
            goal_pose (list): Goal pose [x, y, theta]
        """
        self.goal_pose = torch.tensor(goal_pose, dtype=torch.float32, device=self.device)
    
    def __call__(self, state, action):
        """
        Compute goal tracking cost
        
        Args:
            state (torch.Tensor): Robot states [x, y, theta] (K x 3)
            action (torch.Tensor): Control actions (K x 2)
            
        Returns:
            torch.Tensor: Goal tracking costs (K,)
        """
        batch_size = state.shape[0]
        costs = torch.zeros(batch_size, device=self.device)
        
        if self.goal_pose is None:
            return costs
        
        # Position error
        pos_error = torch.norm(state[:, :2] - self.goal_pose[:2], dim=1)
        
        # Angle error
        angle_diff = state[:, 2] - self.goal_pose[2]
        angle_error = torch.abs(torch.atan2(torch.sin(angle_diff), torch.cos(angle_diff)))
        
        # Combine costs
        costs = self.goal_weight * pos_error + self.angle_weight * angle_error
        
        return costs


class ControlEffortCost:
    """Cost function for penalizing control effort"""
    
    def __init__(self, linear_weight=0.1, angular_weight=0.1, device='cpu'):
        """
        Initialize control effort cost
        
        Args:
            linear_weight (float): Weight for linear velocity cost
            angular_weight (float): Weight for angular velocity cost
            device (str): PyTorch device
        """
        self.linear_weight = linear_weight
        self.angular_weight = angular_weight
        self.device = device
    
    def __call__(self, state, action):
        """
        Compute control effort cost
        
        Args:
            state (torch.Tensor): Robot states (K x 3)
            action (torch.Tensor): Control actions [v, w] (K x 2)
            
        Returns:
            torch.Tensor: Control effort costs (K,)
        """
        linear_cost = self.linear_weight * torch.abs(action[:, 0])
        angular_cost = self.angular_weight * torch.abs(action[:, 1])
        
        total_cost = linear_cost + angular_cost
        return total_cost


class CombinedCostFunction:
    """Combined cost function that includes all individual costs"""
    
    def __init__(self, device='cpu'):
        """
        Initialize combined cost function
        
        Args:
            device (str): PyTorch device
        """
        self.device = device
        self.obstacle_cost = ObstacleAvoidanceCost(device=device)
        self.goal_cost = GoalTrackingCost(device=device)
        self.control_cost = ControlEffortCost(device=device)
    
    def update_obstacles(self, obstacle_points):
        """Update obstacles for avoidance cost"""
        self.obstacle_cost.update_obstacles(obstacle_points)
    
    def set_goal(self, goal_pose):
        """Set goal for tracking cost"""
        self.goal_cost.set_goal(goal_pose)
    
    def __call__(self, state, action):
        """
        Compute total cost
        
        Args:
            state (torch.Tensor): Robot states (K x 3)
            action (torch.Tensor): Control actions (K x 2)
            
        Returns:
            torch.Tensor: Total costs (K,)
        """
        obstacle_cost = self.obstacle_cost(state, action)
        goal_cost = self.goal_cost(state, action)
        control_cost = self.control_cost(state, action)
        
        total_cost = obstacle_cost + goal_cost + control_cost
        return total_cost