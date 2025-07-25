"""
Dynamics models for MPPI controller
"""
import torch
import numpy as np


class DifferentialDriveDynamics:
    """Differential drive robot dynamics model"""
    
    def __init__(self, dt=0.1, device='cpu'):
        """
        Initialize differential drive dynamics
        
        Args:
            dt (float): Time step size
            device (str): PyTorch device ('cpu' or 'cuda')
        """
        self.dt = dt
        self.device = device
    
    def __call__(self, state, action):
        """
        Predict next state given current state and action
        
        Args:
            state (torch.Tensor): Current state [x, y, theta] (K x 3)
            action (torch.Tensor): Control action [v, w] (K x 2)
            
        Returns:
            torch.Tensor: Next state [x, y, theta] (K x 3)
        """
        # Extract state components
        x = state[:, 0]
        y = state[:, 1]
        theta = state[:, 2]
        
        # Extract control components
        v = action[:, 0]  # linear velocity
        w = action[:, 1]  # angular velocity
        
        # Forward dynamics using Euler integration
        next_x = x + v * torch.cos(theta) * self.dt
        next_y = y + v * torch.sin(theta) * self.dt
        next_theta = theta + w * self.dt
        
        # Normalize angle to [-pi, pi]
        next_theta = torch.atan2(torch.sin(next_theta), torch.cos(next_theta))
        
        # Stack and return next state
        next_state = torch.stack([next_x, next_y, next_theta], dim=1)
        return next_state.to(self.device)


class AckermannDynamics:
    """Ackermann steering model (bicycle model)"""
    
    def __init__(self, wheelbase=2.7, dt=0.1, device='cpu'):
        """
        Initialize Ackermann dynamics
        
        Args:
            wheelbase (float): Distance between front and rear axles (m)
            dt (float): Time step size
            device (str): PyTorch device ('cpu' or 'cuda')
        """
        self.wheelbase = wheelbase
        self.dt = dt
        self.device = device
    
    def __call__(self, state, action):
        """
        Predict next state using bicycle model
        
        Args:
            state (torch.Tensor): Current state [x, y, theta] (K x 3)
            action (torch.Tensor): Control action [v, delta] (K x 2)
                                  v: forward velocity, delta: steering angle
            
        Returns:
            torch.Tensor: Next state [x, y, theta] (K x 3)
        """
        # Extract state components
        x = state[:, 0]
        y = state[:, 1]
        theta = state[:, 2]
        
        # Extract control components
        v = action[:, 0]      # forward velocity
        delta = action[:, 1]  # steering angle
        
        # Bicycle model dynamics
        next_x = x + v * torch.cos(theta) * self.dt
        next_y = y + v * torch.sin(theta) * self.dt
        next_theta = theta + (v / self.wheelbase) * torch.tan(delta) * self.dt
        
        # Normalize angle
        next_theta = torch.atan2(torch.sin(next_theta), torch.cos(next_theta))
        
        # Stack and return next state
        next_state = torch.stack([next_x, next_y, next_theta], dim=1)
        return next_state.to(self.device)