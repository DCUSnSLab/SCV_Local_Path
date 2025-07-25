"""
Laser scan processor for obstacle detection
"""
import numpy as np
import torch
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped


class LaserScanProcessor:
    """Process LaserScan messages to extract obstacle points"""
    
    def __init__(self, min_range=0.1, max_range=5.0, angle_increment_filter=1, tf_buffer=None, footprint: np.ndarray | None = None, fp_padding = 0):
        self.min_range = min_range
        self.max_range = max_range
        self.angle_increment_filter = angle_increment_filter
        self.tf_buffer = tf_buffer
        self.footprint = footprint
        self.fp_padding = fp_padding

        if footprint is not None and fp_padding > 0:
            centroid = footprint.mean(axis=0)
            dirs = footprint - centroid
            norms = np.linalg.norm(dirs, axis=1, keepdims=True)
            scale = 1 + (fp_padding / (norms + 1e-6))
            self.padded_footprint = centroid + dirs * scale
        else:
            self.padded_footprint = footprint
    
    def process_scan(self, scan_msg: LaserScan, target_frame='odom'):
        #
        ranges = np.array(scan_msg.ranges)
        
        # 거리 제한
        valid_mask = (ranges >= self.min_range) & (ranges <= self.max_range) & np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]
        valid_indices = np.where(valid_mask)[0]
        
        # 포인트 샘플링
        if self.angle_increment_filter > 1:
            subsample_mask = valid_indices % self.angle_increment_filter == 0
            valid_ranges = valid_ranges[subsample_mask]
            valid_indices = valid_indices[subsample_mask]
        
        if len(valid_ranges) == 0:
            return np.array([]).reshape(0, 2)
        
        # 2d로 변경
        angles = scan_msg.angle_min + valid_indices * scan_msg.angle_increment

        x = valid_ranges * np.cos(angles)
        y = valid_ranges * np.sin(angles)

        obstacle_points = np.column_stack([x, y])
        
        # 변환한 포인트들 tf 변경
        if self.tf_buffer is not None and target_frame != scan_msg.header.frame_id:
            try:
                # Get the transform once (much faster than per-point transforms)
                transform = self.tf_buffer.lookup_transform(
                    target_frame, scan_msg.header.frame_id, 
                    scan_msg.header.stamp, timeout=tf2_ros.Duration(seconds=0.01))
                
                # Extract rotation and translation
                t = transform.transform.translation
                r = transform.transform.rotation
                
                # Convert quaternion to rotation matrix (2D)
                # For 2D case, we only need the z-rotation
                yaw = 2.0 * np.arctan2(r.z, r.w)
                cos_yaw = np.cos(yaw)
                sin_yaw = np.sin(yaw)
                
                # Apply transformation to all points at once (vectorized)
                transformed_x = cos_yaw * obstacle_points[:, 0] - sin_yaw * obstacle_points[:, 1] + t.x
                transformed_y = sin_yaw * obstacle_points[:, 0] + cos_yaw * obstacle_points[:, 1] + t.y
                
                obstacle_points = np.column_stack([transformed_x, transformed_y])
                    
            except Exception as e:
                # If transformation fails, return points in original frame
                pass
        
        return obstacle_points
    
    def filter_by_robot_footprint(self, points):
        if points.size == 0:
            return points

        # Use precomputed padded footprint polygon
        path = Path(self.padded_footprint)
        inside = path.contains_points(points)
        return points[~inside]