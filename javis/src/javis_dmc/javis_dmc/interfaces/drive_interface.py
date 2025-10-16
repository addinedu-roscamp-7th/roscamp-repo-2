"""
Drive Interface for JAVIS DMC.
Handles robot movement and navigation.
"""

from abc import abstractmethod
from typing import Optional
from concurrent.futures import Future
from geometry_msgs.msg import Pose
from .base_interface import BaseInterface


class DriveInterface(BaseInterface):
    """
    Abstract interface for robot drive control (DDC communication).
    
    This interface handles:
    - Navigation to target positions
    - Person following mode
    - Emergency stop functionality
    - Current pose information
    """
    
    @abstractmethod
    def move_to_target(self, pose: Pose, location_name: str = "") -> Future:
        """
        Move robot to target pose.
        
        Args:
            pose: Target pose in map frame
            location_name: Human-readable location name (e.g., "충전소", "반납대")
            
        Returns:
            Future: Action future for tracking progress
        """
        pass
    
    @abstractmethod
    def enable_follow_mode(self, tracking_topic: str, destination: Pose, 
                          follow_distance: float = 1.5) -> bool:
        """
        Enable person following mode.
        
        Args:
            tracking_topic: Topic to receive tracking information
            destination: Final destination pose
            follow_distance: Distance to maintain from person (meters)
            
        Returns:
            bool: True if successfully enabled
        """
        pass
    
    @abstractmethod
    def disable_follow_mode(self) -> bool:
        """
        Disable person following mode.
        
        Returns:
            bool: True if successfully disabled
        """
        pass
    
    @abstractmethod
    def stop(self) -> bool:
        """
        Emergency stop the robot.
        
        Returns:
            bool: True if successfully stopped
        """
        pass
    
    @abstractmethod
    def rotate_in_place(self, angle: float) -> bool:
        """
        Rotate robot in place.
        
        Args:
            angle: Rotation angle in degrees (360 = full rotation)
            
        Returns:
            bool: True if successfully completed
        """
        pass
    
    @abstractmethod
    def get_current_pose(self) -> Optional[Pose]:
        """
        Get current robot pose.
        
        Returns:
            Optional[Pose]: Current pose if available, None otherwise
        """
        pass
