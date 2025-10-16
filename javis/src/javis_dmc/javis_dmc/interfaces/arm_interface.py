"""
Arm Interface for JAVIS DMC.
Handles robot arm control and manipulation tasks.
"""

from abc import abstractmethod
from typing import List
from concurrent.futures import Future
from geometry_msgs.msg import Pose
from .base_interface import BaseInterface


class ArmInterface(BaseInterface):
    """
    Abstract interface for robot arm control (DAC communication).
    
    This interface handles:
    - Book manipulation (pick/place)
    - Trash collection and disposal
    - Carrier management
    - Pose changes
    """
    
    @abstractmethod
    def pick_book(self, book_id: str, book_pose: Pose, carrier_slot_id: int) -> Future:
        """
        Pick up a book and place it in the internal carrier.
        
        Args:
            book_id: Unique book identifier
            book_pose: 3D pose of the book
            carrier_slot_id: Internal carrier slot number
            
        Returns:
            Future: Action future for tracking progress
        """
        pass
    
    @abstractmethod
    def place_book(self, book_id: str, carrier_slot_id: int, 
                   target_pose: Pose, target_id: int, 
                   target_type: int) -> Future:
        """
        Place a book from carrier to target location.
        
        Args:
            book_id: Unique book identifier
            carrier_slot_id: Internal carrier slot number
            target_pose: Target placement pose
            target_id: storage_box_id or shelf_id
            target_type: 0=STORAGE_BOX, 1=BOOKSHELF
            
        Returns:
            Future: Action future for tracking progress
        """
        pass
    
    @abstractmethod
    def collect_books(self, book_ids: List[str], book_poses: List[Pose]) -> Future:
        """
        Collect multiple books in sequence.
        
        Args:
            book_ids: List of book identifiers
            book_poses: List of corresponding book poses
            
        Returns:
            Future: Action future for tracking progress
        """
        pass
    
    @abstractmethod
    def collect_trash(self, trash_pose: Pose) -> Future:
        """
        Collect trash from specified location.
        
        Args:
            trash_pose: 3D pose of trash location
            
        Returns:
            Future: Action future for tracking progress
        """
        pass
    
    @abstractmethod
    def dispose_trash(self, bin_pose: Pose) -> Future:
        """
        Dispose collected trash to bin.
        
        Args:
            bin_pose: 3D pose of trash bin
            
        Returns:
            Future: Action future for tracking progress
        """
        pass
    
    @abstractmethod
    def change_pose(self, pose_type: str) -> bool:
        """
        Change arm to predefined pose.
        
        Args:
            pose_type: Type of pose ("home", "carry", "ready", etc.)
            
        Returns:
            bool: True if successfully changed
        """
        pass
