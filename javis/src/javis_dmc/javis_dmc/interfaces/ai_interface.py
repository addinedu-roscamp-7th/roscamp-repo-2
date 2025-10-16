"""
AI Interface for JAVIS DMC.
Handles computer vision and AI recognition tasks.
"""

from abc import abstractmethod
from typing import List, Optional, Callable, Dict, Any
from geometry_msgs.msg import Pose
from .base_interface import BaseInterface


class BookPose:
    """Data class for book detection result."""
    def __init__(self, book_id: str, pose: Pose, confidence: float):
        self.book_id = book_id
        self.pose = pose
        self.confidence = confidence


class BoxStatus:
    """Data class for storage box status."""
    def __init__(self, box_id: int, is_empty: bool, book_count: int):
        self.box_id = box_id
        self.is_empty = is_empty
        self.book_count = book_count


class ShelfInfo:
    """Data class for bookshelf information."""
    def __init__(self, shelf_id: str, pose: Pose, capacity: int, current_books: int):
        self.shelf_id = shelf_id
        self.pose = pose
        self.capacity = capacity
        self.current_books = current_books


class TrashInfo:
    """Data class for trash detection."""
    def __init__(self, trash_type: str, pose: Pose, confidence: float):
        self.trash_type = trash_type
        self.pose = pose
        self.confidence = confidence


class AIInterface(BaseInterface):
    """
    Abstract interface for AI vision system (AIS communication).
    
    This interface handles:
    - Book detection and identification
    - Person tracking
    - Storage box monitoring
    - Trash detection
    - Bookshelf analysis
    """
    
    @abstractmethod
    def detect_book(self, book_id: str) -> Optional[BookPose]:
        """
        Detect and locate a specific book.
        
        Args:
            book_id: Book identifier to search for
            
        Returns:
            Optional[BookPose]: Book pose if detected, None otherwise
        """
        pass
    
    @abstractmethod
    def check_storage_box(self, box_id: int) -> BoxStatus:
        """
        Check the status of a storage box.
        
        Args:
            box_id: Storage box identifier
            
        Returns:
            BoxStatus: Current status of the box
        """
        pass
    
    @abstractmethod
    def verify_book_position(self, book_id: str, expected_pose: Pose) -> bool:
        """
        Verify if a book is at the expected position.
        
        Args:
            book_id: Book identifier
            expected_pose: Expected book pose
            
        Returns:
            bool: True if book is at expected position
        """
        pass
    
    @abstractmethod
    def identify_bookshelf(self, shelf_area: Pose) -> Optional[ShelfInfo]:
        """
        Identify and analyze a bookshelf.
        
        Args:
            shelf_area: Approximate shelf location
            
        Returns:
            Optional[ShelfInfo]: Shelf information if identified
        """
        pass
    
    @abstractmethod
    def detect_trash(self, seat_pose: Pose) -> List[TrashInfo]:
        """
        Detect trash around a seat area.
        
        Args:
            seat_pose: Seat location to check
            
        Returns:
            List[TrashInfo]: List of detected trash items
        """
        pass
    
    @abstractmethod
    def register_person(self) -> Optional[str]:
        """
        Register a person for tracking.
        
        Returns:
            Optional[str]: Tracking ID if successful, None otherwise
        """
        pass
    
    @abstractmethod
    def change_tracking_mode(self, mode: str) -> bool:
        """
        Change person tracking mode.
        
        Args:
            mode: Tracking mode ("follow", "observe", "disabled")
            
        Returns:
            bool: True if mode changed successfully
        """
        pass
    
    @abstractmethod
    def subscribe_tracking_status(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        """
        Subscribe to person tracking status updates.
        
        Args:
            callback: Function to call when tracking status updates
            
        Returns:
            bool: True if subscription successful
        """
        pass
