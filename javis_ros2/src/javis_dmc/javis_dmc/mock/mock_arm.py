"""
Mock Arm Interface implementation.
"""

import time
from typing import List
from concurrent.futures import Future
from geometry_msgs.msg import Pose
from ..interfaces.arm_interface import ArmInterface
from .mock_base import MockBase, MockResponse


class MockArmInterface(ArmInterface, MockBase):
    """
    Mock implementation of ArmInterface for testing.
    """
    
    def __init__(self, node, namespace: str = ""):
        ArmInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)
        
        # Mock carrier state
        self._carrier_slots = {}  # slot_id -> book_id
        self._current_pose = "home"
    
    def initialize(self) -> bool:
        """Initialize mock arm interface."""
        self.logger.info("Mock arm interface initialized")
        self._set_initialized(True)
        return True
    
    def pick_book(self, book_id: str, book_pose: Pose, carrier_slot_id: int) -> Future:
        """Mock pick book implementation."""
        self.logger.info(f"Mock pick_book: {book_id} -> slot {carrier_slot_id}")
        
        additional_data = {
            'book_id': book_id,
            'carrier_slot_id': carrier_slot_id,
            'message': "OK"
        }
        
        future = self.create_mock_future('pick_book', additional_data)
        
        # Update carrier state after successful pick
        def update_carrier_callback(fut):
            result = fut.result()
            if result.success:
                self._carrier_slots[carrier_slot_id] = book_id
                self.logger.debug(f"Book {book_id} placed in carrier slot {carrier_slot_id}")
            else:
                error_messages = ["BOOK_NOT_FOUND", "GRIPPER_ERROR", "CARRIER_FULL"]
                result.message = result.error_code or "GRIPPER_ERROR"
        
        future.add_done_callback(update_carrier_callback)
        return future
    
    def place_book(self, book_id: str, carrier_slot_id: int, 
                   target_pose: Pose, target_id: int, 
                   target_type: int) -> Future:
        """Mock place book implementation."""
        target_type_str = "STORAGE_BOX" if target_type == 0 else "BOOKSHELF"
        self.logger.info(f"Mock place_book: {book_id} from slot {carrier_slot_id} "
                        f"to {target_type_str} {target_id}")
        
        additional_data = {
            'book_id': book_id,
            'target_id': target_id,
            'target_type': target_type,
            'message': "OK"
        }
        
        future = self.create_mock_future('place_book', additional_data)
        
        # Update carrier state after successful place
        def update_carrier_callback(fut):
            result = fut.result()
            if result.success:
                if carrier_slot_id in self._carrier_slots:
                    del self._carrier_slots[carrier_slot_id]
                    self.logger.debug(f"Book {book_id} removed from carrier slot {carrier_slot_id}")
        
        future.add_done_callback(update_carrier_callback)
        return future
    
    def collect_books(self, book_ids: List[str], book_poses: List[Pose]) -> Future:
        """Mock collect books implementation."""
        self.logger.info(f"Mock collect_books: {len(book_ids)} books")
        
        additional_data = {
            'book_ids': book_ids,
            'collected_count': len(book_ids),
            'message': "OK"
        }
        
        return self.create_mock_future('collect_books', additional_data)
    
    def collect_trash(self, trash_pose: Pose) -> Future:
        """Mock collect trash implementation."""
        self.logger.info(f"Mock collect_trash at ({trash_pose.position.x:.2f}, "
                        f"{trash_pose.position.y:.2f})")
        
        additional_data = {
            'trash_pose': trash_pose,
            'message': "OK"
        }
        
        return self.create_mock_future('collect_trash', additional_data)
    
    def dispose_trash(self, bin_pose: Pose) -> Future:
        """Mock dispose trash implementation."""
        self.logger.info(f"Mock dispose_trash at bin ({bin_pose.position.x:.2f}, "
                        f"{bin_pose.position.y:.2f})")
        
        additional_data = {
            'bin_pose': bin_pose,
            'message': "OK"
        }
        
        return self.create_mock_future('dispose_trash', additional_data)
    
    def change_pose(self, pose_type: str) -> bool:
        """Mock change pose implementation."""
        self.logger.info(f"Mock change_pose: {pose_type}")
        response = self.get_mock_response('change_pose')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._current_pose = pose_type
            self.logger.debug(f"Arm pose changed to {pose_type}")
        
        return response.success
    
    def get_carrier_status(self) -> dict:
        """Get mock carrier status (additional method for testing)."""
        return {
            'slots': self._carrier_slots.copy(),
            'current_pose': self._current_pose,
            'available_slots': [i for i in range(1, 9) if i not in self._carrier_slots]
        }
    
    def shutdown(self):
        """Shutdown mock arm interface."""
        self.cleanup()
        super().shutdown()
