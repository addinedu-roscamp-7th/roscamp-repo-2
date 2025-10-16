"""
Mock Drive Interface implementation.
"""

import time
from typing import Optional
from concurrent.futures import Future
from geometry_msgs.msg import Pose, Point, Quaternion
from ..interfaces.drive_interface import DriveInterface
from .mock_base import MockBase, MockResponse


class MockDriveInterface(DriveInterface, MockBase):
    """
    Mock implementation of DriveInterface for testing.
    """
    
    def __init__(self, node, namespace: str = ""):
        DriveInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)
        
        # Mock current pose
        self._current_pose = Pose()
        self._current_pose.position = Point(x=0.0, y=0.0, z=0.0)
        self._current_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Mock follow mode state
        self._follow_mode_enabled = False
    
    def initialize(self) -> bool:
        """Initialize mock drive interface."""
        # Start publishing current pose
        self.start_topic_publishing("current_pose", rate_hz=10.0)
        self._set_initialized(True)
        return True
    
    def move_to_target(self, pose: Pose, location_name: str = "") -> Future:
        """Mock move to target implementation."""
        self.logger.info(f"Mock move_to_target: {location_name} at ({pose.position.x}, {pose.position.y})")
        
        # Create additional data for result
        additional_data = {
            'final_pose': pose,
            'distance_traveled': 5.0,  # Mock distance
            'location_name': location_name
        }
        
        future = self.create_mock_future('move_to_target', additional_data)
        
        # Update current pose after successful move (simulated)
        def update_pose_callback(fut):
            result = fut.result()
            if result.success:
                self._current_pose = pose
        
        future.add_done_callback(update_pose_callback)
        return future
    
    def enable_follow_mode(self, tracking_topic: str, destination: Pose, 
                          follow_distance: float = 1.5) -> bool:
        """Mock enable follow mode implementation."""
        response = self.get_mock_response('enable_follow_mode')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._follow_mode_enabled = True
            self.logger.info(f"Mock follow mode enabled: topic={tracking_topic}, distance={follow_distance}m")
        else:
            self.logger.warn("Mock follow mode enable failed")
        
        return response.success
    
    def disable_follow_mode(self) -> bool:
        """Mock disable follow mode implementation."""
        response = self.get_mock_response('disable_follow_mode')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._follow_mode_enabled = False
            self.logger.info("Mock follow mode disabled")
        else:
            self.logger.warn("Mock follow mode disable failed")
        
        return response.success
    
    def stop(self) -> bool:
        """Mock emergency stop implementation."""
        response = self.get_mock_response('stop')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self.logger.info("Mock emergency stop executed")
        else:
            self.logger.warn("Mock emergency stop failed")
        
        return response.success
    
    def rotate_in_place(self, angle: float) -> bool:
        """Mock rotate in place implementation."""
        self.logger.info(f"Mock rotate_in_place: {angle} degrees")
        response = self.get_mock_response('rotate_in_place')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        return response.success
    
    def get_current_pose(self) -> Optional[Pose]:
        """Mock get current pose implementation."""
        return self._current_pose
    
    def _publish_topic_data(self, topic_name: str):
        """Publish mock topic data."""
        if topic_name == "current_pose":
            # In real implementation, this would publish to ROS topic
            # For mock, we just log occasionally
            if hasattr(self, '_last_pose_log_time'):
                if time.time() - self._last_pose_log_time > 5.0:  # Log every 5 seconds
                    self.logger.debug(f"Mock current pose: ({self._current_pose.position.x:.2f}, "
                                    f"{self._current_pose.position.y:.2f})")
                    self._last_pose_log_time = time.time()
            else:
                self._last_pose_log_time = time.time()
    
    def shutdown(self):
        """Shutdown mock drive interface."""
        self.cleanup()
        super().shutdown()
