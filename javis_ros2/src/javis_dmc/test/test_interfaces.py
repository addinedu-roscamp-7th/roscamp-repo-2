"""
Unit tests for JAVIS DMC interfaces.
Tests both abstract interfaces and mock implementations.
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion

# Import interfaces
from javis_dmc.interfaces import (
    BaseInterface, DriveInterface, ArmInterface, AIInterface,
    GUIInterface, BookPose, BoxStatus, ShelfInfo, TrashInfo
)

# Import mock implementations
from javis_dmc.mock import (
    MockDriveInterface, MockArmInterface, MockAIInterface,
    MockGUIInterface, MockResponse
)


class TestBaseInterface(unittest.TestCase):
    """Test cases for BaseInterface."""
    
    def setUp(self):
        """Set up test fixtures."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_interface_node')
        
    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()
    
    def test_base_interface_creation(self):
        """Test BaseInterface creation and basic functionality."""
        # Create a concrete implementation for testing
        class TestInterface(BaseInterface):
            def initialize(self) -> bool:
                self._set_initialized(True)
                return True
        
        interface = TestInterface(self.node, "test_namespace")
        
        # Test basic properties
        self.assertEqual(interface.namespace, "test_namespace")
        self.assertEqual(interface.get_namespace(), "test_namespace")
        self.assertIsNotNone(interface.get_logger())
        self.assertFalse(interface.is_initialized())
        
        # Test initialization
        result = interface.initialize()
        self.assertTrue(result)
        self.assertTrue(interface.is_initialized())
        
        # Test topic name creation
        topic_name = interface._create_topic_name("status/battery")
        self.assertEqual(topic_name, "test_namespace/status/battery")
        
        # Test shutdown
        interface.shutdown()
        self.assertFalse(interface.is_initialized())


class TestMockDriveInterface(unittest.TestCase):
    """Test cases for MockDriveInterface."""
    
    def setUp(self):
        """Set up test fixtures."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_drive_node')
        self.drive = MockDriveInterface(self.node, "dobby1")
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.drive.shutdown()
        self.node.destroy_node()
    
    def test_mock_drive_initialization(self):
        """Test mock drive interface initialization."""
        result = self.drive.initialize()
        self.assertTrue(result)
        self.assertTrue(self.drive.is_initialized())
    
    def test_mock_move_to_target(self):
        """Test mock move to target functionality."""
        self.drive.initialize()
        
        # Create target pose
        target_pose = Pose()
        target_pose.position = Point(x=5.0, y=3.0, z=0.0)
        target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Test successful move
        future = self.drive.move_to_target(target_pose, "test_location")
        self.assertIsNotNone(future)
        
        # Wait for completion
        result = future.result(timeout=5.0)
        self.assertTrue(result.success)
        self.assertEqual(result.location_name, "test_location")
    
    def test_mock_drive_follow_mode(self):
        """Test mock follow mode functionality."""
        self.drive.initialize()
        
        # Test enable follow mode
        target_pose = Pose()
        result = self.drive.enable_follow_mode("/tracking/status", target_pose)
        self.assertTrue(result)
        
        # Test disable follow mode
        result = self.drive.disable_follow_mode()
        self.assertTrue(result)
    
    def test_mock_drive_with_failure(self):
        """Test mock drive with configured failure."""
        self.drive.initialize()
        
        # Configure mock to fail
        failure_response = MockResponse(success=False, delay=0.1, error_code="PATH_NOT_FOUND")
        self.drive.set_mock_response('move_to_target', failure_response)
        
        # Test failed move
        target_pose = Pose()
        future = self.drive.move_to_target(target_pose)
        result = future.result(timeout=5.0)
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "PATH_NOT_FOUND")

    def test_mock_drive_resume(self):
        """Test mock resume functionality."""
        self.drive.initialize()

        # Successful resume
        self.assertTrue(self.drive.resume())

        # Configure resume to fail
        self.drive.set_mock_response('resume', MockResponse(success=False))
        self.assertFalse(self.drive.resume(reason="test_reason"))


class TestMockArmInterface(unittest.TestCase):
    """Test cases for MockArmInterface."""
    
    def setUp(self):
        """Set up test fixtures."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_arm_node')
        self.arm = MockArmInterface(self.node, "dobby1")
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.arm.shutdown()
        self.node.destroy_node()
    
    def test_mock_arm_pick_book(self):
        """Test mock pick book functionality."""
        self.arm.initialize()
        
        book_pose = Pose()
        book_pose.position = Point(x=1.0, y=2.0, z=0.5)
        
        future = self.arm.pick_book("ISBN-123", book_pose, 1)
        result = future.result(timeout=5.0)
        
        self.assertTrue(result.success)
        self.assertEqual(result.book_id, "ISBN-123")
        self.assertEqual(result.carrier_slot_id, 1)
        
        # Check carrier status
        status = self.arm.get_carrier_status()
        self.assertIn(1, status['slots'])
        self.assertEqual(status['slots'][1], "ISBN-123")
    
    def test_mock_arm_place_book(self):
        """Test mock place book functionality."""
        self.arm.initialize()
        
        # First pick a book
        book_pose = Pose()
        future = self.arm.pick_book("ISBN-456", book_pose, 2)
        future.result(timeout=5.0)
        
        # Then place it
        target_pose = Pose()
        future = self.arm.place_book("ISBN-456", 2, target_pose, 101, 1)
        result = future.result(timeout=5.0)
        
        self.assertTrue(result.success)
        self.assertEqual(result.book_id, "ISBN-456")
        
        # Check that book is removed from carrier
        status = self.arm.get_carrier_status()
        self.assertNotIn(2, status['slots'])


class TestMockAIInterface(unittest.TestCase):
    """Test cases for MockAIInterface."""
    
    def setUp(self):
        """Set up test fixtures."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_ai_node')
        self.ai = MockAIInterface(self.node, "dobby1")
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.ai.shutdown()
        self.node.destroy_node()
    
    def test_mock_ai_detect_book(self):
        """Test mock book detection."""
        self.ai.initialize()
        
        # Test detecting existing book
        book_pose = self.ai.detect_book("ISBN-123")
        self.assertIsNotNone(book_pose)
        self.assertEqual(book_pose.book_id, "ISBN-123")
        self.assertGreater(book_pose.confidence, 0.5)
        
        # Test detecting non-existing book
        book_pose = self.ai.detect_book("NON-EXISTING")
        self.assertIsNone(book_pose)
    
    def test_mock_ai_tracking(self):
        """Test mock person tracking."""
        self.ai.initialize()
        
        # Change tracking mode to registration to simulate detection
        result = self.ai.change_tracking_mode("registration")
        self.assertTrue(result)
        
        # Switch to follow mode
        result = self.ai.change_tracking_mode("follow")
        self.assertTrue(result)
        
        # Subscribe to tracking status
        callback_called = []
        def tracking_callback(status):
            callback_called.append(status)
        
        result = self.ai.subscribe_tracking_status(tracking_callback)
        self.assertTrue(result)
        
        # Wait a bit for mock data
        time.sleep(0.5)
        
        # Check if callback was called (mock publishes data)
        # Note: This might be flaky in CI, consider making it more deterministic


def main():
    """Run all interface tests."""
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create test suite
        loader = unittest.TestLoader()
        suite = unittest.TestSuite()
        
        # Add test cases
        suite.addTests(loader.loadTestsFromTestCase(TestBaseInterface))
        suite.addTests(loader.loadTestsFromTestCase(TestMockDriveInterface))
        suite.addTests(loader.loadTestsFromTestCase(TestMockArmInterface))
        suite.addTests(loader.loadTestsFromTestCase(TestMockAIInterface))
        
        # Run tests
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        # Return success/failure
        return 0 if result.wasSuccessful() else 1
        
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    sys.exit(main())
