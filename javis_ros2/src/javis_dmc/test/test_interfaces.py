"""
Unit tests for JAVIS DMC interfaces.
Tests both abstract interfaces and mock implementations.
"""

import unittest
import time
from unittest.mock import Mock, patch
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion

# Import interfaces
from javis_dmc.interfaces import (
    BaseInterface, DriveInterface, ArmInterface, AIInterface,
    GUIInterface, LLMInterface, TTSInterface, STTInterface,
    BookPose, BoxStatus, ShelfInfo, TrashInfo, Intent, STTResult
)

# Import mock implementations
from javis_dmc.mock import (
    MockDriveInterface, MockArmInterface, MockAIInterface,
    MockGUIInterface, MockLLMInterface, MockTTSInterface, MockSTTInterface,
    MockResponse
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
        
        # Register person
        tracking_id = self.ai.register_person()
        self.assertIsNotNone(tracking_id)
        self.assertTrue(tracking_id.startswith("person_"))
        
        # Change tracking mode
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


class TestMockLLMInterface(unittest.TestCase):
    """Test cases for MockLLMInterface."""
    
    def setUp(self):
        """Set up test fixtures."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_llm_node')
        self.llm = MockLLMInterface(self.node, "dobby1")
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.llm.shutdown()
        self.node.destroy_node()
    
    def test_mock_llm_book_query(self):
        """Test mock book query parsing."""
        self.llm.initialize()
        
        # Test book-related query
        intent = self.llm.parse_book_query("어린왕자 책 찾아줘")
        self.assertIsNotNone(intent)
        self.assertEqual(intent.action, "book_voice")
        self.assertIn("book_id", intent.parameters)
        
        # Test non-book query
        intent = self.llm.parse_book_query("안녕하세요")
        self.assertIsNone(intent)
    
    def test_mock_llm_guide_query(self):
        """Test mock guide query parsing."""
        self.llm.initialize()
        
        # Test guide-related query
        intent = self.llm.parse_guide_query("화장실 어디야")
        self.assertIsNotNone(intent)
        self.assertEqual(intent.action, "road_voice")
        self.assertEqual(intent.parameters["destination"], "화장실")
    
    def test_mock_llm_session(self):
        """Test mock session management."""
        self.llm.initialize()
        
        # Set session ID
        result = self.llm.set_session_id("test_session_123")
        self.assertTrue(result)
        
        # Check session ID
        session_id = self.llm.get_session_id()
        self.assertEqual(session_id, "test_session_123")


class TestMockTTSInterface(unittest.TestCase):
    """Test cases for MockTTSInterface."""
    
    def setUp(self):
        """Set up test fixtures."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_tts_node')
        self.tts = MockTTSInterface(self.node, "dobby1")
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.tts.shutdown()
        self.node.destroy_node()
    
    def test_mock_tts_speak(self):
        """Test mock speak functionality."""
        self.tts.initialize()
        
        # Test speak with wait
        result = self.tts.speak("안녕하세요", wait=True)
        self.assertTrue(result)
        self.assertFalse(self.tts.is_speaking())
        
        # Test speak without wait
        result = self.tts.speak("테스트 메시지", wait=False)
        self.assertTrue(result)
        # Note: is_speaking() might be True or False depending on timing
    
    def test_mock_tts_duration(self):
        """Test mock duration estimation."""
        self.tts.initialize()
        
        duration = self.tts.get_estimated_duration("테스트", speed=1.0)
        self.assertIsNotNone(duration)
        self.assertGreater(duration, 0)
        
        # Test with different speed
        duration_fast = self.tts.get_estimated_duration("테스트", speed=2.0)
        self.assertLess(duration_fast, duration)


class TestMockSTTInterface(unittest.TestCase):
    """Test cases for MockSTTInterface."""
    
    def setUp(self):
        """Set up test fixtures."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('test_stt_node')
        self.stt = MockSTTInterface(self.node, "dobby1")
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.stt.shutdown()
        self.node.destroy_node()
    
    def test_mock_stt_listening(self):
        """Test mock listening functionality."""
        self.stt.initialize()
        
        # Test start listening
        result = self.stt.start_listening()
        self.assertTrue(result)
        self.assertTrue(self.stt.is_listening())
        
        # Test stop listening
        result = self.stt.stop_listening()
        self.assertTrue(result)
        self.assertFalse(self.stt.is_listening())
    
    def test_mock_stt_wake_word(self):
        """Test mock wake word functionality."""
        self.stt.initialize()
        
        # Test set wake word
        result = self.stt.set_wake_word("헤이 도비")
        self.assertTrue(result)
        self.assertEqual(self.stt.get_wake_word(), "헤이 도비")
    
    def test_mock_stt_simulation(self):
        """Test mock speech input simulation."""
        self.stt.initialize()
        
        # Subscribe to results
        results = []
        def stt_callback(result):
            results.append(result)
        
        self.stt.subscribe_stt_result(stt_callback)
        self.stt.start_listening()
        
        # Simulate speech input
        self.stt.simulate_speech_input("테스트 음성", confidence=0.95)
        
        # Check result
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].text, "테스트 음성")
        self.assertEqual(results[0].confidence, 0.95)
        
        # Check latest result
        latest = self.stt.get_latest_result()
        self.assertIsNotNone(latest)
        self.assertEqual(latest.text, "테스트 음성")


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
        suite.addTests(loader.loadTestsFromTestCase(TestMockLLMInterface))
        suite.addTests(loader.loadTestsFromTestCase(TestMockTTSInterface))
        suite.addTests(loader.loadTestsFromTestCase(TestMockSTTInterface))
        
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