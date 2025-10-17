"""
Mock STT Interface implementation.
"""

import time
import random
import threading
from typing import Optional, Callable
from ..interfaces.stt_interface import STTInterface, STTResult
from .mock_base import MockBase, MockResponse


class MockSTTInterface(STTInterface, MockBase):
    """
    Mock implementation of STTInterface for testing.
    """
    
    def __init__(self, node, namespace: str = ""):
        STTInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)
        
        self._callbacks = []
        self._latest_result = None
        self._wake_word = "도비야"
        self._is_listening = False
        
        # Mock recognition patterns
        self._mock_phrases = [
            "도비야",
            "화장실 어디야",
            "열람실 안내해줘",
            "어린왕자 책 찾아줘",
            "아메리카노 주문할게",
            "고마워",
            "안녕"
        ]
    
    def initialize(self) -> bool:
        """Initialize mock STT interface."""
        self.logger.info("Mock STT interface initialized")
        self._set_initialized(True)
        return True
    
    def subscribe_stt_result(self, callback: Callable[[STTResult], None]) -> bool:
        """Mock subscribe STT result implementation."""
        self.logger.info("Mock subscribe_stt_result")
        self._callbacks.append(callback)
        return True
    
    def get_latest_result(self) -> Optional[STTResult]:
        """Mock get latest result implementation."""
        return self._latest_result
    
    def set_wake_word(self, wake_word: str) -> bool:
        """Mock set wake word implementation."""
        self.logger.info(f"Mock set_wake_word: '{wake_word}'")
        response = self.get_mock_response('set_wake_word')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._wake_word = wake_word
            self.logger.debug(f"Mock wake word set to: {wake_word}")
        
        return response.success
    
    def start_listening(self) -> bool:
        """Mock start listening implementation."""
        self.logger.info("Mock start_listening")
        response = self.get_mock_response('start_listening')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._is_listening = True
            # Start publishing mock STT results
            self.start_topic_publishing("stt_result", rate_hz=0.2)  # Every 5 seconds
            self.logger.debug("Mock listening started")
        
        return response.success
    
    def stop_listening(self) -> bool:
        """Mock stop listening implementation."""
        self.logger.info("Mock stop_listening")
        response = self.get_mock_response('stop_listening')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._is_listening = False
            self.stop_topic_publishing()
            self.logger.debug("Mock listening stopped")
        
        return response.success
    
    def simulate_speech_input(self, text: str, confidence: float = 0.9, 
                             language: str = "ko-KR"):
        """Simulate speech input (for testing)."""
        if not self._is_listening:
            self.logger.warn("Cannot simulate speech input - not listening")
            return
        
        result = STTResult(text, confidence, language)
        self._latest_result = result
        
        # Call registered callbacks
        for callback in self._callbacks:
            try:
                callback(result)
            except Exception as e:
                self.logger.error(f"Error in STT callback: {e}")
        
        self.logger.info(f"Mock simulated speech input: '{text}' (confidence={confidence:.2f})")
    
    def _publish_topic_data(self, topic_name: str):
        """Publish mock topic data."""
        if topic_name == "stt_result" and self._is_listening:
            # Randomly generate speech recognition results
            if random.random() < 0.3:  # 30% chance to generate result
                text = random.choice(self._mock_phrases)
                confidence = random.uniform(0.7, 0.95)
                self.simulate_speech_input(text, confidence)
    
    def is_listening(self) -> bool:
        """Check if currently listening (for testing)."""
        return self._is_listening
    
    def get_wake_word(self) -> str:
        """Get current wake word (for testing)."""
        return self._wake_word
    
    def shutdown(self):
        """Shutdown mock STT interface."""
        self.cleanup()
        super().shutdown()
