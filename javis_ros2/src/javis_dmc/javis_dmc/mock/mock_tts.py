"""
Mock TTS Interface implementation.
"""

import time
from typing import Optional
from ..interfaces.tts_interface import TTSInterface
from .mock_base import MockBase, MockResponse


class MockTTSInterface(TTSInterface, MockBase):
    """
    Mock implementation of TTSInterface for testing.
    """
    
    def __init__(self, node, namespace: str = ""):
        TTSInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)
        
        self._is_speaking = False
        self._current_text = None
    
    def initialize(self) -> bool:
        """Initialize mock TTS interface."""
        self.logger.info("Mock TTS interface initialized")
        self._set_initialized(True)
        return True
    
    def speak(self, text: str, language: str = "ko-KR", 
              speed: float = 1.0, wait: bool = True) -> bool:
        """Mock speak implementation."""
        self.logger.info(f"Mock speak: '{text}' (lang={language}, speed={speed:.1f})")
        response = self.get_mock_response('speak')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._is_speaking = True
            self._current_text = text
            
            # Estimate duration based on text length
            estimated_duration = len(text) * 0.1 / speed  # rough estimate
            
            if wait:
                # Simulate speaking time
                time.sleep(min(estimated_duration, 5.0))  # max 5 seconds for mock
                self._is_speaking = False
                self._current_text = None
            
            self.logger.debug(f"Mock speech {'completed' if wait else 'started'}")
        
        return response.success
    
    def stop_speaking(self) -> bool:
        """Mock stop speaking implementation."""
        self.logger.info("Mock stop_speaking")
        response = self.get_mock_response('stop_speaking')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._is_speaking = False
            self._current_text = None
            self.logger.debug("Mock speech stopped")
        
        return response.success
    
    def get_estimated_duration(self, text: str, language: str = "ko-KR", 
                              speed: float = 1.0) -> Optional[float]:
        """Mock get estimated duration implementation."""
        response = self.get_mock_response('get_estimated_duration')
        
        if not response.success:
            return None
        
        # Simple duration estimation: 0.1 seconds per character
        duration = len(text) * 0.1 / speed
        self.logger.debug(f"Mock estimated duration for '{text}': {duration:.2f}s")
        return duration
    
    def is_speaking(self) -> bool:
        """Check if currently speaking (for testing)."""
        return self._is_speaking
    
    def get_current_text(self) -> Optional[str]:
        """Get currently speaking text (for testing)."""
        return self._current_text
    
    def shutdown(self):
        """Shutdown mock TTS interface."""
        self.cleanup()
        super().shutdown()
