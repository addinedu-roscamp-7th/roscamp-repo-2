"""
STT (Speech-to-Text) Interface for JAVIS DMC.
Handles speech recognition functionality.
"""

from abc import abstractmethod
from typing import Optional, Callable
from .base_interface import BaseInterface


class STTResult:
    """Data class for speech recognition result."""
    def __init__(self, text: str, confidence: float, language: str):
        self.text = text
        self.confidence = confidence
        self.language = language


class STTInterface(BaseInterface):
    """
    Abstract interface for Speech-to-Text (ROS2 Topic Subscriber).
    
    This interface handles:
    - Speech recognition
    - Wake word detection
    - Continuous listening
    """
    
    @abstractmethod
    def subscribe_stt_result(self, callback: Callable[[STTResult], None]) -> bool:
        """
        Subscribe to speech recognition results.
        
        Args:
            callback: Function to call when speech is recognized
            
        Returns:
            bool: True if subscription successful
        """
        pass
    
    @abstractmethod
    def get_latest_result(self) -> Optional[STTResult]:
        """
        Get the most recent speech recognition result.
        
        Returns:
            Optional[STTResult]: Latest STT result if available
        """
        pass
    
    @abstractmethod
    def set_wake_word(self, wake_word: str) -> bool:
        """
        Set the wake word for activation.
        
        Args:
            wake_word: Wake word to listen for (e.g., "도비야")
            
        Returns:
            bool: True if wake word set successfully
        """
        pass
    
    @abstractmethod
    def start_listening(self) -> bool:
        """
        Start continuous speech recognition.
        
        Returns:
            bool: True if listening started successfully
        """
        pass
    
    @abstractmethod
    def stop_listening(self) -> bool:
        """
        Stop continuous speech recognition.
        
        Returns:
            bool: True if listening stopped successfully
        """
        pass
