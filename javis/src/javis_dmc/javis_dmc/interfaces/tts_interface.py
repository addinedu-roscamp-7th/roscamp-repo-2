"""
TTS (Text-to-Speech) Interface for JAVIS DMC.
Handles speech synthesis functionality.
"""

from abc import abstractmethod
from typing import Optional
from .base_interface import BaseInterface


class TTSInterface(BaseInterface):
    """
    Abstract interface for Text-to-Speech (ROS2 Service).
    
    This interface handles:
    - Text-to-speech conversion
    - Speech synthesis control
    - Audio output management
    """
    
    @abstractmethod
    def speak(self, text: str, language: str = "ko-KR", 
              speed: float = 1.0, wait: bool = True) -> bool:
        """
        Convert text to speech and play audio.
        
        Args:
            text: Text to convert to speech
            language: Language code ("ko-KR", "en-US")
            speed: Speech speed (0.5 ~ 2.0, default: 1.0)
            wait: Whether to wait for speech completion
            
        Returns:
            bool: True if speech request successful
        """
        pass
    
    @abstractmethod
    def stop_speaking(self) -> bool:
        """
        Stop current speech output.
        
        Returns:
            bool: True if stop successful
        """
        pass
    
    @abstractmethod
    def get_estimated_duration(self, text: str, language: str = "ko-KR", 
                              speed: float = 1.0) -> Optional[float]:
        """
        Get estimated speech duration for given text.
        
        Args:
            text: Text to estimate duration for
            language: Language code
            speed: Speech speed
            
        Returns:
            Optional[float]: Estimated duration in seconds, None if unavailable
        """
        pass
