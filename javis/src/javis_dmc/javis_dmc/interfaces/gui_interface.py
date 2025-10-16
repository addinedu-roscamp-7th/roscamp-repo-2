"""
GUI Interface for JAVIS DMC.
Handles GUI display and user interaction.
"""

from abc import abstractmethod
from typing import Dict, Any, Callable
from .base_interface import BaseInterface


class GUIInterface(BaseInterface):
    """
    Abstract interface for GUI control (Dobby GUI communication).
    
    This interface handles:
    - Screen updates and display
    - User interaction events
    - Status visualization
    """
    
    @abstractmethod
    def update_screen(self, screen_type: str, data: Dict[str, Any]) -> bool:
        """
        Update GUI screen with new data.
        
        Args:
            screen_type: Type of screen to update ("main", "task", "battery", etc.)
            data: Data to display on screen
            
        Returns:
            bool: True if update successful
        """
        pass
    
    @abstractmethod
    def subscribe_screen_event(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        """
        Subscribe to GUI screen events.
        
        Args:
            callback: Function to call when screen events occur
            
        Returns:
            bool: True if subscription successful
        """
        pass
