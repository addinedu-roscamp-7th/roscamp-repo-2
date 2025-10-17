"""
Mock GUI Interface implementation.
"""

import time
from typing import Dict, Any, Callable
from ..interfaces.gui_interface import GUIInterface
from .mock_base import MockBase, MockResponse


class MockGUIInterface(GUIInterface, MockBase):
    """
    Mock implementation of GUIInterface for testing.
    """
    
    def __init__(self, node, namespace: str = ""):
        GUIInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)
        
        self._screen_callbacks = []
        self._current_screen = "main"
        self._screen_data = {}
    
    def initialize(self) -> bool:
        """Initialize mock GUI interface."""
        self.logger.info("Mock GUI interface initialized")
        self._set_initialized(True)
        return True
    
    def update_screen(self, screen_type: str, data: Dict[str, Any]) -> bool:
        """Mock update screen implementation."""
        self.logger.info(f"Mock update_screen: {screen_type}")
        response = self.get_mock_response('update_screen')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._current_screen = screen_type
            self._screen_data = data.copy()
            self.logger.debug(f"Mock screen updated to {screen_type} with data: {data}")
        
        return response.success
    
    def subscribe_screen_event(self, callback: Callable[[Dict[str, Any]], None]) -> bool:
        """Mock subscribe screen event implementation."""
        self.logger.info("Mock subscribe_screen_event")
        self._screen_callbacks.append(callback)
        return True
    
    def simulate_screen_event(self, event_type: str, event_data: Dict[str, Any] = None):
        """Simulate a screen event (for testing)."""
        event = {
            'event_type': event_type,
            'screen': self._current_screen,
            'data': event_data or {},
            'timestamp': time.time()
        }
        
        for callback in self._screen_callbacks:
            try:
                callback(event)
            except Exception as e:
                self.logger.error(f"Error in screen event callback: {e}")
    
    def get_current_screen_info(self) -> Dict[str, Any]:
        """Get current screen information (for testing)."""
        return {
            'screen_type': self._current_screen,
            'data': self._screen_data.copy()
        }
    
    def shutdown(self):
        """Shutdown mock GUI interface."""
        self.cleanup()
        super().shutdown()
