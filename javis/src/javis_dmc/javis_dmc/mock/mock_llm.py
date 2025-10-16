"""
Mock LLM Interface implementation.
"""

import time
import random
from typing import Optional
from ..interfaces.llm_interface import LLMInterface, Intent
from .mock_base import MockBase, MockResponse


class MockLLMInterface(LLMInterface, MockBase):
    """
    Mock implementation of LLMInterface for testing.
    """
    
    def __init__(self, node, namespace: str = ""):
        LLMInterface.__init__(self, node, namespace)
        MockBase.__init__(self, node, namespace)
        
        self._session_id = None
        
        # Mock intent patterns
        self._book_patterns = ["책", "도서", "찾", "어디"]
        self._guide_patterns = ["화장실", "열람실", "안내", "길"]
        self._order_patterns = ["주문", "아메리카노", "커피", "음료"]
    
    def initialize(self) -> bool:
        """Initialize mock LLM interface."""
        self.logger.info("Mock LLM interface initialized")
        self._set_initialized(True)
        return True
    
    def parse_book_query(self, text: str) -> Optional[Intent]:
        """Mock parse book query implementation."""
        self.logger.info(f"Mock parse_book_query: '{text}'")
        response = self.get_mock_response('parse_book_query')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if not response.success:
            return None
        
        # Check if text contains book-related keywords
        if any(pattern in text for pattern in self._book_patterns):
            intent = Intent(
                action="book_voice",
                parameters={
                    "query": text,
                    "book_id": "ISBN-123" if "어린왕자" in text else None,
                    "response": f"{text}에 대한 도서 정보를 찾았습니다."
                },
                confidence=random.uniform(0.8, 0.95)
            )
            return intent
        
        return None
    
    def parse_guide_query(self, text: str) -> Optional[Intent]:
        """Mock parse guide query implementation."""
        self.logger.info(f"Mock parse_guide_query: '{text}'")
        response = self.get_mock_response('parse_guide_query')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if not response.success:
            return None
        
        # Check if text contains guide-related keywords
        if any(pattern in text for pattern in self._guide_patterns):
            intent = Intent(
                action="road_voice",
                parameters={
                    "query": text,
                    "destination": "화장실" if "화장실" in text else "열람실",
                    "response": f"{text} 안내를 시작하겠습니다."
                },
                confidence=random.uniform(0.8, 0.95)
            )
            return intent
        
        return None
    
    def parse_order_query(self, text: str) -> Optional[Intent]:
        """Mock parse order query implementation."""
        self.logger.info(f"Mock parse_order_query: '{text}'")
        response = self.get_mock_response('parse_order_query')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if not response.success:
            return None
        
        # Check if text contains order-related keywords
        if any(pattern in text for pattern in self._order_patterns):
            intent = Intent(
                action="drink_voice",
                parameters={
                    "query": text,
                    "item": "아메리카노" if "아메리카노" in text else "음료",
                    "response": f"{text} 주문이 접수되었습니다."
                },
                confidence=random.uniform(0.8, 0.95)
            )
            return intent
        
        return None
    
    def set_session_id(self, session_id: str) -> bool:
        """Mock set session ID implementation."""
        self.logger.info(f"Mock set_session_id: {session_id}")
        response = self.get_mock_response('set_session_id')
        
        if response.delay > 0:
            time.sleep(response.delay)
        
        if response.success:
            self._session_id = session_id
            self.logger.debug(f"Mock session ID set to: {session_id}")
        
        return response.success
    
    def get_session_id(self) -> Optional[str]:
        """Get current session ID (for testing)."""
        return self._session_id
    
    def shutdown(self):
        """Shutdown mock LLM interface."""
        self.cleanup()
        super().shutdown()
