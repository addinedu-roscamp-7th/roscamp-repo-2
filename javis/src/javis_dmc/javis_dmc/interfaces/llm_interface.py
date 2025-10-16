"""
LLM Interface for JAVIS DMC.
Handles Large Language Model integration for natural language processing.
"""

from abc import abstractmethod
from typing import Dict, Any, Optional
from .base_interface import BaseInterface


class Intent:
    """Data class for parsed user intent."""
    def __init__(self, action: str, parameters: Dict[str, Any], confidence: float):
        self.action = action  # "book_voice", "road_voice", "drink_voice"
        self.parameters = parameters
        self.confidence = confidence


class LLMInterface(BaseInterface):
    """
    Abstract interface for LLM communication (HTTP Client).
    
    This interface handles:
    - Natural language query parsing
    - Intent recognition
    - Response generation
    - Session management
    """
    
    @abstractmethod
    def parse_book_query(self, text: str) -> Optional[Intent]:
        """
        Parse book-related query using LLM.
        
        Args:
            text: User's natural language query
            
        Returns:
            Optional[Intent]: Parsed intent if successful
        """
        pass
    
    @abstractmethod
    def parse_guide_query(self, text: str) -> Optional[Intent]:
        """
        Parse navigation/guide query using LLM.
        
        Args:
            text: User's natural language query
            
        Returns:
            Optional[Intent]: Parsed intent if successful
        """
        pass
    
    @abstractmethod
    def parse_order_query(self, text: str) -> Optional[Intent]:
        """
        Parse order/service query using LLM.
        
        Args:
            text: User's natural language query
            
        Returns:
            Optional[Intent]: Parsed intent if successful
        """
        pass
    
    @abstractmethod
    def set_session_id(self, session_id: str) -> bool:
        """
        Set session ID for conversation tracking.
        
        Args:
            session_id: Unique session identifier
            
        Returns:
            bool: True if session set successfully
        """
        pass
