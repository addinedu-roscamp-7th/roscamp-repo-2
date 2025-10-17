"""
Mock implementations for JAVIS DMC interfaces.
Used for testing and development without real hardware.
"""

from .mock_base import MockBase
from .mock_drive import MockDriveInterface
from .mock_arm import MockArmInterface
from .mock_ai import MockAIInterface
from .mock_gui import MockGUIInterface
from .mock_llm import MockLLMInterface
from .mock_tts import MockTTSInterface
from .mock_stt import MockSTTInterface

__all__ = [
    'MockBase',
    'MockDriveInterface',
    'MockArmInterface', 
    'MockAIInterface',
    'MockGUIInterface',
    'MockLLMInterface',
    'MockTTSInterface',
    'MockSTTInterface'
]
