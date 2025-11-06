"""
VoicePipeline 서비스
"""
import logging
import time
from typing import Dict, Any, Optional
import asyncio

# API 폴더 내부 모듈 import
from ..voice_pipeline import VoicePipeline
from ..config import settings
from ..utils.error_handlers import (
    ModelNotLoadedError,
    STTError,
    NLUError,
    TTSError,
    PipelineError
)

logger = logging.getLogger(__name__)


class VoiceService:
    """
    VoicePipeline 싱글톤 래퍼
    FastAPI 수명주기에서 한 번만 초기화됨
    """

    _instance: Optional['VoiceService'] = None
    _pipeline: Optional[VoicePipeline] = None
    _initialized: bool = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """초기화 (싱글톤이므로 한 번만 실행)"""
        if not self._initialized:
            self._initialize_pipeline()

    def _initialize_pipeline(self) -> None:
        """VoicePipeline 초기화"""
        try:
            logger.info("Initializing VoicePipeline...")
            self._pipeline = VoicePipeline(config_path=settings.CONFIG_PATH)
            self._initialized = True
            logger.info("VoicePipeline initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize VoicePipeline: {e}")
            self._initialized = False
            raise ModelNotLoadedError(
                message="음성 파이프라인 초기화 실패",
                details=str(e)
            )

    @property
    def pipeline(self) -> VoicePipeline:
        """VoicePipeline 인스턴스 반환"""
        if not self._initialized or self._pipeline is None:
            raise ModelNotLoadedError(
                message="VoicePipeline이 초기화되지 않았습니다"
            )
        return self._pipeline

    def is_ready(self) -> bool:
        """서비스 준비 상태 확인"""
        return self._initialized and self._pipeline is not None

    def get_status(self) -> Dict[str, Any]:
        """시스템 상태 반환"""
        if not self.is_ready():
            return {
                "initialized": False,
                "components": {
                    "stt": False,
                    "nlu": False,
                    "tts": False
                }
            }

        try:
            pipeline_status = self._pipeline.get_status()
            components = pipeline_status.get("components", {})

            return {
                "initialized": True,
                "components": {
                    "stt": components.get("stt_ready", False),
                    "nlu": components.get("nlu_ready", False),
                    "tts": components.get("tts_ready", False)
                }
            }
        except Exception as e:
            logger.error(f"Failed to get status: {e}")
            return {
                "initialized": self._initialized,
                "components": {
                    "stt": False,
                    "nlu": False,
                    "tts": False
                }
            }

    async def transcribe(
        self,
        audio_file_path: str,
        language: str = "ko"
    ) -> Dict[str, Any]:
        """
        음성을 텍스트로 변환
        """
        if not self.is_ready():
            raise ModelNotLoadedError()

        try:
            start_time = time.time()

            # STT 수행 (동기 함수를 비동기로 실행)
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                None,
                self._pipeline.stt.transcribe_file,
                audio_file_path
            )

            transcription = result.get("text", "") if isinstance(result, dict) else str(result)

            duration = time.time() - start_time

            # Check if STT returned an error
            if isinstance(result, dict) and "error" in result and result["error"] is not None:
                error_msg = result.get("error", "Unknown error")
                logger.error(f"STT returned error: {error_msg}")
                raise STTError(
                    message="음성 인식 중 오류 발생",
                    details=f"STT Error: {error_msg}"
                )

            if not transcription or transcription.strip() == "":
                # Get more context from result
                audio_info = ""
                if isinstance(result, dict):
                    confidence = result.get("confidence", "unknown")
                    language = result.get("language", "unknown")
                    audio_info = f" (confidence: {confidence}, language: {language})"
                
                logger.warning(f"Empty transcription returned{audio_info}")
                logger.warning(f"Full STT result: {result}")
                
                raise STTError(
                    message="음성을 인식할 수 없습니다",
                    details=f"No speech detected in audio. STT returned empty text{audio_info}. Check if audio file contains clear speech."
                )

            return {
                "transcription": transcription,
                "language": language,
                "duration": round(duration, 2)
            }

        except STTError:
            raise
        except Exception as e:
            logger.error(f"STT failed: {e}")
            raise STTError(
                message="음성 인식 중 오류 발생",
                details=str(e)
            )

    async def live_transcribe(
        self,
        timeout: int = 5,
        phrase_time_limit: int = 10
    ) -> Dict[str, Any]:
        """
        마이크에서 직접 음성을 입력받아 텍스트로 변환
        """
        if not self.is_ready():
            raise ModelNotLoadedError()

        try:
            # 실시간 음성 인식 수행 (동기 함수를 비동기로 실행)
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                None,
                self._pipeline.stt.listen_and_transcribe,
                timeout,
                phrase_time_limit
            )

            # 에러 체크
            if "error" in result and result["error"]:
                logger.error(f"Live STT error: {result['error']}")
                return result

            # 빈 텍스트 체크
            if not result.get("text"):
                logger.warning("No speech detected in live recording")
                return {
                    "text": "",
                    "confidence": 0.0,
                    "language": "ko",
                    "error": "No speech detected"
                }

            logger.info(f"Live STT success: '{result['text']}'")
            return result

        except Exception as e:
            logger.error(f"Live STT failed: {e}")
            return {
                "text": "",
                "confidence": 0.0,
                "language": "ko",
                "error": str(e)
            }

    async def list_audio_devices(self) -> list:
        """
        사용 가능한 오디오 입력 장치 목록 반환
        """
        if not self.is_ready():
            raise ModelNotLoadedError()

        try:
            # AudioProcessor에서 장치 목록 가져오기
            loop = asyncio.get_event_loop()
            devices = await loop.run_in_executor(
                None,
                self._pipeline.audio_processor.list_audio_devices
            )

            return devices

        except Exception as e:
            logger.error(f"Failed to list audio devices: {e}")
            return []

    async def understand(
        self,
        text: str,
        context: Optional[list] = None
    ) -> Dict[str, Any]:
        """
        텍스트를 이해하고 응답 생성
        """
        if not self.is_ready():
            raise ModelNotLoadedError()

        try:
            # NLU 수행 (동기 함수를 비동기로 실행)
            loop = asyncio.get_event_loop()
            nlu_result = await loop.run_in_executor(
                None,
                self._pipeline.nlu.understand,
                text
            )

            # 기본 응답 구조 보장
            return {
                "intent": nlu_result.get("intent", "unknown"),
                "entities": nlu_result.get("entities", {}),
                "response": nlu_result.get("response", "응답을 생성할 수 없습니다."),
                "confidence": nlu_result.get("confidence", 0.0)
            }

        except Exception as e:
            logger.error(f"NLU failed: {e}")
            raise NLUError(
                message="자연어 이해 중 오류 발생",
                details=str(e)
            )

    async def synthesize(
        self,
        text: str,
        voice: str = "ko-KR-SunHiNeural",
        rate: str = "+0%",
        pitch: str = "+0Hz"
    ) -> bytes:
        """
        텍스트를 음성으로 합성
        """
        if not self.is_ready():
            raise ModelNotLoadedError()

        try:
            # TTS 수행 (동기 함수를 비동기로 실행)
            loop = asyncio.get_event_loop()
            audio_data = await loop.run_in_executor(
                None,
                lambda: self._pipeline.tts.synthesize(
                    text=text,
                    output_path=None,
                    voice=voice,
                    rate=rate,
                    pitch=pitch
                )
            )

            if not audio_data:
                raise TTSError(
                    message="음성 합성 실패",
                    details="No audio data generated"
                )

            return audio_data

        except TTSError:
            raise
        except Exception as e:
            logger.error(f"TTS failed: {e}")
            raise TTSError(
                message="음성 합성 중 오류 발생",
                details=str(e)
            )

    async def process_pipeline(
        self,
        audio_file_path: str,
        language: str = "ko",
        voice: str = "ko-KR-SunHiNeural",
        return_audio: bool = True
    ) -> Dict[str, Any]:
        """
        전체 파이프라인 실행
        """
        if not self.is_ready():
            raise ModelNotLoadedError()

        try:
            total_start = time.time()

            # STT
            stt_start = time.time()
            stt_result = await self.transcribe(audio_file_path, language)
            stt_time = time.time() - stt_start
            transcription = stt_result["transcription"]

            # NLU
            nlu_start = time.time()
            nlu_result = await self.understand(transcription)
            nlu_time = time.time() - nlu_start
            response_text = nlu_result["response"]

            # TTS (선택적)
            audio_data = None
            tts_time = None
            if return_audio:
                tts_start = time.time()
                audio_data = await self.synthesize(response_text, voice)
                tts_time = time.time() - tts_start

            total_time = time.time() - total_start

            result = {
                "transcription": transcription,
                "intent": nlu_result["intent"],
                "response_text": response_text,
                "entities": nlu_result.get("entities", {}),
                "processing_time": {
                    "stt": round(stt_time, 2),
                    "nlu": round(nlu_time, 2),
                    "tts": round(tts_time, 2) if tts_time else None,
                    "total": round(total_time, 2)
                }
            }

            if audio_data:
                result["audio_data"] = audio_data

            return result

        except (STTError, NLUError, TTSError):
            raise
        except Exception as e:
            logger.error(f"Pipeline processing failed: {e}")
            raise PipelineError(
                message="파이프라인 처리 중 오류 발생",
                details=str(e)
            )

    def shutdown(self) -> None:
        """서비스 종료"""
        try:
            logger.info("Shutting down VoiceService...")
            if self._pipeline:
                # VoicePipeline 정리 (있다면)
                pass
            self._initialized = False
            logger.info("VoiceService shutdown complete")
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")


# 전역 서비스 인스턴스
voice_service = VoiceService()
