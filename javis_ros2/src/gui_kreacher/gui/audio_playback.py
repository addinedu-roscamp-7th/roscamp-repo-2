"""
오디오 재생 유틸리티 - WAV/MP3 스피커 출력
"""

import logging
import io
import os
from typing import Optional, Union
from pathlib import Path

logger = logging.getLogger(__name__)

# 재생 라이브러리 가용성 확인
PYGAME_AVAILABLE = False
SIMPLEAUDIO_AVAILABLE = False
PYAUDIO_AVAILABLE = False
WINSOUND_AVAILABLE = False

try:
    import pygame
    pygame.mixer.init(frequency=24000, size=-16, channels=2, buffer=512)
    PYGAME_AVAILABLE = True
    logger.info("✓ pygame 사용 가능")
except ImportError:
    logger.debug("pygame 미설치")
except Exception as e:
    logger.warning(f"pygame 초기화 실패: {e}")

try:
    import simpleaudio
    SIMPLEAUDIO_AVAILABLE = True
    logger.info("✓ simpleaudio 사용 가능")
except ImportError:
    logger.debug("simpleaudio 미설치")

try:
    import pyaudio
    PYAUDIO_AVAILABLE = True
    logger.info("✓ pyaudio 사용 가능")
except ImportError:
    logger.debug("pyaudio 미설치")

try:
    import winsound
    WINSOUND_AVAILABLE = True
    logger.info("✓ winsound 사용 가능 (Windows)")
except ImportError:
    logger.debug("winsound 미설치 (Windows 전용)")


class AudioPlayer:
    """
    오디오 파일/데이터 재생 관리자
    """
    
    def __init__(self, backend: Optional[str] = None):
        """
        AudioPlayer 초기화
        """
        self.logger = logging.getLogger(__name__)
        self.backend = self._select_backend(backend)
        self.current_sound = None
        
        self.logger.info(f"🔊 AudioPlayer 초기화 - 백엔드: {self.backend}")
    
    def _select_backend(self, preferred: Optional[str] = None) -> str:
        """
        최적의 재생 백엔드 선택
        """
        if preferred:
            if preferred == "pygame" and PYGAME_AVAILABLE:
                return "pygame"
            elif preferred == "simpleaudio" and SIMPLEAUDIO_AVAILABLE:
                return "simpleaudio"
            elif preferred == "pyaudio" and PYAUDIO_AVAILABLE:
                return "pyaudio"
            elif preferred == "winsound" and WINSOUND_AVAILABLE:
                return "winsound"
        
        # 자동 선택 (우선순위)
        if PYGAME_AVAILABLE:
            return "pygame"
        elif WINSOUND_AVAILABLE:
            return "winsound"
        elif SIMPLEAUDIO_AVAILABLE:
            return "simpleaudio"
        elif PYAUDIO_AVAILABLE:
            return "pyaudio"
        else:
            return "none"
    
    def play_file(self, file_path: str, blocking: bool = True, wait: bool = True) -> bool:
        """
        파일에서 오디오 재생
        """
        try:
            if not os.path.exists(file_path):
                self.logger.error(f"❌ 파일 없음: {file_path}")
                return False
            
            self.logger.info(f"🔊 재생 중: {Path(file_path).name} ({self.backend})")
            
            if self.backend == "pygame":
                return self._play_pygame(file_path, blocking=blocking)
            elif self.backend == "winsound":
                return self._play_winsound(file_path)
            elif self.backend == "simpleaudio":
                return self._play_simpleaudio(file_path, wait=wait)
            elif self.backend == "pyaudio":
                return self._play_pyaudio(file_path, blocking=blocking)
            else:
                self.logger.error("❌ 사용 가능한 재생 백엔드 없음")
                return False
        
        except Exception as e:
            self.logger.error(f"❌ 재생 실패: {e}")
            return False
    
    def play_bytes(self, audio_data: bytes, format_: str = "wav", 
                   blocking: bool = True, wait: bool = True) -> bool:
        """
        바이트 데이터에서 오디오 재생
        """
        try:
            if not audio_data:
                self.logger.error("❌ 오디오 데이터 없음")
                return False
            
            self.logger.info(f"🔊 메모리 재생 중 ({format_}, {len(audio_data)} bytes, {self.backend})")
            
            if self.backend == "pygame":
                return self._play_bytes_pygame(audio_data, format_=format_, blocking=blocking)
            elif self.backend == "winsound":
                return self._play_bytes_winsound(audio_data)
            elif self.backend == "simpleaudio":
                return self._play_bytes_simpleaudio(audio_data, format_=format_, wait=wait)
            elif self.backend == "pyaudio":
                return self._play_bytes_pyaudio(audio_data, format_=format_, blocking=blocking)
            else:
                self.logger.error("❌ 사용 가능한 재생 백엔드 없음")
                return False
        
        except Exception as e:
            self.logger.error(f"❌ 메모리 재생 실패: {e}")
            return False
    
    # ============ pygame 백엔드 ============
    
    def _play_pygame(self, file_path: str, blocking: bool = True) -> bool:
        """pygame을 사용한 파일 재생"""
        try:
            import pygame
            
            # pygame mixer 상태 확인 및 재초기화
            if not pygame.mixer.get_init():
                pygame.mixer.init(frequency=24000, size=-16, channels=2, buffer=512)
                self.logger.debug("pygame mixer 재초기화됨")
            
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            
            if blocking:
                while pygame.mixer.music.get_busy():
                    pygame.time.delay(100)
                self.logger.info("✅ 재생 완료")
            
            return True
        except Exception as e:
            self.logger.error(f"❌ pygame 재생 실패: {e}")
            return False
    
    def _play_bytes_pygame(self, audio_data: bytes, format_: str = "wav", blocking: bool = True) -> bool:
        """pygame을 사용한 바이트 데이터 재생"""
        try:
            import pygame

            # pygame mixer 상태 확인 및 재초기화
            if not pygame.mixer.get_init():
                pygame.mixer.init(frequency=24000, size=-16, channels=2, buffer=512)
                self.logger.debug("pygame mixer 재초기화됨")

            audio_buffer = io.BytesIO(audio_data)
            sound = pygame.mixer.Sound(audio_buffer)
            channel = sound.play()
            self.current_sound = sound

            if blocking and channel:
                while channel.get_busy():
                    pygame.time.delay(100)
                self.logger.info("✅ 재생 완료")

            return True
        except Exception as e:
            self.logger.error(f"❌ pygame 바이트 재생 실패: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    # ============ winsound 백엔드 (Windows) ============
    
    def _play_winsound(self, file_path: str) -> bool:
        """winsound를 사용한 파일 재생 (Windows만)"""
        try:
            import winsound
            
            winsound.PlaySound(file_path, winsound.SND_FILENAME | winsound.SND_WAIT)
            self.logger.info("✅ 재생 완료")
            return True
        except Exception as e:
            self.logger.error(f"❌ winsound 재생 실패: {e}")
            return False
    
    def _play_bytes_winsound(self, audio_data: bytes) -> bool:
        """winsound를 사용한 바이트 데이터 재생 (Windows만)"""
        try:
            import winsound
            import tempfile
            
            # 임시 파일에 저장
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                tmp.write(audio_data)
                tmp_path = tmp.name
            
            try:
                winsound.PlaySound(tmp_path, winsound.SND_FILENAME | winsound.SND_WAIT)
                self.logger.info("✅ 재생 완료")
                return True
            finally:
                os.unlink(tmp_path)
        
        except Exception as e:
            self.logger.error(f"❌ winsound 바이트 재생 실패: {e}")
            return False
    
    # ============ simpleaudio 백엔드 ============
    
    def _play_simpleaudio(self, file_path: str, wait: bool = True) -> bool:
        """simpleaudio를 사용한 파일 재생"""
        try:
            import scipy.io.wavfile as wavfile
            import simpleaudio
            
            sample_rate, audio_data = wavfile.read(file_path)
            play_obj = simpleaudio.play_buffer(audio_data, 1, 2, sample_rate)
            
            if wait:
                play_obj.wait_done()
                self.logger.info("✅ 재생 완료")
            else:
                self.current_sound = play_obj
            
            return True
        except Exception as e:
            self.logger.error(f"❌ simpleaudio 재생 실패: {e}")
            return False
    
    def _play_bytes_simpleaudio(self, audio_data: bytes, format_: str = "wav", wait: bool = True) -> bool:
        """simpleaudio를 사용한 바이트 데이터 재생"""
        try:
            import scipy.io.wavfile as wavfile
            import simpleaudio
            
            audio_buffer = io.BytesIO(audio_data)
            sample_rate, audio_data_np = wavfile.read(audio_buffer)
            play_obj = simpleaudio.play_buffer(audio_data_np, 1, 2, sample_rate)
            
            if wait:
                play_obj.wait_done()
                self.logger.info("✅ 재생 완료")
            else:
                self.current_sound = play_obj
            
            return True
        except Exception as e:
            self.logger.error(f"❌ simpleaudio 바이트 재생 실패: {e}")
            return False
    
    # ============ pyaudio 백엔드 ============
    
    def _play_pyaudio(self, file_path: str, blocking: bool = True) -> bool:
        """pyaudio를 사용한 파일 재생"""
        try:
            import scipy.io.wavfile as wavfile
            import pyaudio
            
            sample_rate, audio_data = wavfile.read(file_path)
            
            p = pyaudio.PyAudio()
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1 if len(audio_data.shape) == 1 else audio_data.shape[1],
                rate=sample_rate,
                output=True
            )
            
            stream.write(audio_data.tobytes())
            stream.stop_stream()
            stream.close()
            p.terminate()
            
            self.logger.info("✅ 재생 완료")
            return True
        except Exception as e:
            self.logger.error(f"❌ pyaudio 재생 실패: {e}")
            return False
    
    def _play_bytes_pyaudio(self, audio_data: bytes, format_: str = "wav", blocking: bool = True) -> bool:
        """pyaudio를 사용한 바이트 데이터 재생"""
        try:
            import scipy.io.wavfile as wavfile
            import pyaudio
            
            audio_buffer = io.BytesIO(audio_data)
            sample_rate, audio_data_np = wavfile.read(audio_buffer)
            
            p = pyaudio.PyAudio()
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1 if len(audio_data_np.shape) == 1 else audio_data_np.shape[1],
                rate=sample_rate,
                output=True
            )
            
            stream.write(audio_data_np.tobytes())
            stream.stop_stream()
            stream.close()
            p.terminate()
            
            self.logger.info("✅ 재생 완료")
            return True
        except Exception as e:
            self.logger.error(f"❌ pyaudio 바이트 재생 실패: {e}")
            return False
    
    def stop(self):
        """재생 중지"""
        try:
            if self.backend == "pygame":
                import pygame
                pygame.mixer.stop()
            elif self.backend == "simpleaudio" and self.current_sound:
                self.current_sound.stop()
            
            self.logger.info("⏹️  재생 중지")
        except Exception as e:
            self.logger.warning(f"⚠️  중지 실패: {e}")
    
    def shutdown(self):
        """리소스 정리"""
        self.stop()
        self.logger.info("🔌 AudioPlayer 종료")


# ============ 편의 함수 ============

def play_audio_file(file_path: str, blocking: bool = True, backend: Optional[str] = None) -> bool:
    """
    오디오 파일을 스피커로 재생하는 간단한 함수
    """
    player = AudioPlayer(backend=backend)
    return player.play_file(file_path, blocking=blocking)


def play_audio_bytes(audio_data: bytes, format_: str = "wav", 
                     blocking: bool = True, backend: Optional[str] = None) -> bool:
    """
    오디오 바이트 데이터를 스피커로 재생하는 간단한 함수
    Example:
        >>> import base64
        >>> audio = base64.b64decode(response['audio_base64'])
        >>> play_audio_bytes(audio, format_="wav")
    """
    player = AudioPlayer(backend=backend)
    return player.play_bytes(audio_data, format_=format_, blocking=blocking)
