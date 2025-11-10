import requests
from typing import Optional

# pip install pygame
try:
    from audio_playback import play_audio_bytes, play_audio_file
    AUDIO_PLAYBACK_AVAILABLE = True
except ImportError:
    AUDIO_PLAYBACK_AVAILABLE = False
    play_audio_bytes = None
    play_audio_file = None


# ==================== 고급 클라이언트 클래스 ====================

class DobyVoiceAdvancedClient:
    """Doby Voice API 고급 클라이언트 - 음성 설정, 속도, 볼륨 조절 지원"""

    def __init__(
            self,
            host: str = "192.168.0.191",
            port: int = 8000,
            voice: str = "ko-KR",
            rate: float = 1.0,
            volume: float = 1.0
    ):
        """
        고급 클라이언트 초기화
        """
        self.base_url = f"http://{host}:{port}"
        self.api_prefix = "/api/v1"
        self.timeout = 120
        """
        음성 설정 초기화
        """
        self.rate = max(0.5, min(2.0, rate))  # 0.5 ~ 2.0 범위로 제한
        self.volume = max(0.0, min(1.0, volume))  # 0.0 ~ 1.0 범위로 제한
        self.voice = voice

    def text_to_speech(
            self,
            text: str,
    ) -> Optional[bytes]:
        """
        TTS: 텍스트를 음성으로 변환
        """
        # 현재 설정의 복사본 만들기

        try:
            url = f"{self.base_url}{self.api_prefix}/tts"
            data = {
                'text': text,
                'voice': self.voice
            }

            print(f"[TTS]   음성 설정")
            print(f"      - 음성: {self.voice}")
            print(f"      - 속도: {self.rate}x")
            print(f"      - 볼륨: {self.volume * 100:.0f}%")
            print(f"[TTS] 텍스트를 음성으로 변환 중...")
            print(f"      - 입력: {text[:50]}{'...' if len(text) > 50 else ''}")

            response = requests.post(url, json=data, timeout=self.timeout)
            response.raise_for_status()

            audio_bytes = response.content
            print(f"[TTS]  변환 완료 (크기: {len(audio_bytes)} bytes)")

            # 스피커 출력
            if AUDIO_PLAYBACK_AVAILABLE and play_audio_bytes:
                try:
                    print(f"[TTS] 🔊 스피커 출력 시작...")
                    play_audio_bytes(audio_bytes, format_="wav", blocking=True)
                    print(f"[TTS] ✅ 재생 완료")
                except Exception as e:
                    print(f"[TTS] ⚠️  재생 실패: {e}")
            else:
                print(f"[TTS] ⚠️  오디오 재생 라이브러리가 없습니다")

        except requests.exceptions.ConnectionError:
            print(f"[TTS] ❌ API 서버에 연결할 수 없습니다")
            return None
        except Exception as e:
            print(f"[TTS] ❌ 오류: {type(e).__name__} - {e}")
            return None

if __name__ == "__main__":
    client = DobyVoiceAdvancedClient()
    client.text_to_speech("제조가 완료되었습니다")
