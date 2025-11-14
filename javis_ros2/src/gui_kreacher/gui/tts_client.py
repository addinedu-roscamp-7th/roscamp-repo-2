"""
의존성 설치:
    pip install requests pyaudio

사용 예제:
    from tts_client import DobyVoiceAdvancedClient
    DobyVoiceAdvancedClient("안녕하세요")
"""

import requests
import wave
import pyaudio
from pathlib import Path
from typing import Optional, Dict, Any


def DobyVoiceAdvancedClient(
    text: str,
    output_file: Optional[str] = None,
    api_url: str = "http://192.168.0.191:8000",
    voice: str = "default",
    language_id: int = 0,
    play_audio: bool = True
) -> Dict[str, Any]:
    """
    TTS API를 호출하여 음성을 생성하고 재생합니다.
    """
    endpoint = f"{api_url}/api/v1/tts"
    
    # API 요청
    payload = {
        "text": text,
        "voice": voice,
        "language_id": language_id
    }
    
    try:
        print(f"🎤 TTS 요청: {text[:50]}...")
        response = requests.post(endpoint, json=payload, timeout=120)
        
        if response.status_code != 200:
            return {
                "status": "error",
                "message": f"API 오류: {response.status_code}",
                "details": response.text
            }
        
        # 오디오 파일 저장
        if output_file is None:
            output_file = "/tmp/tts_output.wav"
        
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, 'wb') as f:
            f.write(response.content)
        
        print(f"✅ 오디오 저장: {output_file}")
        
        # 오디오 재생
        if play_audio:
            play_wav(output_file)
        
        return {
            "status": "success",
            "file_path": output_file,
            "message": "TTS 성공"
        }
        
    except requests.exceptions.Timeout:
        return {
            "status": "error",
            "message": "API 타임아웃 (120초 초과)"
        }
    except Exception as e:
        return {
            "status": "error",
            "message": f"오류 발생: {str(e)}"
        }


def play_wav(file_path: str):
    """
    PyAudio를 사용하여 WAV 파일을 재생합니다.
    
    Args:
        file_path: WAV 파일 경로
    """
    try:
        print(f"🔊 오디오 재생 중...")
        
        # WAV 파일 열기
        wf = wave.open(file_path, 'rb')
        
        # PyAudio 초기화
        p = pyaudio.PyAudio()
        
        # 스트림 열기
        stream = p.open(
            format=p.get_format_from_width(wf.getsampwidth()),
            channels=wf.getnchannels(),
            rate=wf.getframerate(),
            output=True
        )
        
        # 데이터 읽기 및 재생
        chunk_size = 1024
        data = wf.readframes(chunk_size)
        
        while data:
            stream.write(data)
            data = wf.readframes(chunk_size)
        
        # 정리
        stream.stop_stream()
        stream.close()
        p.terminate()
        wf.close()
        
        print("✅ 재생 완료")
        
    except Exception as e:
        print(f"❌ 재생 오류: {e}")



class LinuxTTSClient:   
    def __init__(self, api_url: str = "http://localhost:8000"):
        """
        Args:
            api_url: API 서버 URL
        """
        self.api_url = api_url
        self.pyaudio_instance = None
    
    def synthesize(
        self,
        text: str,
        voice: str = "default",
        language_id: int = 0
    ) -> Optional[bytes]:
        """
        텍스트를 음성으로 변환합니다.
        
        Returns:
            WAV 오디오 바이트 또는 None (오류 시)
        """
        endpoint = f"{self.api_url}/api/v1/tts"
        payload = {
            "text": text,
            "voice": voice,
            "language_id": language_id
        }
        
        try:
            response = requests.post(endpoint, json=payload, timeout=120)
            if response.status_code == 200:
                return response.content
            else:
                print(f"❌ API 오류: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ 오류: {e}")
            return None
    
    def play_audio(self, audio_data: bytes):
        """
        오디오 데이터를 재생합니다.
        
        Args:
            audio_data: WAV 형식의 오디오 바이트
        """
        import io
        
        try:
            # 바이트를 파일처럼 다루기
            audio_io = io.BytesIO(audio_data)
            wf = wave.open(audio_io, 'rb')
            
            # PyAudio 초기화
            if self.pyaudio_instance is None:
                self.pyaudio_instance = pyaudio.PyAudio()
            
            p = self.pyaudio_instance
            
            # 스트림 열기
            stream = p.open(
                format=p.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True
            )
            
            # 재생
            chunk_size = 1024
            data = wf.readframes(chunk_size)
            
            while data:
                stream.write(data)
                data = wf.readframes(chunk_size)
            
            # 정리
            stream.stop_stream()
            stream.close()
            wf.close()
            
        except Exception as e:
            print(f"❌ 재생 오류: {e}")
    
    def speak(
        self,
        text: str,
        save_to: Optional[str] = None,
        voice: str = "default",
        language_id: int = 0
    ) -> bool:
        """
        """
        print(f"🎤 TTS: {text[:50]}...")
        
        audio_data = self.synthesize(text, voice, language_id)
        if audio_data is None:
            return False
        
        # 파일 저장
        if save_to:
            Path(save_to).parent.mkdir(parents=True, exist_ok=True)
            with open(save_to, 'wb') as f:
                f.write(audio_data)
            print(f"💾 저장: {save_to}")
        
        # 재생
        print("🔊 재생 중...")
        self.play_audio(audio_data)
        print("✅ 완료")
        
        return True
    
    def __del__(self):
        """리소스 정리"""
        if self.pyaudio_instance:
            self.pyaudio_instance.terminate()


# 테스트 코드
if __name__ == "__main__":
    import sys
    
    print("=== Linux TTS 클라이언트 테스트 ===\n")
    
    # 1. 간단한 함수 사용
    print("1️⃣ 함수 방식 테스트")
    result = DobyVoiceAdvancedClient(
        "안녕하세요, 음성 테스트입니다.",
        output_file="/tmp/test1.wav"
    )
    print(f"결과: {result}\n")
    print("\n✅ 모든 테스트 완료")
