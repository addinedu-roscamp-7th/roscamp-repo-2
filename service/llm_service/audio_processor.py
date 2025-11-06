import numpy as np
import logging
import threading
import queue
import time
from typing import Optional, Callable, Dict, Any, List
from pathlib import Path

try:
    import pyaudio
    import webrtcvad
    import librosa
    AUDIO_LIBS_AVAILABLE = True
except ImportError:
    AUDIO_LIBS_AVAILABLE = False
    logging.warning("Audio libraries not available. Install with: pip install pyaudio webrtcvad librosa")

class AudioProcessor:
    def __init__(
        self,
        sample_rate: int = 16000,
        chunk_size: int = 4096,
        channels: int = 1,
        vad_mode: int = 2,  # VAD aggressiveness (0-3)
        silence_threshold: float = 1.0,  # seconds
        max_recording_time: float = 10.0  # seconds
    ):
        """
        Initialize Audio Processor with VAD
        """
        self.logger = logging.getLogger(__name__)

        if not AUDIO_LIBS_AVAILABLE:
            self.logger.warning("Audio libraries not available. Audio input will be disabled.")

        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.channels = channels
        self.vad_mode = vad_mode
        self.silence_threshold = silence_threshold
        self.max_recording_time = max_recording_time

        # Audio format
        if AUDIO_LIBS_AVAILABLE:
            self.format = pyaudio.paInt16
        else:
            self.format = None
        self.bytes_per_sample = 2

        # Initialize PyAudio
        if AUDIO_LIBS_AVAILABLE:
            self.audio = pyaudio.PyAudio()
            # Initialize VAD
            self.vad = webrtcvad.Vad(vad_mode)
        else:
            self.audio = None
            self.vad = None

        # Recording state
        self.is_recording = False
        self.audio_buffer = queue.Queue()
        self.recording_thread = None

        self.logger.info(f"Audio processor initialized - SR: {sample_rate}, Chunk: {chunk_size}")

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """PyAudio callback for continuous audio capture"""
        if self.is_recording:
            self.audio_buffer.put(in_data)
        return (None, pyaudio.paContinue)

    def start_continuous_listening(self, callback: Callable[[np.ndarray], None]):
        """
        Start continuous audio listening with VAD
        """
        try:
            self.logger.info("Starting continuous audio listening")

            stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size,
                stream_callback=self._audio_callback
            )

            self.is_recording = True
            stream.start_stream()

            # Audio processing thread
            def process_audio():
                audio_frames = []
                silence_start = None
                is_speaking = False

                while self.is_recording:
                    try:
                        # Get audio chunk
                        if not self.audio_buffer.empty():
                            audio_chunk = self.audio_buffer.get(timeout=0.1)
                            audio_frames.append(audio_chunk)

                            # Convert to numpy array for VAD
                            audio_np = np.frombuffer(audio_chunk, dtype=np.int16)

                            # Check for voice activity
                            if self._is_speech(audio_chunk):
                                if not is_speaking:
                                    self.logger.debug("Speech detected")
                                    is_speaking = True
                                silence_start = None
                            else:
                                if is_speaking and silence_start is None:
                                    silence_start = time.time()

                            # Check for end of speech
                            if (is_speaking and silence_start and
                                time.time() - silence_start > self.silence_threshold):

                                # Process accumulated audio
                                if audio_frames:
                                    complete_audio = b''.join(audio_frames)
                                    audio_np = np.frombuffer(complete_audio, dtype=np.int16)
                                    audio_float = audio_np.astype(np.float32) / 32768.0

                                    # Call the callback with processed audio
                                    callback(audio_float)

                                # Reset for next speech segment
                                audio_frames = []
                                is_speaking = False
                                silence_start = None

                            # Limit buffer size to prevent memory issues
                            max_frames = int(self.max_recording_time * self.sample_rate / self.chunk_size)
                            if len(audio_frames) > max_frames:
                                audio_frames = audio_frames[-max_frames:]

                        else:
                            time.sleep(0.01)

                    except queue.Empty:
                        continue
                    except Exception as e:
                        self.logger.error(f"Error in audio processing: {e}")

            self.recording_thread = threading.Thread(target=process_audio, daemon=True)
            self.recording_thread.start()

            return stream

        except Exception as e:
            self.logger.error(f"Failed to start continuous listening: {e}")
            raise

    def stop_continuous_listening(self, stream):
        """Stop continuous audio listening"""
        try:
            self.is_recording = False
            if stream:
                stream.stop_stream()
                stream.close()
            self.logger.info("Stopped continuous audio listening")

        except Exception as e:
            self.logger.error(f"Error stopping audio listening: {e}")

    def record_audio(self, duration: Optional[float] = None) -> np.ndarray:
        """
        Record audio for specified duration or until silence
        """
        if not AUDIO_LIBS_AVAILABLE:
            self.logger.error("Audio libraries not available for recording")
            return np.array([], dtype=np.float32)

        try:
            self.logger.info(f"Starting audio recording (duration: {duration})")

            stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            audio_frames = []
            start_time = time.time()
            silence_start = None
            is_speaking = False

            while True:
                # Read audio chunk
                audio_chunk = stream.read(self.chunk_size, exception_on_overflow=False)
                audio_frames.append(audio_chunk)

                # Check duration limit
                if duration and time.time() - start_time > duration:
                    break

                # Voice activity detection
                if self._is_speech(audio_chunk):
                    if not is_speaking:
                        self.logger.debug("Speech started")
                        is_speaking = True
                    silence_start = None
                else:
                    if is_speaking and silence_start is None:
                        silence_start = time.time()

                # Stop on silence (only if speech was detected)
                if (is_speaking and silence_start and
                    time.time() - silence_start > self.silence_threshold):
                    self.logger.debug("Speech ended")
                    break

                # Maximum recording time safety
                if time.time() - start_time > self.max_recording_time:
                    self.logger.warning("Maximum recording time reached")
                    break

            stream.stop_stream()
            stream.close()

            # Convert to numpy array
            if audio_frames:
                complete_audio = b''.join(audio_frames)
                audio_np = np.frombuffer(complete_audio, dtype=np.int16)
                audio_float = audio_np.astype(np.float32) / 32768.0

                self.logger.info(f"Recording completed - {len(audio_float)} samples")
                return audio_float
            else:
                return np.array([], dtype=np.float32)

        except Exception as e:
            self.logger.error(f"Audio recording failed: {e}")
            return np.array([], dtype=np.float32)

    def _is_speech(self, audio_chunk: bytes) -> bool:
        """
        Check if audio chunk contains speech using VAD
        """
        try:
            # VAD requires specific frame sizes
            frame_size = int(self.sample_rate * 0.01)  # 10ms frame
            if len(audio_chunk) < frame_size * self.bytes_per_sample:
                return False

            # Trim to frame size
            frame_bytes = frame_size * self.bytes_per_sample
            audio_frame = audio_chunk[:frame_bytes]

            return self.vad.is_speech(audio_frame, self.sample_rate)

        except Exception:
            return False

    def load_audio_file(self, file_path: str) -> np.ndarray:
        """
        Load audio file and resample if necessary
        """
        try:
            audio_data, sr = librosa.load(
                file_path,
                sr=self.sample_rate,
                mono=True
            )

            self.logger.info(f"Loaded audio file: {file_path} ({len(audio_data)} samples)")
            return audio_data

        except Exception as e:
            self.logger.error(f"Failed to load audio file: {e}")
            return np.array([], dtype=np.float32)

    def save_audio_file(self, audio_data: np.ndarray, file_path: str) -> bool:
        """
        Save audio data to file
        """
        try:
            import soundfile as sf

            sf.write(file_path, audio_data, self.sample_rate)
            self.logger.info(f"Audio saved to: {file_path}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save audio file: {e}")
            return False

    def get_audio_info(self) -> Dict[str, Any]:
        """Get audio processor configuration"""
        return {
            "sample_rate": self.sample_rate,
            "chunk_size": self.chunk_size,
            "channels": self.channels,
            "vad_mode": self.vad_mode,
            "silence_threshold": self.silence_threshold,
            "max_recording_time": self.max_recording_time
        }

    def list_audio_devices(self) -> List[Dict[str, Any]]:
        """List available audio input devices"""
        devices = []
        try:
            for i in range(self.audio.get_device_count()):
                device_info = self.audio.get_device_info_by_index(i)
                if device_info['maxInputChannels'] > 0:
                    devices.append({
                        'index': i,
                        'name': device_info['name'],
                        'channels': device_info['maxInputChannels'],
                        'sample_rate': device_info['defaultSampleRate']
                    })

            return devices

        except Exception as e:
            self.logger.error(f"Failed to list audio devices: {e}")
            return []

    def __del__(self):
        """Cleanup audio resources"""
        try:
            if hasattr(self, 'audio'):
                self.audio.terminate()
        except:
            pass