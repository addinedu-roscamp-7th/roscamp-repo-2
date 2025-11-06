import logging
import tempfile
import os
import threading
import time
import io
from pathlib import Path
from typing import Optional

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    logging.warning("pygame not installed. Audio playback will be limited.")

try:
    import pydub
    from pydub import AudioSegment
    from pydub.playback import play
    PYDUB_AVAILABLE = True
except ImportError:
    PYDUB_AVAILABLE = False
    logging.warning("pydub not installed. Using pygame for audio playback.")

class AudioPlayer:
    def __init__(self):
        """Initialize audio player"""
        self.logger = logging.getLogger(__name__)
        self.is_playing = False
        self.current_thread = None

        # Initialize pygame mixer
        if PYGAME_AVAILABLE:
            try:
                pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=512)
                self.pygame_initialized = True
                self.logger.info("Pygame audio player initialized")
            except Exception as e:
                self.pygame_initialized = False
                self.logger.warning(f"Failed to initialize pygame: {e}")
        else:
            self.pygame_initialized = False

    def play_audio_data(self, audio_data: bytes, file_format: str = "wav") -> bool:
        """
        Play audio data
        """
        if not audio_data:
            self.logger.warning("No audio data to play")
            return False

        try:
            self.logger.info(f"AUDIO PLAYER: Starting playback of {len(audio_data)} bytes in {file_format} format")

            # Use pygame first (reliable on Windows)
            if self.pygame_initialized:
                self.logger.info("AUDIO PLAYER: Using pygame for playback")
                return self._play_with_pygame(audio_data, file_format)
            # Fallback to pydub (requires ffmpeg for MP3)
            elif PYDUB_AVAILABLE:
                self.logger.info("AUDIO PLAYER: Using pydub for playback")
                return self._play_with_pydub(audio_data, file_format)
            # Last resort: save to file and use system player
            else:
                self.logger.info("AUDIO PLAYER: Using system player for playback")
                return self._play_with_system(audio_data, file_format)

        except Exception as e:
            self.logger.error(f"Audio playback failed: {e}")
            return False

    def _play_with_pydub(self, audio_data: bytes, file_format: str) -> bool:
        """Play audio using pydub"""
        try:
            # For MP3 format, try to convert to WAV first (no ffmpeg needed for WAV)
            if file_format.lower() == "mp3":
                try:
                    # Try to load MP3 directly first
                    audio_segment = AudioSegment.from_file(
                        io.BytesIO(audio_data),
                        format="mp3"
                    )
                except Exception as mp3_error:
                    self.logger.warning(f"MP3 loading failed (missing ffmpeg): {mp3_error}")
                    # Fallback: save as temporary file and let pygame handle it
                    return False
            else:
                # For other formats (like WAV), load directly
                audio_segment = AudioSegment.from_file(
                    io.BytesIO(audio_data),
                    format=file_format
                )

            # Play in a separate thread to avoid blocking
            def play_thread():
                self.is_playing = True
                try:
                    play(audio_segment)
                    self.logger.info("Pydub playback completed successfully")
                except Exception as e:
                    self.logger.error(f"Pydub playback error: {e}")
                finally:
                    self.is_playing = False

            self.current_thread = threading.Thread(target=play_thread, daemon=True)
            self.current_thread.start()

            self.logger.info(f"Audio playback started with pydub ({file_format})")
            return True

        except Exception as e:
            self.logger.error(f"Pydub playback failed: {e}")
            return False

    def _play_with_pygame(self, audio_data: bytes, file_format: str) -> bool:
        """Play audio using pygame"""
        try:
            # Create temporary file in system temp directory
            temp_dir = tempfile.gettempdir()
            temp_file_path = os.path.join(temp_dir, f"dobi_tts_{int(time.time() * 1000)}.{file_format}")

            # Write audio data to file
            with open(temp_file_path, 'wb') as f:
                f.write(audio_data)

            self.logger.info(f"AUDIO FILE DEBUG: Created temp audio file: {temp_file_path}")
            self.logger.info(f"AUDIO FILE DEBUG: File size: {len(audio_data)} bytes, Format: {file_format}")

            # Log the stack trace to see what text was being processed
            import traceback
            self.logger.info(f"AUDIO FILE DEBUG: Call stack: {traceback.format_stack()[-3:-1]}")

            def play_thread():
                self.is_playing = True
                try:
                    self.logger.info(f"PYGAME: Starting audio playback from file: {temp_file_path}")
                    # For MP3 files, try pygame first, then system fallback
                    if file_format.lower() == "mp3":
                        try:
                            self.logger.info(f"PYGAME: Loading MP3 file: {temp_file_path}")
                            pygame.mixer.music.load(temp_file_path)
                            pygame.mixer.music.play()
                            self.logger.info("PYGAME: MP3 playback started")

                            # Wait for playback to finish
                            while pygame.mixer.music.get_busy():
                                time.sleep(0.1)
                            self.logger.info("PYGAME: MP3 playback completed")
                        except Exception as mp3_error:
                            self.logger.warning(f"Pygame MP3 playback failed: {mp3_error}, trying system player")
                            # Fallback to system player for MP3
                            self._play_with_system_sync(temp_file_path)
                    else:
                        # For WAV and other formats, use pygame
                        self.logger.info(f"PYGAME: Loading {file_format} file: {temp_file_path}")
                        pygame.mixer.music.load(temp_file_path)
                        pygame.mixer.music.play()
                        self.logger.info(f"PYGAME: {file_format} playback started")

                        # Wait for playback to finish
                        while pygame.mixer.music.get_busy():
                            time.sleep(0.1)
                        self.logger.info(f"PYGAME: {file_format} playback completed")

                    # Add small delay to ensure playback completion
                    time.sleep(0.2)

                except Exception as e:
                    self.logger.error(f"Pygame playback error: {e}")
                    # Final fallback to system player
                    try:
                        self._play_with_system_sync(temp_file_path)
                    except Exception as fallback_error:
                        self.logger.error(f"System fallback also failed: {fallback_error}")
                finally:
                    self.is_playing = False
                    # Clean up temp file after delay
                    time.sleep(0.5)
                    try:
                        if os.path.exists(temp_file_path):
                            os.unlink(temp_file_path)
                            self.logger.debug(f"Cleaned up temp file: {temp_file_path}")
                    except Exception as cleanup_error:
                        self.logger.warning(f"Failed to cleanup temp file: {cleanup_error}")

            self.current_thread = threading.Thread(target=play_thread, daemon=True)
            self.current_thread.start()

            self.logger.info(f"Audio playback started with pygame ({file_format})")
            return True

        except Exception as e:
            self.logger.error(f"Pygame playback failed: {e}")
            return False

    def _play_with_system_sync(self, file_path: str):
        """Play audio file using system player synchronously"""
        import subprocess
        import platform

        system = platform.system()
        file_extension = Path(file_path).suffix.lower()

        try:
            if system == "Windows":
                # For MP3 files, use Windows Media Player
                if file_extension == ".mp3":
                    # Try Windows Media Player first
                    try:
                        subprocess.run([
                            "powershell", "-c",
                            f"Add-Type -AssemblyName presentationCore; "
                            f"$mediaPlayer = New-Object system.windows.media.mediaplayer; "
                            f"$mediaPlayer.open([uri]'{file_path}'); "
                            f"$mediaPlayer.play(); "
                            f"Start-Sleep -Seconds 10"
                        ], check=True, capture_output=True, timeout=15)
                    except:
                        # Fallback to start command
                        subprocess.run(["start", "/wait", "", file_path], shell=True, timeout=15)
                else:
                    # For WAV files, use Media.SoundPlayer
                    subprocess.run([
                        "powershell", "-c",
                        f"(New-Object Media.SoundPlayer '{file_path}').PlaySync()"
                    ], check=True, capture_output=True, timeout=15)
            elif system == "Darwin":  # macOS
                subprocess.run(["afplay", file_path], check=True, timeout=15)
            elif system == "Linux":
                if file_extension == ".mp3":
                    subprocess.run(["mpg123", file_path], check=True, timeout=15)
                else:
                    subprocess.run(["aplay", file_path], check=True, timeout=15)

        except Exception as e:
            self.logger.error(f"System sync playback failed: {e}")
            # Final fallback - try opening with default application
            try:
                if system == "Windows":
                    os.startfile(file_path)
                elif system == "Darwin":
                    subprocess.run(["open", file_path], timeout=5)
                elif system == "Linux":
                    subprocess.run(["xdg-open", file_path], timeout=5)
                time.sleep(3)  # Give it time to play
            except Exception as final_error:
                self.logger.error(f"Final fallback failed: {final_error}")

    def _play_with_system(self, audio_data: bytes, file_format: str) -> bool:
        """Play audio using system default player"""
        try:
            # Save to temporary file
            with tempfile.NamedTemporaryFile(suffix=f".{file_format}", delete=False) as tmp_file:
                tmp_file.write(audio_data)
                tmp_file.flush()

                def play_thread():
                    self.is_playing = True
                    try:
                        import subprocess
                        import platform

                        system = platform.system()
                        if system == "Windows":
                            # Use Windows Media Player
                            subprocess.run(["start", "", tmp_file.name], shell=True, check=True)
                        elif system == "Darwin":  # macOS
                            subprocess.run(["afplay", tmp_file.name], check=True)
                        elif system == "Linux":
                            subprocess.run(["aplay", tmp_file.name], check=True)

                        # Wait a bit then clean up
                        time.sleep(2)

                    except Exception as e:
                        self.logger.error(f"System playback error: {e}")
                    finally:
                        self.is_playing = False
                        # Clean up temp file after delay
                        time.sleep(1)
                        try:
                            os.unlink(tmp_file.name)
                        except:
                            pass

                self.current_thread = threading.Thread(target=play_thread, daemon=True)
                self.current_thread.start()

                self.logger.info("Audio playback started with system player")
                return True

        except Exception as e:
            self.logger.error(f"System playback failed: {e}")
            return False

    def play_file(self, file_path: str) -> bool:
        """
        Play audio file
        """
        try:
            if not Path(file_path).exists():
                self.logger.error(f"Audio file not found: {file_path}")
                return False

            with open(file_path, 'rb') as f:
                audio_data = f.read()

            file_format = Path(file_path).suffix[1:]  # Remove the dot
            return self.play_audio_data(audio_data, file_format)

        except Exception as e:
            self.logger.error(f"File playback failed: {e}")
            return False

    def stop(self):
        """Stop current audio playback"""
        try:
            if self.pygame_initialized and pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()

            self.is_playing = False
            self.logger.info("Audio playback stopped")

        except Exception as e:
            self.logger.error(f"Stop playback failed: {e}")

    def is_playing_audio(self) -> bool:
        """
        Check if audio is currently playing
        """
        if self.pygame_initialized:
            return pygame.mixer.music.get_busy() or self.is_playing
        return self.is_playing

    def wait_for_completion(self, timeout: float = 30.0):
        """
        Wait for current audio playback to complete
        """
        start_time = time.time()
        while self.is_playing_audio() and (time.time() - start_time) < timeout:
            time.sleep(0.1)

    def get_player_info(self) -> dict:
        """
        Get audio player information
        """
        return {
            "pygame_available": PYGAME_AVAILABLE,
            "pygame_initialized": self.pygame_initialized,
            "pydub_available": PYDUB_AVAILABLE,
            "is_playing": self.is_playing_audio()
        }