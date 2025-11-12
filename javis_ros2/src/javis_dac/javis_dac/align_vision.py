import cv2
import logging
import os
import threading
from datetime import datetime
import time

from javis_dac.config import Config


class AlignVision:
    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return
        self._initialized = True

        self.logger = logging.getLogger(__name__)

        # 설정 로드
        self.config = Config()
        self.cap = self.open_camera()
    
    # =========================================================
    # 📷 카메라
    # =========================================================
    def open_camera(self):
        self.logger.info("[CAMERA] Searching for camera...")
        for cam_id in range(0, 10):
            cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.camera_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.camera_height)
            cap.set(cv2.CAP_PROP_FPS, self.config.camera_fps)

            valid_frames = 0
            for i in range(5):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    valid_frames += 1
                else:
                    time.sleep(0.1)

            if valid_frames >= 3:
                self.logger.info("[CAMERA] Connected (ID=%s) valid_frames=%s", cam_id, valid_frames)
                return cap
            else:
                self.logger.warning("[CAMERA] Camera %s failed (valid=%s)", cam_id, valid_frames)
                cap.release()

        raise RuntimeError("❌ No available camera (0~9 checked)")


    def get_latest_frame(self, timeout=1.0, caller="unknown"):
        save_dir = os.path.join(os.getcwd(), "frames")
        os.makedirs(save_dir, exist_ok=True)

        # --- 내부 버퍼 강제 비우기 ---
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(5):
            self.cap.grab()
            time.sleep(0.02)

        # --- 최신 프레임 획득 ---
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.logger.warning("⚠️ [get_latest_frame] Timeout (%s)", caller)
            return None

        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(save_dir, f"{ts}_{caller}.jpg")
        cv2.imwrite(path, frame)
        self.logger.debug("Saved frame path=%s caller=%s", path, caller)

        return frame

    def preprocess_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        return gray

    # =========================================================
    # 🧹 종료 처리
    # =========================================================
    def close(self):
        self.logger.info("🧹 AlignVision shutdown...")
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
                self.logger.info("📷 카메라 해제 완료")
        except Exception as e:
            self.logger.warning("⚠️ 카메라 해제 실패: %s", e)

        self.logger.info("✅ AlignVision shutdown complete")

    def __del__(self):
        self.close()
