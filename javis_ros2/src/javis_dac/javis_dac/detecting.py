import asyncio
import logging
import threading
import time
from datetime import datetime

import cv2
import numpy as np

from javis_dac.align_vision import AlignVision
from javis_dac.config import Config
from javis_dac.robot_move import RobotMove

class Detecting:
    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        self.detected_markers = []
        self.logger = logging.getLogger(__name__)
        self.align = AlignVision.get_instance()
        self.robot_move = RobotMove.get_instance()
        self.config = Config()

        self.aru = cv2.aruco
        self.dict = self.aru.getPredefinedDictionary(self.aru.DICT_4X4_50)

        ver = cv2.__version__
        self.logger.info("📦 OpenCV version: %s", ver)

        try:
            major, minor, *_ = map(int, ver.split("."))
        except:
            major, minor = 4, 5

        # ✅ OpenCV 4.7 이상: 새로운 ArUco API
        if major > 4 or (major == 4 and minor >= 7):
            params = self.aru.DetectorParameters()
            self.detector = self.aru.ArucoDetector(self.dict, params)
            self.detect_func = lambda gray: self.detector.detectMarkers(gray)

        # ✅ OpenCV 4.6 이하: 구버전 API
        else:
            self.params = self.aru.DetectorParameters_create()
            self.detect_func = lambda gray: self.aru.detectMarkers(gray, self.dict, parameters=self.params)

        self.logger.info("✅ Detecting initialized successfully.")
        
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = Detecting()
        return cls._instance

    def aru_detect_func(self, gray):
        return self.detect_func(gray)
    
    def reset_detected_markers(self):

        self.detected_markers = []
        self.logger.info("🧹 detected_markers 리스트가 초기화되었습니다.")

    def get_marker_info(self, marker_id: int):
        for marker in self.detected_markers:
            if marker["id"] == marker_id:
                self.logger.info("🔍 검색 결과 → ID=%s", marker_id)
                return marker

        self.logger.warning("❌ ID=%s 마커 정보 없음", marker_id)
        return None


    def get_all_detected_markers(self, sort_by="id"):

        if not hasattr(self, "detected_markers") or not self.detected_markers:
            self.logger.warning("⚠️ 감지된 마커 정보가 없습니다.")
            return []

        valid_keys = {"id", "timestamp", "dist_pix"}
        if sort_by not in valid_keys:
            self.logger.warning("⚠️ 알 수 없는 sort_by='%s' → 'id' 기준으로 정렬합니다.", sort_by)
            sort_by = "id"

        sorted_list = sorted(self.detected_markers, key=lambda x: x.get(sort_by, 0))
        self.logger.info("📦 총 %s개 마커 정보 반환:", len(sorted_list))
        for m in sorted_list:
            self.logger.info("  - ID=%s, dist_pix=%.1f, pose=%s", m['id'], m['dist_pix'], m['pose'])
        return sorted_list
    
    # =========================================================
    # 🎯 대략 정렬
    # =========================================================
    async def approx_align_marker(self, marker_id):

        prev_dist = None
        stagnant_count = 0
        MAX_STAGNANT = 6

        while True:
            frame = self.align.get_latest_frame(caller="approx_align_marker")
            if frame is None:
                self.logger.warning("⚠️ [approx_align_marker] 프레임 없음 — 재시도")
                continue

            result = self.compute_marker_offset(frame, marker_id)
            if result is None:
                self.logger.warning("⚠️ 마커 %s 인식 실패 — 재시도", marker_id)
                continue

            dx, dy, dist_pix, (cx, cy), target = result
            self.logger.info("📏 거리: %.1fpx (dx=%.1f, dy=%.1f)", dist_pix, dx, dy)

            # 🎯 정렬 완료 조건
            if dist_pix < 60:
                self.logger.info("✅ 중심 정렬 완료 (%.1fpx)", dist_pix)
                self._prev_center_dist = 300   
                self._stagnant_count = 0
                return self.robot_move.get_coords()

            # 🌀 정체 상태 방지
            if prev_dist is not None and abs(prev_dist - dist_pix) < 1.5:
                stagnant_count += 1
                self.logger.warning("⚠️ 변화 없음 (%s/%s)", stagnant_count, MAX_STAGNANT)
                if stagnant_count >= MAX_STAGNANT:
                    self.logger.error("❌ 정렬 실패 — 변화 없음")
                    return None
            else:
                stagnant_count = 0
            prev_dist = dist_pix

            # 이동 계산
            k = 0.1 if dist_pix > 60 else 0.08 if dist_pix > 25 else 0.025
            move_x = -dy * k
            move_y = -dx * k
            if abs(move_x) < self.align.MIN_MOVE:
                move_x = np.sign(move_x) * self.align.MIN_MOVE
            if abs(move_y) < self.align.MIN_MOVE:
                move_y = np.sign(move_y) * self.align.MIN_MOVE

            coords = self.robot_move.get_coords()
            
            coords[0] += move_x
            coords[1] += move_y
            coords[2] = 230
            coords[3] = -180 
            coords[4] = 0
            
            await self.align.safe_move(coords, speed=self.align.SPEED)  # ✅ 안전 이동 (send_coords + 위치 확인)
            self.logger.info("➡️ move_x=%.2f, move_y=%.2f, dist=%.1f", move_x, move_y, dist_pix)
            time.sleep(self.align.SETTLE_WAIT)

    # =========================================================
    # 📐 화면 중심과 아르코마커간 거리 계산
    # =========================================================
    def compute_marker_offset(self, frame, marker_id):
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect_func(gray)

        if ids is None or marker_id not in ids.flatten():
            return None
        idx = list(ids.flatten()).index(marker_id)
        pts = corners[idx][0]
        target = np.mean(pts, axis=0)
        target = target.astype(int)
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        dx, dy = target[0] - cx, target[1] - cy
        dist_pix = np.hypot(dx, dy)
        return dx, dy, dist_pix, (cx, cy), tuple(target)

    # =========================================================
    # 🎯 중심 정렬 (1스텝 단위, safe_move + 정체 감지 포함)
    # =========================================================
    async def center_align_marker_step(self, marker_info, center_tol):
       
        self.logger.debug("marker_info=%s", marker_info)
        self.logger.info("[%s] Center aligning marker step (ID=%s)", datetime.now().strftime("%Y-%m-%d %H:%M:%S"), marker_info["id"])

        if not hasattr(self, "_prev_center_dist"):
            self._prev_center_dist = 300
            self._stagnant_count = 0

        sample_count = 0
        
        # 이부분에서 거리에 따라서 값을 여러개 받아서 정확하게 정렬
        if self._prev_center_dist < 20:
            sample_count = 3
        elif self._prev_center_dist < 40:
            sample_count = 2
        else:
            sample_count = 1
            
        self.logger.info("🎞 샘플 수 설정: %s (prev_dist=%.1f)", sample_count, self._prev_center_dist)
    
        # 📸 여러 프레임에서 평균 dx, dy 계산
        dx_list, dy_list, dist_list = [], [], []
        for i in range(sample_count):
            frame = self.align.get_latest_frame(caller=f"center_align_marker[{i}]")
            if frame is None:
                self.logger.warning("⚠️ 프레임 %s/%s 없음 — skip", i + 1, sample_count)
                continue

            result = self.compute_marker_offset(frame, marker_info["id"])
            if result is None:
                self.logger.warning("⚠️ 마커 %s 인식 실패 (%s/%s)", marker_info['id'], i + 1, sample_count)
                continue

            dx, dy, dist_pix, _, _ = result
            dx_list.append(dx)
            dy_list.append(dy)
            dist_list.append(dist_pix)

            time.sleep(0.05)

        if len(dx_list) == 0:
            self.logger.error("❌ 모든 프레임 인식 실패")
            return None, None

        dx_mean = np.mean(dx_list)
        dy_mean = np.mean(dy_list)
        dist_mean = np.mean(dist_list)
        self.logger.info("📏 평균 거리: %.1fpx (dx=%.1f, dy=%.1f) from %s frames", dist_mean, dx_mean, dy_mean, len(dx_list))
        
        if dist_pix < center_tol:
            self.logger.info("✅ 중심 정렬 완료 (%.1fpx)", dist_pix)
            self._stagnant_count = 0
            return True, self.robot_move.get_coords()
        
        if self._prev_center_dist is not None and abs(self._prev_center_dist - dist_pix) < 1.5:
            self._stagnant_count += 1
            self.logger.warning("⚠️ 변화 없음 (%s/6)", self._stagnant_count)
            if self._stagnant_count >= 6:
                self.logger.error("❌ 정렬 실패 — 변화 없음 (자동 종료)")
                return None, None
        else:
            self._stagnant_count = 0

        self._prev_center_dist = dist_pix

        k = 0.1 if dist_pix > 60 else 0.08 if dist_pix > 25 else 0.025
        move_x = -dy * k
        move_y = -dx * k
        
        if abs(move_x) < self.config.min_move:
            move_x = np.sign(move_x) * self.config.min_move
        if abs(move_y) < self.config.min_move:
            move_y = np.sign(move_y) * self.config.min_move

        coords = self.robot_move.get_coords()
        coords[0] += move_x
        coords[1] += move_y
        coords[2] = 230
        coords[3] = -180 
        coords[4] = 0
        
        self.logger.info("➡️ move_x=%.2f, move_y=%.2f, dist=%.1f", move_x, move_y, dist_pix)
        await self.align.safe_move(coords, speed=self.config.speed)
        time.sleep(self.config.settle_wait)

        return False, dist_pix
    
    # =========================================================
    # 🔍 단일 위치에서 마커 감지 (base + offset 기반, ID 반환 버전)
    # =========================================================
    async def scan_for_marker(self, base, target_id, dx=0.0, dy=0.0):
        """
        base 좌표를 기준으로 (dx, dy) 오프셋 위치로 이동 후 마커 감지.
        🎯 목표 마커(target_id)가 감지되면 해당 ID 반환,
        ❗ 아니면 감지된 다른 마커 중 첫 번째 ID 반환,
        ❌ 아무것도 감지되지 않으면 None 반환.
        """
        # base 언패킹
        x, y, z, rx, ry, rz = base

        # 목표 좌표 계산
        target = [x + dx, y + dy, z, rx, ry, rz]
        self.logger.info("➡️ 탐색 위치 이동: dx=%.1f, dy=%.1f", dx, dy)
        await self.robot_move.safe_move(target, speed=self.config.speed)
        await asyncio.sleep(self.config.settle_wait + 0.2)

        # 프레임 획득
        frame = self.align.get_latest_frame(caller=f"scan_dx{dx}_dy{dy}")
        if frame is None:
            self.logger.warning("⚠️ 프레임 없음 — skip")
            return None

        gray = self.align.preprocess_frame(frame)
        corners, ids, _ = self.detect_func(gray)
        if ids is None or len(ids) == 0:
            self.logger.warning("❌ 마커 감지 실패 (IDs=None)")
            return None

        detected = [int(i) for i in ids.flatten()]
        self.logger.info("✅ 감지된 마커 IDs: %s", detected)

        # 🎯 목표 ID 우선 반환
        if target_id in detected:
            self.logger.info("🎯 목표 마커(ID=%s) 감지 성공!", target_id)
            return target_id

        alt_id = detected[0]
        self.logger.info("⚙️ 목표 아님 → anchor 후보 마커 감지 (ID=%s)", alt_id)
        return alt_id

    async def y_search_at_x(self, base, x_offset, target_id):
        x, y, z, rx, ry, rz = base
        Y_STEP, Y_RANGE = 50, 100
        self.logger.info("🚦 [Y-SCAN] X offset=%.1f → Y range %s~%s step=%s", x_offset, -Y_RANGE, Y_RANGE, Y_STEP)

        for dy in range(-Y_RANGE, Y_RANGE + 1, Y_STEP):
            target = [x + x_offset, y + dy, z, rx, ry, rz]
            self.logger.info("➡️ 이동: Y offset=%s", dy)
            await self.robot_move.safe_move(target, speed=self.config.speed)
            await asyncio.sleep(self.config.settle_wait + 0.2)

            frame = self.align.get_latest_frame(caller=f"y_search_x{round(x_offset,1)}_y{dy}")
            if frame is None:
                self.logger.warning("⚠️ 프레임 없음 — skip")
                continue

            gray = self.align.preprocess_frame(frame)
            corners, ids, _ = self.detect_func(gray)
            if ids is None or len(ids) == 0:
                continue

            detected = [int(i) for i in ids.flatten()]
            self.logger.info("✅ 감지된 마커 IDs: %s", detected)

            for marker_id in detected:
                # 🎯 중심 오프셋 계산 (픽셀 기준)
                result = self.compute_marker_offset(frame, marker_id)
                if result is None:
                    continue

                dx, dy_px, dist_pix, (cx, cy), target_center = result
                marker_info = {
                    "id": marker_id,
                    "pose": target.copy(),
                    "dist_pix": float(dist_pix),
                    "timestamp": time.time()
                }

                # 중복 검사: 같은 ID 있으면 더 가까운 쪽으로 교체
                existing = next((m for m in self.detected_markers if m["id"] == marker_id), None)
                if existing:
                    if marker_info["dist_pix"] < existing["dist_pix"]:
                        self.logger.info("🔁 ID=%s 갱신 (기존 %.1fpx → 새 %.1fpx)", marker_id, existing['dist_pix'], marker_info['dist_pix'])
                        self.detected_markers = [m for m in self.detected_markers if m["id"] != marker_id]
                        self.detected_markers.append(marker_info)
                else:
                    self.detected_markers.append(marker_info)
                    self.logger.info("📦 추가: %s", marker_info)

                # 🎯 목표 마커면 여기서 바로 저장 후 종료
                if marker_id == target_id:
                    self.logger.info("🎯 목표 마커 %s 발견 (Y offset=%s, X offset=%.1f)", target_id, dy, x_offset)
                    self.logger.info("💾 [Final] 목표 마커 ID=%s 정보 저장 완료", target_id)
                    return True  # ✅ 즉시 탐색 종료
