import cv2
import time
import numpy as np
import os
import asyncio
import math
from datetime import datetime
from javis_dac.mc_singleton import MyCobotManager
import threading


class AlignVisionManager:
    """📷 ArUco 기반 탐색 + 정렬 + Yaw + TransferBook 통합 싱글톤 (print 버전)"""
    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self, port="/dev/ttyJETCOBOT", baud=1_000_000):
        if getattr(self, "_initialized", False):
            return
        self._initialized = True

        # 기본 설정
        self.PORT = port
        self.BAUD = baud
        self.SPEED = 25
        self.SETTLE_WAIT = 1.0
        self.CENTER_TOL = 10.0
        self.MIN_MOVE = 2.0
        self.STEP = 20
        self.Z_FIXED = 250.0
        self.X_SAFE_MIN, self.X_SAFE_MAX = 160, 200
        self.Y_SAFE_MIN, self.Y_SAFE_MAX = -140, 140
        self.FORWARD_X_MM = 45.0
        self.FORWARD_Y_MM = -10
        self.PICK_Z_HALF = 180.0
        self.PICK_Z_DOWN = 150.0
        
        self.detected_markers = []

        # 슬롯 상태 (True = 채워짐 / False = 비어있음)
        self.slot_status = {
            1: None,
            2: None,
            3: None,
        }

        
        self.book_to_shelf = {
            1: 22,
            2: 23,
            3: 24,
            4: 22,
            5: 25,
        }
        
        self.K = np.array([[1200.0, 0.0, 640.0],
                           [0.0, 1200.0, 360.0],
                           [0.0, 0.0, 1.0]], np.float32)
        self.dist = np.zeros(5, np.float32)

        print("🦾 Initializing MyCobotManager...")
        self.mc = MyCobotManager.get_instance()
        print("✅ MyCobot connected")

        self.home = [200, 0, 230., -180., 0., -45.]
        self.mc.send_coords(self.home, 25, 1)

        self.cap = self.open_camera()

        self.aru = cv2.aruco
        self.dict = self.aru.getPredefinedDictionary(self.aru.DICT_4X4_50)

        ver = cv2.__version__
        print(f"📦 OpenCV version: {ver}")

        try:
            major, minor, *_ = map(int, ver.split("."))
        except:
            major, minor = 4, 5
        if major > 4 or (major == 4 and minor >= 7):
            params = self.aru.DetectorParameters_create()
            self.detector = self.aru.ArucoDetector(self.dict, params)
            self.detect_func = lambda gray: self.detector.detectMarkers(gray)
        else:
            self.params = self.aru.DetectorParameters_create()
            self.detect_func = lambda gray: self.aru.detectMarkers(gray, self.dict, parameters=self.params)

        print("✅ AlignVisionManager initialized successfully.")
        
    def find_empty_slot(self):
        """비어있는 슬롯 찾기"""
        for slot_id, book_id in self.slot_status.items():
            if book_id is None:
                return slot_id
        return None  # 모든 슬롯이 찼을 때

    def add_book(self, slot_id, book_id):
        """책을 슬롯에 배치"""
        if self.slot_status[slot_id] is not None:
            print(f"⚠️ Slot {slot_id} 이미 책 {self.slot_status[slot_id]} 이 있습니다!")
            return
        self.slot_status[slot_id] = book_id
        print(f"📚 책 {book_id} 을(를) Slot {slot_id} 에 놓았습니다.")
    
    def get_slot_by_book(self, book_id):
        """책 번호로 해당 책이 들어있는 슬롯을 찾는다"""
        for slot_id, current_book in self.slot_status.items():
            if current_book == book_id:
                return slot_id
        return None  # 찾지 못한 경우   
    
    def remove_book(self, slot_id):
        """슬롯에서 책 꺼내기"""

        if self.slot_status[slot_id] is None:
            print(f"⚠️ Slot {slot_id} 은 이미 비어 있습니다.")
            return
        book_id = self.slot_status[slot_id]
        self.slot_status[slot_id] = None
        print(f"📦 책 {book_id} 을(를) Slot {slot_id} 에서 꺼냈습니다.")
    
    
    def get_shelf_by_book(self, book_id):
        """책 번호로 책장이 몇 번인지 조회"""
        return self.book_to_shelf.get(book_id, None)
    
    def get_book_by_shelf(self, shelf_id):
        """책장으로 책번호가 몇 번인지 조회"""
        return self.book_to_shelf.get(shelf_id, None)
    
    def reset_detected_markers(self):

        self.detected_markers = []
        print("🧹 detected_markers 리스트가 초기화되었습니다.")

    def get_marker_info(self, marker_id: int):
        for marker in self.detected_markers:
            if marker["id"] == marker_id:
                print(f"🔍 검색 결과 → ID={marker_id}")
                return marker

        print(f"❌ ID={marker_id} 마커 정보 없음")
        return None


    def get_all_detected_markers(self, sort_by="id"):

        if not hasattr(self, "detected_markers") or not self.detected_markers:
            print("⚠️ 감지된 마커 정보가 없습니다.")
            return []

        valid_keys = {"id", "timestamp", "dist_pix"}
        if sort_by not in valid_keys:
            print(f"⚠️ 알 수 없는 sort_by='{sort_by}' → 'id' 기준으로 정렬합니다.")
            sort_by = "id"

        sorted_list = sorted(self.detected_markers, key=lambda x: x.get(sort_by, 0))
        print(f"📦 총 {len(sorted_list)}개 마커 정보 반환:")
        for m in sorted_list:
            print(f"  - ID={m['id']}, dist_pix={m['dist_pix']:.1f}, pose={m['pose']}")
        return sorted_list

    # =========================================================
    # 📷 카메라
    # =========================================================
    def open_camera(self):
        print("[CAMERA] 🔍 Searching for camera...")
        for cam_id in range(0, 10):
            cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 5)

            valid_frames = 0
            for i in range(5):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    valid_frames += 1
                else:
                    time.sleep(0.1)

            if valid_frames >= 3:
                print(f"[CAMERA] ✅ Connected (ID={cam_id}) with {valid_frames}/5 valid frames")
                return cap
            else:
                print(f"[CAMERA] ⚠️ Camera {cam_id} failed (valid={valid_frames})")
                cap.release()

        raise RuntimeError("❌ No available camera (0~9 checked)")


    def get_latest_frame(self, timeout=1.0, caller="unknown"):
        start_time = time.time()
        latest = None
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
            print(f"⚠️ [get_latest_frame] Timeout ({caller})")
            return None

        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(save_dir, f"{ts}_{caller}.jpg")
        cv2.imwrite(path, frame)
        print(f"💾 Saved NEW frame → {path}")

        return frame



    def preprocess_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        return gray
    
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
        print(f"\n➡️ 탐색 위치 이동: dx={dx}, dy={dy}")
        await self.safe_move(target, speed=self.SPEED)
        await asyncio.sleep(self.SETTLE_WAIT + 0.2)

        # 프레임 획득
        frame = self.get_latest_frame(caller=f"scan_dx{dx}_dy{dy}")
        if frame is None:
            print("⚠️ 프레임 없음 — skip")
            return None

        # 감지 수행
        gray = self.preprocess_frame(frame)
        corners, ids, _ = self.detect_func(gray)
        if ids is None or len(ids) == 0:
            print("❌ 마커 감지 실패 (IDs=None)")
            return None

        detected = [int(i) for i in ids.flatten()]
        print(f"✅ 감지된 마커 IDs: {detected}")

        # 🎯 목표 ID 우선 반환
        if target_id in detected:
            print(f"🎯 목표 마커(ID={target_id}) 감지 성공!")
            return target_id

        # 📍 그 외 다른 마커 감지 시, 첫 번째 마커 반환
        alt_id = detected[0]
        print(f"⚙️ 목표 아님 → anchor 후보 마커 감지 (ID={alt_id})")
        return alt_id

    async def y_search_at_x(self, base, x_offset, target_id):
        x, y, z, rx, ry, rz = base
        Y_STEP, Y_RANGE = 50, 100
        print(f"\n🚦 [Y-SCAN] X offset={x_offset} → Y range {(-Y_RANGE)}~{Y_RANGE} step={Y_STEP}")

        for dy in range(-Y_RANGE, Y_RANGE + 1, Y_STEP):
            target = [x + x_offset, y + dy, z, rx, ry, rz]
            print(f"➡️ 이동: Y offset={dy}")
            await self.safe_move(target, speed=self.SPEED)
            await asyncio.sleep(self.SETTLE_WAIT + 0.2)

            frame = self.get_latest_frame(caller=f"y_search_x{round(x_offset,1)}_y{dy}")
            if frame is None:
                print("⚠️ 프레임 없음 — skip")
                continue

            gray = self.preprocess_frame(frame)
            corners, ids, _ = self.detect_func(gray)
            if ids is None or len(ids) == 0:
                continue

            detected = [int(i) for i in ids.flatten()]
            print(f"✅ 감지된 마커 IDs: {detected}")

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
                        print(f"🔁 ID={marker_id} 갱신 (기존 {existing['dist_pix']:.1f}px → 새 {marker_info['dist_pix']:.1f}px)")
                        self.detected_markers = [m for m in self.detected_markers if m["id"] != marker_id]
                        self.detected_markers.append(marker_info)
                else:
                    self.detected_markers.append(marker_info)
                    print(f"📦 추가: {marker_info}")

                # 🎯 목표 마커면 여기서 바로 저장 후 종료
                if marker_id == target_id:
                    print(f"🎯 목표 마커 {target_id} 발견 (Y offset={dy}, X offset={x_offset})")
                    print(f"💾 [Final] 목표 마커 ID={target_id} 정보 저장 완료")
                    return True  # ✅ 즉시 탐색 종료


    # =========================================================
    # 🔁 안전 이동 (await 기반, 실제 이동 변화 감지 포함)
    # =========================================================
    async def safe_move(self, target, speed=25, tol=2.0, timeout=8.0, move_tol=2.0):
        """
        send_coords 후 이동 완료될 때까지 비동기 대기.
        이동 전/후 좌표 변화를 감지해 실제로 움직였는지 판단함.
        - 이동 전/후 거리(diff)가 move_tol(mm) 미만이면 '이동 실패'로 간주.
        """
        print(f"\n➡️ [safe_move] 이동 명령: {target} (speed={speed})")

        # 🏁 이동 전 좌표 기록
        start_pose = self.mc.get_coords()
        if start_pose is None:
            print("⚠️ [safe_move] 초기 좌표 읽기 실패 (None 반환)")
            return False

        # 명령 전송
        self.mc.send_coords(target, speed, 1)
        start_time = time.time()

        # 이동 완료 대기 루프
        while time.time() - start_time < timeout:
            moving = self.mc.is_moving()
            if moving == 0:  # 모션 종료 감지
                break
            await asyncio.sleep(0.3)

        # 이동 후 좌표 확인
        end_pose = self.mc.get_coords()
        if end_pose is None:
            print("⚠️ [safe_move] 이동 후 좌표 읽기 실패")
            return False

        # 이동 거리 계산
        diff = np.linalg.norm(np.array(end_pose[:3]) - np.array(start_pose[:3]))
        print(f"📏 [safe_move] 실제 이동 거리: {diff:.2f} mm")

        if diff < move_tol:
            print(f"❌ [safe_move] 이동 변화 미미 ({diff:.2f}mm) → 실패 간주")
            return False

        # 목표 근접 확인
        diff_target = np.linalg.norm(np.array(end_pose[:3]) - np.array(target[:3]))
        if diff_target < tol:
            print(f"✅ [safe_move] 목표 근처 도착 (diff={diff_target:.2f}mm)")
        else:
            print(f"⚠️ [safe_move] 목표까지 거리 남음: {diff_target:.2f}mm")

        print("✅ [safe_move] 이동 완료 (좌표 변화 정상)")
        return True


    # =========================================================
    # 🎯 대략 정렬
    # =========================================================
    async def approx_align_marker(self, marker_id):

        prev_dist = None
        stagnant_count = 0
        MAX_STAGNANT = 6

        while True:
            frame = self.get_latest_frame(caller="approx_align_marker")
            if frame is None:
                print("⚠️ [approx_align_marker] 프레임 없음 — 재시도")
                continue

            result = self.compute_marker_offset(frame, marker_id)
            if result is None:
                print(f"⚠️ 마커 {marker_id} 인식 실패 — 재시도")
                continue

            dx, dy, dist_pix, (cx, cy), target = result
            print(f"📏 거리: {dist_pix:.1f}px (dx={dx:.1f}, dy={dy:.1f})")

            # 🎯 정렬 완료 조건
            if dist_pix < 60:
                print(f"✅ 중심 정렬 완료 ({dist_pix:.1f}px)")
                return self.mc.get_coords()

            # 🌀 정체 상태 방지
            if prev_dist is not None and abs(prev_dist - dist_pix) < 1.5:
                stagnant_count += 1
                print(f"⚠️ 변화 없음 ({stagnant_count}/{MAX_STAGNANT})")
                if stagnant_count >= MAX_STAGNANT:
                    print("❌ 정렬 실패 — 변화 없음")
                    return None
            else:
                stagnant_count = 0
            prev_dist = dist_pix

            # 이동 계산
            k = 0.1 if dist_pix > 60 else 0.08 if dist_pix > 25 else 0.025
            move_x = -dy * k
            move_y = -dx * k
            if abs(move_x) < self.MIN_MOVE: move_x = np.sign(move_x) * self.MIN_MOVE
            if abs(move_y) < self.MIN_MOVE: move_y = np.sign(move_y) * self.MIN_MOVE

            coords = self.mc.get_coords()
            coords[0] += move_x
            coords[1] += move_y
            await self.safe_move(coords, speed=self.SPEED)  # ✅ 안전 이동 (send_coords + 위치 확인)
            print(f"➡️ move_x={move_x:.2f}, move_y={move_y:.2f}, dist={dist_pix:.1f}")
            time.sleep(self.SETTLE_WAIT)

    # =========================================================
    # 📐 화면 중심과 아르코마커간 거리 계산
    # =========================================================
    def compute_marker_offset(self, frame, marker_id):
        if marker_id >= 20:
            mode = "center"   # 책장
        else:
            mode = "top"      # 책
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect_func(gray)

        if ids is None or marker_id not in ids.flatten():
            return None
        idx = list(ids.flatten()).index(marker_id)
        pts = corners[idx][0]
        target = np.mean([pts[0], pts[1]], axis=0) if mode == "top" else np.mean(pts, axis=0)
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
       
        coords = self.mc.get_coords()
        await self.safe_move(coords, speed=self.SPEED)
       
        print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] Center aligning marker step (ID={marker_info["id"]})")
       
        frame = self.get_latest_frame(caller="center_align_marker")
        
        if frame is None:
            print("⚠️ 프레임 없음 — skip")
            return None, None

        result = self.compute_marker_offset(frame, marker_info["id"])
        if result is None:
            print(f"⚠️ 마커 {marker_info["id"]} 인식 실패")
            return None, None

        dx, dy, dist_pix, (cx, cy), target = result
        print(f"📏 거리: {dist_pix:.1f}px (dx={dx:.1f}, dy={dy:.1f})")

        # 🎯 정렬 완료 조건
        if dist_pix < center_tol:
            print(f"✅ 중심 정렬 완료 ({dist_pix:.1f}px)")
            return True, self.mc.get_coords()

        # 🌀 이전 거리 비교 (정체 감지)
        if not hasattr(self, "_prev_center_dist"):
            self._prev_center_dist = None
            self._stagnant_count = 0
        
        if self._prev_center_dist is not None and abs(self._prev_center_dist - dist_pix) < 1.5:
            self._stagnant_count += 1
            print(f"⚠️ 변화 없음 ({self._stagnant_count}/6)")
            if self._stagnant_count >= 6:
                print("❌ 정렬 실패 — 변화 없음 (자동 종료)")
                return None, None
        else:
            self._stagnant_count = 0

        self._prev_center_dist = dist_pix

        # 📦 이동 계산
        k = 0.15 if dist_pix > 60 else 0.1 if dist_pix > 25 else 0.05
        move_x = -dy * k
        move_y = -dx * k
        if abs(move_x) < self.MIN_MOVE:
            move_x = np.sign(move_x) * self.MIN_MOVE
        if abs(move_y) < self.MIN_MOVE:
            move_y = np.sign(move_y) * self.MIN_MOVE

        coords = self.mc.get_coords()
        coords[0] += move_x
        coords[1] += move_y

        print(f"➡️ move_x={move_x:.2f}, move_y={move_y:.2f}, dist={dist_pix:.1f}")
        await self.safe_move(coords, speed=self.SPEED)  # ✅ 안전 이동 (send_coords + 위치 확인)
        time.sleep(self.SETTLE_WAIT)

        return False, dist_pix

    # =========================================================
    # 🧭 Yaw 정렬
    # =========================================================
    def align_yaw(self, marker_id):
        print(f"\n🧭 Yaw 정렬 시작 — ID={marker_id}")
        frame = self.get_latest_frame(caller="align_yaw")
        if frame is None:
            print("❌ 카메라 프레임 실패 (Yaw)")
            return False

        gray = self.preprocess_frame(frame)
        corners, ids, _ = self.detect_func(gray)
        if ids is None or marker_id not in ids.flatten():
            print("❌ 마커 인식 실패 (Yaw)")
            return False

        idx = list(ids.flatten()).index(marker_id)
        pts = corners[idx]
        success, rvec, tvec = cv2.solvePnP(
            np.array([[-15, 15, 0], [15, 15, 0], [15, -15, 0], [-15, -15, 0]], np.float32),
            pts, self.K, self.dist)
        if not success:
            print("❌ Pose 계산 실패 (Yaw)")
            return False

        R, _ = cv2.Rodrigues(rvec)
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        print(f"📐 감지된 Yaw: {yaw:.2f}°")
        
        angles = self.mc.get_angles()
        if angles is None:
            print("❌ 관절 각도 읽기 실패")
            return False
        
        angles[5] += yaw
        
        self.mc.send_angles(angles, self.SPEED)
        print(f"🧭 Yaw {yaw:.2f}° 보정 중...")
        time.sleep(self.SETTLE_WAIT + 0.5)
        print("✅ Yaw 보정 완료")
        return True
    
    # =========================================================
    # ⚙️ 동기 대기 함수
    # =========================================================
    def wait_motion_done(self, wait_time=8.0, check_interval=0.1):
        """MyCobot 동작 완료를 동기적으로 기다림. (-1도 완료로 간주)"""
        start = time.time()
        while True:
            state = self.mc.is_moving()
            if state in (0, -1):
                return True
            if time.time() - start > wait_time:
                print("⚠️ Timeout, continue...")
                return False
            time.sleep(check_interval)


    # =========================================================
    # 🚀 관절 전송 함수 (동기 버전)
    # =========================================================
    async def send_angles_sync(self, angles, speed=25, wait_time=8.0):
        """관절 각도 전송 후 동작 완료 대기"""
        self.mc.send_angles(angles, speed)
        self.wait_motion_done(wait_time)


    # =========================================================
    # 📦 도비 ↔ 책장 통합 이동 함수 (프로젝트 맞춤형)
    # =========================================================
    async def transfer_book(self, mode, shelf_pose, arco_id):
        """
        도비 저장소 <-> 책장 간 전송 시퀀스
        mode: "DOBBY_TO_SHELF" or "SHELF_TO_DOBBY"
        """
        print(f"\n🚀 Transfer 시작 ({mode})")
        time.sleep(1.0)

        home = [200, 0, 230., -180., 0., -45.]

        # 슬롯별 단계별 관절 포즈
        SLOT_POSES = {
            1: [
                [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                [-140.71, -34.01, -17.66, -38.58, 3.95, -9.93],
                [-149.06, -45.35, -48.16, 7.29, 5.62, -12.56],
                [-149.32, -58.35, -64.51, 36.73, 5.44, -12.56],
            ],
            2: [
                [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                [-165.67, -11.33, -58.0, -15.64, 1.58, -31.28],
                [-164.79, -45.0, -49.83, 5.71, 6.15, -28.47],
                [-165.05, -57.56, -65.91, 35.33, 5.62, -28.47],
            ],
            3: [
                [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                [166.64, -44.12, -1.05, -46.23, 4.92, -59.15],
                [167.08, -72.68, 0.35, -14.76, 6.5, -54.14],
                [166.81, -87.8, 1.14, -2.72, 6.32, -54.84]
            ],
        }

        
        
        # =========================================================
        # 🟦 도비 → 책장
        # =========================================================
        if mode == "DOBBY_TO_SHELF":
            
            book_id = self.get_book_by_shelf(self, arco_id)
            
            carrier_slot_id = self.get_slot_by_book(book_id)
            
            if carrier_slot_id is None:
                print(f"❌ 책 ID={book_id} 에 해당하는 슬롯을 찾을 수 없습니다!")
                return False
            
            poses = SLOT_POSES.get(carrier_slot_id)
            
            print("🚗 [도비 → 책장] 전송 시퀀스 시작")
            
            self.mc.set_gripper_value(100, 50)
            print("🤏 그리퍼 열림")
            
            await self.send_angles_sync(poses[0], 50)

            await self.send_angles_sync(poses[1], 50)

            await self.send_angles_sync(poses[2], 30)

            await self.send_angles_sync(poses[3], 30)
                
            self.mc.set_gripper_value(0, 50)
            print("🤏 그리퍼 닫힘")
            
            await self.send_angles_sync(poses[2], 30)
            
            await self.send_angles_sync(poses[1], 30)
            
            await self.send_angles_sync(poses[0], 30)

            await self.safe_move(shelf_pose, speed=25)
            
            time.sleep(self.SETTLE_WAIT)
            print("📍 책 위치로 이동 중...")

            approach = self.mc.get_coords()
            approach[0] += self.FORWARD_X_MM - 3.5
            approach[1] += self.FORWARD_Y_MM
            approach[2] = self.PICK_Z_HALF
            await self.safe_move(approach, speed=25)

            down = self.mc.get_coords()
            down[2] = self.PICK_Z_DOWN
            await self.safe_move(down, speed=25)

            self.mc.set_gripper_value(100, 50)
            print("📗 책 배치 완료")

            await self.safe_move(shelf_pose, speed=30)
            
            self.remove_book(carrier_slot_id, book_id)
            
            print("✅ 도비→책장 완료")

        # =========================================================
        # 🟥 책장 → 도비
        # =========================================================
        elif mode == "SHELF_TO_DOBBY":
            print("\n==============================")
            print(f"📦 [SHELF_TO_DOBBY] 시작 — arco_id={arco_id}")
            print("==============================")

            carrier_slot_id = self.find_empty_slot()
            print(f"🔍 선택된 빈 슬롯: {carrier_slot_id}")

            if carrier_slot_id is None:
                print("❌ 비어있는 슬롯이 없습니다!")
                return False
            
            poses = SLOT_POSES.get(carrier_slot_id)
            if poses is None:
                print(f"⚠️ SLOT_POSES[{carrier_slot_id}] 없음 — 시퀀스 종료")
                return False
            
            print(f"📕 [책장 → 도비] 시퀀스 시작 (slot={carrier_slot_id})")

            self.mc.set_gripper_value(100, 50)
            print("🤏 그리퍼 열림 (책 잡기 전)")

            print(f"➡️ 1️⃣ 책장 위치로 이동: shelf_pose={shelf_pose}")
            await self.safe_move(shelf_pose, speed=30)
            time.sleep(self.SETTLE_WAIT)
            print("📍 책 위치로 접근 중...")

            approach = self.mc.get_coords()
            print(f"📸 현재 좌표 (approach 전): {approach}")
            approach[0] += self.FORWARD_X_MM
            approach[1] += self.FORWARD_Y_MM
            approach[2] = self.PICK_Z_HALF
            print(f"➡️ 접근 좌표 (FORWARD 적용): {approach}")
            await self.safe_move(approach, speed=25)

            final_down = self.mc.get_coords()
            final_down[2] = self.PICK_Z_DOWN
            print(f"⬇️ 최종 하강 좌표: {final_down}")
            await self.safe_move(final_down, speed=25)

            self.mc.set_gripper_value(0, 50)
            print("📕 책 집기 완료 (그리퍼 닫힘)")

            lift = self.mc.get_coords()
            lift[2] = self.PICK_Z_HALF
            print(f"⬆️ 리프트 좌표: {lift}")
            await self.safe_move(lift, speed=25)

            time.sleep(1.0)
            print("🦾 도비 슬롯 복귀 시작 (단계별 이동)")

            print("  ▶ 1단계 → poses[0]")
            await self.send_angles_sync(poses[0], 10)

            print("  ▶ 2단계 → poses[1]")
            await self.send_angles_sync(poses[1], 50)

            print("  ▶ 3단계 → poses[2]")
            await self.send_angles_sync(poses[2], 30)

            print("  ▶ 4단계 → poses[3]")
            await self.send_angles_sync(poses[3], 30)

            self.mc.set_gripper_value(100, 50)
            print("🤏 그리퍼 열림 (책 내려놓기 전)")

            time.sleep(1.0)

            print("  ◀ 복귀 경로 되돌리기 시작")
            await self.send_angles_sync(poses[2], 30)
            await self.send_angles_sync(poses[1], 30)
            await self.send_angles_sync(poses[0], 30)
            print("✅ 도비 슬롯 복귀 완료")

            self.add_book(carrier_slot_id, arco_id)
            print(f"📚 Slot {carrier_slot_id} 에 책 {arco_id} 등록 완료")
            print("📦 도비 내부 슬롯 상태:", self.slot_status)

        else:
            print(f"❌ 잘못된 mode 값: {mode}")
            return False

        # 홈 복귀
        print("🏠 홈 포즈 복귀 중...")
        await self.safe_move(home, speed=30)
        print("🏁 홈 복귀 완료 (Transfer 종료)")
        print("==============================\n")
        return True


    # =========================================================
    # 🧹 종료 처리
    # =========================================================
    def close(self):
        print("\n🧹 시스템 종료 중...")
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
                print("📷 카메라 해제 완료")
        except Exception as e:
            print(f"⚠️ 카메라 해제 실패: {e}")

        try:
            self.mc.send_coords(self.home, 25, 1)
            print("🏠 로봇 홈으로 복귀 완료")
        except Exception as e:
            print(f"⚠️ 로봇 복귀 실패: {e}")

        print("✅ 시스템 종료 완료")

    def __del__(self):
        self.close()
