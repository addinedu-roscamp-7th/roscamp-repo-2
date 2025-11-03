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
        self.FORWARD_X_MM = 50.0
        self.FORWARD_Y_MM = -15
        self.PICK_Z_HALF = 200.0
        self.PICK_Z_DOWN = 140.0

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
        Y_STEP, Y_RANGE = 25, 100

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

            if target_id in detected:
                print(f"🎯 목표 마커 {target_id} 발견 (Y offset={dy}, X offset={x_offset})")
                return True
        print("❌ 목표 마커 탐색 실패 (Y축 종료)")
        return False



    # =========================================================
    # 🔁 안전 이동 (await 기반, 이동 변화 감지 포함)
    # =========================================================
    async def safe_move(self, target, speed=25, tol=2.0, timeout=8.0, min_move=1.0):
        """
        send_coords 후 이동 완료될 때까지 비동기 대기.
        이동 전/후 좌표 변화를 감지해 실제로 움직였는지 판단함.
        """
        # 🏁 이동 명령 전 좌표 기록
        start_coords = self.mc.get_coords()
        if start_coords is None:
            print("⚠️ [safe_move] 초기 좌표 읽기 실패")
            return False

        self.mc.send_coords(target, speed, 1)
        start_time = time.time()
        last_coords = start_coords

        while time.time() - start_time < timeout:
            coords = self.mc.get_coords()
            if coords is None:
                await asyncio.sleep(0.1)
                continue

            # 1️⃣ 목표 근접 여부 검사
            diff_target = np.linalg.norm(np.array(coords[:3]) - np.array(target[:3]))
            if diff_target < tol:
                print(f"✅ [safe_move] 목표 근처 도착 (diff={diff_target:.2f})")
                return True

            # 2️⃣ 실제 이동 여부 검사
            diff_move = np.linalg.norm(np.array(coords[:3]) - np.array(last_coords[:3]))
            if diff_move > min_move:
                last_coords = coords  # 움직였으면 갱신
            else:
                print(f"⚠️ [safe_move] 좌표 변화 없음 ({diff_move:.2f}mm) → 대기 중...")

            await asyncio.sleep(0.2)

        # 타임아웃 시, 최종 비교
        end_coords = self.mc.get_coords()
        total_move = np.linalg.norm(np.array(end_coords[:3]) - np.array(start_coords[:3]))
        if total_move > min_move:
            print(f"⚠️ [safe_move] 타임아웃이지만 이동 감지됨 ({total_move:.2f}mm)")
            return True

        print(f"❌ [safe_move] Timeout — 이동 없음 ({total_move:.2f}mm)")
        return False

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

            result = self.compute_marker_offset(frame, marker_id, "center")
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
    # 📐 중심 계산
    # =========================================================
    def compute_marker_offset(self, frame, marker_id, mode="center"):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect_func(gray)
        if ids is None or marker_id not in ids.flatten():
            return None
        idx = list(ids.flatten()).index(marker_id)
        corner = corners[idx][0]
        cx = np.mean(corner[:, 0])
        cy = np.mean(corner[:, 1])
        h, w = gray.shape[:2]
        target = (w / 2, h / 2)
        dx = cx - target[0]
        dy = cy - target[1]
        dist_pix = math.sqrt(dx ** 2 + dy ** 2)
        return dx, dy, dist_pix, (cx, cy), target

    # =========================================================
    # 🎯 중심 정렬 (1스텝 단위, safe_move + 정체 감지 포함)
    # =========================================================
    async def center_align_marker_step(self, marker_id, center_tol, mode):
       
        print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] mode: {mode} — Center aligning marker step (ID={marker_id})")
       
        for _ in range(3):
            self.cap.grab()
        frame = self.get_latest_frame(caller="center_align_marker")
        
        if frame is None:
            print("⚠️ 프레임 없음 — skip")
            return None, None

        result = self.compute_marker_offset(frame, marker_id, mode)
        if result is None:
            print(f"⚠️ 마커 {marker_id} 인식 실패")
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
    async def transfer_book(self, mode, shelf_pose):
        """
        도비 저장소 <-> 책장 간 전송 시퀀스
        mode: "DOBBY_TO_SHELF" or "SHELF_TO_DOBBY"
        """
        print(f"\n🚀 Transfer 시작 ({mode})")
        time.sleep(1.0)

        home = [200, 0, 230., -180., 0., -45.]

        pose0 = [18.8, -26.36, -18.45, -45.61, 0.26, -26.36]
        pose1 = [-166, -11.42, -58.09, -15.9, 1.4, -31.2]
        pose2 = [-166, -50.8, -60.09, 20.3, 5.53, -32.6]
        pose3 = [-166, -62.75, -58.44, 30.76, 3.51, -30.41]
        poses = [pose0, pose1, pose2, pose3]

        # =========================================================
        # 🟦 도비 → 책장
        # =========================================================
        if mode == "DOBBY_TO_SHELF":
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
            approach[0] += self.FORWARD_X_MM
            approach[1] += self.FORWARD_Y_MM
            approach[2] = self.PICK_Z_HALF
            await self.safe_move(approach, speed=25)

            down = self.mc.get_coords()
            down[2] = self.PICK_Z_DOWN
            await self.safe_move(down, speed=25)

            self.mc.set_gripper_value(100, 50)
            print("📗 책 배치 완료")

            await self.safe_move(shelf_pose, speed=30)
            print("✅ 도비→책장 완료")

        # =========================================================
        # 🟥 책장 → 도비
        # =========================================================
        elif mode == "SHELF_TO_DOBBY":
            print("📕 [책장 → 도비] 시퀀스 시작")
            
            self.mc.set_gripper_value(100, 50)
            print("🤏 그리퍼 열림")
            
            await self.safe_move(shelf_pose, speed=30)
            time.sleep(self.SETTLE_WAIT)
            print("📍 책 위치로 이동 중...")

            approach = self.mc.get_coords()
            approach[0] += self.FORWARD_X_MM + 2
            approach[1] += self.FORWARD_Y_MM
            approach[2] = self.PICK_Z_HALF
            await self.safe_move(approach, speed=25)

            final_down = self.mc.get_coords()
            final_down[2] = self.PICK_Z_DOWN
            await self.safe_move(final_down, speed=25)

            self.mc.set_gripper_value(0, 50)
            print("📕 책 집기 완료")

            lift = self.mc.get_coords()
            lift[2] = self.PICK_Z_HALF + 2
            self.safe_move(lift, speed=25)

            # 6️⃣ 도비 복귀 단계별 이동

            await self.send_angles_sync(poses[0], 50)

            await self.send_angles_sync(poses[1], 50)

            await self.send_angles_sync(poses[2], 30)

            await self.send_angles_sync(poses[3], 30)
                
            self.mc.set_gripper_value(100, 50)
            
            time.sleep(1.0)
            
            await self.send_angles_sync(poses[2], 30)
            
            await self.send_angles_sync(poses[1], 30)
            
            await self.send_angles_sync(poses[0], 30)
            

        else:
            print(f"❌ 잘못된 mode 값: {mode}")
            return False

        # 홈 복귀
        await self.safe_move(home, speed=30)
        print("🏠 홈 복귀 완료")
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
