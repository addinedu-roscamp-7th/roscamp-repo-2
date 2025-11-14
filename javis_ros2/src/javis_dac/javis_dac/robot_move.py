import asyncio
import logging
import threading
import time
from typing import Dict, List
import cv2

import numpy as np

from javis_dac.config import Config
from javis_dac.mc_singleton import MyCobotManager
from javis_dac.slot_inventory import SlotInventory
from javis_dac.align_vision import AlignVision

class RobotMove:
    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.config = Config()
        self.mc = MyCobotManager.get_instance()
        self.slot_inventory = SlotInventory.get_instance()
        self.align_vision = AlignVision.get_instance()
        
        self.K = np.array([[1200.0, 0.0, 640.0],
                    [0.0, 1200.0, 360.0],
                    [0.0, 0.0, 1.0]], np.float32)
        self.dist = np.zeros(5, np.float32)
        
        self.SPEED = self.config.speed
        self.SETTLE_WAIT = self.config.settle_wait
        
        self.MIN_MOVE = self.config.min_move
        self.STEP = self.config.step
        self.Z_FIXED = self.config.z_fixed
        self.X_SAFE_MIN, self.X_SAFE_MAX = self.config.x_safe_min, self.config.x_safe_max
        self.Y_SAFE_MIN, self.Y_SAFE_MAX = self.config.y_safe_min, self.config.y_safe_max
        self.FORWARD_X_MM = self.config.forward_x_mm
        self.FORWARD_Y_MM = self.config.forward_y_mm
        self.PICK_Z_HALF = self.config.pick_z_half
        self.PICK_Z_DOWN = self.config.pick_z_down
        
        
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = RobotMove()
        return cls._instance


    
    def get_coords(self):
        return self.mc.get_coords()
    
    # =========================================================
    # 🔁 안전 이동 (await 기반, 실제 이동 변화 감지 포함)
    # =========================================================
    
    async def safe_move(self, target, speed=25, tol=2.0, timeout=8.0, move_tol=2.0):
        """
        send_coords 후 이동 완료될 때까지 비동기 대기.
        이동 전/후 좌표 변화를 감지해 실제로 움직였는지 판단함.
        - 이동 전/후 거리(diff)가 move_tol(mm) 미만이면 '이동 실패'로 간주.
        """
        self.logger.info("➡️ [safe_move] 이동 명령: %s (speed=%s)", target, speed)

        # 🏁 이동 전 좌표 기록
        start_pose = self.mc.get_coords()
        if start_pose is None:
            self.logger.warning("⚠️ [safe_move] 초기 좌표 읽기 실패 (None 반환)")
            return False

        self.mc.send_coords(target, speed, 1)
        start_time = time.time()

        while time.time() - start_time < timeout:
            moving = self.mc.is_moving()
            if moving == 0:
                break
            await asyncio.sleep(0.3)

        end_pose = self.mc.get_coords()
        if end_pose is None:
            self.logger.warning("⚠️ [safe_move] 이동 후 좌표 읽기 실패")
            return False

        diff = np.linalg.norm(np.array(end_pose[:3]) - np.array(start_pose[:3]))
        self.logger.info("📏 [safe_move] 전: {start_pose} 후: {end_pose}")
        self.logger.info("📏 [safe_move] 실제 이동 거리: %.2f mm", diff)

        if diff < move_tol:
            self.logger.warning("❌ [safe_move] 이동 변화 미미 (%.2fmm) → 실패 간주", diff)
            return False

        diff_target = np.linalg.norm(np.array(end_pose[:3]) - np.array(target[:3]))
        if diff_target < tol:
            self.logger.info("✅ [safe_move] 목표 근처 도착 (diff=%.2fmm)", diff_target)
        else:
            self.logger.warning("⚠️ [safe_move] 목표까지 거리 남음: %.2fmm", diff_target)

        self.logger.info("✅ [safe_move] 이동 완료 (좌표 변화 정상)")
        return True
    
    # =========================================================
    # 🧭 Yaw 정렬
    # =========================================================
    async def align_yaw(self, marker_id, corners, ids):
        print(f"\n🧭 Yaw 정렬 시작 — ID={marker_id}")
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

        pose = self.mc.get_angles()
        if pose is None:
            print("❌ 관절 각도 읽기 실패")
            return False

        pose[5] += yaw
        await self.send_angles_sync(pose, 50)

        self.safe_move(pose, speed=self.SPEED)

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
                self.logger.warning("⚠️ Timeout, continue...")
                return False
            time.sleep(check_interval)


    # =========================================================
    # 🚀 관절 전송 함수 (동기 버전)
    # =========================================================
    async def send_angles_sync(self, angles, speed=None, wait_time=8.0):
        """관절 각도 전송 후 동작 완료 대기"""
        if speed is None:
            speed = self.config.speed
        self.mc.send_angles(angles, speed)
        self.wait_motion_done(wait_time)


    # =========================================================
    # 📦 도비 ↔ 책장 통합 이동 함수 (프로젝트 맞춤형)
    # =========================================================
    async def transfer_book(self, mode, shelf_pose, arco_id, dobby_num):
        """
        도비 저장소 <-> 책장 간 전송 시퀀스
        mode: "DOBBY_TO_SHELF" or "SHELF_TO_DOBBY"
        """
        print(f"\n🚀 Transfer 시작 ({mode})")
        time.sleep(1.0)

        home = [200, 0, 230., -180., 0., -45.]
        if dobby_num == 1:
            slot_poses = {
                0: [
                    [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                    [-140.71, -34.01, -17.66, -38.58, 3.95, -9.93],
                    [-149.06, -45.35, -48.16, 7.29, 5.62, -12.56],
                    [-149.32, -58.35, -64.51, 36.73, 5.44, -12.56],
                ],
                1: [
                    [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                    [-165.67, -11.33, -58.0, -15.64, 1.58, -31.28],
                    [-164.79, -45.0, -49.83, 5.71, 6.15, -28.47],
                    [-165.05, -57.56, -65.91, 35.33, 5.62, -28.47],
                ],
                2: [
                    [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                    [166.64, -44.12, -1.05, -46.23, 4.92, -59.15],
                    [167.08, -72.68, 0.35, -14.76, 6.5, -54.14],
                    [166.81, -87.8, 1.14, -2.72, 6.32, -54.84]
                ],
            }

            pick_x_offset = self.config.pick_x_offset_dobby1
            pick_y_offset = self.config.pick_y_offset_dobby1
            pick_z_offset = self.config.pick_z_offset_dobby1
            
            place_x_offset = self.config.place_x_offset_dobby1
            place_y_offset = self.config.place_y_offset_dobby1
            place_z_offset = self.config.place_z_offset_dobby1
            
            
        elif dobby_num == 2:
            slot_poses = {
                0: [
                    [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                    [-140.71, -34.01, -17.66, -38.58, 3.95, -9.93],
                    [-149.06, -45.35, -48.16, 7.29, 5.62, -12.56],
                    [-149.32, -58.35, -64.51, 36.73, 5.44, -12.56],
                ],
                1: [
                    [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                    [-165.67, -11.33, -58.0, -15.64, 1.58, -31.28],
                    [-164.79, -45.0, -49.83, 5.71, 6.15, -28.47],
                    [-165.05, -57.56, -65.91, 35.33, 5.62, -28.47],
                ],
                2: [
                    [18.8, -26.36, -18.45, -45.61, 0.26, -26.36],
                    [166.64, -44.12, -1.05, -46.23, 4.92, -59.15],
                    [167.08, -72.68, 0.35, -14.76, 6.5, -54.14],
                    [166.81, -87.8, 1.14, -2.72, 6.32, -54.84]
                ],
            }

            pick_x_offset = self.config.pick_x_offset_dobby2
            pick_y_offset = self.config.pick_y_offset_dobby2
            pick_z_offset = self.config.pick_z_offset_dobby2
            
            place_x_offset = self.config.place_x_offset_dobby2
            place_y_offset = self.config.place_y_offset_dobby2
            place_z_offset = self.config.place_z_offset_dobby2

        
        # =========================================================
        # 🟦 도비 → 책장 place
        # =========================================================
        if mode == "DOBBY_TO_SHELF":
            
            book_id = self.slot_inventory.get_book_by_shelf(arco_id)
            
            carrier_slot_id = self.slot_inventory.get_slot_by_book(book_id)
            
            print(f"arco_id:{arco_id} book_id :{book_id} carrier_slot_id:{carrier_slot_id}")
            
            if carrier_slot_id is None:
                print(f"❌ 책 ID={book_id} 에 해당하는 슬롯을 찾을 수 없습니다!")
                return False
            
            poses = slot_poses.get(carrier_slot_id)
            
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
            approach[0] += (self.FORWARD_X_MM + place_x_offset)
            approach[1] += (self.FORWARD_Y_MM + place_y_offset)
            approach[2] = self.PICK_Z_HALF
            await self.safe_move(approach, speed=25)

            down = self.mc.get_coords()
            down[2] = self.PICK_Z_DOWN
            await self.safe_move(down, speed=25)

            self.mc.set_gripper_value(100, 50)
            print("📗 책 배치 완료")

            await self.safe_move(shelf_pose, speed=30)
            
            self.slot_inventory.remove_book(carrier_slot_id)
            
            print("✅ 도비→책장 완료")

        # =========================================================
        # 🟥 책장 → 도비 pick
        # =========================================================
        elif mode == "SHELF_TO_DOBBY":
            print("\n==============================")
            print(f"📦 [SHELF_TO_DOBBY] 시작 — arco_id={arco_id}")
            print("==============================")

            carrier_slot_id = self.slot_inventory.find_empty_slot()
            print(f"🔍 선택된 빈 슬롯: {carrier_slot_id}")

            if carrier_slot_id is None:
                print("❌ 비어있는 슬롯이 없습니다!")
                return False
            
            poses = slot_poses.get(carrier_slot_id)
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
            approach[0] += (self.FORWARD_X_MM + pick_x_offset)
            approach[1] += (self.FORWARD_Y_MM + pick_y_offset)
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
            lift[2] = self.PICK_Z_HALF + pick_z_offset
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
            
            self.slot_inventory.add_book(carrier_slot_id, arco_id)
            print(f"📚 Slot {carrier_slot_id} 에 책 {arco_id} 등록 완료")
            print("📦 도비 내부 슬롯 상태:", self.slot_inventory.slot_status)

        else:
            print(f"❌ 잘못된 mode 값: {mode}")
            return False

        # 홈 복귀
        print("🏠 홈 포즈 복귀 중...")
        await self.safe_move(home, speed=30)
        print("🏁 홈 복귀 완료 (Transfer 종료)")
        print("==============================\n")
        return True
