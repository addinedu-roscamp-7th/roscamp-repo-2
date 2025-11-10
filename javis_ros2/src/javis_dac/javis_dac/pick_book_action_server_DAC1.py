import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from javis_interfaces.action import PickBook
from javis_dac.align_vision_manager import AlignVisionManager
from javis_dac.mc_singleton import MyCobotManager
import asyncio
import time
import traceback

class PickBookActionServer_DAC1(Node):
    def __init__(self):
        super().__init__('pick_book_action')

        self._action_server = ActionServer(
            self,
            PickBook,
            'dobby1/arm/pick_book',
            self.execute_callback
        )

        # 🦾 싱글톤 인스턴스 획득
        self.mc = MyCobotManager.get_instance()
        self.align = AlignVisionManager.get_instance()
        self.get_logger().info("✅ PlaceBookActionServer initialized.")

    # =========================================================
    # 🔹 전체 시퀀스를 하나의 비동기 루프에서 순차 실행
    # =========================================================


    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = PickBook.Feedback()
        result = PickBook.Result()
        self.get_logger().info(f"📚 PickBook goal received → Book ID: {goal.book_id}")

        try:
            # 하나의 루프 생성 → 순차 실행
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(
                self._run_pick_sequence(goal_handle, feedback, result)
            )
            loop.close()

        except Exception as e:
            # 🔥 전체 스택 로그 출력 (깊이 포함)
            tb_str = traceback.format_exc()
            msg = f"❌ PickBook failed: {type(e).__name__}: {e}\n{tb_str}"
            self.get_logger().error(msg)

            feedback.current_action = msg
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()

            result.success = False
            result.message = str(e)
        return result

    # =========================================================
    # 🧩 순차 실행 시퀀스 (비동기)
    # =========================================================
    async def _run_pick_sequence(self, goal_handle, feedback, result):

        goal = goal_handle.request
        
        found_target = int(goal.book_id)
        
        base_angles = [19.07, -15.38, -49.21, -25.04, 0.26, -26.36]
        
        await self.align.send_angles_sync(base_angles, 25)
        
        base = self.mc.get_coords()
        
        feedback.current_action = "[STEP 1] 8방향 탐색 중..."
        goal_handle.publish_feedback(feedback)

        self.align.reset_detected_markers()
        
        for dx, dy in [(0,0), (30,0), (-30,0), (0,30), (0,-30)]:
            detected_id = await self.align.scan_for_marker(base, target_id=found_target, dx=dx, dy=dy)

            if detected_id is None:
                continue  # 아무것도 감지 안 됨 → 다음 위치 탐색

            if detected_id == found_target:
                print("🎯 목표 마커 찾음 → 탐색 종료")
                break
            else:
                print(f"⚙️ anchor({detected_id}) 기준으로 대략 정렬 시도")
                await self.align.approx_align_marker(detected_id)
                break
        
        # --- 2️⃣ Y축 탐색 (단, 목표 마커 미발견 시에만 실행) ---
        if found_target != detected_id:
            feedback.current_action = "[STEP 2] Y축 탐색 시작..."
            goal_handle.publish_feedback(feedback)

            approx_pose = self.align.mc.get_coords()

            for x_offset in [-20, 0, -30]:
                print(f"📍 Y축 탐색 시작 (x_offset={x_offset})")
                ok = await self.align.y_search_at_x(approx_pose, x_offset, found_target)
                if ok:
                    print(f"✅ 목표 마커 Y축 탐색 성공 (x_offset={x_offset})")
                    break
                
        else:            
            print("⏩ 목표 마커 감지됨 → Y축 탐색 생략")
        
        
        # 리스트로 받아서 [아르코마커,6좌표] 리스트 => 반납대를 위한 방법
        # 시간도 추가해서 시간을 단축?
        if found_target == 0:
            markers_info = self.align.get_all_detected_markers()
            
        elif found_target == detected_id:
            marker_info = {
                "id": detected_id,
                "pose": self.align.mc.get_coords(),
                "dist_pix": float(300),
                "timestamp": time.time()
            }
            markers_info = [marker_info]

        else:
            marker_info = self.align.get_marker_info(found_target)
            if not marker_info:
                raise ValueError(f"❌ ID={found_target} 마커 정보를 찾을 수 없습니다.")
            markers_info = [marker_info]  # ✅ 리스트로 감싸기

        if not markers_info:
            raise ValueError(f"❌ ID={found_target} 마커 정보를 찾을 수 없습니다.")

        
        for marker_info in markers_info:
            
            # 2️⃣ 중심 정렬 (step-by-step 반복)
            feedback.current_action = "[STEP 3] Center aligning marker..."
            goal_handle.publish_feedback(feedback)

            pose = marker_info["pose"]    
            await self.align.safe_move(pose, 40)
            pose = None
            
            for i in range(50):  # 최대 50 스텝까지만 시도
                done, val = await self.align.center_align_marker_step(marker_info, self.align.CENTER_TOL)

                if done:
                    self.get_logger().info(f"✅ 중심 정렬 완료 ({i+1} steps)")
                    pose = val
                    break
                
                if(i==1):
                    # 3️⃣ Yaw 정렬
                    feedback.current_action = "[STEP 2] Aligning yaw..."
                    goal_handle.publish_feedback(feedback)
                    await self.align.align_yaw(marker_info["id"])
                    
                if val is None:
                    self.get_logger().warn(f"⚠️ 중심 정렬 중단 (변화 없음 또는 인식 실패, step={i+1})")
                    break

                # 각 스텝마다 피드백 전송 (실시간 모니터링용)
                feedback.current_action = f"[STEP 2] Aligning... (step {i+1})"
                goal_handle.publish_feedback(feedback)

            if pose is None:
                raise RuntimeError("❌ Center alignment failed or stagnant detected")

            # 3️⃣ Yaw 정렬
            feedback.current_action = "[STEP 4] Aligning yaw..."
            goal_handle.publish_feedback(feedback)
            await self.align.align_yaw(marker_info["id"])


            # 4️⃣ 책장 → 도비 이동
            feedback.current_action = "[STEP 5] Moving book (Shelf → Dobby)..."
            goal_handle.publish_feedback(feedback)
            shelf_pose = self.align.mc.get_coords()
            await self.align.transfer_book("SHELF_TO_DOBBY", shelf_pose, marker_info["id"], 1)
        
        
        # ✅ 완료
        feedback.current_action = "[DONE] PickBook complete!"
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.message = "PickBook complete."
        self.get_logger().info("✅ PickBook sequence complete.")


def main(args=None):
    rclpy.init(args=args)
    node = PickBookActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
