import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from javis_interfaces.action import PlaceBook
from javis_dac.align_vision_manager import AlignVisionManager
from javis_dac.mc_singleton import MyCobotManager
import traceback
import asyncio
import time

class PlaceBookActionServer_DAC1(Node):
    def __init__(self):
        super().__init__('place_book_action')
        self._action_server = ActionServer(
            self,
            PlaceBook,
            'dobby1/arm/place_book',
            self.execute_callback
        )

        try:
            # 🦾 싱글톤 인스턴스 획득
            self.mc = MyCobotManager.get_instance()
            self.align = AlignVisionManager.get_instance()
            self.get_logger().info("✅ PlaceBookActionServer_DAC1 initialized.")
        except Exception as e:
            self.get_logger().error(f"❌ Initialization failed: {e}")
            raise

    # =========================================================
    # 🔹 전체 시퀀스를 하나의 비동기 루프에서 순차 실행
    # =========================================================
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = PlaceBook.Feedback()
        result = PlaceBook.Result()
        self.get_logger().info(f"📚 PlaceBook goal received → Book ID: {goal.book_id}")

        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(
                self._run_place_sequence(goal_handle, feedback, result)
            )
            loop.close()

        except Exception as e:
            tb_str = traceback.format_exc()
            msg = f"❌ PlaceBook failed: {type(e).__name__}: {e}\n{tb_str}"
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
    async def _run_place_sequence(self, goal_handle, feedback, result):
        goal = goal_handle.request
        found_target = self.align.get_shelf_by_book(int(goal.book_id))
        base_angles = [19.07, -15.38, -49.21, -25.04, 0.26, -26.36]

        await self.align.send_angles_sync(base_angles, 25)
        base = self.mc.get_coords()

        feedback.current_action = "[STEP 1] 마커 탐색 중..."
        goal_handle.publish_feedback(feedback)

        detected_id = None
        for dx, dy in [(0,0), (30,0), (-30,0), (0,30), (0,-30)]:
            detected_id = await self.align.scan_for_marker(base, target_id=found_target, dx=dx, dy=dy)

            if detected_id is None:
                continue

            if detected_id == found_target:
                print(f"🎯 목표 마커(ID={detected_id}) 탐색 성공 → 탐색 종료")
                break
            else:
                print(f"⚙️ 보조 마커(ID={detected_id}) 기준 대략 정렬 시도 중...")
                await self.align.approx_align_marker(detected_id)
                break

        # --- 2️⃣ Y축 탐색 (단, 목표 마커 미발견 시에만 실행) ---
        if found_target != detected_id:
            feedback.current_action = "[STEP 2] Y축 방향 탐색 중..."
            goal_handle.publish_feedback(feedback)

            approx_pose = self.align.mc.get_coords()

            for x_offset in [-20, 0, -30]:
                print(f"📍 Y축 탐색 실행 (x_offset={x_offset})")
                ok = await self.align.y_search_at_x(approx_pose, x_offset, found_target)
                if ok:
                    print(f"✅ 목표 마커(ID={found_target}) Y축 탐색 성공 (x_offset={x_offset})")
                    break

            if not found_target:
                raise RuntimeError("❌ Y축 탐색 실패 — 목표 마커 발견 안 됨")
        else:
            print("⏩ 목표 마커가 이미 감지됨 — Y축 탐색 생략")

        # --- 3️⃣ 마커 정보 확보 ---
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
            markers_info = [marker_info]

        if not markers_info:
            raise ValueError(f"❌ ID={found_target} 마커 정보를 찾을 수 없습니다.")

        # --- 4️⃣ 중심 정렬 + Yaw 정렬 + 책 이동 ---
        for marker_info in markers_info:
            feedback.current_action = "[STEP 3] 중심 정렬 중..."
            goal_handle.publish_feedback(feedback)

            pose = marker_info["pose"]
            await self.align.safe_move(pose, 40)
            pose = None

            for i in range(50):
                done, val = await self.align.center_align_marker_step(marker_info, self.align.CENTER_TOL)

                if done:
                    self.get_logger().info(f"✅ 중심 정렬 완료 ({i+1}회 반복)")
                    pose = val
                    break

                if i == 1:
                    feedback.current_action = "[STEP 3-1] Yaw 정렬 중..."
                    goal_handle.publish_feedback(feedback)
                    await self.align.align_yaw(marker_info["id"])

                if val is None:
                    self.get_logger().warn(f"⚠️ 중심 정렬 중단 (인식 실패 또는 변화 없음, step={i+1})")
                    break

                feedback.current_action = f"[STEP 3] 정렬 진행 중... (step {i+1})"
                goal_handle.publish_feedback(feedback)

            if pose is None:
                raise RuntimeError("❌ 중심 정렬 실패 또는 이동 정체 발생")

            feedback.current_action = "[STEP 4] 최종 Yaw 정렬 중..."
            goal_handle.publish_feedback(feedback)
            await self.align.align_yaw(marker_info["id"])

            # --- 5️⃣ 도비 → 책장 이동 ---
            feedback.current_action = f"[STEP 5] 책 배치 중 (Dobby → Shelf, ID={marker_info['id']})..."
            goal_handle.publish_feedback(feedback)

            shelf_pose = self.align.mc.get_coords()
            await self.align.transfer_book("DOBBY_TO_SHELF", shelf_pose, found_target, 1)

        # ✅ 완료
        feedback.current_action = "[DONE] PlaceBook 완료!"
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.message = "PlaceBook complete."
        self.get_logger().info("✅ PlaceBook 시퀀스 정상 완료.")


def main(args=None):
    rclpy.init(args=args)
    node = PlaceBookActionServer_DAC1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 사용자가 중단함 (KeyboardInterrupt).")
    except Exception as e:
        node.get_logger().error("❌ PlaceBookActionServer_DAC1에서 처리되지 않은 예외 발생:")
        node.get_logger().error(f"{type(e).__name__}: {e}")
        node.get_logger().error(traceback.format_exc())
    finally:
        node.get_logger().info("🔻 PlaceBookActionServer_DAC1 종료 중...")
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("✅ ROS2 종료 완료.")


if __name__ == '__main__':
    main()
