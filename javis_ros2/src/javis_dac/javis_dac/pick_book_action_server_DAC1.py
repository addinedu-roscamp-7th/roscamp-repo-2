import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from javis_interfaces.action import PickBook
from javis_dac.align_vision import AlignVision
from javis_dac.slot_inventory import SlotInventory
from javis_dac.detecting import Detecting
from javis_dac.robot_move import RobotMove
from javis_dac.config import Config
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
        self.align_vision = AlignVision.get_instance()
        self.slot_inventory = SlotInventory.get_instance()
        self.detecting = Detecting.get_instance()
        self.robot_move = RobotMove.get_instance()
        self.config = Config()
        
        self.get_logger().info("✅ PickBookActionServer initialized.")

    # =========================================================
    # 🔹 전체 시퀀스를 하나의 비동기 루프에서 순차 실행
    # =========================================================
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = PickBook.Feedback()
        result = PickBook.Result()

        self.get_logger().info(f"📚 PickBook goal received → Book ID: {goal.book_id}")

        try:
            # 비동기 루프 생성 및 실행
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

        if self.slot_inventory.find_empty_slot() is None:
            feedback.current_action = "⚠️ 저장소가 가득 차 있습니다."
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result.success = True
            result.message = "PickBook end"
            return

        await self.robot_move.send_angles_sync(base_angles, 25)
        base = self.robot_move.get_coords()

        feedback.current_action = "[STEP 1] 마커 탐색 중..."
        goal_handle.publish_feedback(feedback)

        self.detecting.reset_detected_markers()

        # 1️⃣ 8방향 탐색
        detected_id = None
        for dx, dy in [(0,0), (30,0), (-30,0), (0,30), (0,-30)]:
            detected_id = await self.detecting.scan_for_marker(base, target_id=found_target, dx=dx, dy=dy)

            if detected_id is None:
                continue

            if detected_id == found_target:
                print(f"🎯 목표 마커(ID={detected_id}) 탐색 성공 — 탐색 종료")
                break
            else:
                print(f"⚙️ 보조 마커(ID={detected_id}) 기준 대략 정렬 시도 중...")
                await self.detecting.approx_align_marker(detected_id)
                break

        # 2️⃣ Y축 탐색 (목표 마커 미발견 시에만)
        if found_target != detected_id:
            feedback.current_action = "[STEP 2] Y축 방향 탐색 중..."
            goal_handle.publish_feedback(feedback)

            approx_pose = self.robot_move.get_coords()

            for x_offset in [-20, 0, -30]:
                print(f"📍 Y축 탐색 실행 (x_offset={x_offset})")
                ok = await self.detecting.y_search_at_x(approx_pose, x_offset, found_target)
                if ok:
                    print(f"✅ 목표 마커 Y축 탐색 성공 (x_offset={x_offset})")
                    break
        else:
            print("⏩ 목표 마커가 이미 감지됨 — Y축 탐색 생략")

        # 3️⃣ 마커 정보 확보
        if found_target == 0:
            markers_info = self.detecting.get_all_detected_markers()
        elif found_target == detected_id:
            marker_info = {
                "id": detected_id,
                "pose": self.robot_move.get_coords(),
                "dist_pix": float(300),
                "timestamp": time.time()
            }
            markers_info = [marker_info]
        else:
            marker_info = self.detecting.get_marker_info(found_target)
            if not marker_info:
                raise ValueError(f"❌ ID={found_target} 마커 정보를 찾을 수 없습니다.")
            markers_info = [marker_info]

        if not markers_info:
            raise ValueError(f"❌ ID={found_target} 마커 정보를 찾을 수 없습니다.")

        # 4️⃣ 중심 정렬 + Yaw 정렬 + 책 이동
        for marker_info in markers_info:
            feedback.current_action = "[STEP 3] 중심 정렬 중..."
            goal_handle.publish_feedback(feedback)

            pose = marker_info["pose"]
            await self.robot_move.safe_move(pose, 40)
            pose = None

            for i in range(50):
                tolerance = self.config.center_tolerance
                done, val = await self.detecting.center_align_marker_step(marker_info, tolerance)

                if done:
                    self.get_logger().info(f"✅ 중심 정렬 완료 ({i+1}회 반복)")
                    pose = val
                    break

                if i == 1:
                    feedback.current_action = "[STEP 3-1] Yaw 정렬 중..."
                    goal_handle.publish_feedback(feedback)
                    await self.robot_move.align_yaw(marker_info["id"])

                if val is None:
                    self.get_logger().warn(f"⚠️ 중심 정렬 중단 (인식 실패 또는 변화 없음, step={i+1})")
                    break

                feedback.current_action = f"[STEP 3] 정렬 진행 중... (step {i+1})"
                goal_handle.publish_feedback(feedback)

            if pose is None:
                raise RuntimeError("❌ 중심 정렬 실패 또는 이동 정체 발생")

            feedback.current_action = "[STEP 4] 최종 Yaw 정렬 중..."
            goal_handle.publish_feedback(feedback)
            await self.robot_move.align_yaw(marker_info["id"])

            feedback.current_action = f"[STEP 5] 책장 → 도비 전송 중 (ID={marker_info['id']})..."
            goal_handle.publish_feedback(feedback)
            shelf_pose = self.robot_move.get_coords()
            await self.robot_move.transfer_book("SHELF_TO_DOBBY", shelf_pose, marker_info["id"], 1)

        # ✅ 완료 처리
        feedback.current_action = "[DONE] PickBook 완료!"
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.message = "PickBook complete."
        self.get_logger().info("✅ PickBook 시퀀스 정상 완료.")


def main(args=None):
    rclpy.init(args=args)
    node = PickBookActionServer_DAC1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 사용자가 중단함 (KeyboardInterrupt).")
    except Exception as e:
        node.get_logger().error("❌ PickBookActionServer에서 처리되지 않은 예외 발생:")
        node.get_logger().error(f"{type(e).__name__}: {e}")
        node.get_logger().error(traceback.format_exc())
    finally:
        node.get_logger().info("🔻 PickBookActionServer 종료 중...")
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("✅ ROS2 종료 완료.")


if __name__ == '__main__':
    main()
