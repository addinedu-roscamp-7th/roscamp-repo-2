import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from javis_interfaces.action import PickBook
from javis_dac.mc_singleton import MyCobotManager
import time


# =========================================================
# ⚙️ 기본 설정
# =========================================================

HOME_POSE  = [180, 0, 250, -180, 0, -45]   # 초기 자세 (Z=250)
LOWER_POSE = [180, 0, 170, -180, 0, -45]   # 하강 자세 (Z=170)
SPEED = 25


class PickBookActionServer(Node):
    def __init__(self):
        super().__init__('pick_book_action')
        self.get_logger().info("✅ PickBook Action Server initializing...")

        # ✅ Action 서버 초기화
        self._action_server = ActionServer(
            self,
            PickBook,
            'dobby1/arm/pick_book',
            self.execute_callback
        )

        # 🤖 MyCobot 싱글톤 초기화
        try:
            self.mc = MyCobotManager.get_instance()
            time.sleep(2.0)
            self.get_logger().info("✅ PickBook Action Server Ready (Fixed Z-pose mode)")
        except Exception as e:
            self.get_logger().error(f"❌ MyCobot 초기화 실패: {e}")
            raise

    # -----------------------------------------------------
    # 📡 보조 함수: 좌표 전송 + 현재 위치 로깅
    # -----------------------------------------------------
    def move_and_log(self, coords, speed, desc=""):
        """로봇 이동 후 현재 좌표 로그"""
        self.mc.send_coords(coords, speed, 1)
        time.sleep(0.3)  # 전송 안정 대기

        current = self.mc.get_coords()
        if current:
            self.get_logger().info(
                f"🤖 {desc} 이동 완료 → "
                f"[X={current[0]:.1f}, Y={current[1]:.1f}, Z={current[2]:.1f}, "
                f"Rx={current[3]:.1f}, Ry={current[4]:.1f}, Rz={current[5]:.1f}]"
            )
        else:
            self.get_logger().warn(f"⚠️ {desc} 이동 후 좌표를 읽지 못했습니다.")

    # =========================================================
    # 🦾 액션 실행 콜백
    # =========================================================
    def execute_callback(self, goal_handle):
        book_id = goal_handle.request.book_id
        feedback = PickBook.Feedback()

        def publish(status: str, desc: str):
            feedback.current_action = f"[{status}] {desc}"
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(feedback.current_action)

        self.get_logger().info(f"📚 PickBook goal received → Book ID: {book_id}")

        try:
            # 1️⃣ 초기화
            publish("INIT", "Moving to home position...")
            self.move_and_log(HOME_POSE, SPEED, "홈 포즈")

            # 2️⃣ 하강
            publish("LOWERING", f"Moving down to Z={LOWER_POSE[2]}mm...")
            self.move_and_log(LOWER_POSE, SPEED, "하강")

            # 3️⃣ 픽업 시뮬레이션
            publish("PICKING", "Simulating book pickup...")
            time.sleep(1.5)
            current = self.mc.get_coords()
            if current:
                self.get_logger().info(f"📖 픽업 위치: {current}")

            # 4️⃣ 상승
            publish("RAISING", f"Returning to Z={HOME_POSE[2]}mm...")
            self.move_and_log(HOME_POSE, SPEED, "상승")

            # ✅ 성공
            goal_handle.succeed()
            result = PickBook.Result()
            result.success = True
            result.book_id = book_id
            result.message = "✅ Fixed-pose pick sequence completed successfully."
            self.get_logger().info(result.message)
            return result

        except Exception as e:
            goal_handle.abort()
            result = PickBook.Result()
            result.success = False
            result.book_id = book_id
            result.message = f"❌ PickBook failed: {e}"
            self.get_logger().error(result.message)
            return result


# =========================================================
# 🚀 메인 실행부
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = PickBookActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Action server stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
