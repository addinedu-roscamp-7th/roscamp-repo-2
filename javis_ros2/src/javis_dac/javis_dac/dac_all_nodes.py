import rclpy
from rclpy.executors import MultiThreadedExecutor
from javis_dac.pick_book_action_server import PickBookActionServer
from javis_dac.place_book_action_server import PlaceBookActionServer
import time


def main(args=None):
    rclpy.init(args=args)
    print("🚀 [DAC] Multi-node Action Executor starting...")

    # =========================================================
    # 📚 PickBook 서버 초기화
    # =========================================================
    try:
        pick_node = PickBookActionServer()
        print("✅ PickBookActionServer initialized.")
    except Exception as e:
        print(f"❌ PickBookActionServer init failed: {e}")
        pick_node = None

    # 살짝 텀을 주어 포트 초기화 타이밍 조정
    time.sleep(0.5)

    # =========================================================
    # 📦 PlaceBook 서버 초기화
    # =========================================================
    try:
        place_node = PlaceBookActionServer()
        print("✅ PlaceBookActionServer initialized.")
    except Exception as e:
        print(f"❌ PlaceBookActionServer init failed: {e}")
        place_node = None

    # =========================================================
    # 🧵 Multi-threaded Executor 실행
    # =========================================================
    executor = MultiThreadedExecutor()
    if pick_node:
        executor.add_node(pick_node)
    if place_node:
        executor.add_node(place_node)

    if not (pick_node or place_node):
        print("⚠️ No valid nodes to run. Shutting down.")
        rclpy.shutdown()
        return

    try:
        print("🌀 Spinning executor (Ctrl+C to stop)...")
        executor.spin()
    except KeyboardInterrupt:
        print("\n🛑 KeyboardInterrupt received — shutting down...")
    finally:
        if pick_node:
            pick_node.destroy_node()
        if place_node:
            place_node.destroy_node()
        rclpy.shutdown()
        print("✅ All DAC nodes stopped cleanly.")


if __name__ == '__main__':
    main()
