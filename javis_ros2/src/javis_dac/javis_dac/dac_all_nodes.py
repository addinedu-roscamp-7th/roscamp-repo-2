import rclpy
from rclpy.executors import MultiThreadedExecutor
from javis_dac.pick_book_action_server_DAC1 import PickBookActionServer_DAC1
from javis_dac.pick_book_action_server_DAC2 import PickBookActionServer_DAC2
from javis_dac.place_book_action_server_DAC1 import PlaceBookActionServer_DAC1
from javis_dac.place_book_action_server_DAC2 import PlaceBookActionServer_DAC2
import time

import socket


def main(args=None):
    rclpy.init(args=args)
    print("🚀 [DAC] Multi-node Action Executor starting...")
    
    # 기준 IP
    ip1 = "192.168.0.167"  # dobby1
    ip2 = "192.168.0.168"  # dobby2

    my_ip = get_local_ip()
    print("📡 Local IP:", my_ip)

    if my_ip == ip1:
        dobby_num = 1
    elif my_ip == ip2:
        dobby_num = 2
    else:
        raise ValueError(f"🚫 Unauthorized IP: {my_ip}.")


    print(f"🤖 dobby_num = {dobby_num}")
    
    if dobby_num == 1 :
        # =========================================================
        # 📚 PickBook 서버 초기화
        # =========================================================
        try:
            pick_node = PickBookActionServer_DAC1()
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
            place_node = PlaceBookActionServer_DAC1()
            print("✅ PlaceBookActionServer initialized.")
        except Exception as e:
            print(f"❌ PlaceBookActionServer init failed: {e}")
            place_node = None
        
        # 살짝 텀을 주어 포트 초기화 타이밍 조정
        time.sleep(0.5)

    if dobby_num == 2 :
        # =========================================================
        # 📚 PickBook 서버 초기화
        # =========================================================
        try:
            pick_node = PickBookActionServer_DAC2()
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
            place_node = PlaceBookActionServer_DAC2()
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

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return ip

if __name__ == '__main__':
    main()
