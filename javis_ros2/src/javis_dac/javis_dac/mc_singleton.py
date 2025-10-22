import threading
from pymycobot.mycobot import MyCobot

class MyCobotManager:
    _instance = None
    _lock = threading.Lock()

    PORT = "/dev/ttyUSB0"
    BAUD = 1_000_000

    def __init__(self):
        if MyCobotManager._instance is not None:
            raise RuntimeError("Use get_instance() instead of direct instantiation")
        self.mc = MyCobot(MyCobotManager.PORT, MyCobotManager.BAUD)
        print(f"✅ [Singleton] MyCobot initialized at {MyCobotManager.PORT}")

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = MyCobotManager()
        return cls._instance.mc
