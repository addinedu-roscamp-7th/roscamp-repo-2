import sys
import os

print("Python executable:", sys.executable)
print("sys.path:", sys.path)

try:
    import ultralytics
    print("ultralytics imported successfully!")
except ImportError as e:
    print(f"Failed to import ultralytics: {e}")

