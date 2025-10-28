import depthai as dai
import cv2
import time
import os

save_dir = "dataset"
os.makedirs(save_dir, exist_ok=True)

pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.preview.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    count = 0

    while count < 500:
        frame = q.get().getCvFrame()
        cv2.imshow("frame", frame)

        cv2.imwrite(os.path.join(save_dir, f"{count:04d}.jpg"), frame)
        count += 1

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
