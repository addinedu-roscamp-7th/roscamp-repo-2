
import cv2
from ultralytics import YOLO
from boxmot import StrongSort
from pathlib import Path
import numpy as np

class ReIDTracker:
    def __init__(self, reid_model_path, device='cpu', half=False):
        self.model = YOLO("yolov8n.pt")
        self.tracker = StrongSort(
            reid_weights=Path(reid_model_path),
            device=device,
            half=half,
            max_age=1200,
            cmc_method="disable"
        )

    def update(self, frame):
        results = self.model(frame, classes=[0], verbose=False, device='cpu')
        tracks = self.tracker.update(results[0].boxes.data.cpu().numpy(), frame)
        return tracks

    def get_primary_target(self, tracks):
        """
        지정된 ID(예: 1)를 가진 타겟을 찾습니다.
        """
        if tracks.shape[0] == 0:
            return None

        target_id_to_track = 1  # 추적할 특정 ID

        for track in tracks:
            # track 배열의 5번째 요소가 track_id 입니다.
            if int(track[4]) == target_id_to_track:
                return track  # ID가 일치하는 타겟을 찾으면 즉시 반환

        return None  # 해당 ID를 가진 타겟을 찾지 못하면 None 반환
