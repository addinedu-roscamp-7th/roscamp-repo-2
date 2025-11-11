import cv2
from ultralytics import YOLO
from boxmot import StrongSort
from pathlib import Path

# Re-ID 모델 가중치 파일 경로
REID_MODEL_PATH = Path("osnet_x0_25_msmt17.pt")

def main():
    # 직접 학습한 모델
    # model = YOLO("251015_study_yolov8.pt")
    model = YOLO("yolov8n.pt")

    # 웹캠 열기 : /dev/video2
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return

    # boxmot의 StrongSort 추적기 초기화
    tracker = StrongSort(
        reid_weights=REID_MODEL_PATH,
        device='cpu',
        half=False,
        max_age=1200,
        cmc_method="disable"
    )

    while True:
        # 웹캠에서 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            break

        # YOLO 모델로 객체 탐지
        results = model(frame, classes=[0], verbose=True, device='cpu')
        
        # 탐지 결과를 NumPy 배열로 변환하여 추적기에 업데이트
        tracks = tracker.update(results[0].boxes.data.cpu().numpy(), frame)

        # 추적 결과가 있을 경우에만 화면에 표시
        if tracks.shape[0] > 0:
            for track in tracks:
                # 필요한 값만 슬라이싱으로 안전하게 가져오기
                x1, y1, x2, y2, track_id = track[:5] # <--- 이 부분이 수정되었습니다.
                
                x1, y1, x2, y2, track_id = map(int, [x1, y1, x2, y2, track_id])

                # 바운딩 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # ID 번호 표시
                cv2.putText(
                    frame,
                    f"ID: {track_id}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        # 처리된 프레임을 화면에 보여주기
        cv2.imshow("YOLOv8 + StrongSort with Re-ID", frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()