# detector.py — 화재·연기 감지 (HuggingFace 사전학습 모델)
#
# rabahdev/fire-smoke-yolov8n (D-Fire로 파인튜닝된 YOLOv8n)을 그대로 씁니다.
# 직접 전이학습 없이 바로 씁니다. 클래스: 0=smoke, 1=fire (모델 카드 기준)

import numpy as np
from huggingface_hub import hf_hub_download
from ultralytics import YOLO

import config

CLASS_NAMES = {0: "smoke", 1: "fire"}


class FireSmokeDetector:
    def __init__(self):
        print("[detector] 모델 다운로드 중 (처음 한 번만, 이후 캐시 사용)...")
        weights_path = hf_hub_download(
            repo_id=config.FIRE_MODEL_REPO, filename=config.FIRE_MODEL_FILE
        )
        self.model = YOLO(weights_path)
        self.device = self._pick_device()
        print(f"[detector] 준비 완료 (device={self.device})")

    @staticmethod
    def _pick_device() -> str:
        import torch
        if torch.backends.mps.is_available():
            return "mps"
        if torch.cuda.is_available():
            return "cuda"
        return "cpu"

    def detect(self, frame_bgr: np.ndarray) -> dict:
        """한 프레임을 검사해 화재·연기 감지 여부를 돌려줍니다.
        박스 좌표(xyxy)는 원본 frame_bgr 해상도 기준으로 돌려줍니다 — ultralytics가
        추론용으로 축소했던 좌표를 원본 크기로 다시 스케일링해서 반환하기 때문에,
        호출부(main.py)가 별도 변환 없이 그대로 그리면 됩니다."""
        results = self.model.predict(
            frame_bgr,
            conf=config.FIRE_CONF_THRESHOLD,
            imgsz=config.FIRE_INFER_WIDTH,
            device=self.device,
            verbose=False,
        )
        boxes = results[0].boxes

        flame_detected = False
        smoke_detected = False
        max_conf = 0.0
        box_list = []

        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = (round(v) for v in box.xyxy[0].tolist())
            if cls_id == 1:  # fire
                flame_detected = True
            elif cls_id == 0:  # smoke
                smoke_detected = True
            max_conf = max(max_conf, conf)
            box_list.append({
                "cls": CLASS_NAMES.get(cls_id, str(cls_id)),
                "conf": round(conf, 3),
                "xyxy": [x1, y1, x2, y2],
            })

        return {
            "flame_detected": flame_detected,
            "smoke_detected": smoke_detected,
            "confidence": round(max_conf, 3) if max_conf > 0 else None,
            "boxes": box_list,
        }
