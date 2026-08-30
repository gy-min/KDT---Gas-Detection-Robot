# dataset_loader.py — split/train, split/val, split/test 폴더의 CSV들을
# (샘플 수, 50, 3) 형태의 시계열 배열과 (샘플 수, 3) 다중 라벨 배열로 불러옵니다.
#
# 센서 3개(MQ3, MQ135, MQ138)를 입력 특징으로, 라벨 3개(에탄올/아세톤/암모니아)를
# 다중 라벨(동시에 여러 개가 1일 수 있음)로 다룹니다.

import csv
from pathlib import Path

import numpy as np

SEQ_LEN = 50
FEATURE_COLS = ["MQ3", "MQ135", "MQ138", "temperature", "humidity"]
LABEL_COLS = ["label_에탄올", "label_아세톤", "label_암모니아"]


def load_one_csv(path: Path):
    with open(path, encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None, None

    # 50개로 고정: 51개면 앞에서 50개만, 그 미만이면 마지막 행으로 패딩
    if len(rows) >= SEQ_LEN:
        rows = rows[:SEQ_LEN]
    else:
        rows = rows + [rows[-1]] * (SEQ_LEN - len(rows))

    seq = np.array([[float(r[c]) for c in FEATURE_COLS] for r in rows], dtype=np.float32)
    label = np.array([int(rows[0][c]) for c in LABEL_COLS], dtype=np.float32)
    return seq, label


def load_split(split_dir: str):
    """split_dir(train/val/test 폴더 경로) 안의 모든 CSV를 불러옵니다.
    반환: X (N, 50, 3), y (N, 3), filenames (N개)"""
    files = sorted(Path(split_dir).glob("*.csv"))
    X, y, names = [], [], []
    for f in files:
        seq, label = load_one_csv(f)
        if seq is None:
            continue
        X.append(seq)
        y.append(label)
        names.append(f.name)
    return np.stack(X), np.stack(y), names


def normalize(X: np.ndarray, mean=None, std=None):
    """센서별(채널별) 평균/표준편차로 정규화. train에서 구한 mean/std를 val/test에도 그대로 씁니다."""
    if mean is None:
        mean = X.mean(axis=(0, 1), keepdims=True)
        std = X.std(axis=(0, 1), keepdims=True) + 1e-6
    return (X - mean) / std, mean, std


if __name__ == "__main__":
    # 간단한 동작 확인
    import sys
    split_dir = sys.argv[1] if len(sys.argv) > 1 else "train"
    X, y, names = load_split(split_dir)
    print(f"{split_dir}: X shape={X.shape}, y shape={y.shape}")
    print(f"라벨 평균(각 클래스 비율): {y.mean(axis=0)}")