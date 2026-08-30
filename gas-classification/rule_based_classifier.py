"""MQ 센서의 baseline 대비 반응 범위로 단일가스를 분류한다."""

import json
from pathlib import Path

import numpy as np


FEATURE_NAMES = (
    "mq3_change_ratio",
    "mq135_change_ratio",
    "mq138_change_ratio",
)

# MQ135는 가스 간 보정 범위가 많이 겹치고 순간 스파이크가 커서 후보를
# 탈락시키는 필수 조건으로 쓰지 않고, 후보 간 순위를 정할 때만 약하게 쓴다.
REQUIRED_FEATURES = ("mq3_change_ratio", "mq138_change_ratio")
# 보정 범위 경계에서 발생하는 작은 측정 흔들림은 범위 폭의 10%까지 허용한다.
RANGE_TOLERANCE = 0.10
FEATURE_WEIGHTS = {
    "mq3_change_ratio": 2.0,
    "mq135_change_ratio": 0.2,
    "mq138_change_ratio": 1.0,
}


def extract_rule_features(baseline_samples, response_samples):
    """각 MQ 센서의 대표 반응값과 baseline 사이의 변화율을 계산한다."""
    baseline = np.asarray(baseline_samples, dtype=np.float32)
    response = np.asarray(response_samples, dtype=np.float32)
    if baseline.ndim != 2 or response.ndim != 2 or baseline.shape[1] < 3 or response.shape[1] < 3:
        raise ValueError("baseline과 response는 각각 (샘플 수, 5) 배열이어야 합니다.")

    base = np.median(baseline[:, :3], axis=0)
    peak = np.max(response[:, :3], axis=0)
    # 기존 MQ3/MQ138 보정 범위와의 호환성은 유지하면서, 스파이크가 잦은
    # MQ135만 단일 최댓값 대신 반응 샘플의 90백분위수를 사용한다.
    peak[1] = np.percentile(response[:, 1], 90)
    ratios = (peak - base) / np.maximum(np.abs(base), 1.0)
    # 하강 반응과 잡음은 이 규칙에서 양의 가스 반응으로 사용하지 않는다.
    ratios = np.maximum(ratios, 0.0)
    return {name: float(value) for name, value in zip(FEATURE_NAMES, ratios)}


def load_database(path):
    path = Path(path)
    if not path.exists():
        return {"version": 1, "samples": {}, "rules": {}}
    with path.open(encoding="utf-8") as file:
        data = json.load(file)
    data.setdefault("samples", {})
    data.setdefault("rules", {})
    return data


def save_database(path, database):
    with Path(path).open("w", encoding="utf-8") as file:
        json.dump(database, file, ensure_ascii=False, indent=2)


def add_calibration(database, gas, features, padding=0.20):
    database["samples"].setdefault(gas, []).append(features)
    rebuild_rules(database, padding)


def rebuild_rules(database, padding=0.20):
    rules = {}
    for gas, samples in database["samples"].items():
        values = np.asarray([[sample[name] for name in FEATURE_NAMES] for sample in samples])
        low = np.percentile(values, 10, axis=0)
        high = np.percentile(values, 90, axis=0)

        # 표본이 적어 범위 폭이 0이어도 중심값의 ±padding을 허용한다.
        margin = np.maximum((high - low) * padding, np.maximum(np.abs((low + high) / 2) * padding, 0.05))
        rules[gas] = {
            name: {"min": float(max(0.0, lo - pad)), "max": float(hi + pad)}
            for name, lo, hi, pad in zip(FEATURE_NAMES, low, high, margin)
        }
    database["rules"] = rules


def classify(features, rules):
    """MQ3/MQ138 범위 후보 중 가중 중심 거리가 가장 가까운 가스를 반환한다."""
    matches = []
    scores = {}
    for gas, ranges in rules.items():
        inside = True
        weighted_distance = 0.0
        total_weight = 0.0
        for name in FEATURE_NAMES:
            value = features[name]
            low, high = ranges[name]["min"], ranges[name]["max"]
            if name in REQUIRED_FEATURES:
                tolerance = (high - low) * RANGE_TOLERANCE
                inside &= low - tolerance <= value <= high + tolerance
            center = (low + high) / 2
            half_width = max((high - low) / 2, 1e-6)
            weight = FEATURE_WEIGHTS[name]
            weighted_distance += weight * abs(value - center) / half_width
            total_weight += weight
        scores[gas] = float(weighted_distance / total_weight)
        if inside:
            matches.append(gas)

    if not matches:
        return "미분류", scores
    return min(matches, key=scores.get), scores
