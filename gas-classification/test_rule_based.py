"""단일가스 범위를 보정하고 MQ3/MQ135/MQ138 반응으로 분류한다."""

import argparse

from live_test_utils import collect_samples, open_serial
from rule_based_classifier import (
    FEATURE_NAMES,
    add_calibration,
    classify,
    extract_rule_features,
    load_database,
    save_database,
)


DEFAULT_DB = "rule_ranges.json"
BASELINE_SAMPLES = 5
RESPONSE_SAMPLES = 50


def parse_args():
    parser = argparse.ArgumentParser(description="MQ 센서 단일가스 룰 기반 분류")
    parser.add_argument("mode", choices=("calibrate", "predict"), help="범위 보정 또는 분류")
    parser.add_argument("--gas", help="보정할 가스 이름 (calibrate에서 필수)")
    parser.add_argument("--port", default="auto", help="시리얼 포트 (기본: 자동 탐색)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--db", default=DEFAULT_DB, help=f"범위 DB 경로 (기본: {DEFAULT_DB})")
    parser.add_argument("--padding", type=float, default=0.20, help="생성 범위 여유 비율 (기본: 0.20)")
    args = parser.parse_args()
    if args.mode == "calibrate" and not args.gas:
        parser.error("calibrate 모드에는 --gas가 필요합니다.")
    if args.padding < 0:
        parser.error("--padding은 0 이상이어야 합니다.")
    return args


def measure(port, baud):
    ser = open_serial(port, baud)
    try:
        print("깨끗한 공기 상태를 유지하세요.")
        baseline = collect_samples(ser, BASELINE_SAMPLES, "Baseline 수집")
        input("이제 단일가스를 노출시킨 직후 Enter를 누르세요...")
        response = collect_samples(ser, RESPONSE_SAMPLES, "가스 반응 수집")
    finally:
        ser.close()
    return extract_rule_features(baseline, response)


def print_features(features):
    print("\n측정된 최대 변화율:")
    for name in FEATURE_NAMES:
        print(f"  {name:<22} {features[name]:.3f} ({features[name] * 100:.1f}%)")


def print_rule(gas, rule):
    print(f"\n[{gas}] 생성 범위")
    for name in FEATURE_NAMES:
        bounds = rule[name]
        print(f"  {name:<22} {bounds['min']:.3f} ~ {bounds['max']:.3f}")


def main():
    args = parse_args()
    database = load_database(args.db)
    features = measure(args.port, args.baud)
    print_features(features)

    if args.mode == "calibrate":
        add_calibration(database, args.gas, features, args.padding)
        save_database(args.db, database)
        count = len(database["samples"][args.gas])
        print_rule(args.gas, database["rules"][args.gas])
        print(f"\n보정 저장 완료: {args.db} ({args.gas} {count}회 측정)")
        if count < 5:
            print("안정적인 범위를 위해 같은 가스를 최소 5회 이상 보정하는 것을 권장합니다.")
        return

    if not database["rules"]:
        raise SystemExit("생성된 규칙이 없습니다. 먼저 calibrate 모드로 가스별 보정을 수행하세요.")
    result, scores = classify(features, database["rules"])
    print("\n=== 룰 기반 최종 판정 ===")
    print(result)
    print("후보별 중심 거리: " + ", ".join(f"{gas}={score:.3f}" for gas, score in scores.items()))


if __name__ == "__main__":
    main()
