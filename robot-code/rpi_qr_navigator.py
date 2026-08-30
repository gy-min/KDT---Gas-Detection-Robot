#!/usr/bin/env python3
"""
라파5 -- 가스 탐지 로봇 내비게이션 (지도 기반 방위 + MQTT 임무 제어)

8/19 개정 -- 4가지 기능 추가:
  1. 초기 위치/방향: 출발 모서리의 QR 2개를 동시 인식해서 시작 노드와
     초기 방위를 스스로 확정 (사람이 입력할 필요 없음)
  2. 목표점: MQTT로 고정형 가스 센서 쪽에서 "어디서 감지됐는지"를 받으면
     그 노드가 목표가 됨
  3. 퇴로: 목표 도착 후 별도 명령이 없으면 그래프 최단경로로 가장 가까운
     출구(TL/TR/BL/BR)까지 자동 복귀
  4. 라파 자체도 ESP32처럼 MQTT로 로그 발행 + 명령 수신

8/20 개정 -- 3가지 수정:
  A. 구역 판별 후 복귀에 U턴 + 가상 출발 구간 도입 (아래 상세)
  B. 노드 도착 반경 확대 (간선당 도착 판정 QR 이 1개뿐이던 문제)
  C. NEAR 플래그를 ESP32 물리 진행도(effective_leg) 기준으로 계산

  A 상세 -- 기존 버그:
    구역 중간에서 3초 정지한 뒤, 로봇은 '진입 노드를 등지고' 서 있다.
    그런데 복귀 경로는 진입 노드에서 시작하도록 만들어져서,
      - U턴이 없으니 로봇은 계속 앞으로(구역 안쪽으로) 굴러갔고
      - 첫 교차로에서 쓸 회전각이 한 칸 밀려 엉뚱한 노드 기준으로 계산됐다.
    (예: A2 구역을 A에서 진입 -> 정지 -> 복귀경로 ['A','TL'] -> tvec 항상
     0.0 -> B/TR 쪽으로 계속 직진하며 영영 복귀 못 함)
    수정: 정지 지점을 가상 노드(_ZMID)로 두고, '_ZMID -> 진입노드' 구간을
    경로 맨 앞에 끼워넣는다. 그 구간의 방위는 구역 방위의 반대(+180)다.
    동시에 ESP32에 CMD:u 를 보내 실제로 제자리 U턴을 시킨다.

상태 머신:
  INIT      -- 출발 모서리에서 QR 2개 동시 인식 대기 (위치+방위 확정)
  IDLE      -- 대기. MQTT 목표 알림이나 수동 명령을 기다림
  NAVIGATE  -- NODE_ROUTE 를 따라 주행 (지도 기반 방위 계산, 기존 로직)
  ZONE_ENTER-- 구역 안쪽 중간 지점까지 진입
  ZONE_STOP -- 가스 판별용 정지
  RETURN    -- 목표 도착 후 자동으로 가장 가까운 출구까지 복귀

필요 설치:
  picamera2 는 라파OS 기본 포함
  sudo apt install -y python3-opencv
  pip3 install pyserial paho-mqtt --break-system-packages

배선 (교차 연결, GPIO13/14):
  라파 GPIO14 (TXD, 8번 핀)  -> ESP32 GPIO13 (RX)
  라파 GPIO15 (RXD, 10번 핀) <- ESP32 GPIO14 (TX)
  GND 공통

MQTT 토픽 (라파 쪽, ESP32 의 robot/1/cmd 와는 별개):
  구독:
    robot/1/gas_target   -- 고정형 센서가 가스 감지 구역(A1~C3)을 알려줌
    robot/1/rpi_cmd      -- 수동 명령. "return"=즉시 복귀, "stop"=정지,
                             "goto:X"=노드 X로 강제 이동
  발행:
    robot/1/rpi_state    -- 0.5초마다 현재 상태(JSON)
    robot/1/rpi_mark     -- 이벤트 로그(노드 도착, 목표 수신, 복귀 시작 등)
    robot/1/rpi_online   -- 생존 여부 (LWT)
"""

import cv2
from picamera2 import Picamera2
from libcamera import Transform
import serial
import json
import os
import time
import math
import re
import sys
import threading
import io
from collections import deque
import socketserver
from http import server as http_server

import paho.mqtt.client as mqtt

# 8/21 통합 -- 가스 센서 5개(MQ2 좌/우, MQ3, MQ135, MQ138)를 별도 아두이노가
# USB 시리얼로 보낸다(gas_sensor_arduino.ino). rule_ranges.json 이 그 아두이노
# 회로 기준으로 보정돼 있어 MQ3/135/138 판별은 반드시 이 경로여야 하고,
# MQ2 도 위험도 계산 일관성을 위해 같은 보드로 옮겼다.
# 라파 SPI(MCP3008)는 더 이상 쓰지 않는다 -- 5채널이 전부 0.0 으로 읽히던
# 문제도 이 교체로 함께 해소된다.
from rule_based_classifier import extract_rule_features, load_database, classify as rule_classify
from serial_connection import resolve_port, port_help_message

from map_data import (
    NODES, EDGES, EXIT_NODES, get_route, find_path, nearest_exit,
    nearest_node_to_coord, zone_info, edge_axis, ZONES,
)

# ── 설정 ──────────────────────────────────────────────
UART_PORT = "/dev/ttyAMA0"
UART_BAUD = 115200
SEND_INTERVAL_SEC = 0.2

MQTT_HOST = os.environ.get("MQTT_HOST", "YOUR_SERVER_IP")   # 예: 192.168.0.3
MQTT_PORT = 1883
ROBOT_ID = 1

TOPIC_GAS_TARGET = f"robot/{ROBOT_ID}/gas_target"
TOPIC_RPI_CMD    = f"robot/{ROBOT_ID}/rpi_cmd"

# 8/19 추가 -- ESP32가 더 이상 자체 WiFi/MQTT 를 안 쓰므로, 라파가 대신
# ESP32 몫의 MQTT 토픽까지 발행/구독한다. ESP32 <-> 라파는 UART 로만 통신.
TOPIC_ESP32_CMD   = f"robot/{ROBOT_ID}/cmd"     # 원래 ESP32가 구독하던 긴급정지 토픽
TOPIC_ESP32_STATE = f"robot/{ROBOT_ID}/state"   # 원래 ESP32가 발행하던 상태 토픽
TOPIC_ESP32_MARK  = f"robot/{ROBOT_ID}/mark"    # 원래 ESP32가 발행하던 이벤트 토픽
TOPIC_ESP32_ONLINE = f"robot/{ROBOT_ID}/online" # ESP32 생존 여부
TOPIC_RPI_STATE  = f"robot/{ROBOT_ID}/rpi_state"
TOPIC_RPI_MARK   = f"robot/{ROBOT_ID}/rpi_mark"
TOPIC_RPI_ONLINE = f"robot/{ROBOT_ID}/rpi_online"

# 8/20 추가 -- 서버(safeDB)가 실제로 구독하는 토픽. config.py MQTT_TOPICS에
# 정확히 이 문자열("robot/event")로만 등록돼 있어 와일드카드가 아니다.
# robot_event/robot_sensor(+판별 시 robot_gas_result)에 그대로 저장된다.
TOPIC_ROBOT_EVENT = "robot/event"

# 8/20 수정 -- 1.6 에서 2.6 으로 확대.
# 지도 전수 검증 결과, 1.6 에서는 각 간선마다 도착을 인정해주는 QR 이
# 정확히 하나뿐이었다. 특히 세로 간선(A-C, C-E, B-D, D-F)은 마지막 QR 이
# 거리 1.5 라 임계값과의 마진이 0.1 밖에 안 됐고, 그 QR 하나를 모션블러로
# 놓치면 node_idx 가 영영 안 올라가 25초 뒤 STUCK_TIMEOUT 으로 정지했다.
# 2.6 으로 올리면 간선마다 후보 QR 이 2개(거리 1.5/2.5 또는 1.0/2.0)가 되어
# 재시도 기회가 생긴다. 노드 간 최소 간격이 3 이상이라 혼동 위험은 없다.
# 도착 판정이 그만큼 일찍 나지만, 실제 회전은 esp32_crossing_count 게이팅이
# 잡아주므로 문제되지 않는다.
WAYPOINT_REACHED_RADIUS = 2.6

# 8/20 추가 -- 도착 반경의 구간 길이 대비 상한.
# 실측 사고: 구역 정지 후 복귀 경로 _ZMID(5,2) -> ML(5,1) 은 구간 길이가 1
# 인데 반경이 2.6 이라, 출발 지점에서 눈앞의 QR (5,3) 을 읽자마자 열 거리
# 2.0 <= 2.6 이 성립해 "ML 도착"이 즉시 떴다. 로봇은 움직이지도 않았는데
# 2초 정지 + U턴 시퀀스가 엉뚱한 자리에서 시작됐다.
# 구간의 이 비율만큼은 실제로 지나와야 도착으로 인정한다.
ARRIVE_LEG_FRACTION = 0.8

# 8/20 추가 -- ZONE_ENTER 전용 반경(2D 유클리드).
# 구역 진입 구간은 방향이 꺾이는 지점이라, 막 지나온 이전 복도의 QR 이
# 우연히 맞아 "진입 완료"로 오판되기 쉽다. 여기만은 좁게 유지한다.
# (예: A2 구역 진입 시 직전 TL-A 구간의 (0,3) 과 목표 (0,5) 의 거리가 2.0)
ZONE_WAYPOINT_RADIUS = 1.6

ZONE_STOP_SECONDS = 3.0   # 구역 중간 지점에서 가스 판별을 위해 정지할 시간

# 8/20 추가 -- 출구 복귀 U턴 시퀀스.
#   EXIT_HALT: 끝 노드(출구)에 도착하면 이만큼 정지했다가 U턴을 시작한다.
#   SEEK_TIMEOUT: U턴·정렬이 끝난 뒤 위치 확인용 QR 을 찾는 단계의 상한.
#     못 찾으면 그냥 정지시킨다(무한 전진 방지).
EXIT_HALT_SECONDS = 2.0
# 8/21 -- 의미 변경. 이제 이 값은 "U턴이 끝난 뒤" QR 을 찾는 데 쓰는
# 시간이다. 피벗에 걸리는 시간은 아래 UTURN_DONE_TIMEOUT_SEC 이 따로 본다.
UTURN_SEEK_TIMEOUT_SEC = 20.0
# 8/21 추가 -- CMD:u 를 보낸 뒤 ESP32 의 uturn_done 마크를 기다리는 상한.
# U턴 1단계(끝라인 통과 + 연장선 끝 도달)는 ESP32 쪽에 타임아웃이 없고,
# 2단계(피벗)에만 uTurnMaxPivotMs(6초) 상한이 있다. 라인이 없는 자리에
# 놓이면 1단계에서 무한 전진할 수 있으므로 라파가 상한을 건다.
# 정상이면 대개 3~5초 안에 uturn_done 이 온다.
# 피벗 상한 6초 + 정렬 상한 6초 + 1단계 전진 시간을 모두 담아야 하므로
# 20초로 둔다. 정상이면 대개 5~8초 안에 두 신호가 모두 온다.
UTURN_DONE_TIMEOUT_SEC = 20.0
LEG_LAG_TIMEOUT_SEC = 3.0   # ESP32 교차로 확인이 이 시간 넘게 안 오면 강제 진행

# 디버그용 실시간 영상 확인 -- http://<라파IP>:8000/
STREAM_ENABLED = True
STREAM_PORT = 8000

# 8/20 -- INIT 에서 읽은 QR 이 이 거리 안의 노드와 맞아야 위치를 확정한다.
# 이보다 멀면 출발 노드를 이미 지나쳤다는 뜻이므로 확정하지 않고 정지한다.
# 모서리 QR 은 노드에서 1.0~1.5 정도 떨어져 있으므로 2.0 이면 넉넉하다.
# 8/20 재조정 -- 실측에서 정확히 2.0(TR 기준 QR (0,10))이 나왔는데 비교가
# "미만"(<)이라 경계값이 계속 실패로 튕겨나갔다. 2.2로 살짝 여유를 둔다.
INIT_NODE_MAX_DIST = 2.2

# 8/20 재개정 -- 반대편 벽 QR 추가.
# 카메라가 로봇 오른쪽에 고정돼 있어 진행 방향에 따라 보이는 벽이 달라진다.
# B3(D->MR, 동쪽)에서는 남쪽 벽인 행 6 이 보이는데 표에 행 5 만 있어
# 2D 거리 0.9 로는 절대 안 맞았다. 실측에서 로봇이 (6,11)만 계속 읽으며
# 정지 QR을 하나도 못 잡고 MR 끝라인까지 갔다.
# 각 구역마다 양쪽 벽 좌표를 모두 넣는다 (행 0/1, 5/6, 11/10).
ZONE_STOP_QR = {
    "A1": [(0, 2), (0, 3), (1, 2), (1, 3)],
    "A2": [(0, 5), (0, 6), (0, 7), (1, 5), (1, 6), (1, 7)],
    "A3": [(0, 9), (0, 10), (1, 9), (1, 10)],
    "B1": [(5, 2), (5, 3), (6, 2), (6, 3)],
    "B2": [(5, 5), (5, 6), (5, 7), (6, 5), (6, 6), (6, 7)],
    "B3": [(5, 9), (5, 10), (6, 9), (6, 10)],
    "C1": [(11, 2), (11, 3), (10, 2), (10, 3)],
    "C2": [(11, 5), (11, 6), (11, 7), (10, 5), (10, 6), (10, 7)],
    "C3": [(11, 9), (11, 10), (10, 9), (10, 10)],
}

# 후보 QR 매칭 반경(2D). QR 좌표는 정수라 정확히 일치하면 0, 이웃이면 1.0
# 이상이므로 0.9 면 오인 없이 넉넉하다.
ZONE_STOP_QR_RADIUS = 0.9

# 8/20 추가 -- ZONE_ENTER 안전망.
# 후보 QR을 전부 놓쳐도 로봇이 구역 밖으로 나가지 않도록 두 겹을 둔다.
#   1) esp32_crossing_count 가 목표를 넘어서면 = 구역 반대편 끝 교차선을
#      물리적으로 통과했다는 뜻이므로 즉시 정지 (카메라와 독립적이라 확실)
#   2) 그마저 못 잡으면 시간으로 끊는다
ZONE_ENTER_TIMEOUT_SEC = 15.0

# 8/20 추가 -- 구역 정지 지점을 나타내는 가상 노드 이름.
# 실제 지도(NODES)에는 없고 이 파일 안에서만 통용된다. map_data 를 건드리지
# 않으려고 별도로 두었으며, nearest_node_to_coord 가 이걸 반환할 일도 없어
# 경로이탈 재탐색 로직과도 충돌하지 않는다.
VIRTUAL_START = "_ZMID"


# ── 가스 센서 5개(MQ2 좌/우, MQ3, MQ135, MQ138) — 별도 아두이노, USB 시리얼 ──
# 8/21 통합. ESP32 UART(/dev/ttyAMA0, 고정 포트)와는 별개 장치라
# 자동탐색(ttyUSB*/ttyACM*)으로 잡는다. 아직 연결 전이어도 죽지 않고
# '미분류' / 세기 0 으로만 계속 진행한다.
GAS_ARDUINO_PORT = "auto"
GAS_ARDUINO_BAUD = 115200
RULE_RANGES_PATH = "rule_ranges.json"
GAS_BASELINE_LEN = 6   # 구역 진입 직전 "정상 공기" 대표값으로 쓸 최근 샘플 개수

GAS_ALERT_WARN_THRESHOLD = 40.0
GAS_ALERT_DANGER_THRESHOLD = 70.0

RULE_DB = load_database(RULE_RANGES_PATH)
if not RULE_DB.get("rules"):
    print(f"[GAS] 경고 -- {RULE_RANGES_PATH}에 보정된 규칙이 없음. 가스 종류는 계속 '미분류'로만 나옴")

gas_lock = threading.Lock()
gas_baseline_buffer = deque(maxlen=GAS_BASELINE_LEN)   # 최근 샘플(정상 공기 대표값)
gas_response_buffer = []                                # ZONE_STOP 동안만 채워짐
gas_collecting_response = False
gas_arduino_serial = None


def parse_gas_line(line: str):
    """gas_sensor_arduino.ino 가 보내는 JSON 한 줄을 dict 로 변환한다.
    예: {"mq2_left":.., "mq2_right":.., "mq3":.., "mq135":.., "mq138":.., ...}
    필수 키 중 하나라도 없으면 버린다."""
    s = line.strip()
    if not s.startswith("{"):
        return None
    try:
        d = json.loads(s)
        required = ("mq2_left", "mq2_right", "mq3", "mq135", "mq138")
        return {k: float(d[k]) for k in required}
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None


def set_gas_collecting(flag: bool):
    """ZONE_STOP 진입/종료 시 gas_arduino_reader 에게 response 버퍼를
    채울지 알려준다 (모듈 전역이라 함수로 감싸 global 없이 쓴다)."""
    global gas_collecting_response
    gas_collecting_response = flag


def gas_arduino_reader(ser):
    """가스 아두이노 -> 라파 시리얼 리더. 백그라운드 스레드에서 계속 돈다.
    평소엔 최근 샘플을 baseline 버퍼에 쌓아두고(구역 진입 직전 정상 공기),
    ZONE_STOP 동안에는 response 버퍼에도 같이 쌓는다."""
    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore")
        except Exception:
            time.sleep(0.1)
            continue
        parsed = parse_gas_line(raw)
        if parsed is None:
            continue
        with gas_lock:
            gas_baseline_buffer.append(parsed)
            if gas_collecting_response:
                gas_response_buffer.append(parsed)


def _avg_key(samples, key):
    vals = [s[key] for s in samples if key in s]
    return sum(vals) / len(vals) if vals else None


def classify_gas(response_samples: list):
    """response_samples: ZONE_STOP 동안 모은 아두이노 샘플 dict 리스트.
    MQ2(좌/우) 평균으로 위험도(strength/alert_level)를, MQ3/135/138(+baseline
    버퍼)로 가스 종류(gas_type, 룰 기반)를 판별한다.
    -> (gas_type, confidence, strength, alert_level)"""
    mq2_left = _avg_key(response_samples, "mq2_left")
    mq2_right = _avg_key(response_samples, "mq2_right")
    mq2_vals = [v for v in (mq2_left, mq2_right) if v is not None]
    strength = round(max(mq2_vals) / 1023.0 * 100.0, 1) if mq2_vals else 0.0
    if strength >= GAS_ALERT_DANGER_THRESHOLD:
        alert_level = "위험"
    elif strength >= GAS_ALERT_WARN_THRESHOLD:
        alert_level = "주의"
    else:
        alert_level = "정상"

    gas_type, confidence = "미분류", 0.0
    with gas_lock:
        baseline_snapshot = list(gas_baseline_buffer)
    if RULE_DB.get("rules") and len(baseline_snapshot) >= 2 and len(response_samples) >= 2:
        try:
            base_arr = [[s["mq3"], s["mq135"], s["mq138"]] for s in baseline_snapshot]
            resp_arr = [[s["mq3"], s["mq135"], s["mq138"]] for s in response_samples]
            features = extract_rule_features(base_arr, resp_arr)
            result, scores = rule_classify(features, RULE_DB["rules"])
            gas_type = result
            if result != "미분류":
                confidence = round(max(0.0, 1.0 - scores[result] / 2.0), 3)
        except Exception as e:
            print(f"[GAS] 룰 기반 분류 실패: {e}")

    return gas_type, confidence, strength, alert_level


def publish_robot_event(location, event_type, sensor_strengths, gas_result=None):
    """robot/event 토픽으로 서버(safeDB)가 기대하는 스키마 그대로 발행한다."""
    payload = {"location": location, "event_type": event_type, **sensor_strengths}
    if gas_result is not None:
        gas_type, confidence, strength, alert_level = gas_result
        payload.update({
            "gas_type": gas_type, "confidence": confidence,
            "strength": strength, "alert_level": alert_level,
        })
    mqtt_client.publish(TOPIC_ROBOT_EVENT, json.dumps(payload))
    print(f"[MQTT OUT] {TOPIC_ROBOT_EVENT} = {payload}")


# ── 상태 (여러 스레드에서 공유) ──────────────────────────
class RobotNav:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = "INIT"          # INIT / IDLE / NAVIGATE / ZONE_ENTER / ZONE_STOP /
                                    # RETURN / EXIT_HALT / UTURN_SEEK /
                                    # DEADEND_HALT / DEADEND_UTURN
        self.current_node = None
        self.current_heading = None
        self.gas_target = None      # MQTT 로 받은 목표 (구역 코드, 예: "A2")
        self.node_route = []        # 지금 실행 중인 경로 (노드 이름 리스트)
        self.leg_headings = []
        self.current_leg = 0
        self.node_idx = 0
        self.route_coords = []
        self.route_axes = []   # route_coords 와 병렬 -- 'row' 축만 볼지 'col' 축만 볼지
        self.route_wp_idx = 0
        self.manual_override = None # "return" / "stop" / ("goto", node)
        # 구역 진입 전용 상태
        self.zone_code = None
        self.zone_entry_node = None   # 구역 진입 시 실제로 어느 끝점에서 들어갈지
        self.zone_stop_coord = None
        self.zone_heading = None
        self.zone_stop_started = None
        self.zone_stop_halt_sent = False
        self.zone_enter_started = 0.0   # 8/20: ZONE_ENTER 진입 시각 (타임아웃용)
        self.esp32_line_lost = False    # 8/20: ESP32가 라인을 놓쳤다고 보고했는가
        # 8/21 추가 -- ESP32가 라파 UART 스트림 끊김으로 스스로 정지했는가.
        self.esp32_link_lost = False
        self._off_route_hits = 0        # 8/20: 경로 밖 간선 QR 연속 확인 횟수
        self._off_route_prev = None     # 8/20: 직전에 읽은 다른 QR (방향 판정용)
        # ESP32가 실제로 물리적 교차로(111)를 감지해서 "intersection" 마크를
        # 보낸 횟수. TVEC 계산은 이 값이 QR 위치 추정(node_idx)을 따라잡을
        # 때까지 기다린다. 카메라가 실제 교차로 도달보다 먼저 "도착"으로
        # 판단해서 회전각이 다음 구간용으로 미리 덮어써지는 경합을 막는다.
        self.esp32_crossing_count = 0
        self.zone_entry_tvec = 0.0
        self.zone_entry_crossing_target = 0
        self._zone_entry_wait_since = 0.0
        self._last_reroute_time = 0.0
        self._node_idx_stuck_since = 0.0
        self.last_qr_coords = None
        self._leg_lag_since = 0.0
        # 8/21 추가 -- NEAR 래치. 어느 구간(effective_leg)에서 창이 열렸는지
        # 기억한다. 노드 옆 QR 을 한 번만 읽고 그 뒤로 모션블러 등으로 계속
        # 놓쳐도 창이 닫히지 않게 하기 위함이다. 반경을 1.6 으로 좁힌 이상,
        # 그 QR 하나를 놓치면 ESP32 가 진짜 교차로까지 기각해 회전이 통째로
        # 누락된다(실측에서 TL->A 구간의 열 4·5 QR 을 연속으로 놓친 전례).
        # 구간이 바뀌면(= ESP32 가 그 교차로를 실제로 통과하면) 자동으로 풀린다.
        self._near_latch_leg = None
        # 8/20 추가 -- 가상 출발 노드(_ZMID)의 실제 좌표.
        # 구역 정지 후 복귀 경로에서만 채워지고, 그 외에는 None.
        self.virtual_start_coord = None
        # 8/20 추가 -- U턴 시퀀스 전용 상태
        self.exit_halt_started = 0.0    # EXIT_HALT / DEADEND_HALT 진입 시각
        self.uturn_started = 0.0        # UTURN_SEEK / DEADEND_UTURN 진입 시각
        self.uturn_exit_node = None     # U턴하는 지점의 노드 이름
        self.esp32_uturn_done = False   # ESP32가 uturn_done 마크를 보냈는가
        # 8/21 추가 -- U턴 완료 후 QR 탐색을 시작한 시각.
        # 탐색 타임아웃을 CMD:u 시점이 아니라 이 시점부터 재기 위함이다.
        self.uturn_seek_since = 0.0
        # 8/21 추가 -- U턴 피벗 직후의 정렬(TURN_ALIGN) 완료 여부.
        # uturn_done 만 기다리면, 아직 좌우로 훑으며 정렬 중일 때
        # 카메라가 QR 을 잡아 그 자리에서 정지시켜 버린다. 라인 위에
        # 제대로 올라앉기 전에 멈추면 다음 임무 출발이 흔들린다.
        self.esp32_align_done = False

nav = RobotNav()

esp32_serial = None


# ── 좌표/방위 계산 유틸 ───────────────────────────────
def parse_qr_payload(payload: str):
    """
    QR 안의 좌표 문자열을 파싱한다.
    실제 QR 형식은 '(11,0)' -- 괄호로 감싸고 쉼표로 행/열을 구분한다.
    괄호 없는 형태('11,0')나 마침표 구분('11.0')도 혹시 몰라 같이 대비한다.
    좌표는 항상 정수(행 0~11, 열 0~12)이다.
    """
    payload = payload.strip()
    m = re.match(r"^\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\)$", payload)
    if m:
        return (float(m.group(1)), float(m.group(2)))
    m = re.match(r"^(-?\d+)\s*,\s*(-?\d+)$", payload)
    if m:
        return (float(m.group(1)), float(m.group(2)))
    m = re.match(r"^(-?\d+)\.(-?\d+)$", payload)
    if m:
        return (float(m.group(1)), float(m.group(2)))
    return None


def normalize_angle(deg):
    while deg > 180: deg -= 360
    while deg < -180: deg += 360
    return deg


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# 8/20 추가 -- 가상 노드(_ZMID)까지 포함해서 좌표/축을 조회하는 래퍼.
# map_data 의 NODES / edge_axis 를 직접 쓰면 _ZMID 에서 KeyError 가 난다.
def node_coord(name):
    if name == VIRTUAL_START:
        return nav.virtual_start_coord
    return NODES[name]


def axis_between(a, b):
    """edge_axis 와 같지만 가상 노드도 처리한다."""
    if a != VIRTUAL_START and b != VIRTUAL_START:
        return edge_axis(a, b)
    ca, cb = node_coord(a), node_coord(b)
    if ca is None or cb is None:
        return None
    return "row" if abs(ca[0] - cb[0]) > abs(ca[1] - cb[1]) else "col"


def nearest_init_node(coords, max_dist):
    """
    INIT 전용 -- 막다른 모서리(TL/TR/BL/BR/ML/MR) 중에서만 가장 가까운
    노드를 찾는다. 교차점은 출발 지점이 될 수 없으므로 후보에서 뺀다.
    max_dist 이내가 없으면 None.
    """
    best, best_d = None, max_dist
    for name in NODES:
        if not is_dead_end(name):
            continue
        d = distance(coords, NODES[name])
        if d < best_d:
            best, best_d = name, d
    return best


def axis_distance(observed, target, axis):
    """
    카메라가 로봇 오른쪽에 고정되어 있어, 진행 방향에 따라 보이는 벽
    (좌/우 어느 쪽 QR)이 달라진다. 로봇은 항상 그 QR의 복도 중심 쪽에
    있으므로, 진행 방향 축만 비교하면 어느 벽을 보든 맞는다.
    axis='row' 면 행만, 'col' 이면 열만 비교. None 이면 2D 유클리드 거리.
    """
    if axis == "row":
        return abs(observed[0] - target[0])
    elif axis == "col":
        return abs(observed[1] - target[1])
    return distance(observed, target)


# 축 매칭의 맹점 보강. 진행축만 보면 같은 열(또는 행)을 공유하는 "전혀 다른
# 노드"(예: TL->A 구간에서 A, C, E 가 모두 열4)와 헷갈릴 수 있다. 정상 주행
# 중엔 물리적으로 그 QR을 읽을 수 없어 문제없지만, 경로 이탈 등으로 이미
# 잘못된 상태라면 오판을 증폭시킨다. 그래서 반대축(벽 쪽)이 벽 간격 수준을
# 크게 벗어나면 매칭을 거부한다.
# 8/20 재조정 -- 3.0 -> 2.0. 실패 케이스가 정확히 3.0 이었고 비교가
# "초과"(>)라 경계값이 그대로 통과했다. F->D 구간에서 C-E 복도의 (8,5)를
# 읽었는데 진행축 2.5 <= 2.6, 반대축 3.0 > 3.0 이 False 가 되어
# 'D 도착'으로 인정됐다. 정상 QR 의 반대축 거리는 최대 1.0 이라 2.0 이면 안전.
OFF_AXIS_SANITY_BOUND = 2.0


def axis_distance_sane(observed, target, axis):
    """반대축이 다른 노드 수준으로 벗어나 있으면 무한대를 반환해 매칭 실패 처리."""
    if axis == "row":
        off_axis = abs(observed[1] - target[1])
    elif axis == "col":
        off_axis = abs(observed[0] - target[0])
    else:
        return distance(observed, target)
    if off_axis > OFF_AXIS_SANITY_BOUND:
        return float("inf")
    return axis_distance(observed, target, axis)


# "지금 진짜 다음 교차로 근처다"를 ESP32에 알려주기 위한 판정.
# 8/21 -- ESP32(main.cpp)가 이제 이 값으로 "모든" 교차로를 게이팅한다.
# 회전뿐 아니라 직진 교차로도 NEAR:0 이면 기각된다. 가짜 교차로 하나가
# esp32_crossing_count 를 1 올리면 그 뒤 회전각이 통째로 한 칸씩 밀리기
# 때문이다. 즉 이 플래그의 정확도가 곧 주행 성패다.
#
# 8/21 재조정 -- 2.8 -> 1.6.
# 2.8 은 노드에서 2칸(약 65cm) 떨어진 QR 까지 창을 열었다. 실측(TL->A->B->D)
# 에서 B(열8) 를 향해 가다 QR (1,6) 을 읽는 순간 NEAR:1 이 열렸고, 0.2초 뒤
# 복도 한가운데의 가짜 111 이 그 창으로 들어와 그 자리에서 우회전했다.
# QR 간격은 가로 복도 1.0, 세로 복도 1.5(노드가 0.5 오프셋이라)이므로
# 1.6 이면 노드 바로 옆 QR 하나만 창을 연다.
#   TL-A : (0,3)/(1,3) -> A(0.5,4) 까지 열 거리 1.0
#   A-B  : (0,7)/(1,7) -> B(0.5,8) 까지 열 거리 1.0
#   B-D  : (2..4,7/9)  -> D(5.5,8) 까지 행 거리 1.5
NEAR_NODE_RADIUS = 1.6


def compute_near_flag(last_coords, node_route, leg_idx):
    """
    최근 QR 좌표가 다음 노드에 충분히 가까우면 True.
    8/20 수정 -- 기존에는 node_idx(카메라 추정)를 썼는데, 카메라가 ESP32의
    물리적 통과보다 앞서면 이미 그 다음 노드까지의 거리를 재게 되어
    "정작 눈앞의 교차로에서 NEAR:0" 이 되는 역전이 있었다. TVEC 과 동일하게
    effective_leg(= ESP32 진행도에 맞춘 값)를 기준으로 계산한다.

    8/21 추가 -- 두 가지 보강.
      1) 반대축 검사(axis_distance_sane). 반경을 1.6 으로 좁히면 "같은 열을
         공유하는 다른 복도의 QR"이 우연히 맞을 확률이 상대적으로 커진다.
         노드 도착 판정과 같은 기준으로 걸러낸다.
      2) 지나침(overshoot) 인정. 노드를 이미 지나친 좌표를 읽었다면 교차로는
         바로 뒤에 있거나 방금 지나간 것이므로 창을 연다. 늦게라도 여는 편이
         영영 안 여는 것보다 낫다 -- 창이 안 열리면 ESP32 가 진짜 교차로까지
         기각해 카운터가 영영 안 올라간다.
    """
    if last_coords is None:
        return False
    if leg_idx + 1 >= len(node_route):
        return False
    cur_node = node_route[leg_idx]
    next_node = node_route[leg_idx + 1]
    axis = axis_between(cur_node, next_node)
    target = node_coord(next_node)
    cur = node_coord(cur_node)
    if target is None:
        return False

    if axis_distance_sane(last_coords, target, axis) <= NEAR_NODE_RADIUS:
        return True

    # 지나침 판정 -- 진행 방향으로 목표 노드를 넘어섰는가.
    if cur is not None and axis in ("row", "col"):
        i = 0 if axis == "row" else 1
        direction = target[i] - cur[i]
        if direction != 0:
            past = (last_coords[i] - target[i]) * (1 if direction > 0 else -1)
            # 반대축이 크게 벗어나 있으면 다른 복도의 QR 이므로 인정하지 않는다.
            if past > 0 and axis_distance_sane(last_coords, target, axis) != float("inf"):
                return True
    return False


# 8/20 추가 -- ESP32의 decideTurnFromVector() 임계값(30도)과 맞춘다.
# 이보다 크면 "실제 회전이 걸려 있다"고 보고 랙 타임아웃을 유예한다.
TURN_PENDING_DEG = 30.0


def _tvec_at(legs, i):
    """i번째 구간에 서 있을 때 다음 교차로에서 필요한 회전각."""
    if i >= len(legs):
        return 0.0
    if i + 1 >= len(legs):
        return 0.0
    return normalize_angle(legs[i + 1] - legs[i])


def heading_between(from_rc, to_rc):
    d_row = to_rc[0] - from_rc[0]
    d_col = to_rc[1] - from_rc[1]
    return math.degrees(math.atan2(d_row, d_col))


def build_leg_headings(node_route):
    headings = []
    for i in range(len(node_route) - 1):
        a = NODES[node_route[i]]
        b = NODES[node_route[i + 1]]
        headings.append(heading_between(a, b))
    return headings


# 8/20 추가 -- OpenCV QR 디코더는 후보 영역의 꼭짓점 4개가 일직선이거나
# 겹치면 contourArea == 0 으로 예외를 던진다(OpenCV 5.0.0 qrcode.cpp).
# 흐릿한 프레임이나 QR을 비스듬히 스칠 때 간헐적으로 발생하며, 특히
# 반전 이미지 경로에서 벽면 노이즈가 QR 후보로 잡히면 더 잘 난다.
# 프레임 하나를 못 읽는 건 이미 정상 상황(payload='')으로 처리하고 있으므로
# 예외도 같은 취급을 한다. 이걸 안 잡으면 주행 중 프로세스가 죽는다.
def _safe_decode(qr_detector, img):
    try:
        payload, points, _ = qr_detector.detectAndDecode(img)
        return payload, points
    except cv2.error:
        return "", None


def _safe_decode_multi(qr_detector, img):
    try:
        ok, payloads, points, _ = qr_detector.detectAndDecodeMulti(img)
        return ok, payloads, points
    except cv2.error:
        return False, [], None


def detect_qr(frame, qr_detector):
    """단일 QR, 검정배경/흰마크 반전 대응."""
    payload, points = _safe_decode(qr_detector, frame)
    if not payload or points is None:
        inverted = cv2.bitwise_not(frame)
        payload2, points2 = _safe_decode(qr_detector, inverted)
        if payload2 or points2 is not None:
            payload, points = payload2, points2
    return payload, points


def detect_qr_multi(frame, qr_detector):
    """모서리 QR 2개 동시 인식 시도 (원본+반전 둘 다)."""
    results = []
    for src in (frame, cv2.bitwise_not(frame)):
        ok, payloads, points = _safe_decode_multi(qr_detector, src)
        if ok and points is not None:
            for p, pl in zip(points, payloads):
                if pl:
                    results.append((pl, p))
    return results


# ── 디버그용 실시간 영상 스트리밍 (HTTP, MJPEG) ──────────
class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def set_frame(self, jpg_bytes):
        with self.condition:
            self.frame = jpg_bytes
            self.condition.notify_all()


stream_output = StreamingOutput()

STREAM_PAGE = """\
<html>
<head><title>라파 카메라 - QR 인식 확인</title></head>
<body style="background:#111; text-align:center;">
<h2 style="color:white;">카메라 화면 + QR 인식 결과</h2>
<img src="stream.mjpg" style="max-width:90%; border:2px solid #444;" />
</body>
</html>
"""


class StreamingHandler(http_server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            content = STREAM_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with stream_output.condition:
                        stream_output.condition.wait()
                        frame = stream_output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, http_server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def start_stream_server():
    srv = StreamingServer(("", STREAM_PORT), StreamingHandler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    print(f"영상 스트리밍 시작: http://<라파IP>:{STREAM_PORT}/")


def draw_qr_overlay(frame_bgr, multi_results, single_result):
    """
    이번 프레임에서 인식된 QR을 화면에 그린다.
    multi_results: [(payload, points4, inverted_bool), ...]
    single_result: (payload, points4) 또는 None
    """
    if multi_results:
        colors = [(0, 0, 255), (255, 120, 0), (0, 255, 255), (0, 255, 0)]
        for idx, (pl, pts, inv) in enumerate(multi_results):
            pts_i = pts.astype(int)
            color = colors[idx % len(colors)]
            for i in range(len(pts_i)):
                p1, p2 = tuple(pts_i[i]), tuple(pts_i[(i + 1) % len(pts_i)])
                cv2.line(frame_bgr, p1, p2, color, 3)
            tag = " [반전]" if inv else ""
            cv2.putText(frame_bgr, f"{pl}{tag}", tuple(pts_i[0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        cv2.putText(frame_bgr, f"동시 인식 {len(multi_results)}개", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
    elif single_result and single_result[1] is not None:
        payload, points = single_result
        pts = points[0].astype(int) if points is not None and len(points) > 0 else None
        if pts is not None:
            for i in range(len(pts)):
                p1, p2 = tuple(pts[i]), tuple(pts[(i + 1) % len(pts)])
                cv2.line(frame_bgr, p1, p2, (0, 0, 255), 3)
            text = payload if payload else "(디코딩 실패)"
            cv2.putText(frame_bgr, text, (int(pts[0][0]), int(pts[2][1]) + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(frame_bgr, f"QR: {text}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame_bgr, "QR 없음", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

    with nav.lock:
        status_line = f"mode={nav.mode} node={nav.current_node} target={nav.gas_target}"
    cv2.putText(frame_bgr, status_line, (10, 460),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

    ok, jpg = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if ok:
        stream_output.set_frame(jpg.tobytes())


# ── MQTT ──────────────────────────────────────────────
mqtt_client = mqtt.Client(client_id=f"rpi-nav{ROBOT_ID}", clean_session=True)


def publish_mark(text):
    mqtt_client.publish(TOPIC_RPI_MARK, text)
    print(f"[MARK] {text}")


def publish_state():
    with nav.lock:
        payload = {
            "mode": nav.mode,
            "node": nav.current_node,
            "heading": nav.current_heading,
            "target": nav.gas_target,
            "route": nav.node_route,
        }
    mqtt_client.publish(TOPIC_RPI_STATE, json.dumps(payload))


def on_mqtt_message(client, userdata, msg):
    payload = msg.payload.decode().strip()
    print(f"[MQTT IN] {msg.topic} = {payload}")

    if msg.topic == TOPIC_GAS_TARGET:
        zone_code = payload.strip().upper()
        try:
            zone_info(zone_code)   # 유효한 구역인지만 미리 검증
            with nav.lock:
                nav.gas_target = zone_code
                still_init = nav.mode == "INIT"
            publish_mark(f"gas_target 수신: 구역 {zone_code}")
            # 8/20 추가 -- 타겟이 들어왔는데 아직 INIT(위치 미확정)이면, 사람이
            # 수동으로 CMD:g를 보내줄 때까지 기다리지 않고 그 즉시 INIT을
            # 밀어붙인다. 이후 정지선(111)을 만나도 QR을 읽을 때까지 계속
            # 전진하는 건 esp32_uart_reader의 "arrive" 처리(위)가 이어서 맡는다.
            # 즉 타겟 도착 시 가장 먼저 실행돼야 하는 로직이 이것 -- INIT부터
            # 끝내고, 그다음에야 IDLE 루프의 실제 출동(dispatch) 로직이 돈다.
            if still_init and esp32_serial is not None:
                esp32_serial.write(b"CMD:g\n")
                publish_mark("gas_target 수신 -- INIT 강제 진행(CMD:g)")
        except ValueError:
            publish_mark(f"gas_target 무시 -- 알 수 없는 구역: {zone_code}")

    elif msg.topic == TOPIC_ESP32_CMD:
        # ESP32 몫의 명령. ESP32는 MQTT를 직접 안 받으므로 라파가 UART 릴레이.
        # 8/20: U턴(u), CPR 실측(c)도 릴레이 대상에 추가.
        if payload in ("s", "g", "u", "c") and esp32_serial is not None:
            esp32_serial.write(f"CMD:{payload}\n".encode("utf-8"))
            print(f"  -> ESP32로 릴레이: CMD:{payload}")

    elif msg.topic == TOPIC_RPI_CMD:
        if payload == "return":
            with nav.lock:
                nav.manual_override = "return"
            publish_mark("수동 명령: 즉시 복귀")
        elif payload == "stop":
            with nav.lock:
                nav.manual_override = "stop"
            publish_mark("수동 명령: 정지")
        elif payload.startswith("goto:"):
            target = payload.split(":", 1)[1].strip()
            if target in NODES:
                with nav.lock:
                    nav.manual_override = ("goto", target)
                publish_mark(f"수동 명령: {target} 로 강제 이동")
            else:
                publish_mark(f"goto 무시 -- 알 수 없는 노드: {target}")


def connect_mqtt():
    mqtt_client.on_message = on_mqtt_message
    mqtt_client.will_set(TOPIC_RPI_ONLINE, "0", retain=True)
    mqtt_client.connect(MQTT_HOST, MQTT_PORT, 60)
    mqtt_client.subscribe(TOPIC_GAS_TARGET)
    mqtt_client.subscribe(TOPIC_RPI_CMD)
    mqtt_client.subscribe(TOPIC_ESP32_CMD)
    mqtt_client.publish(TOPIC_RPI_ONLINE, "1", retain=True)
    mqtt_client.loop_start()


def esp32_uart_reader(ser):
    """ESP32 -> 라파 UART 리더. STATE/MARK 를 받아 MQTT로 대리 발행."""
    buf = ""
    esp32_seen_once = False
    while True:
        try:
            n = ser.in_waiting
        except Exception:
            time.sleep(0.05)
            continue
        if n:
            try:
                chunk = ser.read(n).decode("utf-8", errors="replace")
            except Exception:
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                if not esp32_seen_once:
                    esp32_seen_once = True
                    mqtt_client.publish(TOPIC_ESP32_ONLINE, "1", retain=True)

                if line.startswith("STATE:"):
                    mqtt_client.publish(TOPIC_ESP32_STATE, line[len("STATE:"):])
                elif line.startswith("MARK:"):
                    mark_text = line[len("MARK:"):]
                    mqtt_client.publish(TOPIC_ESP32_MARK, mark_text)
                    if mark_text.startswith("intersection"):
                        # ESP32가 실제로 물리적 교차로를 감지한 순간.
                        with nav.lock:
                            nav.esp32_crossing_count += 1
                    elif mark_text.startswith("rpi_link_lost"):
                        # 8/21: ESP32 가 "라파 스트림이 2초 끊겼다"고 판단해
                        # 스스로 정지했다. 이걸 라파가 모르면 멈춘 로봇에
                        # 계속 POS 만 보내며 조용히 무한 대기한다
                        # (실측: IDLE 대기 중 들어온 C3 출동이 이 경로로 막혔다).
                        with nav.lock:
                            nav.esp32_link_lost = True
                    elif mark_text.startswith("line_lost"):
                        # 8/20: ESP32가 라인을 놓쳐 스스로 정지했다. 이걸
                        # 라파가 모르면 멈춘 로봇에 계속 POS 만 보내며
                        # 끝없이 대기한다.
                        with nav.lock:
                            nav.esp32_line_lost = True
                    elif mark_text.startswith("align_done") or \
                            mark_text.startswith("align_giveup"):
                        # 8/21: 회전 마무리 정렬이 끝났다(포기 포함).
                        # UTURN_SEEK 가 QR 탐색을 시작해도 되는 시점을 판단한다.
                        with nav.lock:
                            nav.esp32_align_done = True
                    elif mark_text.startswith("uturn_done"):
                        # 8/20: 제자리 U턴 완료(타임아웃 포함). ML/MR 같은
                        # 막다른 지점에서 U턴 후 주행을 이어가려면 이 신호가
                        # 필요하다.
                        with nav.lock:
                            nav.esp32_uturn_done = True
                    elif mark_text == "arrive":
                        # 8/20 추가 -- ESP32는 Pi의 모드를 모르고 정지선을
                        # 만나면 무조건 PH_STOP으로 선다. INIT 단계(아직 QR로
                        # 위치를 확정 못한 상태)에서 이러면 사람이 매번 CMD:g를
                        # 다시 보내줘야만 계속 전진했다 -- 팀 요청대로, QR을
                        # 아직 못 읽었으면 정지선에서 자동으로 다시 출발시켜서
                        # QR을 읽을 때까지 계속 전진하게 한다.
                        with nav.lock:
                            still_init = nav.mode == "INIT"
                        if still_init:
                            ser.write(b"CMD:g\n")
        else:
            time.sleep(0.02)


# ── 메인 내비게이션 로직 ──────────────────────────────
def main():
    global esp32_serial
    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.1)
        esp32_serial = ser
        print(f"UART 연결됨: {UART_PORT} @ {UART_BAUD}bps")
    except serial.SerialException as e:
        print(f"UART 포트 열기 실패: {e}")
        sys.exit(1)

    connect_mqtt()
    print("MQTT 연결됨")

    # 8/21 통합 -- 가스 센서 5개는 전부 이 아두이노(USB 시리얼) 하나에서 온다.
    # 하드웨어 미장착이어도 죽지 않고 '미분류' / 세기 0 으로 계속 진행한다.
    global gas_arduino_serial
    try:
        gas_port = resolve_port(GAS_ARDUINO_PORT)
        gas_arduino_serial = serial.Serial(gas_port, GAS_ARDUINO_BAUD, timeout=1.0)
        time.sleep(2)  # 아두이노 리셋 대기
        gas_thread = threading.Thread(
            target=gas_arduino_reader, args=(gas_arduino_serial,), daemon=True)
        gas_thread.start()
        print(f"가스 판별 아두이노 연결됨: {gas_port} @ {GAS_ARDUINO_BAUD}bps")
    except Exception as e:
        print(f"[GAS] 가스 판별 아두이노 연결 실패({e}) -- 가스 종류는 계속 '미분류'로만 진행")
        print(port_help_message())

    reader_thread = threading.Thread(target=esp32_uart_reader, args=(ser,), daemon=True)
    reader_thread.start()
    print("ESP32 UART 리더 스레드 시작 (STATE/MARK -> MQTT 대리 발행)")

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        transform=Transform(hflip=True, vflip=True),   # 카메라가 180도 뒤집혀 장착됨
    ))
    picam2.start()
    time.sleep(1.0)

    # 주행 중 QR이 모션 블러로 인식 안 되는 문제 대응.
    EXPOSURE_TIME_US = 3000
    ANALOGUE_GAIN = 6.0
    try:
        picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": EXPOSURE_TIME_US,
            "AnalogueGain": ANALOGUE_GAIN,
        })
        print(f"카메라 연결됨 -- 수동 노출 고정 (ExposureTime={EXPOSURE_TIME_US}us, Gain={ANALOGUE_GAIN})\n")
    except Exception as e:
        print(f"카메라 연결됨 -- 수동 노출 설정 실패({e}), 자동노출로 계속 진행\n")

    if STREAM_ENABLED:
        start_stream_server()

    qr_detector = cv2.QRCodeDetector()
    last_send_time = 0
    last_state_publish = 0
    last_init_diag = 0
    last_nav_qr_diag = 0
    # 8/20 추가 -- 주행 중 로봇 위치를 robot/event로 주기 발행(웹 대시보드
    # 로봇 마커용). 프론트(useLiveData.parseQrLocation)가 "(x,y)" 형식의
    # location만 좌표로 해석하므로 그 형식 그대로 보낸다.
    last_pos_publish_time = 0
    POS_PUBLISH_INTERVAL_SEC = 1.0
    POS_BROADCAST_MODES = {"NAVIGATE", "RETURN", "ZONE_ENTER"}

    print("=== INIT: 로봇을 출발 모서리에 '맵 안쪽'을 향해 놓고 CMD:g 로 출발시키세요.")
    print("          전진하다 QR 1개를 읽으면 위치를 확정하고 자동으로 정지합니다. ===")

    try:
        while True:
            frame = picam2.capture_array()
            now = time.time()

            # 8/20 추가 -- 주행 중(NAVIGATE/RETURN/ZONE_ENTER)에는 최근 QR
            # 좌표를 1초 간격으로 robot/event에 실어보내 대시보드 로봇 마커를
            # 움직인다. 센서/판별값은 안 실음(그건 ZONE_STOP에서 따로 발행).
            with nav.lock:
                _mode_now = nav.mode
                _last_coords = nav.last_qr_coords
                _zone_code_now = nav.zone_code
            if _mode_now in POS_BROADCAST_MODES and _last_coords is not None \
                    and now - last_pos_publish_time >= POS_PUBLISH_INTERVAL_SEC:
                last_pos_publish_time = now
                # 8/20 수정 -- 지금 이 로봇은 자율 순찰 루프가 따로 없고 오직
                # gas_target 출동으로만 움직인다. nav.zone_code가 채워져 있으면
                # (구역 판별하러 가는/복귀하는 중) 그 이동 전체가 가스 대응이므로
                # event_type도 그렇게 남긴다. zone_code가 없는 건 수동 goto 등
                # 진짜 순찰성 이동뿐이라 patrol 그대로 둔다.
                _event_type = "gas_response" if _zone_code_now else "patrol"
                publish_robot_event(f"({_last_coords[0]:.0f},{_last_coords[1]:.0f})", _event_type, {})

            # ── 8/21 이동 -- ESP32 정지 감시를 모드 분기보다 "앞"으로 ──
            # 원래 이 감시는 IDLE 블록 아래에 있었다. 그런데 INIT 과 IDLE 은
            # 그 앞에서 continue 로 빠져나가므로 감시를 통과하지 않았다.
            # 그 결과 INIT 중에 ESP32 가 라인을 놓쳐 정지해도 라파는 그것을
            # 모른 채 "INIT 전진 중 -- QR 대기" 만 영원히 찍었다.
            # (실측: 직전 주행이 라인 밖 000 에서 끝난 뒤 재시작했더니
            #  gas_target 을 받고 CMD:g 를 보냈는데도 QR 이 끝내 안 읽혔다)
            # 이제 모든 모드에서 감시한다.
            with nav.lock:
                lost = nav.esp32_line_lost
                if lost:
                    nav.esp32_line_lost = False
                link_lost = nav.esp32_link_lost
                if link_lost:
                    nav.esp32_link_lost = False
                lost_mode = nav.mode

            if lost:
                if lost_mode == "INIT":
                    # INIT 은 위치 미확정 상태라 IDLE 로 떨어뜨려도 의미가 없다.
                    # 사람이 로봇을 라인 위에 다시 놓고 시작해야 한다.
                    publish_mark("[라인 놓침] INIT 중 ESP32가 라인을 잃고 정지 -- "
                                 "로봇이 라인 위에 제대로 올라가 있는지 확인하고 "
                                 "다시 출발시킬 것 (CMD:g)")
                elif lost_mode in ("NAVIGATE", "RETURN", "ZONE_ENTER"):
                    publish_mark(f"[라인 놓침] ESP32가 {lost_mode} 중 라인을 잃고 정지 -- "
                                 f"대기 상태로 전환, 수동 개입 필요")
                    with nav.lock:
                        nav.mode = "IDLE"
                        nav.zone_code = None
                    continue

            # ── ESP32 UART 스트림 끊김 감시 ──
            # ESP32는 라파 POS 스트림이 2초 끊기면 rpi_link_lost 로 스스로
            # 정지한다. 그동안 라파는 그 마크를 처리하지 않아, 멈춘 로봇에
            # 계속 POS 만 보내며 아무 표시 없이 무한 대기했다.
            # 여기서 CMD:g 로 자동 재개하지 않는 이유 -- 스트림이 끊겼다는 건
            # 통신 계통에 문제가 생겼다는 뜻이고, 원인을 모른 채 다시
            # 출발시키면 그다음엔 제어 없이 굴러갈 수 있다.
            if link_lost:
                publish_mark(f"[스트림 끊김] ESP32가 라파 UART 스트림 2초 단절로 "
                             f"자체 정지 ({lost_mode} 중) -- 대기 상태로 전환. "
                             f"UART 배선과 라파 부하 확인 필요")
                if lost_mode != "INIT":
                    with nav.lock:
                        nav.mode = "IDLE"
                        nav.zone_code = None
                    continue

            # ═══════════════════ INIT: 전진하며 QR 1개로 위치 확정 ═══════════════════
            # 8/20 전면 교체 -- 기존에는 출발 모서리에서 QR 2개를 동시에 인식해
            # 위치와 방위를 함께 구했다. 그런데 그렇게 구한 방위(current_heading)는
            # 어디에도 쓰이지 않는다. 주행 회전각은 전부 지도의 절대 방위 차이로
            # 계산하므로 초기 방위와 무관하다. 실제로 같은 자리에서 시작해도
            # QR 인식 순서에 따라 0도/180도가 번갈아 나왔는데 주행에는 아무
            # 영향이 없었다 -- 그 자체가 안 쓰인다는 증거다.
            #
            # 이제는 로봇을 출발 모서리에 맵 안쪽을 향해 놓고 시작한다.
            # 전진하다 QR 1개를 읽으면 그 좌표로 위치를 확정하고 정지한다.
            # 방위는 "맵 안쪽"으로 이미 정해져 있으므로 따로 구할 필요가 없다.
            if nav.mode == "INIT":
                payload, points = detect_qr(frame, qr_detector)
                coords = parse_qr_payload(payload) if payload else None

                if STREAM_ENABLED:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    draw_qr_overlay(frame_bgr, None, (payload, points))

                with nav.lock:
                    override = nav.manual_override
                    nav.manual_override = None
                if override == "stop":
                    ser.write(b"CMD:s\n")
                    publish_mark("INIT 중 정지 명령 -- 대기")
                    time.sleep(0.05)
                    continue

                if now - last_init_diag > 5.0:
                    last_init_diag = now
                    msg = (f"INIT 전진 중 -- QR 대기 (payload={payload!r}). "
                           f"출발 모서리에서 맵 안쪽을 향해 놓고 g 명령으로 출발시킬 것")
                    print(msg)
                    publish_mark(msg)

                if coords is not None:
                    print(f"[QR] 읽음: {coords}")
                    # 출발 지점은 반드시 막다른 모서리다. 교차점(A~F)으로는
                    # 확정하지 않는다 -- nearest_node_to_coord 는 진행 방향을
                    # 모르므로, QR 을 늦게 읽어 복도 중간에서 잡히면 아직
                    # 지나지도 않은 반대편 노드로 오판한다.
                    # (예: BL 출발인데 (11,3)을 읽으면 E 로 확정 -> 경로 전체가 어긋남)
                    found_node = nearest_init_node(coords, INIT_NODE_MAX_DIST)
                    if found_node is None:
                        # 노드에서 너무 먼 QR -- 이미 출발 노드를 지나쳤을 수 있다.
                        # 여기서 위치를 확정하면 경로가 어긋나므로 세우고 알린다.
                        ser.write(b"CMD:s\n")
                        publish_mark(f"[INIT 실패] QR {coords} 가 어느 노드에서도 "
                                     f"{INIT_NODE_MAX_DIST} 이내가 아님 -- 출발 노드를 지나쳤을 수 "
                                     f"있음. 로봇을 모서리로 되돌리고 다시 시작할 것")
                    else:
                        ser.write(b"CMD:s\n")
                        with nav.lock:
                            nav.current_node = found_node
                            nav.current_heading = None   # 쓰이지 않음
                            nav.mode = "IDLE"
                        publish_mark(f"INIT 완료: QR {coords} -> 위치 {found_node} -- 정지, 목표 대기")
                        print(f"=== INIT 완료: {found_node} -> IDLE ===")

                # ESP32 스트림 타임아웃(2초)에 걸리지 않게 계속 보낸다.
                # TVEC 0 이라 교차로를 만나도 직진 통과한다.
                if now - last_send_time >= SEND_INTERVAL_SEC:
                    last_send_time = now
                    ser.write(b"POS:INIT,TVEC:0.0,NEAR:0\n")

                if now - last_state_publish > 0.5:
                    last_state_publish = now
                    publish_state()
                time.sleep(0.03)
                continue

            # ═══════════════════ IDLE: 목표 대기 ═══════════════════
            if nav.mode == "IDLE":
                if STREAM_ENABLED:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    draw_qr_overlay(frame_bgr, None, None)
                with nav.lock:
                    override = nav.manual_override
                    nav.manual_override = None
                    target = nav.gas_target
                    cur = nav.current_node

                if override == "stop":
                    pass
                elif isinstance(override, tuple) and override[0] == "goto":
                    dest = override[1]
                    path = find_path(cur, dest)
                    if path:
                        _start_route(path)
                        publish_mark(f"강제 이동 시작: {cur} -> {dest}")
                elif target is not None:
                    # target 은 구역 코드(예: "A2"). 그 구역의 두 끝점 중
                    # 지금 위치에서 더 가까운(짧은 경로) 쪽으로 진입한다.
                    zstart, zend = ZONES[target]
                    path_a = find_path(cur, zstart)
                    path_b = find_path(cur, zend)
                    candidates = []
                    if path_a: candidates.append((zstart, path_a))
                    if path_b: candidates.append((zend, path_b))
                    if candidates:
                        entry_node, path = min(candidates, key=lambda x: len(x[1]))
                    else:
                        entry_node, path = None, None

                    if path:
                        _start_route(path)
                        with nav.lock:
                            nav.zone_code = target
                            nav.zone_entry_node = entry_node
                            nav.gas_target = None
                        publish_mark(f"구역 {target} 진입 위해 '{entry_node}' 로 이동 시작"
                                     f" (경로 {len(path)}구간)")
                    else:
                        publish_mark(f"구역 {target} 진입 노드로 가는 경로 없음")
                        with nav.lock:
                            nav.gas_target = None

                # 8/21 추가 -- IDLE 에서도 ESP32 로 POS 를 계속 보낸다.
                #
                # 이걸 안 보내던 것이 "대기 중에 타겟이 들어와도 로봇이 안 가는"
                # 버그의 원인이었다. ESP32 는 라파 스트림이 2초 끊기면
                # rpi_link_lost 로 스스로 정지하는 안전장치를 갖고 있는데,
                # 그 판정이 이렇게 돈다:
                #   if (state == RUNNING && millis() - lastRpiMsgTime >= 2000)
                # IDLE 로 몇 초 대기하는 동안 lastRpiMsgTime 이 낡은 채로 있다가,
                # _start_route 가 CMD:g 를 보내 state 가 RUNNING 이 되는 순간
                # "바로 다음 루프"에 이 조건이 참이 되어 즉시 다시 멈춘다.
                # (라파의 첫 POS 는 0.2초 뒤에나 도착한다)
                # 지금까지 안 드러난 이유는, 성공한 주행에서는 gas_target 이
                # 전부 INIT 중에 들어와 IDLE 을 한 루프(0.05초)만 스쳐 지나갔기
                # 때문이다. IDLE 로 실제 대기한 이번 C3 출동에서 처음 재현됐다.
                #
                # ESP32 는 STOPPED 상태라 이 POS 로 움직이지 않는다. 스트림이
                # 살아 있다는 것만 알려주는 하트비트다.
                if now - last_send_time >= SEND_INTERVAL_SEC:
                    last_send_time = now
                    idle_pos = cur or "IDLE"
                    ser.write(f"POS:{idle_pos},TVEC:0.0,NEAR:0\n".encode("utf-8"))

                if now - last_state_publish > 0.5:
                    last_state_publish = now
                    publish_state()
                time.sleep(0.05)
                continue

            # ═══════════════════ EXIT_HALT: 출구 도착 후 2초 정지 ═══════════════════
            # 8/20 추가. 이 시점에 로봇은 끝라인 위에 서 있고 라인센서는 111 이다.
            # 2초 뒤 CMD:u 를 보내면 ESP32가 알아서
            #   111 이탈 확인 -> 연장선 끝까지 전진 -> 000 -> 제자리 피벗 -> 라인 재획득
            # 까지 수행한다. 라파는 그동안 관여하지 않는다.
            if nav.mode in ("EXIT_HALT", "DEADEND_HALT"):
                if STREAM_ENABLED:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    draw_qr_overlay(frame_bgr, None, None)

                with nav.lock:
                    override = nav.manual_override
                    nav.manual_override = None
                    elapsed = time.time() - nav.exit_halt_started
                    exit_node = nav.uturn_exit_node

                if override == "stop":
                    ser.write(b"CMD:s\n")
                    with nav.lock:
                        nav.mode = "IDLE"
                    publish_mark("출구 정지 중 정지 명령 -> IDLE")
                    continue

                if elapsed >= EXIT_HALT_SECONDS:
                    ser.write(b"CMD:u\n")
                    with nav.lock:
                        at_exit = nav.mode == "EXIT_HALT"
                        nav.mode = "UTURN_SEEK" if at_exit else "DEADEND_UTURN"
                        nav.uturn_started = time.time()
                        nav.esp32_uturn_done = False
                        nav.uturn_seek_since = 0.0   # 8/21: 탐색 구간 시작 전
                        nav.esp32_align_done = False # 8/21: 정렬 완료 플래그도 새로
                    publish_mark(f"'{exit_node}' U턴 시작 -- "
                                 + ("완료 신호(uturn_done) 대기 후 QR 확인"
                                    if at_exit else "이후 최단 출구로 재출발"))

                if now - last_send_time >= SEND_INTERVAL_SEC:
                    last_send_time = now
                    line = f"POS:{exit_node}_halt,TVEC:0.0,NEAR:0\n"
                    ser.write(line.encode("utf-8"))

                if now - last_state_publish > 0.5:
                    last_state_publish = now
                    publish_state()
                time.sleep(0.05)
                continue

            # ═══════════════════ DEADEND_UTURN: ML/MR U턴 완료 대기 후 재출발 ═══════════════════
            # 8/20 추가. ML/MR 은 출구가 아니므로 여기서 멈추면 안 된다.
            # ESP32의 uturn_done 마크를 기다렸다가, 그 지점에서 최단 출구까지의
            # 경로로 곧바로 다시 주행을 건다. U턴 직후 로봇은 그 노드에서
            # 이웃 노드 쪽을 향하고 있으므로 가상 구간(head_leg) 없이
            # 평범한 경로로 시작하면 된다.
            if nav.mode == "DEADEND_UTURN":
                if STREAM_ENABLED:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    draw_qr_overlay(frame_bgr, None, None)

                with nav.lock:
                    override = nav.manual_override
                    nav.manual_override = None
                    done = nav.esp32_uturn_done
                    elapsed = time.time() - nav.uturn_started
                    node_here = nav.uturn_exit_node

                if override == "stop":
                    ser.write(b"CMD:s\n")
                    with nav.lock:
                        nav.mode = "IDLE"
                    publish_mark("U턴 대기 중 정지 명령 -> IDLE")
                    continue

                # 8/21: 여기도 "완료 신호 대기" 이므로 UTURN_DONE_TIMEOUT_SEC 로 통일.
                # (UTURN_SEEK_TIMEOUT_SEC 는 이제 U턴 후 QR 탐색 전용 값이다)
                if done or elapsed >= UTURN_DONE_TIMEOUT_SEC:
                    if not done:
                        publish_mark(f"[경고] '{node_here}' U턴 완료 신호가 "
                                     f"{UTURN_DONE_TIMEOUT_SEC:.0f}초 안에 안 옴 -- 그냥 진행")
                    # CMD:g 로 ESP32의 라인폭 판정 억제(uturnActive)를 해제한다.
                    # 이걸 안 하면 복귀 중 교차로가 하나도 카운트되지 않아
                    # 회전각 게이팅이 통째로 어긋난다.
                    ser.write(b"CMD:g\n")
                    try:
                        exit_node, path = nearest_exit(node_here)
                    except ValueError as exc:
                        ser.write(b"CMD:s\n")
                        publish_mark(f"[복귀 실패] '{node_here}' 에서 출구 없음 -- {exc}")
                        with nav.lock:
                            nav.mode = "IDLE"
                        continue
                    with nav.lock:
                        nav.current_node = node_here
                        nav.esp32_uturn_done = False
                    _start_route(path, mode="RETURN")
                    publish_mark(f"'{node_here}' U턴 완료 -- 출구 {exit_node} 로 재출발 "
                                 f"({' -> '.join(path)})")

                if now - last_send_time >= SEND_INTERVAL_SEC:
                    last_send_time = now
                    line = f"POS:{node_here}_uturn,TVEC:0.0,NEAR:0\n"
                    ser.write(line.encode("utf-8"))

                if now - last_state_publish > 0.5:
                    last_state_publish = now
                    publish_state()
                time.sleep(0.03)
                continue

            # ═══════════════════ UTURN_SEEK: 출구 U턴 완료 대기 -> QR 확인 -> 대기 ═══════════════════
            # 8/20 추가. EXIT_HALT 에서 CMD:u 를 보낸 직후 들어온다.
            #
            # 8/21 전면 수정 -- U턴이 통째로 취소되던 버그.
            # 기존에는 CMD:u 를 보낸 "직후부터" QR 탐색을 시작했다. 그런데 그
            # 시점의 로봇은 U턴 1단계(TURN_U_CLEAR)로 끝라인을 향해 전진하는
            # 중이고, 출구 모서리에는 QR 이 2개 나란히 있어 곧바로 걸린다.
            # 그러면 여기서 CMD:s 를 보내는데, ESP32 는 그 명령으로
            # uturnActive 를 풀며 U턴 시퀀스 자체를 취소해 버린다.
            # 결과적으로 로봇은 제자리 회전(TURN_U_PIVOT)을 시작도 못 하고
            # 출구를 바라본 채 멈춰 있었다.
            #
            # 실측(TR 복귀)에서 확인된 증거:
            #   - 마크가 uturn_start -> uturn_endline_cleared 까지만 찍히고
            #     uturn_done 이 끝내 없음 (1단계에서 끊김)
            #   - state 의 rpm 이 30/33 처럼 둘 다 양수. 제자리 피벗이면
            #     반드시 좌우 부호가 반대여야 하는데 그런 구간이 없음
            #   - 읽힌 QR 이 (1,11)/(1,12) = 행 1. 카메라는 로봇 오른쪽 고정
            #     이므로 TR 에서 행 1 은 "동쪽(출구)을 볼 때" 보이는 벽이다.
            #     U턴을 마쳤다면 반대편인 행 0 이 나와야 한다.
            #
            # 수정: 짝인 DEADEND_UTURN 은 원래 esp32_uturn_done 을 기다리게
            # 돼 있었는데 여기만 빠져 있었다. 동일하게 맞춘다.
            #   1단계) uturn_done 마크가 올 때까지 QR 탐색을 아예 하지 않는다.
            #   2단계) 그 뒤에야 QR 을 찾고, 잡히면 정지시킨다.
            # 탐색 타임아웃도 CMD:u 시점이 아니라 uturn_done 시점부터 잰다.
            # (기존에는 피벗에 걸리는 시간까지 20초 안에 포함돼 있었다)
            if nav.mode == "UTURN_SEEK":
                with nav.lock:
                    override = nav.manual_override
                    nav.manual_override = None
                    uturn_done = nav.esp32_uturn_done
                    align_done = nav.esp32_align_done
                    exit_node = nav.uturn_exit_node
                    seek_since = nav.uturn_seek_since
                    turn_elapsed = time.time() - nav.uturn_started

                if override == "stop":
                    ser.write(b"CMD:s\n")
                    with nav.lock:
                        nav.mode = "IDLE"
                    publish_mark("U턴 탐색 중 정지 명령 -> IDLE")
                    continue

                # ── 1단계: U턴 + 정렬 완료 신호 대기 ──
                # 이 동안에는 QR 을 읽지 않는다. 읽어서 정지시키면 U턴이 취소된다.
                # uturn_done(피벗 끝) 만으로는 부족하다. ESP32 는 그 뒤에도
                # TURN_ALIGN 으로 좌우를 훑으며 라인 위에 올라앉는데, 그 도중에
                # 세우면 라인에서 벗어난 자세로 다음 임무를 시작하게 된다.
                # align_done(또는 align_giveup)까지 받고 나서 탐색을 시작한다.
                if not (uturn_done and align_done):
                    if turn_elapsed >= UTURN_DONE_TIMEOUT_SEC:
                        ser.write(b"CMD:s\n")
                        with nav.lock:
                            nav.mode = "IDLE"
                            nav.current_node = exit_node
                        stage = ("피벗(uturn_done)" if not uturn_done
                                 else "정렬(align_done)")
                        publish_mark(f"[경고] '{exit_node}' U턴 {stage} 완료 신호가 "
                                     f"{UTURN_DONE_TIMEOUT_SEC:.0f}초 안에 안 옴 -- 정지. "
                                     f"로봇 자세 수동 확인 필요")
                        continue

                    if STREAM_ENABLED:
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        draw_qr_overlay(frame_bgr, None, None)

                    # ESP32 스트림 타임아웃(2초)에 걸리지 않게 계속 보내준다.
                    if now - last_send_time >= SEND_INTERVAL_SEC:
                        last_send_time = now
                        ser.write(f"POS:{exit_node}_uturn,TVEC:0.0,NEAR:0\n".encode("utf-8"))

                    if now - last_state_publish > 0.5:
                        last_state_publish = now
                        publish_state()
                    time.sleep(0.03)
                    continue

                # U턴 완료 직후 한 번만 -- 탐색 구간 시작 시각을 기록한다.
                if seek_since == 0.0:
                    with nav.lock:
                        nav.uturn_seek_since = time.time()
                        seek_since = nav.uturn_seek_since
                    publish_mark(f"'{exit_node}' U턴 + 정렬 완료 -- 이제 맵 안쪽을 향함. "
                                 f"위치 확인용 QR 탐색 시작")

                # ── 2단계: QR 탐색 ──
                multi = detect_qr_multi(frame, qr_detector)
                seen = [parse_qr_payload(pl) for pl, _ in multi]
                seen = [c for c in seen if c is not None]
                # 8/21 추가 -- 동시 인식(multi)이 실패하는 프레임이 많아 단일
                # 인식도 같이 시도한다. INIT 이 단일 인식만으로 잘 동작하므로
                # 여기서만 2개를 고집할 이유가 없다.
                if not seen:
                    single_payload, single_pts = detect_qr(frame, qr_detector)
                    c = parse_qr_payload(single_payload) if single_payload else None
                    if c is not None:
                        seen.append(c)
                # 같은 QR 이 원본/반전 양쪽에서 중복으로 잡히므로 좌표로 중복 제거
                uniq = []
                for c in seen:
                    if all(distance(c, u) > 0.01 for u in uniq):
                        uniq.append(c)

                if STREAM_ENABLED:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    draw_qr_overlay(frame_bgr, [(pl, pts, False) for pl, pts in multi], None)

                elapsed = time.time() - seek_since

                # 8/21 -- 성공 조건 완화. 2개 동시 인식은 U턴 후 반대편 벽에서
                # 항상 잡힌다는 보장이 없다. INIT 과 같은 기준(출구 노드에서
                # INIT_NODE_MAX_DIST 이내의 QR 1개)도 인정한다.
                exit_coord = NODES.get(exit_node)
                near_qr = None
                if exit_coord is not None:
                    for c in uniq:
                        if distance(c, exit_coord) <= INIT_NODE_MAX_DIST:
                            near_qr = c
                            break

                if len(uniq) >= 2 or near_qr is not None:
                    ser.write(b"CMD:s\n")
                    with nav.lock:
                        nav.mode = "IDLE"
                        nav.current_node = exit_node
                    reason = (f"QR 2개 확인 {uniq[:2]}" if len(uniq) >= 2
                              else f"QR {near_qr} 이 '{exit_node}' 근처")
                    publish_mark(f"{reason} -- 정지, '{exit_node}' 에서 맵을 향해 대기. "
                                 f"다음 gas_target 수신 시 즉시 출동")
                elif elapsed >= UTURN_SEEK_TIMEOUT_SEC:
                    ser.write(b"CMD:s\n")
                    with nav.lock:
                        nav.mode = "IDLE"
                        nav.current_node = exit_node
                    publish_mark(f"[경고] U턴 완료 후 {UTURN_SEEK_TIMEOUT_SEC:.0f}초 안에 QR 을 "
                                 f"못 찾음 -- 그냥 정지. 로봇 자세 수동 확인 필요")

                # ESP32 스트림 타임아웃(2초)에 걸리지 않게 계속 보내준다.
                if now - last_send_time >= SEND_INTERVAL_SEC:
                    last_send_time = now
                    line = f"POS:{exit_node}_uturn,TVEC:0.0,NEAR:0\n"
                    ser.write(line.encode("utf-8"))

                if now - last_state_publish > 0.5:
                    last_state_publish = now
                    publish_state()
                time.sleep(0.03)
                continue

            # ═══════════════════ ZONE_ENTER: 구역 중간 지점까지 진입 ═══════════════════
            if nav.mode == "ZONE_ENTER":
                payload, points = detect_qr(frame, qr_detector)
                coords = parse_qr_payload(payload) if payload else None

                if STREAM_ENABLED:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    draw_qr_overlay(frame_bgr, None, (payload, points))

                with nav.lock:
                    override = nav.manual_override
                    nav.manual_override = None
                if override == "stop":
                    with nav.lock:
                        nav.mode = "IDLE"
                        nav.zone_code = None
                    publish_mark("구역 진입 중 정지 명령 -> IDLE")
                    continue

                zone_done = False
                done_reason = ""

                if coords is not None:
                    print(f"[QR] 읽음: {coords}")
                    with nav.lock:
                        # 8/21 추가 -- ZONE_ENTER 에서도 최근 QR 좌표를 갱신한다.
                        # 여태 NAVIGATE/RETURN 에서만 갱신해서, 구역 진입 내내
                        # robot/event 의 location 이 진입 직전 좌표에 얼어붙어
                        # 있었다(실측: (7,3)/(8,3)/(9,3) 을 읽는 동안에도
                        # 대시보드에는 계속 (3,3) 으로 발행됨).
                        nav.last_qr_coords = coords
                        candidates = list(nav.route_coords)
                    for cand in candidates:
                        if distance(coords, cand) <= ZONE_STOP_QR_RADIUS:
                            zone_done = True
                            done_reason = f"정지 QR {cand} 인식"
                            break

                # ── 안전망 1: 물리적 끝점 통과 (카메라와 독립) ──
                # esp32_crossing_count 가 목표를 넘어섰다는 건 구역 반대편
                # 끝의 교차선을 실제로 지났다는 뜻이다. 후보 QR을 전부
                # 놓쳤더라도 여기서 확실히 멈춘다.
                if not zone_done:
                    with nav.lock:
                        overshot = nav.esp32_crossing_count > nav.zone_entry_crossing_target
                    if overshot:
                        zone_done = True
                        done_reason = "구역 끝 교차선 통과 감지 (QR 미인식)"

                # ── 안전망 2: 시간 ──
                if not zone_done:
                    with nav.lock:
                        ze_elapsed = time.time() - nav.zone_enter_started
                    if ze_elapsed >= ZONE_ENTER_TIMEOUT_SEC:
                        zone_done = True
                        done_reason = f"{ZONE_ENTER_TIMEOUT_SEC:.0f}초 타임아웃 (QR·교차선 모두 미감지)"

                if zone_done:
                    with nav.lock:
                        nav.mode = "ZONE_STOP"
                        nav.zone_stop_started = time.time()
                        nav.zone_stop_halt_sent = False
                        nav.current_node = None   # 특정 노드가 아니라 구역 내부
                        zcode = nav.zone_code
                    publish_mark(f"구역 {zcode} 정지 지점 도달 ({done_reason}) -- "
                                 f"{ZONE_STOP_SECONDS:.0f}초 정지 시작")

                if now - last_send_time >= SEND_INTERVAL_SEC:
                    last_send_time = now
                    with nav.lock:
                        # ESP32가 실제로 그 교차로를 물리적으로 지났다고
                        # 확인해줄 때까지는 저장해둔 회전각을 계속 재전송.
                        # 확인이 너무 오래 안 오면 그냥 진행.
                        waiting = nav.esp32_crossing_count < nav.zone_entry_crossing_target
                        if waiting:
                            if nav._zone_entry_wait_since == 0:
                                nav._zone_entry_wait_since = time.time()
                            timed_out = (time.time() - nav._zone_entry_wait_since) > LEG_LAG_TIMEOUT_SEC
                        else:
                            nav._zone_entry_wait_since = 0
                            timed_out = False

                        # 8/21 수정 -- 타임아웃이 "대기 중인 회전"을 지우지 못하게.
                        #
                        # 실측(B2 출동, TL->A->C) 실패 재현:
                        #   'C' 도착 판정이 QR (3,3) 에서 났다. C 는 (5.5,4) 이므로
                        #   2.5칸(복도 한가운데) 앞이다. 세로 구간의 QR 은 행 2/3/4 라
                        #   C 까지 거리가 3.5/2.5/1.5 인데, 반경 2.6 이 행 3 을 통과시킨다.
                        #   그 지점에서 ZONE_ENTER 로 들어가 TVEC:-90 을 재전송하기
                        #   시작했지만, 남은 2.5칸을 3초 안에 못 갔다.
                        #   -> LEG_LAG_TIMEOUT_SEC(3초) 이 발동해 TVEC 을 0 으로 덮음
                        #   -> 진짜 C 교차로에서 tvec=0 이라 직진 통과
                        #   -> B2 구역이 아니라 C->E 복도로 내려가다 line_lost
                        #
                        # 이 타임아웃의 원래 목적은 "ESP32 가 라인폭으로 잡지 못하는
                        # 직진 통과점에서 영원히 막히는 것"을 푸는 것이다. 회전이
                        # 걸려 있을 때는 해당되지 않는다. 회전은 물리적 교차로 확인을
                        # 끝까지 기다려야 하고, 정말 막히면 ZONE_ENTER_TIMEOUT_SEC(15초)
                        # 이 최후 방어선으로 잡아준다.
                        # (NAVIGATE 쪽 같은 성격의 타임아웃은 8/21 에 이미 고쳤는데,
                        #  ZONE_ENTER 에 별도로 하나 더 있는 것을 놓쳤다)
                        turn_pending_zone = abs(nav.zone_entry_tvec) > TURN_PENDING_DEG
                        if turn_pending_zone:
                            timed_out = False
                            nav._zone_entry_wait_since = time.time()
                        tvec = 0.0 if (not waiting or timed_out) else nav.zone_entry_tvec
                        pos_label = f"{nav.zone_code}_enter"
                        # 8/21 추가 -- NEAR 를 상시 1 로 보내지 않는다.
                        # 진입 교차로(구역 시작점)를 실제로 통과하기 전까지는
                        # 1 이어야 그 회전이 실행된다. 하지만 통과한 뒤에도 1 로
                        # 두면, 구역 안 복도에서 뜨는 가짜 111 을 ESP32 가 그대로
                        # 받아들여 esp32_crossing_count 를 올린다. 그러면
                        # "구역 끝 교차선 통과 감지" 안전망(overshot)이 오작동해
                        # 구역 중앙에 닿기도 전에 정지 판별이 시작된다.
                        near_here = 1 if waiting else 0
                    line = f"POS:{pos_label},TVEC:{tvec:.1f},NEAR:{near_here}\n"
                    ser.write(line.encode("utf-8"))
                    print(f"  -> 송신: {line.strip()}")

                if now - last_state_publish > 0.5:
                    last_state_publish = now
                    publish_state()
                time.sleep(0.03)
                continue

            # ═══════════════════ ZONE_STOP: 가스 판별 정지 ═══════════════════
            if nav.mode == "ZONE_STOP":
                if STREAM_ENABLED:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    draw_qr_overlay(frame_bgr, None, None)
                with nav.lock:
                    override = nav.manual_override
                    nav.manual_override = None
                    elapsed = time.time() - nav.zone_stop_started
                    zone_code = nav.zone_code
                    just_entered = not nav.zone_stop_halt_sent

                if override == "stop":
                    with nav.lock:
                        nav.mode = "IDLE"
                        nav.zone_code = None
                    publish_mark("구역 정지 중 정지 명령 -> IDLE")
                    continue

                # TVEC:0(직진)만 보내면 ESP32가 실제로 안 멈추고 계속 주행해서
                # 3초 "정지" 동안 라인 밖으로 벗어나 line_lost 가 나던 버그.
                # ZONE_STOP 진입 시 실제 정지(CMD:s)를 한 번 보낸다.
                if just_entered:
                    ser.write(b"CMD:s\n")
                    with nav.lock:
                        nav.zone_stop_halt_sent = True
                    # 8/21 -- 아두이노 리더 스레드가 response 버퍼를 채우기 시작한다.
                    # 이 직전까지 쌓인 baseline 버퍼(최근 6샘플)가 "정상 공기"
                    # 대표값이 되어 변화율 계산의 기준이 된다.
                    with gas_lock:
                        gas_response_buffer.clear()
                    set_gas_collecting(True)
                    publish_mark("구역 정지 -- ESP32에 실제 정지 명령 전송")

                # 5채널 샘플링은 gas_arduino_reader 스레드가 백그라운드에서
                # 알아서 한다 -- 여기서는 대기만.

                if now - last_send_time >= SEND_INTERVAL_SEC:
                    last_send_time = now
                    line = f"POS:{zone_code}_stop,TVEC:0.0,NEAR:0\n"
                    ser.write(line.encode("utf-8"))
                    print(f"  -> 송신(정지 유지): {line.strip()}")

                if elapsed >= ZONE_STOP_SECONDS:
                    # ── 8/20 전면 수정: 방향 선택형 복귀 ──
                    # 지금 로봇은 구역 한가운데에서 진입 노드를 '등지고' 있다.
                    # 기존 코드는 그냥 CMD:g 로 재개시키고 경로를 진입 노드에서
                    # 시작해서, (1) 로봇은 구역 안쪽으로 계속 굴러가고
                    # (2) 첫 교차로 회전각이 한 칸 밀려 엉뚱한 노드 기준으로
                    # 계산되는 두 가지 오류가 있었다.
                    with nav.lock:
                        nav.zone_stop_halt_sent = False
                        entry_node = nav.zone_entry_node
                        stop_coord = nav.zone_stop_coord

                    # 8/20 추가 -- 정지 구간 동안 모은 샘플을 채널별 평균 내어
                    # 판별하고, 서버(safeDB)가 구독하는 robot/event로 발행한다.
                    # location은 프론트(parseQrLocation)가 좌표로 해석할 수 있게
                    # "(x,y)" 형식으로 보낸다 -- 구역코드(zone_code)는 mark 로그에만 남긴다.
                    # 8/21 -- 수집을 멈추고 그동안 쌓인 샘플로 판별한다.
                    set_gas_collecting(False)
                    with gas_lock:
                        response_samples = list(gas_response_buffer)

                    if response_samples:
                        avg = {}
                        for key in ("mq2_left", "mq2_right", "mq3", "mq135", "mq138"):
                            v = _avg_key(response_samples, key)
                            if v is not None:
                                avg[key] = round(v, 1)
                        gas_result = classify_gas(response_samples)
                        # 8/21 -- 계산상의 중간 좌표(stop_coord)가 아니라 실제로
                        # 정지 직전에 읽은 QR 을 위치로 쓴다. 둘은 자주 어긋난다
                        # (실측 B1: 계산 (5,2) vs 실제 정지 (5,3)). 대시보드
                        # 좌표가 한 칸 틀어지는 것을 막는다.
                        with nav.lock:
                            actual_coord = nav.last_qr_coords
                        use_coord = actual_coord or stop_coord
                        loc = (f"({use_coord[0]:.0f},{use_coord[1]:.0f})"
                               if use_coord else zone_code)
                        publish_robot_event(loc, "gas_response", avg, gas_result)
                        publish_mark(f"구역 {zone_code} 가스 판별: {gas_result[0]} "
                                     f"신뢰도={gas_result[1]} 세기={gas_result[2]} "
                                     f"등급={gas_result[3]} (샘플 {len(response_samples)}개)")
                    else:
                        publish_mark(f"구역 {zone_code} 가스 판별 실패 -- 센서 샘플 없음 "
                                     f"(아두이노 연결 확인)")

                    try:
                        plan = plan_zone_return(zone_code, entry_node)
                    except Exception as exc:
                        # 복귀 계획을 못 세우면 굴러가게 두는 것보다 멈추는 게 낫다.
                        ser.write(b"CMD:s\n")
                        publish_mark(f"[복귀 실패] 구역 {zone_code} / 진입 '{entry_node}' -- "
                                     f"{exc} -- 정지 유지, 수동 개입 필요")
                        with nav.lock:
                            nav.mode = "IDLE"
                            nav.zone_code = None
                            nav.zone_entry_node = None
                        continue

                    # 8/20: 새 설계에서는 정지 지점에서 곧바로 U턴하지 않는다.
                    # 복도 한가운데엔 끝라인도 연장선도 없어 U턴 시퀀스가
                    # 성립하지 않기 때문이다. 항상 전진 방향 그대로 구역
                    # 끝점까지 간 뒤, 거기가 막다른 지점(ML/MR)이면 도착 후
                    # DEADEND_HALT -> CMD:u 로 U턴한다.
                    ser.write(b"CMD:g\n")
                    if plan["deadend"]:
                        publish_mark(f"구역 {zone_code} 판별 완료 -- '{plan['path'][-1]}' 까지 "
                                     f"전진 후 그곳에서 U턴 예정")
                    else:
                        publish_mark(f"구역 {zone_code} 판별 완료 -- 전진 방향 그대로 "
                                     f"출구 {plan['exit']} 로 복귀")

                    with nav.lock:
                        nav.current_node = None
                        nav.zone_code = None
                        nav.zone_entry_node = None
                    _start_route(
                        plan["path"], mode="RETURN",
                        head_leg=(plan["head"], plan["qr"], plan["axis"], plan["stop"]),
                    )
                    publish_mark(f"복귀 경로: {VIRTUAL_START} -> {' -> '.join(plan['path'])} "
                                 f"(목표 {plan['exit'] or plan['path'][-1]}, "
                                 f"시작 방위 {plan['head']:.0f}도)")

                if now - last_state_publish > 0.5:
                    last_state_publish = now
                    publish_state()
                time.sleep(0.05)
                continue

            # ═══════════════════ NAVIGATE / RETURN: 노드 경로 주행 ═══════════════════
            payload, points = detect_qr(frame, qr_detector)
            coords = None
            if payload and points is not None and len(points) > 0:
                coords = parse_qr_payload(payload)

            # 주행 중 QR 인식 시도 결과를 2초마다 MQTT로도 보고.
            if now - last_nav_qr_diag > 2.0:
                last_nav_qr_diag = now
                diag = (f"[NAV-QR] payload={payload!r} coords={coords} "
                        f"wp_idx={nav.route_wp_idx}/{len(nav.route_coords)}")
                print(diag)
                publish_mark(diag)

            if STREAM_ENABLED:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                draw_qr_overlay(frame_bgr, None, (payload, points))

            with nav.lock:
                override = nav.manual_override
                nav.manual_override = None
            if override == "stop":
                with nav.lock:
                    nav.mode = "IDLE"
                publish_mark("주행 중 정지 명령 -> IDLE")
                continue

            if coords is not None:
                print(f"[QR] 읽음: {coords}")
                with nav.lock:
                    # 8/20: 좌표가 바뀐 경우에만 직전 값으로 보관
                    # (경로 이탈 시 진행 방향 판정에 쓰인다)
                    if nav.last_qr_coords is not None and \
                       distance(coords, nav.last_qr_coords) >= 0.5:
                        nav._off_route_prev = nav.last_qr_coords
                    nav.last_qr_coords = coords
                    if nav.route_wp_idx < len(nav.route_coords):
                        target_wp = nav.route_coords[nav.route_wp_idx]
                        wp_axis = nav.route_axes[nav.route_wp_idx] if nav.route_wp_idx < len(nav.route_axes) else None
                        if axis_distance(coords, target_wp, wp_axis) <= WAYPOINT_REACHED_RADIUS:
                            nav.route_wp_idx += 1

                    if nav.node_idx + 1 < len(nav.node_route):
                        cur_node_name = nav.node_route[nav.node_idx]
                        next_node = nav.node_route[nav.node_idx + 1]
                        next_node_coord = node_coord(next_node)
                        # route_coords 매칭과 동일하게 진행 축(행 또는 열)만
                        # 비교한다. 2D 원형 반경은 QR이 그 좁은 원 안에 정확히
                        # 찍혀야만 인정되는데, 실시간 주행 중엔 그 순간을 놓치기
                        # 쉬워 노드 도착이 누락되곤 했다.
                        arrive_axis = axis_between(cur_node_name, next_node)
                        # 8/20: 반경이 구간 길이를 넘으면 출발점에서 곧바로
                        # 도착이 성립한다. 구간 길이에 비례해 상한을 건다.
                        cur_coord = node_coord(cur_node_name)
                        arrive_radius = WAYPOINT_REACHED_RADIUS
                        if cur_coord is not None:
                            leg_len = axis_distance(cur_coord, next_node_coord, arrive_axis)
                            if leg_len > 0:
                                arrive_radius = min(arrive_radius, leg_len * ARRIVE_LEG_FRACTION)
                        # 진행축만 넓게 보되, 반대축(벽 쪽)이 다른 노드 수준으로
                        # 크게 벗어나 있으면 거부.
                        if axis_distance_sane(coords, next_node_coord, arrive_axis) <= arrive_radius:
                            nav.node_idx += 1
                            nav.current_node = next_node
                            nav._node_idx_stuck_since = time.time()
                            publish_mark(f"'{next_node}' 도착")

                # 8/20: 경로에 없는 간선의 QR -- 복도 한가운데 이탈. 즉시 정지.
                if check_off_route_edge(coords, ser):
                    continue

                # 예상 밖 위치 감지(노드 근접) -- 벗어났으면 정지+재탐색
                if check_off_route_and_reroute(coords, ser):
                    continue

            # 같은 노드에서 너무 오래 정체되면 완전 정지 (무한 루프 방지).
            # QR을 못 읽는 동안에도 감시해야 하므로 coords 유무와 무관하게 확인.
            if check_stuck_and_stop(ser):
                continue

            # ── 8/21 추가: 막다른 지점 도착을 "끝라인 통과"로도 인정 ──
            # 복귀 목적지(TL/TR/BL/BR/ML/MR)는 전부 이웃이 하나뿐인 막다른
            # 지점이고, 그 앞에는 끝라인이 깔려 있다. 그 끝라인을 밟는 순간
            # ESP32 가 intersection 마크를 보내므로, 카메라 QR 판정보다
            # 확실하고 빠르다.
            #
            # 왜 필요한가 -- 실측(B1 -> ML 복귀) 실패:
            #   구역 정지 지점을 plan_zone_return 은 seg[mid] = (5,2) 로
            #   가정하는데, 실제로 로봇이 선 곳은 (5,3) 이었다. 그 한 칸
            #   차이로 _ZMID -> ML 구간 길이가 2 에서 1 로 줄었고,
            #   ARRIVE_LEG_FRACTION(0.8) 이 도착 반경을 0.8 로 깎아버렸다.
            #   그러면 (5,2)의 거리 1.0 은 탈락하고 노드 QR (5,1) 에 정확히
            #   닿아야만 도착이 성립한다. 카메라는 라인센서보다 뒤에 있으므로
            #   그 시점에 센서는 이미 ML 을 지나쳐 있었다:
            #     'ML' 도착 -> ML_halt ls:"000" -> uturn_pivot_start_no111
            #     -> align_done_edge -> line_lost
            #   끝라인도 연장선도 없는 맨바닥에서 U턴을 시도한 것이다.
            #   (로그에는 그 직전에 "intersection pos=? tvec=0 w=3.3" 이
            #    찍혀 있었다. 진짜 끝라인을 밟았다는 신호가 이미 있었는데
            #    도착 판정에 쓰지 않고 있었을 뿐이다)
            #
            # QR 판정은 그대로 두고 둘 중 먼저 걸리는 쪽을 쓴다.
            # NAVIGATE 에는 적용하지 않는다 -- 구역 진입 노드는 막다른
            # 지점이 아니고, 그쪽은 회전각 게이팅이 따로 돌기 때문이다.
            endline_arrival = None
            with nav.lock:
                if nav.mode == "RETURN" and len(nav.node_route) >= 2:
                    final_node = nav.node_route[-1]
                    if is_dead_end(final_node) \
                            and nav.node_idx < len(nav.node_route) - 1 \
                            and nav.esp32_crossing_count >= len(nav.leg_headings):
                        nav.node_idx = len(nav.node_route) - 1
                        nav.current_node = final_node
                        endline_arrival = final_node
            if endline_arrival:
                publish_mark(f"'{endline_arrival}' 끝라인 통과 감지 -- QR 판정보다 "
                             f"먼저 도착 확정 (U턴 시퀀스를 제자리에서 시작하기 위함)")

            with nav.lock:
                finished = (nav.node_idx >= len(nav.leg_headings)) and \
                           (nav.node_idx == len(nav.node_route) - 1)

            if finished:
                with nav.lock:
                    mode_was = nav.mode
                    cur = nav.current_node
                    zone_code = nav.zone_code
                if mode_was == "NAVIGATE" and zone_code:
                    # 구역 진입 지점 도착 -- 구역 안쪽 중간 지점까지 진입 시작
                    with nav.lock:
                        entry_node = nav.zone_entry_node
                    start_node, enter_qr, enter_axis, stop_coord, zheading = zone_info(zone_code, entry_node)
                    # 8/20: 순서대로 다 읽어야 하는 enter_qr 대신, "하나라도
                    # 읽으면 정지"하는 후보 집합을 쓴다. 테이블에 없으면
                    # 예전 방식(enter_qr)으로 안전하게 되돌아간다.
                    stop_candidates = ZONE_STOP_QR.get(zone_code, enter_qr)
                    with nav.lock:
                        prev_heading = nav.leg_headings[-1] if nav.leg_headings else 0.0
                        nav.mode = "ZONE_ENTER"
                        nav.route_coords = stop_candidates
                        nav.route_axes = [enter_axis] * len(stop_candidates)
                        nav.route_wp_idx = 0
                        nav.zone_enter_started = time.time()
                        nav.zone_stop_coord = stop_coord
                        nav.zone_heading = zheading
                        # 회전각을 저장해두고, ESP32가 실제 마지막 교차로를
                        # 감지했다고 보고할 때까지 ZONE_ENTER 에서 계속 재전송.
                        nav.zone_entry_tvec = normalize_angle(zheading - prev_heading)
                        nav.zone_entry_crossing_target = len(nav.leg_headings)
                        turn_tvec = nav.zone_entry_tvec
                    line = f"POS:{zone_code}_turn,TVEC:{turn_tvec:.1f},NEAR:1\n"
                    ser.write(line.encode("utf-8"))
                    publish_mark(f"'{start_node}' 도착 -- 구역 {zone_code} 진입 시작 (회전각 {turn_tvec:.0f}도)")

                    # 회전각이 거의 180도면 방금 온 길을 되돌아가야 한다는 뜻이다.
                    # 8/20: U턴 시퀀스는 끝라인 + 연장선이 있는 막다른 지점에서만
                    # 성립한다(연장선 끝의 000 을 기다리므로). 교차점에서는
                    # 물리적으로 불가능하니 시도하지 않고 경고만 남긴다.
                    if abs(turn_tvec) > 150.0:
                        if is_dead_end(start_node):
                            ser.write(b"CMD:u\n")
                            publish_mark(f"U턴 즉시 실행 (회전각 {turn_tvec:.0f}도, 감지 대기 안 함)")
                        else:
                            publish_mark(f"[경고] '{start_node}' 는 막다른 지점이 아니라 "
                                         f"U턴 불가 (회전각 {turn_tvec:.0f}도) -- 경로 재검토 필요")
                elif mode_was == "NAVIGATE":
                    publish_mark(f"'{cur}' 도착 -- 대기 상태로 전환")
                    with nav.lock:
                        nav.mode = "IDLE"
                        nav.node_route = []
                elif mode_was == "RETURN":
                    # 8/20 개정 -- 도착한 곳이 출구인지 중간 막다른 지점인지로 갈린다.
                    #   출구(TL/TR/BL/BR): 2초 정지 -> U턴 -> 정렬 -> QR 확인 후 대기
                    #   막다른 지점(ML/MR): 2초 정지 -> U턴 -> 최단 출구로 재출발
                    # 둘 다 U턴 자체는 동일한 ESP32 시퀀스(CMD:u)를 쓴다.
                    ser.write(b"CMD:s\n")
                    with nav.lock:
                        nav.mode = "EXIT_HALT" if cur in EXIT_NODES else "DEADEND_HALT"
                        nav.exit_halt_started = time.time()
                        nav.uturn_exit_node = cur
                        nav.esp32_uturn_done = False
                        nav.node_route = []
                        nav.zone_code = None
                        nav.virtual_start_coord = None
                    if cur in EXIT_NODES:
                        publish_mark(f"출구 '{cur}' 도착 -- {EXIT_HALT_SECONDS:.0f}초 정지 후 U턴")
                    else:
                        publish_mark(f"막다른 지점 '{cur}' 도착 -- {EXIT_HALT_SECONDS:.0f}초 정지 후 "
                                     f"U턴하고 최단 출구로 복귀")
                continue

            if now - last_send_time >= SEND_INTERVAL_SEC:
                last_send_time = now
                with nav.lock:
                    # ESP32 가 실제로 교차로를 감지해서 보고한 횟수를 넘어서지
                    # 않게 제한 -- 카메라(node_idx)가 미리 도착 판정을 내려도,
                    # ESP32 가 그 물리적 교차로를 지나기 전까지는 회전각을
                    # 다음 구간용으로 앞당겨 보내지 않는다.
                    # 다만 직진 통과 지점(예: C)처럼 ESP32가 라인폭을 "교차로"로
                    # 아예 못 잡는 경우도 있어, 너무 오래 안 따라잡으면 카메라
                    # 판단을 믿고 강제로 진행시킨다.
                    if nav.node_idx > nav.esp32_crossing_count:
                        if nav._leg_lag_since == 0:
                            nav._leg_lag_since = time.time()
                    else:
                        nav._leg_lag_since = 0

                    gated_leg = min(nav.node_idx, nav.esp32_crossing_count)
                    gated_tvec = _tvec_at(nav.leg_headings, gated_leg)

                    # 8/20 중요 수정 -- 회전이 대기 중이면 타임아웃을 적용하지 않는다.
                    # 이 타임아웃은 원래 "직진 통과 지점을 ESP32가 교차로로 못
                    # 잡아 영원히 막히는 것"을 풀려는 장치다. 그런데 회전 대기
                    # 중에 발동하면 effective_leg 를 앞당겨 그 회전각을 0으로
                    # 덮어써 버린다. 실측에서 정확히 이 일이 났다:
                    #   도착 반경 2.6 으로 (11,2)에서 'E 도착' -> 3초 뒤 타임아웃
                    #   -> TVEC -82.9 가 0.0 으로 지워짐 -> 실제 E 교차로에서 직진
                    #   -> 경로 이탈 -> 벽 충돌
                    # 회전이 걸려 있으면 물리적 교차로 확인을 끝까지 기다린다.
                    # 정말 막히면 STUCK_TIMEOUT 이 최후 방어선으로 잡아준다.
                    turn_pending = abs(gated_tvec) > TURN_PENDING_DEG

                    # 8/21 추가 -- 타임아웃이 "회전을 무장"하는 것도 막는다.
                    # 기존에는 현재 구간의 회전 여부(turn_pending)만 봤다.
                    # 그런데 도착 판정이 2칸 일찍 나면(반경 2.6, QR 간격 1.0)
                    # node_idx 가 실제보다 앞서고, 3초 뒤 타임아웃이 발동해
                    # "아직 도달하지도 않은 다음 노드의 회전각"을 미리 실어보낸다.
                    # 실측 재현: TL->A 구간에서 QR (1,2) 로 'A 도착'(A 는 열4) ->
                    # 3초 뒤 effective_leg 가 1 로 밀림 -> TVEC 90(B 에서 할 회전)이
                    # A 근처에서 무장 -> A 에서 NEAR 가 열리는 순간 A 에서 우회전.
                    # 이 타임아웃의 목적은 "ESP32 가 못 잡는 직진 통과점 뚫기"
                    # 하나뿐이므로, 앞당긴 결과도 직진일 때만 허용한다.
                    forced_tvec = _tvec_at(nav.leg_headings, nav.node_idx)
                    forced_is_straight = abs(forced_tvec) <= TURN_PENDING_DEG

                    if turn_pending:
                        effective_leg = gated_leg
                        nav._leg_lag_since = 0
                    elif (nav._leg_lag_since
                          and (time.time() - nav._leg_lag_since) > LEG_LAG_TIMEOUT_SEC
                          and forced_is_straight):
                        effective_leg = nav.node_idx   # 직진 -> 직진일 때만 강제 진행
                    else:
                        effective_leg = gated_leg

                    tvec = _tvec_at(nav.leg_headings, effective_leg)
                    pos_label = nav.current_node or "?"
                    # 8/20: node_idx 가 아니라 effective_leg 기준.
                    # 8/21: 래치 적용. 한 번 열린 창은 그 구간이 끝날 때까지 유지한다.
                    if nav._near_latch_leg != effective_leg:
                        nav._near_latch_leg = None
                    if compute_near_flag(nav.last_qr_coords, nav.node_route, effective_leg):
                        nav._near_latch_leg = effective_leg
                    # 8/21 추가 -- 두 번째 개방 조건: 카메라가 이미 도착 판정을 냈다.
                    #
                    # 반경 1.6 은 "노드 바로 옆 QR 한 개"만 창을 연다. 그 QR 을
                    # 놓치면 창이 영영 안 열리고, 그러면 ESP32 가 진짜 교차로까지
                    # 기각해 회전이 통째로 누락된다.
                    # 실측(BL->E->C->D, B3 출동)에서 정확히 이렇게 무너졌다:
                    #   E->C 구간에서 (8,5) 다음 행 7/6/5 를 연속으로 놓침
                    #   (C 에 가장 가까운 QR 은 행 7, 거리 1.5)
                    #   -> NEAR 가 끝까지 0 -> 진짜 C 교차로가
                    #      crossing_rejected_not_near tvec=90 으로 기각
                    #   -> 90도 회전 누락 -> 그대로 북진해 C-A 복도로 이탈
                    #
                    # node_idx > effective_leg 는 "카메라는 다음 노드에 닿았다고
                    # 보는데 ESP32 는 아직 그 교차로를 통과하지 못했다"는 뜻이다.
                    # 즉 교차로가 바로 눈앞이거나 방금 지나간 상태다. 창을 연다.
                    #
                    # 트레이드오프 -- 도착 판정 반경이 2.6 이라 이 조건은 최대
                    # 2칸 일찍 열릴 수 있다. 8/20 의 오회전 사고가 그 창으로
                    # 들어온 가짜 111 이었다. 다만 그 가짜는 폭이 한 루프(약
                    # 0.5cm)짜리였고, 이후 ESP32 에 crossMinCm=1.5 하한을 넣어
                    # 그 부류는 폭에서 이미 걸러진다(이후 실측 교차로 폭은
                    # 1.7~2.9cm). 회전을 통째로 놓치는 쪽이 지금은 훨씬 잦고
                    # 피해도 크므로 이쪽을 택한다.
                    if nav.node_idx > effective_leg:
                        nav._near_latch_leg = effective_leg
                    near_flag = (nav._near_latch_leg == effective_leg)

                line = f"POS:{pos_label},TVEC:{tvec:.1f},NEAR:{1 if near_flag else 0}\n"
                ser.write(line.encode("utf-8"))
                print(f"  -> 송신: {line.strip()}")

            if now - last_state_publish > 0.5:
                last_state_publish = now
                publish_state()

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n종료")
    except Exception:
        # 8/20 추가 -- 예상 못 한 예외로 프로세스가 죽으면 로봇이 마지막
        # 명령 그대로 굴러가다 벽에 부딪힌다(KeyError: 'uturn' 으로 실제 발생).
        # 무조건 세우고, 트레이스백을 MQTT 로도 남겨 원격에서 원인을 본다.
        import traceback
        tb = traceback.format_exc()
        print(tb)
        try:
            ser.write(b"CMD:s\n")
        except Exception:
            pass
        try:
            # MQTT 페이로드가 너무 길어지지 않게 마지막 몇 줄만
            publish_mark("[치명적 오류] " + " | ".join(tb.strip().splitlines()[-3:]))
            time.sleep(0.3)   # 발행이 나갈 시간
        except Exception:
            pass
    finally:
        try:
            ser.write(b"CMD:s\n")   # 종료 시엔 항상 정지시킨다
            time.sleep(0.1)
        except Exception:
            pass
        picam2.stop()
        ser.close()
        mqtt_client.loop_stop()


# ── 실시간 경로 이탈 감지 + 자동 재탐색 ──────────────────
# 지금 읽은 QR 좌표가 "기대하는 현재 노드/다음 노드"와는 안 맞는데, 지도의
# 다른 노드와는 아주 가깝게 일치하면 -- 경로를 벗어난 것으로 보고 즉시 정지
# -> 그 지점에서 원래 목적지까지 새 경로 계산 -> 재개.
OFF_ROUTE_NODE_RADIUS = 0.8   # 이 안에 들어와야 "그 노드에 확실히 와 있다"
REROUTE_COOLDOWN_SEC = 5.0    # 연쇄 재탐색 방지


# ── 8/20 추가: 간선 기반 경로 이탈 감지 ──────────────────
# 기존 노드 근접(0.8) 판정은 복도 한가운데서는 절대 걸리지 않는다. 실측에서
# E 를 직진 통과한 뒤 E-F 구간 QR을 계속 읽으면서도 이탈로 잡히지 않아,
# 아무 경고 없이 F 를 지나 벽까지 갔다.
# "지금 읽은 QR이 어느 간선의 것인가"로 보면 즉시 잡힌다 -- (11,5)는 E-F
# 간선 QR인데 경로가 BL->E->C 면 그 간선은 경로에 없다.
# 8/20 개정 -- 반대편 벽 좌표까지 등록.
# EDGES 는 한쪽 벽만 담고 있어, 카메라가 반대편 벽을 볼 때 읽는 좌표가
# 전부 "미등록 = 판단 불가"로 무시됐다. 실측에서 (10,7)/(9,5)/(8,5) 를
# 읽으며 E-F, C-E 복도를 지나가는데도 경고 하나 없이 진행됐다.
_OPP_ROW = {0: 1, 1: 0, 5: 6, 6: 5, 11: 10, 10: 11}
_OPP_COL = {3: 5, 5: 3, 7: 9, 9: 7}

_QR_TO_EDGE = {}
for _e, _qrs in EDGES.items():
    _a, _b = _e
    _ca, _cb = NODES[_a], NODES[_b]
    _ax = "row" if abs(_ca[0] - _cb[0]) > abs(_ca[1] - _cb[1]) else "col"
    for _q in _qrs:
        _r, _c = int(_q[0]), int(_q[1])
        _both = [(_r, _c)]
        if _ax == "col":          # 가로 복도 -- 반대편 벽은 다른 '행'
            _both.append((_OPP_ROW.get(_r, _r), _c))
        else:                     # 세로 복도 -- 반대편 벽은 다른 '열'
            _both.append((_r, _OPP_COL.get(_c, _c)))
        for _k in _both:
            _QR_TO_EDGE.setdefault(_k, set()).add(frozenset(_e))

# 노드 바로 근처에서는 판정하지 않는다. 교차로를 지나는 순간 다음 간선의
# 첫 QR이 잠깐 보일 수 있어(예: E 통과 중 (11,5)) 오탐이 나기 때문이다.
# 한 칸 정도 여유를 준다.
OFF_ROUTE_EDGE_MIN_DIST = 2.0
# 한 번의 오인식으로 멈추지 않도록 연속 확인을 요구한다.
OFF_ROUTE_EDGE_HITS = 2


def check_off_route_edge(coords, ser):
    """
    읽은 QR 이 현재 경로에 없는 간선의 것이면, 그 자리에서 목적지까지
    새 경로를 계산해 주행을 이어간다.

    8/20 개정 -- 기존에는 정지만 했다. "U턴 없이는 되돌아갈 수 없다"는
    이유였는데, 실제 이탈은 방향이 뒤집힌 게 아니라 다른 복도로 들어간
    경우다. 지금 향하는 끝점에서 다시 길을 찾으면 U턴 없이 실행 가능한
    경로가 나온다. 진행 방위를 가상 구간(head_leg)으로 넣어야 새 경로의
    첫 교차로 회전각이 맞는다.
    """

    with nav.lock:
        if nav.mode not in ("NAVIGATE", "RETURN"):
            nav._off_route_hits = 0
            return False
        # 8/20 재조정 -- 억제 범위를 좁힌다.
        # 기존 "VIRTUAL_START in nav.node_route" 는 복귀 구간 전체에서
        # 재탐색을 꺼버렸다. 실측에서 A1 복귀 중 로봇이 TL 이 아니라 TR
        # 쪽으로 갔는데도 경고 한 번 없이 그대로 진행됐다.
        # 막으려던 것은 "복귀 시작 직후 방금 읽은 정지 QR 을 다시 읽는 것"
        # 뿐이므로, 가상 출발 좌표 근처에서만 판정을 미룬다.
        vstart = nav.virtual_start_coord
        if vstart is not None and distance(coords, vstart) < 2.0:
            nav._off_route_hits = 0
            return False
        route = list(nav.node_route)

    key = (int(round(coords[0])), int(round(coords[1])))
    edges_here = _QR_TO_EDGE.get(key)
    if not edges_here:
        return False   # EDGES 에도 반대편 벽에도 없는 좌표 -- 판단 불가

    with nav.lock:
        if nav.mode not in ("NAVIGATE", "RETURN"):
            nav._off_route_hits = 0
            return False
        route = list(nav.node_route)
        mode_now = nav.mode
        zone_backup = nav.zone_code
        zone_entry_backup = nav.zone_entry_node
        prev_coords = nav._off_route_prev

    if len(route) < 2:
        return False

    route_edges = {frozenset((route[i], route[i + 1])) for i in range(len(route) - 1)}
    if edges_here & route_edges:
        with nav.lock:
            nav._off_route_hits = 0
        return False   # 경로상의 간선 -- 정상

    # 노드 근처면 통과 순간의 스침일 수 있으므로 보류
    for name in route:
        nc = node_coord(name)
        if nc is not None and distance(coords, nc) < OFF_ROUTE_EDGE_MIN_DIST:
            return False

    with nav.lock:
        nav._off_route_hits += 1
        hits = nav._off_route_hits
    if hits < OFF_ROUTE_EDGE_HITS:
        return False

    edge = next(iter(edges_here))
    edge_name = " - ".join(sorted(edge))
    goal = route[-1]

    head, came, how = edge_heading_nodes(edge, coords, prev_coords, goal)
    new_path = find_path_no_reverse(head, came, goal)

    ser.write(b"CMD:s\n")
    with nav.lock:
        nav._off_route_hits = 0

    if not new_path:
        # 8/21 개정 -- 막다른 지점을 향해 이탈한 경우 스스로 빠져나오게 한다.
        # 실측(B3 출동)에서 B 를 지나쳐 TR 쪽으로 갔는데, TR 은 이웃이 B 하나뿐
        # 이라 "되돌아가지 않는 경로"가 존재할 수 없어 복도 한가운데 멈춰 섰다.
        # 그런데 TR 은 끝라인 + 연장선이 있는 막다른 지점이라 U턴이 가능하다.
        # 그대로 끝점까지 간 뒤 도착 처리(EXIT_HALT/DEADEND_HALT)에 넘기면
        # 2초 정지 -> U턴 -> 맵 안쪽을 향해 대기까지 자동으로 이어진다.
        # 이번 임무는 포기하지만, 사람이 로봇을 집어 옮기지 않아도 다음
        # gas_target 을 바로 받을 수 있는 자세로 복귀한다.
        if is_dead_end(head):
            cur_heading = heading_between(NODES[came], NODES[head])
            seg = EDGES.get((came, head)) or list(reversed(EDGES.get((head, came), [])))
            axis = edge_axis(came, head)
            publish_mark(f"[경로 이탈] QR {key} 는 '{edge_name}' 간선 -- '{goal}' 로 가는 "
                         f"경로 없음. '{head}' 는 막다른 지점이므로 거기까지 가서 "
                         f"U턴 후 대기한다 (이번 임무 포기)")
            with nav.lock:
                nav.current_node = None
                nav.zone_code = None
                nav.zone_entry_node = None
            _start_route([head], mode="RETURN",
                         head_leg=(cur_heading, list(seg), axis, coords))
            return True

        publish_mark(f"[경로 이탈] QR {key} 는 '{edge_name}' 간선 -- '{head}' 에서 "
                     f"'{goal}' 까지 되돌아가지 않는 경로가 없음. 정지, 수동 개입 필요")
        with nav.lock:
            nav.mode = "IDLE"
        return True

    cur_heading = heading_between(NODES[came], NODES[head])
    seg = EDGES.get((came, head)) or list(reversed(EDGES.get((head, came), [])))
    axis = edge_axis(came, head)

    publish_mark(f"[경로 이탈 -> 재탐색] QR {key} 는 '{edge_name}' 간선. "
                 f"진행 방향 판정({how}) -> '{head}' 쪽. "
                 f"새 경로: {' -> '.join(new_path)} (목표 {goal})")

    with nav.lock:
        nav.current_node = None
    _start_route(new_path, mode=mode_now,
                 head_leg=(cur_heading, list(seg), axis, coords))
    with nav.lock:
        nav.zone_code = zone_backup
        nav.zone_entry_node = zone_entry_backup
    return True


def check_off_route_and_reroute(coords, ser):
    """경로 이탈이 확인되면 정지 -> 재탐색 -> 재개. 실제로 재탐색했으면 True."""
    with nav.lock:
        if nav.mode not in ("NAVIGATE", "RETURN"):
            return False
        expected_nodes = set(nav.node_route[max(0, nav.node_idx - 1):]) if nav.node_route else set()
        final_target = nav.node_route[-1] if nav.node_route else None
        mode_now = nav.mode
        zone_code_backup = nav.zone_code
        last_reroute = nav._last_reroute_time

    if final_target is None:
        return False
    if time.time() - last_reroute < REROUTE_COOLDOWN_SEC:
        return False

    detected_node = nearest_node_to_coord(coords, max_dist=OFF_ROUTE_NODE_RADIUS)
    if detected_node is None or detected_node in expected_nodes:
        return False   # 기대하는 범위 안 -- 정상

    publish_mark(f"[재탐색] 예상 밖 위치 감지: '{detected_node}' 근처 -- 정지하고 경로 재계산")
    ser.write(b"CMD:s\n")
    time.sleep(0.3)

    new_path = find_path(detected_node, final_target)
    with nav.lock:
        nav._last_reroute_time = time.time()

    if not new_path:
        publish_mark(f"[재탐색] '{detected_node}' -> '{final_target}' 경로 없음 -- 정지 유지, 수동 개입 필요")
        return True

    _start_route(new_path, mode=mode_now)
    with nav.lock:
        nav.zone_code = zone_code_backup
        nav.current_node = detected_node
    publish_mark(f"[재탐색] 새 경로: {' -> '.join(new_path)} -- 재개")
    ser.write(b"CMD:g\n")
    return True


# 같은 노드에서 너무 오래(정체) 벗어나지 못하면 완전 정지.
STUCK_TIMEOUT_SEC = 25.0


def check_stuck_and_stop(ser):
    """너무 오래 같은 노드에 머물러 있으면 True 반환하며 완전 정지시킴."""
    with nav.lock:
        if nav.mode not in ("NAVIGATE", "RETURN"):
            return False
        stuck_since = nav._node_idx_stuck_since
        cur = nav.current_node
    if stuck_since == 0:
        return False
    if time.time() - stuck_since > STUCK_TIMEOUT_SEC:
        ser.write(b"CMD:s\n")
        publish_mark(f"[정체 감지] '{cur}' 에서 {STUCK_TIMEOUT_SEC:.0f}초 넘게 진전 없음 -- "
                     f"완전 정지, 수동 개입 필요")
        with nav.lock:
            nav.mode = "IDLE"
        return True
    return False


# ── 막다른 지점 / 되돌아오지 않는 경로 탐색 (8/20 추가) ────────
# ML, MR 은 출구가 아니다. 라인은 다른 출입구와 같은 형태(끝라인 + 짧은
# 연장 직선)로 깔려 있지만 임무 종료 지점으로 쓰지 않는다. 다만 연장선이
# 있으므로 U턴은 가능하다 -- 이게 중요하다.
#
# 새 U턴 시퀀스(TURN_U_CLEAR)는 연장선 끝의 000 을 기다리므로, 라인이
# 계속 이어지는 복도 한가운데서는 성립하지 않는다. 따라서 U턴은 반드시
# 막다른 지점(연결된 이웃이 하나뿐인 노드)에서만 시도해야 한다.
def _adjacent(node):
    out = set()
    for (a, b) in EDGES:
        if a == node:
            out.add(b)
        elif b == node:
            out.add(a)
    return out


def is_dead_end(node):
    """이웃이 하나뿐 -- 끝라인 + 연장선이 있어 U턴이 가능한 지점."""
    return len(_adjacent(node)) <= 1


def find_exit_without_reversing(start, came_from):
    """
    start 에서 출구까지의 최단 경로를 찾되, 첫 걸음이 came_from(방금 온 쪽)
    으로 되돌아가는 경로는 배제한다.

    교차점에는 연장선이 없어 제자리 180도를 할 수 없으므로, 도착 직후
    온 길로 되돌아가는 경로는 물리적으로 실행 불가능하다. 그런 경로를
    배제한 최단 출구를 반환하고, 아예 없으면(막다른 지점) None.
    start 자체가 출구면 [start].
    """
    from collections import deque
    if start in EXIT_NODES:
        return [start]
    visited = {start}
    queue = deque()
    for nxt in sorted(_adjacent(start)):
        if nxt == came_from:
            continue
        visited.add(nxt)
        queue.append([start, nxt])
    while queue:
        path = queue.popleft()
        node = path[-1]
        if node in EXIT_NODES:
            return path
        for nxt in sorted(_adjacent(node)):
            if nxt not in visited:
                visited.add(nxt)
                queue.append(path + [nxt])
    return None


def find_path_no_reverse(start, came_from, goal):
    """start 에서 goal 까지, 첫 걸음이 came_from 으로 되돌아가지 않는 최단 경로."""
    from collections import deque
    if start == goal:
        return [start]
    visited = {start}
    q = deque()
    for nxt in sorted(_adjacent(start)):
        if nxt == came_from:
            continue
        if nxt == goal:
            return [start, nxt]
        visited.add(nxt)
        q.append([start, nxt])
    while q:
        path = q.popleft()
        for nxt in sorted(_adjacent(path[-1])):
            if nxt == goal:
                return path + [nxt]
            if nxt not in visited:
                visited.add(nxt)
                q.append(path + [nxt])
    return None


def edge_heading_nodes(edge, coords, prev_coords, goal):
    """
    이탈한 간선 위에서 (향하는 끝점, 온 쪽 끝점, 판정근거) 를 고른다.
    직전에 읽은 다른 QR 과 비교해 좌표가 어느 쪽으로 움직였는지로 판정하고,
    비교할 좌표가 없으면 목적지까지 짧은 쪽으로 추정한다.
    """
    a, b = sorted(edge)
    ca, cb = NODES[a], NODES[b]
    ax = "row" if abs(ca[0] - cb[0]) > abs(ca[1] - cb[1]) else "col"
    i = 0 if ax == "row" else 1
    if prev_coords is not None and abs(coords[i] - prev_coords[i]) >= 0.5:
        increasing = coords[i] > prev_coords[i]
        if increasing:
            head = a if ca[i] > cb[i] else b
        else:
            head = a if ca[i] < cb[i] else b
        return head, (b if head == a else a), "직전 QR 이동 방향"
    pa = find_path_no_reverse(a, b, goal)
    pb = find_path_no_reverse(b, a, goal)
    head = a if (len(pa) if pa else 99) <= (len(pb) if pb else 99) else b
    return head, (b if head == a else a), "목적지까지 짧은 쪽(추정)"


def _zone_segment(zone_code, entry_node):
    """구역의 QR 좌표 전체를 entry_node -> 반대편 끝점 순서로 반환."""
    s, e = ZONES[zone_code]
    if entry_node not in (s, e):
        raise ValueError(f"구역 {zone_code} 의 진입 노드가 아님: {entry_node!r} (가능: {s}/{e})")
    other = e if entry_node == s else s
    seg = EDGES.get((entry_node, other)) or list(reversed(EDGES.get((other, entry_node), [])))
    if not seg:
        raise ValueError(f"구역 {zone_code} ({entry_node}->{other}) 의 QR 경로가 정의되어 있지 않음")
    return list(seg), other


def plan_zone_return(zone_code, entry_node):
    """
    구역 정지 지점에서의 복귀 계획. 8/20 전면 개정 -- 복도 한가운데 U턴을
    없앴다(연장선이 없어 물리적으로 불가능). 항상 전진해서 반대편 끝점까지
    간 뒤, 거기서 되돌아오지 않고 갈 수 있는 출구로 향한다. 그런 출구가
    없으면 그 끝점이 막다른 지점(ML/MR 등)이라는 뜻이므로, 일단 거기까지만
    가고 도착 후 U턴 시퀀스를 돌린다.

    반환 dict:
      head    : 복귀 시작 시점의 절대 방위 (= 구역 진행 방위 그대로)
      qr      : 정지 지점 -> 반대편 끝점 사이에서 볼 QR 좌표 순서
      axis    : 그 구간의 축
      stop    : 정지 지점 좌표 (가상 노드 _ZMID 의 좌표)
      path    : 반대편 끝점부터의 노드 경로
      exit    : 목표 출구 (막다른 지점이면 None -- 도착 후 재계획)
      deadend : 도착 지점에서 U턴이 필요한가
    """
    seg, other = _zone_segment(zone_code, entry_node)
    mid = len(seg) // 2
    stop = seg[mid]
    axis = edge_axis(entry_node, other)
    zheading = heading_between(NODES[entry_node], NODES[other])

    path = find_exit_without_reversing(other, entry_node)
    if path:
        return dict(head=zheading, qr=seg[mid:], axis=axis, stop=stop,
                    path=path, exit=path[-1], deadend=False)

    # 되돌아오지 않고는 출구에 못 감 -- 막다른 지점이다.
    # 일단 그 끝점까지만 가고, 도착 후 U턴 시퀀스를 돌린 뒤 재계획한다.
    return dict(head=zheading, qr=seg[mid:], axis=axis, stop=stop,
                path=[other], exit=None, deadend=True)


def _start_route(node_route, mode="NAVIGATE", head_leg=None):
    """
    새 경로로 주행을 시작하도록 nav 상태를 설정.

    head_leg (8/20 추가):
        None 이거나 (heading_deg, qr_coords, axis, start_coord) 튜플.
        "로봇이 node_route[0] 을 아직 안 지났고, 심지어 그 반대 방향을 보고
        있는" 상황 -- 즉 구역 판별 정지 후 복귀 -- 를 표현한다.
        가상 노드 _ZMID 를 경로 맨 앞에 붙이고 그 구간의 방위를 직접 준다.
        이렇게 하면 leg_headings 가 한 칸 앞으로 밀려서, 진입 노드 교차로에서
        쓰이는 회전각이 (진입노드->다음노드) - (복귀 시작 방위) 로 정확히
        계산된다. 예: B2 구역을 C 에서 진입했다가 복귀할 때,
        legs = [180(서쪽), -90(C->A), 172.9(A->TL)] 이므로 C 교차로에서
        tvec = -90 - 180 = -270 -> 정규화 +90 -> 우회전. 서진 중 북쪽으로
        꺾는 것이므로 물리적으로 맞다.
    """
    legs = build_leg_headings(node_route)
    coords, axes = get_route(node_route)
    route = list(node_route)
    vstart_coord = None

    if head_leg is not None:
        h, hcoords, haxis, vstart_coord = head_leg
        legs = [h] + legs
        coords = list(hcoords) + list(coords)
        axes = [haxis] * len(hcoords) + list(axes)
        route = [VIRTUAL_START] + route

    with nav.lock:
        nav.virtual_start_coord = vstart_coord
        nav.node_route = route
        nav.leg_headings = legs
        nav.route_coords = coords
        nav.route_axes = axes
        nav.current_leg = 0
        nav.node_idx = 0
        nav.route_wp_idx = 0
        nav.esp32_crossing_count = 0   # 새 경로 시작 시 교차로 카운터 초기화
        nav._leg_lag_since = 0.0
        nav._node_idx_stuck_since = time.time()
        # 새 경로 시작 시 낡은 QR 좌표를 반드시 비운다. 안 비우면 이전
        # 세션에서 남은 좌표가 NEAR 계산에 잘못 쓰여, 아직 QR을 하나도 못
        # 읽었는데 "다음 노드 근처"로 오판해 출발 지점에서 가짜 교차로 회전이
        # 실행되는 사고로 이어졌다.
        nav.last_qr_coords = None
        nav._off_route_prev = None
        nav._zone_entry_wait_since = 0.0
        nav._near_latch_leg = None   # 8/21: 새 경로 시작 시 NEAR 래치 해제
        nav.mode = mode

    # 8/20 추가 -- 여기서 반드시 ESP32를 실제로 출발(CMD:g)시켜야 한다.
    # 이전에는 IDLE에서 gas_target/goto로 처음 출발할 때만 이게 빠져있었다:
    # 라파 쪽 mode는 NAVIGATE로 바뀌고 로그도 찍히지만, ESP32는 INIT 종료
    # 때 받은 CMD:s 이후 STOPPED에 그대로 머물러 있어서 실제로는 안 움직였다
    # (다른 호출부인 ZONE_STOP 복귀/U턴 후 재출발는 원래도 CMD:g를 따로
    # 보내고 있었어서 그쪽은 문제없었음 — 이제 여기 한 곳에서 전부 커버).
    if esp32_serial is not None:
        # 8/21 추가 -- CMD:g 바로 앞에 POS 를 한 줄 먼저 보낸다.
        # ESP32 의 pollRpiSerial() 은 한 루프에서 버퍼에 쌓인 줄을 순서대로
        # 다 읽으므로, 이 두 줄은 같은 루프에서 처리된다. POS 가 먼저
        # lastRpiMsgTime 을 갱신한 뒤 CMD:g 가 state 를 RUNNING 으로 만들기
        # 때문에, 그 직후의 스트림 타임아웃 검사(2초)에 걸리지 않는다.
        # IDLE 하트비트(위)와 중복이지만, 호출 경로가 여럿이라 여기서
        # 한 번 더 확실히 막아둔다. TVEC:0.0 이라 출발 지점에서 회전이
        # 걸릴 일은 없다.
        first_pos = route[0] if route else "?"
        esp32_serial.write(f"POS:{first_pos},TVEC:0.0,NEAR:0\n".encode("utf-8"))
        esp32_serial.write(b"CMD:g\n")

    print(f"=== {mode} 시작: {' -> '.join(route)} ===")


if __name__ == "__main__":
    main()