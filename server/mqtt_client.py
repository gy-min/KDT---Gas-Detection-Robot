# mqtt_client.py -> MQTT로 센서/로봇 데이터를 받아 DB에 저장하는 역할
#
# 원본 대비 수정한 부분:
#   1. paho-mqtt 버전 호환 처리 (1.x/2.x 모두 동작하게)
#   2. on_connect 콜백을 최신 API 시그니처로 (reason_code, properties)
#   3. subscribe에 QoS 반영
#   4. models.py의 Pydantic 모델로 저장 전에 형식을 검증
#      (필드 이름·타입이 잘못된 메시지를 DB까지 보내지 않고 여기서 걸러냄)
#   5. JSON 파싱 실패 / 형식 오류 / DB 저장 실패를 구분해서 에러 로그
#   6. realtime_callback을 스레드 안전하게 호출하도록 함수로 감쌈
#      (MQTT는 별도 스레드, FastAPI의 WebSocket은 asyncio라 그냥 호출하면 안 됨.
#       set_realtime_callback으로 asyncio 루프와 연결된 함수를 주입받습니다)
#   7. 8/20 추가 -- 고정 센서(sensor/reading)가 새로 '위험'으로 전이하면
#      robot/{ROBOT_ID}/gas_target 으로 그 구역코드를 자동 발행해 로봇을 출동시킴.
#      (라파5.txt의 on_mqtt_message가 이 토픽을 구독해서 목표를 받는다)
#   8. 8/20 추가 -- YOLO 화재 감지(fire/event, location이 cctv_zone_map으로
#      zone_A1 형식으로 이미 변환돼서 옴)와 고정 가스 센서(sensor/reading)를
#      구역 단위로 가중 융합해서, 둘 중 하나만으론 위험 기준에 못 미쳐도
#      합산 점수가 높으면 로봇을 출동시킴. 기존 "가스 단독 위험" 트리거는
#      그대로 두고, 이 융합 트리거를 별도로 추가한다(둘 다 작동).

import json
import time

import paho.mqtt.client as mqtt
from pydantic import ValidationError

import config
import database
from models import RobotEventData, FixedSensorData, FireEventData

# main.py에서 set_realtime_callback()으로 주입해줍니다.
# 시그니처: callback(topic: str, payload: dict) -> None  (동기 함수, 내부에서 스레드 안전하게 처리)
_realtime_callback = None


def set_realtime_callback(callback):
    """FastAPI 쪽에서 WebSocket으로 데이터를 흘려보낼 콜백을 등록합니다."""
    global _realtime_callback
    _realtime_callback = callback


# 8/20 추가 -- 로봇 출동 자동 트리거용 상태.
# 같은 구역이 '위험' 상태를 유지하는 동안 매 메시지마다 다시 출동시키지
# 않도록, 직전 상태를 기억해뒀다가 '위험이 아님 -> 위험'으로 바뀔 때만 쏜다.
ROBOT_ID = 1
TOPIC_GAS_TARGET = f"robot/{ROBOT_ID}/gas_target"
_last_zone_status = {}


def _zone_id_to_code(zone_id: str):
    """'zone_A2' -> 'A2'. map_data.py의 ZONES(A1~C3)와 형식을 맞춘다.
    레거시 구역(zone_A/B/C/D, 끝에 숫자 없음)은 로봇 맵에 대응 노드가 없어
    None을 반환해 트리거를 건너뛴다."""
    if not zone_id.lower().startswith("zone_"):
        return None
    code = zone_id[len("zone_"):].upper()
    if len(code) < 2 or not code[-1].isdigit():
        return None
    return code


def _maybe_trigger_robot(zone_id: str, status: str, client):
    prev = _last_zone_status.get(zone_id)
    _last_zone_status[zone_id] = status
    if status != "위험" or prev == "위험":
        return
    code = _zone_id_to_code(zone_id)
    if code is None:
        print(f"[MQTT] 로봇 출동 생략 -- 로봇 맵에 없는 구역: {zone_id}")
        return
    client.publish(TOPIC_GAS_TARGET, code)
    print(f"[MQTT] 고정 센서 위험 감지({zone_id}) -> 로봇 출동: {TOPIC_GAS_TARGET} = {code}")


# 8/20 추가 -- 화재(YOLO confidence 0~1) + 가스(MQ2 strength 0~100) 구역별 융합.
# fused = FIRE_WEIGHT*fire_confidence + GAS_WEIGHT*(gas_strength/100)
# 둘 중 하나가 오래돼서(FUSION_STALENESS_SEC 초과) 최신이 아니면 그 항은 0으로 본다
# (예: 화재 신호만 방금 왔고 그 구역 가스는 한참 전 값이면 가스는 안 섞는다).
FIRE_WEIGHT = 0.4
GAS_WEIGHT = 0.6
FUSION_THRESHOLD = 0.6
FUSION_STALENESS_SEC = 30

_latest_fire = {}    # zone_id -> (confidence 0~1, timestamp)
_latest_gas = {}     # zone_id -> (strength 0~100, timestamp)
_fusion_armed = {}   # zone_id -> bool (재출동 방지용 -- 점수가 임계값 밑으로 내려가야 재무장)


def _compute_fused_score(zone_id: str) -> float:
    now = time.time()
    fire = _latest_fire.get(zone_id)
    gas = _latest_gas.get(zone_id)
    fire_conf = fire[0] if fire and (now - fire[1]) <= FUSION_STALENESS_SEC else 0.0
    gas_strength = gas[0] if gas and (now - gas[1]) <= FUSION_STALENESS_SEC else 0.0
    return FIRE_WEIGHT * fire_conf + GAS_WEIGHT * (gas_strength / 100.0)


def _maybe_trigger_fusion(zone_id: str, client):
    score = _compute_fused_score(zone_id)
    armed = _fusion_armed.get(zone_id, False)
    if score >= FUSION_THRESHOLD:
        if armed:
            return  # 이미 이 이벤트로 출동시킴 -- 점수 유지 중엔 재출동 안 함
        _fusion_armed[zone_id] = True
        code = _zone_id_to_code(zone_id)
        if code is None:
            print(f"[MQTT] 융합 출동 생략 -- 로봇 맵에 없는 구역: {zone_id}")
            return
        client.publish(TOPIC_GAS_TARGET, code)
        print(f"[MQTT] 화재+가스 융합 위험({zone_id}, score={score:.2f} >= {FUSION_THRESHOLD}) "
              f"-> 로봇 출동: {TOPIC_GAS_TARGET} = {code}")
    else:
        _fusion_armed[zone_id] = False


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[MQTT] 연결됨 (code={reason_code})")
    for topic, qos in config.MQTT_TOPICS:
        client.subscribe(topic, qos=qos)
        print(f"[MQTT] 구독: {topic} (QoS {qos})")


def on_message(client, userdata, msg):
    topic = msg.topic

    # 1) JSON 파싱
    try:
        raw = json.loads(msg.payload.decode())
    except json.JSONDecodeError as e:
        print(f"[MQTT] JSON 파싱 실패: {topic} → {msg.payload!r} ({e})")
        return

    # 2) 토픽별로 Pydantic 모델 검증 → dict로 변환 (None 필드는 제외)
    try:
        if topic.startswith("robot/"):
            validated = RobotEventData(**raw).model_dump(exclude_none=True)
            save_fn = database.save_robot_event
        elif topic.startswith("sensor/"):
            validated = FixedSensorData(**raw).model_dump(exclude_none=True)
            save_fn = database.save_fixed_sensor_reading
        elif topic.startswith("fire/"):
            validated = FireEventData(**raw).model_dump(exclude_none=True)
            save_fn = database.save_fire_event
        else:
            print(f"[MQTT] 처리 대상이 아닌 토픽: {topic}")
            return
    except ValidationError as e:
        # 필드 이름이 잘못됐거나 타입이 안 맞는 경우 여기서 걸러집니다.
        print(f"[MQTT] 데이터 형식 오류: {topic} → {raw}\n{e}")
        return

    # 3) DB 저장 (여기서 나는 에러는 "필수값 누락" 등 스키마 레벨 문제)
    try:
        save_fn(validated)
    except Exception as e:
        print(f"[MQTT] DB 저장 실패: {topic} → {validated} ({e})")
        return

    # 4) 웹으로 실시간 전달 (검증·정제된 데이터를 그대로 보냄)
    if _realtime_callback:
        try:
            _realtime_callback(topic, validated)
        except Exception as e:
            print(f"[MQTT] 실시간 전달 실패: {e}")

    # 5) 고정 센서가 새로 '위험'에 진입했으면 로봇 자동 출동 (가스 단독 트리거, 기존 유지)
    if topic.startswith("sensor/") and "zone_id" in validated and "status" in validated:
        try:
            _maybe_trigger_robot(validated["zone_id"], validated["status"], client)
        except Exception as e:
            print(f"[MQTT] 로봇 출동 트리거 실패: {e}")

    # 6) 화재+가스 융합 트리거 (5번과 별개로 항상 같이 돈다)
    try:
        if topic.startswith("sensor/") and "zone_id" in validated:
            zone_id = validated["zone_id"]
            strength = validated.get("strength")
            if strength is not None:
                _latest_gas[zone_id] = (float(strength), time.time())
                _maybe_trigger_fusion(zone_id, client)
        elif topic.startswith("fire/"):
            location = validated.get("location") or ""
            if location.lower().startswith("zone_"):
                confidence = validated.get("confidence")
                if confidence is None:
                    # confidence가 안 왔으면 flame_detected 여부로 대략 채운다
                    confidence = 0.9 if validated.get("flame_detected") else 0.5
                _latest_fire[location] = (float(confidence), time.time())
                _maybe_trigger_fusion(location, client)
    except Exception as e:
        print(f"[MQTT] 화재+가스 융합 트리거 실패: {e}")


def start_mqtt():
    # paho-mqtt 1.x / 2.x 모두 호환
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except (AttributeError, TypeError):
        client = mqtt.Client()

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(config.MQTT_HOST, config.MQTT_PORT, 60)
    client.loop_start()  # 백그라운드 스레드에서 계속 수신

    return client
