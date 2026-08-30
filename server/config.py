# config.py — DB + MQTT 접속 설정

import os

# ---------------- DB ----------------
DB_HOST = os.environ.get("DB_HOST", "localhost")
DB_PORT = int(os.environ.get("DB_PORT", 3306))
DB_USER = os.environ.get("DB_USER", "safe_app")
DB_PASSWORD = os.environ.get("DB_PASSWORD", "changeme")  # 실제 값은 배포 서버 환경변수로 설정
DB_NAME = os.environ.get("DB_NAME", "safeDB")

# ---------------- MQTT ----------------
# 같은 서버에 Mosquitto가 떠 있다는 가정 (localhost). 브로커를 다른 곳에 두면 IP로 변경.
MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", 1883))

# 구독할 토픽 목록: (토픽, QoS)
# payload는 JSON 객체 하나로, 이벤트 하나의 정보를 통째로 담아 보냅니다.
#   robot/event    : {"location": "(3,5)", "event_type": "gas_response",
#                     "mq2_left":.., "mq2_right":.., "mq3":.., "mq6":.., "mq135":..,
#                     "temperature":.., "humidity":..,
#                     "gas_type":.., "confidence":.., "strength":.., "alert_level":..}  (gas_type 이하는 판별 시에만)
#   sensor/reading : {"zone_id": "zone_A", "rs_value":.., "strength":.., "status":".."}
#   fire/event     : {"location":.., "source":.., "flame_detected":.., "smoke_detected":..,
#                      "confidence":.., "alert_level":..}
MQTT_TOPICS = [
    ("robot/event", 0),
    ("sensor/reading", 0),
    ("fire/event", 0),
]

# ---------------- 서버 ----------------
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8080   # 8000은 CCTV 미디어 서버가 사용 중
CORS_ORIGINS = ["*"]
