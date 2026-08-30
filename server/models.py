# models.py => 통신때 전달되는 데이터 타입을 정의 (ER 설계 기준)
# MQTT로 들어온 payload를 저장 전에 검증하는 용도로 사용합니다.
# mqtt는 dict로 오므로 필드는 모두 Optional (없는 값은 None으로 취급).
#
# 원본 대비 수정: val_mq2_left 등 "val_" 접두사를 제거했습니다.
# database.py가 payload["mq2_left"] 형태로 읽기 때문에, 접두사가 있으면
# 실제 저장 시 KeyError로 이어집니다. 필드명은 DB 컬럼명과 그대로 맞췄습니다.
#
# 8/20 수정: 로봇 가스 센서 구성이 MQ2(좌/우)/MQ3/MQ135/MQ138(라파 ADC,
# MCP3008)로 바뀌어 mq6를 빼고 mq138을 추가했습니다.

from pydantic import BaseModel
from typing import Optional
from datetime import datetime


# 로봇 감지 이벤트 payload (topic: robot/#)
# -> robot_event + robot_sensor (+ 판별 시 robot_gas_result)
class RobotEventData(BaseModel):
    location: Optional[str] = None      # QR 좌표
    event_type: Optional[str] = None    # 'patrol' / 'gas_response'
    # robot_sensor 원시값
    mq2_left: Optional[float] = None
    mq2_right: Optional[float] = None
    mq3: Optional[float] = None
    mq135: Optional[float] = None
    mq138: Optional[float] = None
    temperature: Optional[float] = None
    humidity: Optional[float] = None
    # robot_gas_result (판별 결과, 있을 때만)
    gas_type: Optional[str] = None
    confidence: Optional[float] = None
    strength: Optional[float] = None
    alert_level: Optional[str] = None


# 고정 구역 센서 측정값 payload (topic: sensor/#, zone_id 포함)
# -> fixed_sensor_reading
class FixedSensorData(BaseModel):
    zone_id: str
    rs_value: Optional[float] = None
    strength: Optional[float] = None
    status: Optional[str] = None
    rs_baseline: Optional[float] = None  # baseline 초기화용(선택)


# 화재 이벤트 payload (topic: fire/#) -> fire_event
class FireEventData(BaseModel):
    location: Optional[str] = None
    source: Optional[str] = None
    flame_detected: Optional[bool] = None   # YOLO
    smoke_detected: Optional[bool] = None   # MQ2
    confidence: Optional[float] = None
    alert_level: Optional[str] = None
    event_time: Optional[datetime] = None
