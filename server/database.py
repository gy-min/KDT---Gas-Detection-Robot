# database.py — MariaDB 연결 + 저장 함수
#
# MQTT로 들어온 payload(dict)를 우리가 확정한 6테이블 스키마에 맞춰 저장합니다.
#   robot_event(+robot_sensor, 판별 시 robot_gas_result)
#   fixed_sensor_reading
#   fire_event

import pymysql
import pymysql.cursors

import config


def get_connection():
    """DB 커넥션을 하나 엽니다. 사용 후 반드시 close() 하세요."""
    return pymysql.connect(
        host=config.DB_HOST,
        port=config.DB_PORT,
        user=config.DB_USER,
        password=config.DB_PASSWORD,
        database=config.DB_NAME,
        cursorclass=pymysql.cursors.DictCursor,
        charset="utf8mb4",
    )


def fetch_all(query: str, params: tuple = ()) -> list[dict]:
    conn = get_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute(query, params)
            return cursor.fetchall()
    finally:
        conn.close()


def fetch_one(query: str, params: tuple = ()) -> dict | None:
    conn = get_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute(query, params)
            return cursor.fetchone()
    finally:
        conn.close()


# ============================================================
# MQTT 수신 데이터 저장 함수
# ============================================================

def save_robot_event(payload: dict) -> int:
    """robot/event 토픽 메시지를 robot_event + robot_sensor (+ robot_gas_result)에 저장합니다.

    DB 제약상 진짜 필수인 건 location, event_type뿐입니다 (NOT NULL 컬럼).
    센서값(mq2_left/mq2_right/mq3/mq135/mq138)은 DB에서 NULL을 허용하므로, 일부만 와도 저장됩니다.
    location/event_type이 없으면 아래 기본값으로 채웁니다 — 실제 로봇 연동 시에는
    QR 위치·이벤트 종류를 반드시 채워서 보내도록 publisher 쪽을 맞춰야 합니다.
    """
    location = payload.get("location", "(test)")
    event_type = payload.get("event_type", "patrol")

    conn = get_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute(
                "INSERT INTO robot_event (location, event_type) VALUES (%s, %s)",
                (location, event_type),
            )
            event_id = cursor.lastrowid

            cursor.execute(
                """INSERT INTO robot_sensor
                   (event_id, mq2_left, mq2_right, mq3, mq135, mq138, temperature, humidity)
                   VALUES (%s, %s, %s, %s, %s, %s, %s, %s)""",
                (
                    event_id,
                    payload.get("mq2_left"), payload.get("mq2_right"),
                    payload.get("mq3"), payload.get("mq135"), payload.get("mq138"),
                    payload.get("temperature"), payload.get("humidity"),
                ),
            )

            # 가스 판별 결과가 같이 왔으면(로봇이 출동해 AI 판별까지 마친 경우) 함께 저장
            if payload.get("gas_type"):
                cursor.execute(
                    """INSERT INTO robot_gas_result
                       (event_id, gas_type, confidence, strength, alert_level)
                       VALUES (%s, %s, %s, %s, %s)""",
                    (
                        event_id, payload["gas_type"],
                        payload.get("confidence"), payload.get("strength"),
                        payload.get("alert_level"),
                    ),
                )

        conn.commit()
        return event_id
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()


def save_fixed_sensor_reading(payload: dict) -> int:
    """sensor/reading 토픽 메시지를 fixed_sensor_reading에 저장합니다.

    필수 키: zone_id, rs_value
    선택 키: strength, status
    rs_baseline이 함께 오면(기준값 초기화/갱신), fixed_sensor_baseline도 upsert합니다.
    """
    if "zone_id" not in payload or "rs_value" not in payload:
        raise ValueError("sensor/reading payload에 zone_id 또는 rs_value가 없습니다")

    conn = get_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute(
                """INSERT INTO fixed_sensor_reading (zone_id, rs_value, strength, status)
                   VALUES (%s, %s, %s, %s)""",
                (
                    payload["zone_id"], payload["rs_value"],
                    payload.get("strength"), payload.get("status"),
                ),
            )
            reading_id = cursor.lastrowid

            if payload.get("rs_baseline") is not None:
                cursor.execute(
                    """INSERT INTO fixed_sensor_baseline (zone_id, rs_baseline)
                       VALUES (%s, %s)
                       ON DUPLICATE KEY UPDATE rs_baseline = VALUES(rs_baseline)""",
                    (payload["zone_id"], payload["rs_baseline"]),
                )

        conn.commit()
        return reading_id
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()


def save_fire_event(payload: dict) -> int:
    """fire/event 토픽 메시지를 fire_event에 저장합니다.

    필수 키: location, source
    선택 키: flame_detected, smoke_detected, confidence, alert_level
    """
    if "location" not in payload or "source" not in payload:
        raise ValueError("fire/event payload에 location 또는 source가 없습니다")

    conn = get_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute(
                """INSERT INTO fire_event
                   (location, source, flame_detected, smoke_detected, confidence, alert_level)
                   VALUES (%s, %s, %s, %s, %s, %s)""",
                (
                    payload["location"], payload["source"],
                    payload.get("flame_detected", False),
                    payload.get("smoke_detected", False),
                    payload.get("confidence"), payload.get("alert_level"),
                ),
            )
            fire_id = cursor.lastrowid
        conn.commit()
        return fire_id
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()


# ============================================================
# 직원 지시사항 (관리자 웹 -> REST로 직접 호출, MQTT 아님)
# ============================================================

def create_instruction(
    event_kind: str | None,
    event_label: str | None,
    instruction: str,
) -> int:
    """화재/가스 이벤트에 대한 지시사항을 기록합니다.
    확인 여부는 추적하지 않고, "어떤 이벤트에 무슨 지시를 내렸는지"만 남깁니다."""
    conn = get_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute(
                """INSERT INTO staff_instruction
                   (event_kind, event_label, instruction)
                   VALUES (%s, %s, %s)""",
                (event_kind, event_label, instruction),
            )
            instruction_id = cursor.lastrowid
        conn.commit()
        return instruction_id
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()
