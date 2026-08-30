# main.py — SafeScout DB 조회용 FastAPI 서버
#
# 웹(React)·앱(Android)에서 REST API로 접근할 수 있게 6개 테이블에 대한
# 조회 엔드포인트를 제공합니다. 지금은 테스트 단계라 SELECT(조회)만 두고,
# 실제 서비스에서는 로봇·센서가 데이터를 넣는 POST 엔드포인트도 추가하면 됩니다.
#
# 실행: uvicorn main:app --host 0.0.0.0 --port 8000

import asyncio
from contextlib import asynccontextmanager
from datetime import datetime, date
from decimal import Decimal

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

import config
import mqtt_client
import auth
from database import fetch_all, fetch_one, create_instruction
from evacuation import compute_route
from pydantic import BaseModel

# ---------------------------------------------------------------- WebSocket 연결 관리
connected_clients: set[WebSocket] = set()
main_loop = None  # startup 시점의 asyncio 이벤트 루프를 저장해둡니다


def on_mqtt_realtime(topic: str, payload: dict):
    """MQTT 콜백(별도 스레드)에서 호출됩니다.
    asyncio 루프는 스레드가 다르므로 run_coroutine_threadsafe로 안전하게 넘겨줍니다.
    payload의 strength는 publisher 쪽에서 이미 0~100으로 변환되어 오므로 그대로 전달합니다."""
    if main_loop is None:
        return
    asyncio.run_coroutine_threadsafe(broadcast_to_web(topic, payload), main_loop)


async def broadcast_to_web(topic: str, payload: dict):
    dead = set()
    for ws in connected_clients:
        try:
            await ws.send_json({"topic": topic, "data": payload})
        except Exception:
            dead.add(ws)
    connected_clients.difference_update(dead)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global main_loop
    main_loop = asyncio.get_running_loop()

    mqtt_client.set_realtime_callback(on_mqtt_realtime)
    client = mqtt_client.start_mqtt()
    print("[server] MQTT 클라이언트 시작됨")

    yield

    client.loop_stop()
    print("[server] 종료")


app = FastAPI(
    title="SafeScout API",
    description="가스·화재 감지 데이터 조회 서버",
    lifespan=lifespan,
)

# 웹·앱에서 다른 포트(도메인)로 요청해도 막히지 않도록 CORS 허용
app.add_middleware(
    CORSMiddleware,
    allow_origins=config.CORS_ORIGINS,
    allow_methods=["*"],
    allow_headers=["*"],
)


def serialize(row: dict) -> dict:
    """datetime, Decimal 등 JSON으로 바로 안 바뀌는 타입을 문자열/숫자로 변환합니다.
    strength 값은 publisher(아두이노/라즈베리파이) 쪽에서 이미 0~100으로 변환해
    보내오므로, 서버에서는 추가 변환 없이 그대로 전달합니다."""
    result = {}
    for key, value in row.items():
        if isinstance(value, (datetime, date)):
            result[key] = value.isoformat()
        elif isinstance(value, Decimal):
            result[key] = float(value)
        else:
            result[key] = value
    return result


def serialize_all(rows: list[dict]) -> list[dict]:
    return [serialize(r) for r in rows]


# ---------------------------------------------------------------- 상태 확인
@app.get("/health")
def health():
    """DB 연결 자체가 되는지 확인하는 용도."""
    try:
        fetch_one("SELECT 1 AS ok")
        return {"db": "connected"}
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"DB 연결 실패: {e}")


class AuthRequest(BaseModel):
    username: str
    password: str
    role: str = "staff"   # 'admin'(관리자 웹) 또는 'staff'(직원 앱). 기본은 직원.


@app.post("/api/auth/register")
def register(req: AuthRequest):
    """회원가입. 웹·앱 모두 이 엔드포인트를 그대로 씁니다 (role로만 구분).
    username이 이미 있으면 에러."""
    if req.role not in ("admin", "staff"):
        raise HTTPException(status_code=400, detail="role은 admin 또는 staff여야 합니다")

    existing = fetch_one("SELECT id FROM users WHERE username = %s", (req.username,))
    if existing:
        raise HTTPException(status_code=400, detail="이미 사용 중인 아이디입니다")

    user_id = auth.create_user(req.username, req.password, req.role)
    return {"id": user_id, "username": req.username, "role": req.role}


@app.post("/api/auth/login")
def login(req: AuthRequest):
    """로그인. 성공하면 사용자 정보를 그대로 돌려줍니다 (토큰 없음, 최소 버전)."""
    user = auth.verify_user(req.username, req.password)
    if not user:
        raise HTTPException(status_code=401, detail="아이디 또는 비밀번호가 틀렸습니다")
    return user


@app.websocket("/ws/realtime")
async def websocket_realtime(websocket: WebSocket):
    """웹이 이 엔드포인트에 연결해두면, MQTT로 새 데이터가 들어올 때마다
    {"topic": ..., "data": {...}} 형태로 즉시 전달받습니다."""
    await websocket.accept()
    connected_clients.add(websocket)
    print(f"[ws] 클라이언트 연결 (현재 {len(connected_clients)}명)")
    try:
        while True:
            await websocket.receive_text()  # 연결 유지 (핑퐁용, 내용은 무시)
    except WebSocketDisconnect:
        connected_clients.discard(websocket)
        print(f"[ws] 클라이언트 연결 종료 (현재 {len(connected_clients)}명)")


# ---------------------------------------------------------------- 1. robot_event
@app.get("/api/robot-events")
def get_robot_events(limit: int = Query(50, le=500)):
    """최근 로봇 이벤트 목록. 최신순."""
    rows = fetch_all(
        "SELECT * FROM robot_event ORDER BY event_time DESC LIMIT %s", (limit,)
    )
    return serialize_all(rows)


@app.get("/api/robot-events/{event_id}")
def get_robot_event_detail(event_id: int):
    """이벤트 하나의 상세 정보. 센서값·가스판별 결과를 함께 묶어서 반환."""
    event = fetch_one("SELECT * FROM robot_event WHERE id = %s", (event_id,))
    if not event:
        raise HTTPException(status_code=404, detail="해당 이벤트가 없습니다")

    sensor = fetch_one("SELECT * FROM robot_sensor WHERE event_id = %s", (event_id,))
    gas_result = fetch_one(
        "SELECT * FROM robot_gas_result WHERE event_id = %s", (event_id,)
    )

    return {
        "event": serialize(event),
        "sensor": serialize(sensor) if sensor else None,
        "gas_result": serialize(gas_result) if gas_result else None,
    }


# ---------------------------------------------------------------- 2. fixed_sensor
@app.get("/api/fixed-sensor/latest")
def get_fixed_sensor_latest():
    """구역별 가장 최근 측정값. 관제 웹 지도 표시용."""
    rows = fetch_all("""
        SELECT r.*
        FROM fixed_sensor_reading r
        INNER JOIN (
            SELECT zone_id, MAX(reading_time) AS max_time
            FROM fixed_sensor_reading
            GROUP BY zone_id
        ) latest
        ON r.zone_id = latest.zone_id AND r.reading_time = latest.max_time
    """)
    return serialize_all(rows)


@app.get("/api/fixed-sensor/{zone_id}/history")
def get_fixed_sensor_history(zone_id: str, limit: int = Query(100, le=1000)):
    """특정 구역의 측정 이력."""
    rows = fetch_all(
        "SELECT * FROM fixed_sensor_reading WHERE zone_id = %s "
        "ORDER BY reading_time DESC LIMIT %s",
        (zone_id, limit),
    )
    return serialize_all(rows)


@app.get("/api/fixed-sensor/baseline")
def get_fixed_sensor_baseline():
    """구역별 기준값 목록."""
    rows = fetch_all("SELECT * FROM fixed_sensor_baseline")
    return serialize_all(rows)


# ---------------------------------------------------------------- 3. fire_event
@app.get("/api/fire-events")
def get_fire_events(limit: int = Query(50, le=500)):
    """최근 화재 감지 이벤트."""
    rows = fetch_all(
        "SELECT * FROM fire_event ORDER BY event_time DESC LIMIT %s", (limit,)
    )
    return serialize_all(rows)


# ---------------------------------------------------------------- 4. 직원 지시사항
class InstructionCreate(BaseModel):
    event_kind: str | None = None    # 예: 'gas' 또는 'fire'
    event_label: str | None = None    # 예: 'B구역 가스 감지' 같은 설명 텍스트
    instruction: str                    # 지시 내용 (필수)


@app.post("/api/instructions")
async def post_instruction(req: InstructionCreate):
    """지시사항 등록 (관리자 웹에서 호출). 확인 여부는 추적하지 않습니다.
    저장 직후 WebSocket으로 접속 중인 모든 클라이언트(직원 앱 포함)에 즉시 알립니다.
    이 함수는 일반 HTTP 요청 처리 중(=이미 같은 asyncio 이벤트 루프 안)이므로,
    MQTT 콜백 때와 달리 run_coroutine_threadsafe 없이 바로 await하면 됩니다."""
    instruction_id = create_instruction(
        req.event_kind, req.event_label, req.instruction
    )

    payload = {
        "id": instruction_id,
        "event_kind": req.event_kind,
        "event_label": req.event_label,
        "instruction": req.instruction,
    }
    await broadcast_to_web("instruction/new", payload)

    return {"id": instruction_id}


@app.get("/api/instructions")
def get_instructions(limit: int = Query(50, le=500)):
    """지시사항 목록 (직원 앱에서 호출). 최신순."""
    rows = fetch_all(
        "SELECT * FROM staff_instruction ORDER BY created_at DESC LIMIT %s", (limit,)
    )
    return serialize_all(rows)


# ---------------------------------------------------------------- 5. 통합 알림 (최근 이벤트 요약)
# ---------------------------------------------------------------- 6. 대피 경로
# 8/20 -- evacuation.py로 완성. 로봇이 쓰는 지도(map_data.py: TL/TR/BL/BR/
# ML/MR/A~F 노드)를 그대로 써서, 현재 위험(gas) 상태인 구역의 간선을 피해
# 다익스트라로 가장 가까운 출구까지의 경로를 계산한다.
@app.get("/api/evacuation-route")
def get_evacuation_route(current_location: str):
    """현재 위치(map_data 노드 이름 예: "C", 또는 "(row,col)" QR 좌표 문자열)를
    기준으로, 지금 위험 상태인 구역을 피해 가장 가까운 출구까지의 경로를
    반환합니다. 경로는 map_data.py 노드 이름의 리스트입니다."""
    danger_rows = fetch_all("""
        SELECT r.zone_id
        FROM fixed_sensor_reading r
        INNER JOIN (
            SELECT zone_id, MAX(reading_time) AS max_time
            FROM fixed_sensor_reading
            GROUP BY zone_id
        ) latest
        ON r.zone_id = latest.zone_id AND r.reading_time = latest.max_time
        WHERE r.status = '위험'
    """)
    danger_zones = [row["zone_id"] for row in danger_rows]

    try:
        route = compute_route(current_location, danger_zones)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    if route is None:
        raise HTTPException(status_code=404, detail="현재 위치에서 갈 수 있는 출구 경로가 없습니다")

    return {"route": route, "blocked_zones": danger_zones}


@app.get("/api/alerts/recent")
def get_recent_alerts(limit: int = Query(20, le=200)):
    """가스·화재 이벤트를 시간순으로 합쳐서 보여주는 통합 알림 목록.
    관제 웹의 '최근 알림' 화면에서 바로 쓰기 좋은 형태입니다."""

    gas_alerts = fetch_all("""
        SELECT
            'gas' AS type,
            re.location,
            gr.gas_type AS detail,
            gr.alert_level,
            re.event_time
        FROM robot_event re
        JOIN robot_gas_result gr ON re.id = gr.event_id
        ORDER BY re.event_time DESC
        LIMIT %s
    """, (limit,))

    fire_alerts = fetch_all("""
        SELECT
            'fire' AS type,
            location,
            source AS detail,
            alert_level,
            event_time
        FROM fire_event
        ORDER BY event_time DESC
        LIMIT %s
    """, (limit,))

    combined = gas_alerts + fire_alerts
    combined.sort(key=lambda r: r["event_time"], reverse=True)
    return serialize_all(combined[:limit])


# ---------------------------------------------------------------- 프론트엔드 정적 파일
# 반드시 맨 마지막에 등록합니다 — "/"에 마운트하지만 위의 /api/*, /ws/* 라우트가
# 먼저 등록돼 있어서 그쪽이 항상 우선 매칭되고, 나머지 모든 경로(프론트 라우팅 포함)만
# frontend_dist/index.html로 떨어집니다(html=True가 SPA 새로고침 시 404 방지).
app.mount("/", StaticFiles(directory="frontend_dist", html=True), name="frontend")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=config.SERVER_HOST, port=config.SERVER_PORT)

