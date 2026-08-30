# main.py — IP 카메라(RTSP) 영상을 WebRTC로 브라우저에 직접 전달하는 서버
#
# 흐름:  IP 카메라 ──(RTSP)──▶ FastAPI + aiortc ──(WebRTC / WHEP)──▶ 브라우저 <video>
#
# MediaMTX 없이 이 서버가 WHEP 엔드포인트 역할을 직접 합니다.
#
# 엔드포인트
#   POST   /api/whep         : SDP offer(text/sdp)를 받아 answer를 돌려줍니다
#   DELETE /api/whep/{id}    : 세션 종료 (WHEP 리소스 해제)
#   GET    /api/status       : 카메라 연결 상태 / FPS / 시청자 수 (JSON)
#   GET    /api/snapshot     : 현재 프레임 한 장 (JPEG)
#
# 실행 방법 (둘 중 아무거나):
#     python main.py
#     uvicorn main:app --host 0.0.0.0 --port 8000

import asyncio
import contextlib
import io
import time
from collections import deque
from contextlib import asynccontextmanager
from fractions import Fraction
from uuid import uuid4

import av
import cv2
import numpy as np
from aiortc import (
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
)
from aiortc.codecs import h264 as h264_codec
from aiortc.codecs import vpx as vpx_codec
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.mediastreams import MediaStreamTrack, VideoStreamTrack
import httpx
from fastapi import FastAPI, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

import json

import paho.mqtt.client as mqtt

import cctv_zone_map
import config
from detector import FireSmokeDetector

VIDEO_CLOCK_RATE = 90000  # WebRTC 영상 타임스탬프의 표준 클럭

# ---------------------------------------------------------------- 비트레이트 상한 해제
# aiortc는 인코더 비트레이트를 모듈 상수로 묶어둡니다(VP8 1.5Mbps / H264 3Mbps).
# 이 상한 때문에 해상도를 올려도 화면이 뭉개지므로 설정값으로 올려줍니다.
# 인코더가 생성될 때 이 전역값을 읽으므로 앱 시작 전에 바꿔두면 됩니다.
#
# ※ aiortc 내부 상수를 직접 건드리는 방식입니다. aiortc를 올릴 때 이름이 바뀌지
#   않았는지 확인하세요. (공개 API로는 목표 비트레이트를 지정할 수 없습니다)
for _codec in (h264_codec, vpx_codec):
    _codec.DEFAULT_BITRATE = config.VIDEO_BITRATE
    _codec.MAX_BITRATE = config.VIDEO_BITRATE

# aiortc의 H264 인코더는 profile=Baseline, level=31로 고정되어 있습니다.
# level 3.1의 상한이 1280x720이라 그보다 크게 보내면 규격을 벗어납니다.
# 1080p로 내보내려면 VP8을 쓰세요 (PREFER_H264=0). VP8은 이런 제약이 없습니다.
H264_MAX_WIDTH = 1280
if config.PREFER_H264 and config.MAX_WIDTH > H264_MAX_WIDTH:
    print(
        f"[warn] MAX_WIDTH={config.MAX_WIDTH}는 H264 level 3.1 상한({H264_MAX_WIDTH})을 넘습니다.\n"
        f"       1080p가 필요하면 PREFER_H264=0으로 VP8을 쓰세요."
    )


# ============================================================== 카메라 소스
class CameraSource:
    """RTSP 연결을 혼자 담당합니다.

    가장 최근 프레임 한 장만 들고 있고, 연결이 끊기면 스스로 재연결합니다.
    브라우저에 나가는 트랙(CameraTrack)은 이 연결의 생사와 무관하게 계속 살아있으므로,
    카메라가 잠깐 끊겼다 돌아와도 WebRTC를 다시 맺을 필요가 없습니다.
    """

    def __init__(self, rtsp_url: str):
        self._rtsp_url = rtsp_url
        self.latest: av.VideoFrame | None = None
        self.connected = False
        self.last_error: str | None = None
        self.resolution: str | None = None
        self.last_frame_at = 0.0
        self._frame_times = deque(maxlen=30)

    @property
    def fps(self) -> float:
        if len(self._frame_times) < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        return (len(self._frame_times) - 1) / span if span > 0 else 0.0

    @property
    def seconds_since_frame(self) -> float | None:
        return time.time() - self.last_frame_at if self.last_frame_at else None

    def _mark(self, frame: av.VideoFrame):
        now = time.time()
        self.last_frame_at = now
        self._frame_times.append(now)
        if self.resolution is None:
            self.resolution = f"{frame.width}x{frame.height}"

    async def run(self):
        """연결 → 프레임 수신 → 끊기면 재연결. 서버가 사는 동안 계속 돕니다."""
        while True:
            player = None
            try:
                # av.open()은 블로킹입니다. 연결이 오래 걸릴 수 있어 스레드에서 엽니다.
                player = await asyncio.to_thread(
                    MediaPlayer,
                    self._rtsp_url,
                    format=config.RTSP_FORMAT,
                    options=config.RTSP_OPTIONS,
                    timeout=config.OPEN_TIMEOUT,
                )
                if player.video is None:
                    raise RuntimeError("입력에 영상 트랙이 없습니다")

                print("[rtsp] 카메라 연결됨")
                self.connected = True
                self.last_error = None

                while True:
                    frame = await asyncio.wait_for(
                        player.video.recv(), timeout=config.FRAME_TIMEOUT
                    )
                    self.latest = frame
                    self._mark(frame)

            except asyncio.CancelledError:
                raise
            except Exception as exc:  # 연결 실패 · 수신 끊김 · 타임아웃 전부 여기로
                self.connected = False
                self.last_error = str(exc) or exc.__class__.__name__
                print(
                    f"[rtsp] 끊김: {self.last_error} — {config.RECONNECT_DELAY}초 후 재시도"
                )
            finally:
                if player is not None and player.video is not None:
                    player.video.stop()

            await asyncio.sleep(config.RECONNECT_DELAY)


# ============================================================== 송출 트랙
class CameraTrack(VideoStreamTrack):
    """브라우저로 나가는 영상 트랙.

    소스가 끊겨도 이 트랙은 끝나지 않습니다. 프레임이 없으면 검은 화면을 내보내
    WebRTC 연결을 유지하고, 카메라가 살아나면 영상이 그대로 이어집니다.
    (트랙이 끝나버리면 브라우저가 재협상을 해야 하는데, 그게 훨씬 번거롭습니다.)

    ▶ 나중에 YOLO를 붙일 자리: `_render()`에서 frame → ndarray로 바꿔 추론하고
      바운딩 박스를 그린 뒤 다시 VideoFrame으로 만들면 됩니다.
    """

    def __init__(self, source: CameraSource, annotate: bool = True):
        super().__init__()
        self._source = source
        self._annotate_enabled = annotate  # 로봇캠처럼 YOLO 박스가 필요 없는 스트림은 False
        self._count = 0
        self._start: float | None = None
        self._blank: np.ndarray | None = None

    async def recv(self):
        pts, time_base = await self._next_timestamp()
        frame = self._render()
        frame.pts = pts
        frame.time_base = time_base
        return frame

    async def _next_timestamp(self):
        """TARGET_FPS에 맞춰 송출 속도를 조절합니다."""
        if self._start is None:
            self._start = time.time()
            self._count = 0
        else:
            self._count += 1
            delay = (self._start + self._count / config.TARGET_FPS) - time.time()
            if delay > 0:
                await asyncio.sleep(delay)
            elif delay < -1.0:
                # 시청자가 없어 한동안 쉬었다 재개된 경우.
                # 밀린 프레임을 몰아 보내지 않도록 시계를 현재 시각으로 맞춥니다.
                self._start = time.time() - self._count / config.TARGET_FPS
        pts = int(self._count * VIDEO_CLOCK_RATE / config.TARGET_FPS)
        return pts, Fraction(1, VIDEO_CLOCK_RATE)

    def _render(self) -> av.VideoFrame:
        frame = self._source.latest
        age = self._source.seconds_since_frame
        if frame is None or age is None or age > config.STALE_AFTER:
            return self._blank_frame()
        return self._scale(self._annotate(frame) if self._annotate_enabled else frame)

    def _annotate(self, frame: av.VideoFrame) -> av.VideoFrame:
        """가장 최근 YOLO 감지 결과(fire_status.boxes)를 프레임에 그립니다.
        박스는 detect_loop가 원본 해상도 기준으로 넘겨주므로(detector.py 참고)
        아직 스케일링 전인 이 시점의 frame과 좌표계가 그대로 맞습니다.
        감지 주기(1.5초)가 프레임 주기(1/15초)보다 훨씬 길어서, 다음 검사 전까지는
        같은 박스가 그대로 유지된 채 여러 프레임에 걸쳐 그려집니다."""
        if not fire_status.boxes:
            return frame
        img = frame.to_ndarray(format="bgr24")
        for box in fire_status.boxes:
            x1, y1, x2, y2 = box["xyxy"]
            color = (0, 0, 255) if box["cls"] == "fire" else (0, 165, 255)  # BGR: 빨강(fire)/주황(smoke)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            label = f'{box["cls"].upper()} {box["conf"]:.2f}'
            cv2.putText(img, label, (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return av.VideoFrame.from_ndarray(img, format="bgr24")

    def _scale(self, frame: av.VideoFrame) -> av.VideoFrame:
        # yuv420p는 가로·세로가 짝수여야 합니다.
        if config.MAX_WIDTH and frame.width > config.MAX_WIDTH:
            width = config.MAX_WIDTH - (config.MAX_WIDTH % 2)
            height = int(frame.height * width / frame.width)
            height -= height % 2
            return frame.reformat(width=width, height=height, format="yuv420p")
        return frame.reformat(format="yuv420p")

    def _blank_frame(self) -> av.VideoFrame:
        if self._blank is None:
            width = (config.MAX_WIDTH or 960) & ~1
            height = (width * 9 // 16) & ~1
            self._blank = np.full((height, width, 3), 18, dtype=np.uint8)
        # 매번 새 프레임을 만듭니다. 같은 객체를 재사용하면 인코더 큐에 남아있는
        # 프레임의 pts를 덮어쓰게 됩니다.
        return av.VideoFrame.from_ndarray(self._blank, format="bgr24").reformat(
            format="yuv420p"
        )


camera = CameraSource(config.RTSP_URL)
camera_track = CameraTrack(camera)

# 로봇캠(2번째 스트림) — ROBOT_RTSP_URL이 비어있으면 아예 안 만듭니다.
robot_camera = CameraSource(config.ROBOT_RTSP_URL) if config.ROBOT_RTSP_URL else None
robot_camera_track = CameraTrack(robot_camera, annotate=False) if robot_camera else None


# ============================================================== 화재·연기 감지
class FireStatus:
    """가장 최근 감지 결과. /api/fire-status가 그대로 보여줍니다."""

    def __init__(self):
        self.flame_detected = False
        self.smoke_detected = False
        self.confidence: float | None = None
        self.last_checked_at: float | None = None
        self.boxes: list[dict] = []  # [{cls, conf, xyxy}, ...] — CameraTrack이 그대로 그림


fire_status = FireStatus()
detector: FireSmokeDetector | None = None
mqtt_client: mqtt.Client | None = None


def setup_mqtt() -> mqtt.Client | None:
    """MQTT 연결에 실패해도 None을 돌려주고 넘어갑니다 — 화재 이벤트 발행만 못 할 뿐,
    영상 스트리밍(WHEP)은 MQTT와 무관하니 브로커가 죽었다고 서버 전체가 죽으면 안 됩니다."""
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except (AttributeError, TypeError):
        client = mqtt.Client()
    try:
        client.connect(config.MQTT_HOST, config.MQTT_PORT, 60)
        client.loop_start()
        print(f"[mqtt] 연결됨 → {config.MQTT_HOST}:{config.MQTT_PORT}")
        return client
    except Exception as exc:
        print(f"[mqtt] 연결 실패, 화재 이벤트 발행 없이 계속 진행: {exc}")
        return None


def _detected_location(result: dict) -> str:
    """감지된 박스의 바닥 접촉점을 평면도 구역으로 변환합니다.
    호모그래피 캘리브레이션(cctv_zone_map)이 이 카메라 각도/위치 기준이라, 카메라가
    움직이면 다시 캘리브레이션해야 합니다. 박스가 없거나 변환 실패하면 config의
    고정값으로 폴백합니다."""
    boxes = result.get("boxes") or []
    if not boxes:
        return config.CCTV_LOCATION
    box = max(boxes, key=lambda b: b["conf"])
    try:
        return cctv_zone_map.box_to_zone(box["xyxy"])
    except Exception as exc:
        print(f"[fire] 구역 변환 실패, 고정값 사용: {exc}")
        return config.CCTV_LOCATION


def publish_fire_event(result: dict):
    """감지 결과를 fire/event 토픽으로 발행합니다.
    DB 서버(mqtt_client.py)가 이미 이 토픽을 구독해 fire_event 테이블에 저장하고
    WebSocket으로 웹·앱에 실시간 전달하도록 되어 있어, 여기서는 발행만 하면 됩니다."""
    if mqtt_client is None:
        return

    # flame이 확정되면 그 자체로 위험, smoke만 잡히면(flame 미확인) 주의.
    alert_level = "위험" if result["flame_detected"] else "주의"

    payload = {
        "location": _detected_location(result),
        "source": "cctv",
        "flame_detected": result["flame_detected"],
        "smoke_detected": result["smoke_detected"],
        "confidence": result["confidence"],
        "alert_level": alert_level,
    }
    mqtt_client.publish(config.MQTT_FIRE_TOPIC, json.dumps(payload))
    print(f"[fire] 감지 이벤트 발행: {payload}")


FIRE_REPUBLISH_INTERVAL = 90.0  # 웹의 FIRE_HOLD_MS(2분)보다 짧게 — 화재가 계속되면 계속 재발행


async def detect_loop():
    """DETECT_INTERVAL마다 최신 프레임 한 장만 검사합니다.
    영상 스트리밍(CameraTrack._render)과는 완전히 분리된 흐름이라
    시청자 수·프레임률에 영향을 주지 않습니다."""
    was_detected = False
    last_published_at = 0.0

    while True:
        await asyncio.sleep(config.DETECT_INTERVAL)

        frame = camera.latest
        if frame is None or detector is None:
            continue

        try:
            frame_bgr = frame.to_ndarray(format="bgr24")
            result = await asyncio.to_thread(detector.detect, frame_bgr)
        except Exception as exc:
            print(f"[fire] 추론 실패: {exc}")
            continue

        fire_status.flame_detected = result["flame_detected"]
        fire_status.smoke_detected = result["smoke_detected"]
        fire_status.confidence = result["confidence"]
        fire_status.boxes = result["boxes"]  # 감지 없으면 빈 리스트 -> 스트림에서 박스도 바로 사라짐
        fire_status.last_checked_at = time.time()

        # smoke는 거리가 멀면 YOLO 신뢰도가 너무 낮아 오탐이 잦음 — 연기/가스는 구역별
        # MQ2 고정 센서(sensor/reading)가 이미 맡고 있으므로, CCTV는 flame만으로 트리거함.
        # smoke_detected는 fire_status/영상엔 계속 표시되지만 발행 트리거에서는 뺌.
        is_detected = result["flame_detected"]

        # "없음 -> 있음"으로 바뀐 순간엔 바로 발행하고, 그 뒤로도 감지가 계속되면
        # FIRE_REPUBLISH_INTERVAL마다 다시 발행합니다 — 안 그러면 화재가 몇 분씩
        # 이어져도 프론트(FIRE_HOLD_MS=2분)가 새 메시지를 못 받아 알림이 꺼져버립니다.
        now = time.time()
        if is_detected and (not was_detected or now - last_published_at >= FIRE_REPUBLISH_INTERVAL):
            publish_fire_event(result)
            last_published_at = now
        was_detected = is_detected

class Session:
    """시청자 한 명. last_seen은 /api/status 호출로 갱신됩니다."""

    def __init__(self, pc: RTCPeerConnection, track: MediaStreamTrack):
        self.pc = pc
        self.track = track
        self.last_seen = time.time()


class StreamHub:
    """WHEP 스트림 하나(협상 + 시청자 세션 관리)를 담당합니다.
    CCTV/로봇캠처럼 같은 서버에서 여러 영상을 독립적으로 내보낼 때 이 클래스를
    하나씩 씁니다 — RTSP 연결은 소스당 1개, 시청자는 relay로 나눠줍니다."""

    def __init__(self, track: CameraTrack, source: CameraSource, label: str):
        self.track = track
        self.source = source
        self.label = label
        self.relay = MediaRelay()
        self.sessions: dict[str, Session] = {}

    async def close_session(self, session_id: str):
        session = self.sessions.pop(session_id, None)
        if session is None:
            return
        session.track.stop()  # relay 구독 해제 — 안 하면 시청자가 나가도 계속 인코딩합니다
        await session.pc.close()
        print(f"[whep:{self.label}] {session_id[:8]} 종료 (남은 시청자 {len(self.sessions)}명)")

    async def reap_sessions(self):
        """하트비트가 끊긴 세션을 회수합니다.

        브라우저가 죽거나, 네트워크가 끊기거나, 탭이 강제 종료되면 DELETE가 오지 않고
        WebRTC 연결도 한동안 'connected'로 남아있습니다. 그동안 인코더가 계속 도므로
        소식이 없는 세션은 서버가 직접 정리합니다.
        """
        while True:
            await asyncio.sleep(5)
            cutoff = time.time() - config.SESSION_TIMEOUT
            stale = [sid for sid, s in self.sessions.items() if s.last_seen < cutoff]
            for session_id in stale:
                print(f"[whep:{self.label}] {session_id[:8]} 하트비트 끊김 — 회수")
                await self.close_session(session_id)

    async def offer(self, offer_sdp: str) -> tuple[str, str]:
        """SDP offer를 받아 (session_id, answer_sdp)를 돌려줍니다."""
        ice_servers = [RTCIceServer(urls=url) for url in config.STUN_SERVERS]
        pc = RTCPeerConnection(RTCConfiguration(iceServers=ice_servers))
        session_id = uuid4().hex

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"[whep:{self.label}] {session_id[:8]} → {pc.connectionState}")
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await self.close_session(session_id)

        # 코덱 우선순위를 H264로 바꿉니다(같은 비트레이트에서 VP8보다 화질이 좋음).
        # 순서 중요: aiortc는 setRemoteDescription 시점에 코덱을 확정하므로 그 전에
        # 트랜시버를 만들어 두고 우선순위를 지정해야 합니다.
        video = pc.addTransceiver("video", direction="sendonly")
        if config.PREFER_H264:
            codecs = RTCRtpSender.getCapabilities("video").codecs
            h264 = [c for c in codecs if c.mimeType == "video/H264"]
            if h264:
                video.setCodecPreferences(h264 + [c for c in codecs if c.mimeType != "video/H264"])

        await pc.setRemoteDescription(RTCSessionDescription(sdp=offer_sdp, type="offer"))

        track = self.relay.subscribe(self.track)
        pc.addTrack(track)
        self.sessions[session_id] = Session(pc, track)

        answer = await pc.createAnswer()
        # aiortc는 trickle ICE를 쓰지 않습니다.
        # setLocalDescription이 ICE 후보 수집이 끝날 때까지 기다렸다가 완성된 SDP를 만듭니다.
        await pc.setLocalDescription(answer)

        print(f"[whep:{self.label}] {session_id[:8]} 시작 (시청자 {len(self.sessions)}명)")
        return session_id, pc.localDescription.sdp

    def status(self, session: str | None = None) -> dict:
        if session and session in self.sessions:
            self.sessions[session].last_seen = time.time()
        age = self.source.seconds_since_frame
        return {
            "camera_connected": self.source.connected,
            "fps": round(self.source.fps, 1),
            "viewers": len(self.sessions),
            "resolution": self.source.resolution,
            "seconds_since_frame": round(age, 1) if age is not None else None,
            "error": self.source.last_error,
        }

    def snapshot_jpeg(self) -> bytes | None:
        frame = self.source.latest
        if frame is None:
            return None
        buffer = io.BytesIO()
        frame.to_image().save(buffer, format="JPEG", quality=80)
        return buffer.getvalue()

    async def close_all(self):
        for session_id in list(self.sessions):
            await self.close_session(session_id)


def register_stream_routes(app: FastAPI, prefix: str, hub: StreamHub):
    """hub 하나에 대해 /whep, /status, /snapshot 엔드포인트를 등록합니다."""

    @app.post(f"{prefix}/whep")
    async def whep(request: Request):
        offer_sdp = (await request.body()).decode()
        session_id, answer_sdp = await hub.offer(offer_sdp)
        return Response(
            content=answer_sdp,
            status_code=201,
            media_type="application/sdp",
            headers={"Location": f"{prefix}/whep/{session_id}"},
        )

    @app.delete(f"{prefix}/whep/{{session_id}}")
    async def whep_delete(session_id: str):
        await hub.close_session(session_id)
        return Response(status_code=204)

    @app.get(f"{prefix}/status")
    def status(session: str | None = None):
        return hub.status(session)

    @app.get(f"{prefix}/snapshot")
    def snapshot():
        data = hub.snapshot_jpeg()
        if data is None:
            return Response(status_code=503)
        return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


# ============================================================== 앱
@asynccontextmanager
async def lifespan(app: FastAPI):
    global detector, mqtt_client

    # 모델 로딩은 시간이 좀 걸리니 스레드로 돌려 서버 시작을 막지 않습니다.
    detector = await asyncio.to_thread(FireSmokeDetector)
    mqtt_client = setup_mqtt()

    tasks = [
        asyncio.create_task(camera.run()),
        asyncio.create_task(cctv_hub.reap_sessions()),
        asyncio.create_task(detect_loop()),
    ]
    if robot_camera is not None:
        tasks.append(asyncio.create_task(robot_camera.run()))
        tasks.append(asyncio.create_task(robot_hub.reap_sessions()))
    print(f"[server] 준비 완료 — http://localhost:{config.SERVER_PORT}/api/status")
    yield
    for task in tasks:
        task.cancel()
    for task in tasks:
        with contextlib.suppress(asyncio.CancelledError):
            await task
    await cctv_hub.close_all()
    if robot_hub is not None:
        await robot_hub.close_all()
    if mqtt_client is not None:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    print("[server] 종료")


app = FastAPI(lifespan=lifespan, title="SENTRY CCTV WebRTC")

app.add_middleware(
    CORSMiddleware,
    allow_origins=config.ALLOWED_ORIGINS,
    allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    # 브라우저가 세션 종료용 Location 헤더를 읽을 수 있어야 합니다.
    expose_headers=["Location"],
)

# CCTV 스트림은 항상 /api/whep, /api/status, /api/snapshot (기존 경로 그대로 유지).
cctv_hub = StreamHub(camera_track, camera, label="cctv")
register_stream_routes(app, "/api", cctv_hub)

# 로봇캠은 ROBOT_RTSP_URL이 설정된 경우에만 /api/robot/* 로 별도 등록됩니다.
robot_hub = StreamHub(robot_camera_track, robot_camera, label="robot") if robot_camera else None
if robot_hub:
    register_stream_routes(app, "/api/robot", robot_hub)


@app.get("/api/fire-status")
def fire_status_endpoint():
    """가장 최근 화재·연기 감지 결과. 관제 웹이 주기적으로 폴링하거나,
    fire/event MQTT 발행을 통해 DB에 이미 저장된 이력과 함께 씁니다."""
    age = (
        time.time() - fire_status.last_checked_at
        if fire_status.last_checked_at
        else None
    )
    return {
        "flame_detected": fire_status.flame_detected,
        "smoke_detected": fire_status.smoke_detected,
        "confidence": fire_status.confidence,
        "seconds_since_check": round(age, 1) if age is not None else None,
    }


# ---------------------------------------------------------------- 로봇캠 릴레이
# 브라우저는 사설 IP(라즈베리파이)에 직접 못 붙는 경우가 있어서(크롬 Private Network
# Access 차단), 이 GCP 서버가 Tailscale로 대신 접속해 그대로 흘려보냅니다.
# ROBOT_CAM_URL이 설정된 경우에만 켭니다.
if config.ROBOT_CAM_URL:

    @app.get("/api/robot-cam/stream")
    async def robot_cam_stream():
        client = httpx.AsyncClient(timeout=None)
        try:
            upstream = await client.send(
                client.build_request("GET", f"{config.ROBOT_CAM_URL}/stream"),
                stream=True,
            )
        except Exception as exc:
            await client.aclose()
            return Response(status_code=502, content=f"로봇캠 연결 실패: {exc}")

        async def relay():
            try:
                async for chunk in upstream.aiter_raw():
                    yield chunk
            finally:
                await upstream.aclose()
                await client.aclose()

        content_type = upstream.headers.get("content-type", "multipart/x-mixed-replace; boundary=frame")
        return StreamingResponse(relay(), media_type=content_type)

    @app.get("/api/robot-cam/")
    def robot_cam_page():
        """/api/robot-cam/stream을 화면 꽉 채워 보여주는 래퍼 페이지.
        프론트는 <img> 대신 이 페이지를 <iframe>에 통째로 띄웁니다 — 같은 origin이라
        <img>가 정상 렌더되고, CSS로 크기도 우리가 원하는 대로 맞출 수 있습니다."""
        html = (
            "<html><body style='margin:0;background:#000;overflow:hidden'>"
            "<img src='/api/robot-cam/stream' "
            "style='width:100%;height:100%;object-fit:cover;display:block'>"
            "</body></html>"
        )
        return Response(content=html, media_type="text/html")


# ---------------------------------------------------------------- 실행 진입점
# 이게 없으면 `python main.py`가 app만 정의하고 곧바로 종료합니다.
# (에러도 안 나고 그냥 꺼져서 원인을 찾기 어렵습니다)
if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host=config.SERVER_HOST, port=config.SERVER_PORT)
