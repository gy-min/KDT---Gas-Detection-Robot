# config.py — 스트리밍 서버 설정
#
# ⚠️ RTSP_URL에는 카메라 계정/비밀번호가 들어갑니다.
#    이 파일에 직접 적어두면 그대로 커밋되니 환경변수로 넘기는 것을 권장합니다.
#
#      PowerShell:  $env:RTSP_URL = "rtsp://admin:1234@192.168.0.100:554/stream1"
#      bash:        export RTSP_URL="rtsp://admin:1234@192.168.0.100:554/stream1"

import os

# ---------------------------------------------------------------- 카메라
# 실제 IP 카메라: "rtsp://<id>:<pw>@<ip>:554/<path>"
# 동영상 파일로 테스트: "C:/path/to/sample.mp4"
# 노트북 웹캠으로 테스트(Windows): RTSP_URL="video=<장치이름>", RTSP_FORMAT="dshow"
#   장치 이름 확인:  ffmpeg -list_devices true -f dshow -i dummy
#
# ⚠️ 경로 끝의 숫자가 스트림을 고릅니다. 이 카메라는:
#      /0  → 1920x1080 (메인 스트림)   ← 화면 출력용
#      /1  →  720x480  (서브 스트림)   ← 나중에 YOLO 추론용으로 쓰면 좋습니다
#    처음엔 /1로 잡혀 있어서 SD 화질이 나오고 있었습니다.
RTSP_URL = os.getenv("RTSP_URL", "rtsp://<id>:<pw>@YOUR_CAMERA_IP:554/1")  # 실제 값은 환경변수로

# 라즈베리파이 카메라(2번째 스트림, 예: 로봇 탑재 카메라). 비어있으면 이 스트림은
# 아예 안 켭니다 — 아직 라즈베리파이 쪽 RTSP 서버(MediaMTX 등)가 안 붙어 있어도
# 기존 CCTV 스트림엔 영향 없습니다.
ROBOT_RTSP_URL = os.getenv("ROBOT_RTSP_URL", "")

# 로봇에 USB로 물린 웹캠(MJPEG, 라즈베리파이가 자체 서빙). 브라우저가 이 사설 IP로
# 직접 못 붙는 경우(크롬 Private Network Access 차단)를 위해, 이 GCP 서버가 Tailscale로
# 대신 접속해서 릴레이합니다. 비어있으면 릴레이 엔드포인트 자체가 안 켜짐.
ROBOT_CAM_URL = os.getenv("ROBOT_CAM_URL", "")

# 입력 포맷. RTSP·동영상 파일은 None(자동 판별), 웹캠은 "dshow".
RTSP_FORMAT = os.getenv("RTSP_FORMAT") or None

# FFmpeg(libav)에 넘길 입력 옵션.
#   rtsp_transport=tcp : UDP는 프레임이 깨져 들어오는 경우가 많아 TCP로 강제합니다.
#   fflags=nobuffer    : 입력 버퍼링을 줄여 지연을 낮춥니다.
#
# ※ 소켓 타임아웃은 여기에 넣지 않습니다. RTSP의 stimeout 옵션은 FFmpeg 버전마다
#   이름이 달라(stimeout → timeout) 버전에 따라 무시되거나 에러가 납니다.
#   대신 아래 OPEN_TIMEOUT을 PyAV에 직접 넘겨 버전과 무관하게 처리합니다.
RTSP_OPTIONS = {
    "rtsp_transport": "tcp",
    "fflags": "nobuffer",
    "max_delay": "500000",
}

# 카메라 접속을 이 시간(초) 안에 못 맺으면 포기하고 재시도합니다.
# 이게 없으면 카메라가 꺼져 있을 때 av.open()이 아주 오래 매달릴 수 있습니다.
OPEN_TIMEOUT = float(os.getenv("OPEN_TIMEOUT", "6"))

# 연결이 끊겼을 때 재시도까지 기다리는 시간(초)
RECONNECT_DELAY = float(os.getenv("RECONNECT_DELAY", "3"))

# 이 시간(초) 동안 프레임이 안 오면 끊긴 것으로 보고 재연결합니다.
FRAME_TIMEOUT = float(os.getenv("FRAME_TIMEOUT", "8"))

# 마지막 프레임이 이 시간(초)을 넘기면 대신 검은 화면을 내보냅니다.
STALE_AFTER = float(os.getenv("STALE_AFTER", "3"))

# ---------------------------------------------------------------- 인코딩
# 카메라 H.264를 디코딩해서 WebRTC용으로 다시 인코딩합니다.
# aiortc는 파이썬(소프트웨어) 인코더를 쓰고 시청자 1명당 인코더가 하나씩 돌지만,
# 실측 결과 1080p H264가 프레임당 4ms 수준이라 해상도보다는 대역폭이 먼저 문제가 됩니다.
#
# 화면에 띄우는 크기(썸네일 ~835px, 팝업 ~1040px) 기준으로는 1280이면 충분합니다.
# 더 선명하게 하려면 1920으로 올리세요. (대역폭이 2배 이상 늘어납니다)
MAX_WIDTH = int(os.getenv("MAX_WIDTH", "1280"))  # 0이면 원본 그대로
TARGET_FPS = float(os.getenv("TARGET_FPS", "15"))

# 송출 비트레이트(bps). 화질을 좌우하는 값입니다.
#
# ⚠️ aiortc 기본값이 낮습니다 — VP8은 최대 1.5Mbps, H264는 최대 3Mbps로 묶여 있어서
#    해상도를 올려도 그 상한에 걸려 뭉개집니다. main.py가 이 값으로 상한을 풀어줍니다.
#    실제 전송량은 브라우저가 보내는 REMB 피드백에 따라 이 값 아래에서 조절됩니다.
VIDEO_BITRATE = int(os.getenv("VIDEO_BITRATE", "4000000"))  # 4 Mbps

# 선호 코덱. H264는 같은 비트레이트에서 VP8보다 화질이 좋습니다.
# 브라우저가 H264를 제안하지 않으면 자동으로 VP8로 협상됩니다.
#
# ⚠️ aiortc의 H264 인코더는 level 3.1로 고정이라 1280x720이 상한입니다.
#    1080p로 내보내려면 PREFER_H264=0 + MAX_WIDTH=1920 조합을 쓰세요.
PREFER_H264 = os.getenv("PREFER_H264", "1") != "0"

# ---------------------------------------------------------------- WebRTC
# 같은 LAN 안에서는 STUN 없이도 붙습니다(host candidate).
# 다른 네트워크에서 접속해야 할 때만 STUN/TURN을 채우세요.
STUN_SERVERS = [s for s in os.getenv("STUN_SERVERS", "").split(",") if s.strip()]

# 시청자는 /api/status를 주기적으로 부르며 자기 세션이 살아있음을 알립니다.
# 이 시간(초) 동안 소식이 없으면 서버가 세션을 회수합니다.
# 브라우저가 죽거나 네트워크가 끊기면 DELETE가 오지 않는데, 그때 인코더가
# 계속 도는 것을 막는 안전장치입니다. 폴링 주기(3초)보다 넉넉해야 합니다.
SESSION_TIMEOUT = float(os.getenv("SESSION_TIMEOUT", "20"))

# ---------------------------------------------------------------- 서버
SERVER_HOST = os.getenv("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.getenv("SERVER_PORT", "8081"))

# 프론트를 Vite 프록시 없이 직접 붙일 때 필요한 CORS 허용 목록.
# (프록시를 쓰면 같은 출처가 되므로 사실상 필요 없지만, 빌드본 배포 시를 위해 열어둡니다)
ALLOWED_ORIGINS = os.getenv(
    "ALLOWED_ORIGINS",
    "http://YOUR_SERVER_IP:8080,http://localhost:5173,http://127.0.0.1:5173,http://localhost:4173",
).split(",")

# ---------------------------------------------------------------- 화재·연기 감지 (YOLO)
# 영상 스트리밍과 분리된 별도 주기로 동작합니다 — 매 프레임 추론하지 않고
# DETECT_INTERVAL마다 최신 프레임 한 장만 검사합니다. (스트리밍 성능에 영향 없음)
FIRE_MODEL_REPO = os.getenv("FIRE_MODEL_REPO", "rabahdev/fire-smoke-yolov8n")
FIRE_MODEL_FILE = os.getenv("FIRE_MODEL_FILE", "best.pt")

FIRE_CONF_THRESHOLD = float(os.getenv("FIRE_CONF_THRESHOLD", "0.4"))
FIRE_INFER_WIDTH = int(os.getenv("FIRE_INFER_WIDTH", "416"))  # 추론용 축소 해상도 (속도)
DETECT_INTERVAL = float(os.getenv("DETECT_INTERVAL", "1.5"))   # 몇 초마다 검사할지

# 감지 시 MQTT로 발행할 fire/event 정보
CCTV_LOCATION = os.getenv("CCTV_LOCATION", "zone_C")  # 이 CCTV가 비추는 구역/위치
MQTT_HOST = os.getenv("MQTT_HOST", "YOUR_SERVER_IP")    # DB 서버(Mosquitto)의 외부 IP
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_FIRE_TOPIC = os.getenv("MQTT_FIRE_TOPIC", "fire/event")
