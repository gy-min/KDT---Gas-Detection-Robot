# 1. 프로젝트 이름

**화재 예방 가스 탐지 로봇**

## 2. 아키텍처 구성도

```
[로봇: ESP32+라즈베리파이]   [고정형 가스센서]   [IP 카메라]
    모터·주행, QR 위치           MQ 센서            RTSP
        │                         │                  │
        │ MQTT (robot/event, sensor/reading)          │ RTSP
        ▼                         ▼                  ▼
┌─────────────────┐        ┌──────────────────┐
│  메인 서버(FastAPI)│◄──────►│ 화재감지 서버(FastAPI)│
│  MQTT 구독→MariaDB │  MQTT   │  YOLOv8n 화재/연기  │
│  WebSocket 브로드캐스트│fire/event│  WebRTC로 웹에 중계 │
│  REST API(로그인·대피경로)│        └──────────────────┘
└─────────┬─────────┘
          │ REST + WebSocket
    ┌─────┴─────┐
    ▼           ▼
[관리자 웹]   [직원 앱]
React        Android(Kotlin)
```

## 3. 소스코드 구성

| 폴더 | 내용 |
|---|---|
| `android-app/` | 현장 직원용 앱 (Kotlin, Jetpack Compose) — 홈/대피경로/119 화면, 비상 경보 |
| `admin-web/` | 관리자 웹 (React + Vite) — 평면도·센서 현황, 지시사항 하달 |
| `server/` | 메인 서버 (FastAPI) — MQTT 구독→DB 저장, WebSocket 실시간 전송, 로그인, 대피 경로 계산 |
| `fire-detection-server/` | 화재 감지 서버 (FastAPI) — IP카메라 영상을 YOLOv8n으로 분석, 감지 시 MQTT 발행 |
| `robot-code/` | 로봇 펌웨어(ESP32, 모터·라인트레이싱) + 라즈베리파이 내비게이션(QR 위치 인식, 자율 주행) |
| `gas-classification/` | MQ 센서 값으로 가스 종류를 판별하는 규칙 기반 알고리즘 |
| `robot-camera/` | 로봇 자체 카메라 스트리밍 |

## 4. 실행 방법

| 구성 요소 | 명령어 |
|---|---|
| 메인 서버 | `pip install -r requirements.txt` → `uvicorn main:app --host 0.0.0.0 --port 8080` |
| 화재 감지 서버 | `pip install -r requirements.txt` → `python main.py` |
| 관리자 웹 | `npm install` → `npm run dev` |
| 로봇 | 라즈베리파이: `python rpi_qr_navigator.py` / ESP32: `c_main.py` 업로드 |
| 직원 앱 | Android Studio로 `android-app/` 열고 빌드 (서버 주소는 `local.properties`에서 설정) |

> 메인 서버·화재감지 서버 실행 전에 MariaDB와 MQTT 브로커(Mosquitto)가 먼저 떠 있어야 합니다.
> 서버 IP·비밀번호 등 실제 값은 공개 저장소라 코드에 없습니다 — 각 폴더 안내(주석/환경변수)를 참고해 로컬에 채워 넣으세요.
