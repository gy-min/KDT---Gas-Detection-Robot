# 백엔드 연동 규격 (실제 FastAPI 서버 기준)

`main.py`/`database.py`/`models.py`/`mqtt_client.py`/`evacuation.py`/`map_data.py`로 받은 실제
서버·로봇 코드를 기준으로 정리했습니다. (예전 버전은 앱이 없는 `/api/monitoring/snapshot`
엔드포인트를 가정하고 있었는데, 그건 실제 서버에 없는 걸로 확인돼서 이 문서를 실서버에 맞게
다시 썼습니다.)

인증/DB/MQTT/WebSocket/대부분의 REST 조회 엔드포인트는 버전이 바뀌어도 그대로입니다. 대피 경로
(`/api/evacuation-route`)만 여러 번 구조가 바뀌었는데, **`evacuation.py`가 8/20에 `map_data.py`
기준으로 전면 재작성되면서 로봇 내비게이션이 쓰는 지도와 서버 대피 경로 계산이 완전히 같은
지도를 쓰도록 확정됐습니다** (`SafeScout_API_계약서.md`의 출구 4개(TL/TR/BL/BR) 스펙과도 일치).
자세한 구조는 4번 항목 참고 — 지금 앱 지도(`EvacGraph.kt`/`EvacScreen.kt`)도 이 구조에 맞춰
다시 그렸습니다.

## 서버 주소

실제 서버 IP는 이 저장소가 공개(public)라 여기 안 적어둡니다 — `local.properties`의
`api.baseUrl`/`api.wsUrl`(gitignored)에 설정합니다. 형식은:

- REST: `http://<서버IP>:8080/api/`
- WebSocket: `ws://<서버IP>:8080/ws/realtime`

(`config.py`의 `SERVER_PORT = 8080` 기준. 3306은 MariaDB 포트라 앱은 거기 안 붙습니다.)

## 1. WebSocket — `/ws/realtime`

**스냅샷을 통째로 주는 게 아니라, 이벤트 하나씩** 옵니다. MQTT로 로봇/센서가 새 메시지를 보낼
때마다 서버가 그대로 중계합니다:

```json
{ "topic": "sensor/reading", "data": { "zone_id": "zone_A1", "rs_value": 320, "strength": 15, "status": "정상" } }
```

`topic`은 네 가지 중 하나:
- (`config.py`의 `MQTT_TOPICS`와 동일, MQTT 중계)
  - `sensor/reading` → `data`는 `fixed_sensor_reading` 한 행 (zone_id, rs_value, strength, status)
  - `robot/event` → `data`는 `robot_event`(+판별 시 `robot_gas_result`) 한 건 (location, event_type,
    가스 판별까지 마쳤으면 gas_type/confidence/alert_level도 포함)
  - `fire/event` → `data`는 `fire_event` 한 건 (location, source, flame_detected, smoke_detected,
    alert_level)
- `instruction/new` → MQTT가 아니라 관리자 웹이 `POST /api/instructions`를 호출한 직후 서버가
  직접 브로드캐스트합니다. `data`는 `{id, event_kind, event_label, instruction}` (`created_at`은
  이 실시간 푸시엔 없음 — REST로 다시 조회할 때만 포함). 앱은 이걸 `DirectiveInfo`로 변환해서
  배너 + 알림(진동)으로 보여줍니다 (`DirectiveNotifier`).

**앱은 이걸 받아서 로컬에 "구역별 최신 상태" 맵을 직접 유지합니다** (`MonitoringRepository`).
`sensor/reading`이 오면 해당 `zone_id`만 갱신, `robot/event`가 오면 로봇 위치만 갱신, 이런 식으로요.
WebSocket은 접속 시점에 지금까지 상태를 안 주기 때문에, 접속 직후엔 항상 REST로 먼저 현재 상태를
읽어옵니다 (아래 2번).

## 2. REST — 초기 로딩 + WebSocket 끊겼을 때 폴백

| 앱이 호출하는 것 | 실제 서버 엔드포인트 | 비고 |
|---|---|---|
| 구역 상태 | `GET /api/fixed-sensor/latest` | 구역별 가장 최근 값 |
| 로봇 위치 | `GET /api/robot-events?limit=1` | 최신 1건만 사용 |
| 화재 상태 | `GET /api/fire-events?limit=1` | 최신 1건만 사용 |
| 관리자 지시사항 | `GET /api/instructions?limit=1` | 최신 1건만 사용 |

5초 간격으로 폴링(WebSocket 끊겼을 때만), 연결되면 폴링 멈추고 WebSocket 델타만 반영합니다.

## 3. 앱이 값을 해석하는 방식 (서버가 안 주는 것들)

실제 서버엔 없지만 앱 UI가 필요로 하는 개념들이라, `MonitoringRepository`에서 계산해서 만듭니다.

- **`emergency.active`(비상 상황 여부)**: 서버에 이 필드 자체가 없습니다. 앱이 다음 조건 중
  하나라도 맞으면 `true`로 계산합니다: ① 구역 상태(`fixed_sensor_reading.status`)가 "위험"인
  구역이 하나라도 있음, ② 로봇 이벤트의 `alert_level`이 위험 수준, ③ 화재 이벤트의 `alert_level`이
  위험 수준이거나 `flame_detected=true`. (`RemoteModels.kt`의 `isDangerLevel`/`statusFromKorean`)
- **구역 상태 문자열**: DB엔 `"정상"/"주의"/"위험"` 한글 그대로 들어있습니다. 앱이 내부적으로
  `NORMAL`/`CAUTION`/`DANGER`로 매핑해서 씁니다. **다른 표현(예: "경계", "danger" 영문 등)을
  쓰신다면 `statusFromKorean()` 매핑에 추가해주셔야 정확히 인식됩니다.**
- **"세기"(Zone 카드에 표시되는 숫자)**: `strength`가 있으면 그걸, 없으면 `rs_value`를 씁니다.
- **`zone_id`는 지도의 한 간선(edge) 단위**: `zone_A1`~`zone_A3`, `zone_B1`~`zone_B3`,
  `zone_C1`~`zone_C3` (`map_data.ZONES`/`SafeScout_API_계약서.md`로 확정). 각 코드는 지도 위
  정확히 한 구간(예: `zone_A2` = A-B 노드 사이)을 가리킵니다 — `EvacGraph.kt`의 `ZONE_EDGES`/
  `ALL_ZONE_IDS`가 이 매핑을 그대로 갖고 있고, `HomeScreen`/`EvacScreen`이 여기 맞춰 지점별로
  조회합니다.
- **로봇 위치 표시**: `robot_event.location`이 `"(행,열)"` 좌표 문자열이고, 이제 이 좌표계가
  지도 노드(TL/A/B/... 등)와 같은 공간이라는 게 확정됐습니다. 그래서 앱은 좌표를 그대로 지도
  위 정확한 위치에 핀으로 찍습니다(`EvacGraph.kt`의 `parseRowCol()`) — 노드에 스냅하지 않고
  실좌표 그대로. 파싱이 안 되는 값이면 지도 아래 "최근 로봇 위치 ... · 순찰 중" 캡션으로
  대체합니다.

## 4. 대피 경로 계산 (`GET /api/evacuation-route`) — 확정된 실제 지도 기준 (2026-08-21)

**`evacuation.py`가 8/20에 `map_data.py` 기준으로 전면 재작성됐습니다.** 이전의 "빗 모양"(V2)/
"사다리 모양"(V3 초안) 좌표계는 전부 지나간 버전이고, 이제는 **로봇의 실제 라인트레이싱
내비게이션이 쓰는 지도(`map_data.py`)와 서버의 대피 경로 계산이 완전히 같은 지도**를 씁니다 —
로봇/센서가 보내는 좌표를 그대로 써도 엉뚱한 경로가 나오지 않습니다. `SafeScout_API_계약서.md`의
출구 4개(TL/TR/BL/BR) 스펙과도 일치합니다.

**지도 구조** (`map_data.py`, 좌표계: (행,열), 행 0~11 위→아래, 열 0~12 왼→오):
- 노드 12개 — 출구 4개(`TL`,`TR`,`BL`,`BR`), 도착점(막다른 지점, 출구 아님) 2개(`ML`,`MR`),
  교차로 6개(`A`~`F`).
- 격자가 아니라 사다리꼴 3줄 + 세로 레일 2줄:
  ```
  TL --- A --- B --- TR
         |     |
  ML --- C --- D --- MR
         |     |
  BL --- E --- F --- BR
  ```
- 가스 센서 구역(zone_id) 9개는 각각 위 그림의 가로 간선 하나에 대응합니다: `A1`=TL-A,
  `A2`=A-B, `A3`=B-TR, `B1`=ML-C, `B2`=C-D, `B3`=D-MR, `C1`=BL-E, `C2`=E-F, `C3`=F-BR.
  세로 레일 간선(A-C, C-E, B-D, D-F) 4개는 센서가 없어서 절대 막히지 않습니다.

**요청**: `GET /api/evacuation-route?current_location=<node id 또는 "(행,열)" 좌표>` —
`current_location`이 노드 이름(`"TL"`,`"C"` 등)이면 그대로, `"(행,열)"` 좌표면 가장 가까운
노드로 스냅해서 계산합니다(`resolve_start_node`, 반경 3.0 이내).

**앱 UI는 관리자 웹과 똑같이 노드 12개가 아니라 3×3 zone 카드(A1~C3)를 보여줍니다** — 사용자는
QR 좌표나 노드 이름을 몰라도 되고, 카드 하나를 탭하면 됩니다. 문제는 zone 카드 하나가 실제로는
노드가 아니라 **간선(두 노드 사이 구간)**이라는 것 — "그 구역 어딘가에 서 있다"를 정확한 노드
하나로 바꿀 방법이 없습니다. 그래서 `EvacGraph.kt`의 `approxNodeForZone()`이 각 zone의 간선
양끝 중 하나를 "대략 이 근처"로 골라서 그 노드 이름을 `current_location`으로 보냅니다(9개
zone이 모두 다른 노드에 매핑되어 겹치지 않습니다). **근사치이지 실제 위치가 아닙니다** — 나중에
관리자 웹/서버 쪽에 zone별 정확한 QR 좌표가 생기면, 그 좌표를 그대로 보내는 쪽으로 바꾸는 게
더 정확합니다.

**응답**: `{"start": ..., "route": ["C","A","TL"], "blocked_zones": ["zone_A2"]}` — `route`는
좌표가 아니라 **노드 이름의 나열**입니다. `blocked_zones`는 이번 계산에서 실제로 우회한 zone_id
목록. 도달 가능한 출구가 없으면 404, `current_location`을 노드로 못 바꾸면 400.

**위험 구역 판정**: 위험(`fixed_sensor_reading.status = "위험"`)인 zone_id가 있으면, 그 zone가
덮는 간선 하나를 통째로 그래프에서 제거하고 우회시킵니다(`resolve_danger_edges`). 노드 자체는
막히지 않으므로 — 시작점으로 탭 못 하는 노드는 없습니다. 도달 가능한 출구가 아예 없을 때만
404가 옵니다.

**앱 처리**: `EvacGraph.kt`가 이 지도(노드 12개, zone-간선 매핑)를 그대로 갖고 있고,
`EvacScreen.kt`가 admin 웹과 같은 3×3 카드 그리드로 그립니다(폐쇄 카드는 빗금 + "폐쇄" 배지,
로봇 핀은 `robot_event.location`과 가장 가까운 zone 카드 위에 표시). 카드를 탭하면
`approxNodeForZone()`으로 얻은 노드 이름을 그대로 `current_location`으로 보내고,
`MonitoringRepository.fetchEvacuationRoute()` / `EvacRouteState.Loaded`가 `route`(노드 이름
리스트)와 `blockedZones`를 담아 UI에 노출합니다.

## 5. 로그인 (`/api/auth/register`, `/api/auth/login`)

`ui/screens/LoginScreen.kt` + `ui/viewmodel/AuthViewModel.kt`에서 실제로 붙였습니다. 회원가입/로그인
둘 다 `role: "staff"`로 고정해서 보냅니다 (관리자 웹용 role 선택지는 이 앱에 노출 안 함).
서버가 토큰을 안 주기 때문에, 로그인 성공 시 받은 `{id, username, role}`을 `SessionPrefs`
(SharedPreferences)에 그대로 저장해서 "로그인 상태"로 취급합니다 — 앱을 껐다 켜도 다시 로그인
안 물어보고, 홈 화면 "로그아웃" 버튼을 눌러야 지워집니다. 나중에 진짜 토큰 기반 인증을 붙이게
되면 `SessionPrefs`/`AuthViewModel` 이 두 곳만 고치면 됩니다.

## 6. MariaDB 스키마 (제공된 코드 기준 실제 컬럼)

```sql
-- robot_event, robot_sensor, robot_gas_result, fixed_sensor_reading, fixed_sensor_baseline,
-- fire_event, users — database.py/auth.py의 INSERT 문에서 확인되는 컬럼 그대로입니다.
-- 정확한 컬럼 타입/제약조건은 실제 CREATE TABLE 문을 공유해주시면 이 문서에 반영하겠습니다.
```
