#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
// 가스 탐지 로봇 — 구동부 통합 제어 (ESP32)
// 8/19 개정 — WiFi/MQTT 전면 제거, 라파를 유일한 MQTT 창구로 삼음.
//
//   이유: ESP32 자체 WiFi가 로봇 위치(맵 안, 금속 부품 근처)에서
//   신호가 약해 MQTT 연결이 계속 끊기고, 그 재연결 시도가 loop()를
//   블로킹해서 모터 제어까지 같이 멈추는 문제가 있었다.
//   라파는 지금까지 MQTT가 안정적이었으므로, ESP32는 UART로만
//   상태/로그를 라파에 보내고, 라파가 그걸 대신 MQTT로 발행한다.
//   긴급정지도 라파가 MQTT robot/1/cmd 를 구독해서 UART로 전달한다.
//
//   ESP32 -> 라파 (UART, 이 방향도 검증 완료):
//     "STATE:{json}\n"   -- publishState() 대체
//     "MARK:{text}\n"    -- publishMark() 대체
//   라파 -> ESP32 (UART, 기존과 동일):
//     "POS:{node},TVEC:{deg}\n" -- 위치/목표방향 지속 스트리밍
//     "CMD:s\n" / "CMD:g\n"      -- 긴급정지 / 디버그 재개 (라파가 MQTT에서 중계)
//
// 배선 검증 완료:
//   좌: RPWM=32, LPWM=33, 엔코더 A=18,B=19  (양수=전진 확인됨)
//   우: RPWM=25, LPWM=26, 엔코더 A=16,B=17  (양수=전진 확인됨)
//   라인센서: L=34, C=35, R=36 (디지털, HIGH=검정)
//   라파 UART: RX=GPIO13, TX=GPIO14, 양방향 검증 완료 (2026-08-19)
// ═══════════════════════════════════════════════════════════════

// ── 라파 UART 설정 ────────────────────────────────────
HardwareSerial RpiSerial(1);
constexpr int RPI_RX_PIN = 13;
constexpr int RPI_TX_PIN = 14;
constexpr unsigned long RPI_BAUD = 115200;

// ── 핀 정의 (실측/검증 완료) ──────────────────────────
constexpr int L_RPWM = 32, L_LPWM = 33;   // BTS7960 #1 (좌)
constexpr int R_RPWM = 25, R_LPWM = 26;   // BTS7960 #2 (우)
// R_EN / L_EN 은 배선표대로 3.3V 하드와이어 고정

constexpr int ENC_L_A = 18, ENC_L_B = 19;
constexpr int ENC_R_A = 16, ENC_R_B = 17;

constexpr int LS_L = 34, LS_C = 35, LS_R = 36;   // 디지털 입력 (검정=HIGH)

// 8/20 변경 -- 20kHz -> 5kHz.
// 오른쪽 엔코더가 손으로 굴릴 땐 카운트되는데 모터 구동 중에만 0 이 되는
// 증상이 확인됐다. BTS7960 의 고주파 스위칭 노이즈가 엔코더 신호선에
// 실려 ISR 을 교란하는 것으로 보인다. 주파수를 낮추면 노이즈 에너지가
// 줄어든다. 대신 모터에서 가청 소음이 조금 난다.
// 근본 해결은 배선(엔코더선을 모터 전원선에서 분리, 트위스트 페어,
// 모터 단자에 0.1uF) 이고 이건 완화책이다.
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RES  = 8;       // 0~255
// 8/20 실측 확정 -- 로봇을 손으로 1m 밀어서 측정.
//   왼쪽 1405 카운트 -> 1405 x 20.42 / 100 = 286.9
//   오른쪽 1485 카운트 -> 1485 x 20.42 / 100 = 303.2
//   평균 295
// 사양표의 1320 CPR 은 A·B 양상 4체배 기준인데 이 코드는 A상 상승 엣지만
// 세므로 실제 계수는 그 1/4 수준이다. 이론값 330 과 11% 차이가 나지만,
// 실측은 기어비·바퀴 유효지름(타이어 눌림)·계수 방식을 모두 흡수하므로
// 실측을 쓴다. rpm 표시뿐 아니라 CM_PER_PULSE 를 통해 라인폭 판정의
// 기준자 역할도 하는 값이다.
constexpr float CPR = 295.0;      // JGB37-520 (실측)

// ── 8/20 추가 : 엔코더 사용 여부 스위치 ───────────────────
// 오른쪽 엔코더가 모터 구동 중에만 카운트를 못 내는 문제가 남아 있다.
// (손으로 굴리면 정상, 핀 이전·PWM 저주파·ISR 디바운스로도 미해결)
// 그 상태로 속도 PID 를 돌리면 "목표 50, 실측 0" 으로 보고 오른쪽 PWM 을
// 255 까지 밀어올려 로봇이 계속 왼쪽으로 휜다.
//
// 8/20 복귀 -- 오른쪽 엔코더 A상 접촉 불량이 해결되어 폐루프로 되돌린다.
// (원시 핀 감시에서 R_A 가 1 로 고정돼 있던 것이 0<->1 로 바뀌고 카운트가
//  실제로 증가하는 것을 확인함)
//
// false 로 두면 속도 PID 를 건너뛰고 라인 PD 출력을 PWM 으로 직접 보낸다
// (개루프). 엔코더를 전혀 쓰지 않으므로 한쪽이 죽어 있어도 주행한다.
// 대신 부하·전압 변화에 따라 속도가 변하고 좌우 특성 차이를 보정하지
// 못한다. 엔코더가 또 말썽이면 false 로 내려서 임시 주행할 수 있다.
constexpr bool USE_ENCODER_PID = true;

// 개루프일 때 base rpm 을 PWM 으로 바꾸는 환산 계수.
// JGB37-520 은 12V 에서 정격 250rpm 이고 PWM 은 0~255 이므로,
// 대략 PWM = rpm * 255/250 ≒ rpm * 1.0 이 출발점이다. 실측하며 조정할 것.
constexpr float OPENLOOP_RPM_TO_PWM = 1.0f;
// 개루프 최소 듀티 -- 이보다 낮으면 모터가 정지 마찰을 못 이긴다.
// 실측: PWM 45 는 안 돌고 75~100 은 돌았다.
constexpr int OPENLOOP_MIN_PWM = 70;

// 개루프 전용 기본 속도. 폐루프의 base(50rpm) 를 그대로 쓰면 환산 PWM 이
// 40~60 이라 전부 최소 듀티 70 으로 올라가 좌우가 같아진다 -- 조향이
// 통째로 사라진다. PWM 눈금에서 최소 듀티보다 충분히 위여야 좌우 차이가
// 살아남으므로 별도 값을 둔다. (110 이면 err=1.0 에서 L=100 R=120)
constexpr float OPENLOOP_BASE = 110.0f;
constexpr float OPENLOOP_TURN_INNER = 64.0f;   // 110 x (29/50), 코너 안쪽

// ── 오도메트리 보정값 (교차로 폭 판별용) ──
constexpr float WHEEL_DIAMETER_MM = 65.0;
constexpr float WHEEL_CIRC_CM = (3.14159265f * WHEEL_DIAMETER_MM) / 10.0f;
constexpr float CM_PER_PULSE = WHEEL_CIRC_CM / CPR;

// ── 튜닝 파라미터 (실측 반영) ──
struct Tuning {
  // 8/20 재재조정 -- 앞서 CPR 변경(1320->295)에 맞춰 45 -> 10 으로 줄였는데
  // 이는 잘못된 계산이었다. kp 와 base 는 둘 다 rpm 단위라 같이 바꾸거나
  // 둘 다 그대로 둬야 하는데, base 는 50 그대로 두고 kp 만 줄여서
  // 조향 비율이 60% -> 12.5% 로 떨어졌다. 상한(base*0.6=48)의 5분의 1만
  // 쓰는 셈이라, 회전 직후 라인이 옆으로 벗어나도 못 따라가 line_lost 가 났다.
  // base 가 숫자 그대로 유지됐으므로 kp 도 원래 값으로 되돌린다.
  // 8/20 재조정 -- kp 45->22, kd 3.0->0.8.
  // 45/3.0 은 CPR=1320 시절(보고 rpm 이 실제의 1/4.5)에서 물려받은 값이라
  // 실제 눈금에서는 과했다. 3센서 방식은 오차가 0.5 단위로 뚝뚝 끊기는데,
  // dt=0.02 에서 오차가 0.5 바뀌면 dErr=25 이고 kd=3.0 이면 미분항만 75 라
  // 모든 전환에서 상한(base*0.6=48)에 걸렸다. 결과적으로 어느 방향으로
  // 얼마나 벗어나든 항상 최대 조향이 나가 좌우 진동이 됐다
  // (실측 rpmL 27/rpmR 92 <-> rpmL 95/rpmR 24 반복).
  // 그 진동으로 라인을 비스듬히 가로지르며 111 이 반복 발생했고,
  // 직선 복도에서 intersection 이 7회 넘게 잡혀 회전 게이팅이 무너졌다.
  // 8/21 재조정 -- 22 -> 32.
  // 22 로는 완전히 한쪽으로 벗어난 상태(error 1.0)에서도 조향이 22 밖에
  // 안 나왔다. 조향 상한이 base*0.6 = 48 인데 절반도 못 쓴 것이다.
  // 이미 라인을 벗어난 상황에서도 완만한 곡선으로 돌아오니, 그 사이 계속
  // 전진해 더 멀어졌다(실측 ML 복귀에서 000 -> line_lost 로 이어짐).
  // 32 면 error 1.0 에서 조향 32, error 0.5 에서 16 이 나온다.
  // 8/21 재조정 (3차) -- 32 -> 24.
  // 22 는 약해서 못 돌아왔고(1차), 32 는 반대로 과했다.
  // 실측 rpm 이 23.7/115.3, 125.4/27.1, 54.2/3.4 처럼 양 끝단을 오갔다.
  // 목표 범위가 base-48 ~ base+48 = 32~128 인데 그 경계에 계속 붙어 있다는
  // 것은 조향이 상한에서 포화됐다는 뜻이다 -- 전형적인 과도 이득이다.
  // 24 면 error 1.0 에서 조향 24, 0.5 에서 12 로 상한(48)에 여유가 생긴다.
  float kp = 24.0;
  float kd = 0.8;
  // 8/21 추가 -- 미분항 저역통과 계수 (0~1, 작을수록 강하게 눌림).
  // 1.0 으로 두면 필터가 꺼져 8/20 이전 동작과 같아진다.
  // 8/21 재조정 -- 0.3 -> 0.5.
  // 0.3 은 진동은 확실히 잡았지만 되돌아오는 힘까지 같이 깎았다.
  // 010->110 전환 순간의 조향이 31 에서 17 로 줄어, 라인 근처에서 떨던
  // 것이 아예 라인을 벗어나는 쪽으로 바뀌었다(로그상 010<->110 이
  // 010 -> 000 -> 100 으로 변함). 0.5 로 절충한다.
  float dErrAlpha = 0.5;
  // 8/20 상향 -- CPR 을 295 로 바로잡으면서 base 50rpm 이 실제 50rpm(약
  // 17cm/s)이 되어 이전보다 크게 느려졌고, 저속에서 차체 떨림이 늘었다.
  // 80rpm(약 27cm/s)으로 올린다. turnInner 는 같은 비율(58%)로 유지.
  // 8/21 재조정 -- 80 -> 65.
  // 센서가 3개뿐이라 오차가 0 / ±0.5 / ±1.0 으로 뚝뚝 끊긴다. 사실상
  // 벤벤(bang-bang) 제어라 진동 진폭이 "속도 x 이득"에 비례한다.
  // 이득(kp)만 낮추면 복귀가 느려지므로 속도도 같이 낮춘다. 느릴수록
  // 한 번의 오차 판정 사이에 덜 움직여 오버슈트가 줄고, QR 인식률도
  // 같이 올라간다(모션 블러 감소).
  // 시연에서 속도가 아쉬우면 이 값부터 5씩 올리며 확인할 것.
  float base = 65.0;         // 직진 목표 rpm
  // 8/20 추가 -- INIT(첫 QR 인식 전) 전용 저속.
  // TR 에서 출발했는데 (0,12)/(0,11)을 모두 놓치고 (0,10)을 읽어,
  // 이미 B 교차로를 지난 뒤에야 위치가 확정됐다. 그 상태로 경로를 세우니
  // 이미 지나온 B 에서 회전 명령이 나가 곧바로 라인을 벗어났다.
  // 위치 확정은 임무 전체의 기준점이므로, 여기서만은 느리게 가서
  // QR 인식 기회를 충분히 준다.
  // 8/20 재조정 -- 40 -> 60. 40rpm 에서는 좌우 기동 문턱 차이가 드러나
  // 왼쪽이 6~24rpm 까지 떨어지며 크게 휘었다(실측 rpmL 6.8 / rpmR 44.1).
  // 60rpm(약 20cm/s)이면 평상시(80)의 3/4 이라 인식 여유는 늘면서
  // 데드밴드는 피한다.
  float initBase = 60.0;
  // 8/20 추가 -- 구역 진입(ZONE_ENTER) 전용 저속.
  // 라파는 이때 POS:{구역}_enter 로 보낸다.
  // 정지 후보 QR 을 놓치면 그만큼 뒤로 밀려 멈추는데, C 에서 B2 로 들어가며
  // (6,5)/(6,6)을 놓치고 (6,7)에서 멈춘 사례가 있었다. (6,7)은 D 바로
  // 앞이라 구역 중앙이 아니어서 가스 측정 지점으로 치우친다.
  // 느리게 가면 QR 인식 기회가 늘어 중앙에서 멈출 확률이 올라간다.
  // 8/20 재조정 -- 50 -> 65. 50 은 좌우 기동 문턱에 너무 가깝다.
  // (INIT 을 40 으로 내렸을 때 왼쪽이 6~24rpm 까지 처지며 크게 휜 전례)
  float zoneEnterBase = 65.0;
  float turnInner = 46.0;    // 코너 안쪽 목표 rpm
  // 8/20 -- 제자리 회전 기준. U턴 실측(제자리, pivot=100 에서 2500ms 에
  // 720도)으로 역산하면 288도/초이고, 75 면 약 216도/초다.
  // 75 는 searchPivot 으로 실제 도는 것이 확인된 값이라 기동 문턱도 안전하다.
  int   pivot = 75;          // 회전 PWM (제자리, 좌우 반대)

  // 8/20 재조정 -- 4.0 -> 12.0.
  // stopbarMinCm 을 99.0 으로 올려 STOPBAR 를 끄면서, 4~99cm 로 측정된
  // 교차선이 교차로도 정지선도 아닌 것으로 조용히 버려지는 사각지대가
  // 생겼다. 실측(B 교차로)에서 ls:111 이 찍혔는데 intersection 이 안 나와
  // 회전이 통째로 누락됐다.
  // 폭이 부풀려지는 이유는 두 가지다. 거리 계산이 좌우 평균이라 조향이
  // 크면 실제 전진보다 길게 잡히고(실측 rpmL 50.8 / rpmR 81.4), 교차선을
  // 비스듬히 지나면 전 센서 검정 구간 자체가 길어진다.
  // STOPBAR 가 꺼져 있으므로 상한을 넉넉히 잡아도 충돌하지 않는다.
  // 8/20 재조정 -- 12.0 -> 8.0.
  // 12.0 은 진동이 심하던 시절의 부풀려진 폭에 맞춘 값이라, 라인을
  // 비스듬히 스치는 것까지 교차로로 받아들였다. PD 를 잡았으니 실제
  // 교차선 폭에 가깝게 좁힌다.
  float crossMaxCm  = 8.0;
  // 8/21 추가 -- 교차선 폭의 하한.
  // 지금까지 조건이 width > 0.1 이라, 루프 한 번(20ms, base=80rpm 기준 약
  // 0.54cm)만 111 이어도 교차로로 인정됐다. 라인을 비스듬히 스칠 때 나오는
  // 111 이 정확히 이 길이라 가짜 교차로의 주 원인이었다.
  // 진짜 교차선은 로봇이 가로질러 지나므로 최소 테이프 폭(약 2cm) 만큼은
  // 111 이 유지된다. 1.5cm 를 하한으로 두면 한 루프짜리 스침은 걸러지고
  // 진짜 교차선은 여유 있게 통과한다.
  // 주의: 이 값을 올릴 때는 실측 폭 로그(cross_width)를 먼저 볼 것.
  float crossMinCm  = 1.0;

  // 8/21 재설계 (3차) -- 라인 상실(000) 시 좌우 스윕 탐색.
  //
  // 1·2차는 "마지막으로 본 방향으로 한쪽으로만" 돌았다. 두 가지 한계가 있다.
  //   (1) 010 에서 곧장 000 으로 빠지면 오차가 0 이라 방향 근거가 없다.
  //       낡은 lastNonZeroError 를 쓰면 엉뚱한 쪽으로 돈다.
  //   (2) 한쪽으로만 돌면 반대편에 라인이 있을 때 영영 못 찾는다.
  //       실측(ML 복귀)에서 100 -> 000 -> 100 -> 000 을 반복하며 계속
  //       왼쪽으로만 돌다가 못 찾고 line_lost 로 끝났다.
  //
  // 이제 제자리에서 좌우로 번갈아, 구간을 점점 길게 잡아 훑는다.
  //   1구간: dir 방향 250ms   (약 49도)
  //   2구간: 반대  방향 500ms (약 98도, 시작점 기준 -49도)
  //   3구간: 다시  방향 750ms (약 +98도)
  //   ...  이런 식으로 진폭이 커진다
  // 8/21 재조정 -- 150 -> 250ms.
  // 150ms(첫 스윙 29도)로는 흔드는 폭이 좁아, U턴 직후처럼 여러 라인이
  // 가까이 있는 자리에서 엉뚱한 라인(끝라인 등)을 먼저 물었다.
  // 첫 스윙을 49도로 넓혀 복도 라인까지 훑도록 한다.
  // 전진은 하지 않는다 -- 제자리라 벽에 더 다가가지 않고, 라인이 앞에
  // 있는지 좌우로 확인만 한다.
  //
  // 탐색을 끝내는 조건은 "라인을 찾음"(센서)이다. 구간 길이만 시간으로
  // 키우는데, 이건 판정이 아니라 훑는 패턴이라 편차에 둔감하다.
  //
  // searchPivotRpm 을 75 로 둔 이유 -- U턴 피벗과 같은 값이다. 그 rpm 에서는
  // 좌우 모터가 목표를 잘 따라간다는 것이 실측으로 확인됐다(목표 -75 에
  // 실제 -71.2). 그보다 낮추면 역회전 쪽이 기동 문턱에 걸려 회전이 느려진다.
  float searchPivotRpm = 75.0;
  unsigned long searchLegMs = 250;
  // 8/21 추가 -- 탐색 진입 지연. 이게 없으면 못 쓴다.
  // 정상 주행 중에도 ls 가 010 -> 000 -> 100 처럼 한두 루프씩 000 을 스친다
  // (라인 폭이 2~3cm 로 들쭉날쭉해 좁은 구간에서 겹침 여유가 0 이 된다).
  // 그 한 루프에 곧바로 제자리 회전을 걸면 주행이 통째로 망가진다.
  // 이 시간 동안 연속으로 라인이 없어야 "진짜 놓쳤다"로 보고 탐색을 켠다.
  // 그 전까지는 평상시 PD 를 그대로 쓴다(오차는 직전 값이 유지된다).
  unsigned long searchStartDelayMs = 250;
  // 8/20 재수정 -- 99.0 으로 올려 STOPBAR 판정을 사실상 끈다.
  // 이 맵에는 실제 정지선이 없는데, 끝라인을 지날 때마다 폭이 4cm 를 넘어
  // arrive -> PH_STOP 으로 빠졌다. PH_STOP 은 CMD:g 없이는 못 나오고
  // 라파는 그 마크를 처리하지 않아, 로봇이 그 자리에 멈춘 채 라파의
  // 15초 타임아웃까지 방치됐다(B3 실측).
  // 정지선을 실제로 쓰게 되면 그때 실측 폭에 맞춰 되살릴 것.
  float stopbarMinCm = 99.0;
  unsigned long crossingAdvanceMs = 500;
  int   pivotTimeoutMs = 900;    // 8/19 재정정: 2500은 과회전(최대 160도) 유발.
                                   // 이제 재탐색(TURN_SEARCH)이 있으니 1차 시도는
                                   // 짧게 끊고 빨리 저속 재탐색으로 넘기는 게 맞음.
  // 8/21 -- 800 -> 3500. 좌우 스윕 탐색이 진폭을 키워가며 훑을 시간.
  // 탐색 중에는 전진이 0(제자리 회전)이라 시간을 늘려도 더 멀어지지 않는다.
  // 구간 길이를 250ms 로 넓혔으므로 250+500+750+1000 = 2500ms 로 네 구간을
  // 훑고, 남는 1초로 다섯 번째 구간 일부까지 간 뒤 못 찾으면 정지한다.
  unsigned long lineLostTimeoutMs = 3500;

  // 8/19 추가 -- 우회전 전용 시간 기반 회전. 센서 패턴 판단 없이 그냥 이
  // 시간만큼 돈다. 실측하면서 값을 맞춰야 함 (90도 근처가 목표).
  // 8/20 재조정 -- 420 -> 300.
  // 420ms 는 90도에 맞춘 값인데, 실측에서 회전이 끝난 시점에 이미 새 라인을
  // 지나쳐 있어 TURN_ALIGN 이 곧바로 반대로 뒤집었다(align_flip_left 직후
  // align_done ms=460). 갔다가 돌아오느라 총 880ms 를 쓰고 자세도 흐트러진다.
  // 일부러 덜 돌려(약 55도) 나머지는 TURN_ALIGN 이 같은 방향으로 채우게 한다.
  // 그러면 뒤집기가 사라지고 정렬이 단조롭게 끝난다.
  unsigned long rightTurnMs = 300;


  float turnAngleThresholdDeg = 30.0;

  // 8/19 추가 -- 회전 중 라인을 놓쳤을 때 반대 방향으로 계속 돌며 재탐색
  // 8/20 수정 -- 45 는 "느린" 게 아니라 아예 안 움직이는 값이었다.
  // L298N 내부 강하 약 2V 에 듀티 45/255(18%)를 곱하면 모터 실효 전압이
  // 1V 남짓이라 정지 마찰을 못 이긴다. 실측 로그에서 재탐색 중 rpm 이
  // 0.8~3.0 사이를 부호까지 뒤집으며 오갔는데, 이는 회전이 아니라 진동
  // 수준의 지터다. 결국 라인을 못 찾고 searchMaxMs(4초)에 걸려
  // pivot_search_giveup 으로 끝났다.
  // 75 는 실제로 도는 최소선. 여전히 pivot(100)보다 느려 훑기 목적은 유지된다.
  int   searchPivot = 75;         // 재탐색 속도(느리게, 단 실제로 도는 값)
  // 8/20 추가 -- CPR 실측 주행 PWM. 라인 추종 없이 좌우 동일 출력으로
  // 직진만 한다. 느릴수록 바퀴 미끄러짐이 줄어 측정이 정확해진다.
  int   calibPwm = 90;
  unsigned long searchMaxMs  = 4000;   // 전체 재탐색 포기 시간
  // 8/20 추가 -- 회전 단계 전용 값.
  // lineLostDebounceMs(150)는 원래 '직진 중' 라인을 잃었는지 보려고 잡은
  // 값이다. 그런데 교차로에서 피벗할 때는 가로선을 벗어나 세로선에 닿기
  // 전까지 센서가 아무것도 못 보는 구간이 정상적으로 존재한다.
  // 회전 각속도가 약 145도/초이므로 150ms 는 22도밖에 안 되어, 그 공백이
  // 조금만 길어도 "놓쳤다"고 오판하고 TURN_SEARCH 로 빠졌다. 그러면 덜 돈
  // 채 라인 추종으로 복귀해 곧바로 이탈한다(실측에서 좌회전마다 재현).
  unsigned long turnLostDebounceMs = 400;  // 회전 중 000 허용 시간 (약 58도)
  // TURN_B 가 이 시간 전에는 010 을 완료로 인정하지 않는다. 회전 초반에
  // 가운데 센서만 스치는 순간을 완료로 오인하는 것을 막는다.
  unsigned long turnMinPhaseMs = 250;
  // 8/20 추가 -- TURN_ALIGN.
  // 회전을 마친 뒤 라인에 제대로 안착했는지 확인하는 단계. 000 이
  // alignFlipMs 이상 지속되면 회전 방향을 뒤집어 반대로 훑는다. 이렇게
  // 오가면 회전이 덜 돌았든 지나쳤든 결국 라인을 다시 잡는다.
  // 기존에는 시간 기반 우회전이 끝나면 곧바로 라인 추종으로 넘어가서,
  // 각도가 조금만 어긋나도 곧바로 line_lost 로 멈췄다.
  unsigned long alignFlipMs = 250;    // 000 이 이만큼 지속되면 방향 전환
  unsigned long alignMaxMs  = 6000;   // 안전 상한 -- 넘으면 정지
  // 8/20 추가 -- 이 시간이 지나면 가운데가 아니어도(가장자리만 걸려도) 완료.
  // 가운데를 못 찾고 무한히 뒤집는 것을 막는 안전판이다.
  unsigned long alignEdgeFallbackMs = 1200;
  // 8/20 추가 -- 회전 완료 후 교차로 감지 재무장까지의 이동 거리.
  // 회전을 마쳐도 로봇은 아직 교차로 위에 있어, 앞으로 나가며 가로선의
  // 나머지를 다시 밟는다. 그 정상적인 111 이 새 교차로로 오인되어
  // FORCED_ADVANCE(500ms 직진)가 걸리고, 그동안 방금 잡은 세로선을
  // 벗어나 line_lost 가 났다(실측 재현).
  // 이 거리를 지날 때까지 라인폭 판정을 무시한다.
  float crossRearmCm = 10.0;

  // 8/20 재추가 -- U턴 시퀀스 (끝라인 + 짧은 연장 직선 배치 기준).
  //   1) TURN_U_CLEAR : 라인 추종 전진. 끝라인(111)을 벗어난 뒤 연장선
  //                     끝의 000 을 만나면 피벗으로.
  //   2) TURN_U_PIVOT : 제자리 회전. 라인이 다시 잡히면 종료.
  // 피벗 각속도는 U턴 실측(제자리, pivot=100 에서 2500ms 에 720도)에서
  // 288도/초였고, 지금 pivot=75 이므로 약 216도/초 -> 180도에 약 830ms.
  // 최소 시간은 그 80% 정도로 두어 180도 근처까지 감지를 막는다.
  // 8/20 개정 -- 진행용 타임아웃 전면 제거.
  // 끝라인 통과, 연장선 끝 도달, 초기 QR 탐색은 거리도 속도도 그때그때
  // 다른데 시간으로 끊으니 순서를 다 마치기 전에 멈춰버렸다.
  // 이제 센서가 실제로 조건을 만족할 때까지 기다린다.
  // 남는 것은 아래 uTurnMaxPivotMs(피벗 안전 상한) 하나뿐이다.
  unsigned long uTurnClearDebounceMs = 150;    // 111 해제 확인용 (오탐 방지)
  unsigned long uTurnLostDebounceMs  = 200;    // 000 확인용 (오탐 방지)
  // 8/20 재조정 -- 650 -> 800.
  // 실측 rpm 70 기준 각속도는 축간거리에 따라 152~228도/초로, 180도에
  // 791~1187ms 가 걸린다. 650ms 는 그 어느 경우에도 180도에 못 미쳐,
  // 열리자마자 라인 끝자락을 스치고 종료됐다(실측 uturn_done ms=660,
  // 약 120도). 그 자세로 전진하니 곧바로 line_lost.
  unsigned long uTurnMinPivotMs      = 700;    // 이 시간 전에는 라인 감지 무시
  // 유일하게 남긴 안전 상한. 라인이 물리적으로 없는 곳에 놓이면 무한히
  // 도는 것을 막는다. 180도에 약 1초이므로 6초면 정상 동작을 방해하지 않는다.
  unsigned long uTurnMaxPivotMs      = 6000;   // 피벗 안전 상한
  // 8/20 추가 -- 피벗 종료 판정 강화.
  // 시간만으로는 축간거리·전압·마찰에 따라 편차가 커서 매번 어긋난다.
  // 가운데 센서가 라인을 물고, 그 상태가 이만큼 지속돼야 완료로 본다.
  // 가장자리 센서만 걸리는 스침(100/001)은 인정하지 않는다.
  // 8/20 재조정 -- 60 -> 20.
  // 센서열 접선속도가 약 25cm/s 라 폭 2cm 라인을 79ms 만에 지나간다.
  // 60ms 유지를 요구하면 여유가 거의 없어 진짜 180도 지점을 놓치고,
  // 한참 지난 뒤에야 잡혔다(실측 ms=1260, 약 229도 과회전).
  // 한 루프(20ms)만 확인하고, 정밀 정렬은 TURN_ALIGN 에 맡긴다.
  unsigned long uTurnLineHoldMs      = 20;
  unsigned long lineLostDebounceMs = 150;  // 8/19: 000이 이만큼 지속돼야
                                             // "진짜 놓침"으로 인정 (찰나의
                                             // 스침으로 오판해서 반대로
                                             // 꺾어버리는 것 방지)
} tune;

// ── 라파 스트림 수신 상태 ──────────────────────────────
// 프로토콜: "POS:<node_id>,TVEC:<angle_deg>\n"
String latestPos = "";
float  latestTVecDeg = 0;
// 8/20 추가 -- 라파가 보내는 NEAR 플래그. "지금 다음 노드 근처다"라는 뜻.
// 그동안 전송만 되고 파싱조차 안 했다. 복도 한가운데서 111 이 잘못
// 잡히면 대기 중이던 회전각이 그 자리에서 실행돼 엉뚱하게 돌아버리는데,
// 이 값으로 걸러낸다.
bool   latestNear = false;
unsigned long lastRpiMsgTime = 0;
constexpr unsigned long RPI_STREAM_TIMEOUT_MS = 2000;

// ── 상태 머신 ────────────────────────────────────────
enum class RobotState { STOPPED, RUNNING, PH_STOP, TEST, STALL };
volatile RobotState state = RobotState::STOPPED;

enum class XPhase { NONE, FORCED_ADVANCE, TURN_A, TURN_B, TURN_SEARCH, TURN_R_TIMED,
                    TURN_ALIGN, TURN_U_CLEAR, TURN_U_PIVOT };
XPhase xPhase = XPhase::NONE;

unsigned long xClearStart = 0;
unsigned long xTurnStart = 0;
unsigned long lineLostSince = 0;

// 8/19 추가 -- TURN_SEARCH(좌우 훑기 재탐색) 상태
unsigned long searchStartTime = 0;   // 전체 재탐색 시작 시각
bool searchSweepRight = false;       // 지금 오른쪽으로 훑는 중인지
char searchTarget = '\0';            // '1'=111 찾는 중(TURN_A 대체), '0'=010 찾는 중(TURN_B 대체)

// 8/19 추가 -- 회전 2단계(TURN_B) 완료 판정을 더 정확하게.
// 우회전이면 001 을 먼저 거친 뒤 011 또는 010, 좌회전이면 100 을 먼저
// 거친 뒤 110 또는 010 이 나와야 "진짜 완료"로 인정한다. 라인이 완전히
// 사라질 때(000)만 반대로 재탐색하고, 그 전엔 계속 원래 방향으로 돈다.
unsigned long allLostSince = 0;   // 8/19: 000이 얼마나 지속됐는지 (찰나의 스침 방지)

char pendingTurn = 'X';   // 'S'=직진, 'L'=좌, 'R'=우, 'X'=정지

// ── 엔코더 카운터 ─────────────────────────────────────
volatile long encLcount = 0, encRcount = 0;

// 8/20 추가 -- CPR 실측 모드 (CMD:c).
// 메인 루프가 매 주기 encLcount 를 읽고 0으로 비우기 때문에 누적값을
// 따로 모아야 한다. 바퀴를 손으로 정확히 한 바퀴 돌렸을 때의 누적
// 카운트가 곧 이 코드의 계수 방식 기준 실제 CPR 이다.
// 8/20 재추가 -- U턴 시퀀스 전용 상태 (라파 통합판이 CMD:u 를 쓴다).
// uturnActive : CMD:u ~ 라파의 CMD:s/CMD:g 까지. 이 동안 라인폭 판정을
//   통째로 무시한다. 복귀하며 끝라인을 안쪽으로 통과할 때 111 이 뜨는데,
//   정지선으로 잡으면 PH_STOP 으로 빠지고 교차로로 잡으면 없는 교차로가
//   카운트되어 라파의 회전각 게이팅이 어긋난다.
// uturnArmed  : 끝라인(111)에서 확실히 벗어난 뒤에만 000 감시를 시작.
bool uturnActive = false;
bool uturnArmed = false;
unsigned long uturnClearSince = 0;
unsigned long uturnLineSince = 0;   // 8/20: 피벗 중 가운데 센서가 라인을 문 시각
bool uturnSaw111 = false;           // 8/20: U턴 1단계에서 끝라인(111)을 실제로 봤는가

// 8/20 추가 -- TURN_ALIGN(회전 마무리 정렬) 전용 상태.
// 000 에서 포기하지 않고 방향을 뒤집어 가며 라인을 다시 찾는다.
bool alignSweepRight = false;      // 지금 훑는 방향
unsigned long alignStart = 0;      // 단계 진입 시각 (최소/최대 시간용)
unsigned long alignLostSince = 0;  // 000 이 시작된 시각 (방향 전환 판정용)
unsigned long alignFlipWindow = 0; // 8/20: 현재 훑기 폭(ms). 뒤집을 때마다 넓어진다

// 8/20 추가 -- 회전 후 교차로 감지 재무장까지 남은 거리(cm).
// 0 보다 크면 라인폭 판정을 무시하고, 주행하며 조금씩 줄어든다.
float crossRearmRemainCm = 0;

// 8/20 이동 -- 원래 updateLineWidth() 바로 위에 있었으나, UART 파서에서
// INIT 종료 시 초기화하느라 더 앞에서 참조하게 되어 선언을 끌어올렸다.
// (그대로 두면 'wasAllBlack was not declared in this scope' 컴파일 오류)
bool wasAllBlack = false;
float crossDistCm = 0;
// 8/21 추가 -- 마지막으로 측정된 전(全)검정 구간 폭(cm). 판정에는 쓰지 않고
// intersection 마크에 실어 보내기만 한다. crossMinCm/crossMaxCm 을 실측에
// 맞춰 조정하려면 진짜 교차선의 폭이 실제로 몇 cm 로 찍히는지 알아야 한다.
float lastWidthCm = 0;

bool calibMode = false;
long calibL = 0, calibR = 0;
unsigned long calibLastReport = 0;
unsigned long lastPulseTimeL = 0, lastPulseTimeR = 0;

// ── 라인 PD 상태 ─────────────────────────────────────
float prevLineError = 0;
// 8/21 추가 -- 라인 PD 의 미분항 저역통과 필터 상태.
// 3센서 방식은 오차가 0 / +-0.5 / +-1.0 으로 뚝뚝 끊긴다. dt=0.02 에서
// 오차가 0.5 바뀌면 dErr 이 25 로 튀어, kd=0.8 만으로도 미분항이 20 이 되어
// 비례항(11)의 두 배가 된다. 그것도 딱 한 루프만. 결과적으로 010<->110
// 전환마다 조향이 한 번씩 최대치로 튀었다가 다음 루프에 0 으로 꺼지는
// 임펄스가 되어 좌우 진동이 났다(실측 rpmL 33.9/rpmR 78.0 <-> 84.7/47.5).
// 그 진동으로 라인을 비스듬히 가로지르며 가짜 111 이 반복 발생했고,
// 그것이 이번 오회전 사고의 1차 원인이다.
// 미분값을 EMA 로 눌러 임펄스를 여러 루프에 퍼뜨린다. kd 자체는 그대로 둔다.
float lineDErrFilt = 0;
// 8/21 추가 -- 마지막으로 "어느 쪽으로 벗어났는지"가 확실했던 오차값.
// 라인을 완전히 놓친 뒤(000) 되돌아갈 방향을 정하는 데 쓴다.
// prevLineError 를 그냥 쓰면 안 되는 이유: 010(오차 0)에서 곧장 000 으로
// 빠지는 경우가 실제로 잦은데, 그때 prevLineError 는 0 이라 "직진"이 된다.
// 라인을 놓쳤는데 똑바로 가면 영영 못 찾는다.
float lastNonZeroError = 0;
// 8/21 추가 -- 라인 상실 시 좌우 스윕 탐색 상태.
//   lineSearchDir     : 지금 도는 방향 (+1 = 좌회전, -1 = 우회전)
//   lineSearchLegIdx  : 몇 번째 구간인가 (구간마다 길이가 길어진다)
//   lineSearchLegUntil: 이 구간이 끝나는 시각
unsigned long lineSearchLegUntil = 0;
int  lineSearchDir = 1;
int  lineSearchLegIdx = 0;
bool lineSearchActive = false;

// ── 속도 PID (내부 루프) ──────────────────────────────
struct PID {
  float kp, ki, kd;
  float integral = 0;
  float prevError = 0;
  float integralLimit = 150;
};
PID pidL = {2.2, 0.35, 0.0};
PID pidR = {2.2, 0.35, 0.0};

float computePID(PID &pid, float target, float actual, float dt) {
  float error = target - actual;
  pid.integral = constrain(pid.integral + error * dt, -pid.integralLimit, pid.integralLimit);
  float derivative = (error - pid.prevError) / dt;
  pid.prevError = error;
  float out = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  return constrain(out, -255.0f, 255.0f);
}

// ── ISR ──────────────────────────────────────────────
// 8/20 추가 -- ISR 디바운스.
// 스위칭 노이즈로 생기는 가짜 엣지를 걸러낸다. 무부하 최고 330rpm 에서
// 실제 펄스 간격이 616us 이므로, 150us 안에 다시 들어온 엣지는 노이즈로
// 본다. 이 값으로도 1356rpm 까지 셀 수 있어 여유가 4배 이상이다.
constexpr unsigned long ENC_DEBOUNCE_US = 150;
volatile unsigned long lastEncUsL = 0, lastEncUsR = 0;

void IRAM_ATTR encLeftISR() {
  unsigned long us = micros();
  if (us - lastEncUsL < ENC_DEBOUNCE_US) return;
  lastEncUsL = us;
  bool b = digitalRead(ENC_L_B);
  encLcount = encLcount + (b ? 1 : -1);
  lastPulseTimeL = millis();
}
void IRAM_ATTR encRightISR() {
  unsigned long us = micros();
  if (us - lastEncUsR < ENC_DEBOUNCE_US) return;
  lastEncUsR = us;
  bool b = digitalRead(ENC_R_B);
  encRcount = encRcount + (b ? 1 : -1);
  lastPulseTimeR = millis();
}

// ── 모터 구동 ──────────────────────────────────────────
void driveMotor(int pinRPWM, int pinLPWM, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(pinRPWM, speed); ledcWrite(pinLPWM, 0); }
  else            { ledcWrite(pinRPWM, 0); ledcWrite(pinLPWM, -speed); }
}
void driveLeft(int s)  { driveMotor(L_RPWM, L_LPWM, s); }
void driveRight(int s) { driveMotor(R_RPWM, R_LPWM, s); }
void stopAll() {
  driveLeft(0); driveRight(0);
  pidL.integral = 0; pidR.integral = 0;
}

float countsToRPM(long dCount, float dtSec) {
  return (dCount / CPR) / dtSec * 60.0;
}

// ── rpm 이동평균 ──
constexpr int RPM_AVG_N = 3;
float rpmLHistory[RPM_AVG_N] = {0};
float rpmRHistory[RPM_AVG_N] = {0};
int rpmHistIdx = 0;

float smoothRPM(float* history, float newVal) {
  history[rpmHistIdx % RPM_AVG_N] = newVal;
  float sum = 0;
  for (int i = 0; i < RPM_AVG_N; i++) sum += history[i];
  return sum / RPM_AVG_N;
}

// ── 라파로 상태/로그 전송 (UART 경유, 라파가 MQTT로 대신 발행) ──
void publishState(float rpmL, float rpmR, int l, int c, int r) {
  char buf[300];
  const char* stName =
    state == RobotState::RUNNING ? "RUNNING" :
    state == RobotState::PH_STOP ? "PH_STOP" :
    state == RobotState::TEST    ? "TEST" :
    state == RobotState::STALL   ? "STALL" : "STOPPED";
  snprintf(buf, sizeof(buf),
    "STATE:{\"state\":\"%s\",\"rpmL\":%.1f,\"rpmR\":%.1f,"
    "\"ls\":\"%d%d%d\",\"pos\":\"%s\",\"tvec\":%.1f,\"pendingTurn\":\"%c\","
    "\"rpiAlive\":%s}\n",
    stName, rpmL, rpmR, l, c, r,
    latestPos.c_str(), latestTVecDeg, pendingTurn,
    (millis() - lastRpiMsgTime < RPI_STREAM_TIMEOUT_MS) ? "true" : "false");
  RpiSerial.print(buf);
}

void publishMark(const char* label) {
  RpiSerial.printf("MARK:%s\n", label);
}

// ── 라파 UART 스트림 파싱 ────────────────────────────────
// 두 종류의 줄을 받는다:
//   "POS:<id>,TVEC:<deg>"  -- 위치/목표방향 지속 스트리밍
//   "CMD:s" / "CMD:g"       -- 라파가 MQTT robot/1/cmd 를 중계한 명령
void pollRpiSerial() {
  static String lineBuf = "";
  while (RpiSerial.available()) {
    char ch = (char)RpiSerial.read();
    if (ch == '\n') {
      lineBuf.trim();
      if (lineBuf.length() > 0) {
        Serial.printf("[RPI RAW] %s\n", lineBuf.c_str());

        if (lineBuf.startsWith("CMD:")) {
          char cmd = lineBuf.length() > 4 ? lineBuf.charAt(4) : '\0';
          if (cmd == 's') {
            // 긴급 정지
            state = RobotState::STOPPED;
            stopAll();
            xPhase = XPhase::NONE;
            xClearStart = 0;
            xTurnStart = 0;
            lineLostSince = 0;
            calibMode = false;     // 8/20: CPR 실측 모드 해제
            uturnActive = false;   // 8/20: U턴 억제 해제
            uturnArmed = false;
            publishMark("estop");
          } else if (cmd == 'g') {
            // 재시작 (디버그용)
            state = RobotState::RUNNING;
            lastPulseTimeL = lastPulseTimeR = millis();
            uturnActive = false;   // 8/20: U턴 억제 해제 (라파가 재개시킬 때)
            uturnArmed = false;
            uturnSaw111 = false;
          } else if (cmd == 'u') {
            // 8/20 재추가 -- 즉시 U턴 (3단계 시퀀스).
            // 라파 통합판이 EXIT_HALT / DEADEND_HALT / ZONE_ENTER 에서 보낸다.
            state = RobotState::RUNNING;
            xPhase = XPhase::TURN_U_CLEAR;
            uturnActive = true;
            uturnArmed = false;
            uturnClearSince = 0;
            uturnLineSince = 0;
            uturnSaw111 = false;
            xTurnStart = 0;
            xClearStart = 0;
            lineLostSince = 0;
            allLostSince = 0;
            lastPulseTimeL = lastPulseTimeR = millis();
            publishMark("uturn_start");
          } else if (cmd == 'c') {
            // 8/20 추가 -- CPR 실측 모드.
            // 모터를 완전히 세우고 카운터를 0으로 맞춘 뒤, 0.3초마다
            // 누적 카운트를 마크로 발행한다. 바퀴를 손으로 정확히 한 바퀴
            // 돌리고 그때의 값을 읽으면 된다. CMD:s 로 빠져나온다.
            state = RobotState::STOPPED;
            stopAll();
            xPhase = XPhase::NONE;
            calibMode = true;
            calibL = 0;
            calibR = 0;
            calibLastReport = 0;
            publishMark("calib_start bar wheel one turn by hand");
          } else {
            RpiSerial.printf("MARK:unknown_cmd_%c\n", cmd ? cmd : '?');
          }
        } else {
          int posIdx  = lineBuf.indexOf("POS:");
          int tvecIdx = lineBuf.indexOf("TVEC:");
          if (posIdx >= 0 && tvecIdx >= 0) {
            int posEnd = lineBuf.indexOf(',', posIdx);
            if (posEnd > posIdx) {
              // 8/20 추가 -- INIT 종료 시 교차로 감지 재무장 지연.
              // INIT 중에는 라인폭 판정을 끄는데, QR 을 읽는 순간 라파가
              // POS:INIT -> POS:TL 로 바꾸면서 판정이 곧바로 다시 켜진다.
              // 그런데 로봇은 아직 그 모서리의 끝라인 근처에 있어, 그
              // 111 이 첫 교차로로 잡힌다. 실측(TL 출발)에서 라파가 보낸
              // TVEC:97.1(A 에서 할 우회전)이 TL 끝라인에서 실행돼
              // 로봇이 곧바로 오른쪽 벽으로 돌아버렸다.
              // 회전 직후와 같은 처리를 걸어 끝라인을 벗어날 때까지 무시한다.
              String prevPos = latestPos;
              latestPos = lineBuf.substring(posIdx + 4, posEnd);
              if (prevPos.startsWith("INIT") && !latestPos.startsWith("INIT")) {
                crossRearmRemainCm = tune.crossRearmCm;
                wasAllBlack = false;
                crossDistCm = 0;
                publishMark("init_done_cross_rearm");
              }
              latestTVecDeg = lineBuf.substring(tvecIdx + 5).toFloat();
              int nearIdx = lineBuf.indexOf("NEAR:");
              latestNear = (nearIdx >= 0) &&
                           (lineBuf.substring(nearIdx + 5).toInt() != 0);
              lastRpiMsgTime = millis();
              Serial.printf("[RPI OK] pos=%s tvec=%.1f\n", latestPos.c_str(), latestTVecDeg);
            } else {
              Serial.println("[RPI ERR] 콤마 위치 이상 — 형식 불일치");
            }
          } else {
            Serial.println("[RPI ERR] 알 수 없는 줄 형식");
          }
        }
      }
      lineBuf = "";
    } else if (ch != '\r') {
      lineBuf += ch;
      if (lineBuf.length() > 80) {
        Serial.println("[RPI ERR] 80자 초과 — 줄바꿈 누락 의심, 버퍼 초기화");
        lineBuf = "";
      }
    }
  }
}

// ── 교차선 폭 측정 (거리 기반) ──────────────────────────
enum class LineWidthResult { NONE, CROSSING, STOPBAR };

LineWidthResult updateLineWidth(int l, int c, int r, float dCmThisLoop) {
  bool allBlack = (l == HIGH && c == HIGH && r == HIGH);

  if (allBlack) {
    crossDistCm += dCmThisLoop;
    wasAllBlack = true;
    return LineWidthResult::NONE;
  }

  if (wasAllBlack) {
    wasAllBlack = false;
    float width = crossDistCm;
    crossDistCm = 0;

    if (width >= tune.stopbarMinCm) return LineWidthResult::STOPBAR;
    // 8/21 -- 하한(crossMinCm) 추가. 아래 lastWidthCm 은 기각 사유를 로그로
    // 보기 위한 것으로, 판정 자체에는 관여하지 않는다.
    lastWidthCm = width;
    if (width >= tune.crossMinCm && width <= tune.crossMaxCm)
      return LineWidthResult::CROSSING;

    // 8/21 추가 -- 폭 때문에 버려진 111 을 로그로 남긴다.
    // 실측(A2 진입, B 교차로)에서 ls 가 110->111->101->111->011 로 분명히
    // 교차로를 지났는데 intersection 도 crossing_rejected_not_near 도 찍히지
    // 않았다. 즉 NEAR 로 기각된 게 아니라 여기서 조용히 사라진 것이다.
    // 유력한 경위: 교차선 폭은 "111 인 동안 전진한 거리"로 재는데, 라인 탐색
    // (제자리 회전) 중에는 전진 거리가 0 이라 진짜 교차선도 0cm 로 측정된다.
    // 원인을 눈으로 확인할 수 있게 폭과 탐색 여부를 같이 남긴다.
    {
      static unsigned long lastWidthRejectMark = 0;
      unsigned long nowMs = millis();
      if (nowMs - lastWidthRejectMark > 300) {
        lastWidthRejectMark = nowMs;
        char wBuf[64];
        snprintf(wBuf, sizeof(wBuf), "crossing_rejected_width w=%.1f search=%d",
                 width, lineSearchActive ? 1 : 0);
        publishMark(wBuf);
      }
    }
    return LineWidthResult::NONE;
  }

  return LineWidthResult::NONE;
}

// ── 라파 TVEC 각도 → 직진/좌/우 분류 ────────────────────
char decideTurnFromVector() {
  // 8/20: 실측 모드에서는 라파 스트림이 없어도 직진 유지
  if (calibMode) return 'S';
  if (millis() - lastRpiMsgTime >= RPI_STREAM_TIMEOUT_MS) return 'X';
  if (latestTVecDeg > tune.turnAngleThresholdDeg) return 'R';
  if (latestTVecDeg < -tune.turnAngleThresholdDeg) return 'L';
  return 'S';
}

void setup() {
  Serial.begin(115200);
  delay(300);

  RpiSerial.begin(RPI_BAUD, SERIAL_8N1, RPI_RX_PIN, RPI_TX_PIN);
  Serial.printf("[SETUP] RpiSerial started: baud=%lu RX=GPIO%d TX=GPIO%d\n",
                RPI_BAUD, RPI_RX_PIN, RPI_TX_PIN);

  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);

  // 8/20 수정 -- INPUT -> INPUT_PULLUP.
  // 풀업 없이 두면 엔코더 출력이 애매한 전압에 머물러, 노이즈로 엣지가
  // 여러 번 잡히거나(카운트 과다) 문턱을 못 넘어 놓친다(카운트 부족).
  // 실측에서 같은 3바퀴를 재는데 좌우가 250~466 /rev 로 양방향 요동쳤고,
  // 커넥터를 좌우 맞바꿔도 증상이 따라가지 않아 ESP32 입력단 문제로
  // 좁혀졌다. 엔코더 출력이 오픈 컬렉터면 풀업이 반드시 필요하다.
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encRightISR, RISING);

  pinMode(LS_L, INPUT); pinMode(LS_C, INPUT); pinMode(LS_R, INPUT);

  stopAll();
  lastPulseTimeL = lastPulseTimeR = millis();

  Serial.println("ready — WiFi/MQTT 없이 UART로만 동작. 라파 스트림 대기 중");
}

void loop() {
  pollRpiSerial();   // 라파 스트림(POS/TVEC/CMD)은 매 루프 논블로킹으로 계속 흡수

  static unsigned long lastLoop = millis();
  static unsigned long lastPublish = 0;
  static unsigned long testStepUntil = 0;
  static int testStep = 0;

  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0f;
  if (dt < 0.02f) return;
  lastLoop = now;

  noInterrupts();
  long dL = encLcount; encLcount = 0;
  long dR = encRcount; encRcount = 0;
  interrupts();

  // 8/20 -- CPR 실측 모드: 델타를 누적해서 주기적으로 보고
  if (calibMode) {
    calibL += dL;
    calibR += dR;
    if (millis() - calibLastReport >= 300) {
      calibLastReport = millis();
      char cBuf[64];
      snprintf(cBuf, sizeof(cBuf), "calib L=%ld R=%ld (CPR now=%.0f)",
               calibL, calibR, (double)CPR);
      publishMark(cBuf);
    }
  }

  float rpmL = countsToRPM(dL, dt);
  float rpmR = countsToRPM(dR, dt);
  rpmL = smoothRPM(rpmLHistory, rpmL);
  rpmR = smoothRPM(rpmRHistory, rpmR);
  rpmHistIdx++;

  int l = digitalRead(LS_L);
  int c = digitalRead(LS_C);
  int r = digitalRead(LS_R);

  // 8/20 수정 -- 회전 중에는 라인센서 패턴이 20ms 루프마다 뒤집혀서
  // ls: 마크가 초당 수십 줄씩 UART 를 채운다. 그러면 라파가 보내는
  // POS/TVEC 수신이 밀려 RPI_STREAM_TIMEOUT_MS(2000) 에 걸릴 위험이 있다.
  // 회전 단계(xPhase != NONE)에서는 마크를 억제하고, 상태 변화 자체는
  // 0.5초마다 나가는 publishState 의 "ls" 필드로 확인할 수 있다.
  static int prevL = -1, prevC = -1, prevR = -1;
  if ((l != prevL || c != prevC || r != prevR) && xPhase == XPhase::NONE) {
    char lsBuf[24];
    snprintf(lsBuf, sizeof(lsBuf), "ls:%d%d%d", l, c, r);
    publishMark(lsBuf);
  }
  prevL = l; prevC = c; prevR = r;

  // ── 라파 스트림 타임아웃 시 안전 정지 ──
  // 8/20: CPR 실측(calibMode) 중에는 예외. 실측은 라파가 INIT 에 머물러
  // POS 스트림을 안 보내는 상태에서도 해야 하는데, 그러면 CMD:g 를 보내도
  // 2초 만에 rpi_link_lost 로 다시 멈춰 측정 자체가 불가능하다.
  // 이 모드는 사람이 옆에서 지켜보며 CMD:s 로 끝내므로 안전하다.
  if (state == RobotState::RUNNING && !calibMode &&
      millis() - lastRpiMsgTime >= RPI_STREAM_TIMEOUT_MS) {
    stopAll();
    state = RobotState::STOPPED;
    publishMark("rpi_link_lost");
  }

  float targetL = 0, targetR = 0;
  bool skipPid = false;

  switch (state) {

    case RobotState::RUNNING: {
      // 8/20: 좌우 평균은 한쪽 엔코더가 죽으면 실제의 절반이 되어
      // 라인폭 판정(crossMaxCm)이 통째로 어긋난다. 살아 있는 쪽만,
      // 둘 다 살아 있으면 평균을 쓴다.
      float dCmThisLoop;
      if (dL != 0 && dR != 0)      dCmThisLoop = ((fabs(dL) + fabs(dR)) / 2.0f) * CM_PER_PULSE;
      else if (dL != 0)            dCmThisLoop = fabs(dL) * CM_PER_PULSE;
      else                         dCmThisLoop = fabs(dR) * CM_PER_PULSE;

      // ── 8/20: CPR 실측 모드는 라인 추종을 아예 하지 않는다 ──
      // 측정하려는 것은 "바퀴가 몇 번 굴렀을 때 몇 cm 갔나" 뿐이다.
      // 라인 추종 PD 가 살아 있으면 조향할 때마다 좌우 바퀴가 서로 다른
      // 거리를 굴러 카운트가 어긋나고, 라인을 놓치면 엉뚱하게 회전한다.
      // 좌우를 같은 PWM 으로 고정해 그냥 밀고 나간다.
      if (calibMode) {
        driveLeft(tune.calibPwm);
        driveRight(tune.calibPwm);
        publishState(rpmL, rpmR, l, c, r);
        return;
      }

      // ★ 강제 직진 블록 시작 — 삭제하려면 이 if 블록 전체만 지우면 됨 ★
      if (xPhase == XPhase::FORCED_ADVANCE) {
        if (xClearStart == 0) xClearStart = now;
        targetL = targetR = tune.turnInner;
        if (now - xClearStart >= tune.crossingAdvanceMs) {
          xClearStart = 0;
          xPhase = XPhase::NONE;
          // 8/20 추가 -- 교차로를 막 통과한 직후다. 통과 중 111 이 이어지며
          // 쌓인 폭 측정 상태를 비우지 않으면 곧바로 같은 교차로를 한 번 더
          // 잡는다. 아직 교차로를 완전히 벗어나지 못했을 수 있으므로
          // 재무장 거리도 함께 건다.
          wasAllBlack = false;
          crossDistCm = 0;
          crossRearmRemainCm = tune.crossRearmCm;
        }
        break;
      }
      // ★ 강제 직진 블록 끝 ★

      // ── 우회전 전용 : 시간 기반, 센서 패턴 판단 없음 ──
      // "처음엔 그냥 시간으로 돌리자" -- 정해진 시간만큼 피벗하고 끝냄.
      // 끝난 뒤 라인이 안 잡히면 평소 라인 PD 쪽의 line_lost 안전장치가 처리.
      if (xPhase == XPhase::TURN_R_TIMED) {
        if (xTurnStart == 0) xTurnStart = now;
        // 8/20 개정 -- 좌우 공용. 제자리 회전(좌우 반대)이며 방향만 다르다.
        // 좌회전도 이 단계를 쓰도록 바꿨다(아래 분기 참고).
        bool timedRight = (pendingTurn == 'R');
        if (timedRight) { driveLeft(tune.pivot); driveRight(-tune.pivot); }
        else            { driveLeft(-tune.pivot); driveRight(tune.pivot); }
        skipPid = true;

        if (now - xTurnStart >= tune.rightTurnMs) {
          xTurnStart = 0;
          prevLineError = 0;
          lineDErrFilt = 0;   // 8/21: 회전 후 미분항 잔상 제거
          lastNonZeroError = 0;  // 8/21: 회전 전 방향 기억도 버린다
          lineSearchActive = false;  // 8/21: 탐색 상태도 초기화
          // 8/20: 시간만 채우고 곧바로 라인 추종으로 넘기지 않는다.
          // rightTurnMs 가 정확히 90도라는 보장이 없어, 조금만 어긋나도
          // 세로선을 비스듬히 만나 훑다가 line_lost 로 멈췄다.
          // TURN_ALIGN 에서 실제로 라인을 잡을 때까지 정렬한다.
          alignSweepRight = timedRight;   // 회전 마무리 -- 같은 방향부터
          alignStart = now;
          alignLostSince = 0;
          alignFlipWindow = 0;
          xPhase = XPhase::TURN_ALIGN;
          // 8/20 추가 -- 폭 측정 상태 초기화.
          // 회전 중 센서가 라인을 훑으며 111 을 지나가는데, 그 잔여 상태가
          // 남은 채 xPhase 가 NONE 으로 돌아오면 111->011 로 떨어지는 순간을
          // 진짜 교차로로 오인한다. 실측에서 우회전 직후 곧바로
          // "intersection pos=B tvec=0" 이 한 번 더 잡혀 FORCED_ADVANCE 가
          // 걸렸고, 500ms 직진하는 동안 방금 꺾어 들어온 라인을 벗어나
          // line_lost 로 이어졌다.
          wasAllBlack = false;
          crossDistCm = 0;
          publishMark(timedRight ? "right_turn_timed_done" : "left_turn_timed_done");
        }
        break;
      }


      // ── 회전 1단계 : 111까지 한쪽 바퀴만 후진 ──
      if (xPhase == XPhase::TURN_A) {
        if (xTurnStart == 0) xTurnStart = now;
        bool wantRight = (pendingTurn == 'R');

        // 8/20 재시도 -- 제자리 회전(좌우 반대).
        // 앞서 한 번 되돌렸던 이유는 TURN_A 에서 라인을 건너뛰었기
        // 때문인데, 그때는 turnLostDebounceMs 가 없어 직진용 150ms 가
        // 적용되고 있었다. 지금은 400ms 라 회전이 빨라져 생기는 공백을
        // 버틴다. pivot 도 100 -> 75 로 낮춰 각속도를 맞췄다.
        if (wantRight) { driveLeft(tune.pivot); driveRight(-tune.pivot); }
        else            { driveLeft(-tune.pivot); driveRight(tune.pivot); }
        skipPid = true;

        bool at111 = (l == HIGH && c == HIGH && r == HIGH);
        bool allLostNow = (l == LOW && c == LOW && r == LOW);
        if (allLostNow) {
          if (allLostSince == 0) allLostSince = now;
        } else {
          allLostSince = 0;
        }
        // 000 이 찰나만 스친 게 아니라 일정 시간 지속됐을 때만 "진짜 놓침"
        // 으로 인정한다. 8/20: 회전 중에는 가로선->세로선 사이 공백이
        // 정상이므로 직진용(150ms)이 아니라 전용(400ms)을 쓴다.
        bool allLostConfirmed = allLostSince != 0 &&
          (now - allLostSince) > tune.turnLostDebounceMs;
        // 8/19: 안전용 최대 시간일 뿐, 정상 트리거는 allLostConfirmed.
        bool safetyTimedOut = (now - xTurnStart) > (unsigned long)tune.pivotTimeoutMs * 4;
        if (at111) {
          xPhase = XPhase::TURN_B;
          xTurnStart = 0;
          allLostSince = 0;
        } else if (allLostConfirmed || safetyTimedOut) {
          // 8/19: 라인을 진짜로 벗어났을 때만 반대 방향으로 재탐색 시작
          xTurnStart = 0;
          allLostSince = 0;
          searchStartTime = now;
          searchSweepRight = !wantRight;   // 원래 방향의 반대부터 훑기 시작
          searchTarget = '1';              // 111 을 찾는 중이었음
          xPhase = XPhase::TURN_SEARCH;
          publishMark(allLostConfirmed ? "pivot_search_start_111_line_lost"
                                        : "pivot_search_start_111_safety_timeout");
        }
        break;
      }

      // ── 회전 2단계 (좌회전 전용, 우회전은 별도 시간 기반 방식 사용) ──
      // 010까지 반대쪽 바퀴만 전진. 라인이 진짜(디바운스) 사라졌을 때만 재탐색.
      if (xPhase == XPhase::TURN_B) {
        if (xTurnStart == 0) xTurnStart = now;
        bool wantRight = (pendingTurn == 'R');   // 이제 이 단계엔 사실상 좌회전만 옴

        bool at010 = (c == HIGH && l == LOW && r == LOW);
        bool allLostNow = (l == LOW && c == LOW && r == LOW);
        if (allLostNow) {
          if (allLostSince == 0) allLostSince = now;
        } else {
          allLostSince = 0;
        }
        // 8/20: 회전 전용 디바운스 (직진용 150ms 는 회전에 너무 짧다)
        bool allLostConfirmed = allLostSince != 0 &&
          (now - allLostSince) > tune.turnLostDebounceMs;
        bool safetyTimedOut = (now - xTurnStart) > (unsigned long)tune.pivotTimeoutMs * 4;
        // 8/20: 회전 초반에 가운데 센서만 잠깐 스치는 것을 완료로 오인하지
        // 않도록, 최소 시간이 지나야 010 을 인정한다.
        bool minPhaseDone = (now - xTurnStart) >= tune.turnMinPhaseMs;

        if (at010 && minPhaseDone) {
          xTurnStart = 0;
          allLostSince = 0;
          prevLineError = 0;
          lineDErrFilt = 0;   // 8/21: 회전 후 미분항 잔상 제거
          lastNonZeroError = 0;  // 8/21: 회전 전 방향 기억도 버린다
          lineSearchActive = false;  // 8/21: 탐색 상태도 초기화
          pendingTurn = 'S';
          xPhase = XPhase::NONE;
          // 8/20 추가 -- 회전 중 훑은 라인이 폭 측정에 남아 회전 직후
          // 가짜 교차로로 잡히는 것을 막는다 (우회전에서 실측 확인된 문제).
          wasAllBlack = false;
          crossDistCm = 0;
          crossRearmRemainCm = tune.crossRearmCm;   // 8/20: 교차로 재감지 방지
        } else if (allLostConfirmed || safetyTimedOut) {
          // 8/20: 한 방향으로만 훑는 TURN_SEARCH 대신, 방향을 오가며
          // 반드시 라인을 다시 잡는 TURN_ALIGN 으로 보낸다.
          xTurnStart = 0;
          allLostSince = 0;
          alignSweepRight = wantRight;
          alignStart = now;
          alignLostSince = 0;
          alignFlipWindow = 0;
          xPhase = XPhase::TURN_ALIGN;
          publishMark("align_start_from_turn_b");
          skipPid = true;
          break;
        } else {
          // 8/20 -- TURN_A 와 동일한 제자리 회전. 회전 중심을 바꾸지 않는다.
          if (wantRight) { driveLeft(tune.pivot); driveRight(-tune.pivot); }
          else            { driveLeft(-tune.pivot); driveRight(tune.pivot); }
        }
        skipPid = true;
        break;
      }

      // ── 회전 재탐색 : 저속으로 반대 방향으로 계속 돌며 라인 찾기 ──
      // 방금 그 방향으로 지나오면서 못 찾은 거니, 반대로 돌리면 지나온
      // 구간을 다시 훑게 되어 라인을 다시 만난다. 좌우로 오갈 필요 없이
      // 찾을 때까지(또는 타임아웃까지) 한 방향으로 계속 저속 회전.
      // ── U턴 1단계 : 끝라인 이탈 후 연장선 끝까지 ──
      // 라인 추종으로 전진하며 두 가지만 본다.
      //   111 -> 끝라인 위. 지나면 무장.
      //   000 -> 연장선 끝. 무장 뒤라면 피벗으로.
      // 그 외(010/110/011/100/001)는 그냥 라인 추종으로 계속 간다.
      //
      // 8/20 개정 -- 기존에는 "111 이 아니면" 무장했다. 그런데 속도가
      // 빠르면 끝라인을 비스듬히 넘으며 111 을 못 보고 110 으로 지나가는데,
      // 그 110 에 무장이 걸려버렸다. 무장은 "끝라인을 실제로 지났다"는
      // 뜻이어야 하므로, 111 을 한 번이라도 본 뒤에만 열리게 한다.
      // 111 을 끝내 못 보면 타임아웃이 대신 무장시킨다.
      if (xPhase == XPhase::TURN_U_CLEAR) {
        if (xClearStart == 0) xClearStart = now;
        targetL = targetR = tune.turnInner;

        bool at111 = (l == HIGH && c == HIGH && r == HIGH);
        bool allLost = (l == LOW && c == LOW && r == LOW);

        if (!uturnArmed) {
          // 8/20 개정 -- 000 은 111 을 봤든 안 봤든 곧바로 피벗 조건이다.
          // 기존에는 uturnSaw111 이 true 여야만 무장했는데, 라파가 막다른
          // 지점 QR 을 읽고 CMD:s 를 보내는 시점에 로봇이 이미 끝라인을
          // 지나 있으면 CMD:u 이후 111 을 다시 볼 일이 없다. 그러면 무장이
          // 영원히 안 되고 000 분기에 도달조차 못 해, 연장선 끝을 지나
          // 라인 밖으로 나가도 계속 전진했다(실측: MR 에서 20초 무응답).
          // 진행용 타임아웃을 없앴으므로 여기서 직접 빠져나갈 길을 준다.
          if (allLost) {
            if (allLostSince == 0) allLostSince = now;
            if (now - allLostSince >= tune.uTurnLostDebounceMs) {
              xClearStart = 0;
              xTurnStart = 0;
              allLostSince = 0;
              xPhase = XPhase::TURN_U_PIVOT;
              publishMark(uturnSaw111 ? "uturn_pivot_start"
                                      : "uturn_pivot_start_no111");
              break;
            }
          } else {
            allLostSince = 0;
            if (at111) {
              // 끝라인을 보는 중 -- 기록해두고, 벗어나면 무장
              uturnSaw111 = true;
              uturnClearSince = 0;
            } else if (uturnSaw111) {
              // 111 을 봤고 이제 벗어났다 -- 디바운스 뒤 무장
              if (uturnClearSince == 0) uturnClearSince = now;
              if (now - uturnClearSince >= tune.uTurnClearDebounceMs) {
                uturnArmed = true;
                publishMark("uturn_endline_cleared");
              }
            } else {
              // 아직 끝라인을 못 봤다(110/010 등) -- 라인 추종으로 계속 전진
              uturnClearSince = 0;
            }
          }
        } else {
          if (allLost) {
            if (allLostSince == 0) allLostSince = now;
          } else {
            allLostSince = 0;
          }
          // 8/20: 시간 제한 없음. 연장선 끝의 000 을 실제로 만날 때까지 간다.
          bool lostConfirmed = allLostSince != 0 &&
                               (now - allLostSince) >= tune.uTurnLostDebounceMs;
          if (lostConfirmed) {
            xClearStart = 0;
            xTurnStart = 0;
            allLostSince = 0;
            xPhase = XPhase::TURN_U_PIVOT;
            publishMark("uturn_pivot_start");
          }
        }
        break;
      }

      // ── U턴 2단계 : 제자리 피벗, 라인이 다시 잡힐 때까지 ──
      // 좌우 반대 회전으로 제자리에서 돈다. 연장선 끝에서 시작하므로
      // 바깥쪽 센서가 아직 끝자락을 물고 있을 수 있어, uTurnMinPivotMs
      // 동안은 감지를 무시한다. 종료는 111 이 아니라 "센서 중 하나라도
      // 라인" -- 직선 위에서 180도 돌면 010 계열이 나오지 111 은 안 나온다.
      if (xPhase == XPhase::TURN_U_PIVOT) {
        if (xTurnStart == 0) xTurnStart = now;
        driveLeft(tune.pivot); driveRight(-tune.pivot);
        skipPid = true;

        unsigned long elapsed = now - xTurnStart;
        // 8/20: "어느 센서든"이 아니라 "가운데 센서가" 물어야 인정한다.
        // 제자리 회전 중 가장자리 센서가 라인 끝자락을 스치는 것과,
        // 실제로 라인 위에 올라선 것을 구분하기 위함이다.
        bool centerOn = (c == HIGH);
        if (centerOn) {
          if (uturnLineSince == 0) uturnLineSince = now;
        } else {
          uturnLineSince = 0;
        }
        bool lineHeld = uturnLineSince != 0 &&
                        (now - uturnLineSince) >= tune.uTurnLineHoldMs;
        bool armed = elapsed >= tune.uTurnMinPivotMs;
        bool timedOut = elapsed >= tune.uTurnMaxPivotMs;

        if ((armed && lineHeld) || timedOut) {
          xTurnStart = 0;
          prevLineError = 0;
          lineDErrFilt = 0;   // 8/21: 회전 후 미분항 잔상 제거
          lastNonZeroError = 0;  // 8/21: 회전 전 방향 기억도 버린다
          lineSearchActive = false;  // 8/21: 탐색 상태도 초기화
          pendingTurn = 'S';
          uturnArmed = false;
          wasAllBlack = false;
          crossDistCm = 0;
          lineLostSince = 0;
          allLostSince = 0;
          crossRearmRemainCm = tune.crossRearmCm;
          // 8/20 개정 -- 곧바로 라인 추종으로 넘기지 않고 TURN_ALIGN 으로.
          // U턴 종료 각도는 피벗 시작 자세에 따라 120~229도로 편차가 커서
          // 시간이나 감지 조건을 조여도 매번 어긋났다. 교차로 회전에서
          // 이미 검증된 정렬 단계에 맡겨 라인 위에 제대로 올려놓는다.
          alignSweepRight = true;
          alignStart = now;
          alignLostSince = 0;
          alignFlipWindow = 0;
          xPhase = XPhase::TURN_ALIGN;
          // uturnActive 는 여기서 끄지 않는다 -- 복귀 주행 중 끝라인을
          // 안쪽으로 통과하며 나오는 111 까지 무시해야 하므로, 라파가
          // CMD:s 또는 CMD:g 를 보낼 때 해제된다.
          uturnLineSince = 0;
          char uBuf[56];
          snprintf(uBuf, sizeof(uBuf), "%s ms=%lu ls=%d%d%d",
                   timedOut ? "uturn_done_timeout" : "uturn_done",
                   elapsed, l, c, r);
          publishMark(uBuf);
        }
        break;
      }

      // ── 회전 마무리 정렬 : 라인을 다시 잡을 때까지 좌우로 오간다 ──
      // 8/20 추가. 기존에는 회전이 끝나면 곧바로 라인 추종으로 넘어갔고,
      // 각도가 조금만 어긋나 000 이 되면 line_lost 로 멈춰버렸다.
      // 여기서는 000 에서 포기하지 않는다. 000 이 alignFlipMs 이상 지속되면
      // 회전 방향을 뒤집어 반대로 훑는다. 덜 돌았든 지나쳤든 오가면서
      // 결국 라인을 다시 만난다.
      // 어느 센서든 라인을 잡으면 완료 -- 정확한 중앙 정렬은 라인 PD 가 한다.
      if (xPhase == XPhase::TURN_ALIGN) {
        // 8/20 수정 -- 한쪽 바퀴 피벗에서 제자리 회전(좌우 반대)으로.
        // 기존 방식은 두 갈래 모두 한쪽 바퀴를 후진시키고 반대쪽을 세웠다.
        // 멈춘 바퀴를 축으로 돌되 미는 방향이 뒤라서, 회전하면서 로봇이
        // 계속 뒤로 물러났다. align_flip 으로 방향이 뒤집힐 때마다 축도
        // 좌우로 바뀌어 "고개를 흔들며 후진"하는 모습이 됐다.
        // 나머지 회전 단계는 이미 제자리 회전으로 통일돼 있다.
        if (alignSweepRight) { driveLeft(tune.searchPivot); driveRight(-tune.searchPivot); }
        else                 { driveLeft(-tune.searchPivot); driveRight(tune.searchPivot); }
        skipPid = true;

        // 8/20 개정 -- "어느 센서든"에서 "가운데 센서가 물어야"로.
        // 기존에는 가장자리만 걸린 100/001 로도 완료했는데, 그 상태로 라인
        // 추종에 넘기면 PD 가 오차 1.0 을 읽고 최대 조향(좌우 90rpm 차이)을
        // 걸어버린다. 실측에서 align_done ls=100 직후 rpmL 16.9 / rpmR 118.6
        // 으로 급선회하며 곧바로 라인을 벗어났다.
        // 가운데가 물린 010/110/011 이면 PD 오차가 0~0.5 라 부드럽게 이어진다.
        // 8/20 재조정 -- 2단계 판정.
        // 가운데만 요구했더니 못 찾고 좌우로 계속 뒤집기만 하는 경우가
        // 생겼다(실측: align_flip 5회 연속, align_done 없음). 그동안 로봇이
        // 표류해 연쇄 재탐색으로 이어졌다.
        // 가운데를 우선하되, alignEdgeFallbackMs 가 지나면 가장자리도
        // 받아들인다. 완료 못 하는 것보다 낫다.
        unsigned long elapsedNow = now - alignStart;
        bool centerOn = (c == HIGH);
        bool edgeOn   = (l == HIGH || r == HIGH);
        bool anyLine  = centerOn ||
                        (edgeOn && elapsedNow >= tune.alignEdgeFallbackMs);
        bool at111   = (l == HIGH && c == HIGH && r == HIGH);
        unsigned long elapsed = now - alignStart;

        // 8/20 추가 -- 111 은 아직 교차로(가로선) 위라는 뜻이므로 완료로
        // 인정하지 않는다. 여기서 끝내면 교차로 한복판에서 라인 추종으로
        // 넘어가 곧바로 다시 교차로를 밟게 된다. 111 을 벗어날 때까지
        // 계속 회전한다.
        // 111 인 동안에도 anyLine 이 참이라 방향 전환(000 판정)은 걸리지 않고,
        // 영영 못 벗어나면 alignMaxMs 가 잡아준다.
        if (at111 && elapsed >= tune.turnMinPhaseMs) {
          static unsigned long lastHoldMark = 0;
          if (now - lastHoldMark > 500) {
            lastHoldMark = now;
            publishMark("align_hold_111");
          }
        }

        // 진입 직후엔 아직 원래 라인 위일 수 있으므로 최소 시간 뒤부터 인정
        if (anyLine && !at111 && elapsed >= tune.turnMinPhaseMs) {
          prevLineError = 0;
          lineDErrFilt = 0;   // 8/21: 회전 후 미분항 잔상 제거
          lastNonZeroError = 0;  // 8/21: 회전 전 방향 기억도 버린다
          lineSearchActive = false;  // 8/21: 탐색 상태도 초기화
          pendingTurn = 'S';
          xPhase = XPhase::NONE;
          wasAllBlack = false;
          crossDistCm = 0;
          lineLostSince = 0;
          allLostSince = 0;
          alignLostSince = 0;
          crossRearmRemainCm = tune.crossRearmCm;   // 8/20: 교차로 재감지 방지
          char aBuf[40];
          snprintf(aBuf, sizeof(aBuf), "align_done%s ls=%d%d%d ms=%lu",
                   centerOn ? "" : "_edge", l, c, r, elapsed);
          publishMark(aBuf);
          break;
        }

        // 000 이 지속되면 방향 전환 -- 이게 "포기하지 않는" 핵심
        if (!anyLine) {
          if (alignLostSince == 0) alignLostSince = now;
          if (alignFlipWindow == 0) alignFlipWindow = tune.alignFlipMs;
          if (now - alignLostSince >= alignFlipWindow) {
            // 8/20 -- 뒤집을 때마다 훑는 폭을 넓힌다.
            // 제자리 회전이라 병진이 없어, 폭이 고정이면 같은 각도 밴드만
            // 반복해 훑다가 그 밖의 라인을 영영 못 찾는다(한 자리에서
            // 좌우로만 흔들리는 증상). 폭을 키우면 반드시 라인을 덮는다.
            alignSweepRight = !alignSweepRight;
            alignLostSince = now;
            alignFlipWindow += tune.alignFlipMs;
            char fBuf[40];
            snprintf(fBuf, sizeof(fBuf), "align_flip_%s w=%lu",
                     alignSweepRight ? "right" : "left", alignFlipWindow);
            publishMark(fBuf);
          }
        } else {
          alignLostSince = 0;
        }

        if (elapsed > tune.alignMaxMs) {
          state = RobotState::STOPPED;
          stopAll();
          xPhase = XPhase::NONE;
          publishMark("align_giveup");
        }
        break;
      }

      if (xPhase == XPhase::TURN_SEARCH) {
        bool at111 = (l == HIGH && c == HIGH && r == HIGH);
        bool at010 = (c == HIGH && l == LOW && r == LOW);
        // 8/20: 재탐색을 시작하자마자 스치는 010 을 완료로 인정하면 덜 돈
        // 채 라인 추종으로 복귀해 곧바로 이탈한다. 최소 시간을 두어
        // 실제로 훑은 뒤의 판정만 받아들인다.
        bool searchMinDone = (now - searchStartTime) >= tune.turnMinPhaseMs;
        bool found = ((searchTarget == '1') ? at111 : at010) && searchMinDone;

        if (found) {
          publishMark("pivot_search_found");
          if (searchTarget == '1') {
            xPhase = XPhase::TURN_B;   // 111 찾았으니 이어서 010까지 2단계 진행
            xTurnStart = 0;
          } else {
            prevLineError = 0;
            lineDErrFilt = 0;   // 8/21: 회전 후 미분항 잔상 제거
            lastNonZeroError = 0;  // 8/21: 회전 전 방향 기억도 버린다
            lineSearchActive = false;  // 8/21: 탐색 상태도 초기화
            pendingTurn = 'S';
            xPhase = XPhase::NONE;     // 010 찾았으니 회전 완료
            wasAllBlack = false;       // 8/20: 폭 측정 잔여 상태 초기화
            crossDistCm = 0;
            crossRearmRemainCm = tune.crossRearmCm;
          }
          skipPid = true;
          break;
        }

        unsigned long totalElapsed = now - searchStartTime;
        if (totalElapsed > tune.searchMaxMs) {
          // 8/20: 여기서 바로 정지하지 않고, 방향을 오가며 끝까지 라인을
          // 찾는 TURN_ALIGN 으로 넘긴다. 최종 포기는 그쪽 alignMaxMs 가 한다.
          alignSweepRight = searchSweepRight;
          alignStart = now;
          alignLostSince = 0;
          alignFlipWindow = 0;
          xPhase = XPhase::TURN_ALIGN;
          publishMark("align_start_from_search");
          skipPid = true;
          break;
        }

        // 저속 제자리 회전, 반대 방향으로 고정 (searchSweepRight 는
        // TURN_SEARCH 진입 시 한 번만 정해지고 안 바뀜)
        if (searchSweepRight) { driveLeft(tune.searchPivot); driveRight(-tune.searchPivot); }
        else                   { driveLeft(-tune.searchPivot); driveRight(tune.searchPivot); }
        skipPid = true;
        break;
      }

      // ── xPhase == NONE : 교차선 폭 측정 + 평상시 라인 추종 ──
      LineWidthResult wr = updateLineWidth(l, c, r, dCmThisLoop);

      // 8/20: 실측 모드에서는 라인폭 판정을 끈다. 측정 구간에 교차선이
      // 있으면 회전해버려 직선 거리 측정이 망가진다.
      // 8/20: U턴 시퀀스 중에도 라인폭 판정을 끈다. 복귀하며 끝라인을
      // 안쪽으로 통과할 때 뜨는 111 이 정지선/교차로로 오인되면 안 된다.
      // 8/20 추가 -- INIT 중에도 라인폭 판정을 끈다.
      // 라파는 INIT 동안 TVEC:0 을 보내므로 교차로를 만나면 무조건
      // FORCED_ADVANCE(500ms 라인추종 없이 직진)가 걸린다. 아직 위치도
      // 모르는 단계에서 그렇게 밀고 나가면 QR 을 더 놓치기만 한다.
      if (calibMode || uturnActive || latestPos.startsWith("INIT"))
        wr = LineWidthResult::NONE;

      // 8/20: 회전 직후 재무장 구간 -- 방금 지나온 교차로를 한 번 더
      // 잡지 않도록, 일정 거리를 벗어날 때까지 판정을 무시한다.
      if (crossRearmRemainCm > 0) {
        crossRearmRemainCm -= dCmThisLoop;
        wr = LineWidthResult::NONE;
        wasAllBlack = false;
        crossDistCm = 0;
        if (crossRearmRemainCm <= 0) {
          crossRearmRemainCm = 0;
          publishMark("cross_rearmed");
        }
      }

      if (wr == LineWidthResult::STOPBAR) {
        stopAll();
        state = RobotState::PH_STOP;
        publishMark("arrive");
        publishState(rpmL, rpmR, l, c, r);
        skipPid = true;
        break;
      }

      // 8/21 개정 -- NEAR 게이팅을 "회전이 걸린 교차로"에서 "모든 교차로"로 확대.
      //
      // 8/20 판(직진은 그대로 통과)은 "직진은 오인해도 피해가 없다"고 봤는데
      // 틀렸다. 라파는 이 intersection 마크를 세어 esp32_crossing_count 로
      // 쓰고, 그 카운터로 TVEC 을 몇 번째 구간 것으로 보낼지 게이팅한다.
      // 즉 가짜 직진 교차로 하나가 카운터를 1 올리면 그 뒤 모든 회전각이
      // 한 칸씩 앞당겨진다.
      //
      // 실측(TL->A->B->D, B3 출동)에서 정확히 이렇게 무너졌다:
      //   1) TL-A 복도 한가운데(열 2 부근)에서 가짜 111 -> tvec 7.1(직진)이라
      //      게이트를 통과, count=1. 라파는 이걸 'A 를 지났다'로 해석.
      //   2) 진짜 A 교차로는 그때 이미 tvec 90(B 에서 할 회전)이 실려 있어
      //      NEAR:0 으로 기각됨. count 는 1 그대로.
      //   3) B 도착 판정이 2칸 일찍 나면서 NEAR:1 이 열린 순간 또 가짜 111 이
      //      들어와 그 자리에서 우회전 실행. count=2.
      //   4) count=2 가 되자 라파의 gated_leg 가 끝까지 밀려 TVEC:0.0 이 되고,
      //      진짜 B 교차로는 직진 통과. 이후 TR 까지 가서 경로이탈 정지.
      // 로그상 "회전했다가 다시 라인을 타고 감"의 정체가 3) 이다.
      //
      // 직진도 막으면 진짜 직진 교차로를 놓쳐 카운터가 밀릴 위험이 있으나,
      // 라파에 그 경우를 푸는 LEG_LAG_TIMEOUT(3초, 회전 대기 중이 아닐 때만
      // 발동)이 이미 있어 회복된다. 반대로 회전 대기 중에는 그 타임아웃이
      // 일부러 억제돼 있으므로, 가짜 카운트를 흘려보내는 쪽이 훨씬 위험하다.
      if (wr == LineWidthResult::CROSSING && !latestNear) {
        wr = LineWidthResult::NONE;
        wasAllBlack = false;
        crossDistCm = 0;
        static unsigned long lastRejectMark = 0;
        if (now - lastRejectMark > 500) {
          lastRejectMark = now;
          // 8/21: tvec 을 같이 남긴다. 기각된 것이 회전이었는지 직진이었는지
          // 로그만 보고 구분할 수 있어야 다음 튜닝이 가능하다.
          char rjBuf[56];
          snprintf(rjBuf, sizeof(rjBuf), "crossing_rejected_not_near tvec=%.0f",
                   latestTVecDeg);
          publishMark(rjBuf);
        }
      }

      if (wr == LineWidthResult::CROSSING) {
        pendingTurn = decideTurnFromVector();

        // 8/21: 측정 폭(w)을 함께 남긴다. crossMinCm/crossMaxCm 튜닝의 근거가
        // 되고, 가짜/진짜 교차로를 사후에 폭으로 구분할 수 있게 해준다.
        char markBuf[72];
        snprintf(markBuf, sizeof(markBuf), "intersection pos=%s tvec=%.0f w=%.1f",
                 latestPos.c_str(), latestTVecDeg, lastWidthCm);
        publishMark(markBuf);

        if (pendingTurn == 'X') {
          publishMark("no_target_stop");
          state = RobotState::STOPPED;
          stopAll();
          skipPid = true;
          break;
        }

        if (pendingTurn == 'S')      xPhase = XPhase::FORCED_ADVANCE;
        // 8/20 개정 -- 좌회전도 시간 기반(TURN_R_TIMED) + TURN_ALIGN 으로 통일.
        // 지금까지 성공한 회전은 전부 우회전이었고, 그건 이 경로를 탔다.
        // 좌회전만 TURN_A(111 탐색) -> TURN_B(010 탐색) 라는 다른 방식을
        // 썼는데 여러 차례 실패했다. 실측(A 교차로)에서는 111 을 끝내
        // 못 찾아 안전 타임아웃 3.6초 동안 약 2바퀴를 헛돌았다.
        // 교차로 한복판에서 제자리 회전하면 두 라인이 겹쳐 깨끗한 111 이
        // 안 나오는 것이 원인으로 보인다. 검증된 쪽으로 통일한다.
        // (TURN_A/TURN_B 는 TURN_ALIGN 실패 시의 예비 경로로만 남는다)
        else                          xPhase = XPhase::TURN_R_TIMED;
        xClearStart = 0;
        xTurnStart = 0;
        break;
      }

      // ── 라인 PD ──
      bool lineDetected = !(l == LOW && c == LOW && r == LOW);
      if (lineDetected) {
        lineLostSince = 0;
      } else {
        if (lineLostSince == 0) lineLostSince = now;
        if (now - lineLostSince >= tune.lineLostTimeoutMs) {
          stopAll();
          state = RobotState::STOPPED;
          publishMark("line_lost");
          publishState(rpmL, rpmR, l, c, r);
          skipPid = true;
          lineLostSince = 0;
          break;
        }
      }

      float error = prevLineError;
      if (c == HIGH && l == LOW && r == LOW) {
        error = 0;
      } else if (l == HIGH && c == LOW && r == LOW) {
        error = 1.0;
      } else if (r == HIGH && c == LOW && l == LOW) {
        error = -1.0;
      } else if (l == HIGH && c == HIGH) {
        error = 0.5;
      } else if (r == HIGH && c == HIGH) {
        error = -0.5;
      }

      // 8/21 -- 방향이 확실한 오차만 따로 보관한다(복구용).
      if (lineDetected && error != 0.0f) lastNonZeroError = error;

      float dErr = (error - prevLineError) / dt;
      prevLineError = error;
      // 8/21 -- 미분항을 EMA 로 필터링. alpha=0.3 이면 계단 입력의 첫 루프
      // 기여가 30% 로 줄고 나머지가 뒤 루프로 퍼진다. 조향 총량은 비슷하게
      // 유지되면서 한 루프짜리 임펄스가 사라진다.
      lineDErrFilt += tune.dErrAlpha * (dErr - lineDErrFilt);
      float steer = tune.kp * error + tune.kd * lineDErrFilt;
      // 8/20 -- 단계별 속도. 라파가 보내는 POS 라벨로 구분한다.
      //   INIT      : 위치 확정 전 -- QR 을 놓치면 임무 전체가 어긋난다
      //   *_enter   : 구역 진입 -- 정지 QR 을 놓치면 엉뚱한 데서 멈춘다
      //   그 외      : 평상시
      float baseNow;
      if (latestPos.startsWith("INIT"))      baseNow = tune.initBase;
      else if (latestPos.endsWith("_enter")) baseNow = tune.zoneEnterBase;
      else                                   baseNow = tune.base;
      float steerLimit = baseNow * 0.6f;

      // 8/21 개정 (3차) -- 라인 상실(000) 시 좌우 스윕 탐색.
      // 제자리에서 좌우로 번갈아 돌되 구간을 점점 길게 잡아, 라인이
      // 앞에 있는지 훑는다. 전진하지 않으므로 그동안 더 멀어지지 않는다.
      // 8/21 -- 잠깐 스치는 000 은 탐색으로 보지 않는다. lineLostSince 는
      // 위에서 이미 "연속으로 라인이 없는 시각"을 잡아두고 있으므로 그걸 쓴다.
      bool searchDue = !lineDetected && lineLostSince != 0 &&
                       (now - lineLostSince >= tune.searchStartDelayMs);
      if (searchDue) {
        if (!lineSearchActive) {
          lineSearchActive = true;
          lineSearchLegIdx = 0;
          // 첫 방향은 마지막으로 본 쪽. 근거가 없으면(010 에서 곧장 000
          // 으로 빠진 경우) 일단 좌회전으로 시작한다 -- 어차피 곧
          // 반대쪽도 훑으므로 첫 선택이 틀려도 회복된다.
          lineSearchDir = (lastNonZeroError > 0) ? 1
                        : (lastNonZeroError < 0) ? -1 : 1;
          lineSearchLegUntil = now + tune.searchLegMs;
          publishMark("line_search_start");
        } else if (now >= lineSearchLegUntil) {
          lineSearchLegIdx++;
          lineSearchDir = -lineSearchDir;   // 반대쪽으로
          // 구간마다 길이를 키워 진폭을 넓힌다 (150, 300, 450, ...)
          lineSearchLegUntil = now + tune.searchLegMs * (lineSearchLegIdx + 1);
        }
        float rot = tune.searchPivotRpm * (float)lineSearchDir;
        targetL = -rot;   // 제자리 회전 (좌우 반대)
        targetR =  rot;
        break;
      }
      if (lineDetected) lineSearchActive = false;   // 라인을 물었으면 탐색 종료

      steer = constrain(steer, -steerLimit, steerLimit);

      targetL = baseNow - steer;
      targetR = baseNow + steer;
      break;
    }

    case RobotState::TEST: {
      if (now > testStepUntil) {
        testStep = (testStep + 1) % 4;
        testStepUntil = now + 500;
      }
      switch (testStep) {
        case 0: targetL = 80; targetR = 0; break;
        case 1: targetL = 0;  targetR = 0; break;
        case 2: targetL = 0;  targetR = 80; break;
        case 3: targetL = 0;  targetR = 0; break;
      }
      break;
    }

    case RobotState::PH_STOP:
    case RobotState::STOPPED:
    case RobotState::STALL:
    default:
      targetL = targetR = 0;
      stopAll();
      skipPid = true;
  }

  if (!skipPid) {
    int pwmL, pwmR;

    if (USE_ENCODER_PID) {
      // ── 폐루프 : 엔코더 rpm 을 피드백으로 속도 PID ──
      if (targetL == 0) { pwmL = 0; pidL.integral = 0; pidL.prevError = 0; }
      else { pwmL = (int)computePID(pidL, targetL, rpmL, dt); }

      if (targetR == 0) { pwmR = 0; pidR.integral = 0; pidR.prevError = 0; }
      else { pwmR = (int)computePID(pidR, targetR, rpmR, dt); }
    } else {
      // ── 개루프 : 목표 rpm 을 PWM 으로 직접 환산 ──
      // 엔코더를 전혀 쓰지 않는다. rpm 눈금(base=50)을 PWM 눈금
      // (OPENLOOP_BASE=110)으로 비례 확대한 뒤, 목표가 0 이 아닌데 환산
      // PWM 이 기동 문턱보다 낮으면 최소 듀티로 끌어올린다.
      float scale = OPENLOOP_BASE / tune.base;
      pwmL = (int)(targetL * scale * OPENLOOP_RPM_TO_PWM);
      pwmR = (int)(targetR * scale * OPENLOOP_RPM_TO_PWM);

      if (targetL != 0 && abs(pwmL) < OPENLOOP_MIN_PWM)
        pwmL = (pwmL >= 0) ? OPENLOOP_MIN_PWM : -OPENLOOP_MIN_PWM;
      if (targetR != 0 && abs(pwmR) < OPENLOOP_MIN_PWM)
        pwmR = (pwmR >= 0) ? OPENLOOP_MIN_PWM : -OPENLOOP_MIN_PWM;

      pwmL = constrain(pwmL, -255, 255);
      pwmR = constrain(pwmR, -255, 255);
    }

    driveLeft(pwmL);
    driveRight(pwmR);

    static unsigned long lastDebug = 0;
    if (now - lastDebug > 200) {
      lastDebug = now;
      Serial.printf("[DBG] %s targetL=%.0f targetR=%.0f rpmL=%.1f rpmR=%.1f pwmL=%d pwmR=%d pos=%s tvec=%.0f\n",
                    USE_ENCODER_PID ? "PID" : "OPEN",
                    targetL, targetR, rpmL, rpmR, pwmL, pwmR,
                    latestPos.c_str(), latestTVecDeg);
    }
  }

  // 8/20 수정 -- 이 블록이 원래 if (!skipPid) 안에 들어 있었다.
  // STOPPED / PH_STOP / 회전 단계에서는 skipPid = true 라 상태 발행이
  // 통째로 건너뛰어졌고, 그 결과 로봇이 멈춰 있는 동안 robot/1/state 가
  // 아예 안 나갔다. 라파에서 보면 ESP32 가 죽은 것처럼 보인다.
  // 상태 보고는 구동 여부와 무관해야 하므로 루프 최상위로 뺀다.
  if (now - lastPublish > 500) {
    lastPublish = now;
    publishState(rpmL, rpmR, l, c, r);
  }
}
