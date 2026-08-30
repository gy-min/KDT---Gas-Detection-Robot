#!/usr/bin/env python3
"""
라파5 <-> ESP32 UART 양방향 통신 검증용 테스트 스크립트.
QR 인식 로직 없이, 고정/순환 값을 주기적으로 쏘면서 동시에
ESP32가 보내오는 것도 실시간으로 화면에 표시한다.

사용 전 확인:
1. 라즈베리파이 UART 활성화 여부
   sudo raspi-config
   -> 3 Interface Options -> I5 Serial Port
   -> "로그인 셸을 시리얼로 접속?" -> No
   -> "시리얼 포트 하드웨어 활성화?" -> Yes
   -> 재부팅

2. 사용할 포트 확인 (라즈베리파이 5 기준 보통 /dev/ttyAMA0)
   ls -l /dev/serial0
   -> 실제로 가리키는 장치(ttyAMA0 등)를 아래 PORT 에 넣을 것

3. pyserial 설치
   pip3 install pyserial --break-system-packages

배선 (교차 연결 확인):
  라파 GPIO14 (TXD, 8번 핀)  -> ESP32 GPIO13 (RX)
  라파 GPIO15 (RXD, 10번 핀) <- ESP32 GPIO14 (TX)
  GND 공통
"""

import serial
import time
import sys
import threading

PORT = "/dev/ttyAMA0"   # ls -l /dev/serial0 결과에 맞춰 수정
BAUD = 115200

# 테스트용 순환 시나리오: 직진 -> 우회전 -> 좌회전 -> 직진 ...
TEST_SEQUENCE = [
    ("A1", 0),      # 직진 (|각도| <= 30)
    ("A1", 45),     # 우회전 (임계값 30 초과, 양수)
    ("B2", 0),      # 직진
    ("B2", -45),    # 좌회전 (임계값 30 초과, 음수)
    ("C3", 0),      # 직진
]


def read_loop(ser):
    """ESP32가 보내오는 걸 계속 읽어서 화면에 표시 (별도 스레드)."""
    buf = ""
    while True:
        try:
            chunk = ser.read(ser.in_waiting or 1).decode("utf-8", errors="replace")
        except serial.SerialException:
            break
        if not chunk:
            continue
        buf += chunk
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if line:
                print(f"  <- 받음: {line}")


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except serial.SerialException as e:
        print(f"포트 열기 실패: {e}")
        print("포트 이름이 맞는지, dialout 그룹에 속해 있는지 확인하세요.")
        print("  sudo usermod -aG dialout $USER   (이후 재로그인 필요)")
        sys.exit(1)

    print(f"=== UART 양방향 테스트 시작 ({PORT}, {BAUD}bps) ===")
    print("Ctrl+C 로 종료\n")

    t = threading.Thread(target=read_loop, args=(ser,), daemon=True)
    t.start()

    idx = 0
    try:
        while True:
            pos, angle = TEST_SEQUENCE[idx % len(TEST_SEQUENCE)]
            line = f"POS:{pos},TVEC:{angle}\n"
            ser.write(line.encode("utf-8"))
            print(f"보냄: {line.strip()}")
            idx += 1
            time.sleep(1.0)   # 1초마다 갱신 (지속 스트리밍 시뮬레이션)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        ser.close()

if __name__ == "__main__":
    main()