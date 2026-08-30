#!/usr/bin/env python3
# 가스(MQ x5) + 온습도(DHT11) 값을 읽어 MQTT로 노트북에 CSV 형태로 전송
# 라즈베리파이 5 / Raspberry Pi OS
#
# 보내는 CSV 열 순서:
#   ts, temperature, humidity, MQ-2_1, MQ-2_2, MQ-3, MQ-6, MQ-135
#   예) 1699999999,25.0,40,320,310,150,200,400

import os
import time

import paho.mqtt.client as mqtt
from gpiozero import MCP3008
import board
import adafruit_dht

# ---------------- 설정 ----------------
BROKER = os.environ.get("MQTT_BROKER", "YOUR_BROKER_IP")   # 노트북(MQTT 브로커) IP
PORT = 1883
ROBOT_ID = 1
TOPIC = f"robot/{ROBOT_ID}/sensors"
INTERVAL = 2.0              # 전송 주기(초)

DIVIDER = 2.0              # 전압분배 배율 (10k+10k -> 2.0, 10k+15k -> 1.667)
VREF = 3.3

# MCP3008 채널 : 센서 이름  (CSV 열 순서가 이 순서대로 나감)
SENSORS = {
    0: "MQ-2_1",
    1: "MQ-2_2",
    2: "MQ-3",
    3: "MQ-6",
    4: "MQ-135",
}

DHT_PIN = board.D4         # DHT11 DATA -> GPIO4 (물리 7번). 다른 핀이면 D17 등으로 변경
# --------------------------------------

# MCP3008 채널 준비 (SPI0, device0 = CE0/GPIO8)
adc = {ch: MCP3008(channel=ch, port=0, device=0) for ch in SENSORS}

# DHT11 준비
dht = adafruit_dht.DHT11(DHT_PIN)

# MQTT 클라이언트 (paho 1.x / 2.x 모두 호환)
try:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
except (AttributeError, TypeError):
    client = mqtt.Client()


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[MQTT] connected (code={reason_code})")


client.on_connect = on_connect
client.connect(BROKER, PORT, 60)
client.loop_start()   # 백그라운드 연결 유지 + 끊기면 자동 재연결

# 온습도 마지막 성공값 (읽기 실패 시 이전 값 유지)
last_temp = None
last_humi = None


def csv_field(x):
    # None이면 빈 칸으로 (엑셀/파싱 편하게)
    return "" if x is None else str(x)


# CSV 헤더(참고용): 노트북 쪽에서 이 순서를 알고 있으면 됨
HEADER = "ts,temperature,humidity," + ",".join(SENSORS.values())
print("CSV columns:", HEADER)
print("start. Ctrl+C to stop\n")

try:
    while True:
        # --- 가스 5채널 읽기 (SENSORS 순서대로) ---
        gas = []
        for ch in SENSORS:
            v = adc[ch].value             # 0.0 ~ 1.0
            gas.append(round(v * 1023))   # raw 0~1023

        # --- 온습도 읽기 (DHT11은 가끔 읽기 실패가 정상) ---
        try:
            t = dht.temperature
            h = dht.humidity
            if t is not None:
                last_temp = t
            if h is not None:
                last_humi = h
        except RuntimeError:
            pass   # 이번 읽기만 실패 -> 이전 값 유지

        # --- CSV 한 줄 만들기 ---
        ts = round(time.time())
        fields = [str(ts), csv_field(last_temp), csv_field(last_humi)]
        fields += [str(g) for g in gas]
        row = ",".join(fields)

        client.publish(TOPIC, row)
        print("published:", row)

        time.sleep(INTERVAL)

except KeyboardInterrupt:
    pass
finally:
    client.loop_stop()
    client.disconnect()
    try:
        dht.exit()
    except Exception:
        pass
    print("\nstopped")
