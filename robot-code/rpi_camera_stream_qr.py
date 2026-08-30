#!/usr/bin/env python3
"""
picamera2 + OpenCV QR 인식 실시간 스트리밍 서버.
브라우저에서 http://<라파IP>:8000/ 으로 접속하면,
카메라 화면에 QR이 잡힐 때마다 빨간 테두리 + 인식된 값이 표시됨.

QR 인식 로직 자체를 미리 눈으로 검증하는 용도.
확인 끝나면 Ctrl+C로 종료하고 rpi_qr_navigator.py로 넘어갈 것.
"""

import io
import time
import socketserver
from http import server
from threading import Condition, Thread

import cv2
from picamera2 import Picamera2
from libcamera import Transform

PAGE = """\
<html>
<head><title>QR 인식 확인</title></head>
<body style="background:#111; text-align:center;">
<h2 style="color:white;">QR 인식 실시간 확인</h2>
<img src="stream.mjpg" style="max-width:90%; border:2px solid #444;" />
</body>
</html>
"""


class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def set_frame(self, jpg_bytes):
        with self.condition:
            self.frame = jpg_bytes
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            content = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def capture_loop():
    qr_detector = cv2.QRCodeDetector()

    while True:
        frame = picam2.capture_array()   # RGB888, numpy array
        # OpenCV는 BGR을 기본으로 쓰므로 그리기/저장 전에 변환
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # ── 1순위: 여러 QR 동시 인식 시도 (모서리 등에서 2개 이상 볼 때) ──
        multi_results = []
        for src, inv_tag in [(frame_bgr, False), (cv2.bitwise_not(frame_bgr), True)]:
            ok, payloads, points, _ = qr_detector.detectAndDecodeMulti(src)
            if ok and points is not None:
                for p, pl in zip(points, payloads):
                    if pl:
                        multi_results.append((pl, p, inv_tag))

        if len(multi_results) >= 2:
            # 2개 이상 잡히면 전부 각각 다른 색으로 테두리 표시
            colors = [(0, 0, 255), (255, 120, 0), (0, 255, 255), (0, 255, 0)]
            for idx, (pl, pts, inv_tag) in enumerate(multi_results):
                pts_i = pts.astype(int)
                color = colors[idx % len(colors)]
                for i in range(len(pts_i)):
                    p1, p2 = tuple(pts_i[i]), tuple(pts_i[(i + 1) % len(pts_i)])
                    cv2.line(frame_bgr, p1, p2, color, 3)
                tag = " [반전]" if inv_tag else ""
                cv2.putText(frame_bgr, f"{pl}{tag}", tuple(pts_i[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
            cv2.putText(frame_bgr, f"동시 인식 {len(multi_results)}개", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)

        else:
            # ── 2순위: 단일 QR 인식 (원본 -> 반전 순으로 시도) ──
            payload, points, _ = qr_detector.detectAndDecode(frame_bgr)
            used_inverted = False
            if not payload or points is None:
                inverted = cv2.bitwise_not(frame_bgr)
                payload2, points2, _ = qr_detector.detectAndDecode(inverted)
                if payload2 or points2 is not None:
                    payload, points = payload2, points2
                    used_inverted = True

            if points is not None and len(points) > 0:
                pts = points[0].astype(int)
                for i in range(len(pts)):
                    p1 = tuple(pts[i])
                    p2 = tuple(pts[(i + 1) % len(pts)])
                    cv2.line(frame_bgr, p1, p2, (0, 0, 255), 3)   # BGR: 빨강

                text = payload if payload else "(디코딩 실패)"
                text_pos = (int(pts[0][0]), int(pts[2][1]) + 30)
                cv2.putText(frame_bgr, text, text_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

                mode_tag = " [반전 인식]" if used_inverted else ""
                cv2.putText(frame_bgr, f"QR: {text}{mode_tag}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame_bgr, "QR 없음", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

        ok, jpg = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            output.set_frame(jpg.tobytes())

        time.sleep(0.03)   # 대략 30fps 상한


picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    transform=Transform(hflip=True, vflip=True),   # 카메라가 180도 뒤집혀 장착됨
))
picam2.start()
time.sleep(1.0)

# 주행 중 QR 잔상(모션 블러) 대응 -- rpi_qr_navigator.py 와 동일 설정
try:
    picam2.set_controls({"AeEnable": False, "ExposureTime": 3000, "AnalogueGain": 6.0})
except Exception as e:
    print(f"수동 노출 설정 실패({e}), 자동노출로 계속 진행")

output = StreamingOutput()

t = Thread(target=capture_loop, daemon=True)
t.start()

try:
    address = ("", 8000)
    srv = StreamingServer(address, StreamingHandler)
    print("스트리밍 서버 시작. 브라우저에서 아래 주소로 접속하세요:")
    print("  http://<라파IP>:8000/")
    print("QR이 잡히면 빨간 테두리와 함께 값이 표시됩니다.")
    print("종료하려면 Ctrl+C")
    srv.serve_forever()
except KeyboardInterrupt:
    print("\n종료")
finally:
    picam2.stop()