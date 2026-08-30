from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse
from starlette.middleware.base import BaseHTTPMiddleware
import cv2
import asyncio
import threading
import time

app = FastAPI()


class PrivateNetworkMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request, call_next):
        response = await call_next(request)
        response.headers["Access-Control-Allow-Private-Network"] = "true"
        response.headers["Access-Control-Allow-Origin"] = "*"
        return response


app.add_middleware(PrivateNetworkMiddleware)

cap = cv2.VideoCapture(8)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

latest_jpeg = None
lock = threading.Lock()


def capture_loop():
    global latest_jpeg
    while True:
        ok, frame = cap.read()
        if ok:
            frame = cv2.flip(frame, 1)
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            with lock:
                latest_jpeg = buf.tobytes()
        time.sleep(0.05)


threading.Thread(target=capture_loop, daemon=True).start()


async def gen_frames():
    while True:
        await asyncio.sleep(0.05)
        with lock:
            frame = latest_jpeg
        if frame is None:
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.get('/')
def index():
    return HTMLResponse('<html><body style="margin:0;background:#000"><img src="/stream" style="width:100%"></body></html>')


@app.get('/stream')
def stream():
    return StreamingResponse(gen_frames(), media_type='multipart/x-mixed-replace; boundary=frame')
