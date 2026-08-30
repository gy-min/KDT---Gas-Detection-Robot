"""시리얼 장치 탐색과 연결에 필요한 공통 함수."""

import glob
import os

import serial
from serial.tools import list_ports


DEFAULT_PORT = "auto"
COMMON_PORT_PATTERNS = ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial/by-id/*")


def available_ports():
    """연결 후보를 안정적인 순서로 반환한다."""
    found = []
    for pattern in COMMON_PORT_PATTERNS:
        found.extend(sorted(glob.glob(pattern)))
    found.extend(port.device for port in list_ports.comports())
    return list(dict.fromkeys(found))


def resolve_port(configured_port=DEFAULT_PORT):
    """CLI 설정, 환경변수, 자동 탐색 순으로 사용할 포트를 결정한다."""
    requested = configured_port or DEFAULT_PORT
    env_port = os.environ.get("SERIAL_PORT")
    if requested == DEFAULT_PORT and env_port:
        requested = env_port

    if requested != DEFAULT_PORT:
        if not os.path.exists(requested):
            raise serial.SerialException(
                f"지정한 시리얼 장치가 없습니다: {requested}"
            )
        return requested

    ports = available_ports()
    if not ports:
        raise serial.SerialException(
            "연결 가능한 시리얼 장치를 찾지 못했습니다. "
            "USB 케이블/전원을 확인한 뒤 장치를 다시 연결하세요."
        )
    return ports[0]


def port_help_message():
    ports = available_ports()
    detected = ", ".join(ports) if ports else "없음"
    return (
        f"현재 감지된 포트: {detected}\n"
        "Linux에서 `python -m serial.tools.list_ports -v`로 확인하고, "
        "예: `--port /dev/ttyACM0` 또는 `SERIAL_PORT=/dev/ttyACM0`로 지정하세요.\n"
        "포트는 보이지만 Permission denied가 나면 사용자를 dialout 그룹에 추가한 뒤 "
        "다시 로그인하세요: `sudo usermod -aG dialout $USER`"
    )
