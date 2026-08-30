# auth.py — 가장 기본적인 회원가입/로그인
#
# 토큰·세션 관리 없이, 로그인 성공 시 사용자 정보만 그대로 돌려주는 최소 버전입니다.
# (보안을 제대로 갖추려면 JWT 토큰, 만료시간, 인증 미들웨어가 필요하지만
#  지금은 "가입하고 로그인이 되는지"만 확인하는 단계라 생략했습니다)
#
# bcrypt를 직접 사용합니다 (passlib은 최신 bcrypt와 호환성 문제가 있어 제외).

import bcrypt

from database import get_connection


def hash_password(password: str) -> str:
    hashed = bcrypt.hashpw(password.encode("utf-8"), bcrypt.gensalt())
    return hashed.decode("utf-8")


def check_password(password: str, hashed: str) -> bool:
    return bcrypt.checkpw(password.encode("utf-8"), hashed.encode("utf-8"))


def create_user(username: str, password: str, role: str = "staff") -> int:
    """회원가입. role: 'admin'(관리자 웹) 또는 'staff'(직원 앱). 성공하면 새 user id를 돌려줍니다."""
    hashed = hash_password(password)

    conn = get_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute(
                "INSERT INTO users (username, password_hash, role) VALUES (%s, %s, %s)",
                (username, hashed, role),
            )
            user_id = cursor.lastrowid
        conn.commit()
        return user_id
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()


def verify_user(username: str, password: str) -> dict | None:
    """로그인. 아이디·비밀번호가 맞으면 사용자 정보를, 틀리면 None을 돌려줍니다."""
    conn = get_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute("SELECT * FROM users WHERE username = %s", (username,))
            user = cursor.fetchone()
    finally:
        conn.close()

    if not user:
        return None
    if not check_password(password, user["password_hash"]):
        return None

    return {"id": user["id"], "username": user["username"], "role": user["role"]}
