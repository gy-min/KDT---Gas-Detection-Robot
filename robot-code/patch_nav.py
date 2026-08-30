#!/usr/bin/env python3
"""
rpi_qr_navigator.py 패치 -- 경로 이탈 시 자동 재탐색 (8/20)

적용 내용
  1) _QR_TO_EDGE 에 반대편 벽 좌표 추가
  2) OFF_AXIS_SANITY_BOUND 3.0 -> 2.0
  3) find_path_no_reverse / edge_heading_nodes 헬퍼 추가
  4) check_off_route_edge 를 "정지" 에서 "재탐색" 으로 교체
  5) 방향 판정용 상태 필드(_off_route_prev) 추가 및 갱신

사용법
    python3 patch_nav.py rpi_qr_navigator.py
원본은 rpi_qr_navigator.py.bak 으로 백업된다. 이미 적용된 항목은 건너뛴다.
"""
import sys, re, shutil, os

NEW_QR_TO_EDGE = '''# 8/20 개정 -- 반대편 벽 좌표까지 등록.
# EDGES 는 한쪽 벽만 담고 있어, 카메라가 반대편 벽을 볼 때 읽는 좌표가
# 전부 "미등록 = 판단 불가"로 무시됐다. 실측에서 (10,7)/(9,5)/(8,5) 를
# 읽으며 E-F, C-E 복도를 지나가는데도 경고 하나 없이 진행됐다.
_OPP_ROW = {0: 1, 1: 0, 5: 6, 6: 5, 11: 10, 10: 11}
_OPP_COL = {3: 5, 5: 3, 7: 9, 9: 7}

_QR_TO_EDGE = {}
for _e, _qrs in EDGES.items():
    _a, _b = _e
    _ca, _cb = NODES[_a], NODES[_b]
    _ax = "row" if abs(_ca[0] - _cb[0]) > abs(_ca[1] - _cb[1]) else "col"
    for _q in _qrs:
        _r, _c = int(_q[0]), int(_q[1])
        _both = [(_r, _c)]
        if _ax == "col":          # 가로 복도 -- 반대편 벽은 다른 '행'
            _both.append((_OPP_ROW.get(_r, _r), _c))
        else:                     # 세로 복도 -- 반대편 벽은 다른 '열'
            _both.append((_r, _OPP_COL.get(_c, _c)))
        for _k in _both:
            _QR_TO_EDGE.setdefault(_k, set()).add(frozenset(_e))
'''

HELPERS = '''

def find_path_no_reverse(start, came_from, goal):
    """start 에서 goal 까지, 첫 걸음이 came_from 으로 되돌아가지 않는 최단 경로."""
    from collections import deque
    if start == goal:
        return [start]
    visited = {start}
    q = deque()
    for nxt in sorted(_adjacent(start)):
        if nxt == came_from:
            continue
        if nxt == goal:
            return [start, nxt]
        visited.add(nxt)
        q.append([start, nxt])
    while q:
        path = q.popleft()
        for nxt in sorted(_adjacent(path[-1])):
            if nxt == goal:
                return path + [nxt]
            if nxt not in visited:
                visited.add(nxt)
                q.append(path + [nxt])
    return None


def edge_heading_nodes(edge, coords, prev_coords, goal):
    """
    이탈한 간선 위에서 (향하는 끝점, 온 쪽 끝점, 판정근거) 를 고른다.
    직전에 읽은 다른 QR 과 비교해 좌표가 어느 쪽으로 움직였는지로 판정하고,
    비교할 좌표가 없으면 목적지까지 짧은 쪽으로 추정한다.
    """
    a, b = sorted(edge)
    ca, cb = NODES[a], NODES[b]
    ax = "row" if abs(ca[0] - cb[0]) > abs(ca[1] - cb[1]) else "col"
    i = 0 if ax == "row" else 1
    if prev_coords is not None and abs(coords[i] - prev_coords[i]) >= 0.5:
        increasing = coords[i] > prev_coords[i]
        if increasing:
            head = a if ca[i] > cb[i] else b
        else:
            head = a if ca[i] < cb[i] else b
        return head, (b if head == a else a), "직전 QR 이동 방향"
    pa = find_path_no_reverse(a, b, goal)
    pb = find_path_no_reverse(b, a, goal)
    head = a if (len(pa) if pa else 99) <= (len(pb) if pb else 99) else b
    return head, (b if head == a else a), "목적지까지 짧은 쪽(추정)"
'''

NEW_CHECK = '''def check_off_route_edge(coords, ser):
    """
    읽은 QR 이 현재 경로에 없는 간선의 것이면, 그 자리에서 목적지까지
    새 경로를 계산해 주행을 이어간다.

    8/20 개정 -- 기존에는 정지만 했다. "U턴 없이는 되돌아갈 수 없다"는
    이유였는데, 실제 이탈은 방향이 뒤집힌 게 아니라 다른 복도로 들어간
    경우다. 지금 향하는 끝점에서 다시 길을 찾으면 U턴 없이 실행 가능한
    경로가 나온다. 진행 방위를 가상 구간(head_leg)으로 넣어야 새 경로의
    첫 교차로 회전각이 맞는다.
    """
    key = (int(round(coords[0])), int(round(coords[1])))
    edges_here = _QR_TO_EDGE.get(key)
    if not edges_here:
        return False   # EDGES 에도 반대편 벽에도 없는 좌표 -- 판단 불가

    with nav.lock:
        if nav.mode not in ("NAVIGATE", "RETURN"):
            nav._off_route_hits = 0
            return False
        route = list(nav.node_route)
        mode_now = nav.mode
        zone_backup = nav.zone_code
        zone_entry_backup = nav.zone_entry_node
        prev_coords = nav._off_route_prev

    if len(route) < 2:
        return False

    route_edges = {frozenset((route[i], route[i + 1])) for i in range(len(route) - 1)}
    if edges_here & route_edges:
        with nav.lock:
            nav._off_route_hits = 0
        return False   # 경로상의 간선 -- 정상

    # 노드 근처면 통과 순간의 스침일 수 있으므로 보류
    for name in route:
        nc = node_coord(name)
        if nc is not None and distance(coords, nc) < OFF_ROUTE_EDGE_MIN_DIST:
            return False

    with nav.lock:
        nav._off_route_hits += 1
        hits = nav._off_route_hits
    if hits < OFF_ROUTE_EDGE_HITS:
        return False

    edge = next(iter(edges_here))
    edge_name = " - ".join(sorted(edge))
    goal = route[-1]

    head, came, how = edge_heading_nodes(edge, coords, prev_coords, goal)
    new_path = find_path_no_reverse(head, came, goal)

    ser.write(b"CMD:s\\n")
    with nav.lock:
        nav._off_route_hits = 0

    if not new_path:
        publish_mark(f"[경로 이탈] QR {key} 는 '{edge_name}' 간선 -- '{head}' 에서 "
                     f"'{goal}' 까지 되돌아가지 않는 경로가 없음. 정지, 수동 개입 필요")
        with nav.lock:
            nav.mode = "IDLE"
        return True

    cur_heading = heading_between(NODES[came], NODES[head])
    seg = EDGES.get((came, head)) or list(reversed(EDGES.get((head, came), [])))
    axis = edge_axis(came, head)

    publish_mark(f"[경로 이탈 -> 재탐색] QR {key} 는 '{edge_name}' 간선. "
                 f"진행 방향 판정({how}) -> '{head}' 쪽. "
                 f"새 경로: {' -> '.join(new_path)} (목표 {goal})")

    with nav.lock:
        nav.current_node = None
    _start_route(new_path, mode=mode_now,
                 head_leg=(cur_heading, list(seg), axis, coords))
    with nav.lock:
        nav.zone_code = zone_backup
        nav.zone_entry_node = zone_entry_backup
    return True
'''

PREV_TRACK = '''                    # 8/20: 좌표가 바뀐 경우에만 직전 값으로 보관
                    # (경로 이탈 시 진행 방향 판정에 쓰인다)
                    if nav.last_qr_coords is not None and \\
                       distance(coords, nav.last_qr_coords) >= 0.5:
                        nav._off_route_prev = nav.last_qr_coords
                    nav.last_qr_coords = coords'''


def apply(src):
    log = []

    # ── 1) _QR_TO_EDGE ──
    if "_OPP_ROW" in src:
        log.append(("1) _QR_TO_EDGE 반대편 벽", "이미 적용됨 -- 건너뜀"))
    else:
        pat = re.compile(
            r"_QR_TO_EDGE = \{\}\n"
            r"for _e, _qrs in EDGES\.items\(\):\n"
            r"    for _q in _qrs:\n"
            r"        _QR_TO_EDGE\.setdefault\(\(int\(_q\[0\]\), int\(_q\[1\]\)\), set\(\)\)\.add\(frozenset\(_e\)\)\n")
        if pat.search(src):
            src = pat.sub(NEW_QR_TO_EDGE, src, count=1)
            log.append(("1) _QR_TO_EDGE 반대편 벽", "적용"))
        else:
            log.append(("1) _QR_TO_EDGE 반대편 벽", "!! 원본 블록을 못 찾음 -- 수동 확인 필요"))

    # ── 2) OFF_AXIS_SANITY_BOUND ──
    if re.search(r"OFF_AXIS_SANITY_BOUND = 2\.0", src):
        log.append(("2) OFF_AXIS_SANITY_BOUND", "이미 2.0 -- 건너뜀"))
    elif re.search(r"OFF_AXIS_SANITY_BOUND = 3\.0", src):
        src = src.replace(
            "OFF_AXIS_SANITY_BOUND = 3.0",
            "# 8/20 재조정 -- 3.0 -> 2.0. 실패 케이스가 정확히 3.0 이었고 비교가\n"
            "# \"초과\"(>)라 경계값이 그대로 통과했다. F->D 구간에서 C-E 복도의 (8,5)를\n"
            "# 읽었는데 진행축 2.5 <= 2.6, 반대축 3.0 > 3.0 이 False 가 되어\n"
            "# 'D 도착'으로 인정됐다. 정상 QR 의 반대축 거리는 최대 1.0 이라 2.0 이면 안전.\n"
            "OFF_AXIS_SANITY_BOUND = 2.0", 1)
        log.append(("2) OFF_AXIS_SANITY_BOUND", "3.0 -> 2.0 적용"))
    else:
        log.append(("2) OFF_AXIS_SANITY_BOUND", "!! 못 찾음 -- 수동 확인 필요"))

    # ── 3) 헬퍼 추가 ──
    if "def find_path_no_reverse" in src:
        log.append(("3) 헬퍼 함수", "이미 적용됨 -- 건너뜀"))
    else:
        anchor = "def _zone_segment(zone_code, entry_node):"
        if anchor in src:
            src = src.replace(anchor, HELPERS.strip() + "\n\n\n" + anchor, 1)
            log.append(("3) 헬퍼 함수", "적용"))
        else:
            log.append(("3) 헬퍼 함수", "!! 삽입 위치를 못 찾음 -- 수동 확인 필요"))

    # ── 4) check_off_route_edge 교체 ──
    if "경로 이탈 -> 재탐색" in src:
        log.append(("4) check_off_route_edge", "이미 적용됨 -- 건너뜀"))
    else:
        start = src.find("def check_off_route_edge(coords, ser):")
        end = src.find("def check_off_route_and_reroute(coords, ser):")
        if start != -1 and end != -1 and start < end:
            src = src[:start] + NEW_CHECK + "\n\n" + src[end:]
            log.append(("4) check_off_route_edge", "재탐색 방식으로 교체"))
        else:
            log.append(("4) check_off_route_edge", "!! 함수 범위를 못 찾음 -- 수동 확인 필요"))

    # ── 5) 상태 필드 + 갱신 ──
    if "_off_route_prev" in src and "self._off_route_prev" in src:
        log.append(("5) _off_route_prev 필드", "이미 적용됨 -- 건너뜀"))
    else:
        f_anchor = "        self._off_route_hits = 0"
        if f_anchor in src:
            src = src.replace(
                f_anchor,
                f_anchor + "        # 8/20: 경로 밖 간선 QR 연속 확인 횟수\n"
                "        self._off_route_prev = None     # 8/20: 직전에 읽은 다른 QR (방향 판정용)"
                if not src[src.find(f_anchor):src.find(f_anchor) + 200].startswith(f_anchor + "        #")
                else f_anchor, 1)
            # 주석이 이미 붙어 있는 형태도 처리
            if "self._off_route_prev" not in src:
                m = re.search(r"( *self\._off_route_hits = 0[^\n]*\n)", src)
                if m:
                    src = src[:m.end()] + \
                          "        self._off_route_prev = None     # 8/20: 직전에 읽은 다른 QR (방향 판정용)\n" + \
                          src[m.end():]
            log.append(("5a) _off_route_prev 필드", "적용"))
        else:
            log.append(("5a) _off_route_prev 필드", "!! 못 찾음 -- 수동 확인 필요"))

        if "nav._off_route_prev = nav.last_qr_coords" in src:
            log.append(("5b) 직전 좌표 갱신", "이미 적용됨"))
        else:
            old = "                    nav.last_qr_coords = coords"
            if old in src:
                src = src.replace(old, PREV_TRACK, 1)
                log.append(("5b) 직전 좌표 갱신", "적용"))
            else:
                log.append(("5b) 직전 좌표 갱신", "!! 못 찾음 -- 수동 확인 필요"))

        old_reset = "        nav.last_qr_coords = None"
        if old_reset in src and "nav._off_route_prev = None\n" not in src.split("def _start_route")[-1]:
            src = src.replace(old_reset,
                              old_reset + "\n        nav._off_route_prev = None", 1)
            log.append(("5c) _start_route 초기화", "적용"))

    return src, log


def main():
    if len(sys.argv) != 2:
        print("사용법: python3 patch_nav.py rpi_qr_navigator.py")
        sys.exit(1)
    path = sys.argv[1]
    if not os.path.exists(path):
        print(f"파일이 없습니다: {path}")
        sys.exit(1)

    src = open(path, encoding="utf-8").read()
    out, log = apply(src)

    print("=" * 60)
    for name, status in log:
        mark = "!!" if status.startswith("!!") else "OK"
        print(f"  [{mark}] {name:28s} {status}")
    print("=" * 60)

    # 구문 검사
    try:
        compile(out, path, "exec")
    except SyntaxError as e:
        print(f"\n[중단] 패치 결과에 구문 오류: {e}")
        print("원본은 그대로 두었습니다.")
        sys.exit(1)

    shutil.copy(path, path + ".bak")
    open(path, "w", encoding="utf-8").write(out)
    print(f"\n구문 검사 통과. 백업: {path}.bak")
    print(f"적용 완료: {path}")


if __name__ == "__main__":
    main()