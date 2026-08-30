"""
가스 탐지 로봇 맵 데이터 (2026-08-19 칠판 확정본 기준)

좌표계: (행, 열) — 행 0~11 (위→아래), 열 0~12 (왼→오)
레일: 세로 2줄(열4, 열8) + 가로 3줄(행0~1 사이, 행5~6 사이, 행10~11 사이)
노드 12개: 출구 4(TL,TR,BL,BR) + 도착점 2(ML,MR) + 교차로 6(A~F)

⚠ 교차로(A~F)의 정확한 (행,열)은 "행0~1 사이"처럼 경계로만 확인되어
있고, 정수 좌표가 아직 확정되지 않았다. 실제 주행에서는 ESP32가
라인센서로 교차로 자체를 물리적으로 감지하므로 이 좌표는 라파가
"다음 방향을 계산하기 위한 참고용"으로만 쓰인다. 아래 값은 잠정치이며
확정되면 CROSSINGS 딕셔너리만 수정하면 된다.
"""

import math

# ── 확정된 QR 좌표 (칠판에서 그대로 옮김) ──────────────
QR_COORDS = {
    # 위쪽 바깥 경계 (행0, 행1)
    "TL_approach": [(0, 0), (0, 1), (0, 2), (0, 3), (1, 0), (1, 1), (1, 2), (1, 3)],
    "TR_approach": [(0, 9), (0, 10), (0, 11), (0, 12), (1, 9), (1, 10), (1, 11), (1, 12)],
    "top_mid":     [(0, 5), (0, 6), (0, 7), (1, 5), (1, 6), (1, 7)],

    # 왼쪽 안쪽 벽 (열3, 열4 사이 접근로) — 위쪽 방
    "A_approach_upper": [(2, 3), (3, 3), (4, 3)],
    "A_approach_lower": [(7, 3), (8, 3), (9, 3)],   # C 접근
    # 오른쪽 안쪽 벽 (열7 근처) — 위쪽 방
    "B_approach_upper": [(2, 7), (3, 7), (4, 7)],
    "B_approach_lower": [(7, 7), (8, 7), (9, 7)],   # D 접근

    # 열5, 열9 쪽 (반대편 안쪽 벽) — 위 코드에서 A/B 접근은 열3/7 기준,
    # 아래는 열5/9 기준. 실제 칠판엔 양쪽 벽 모두 QR이 있음.
    "A_approach_upper_2": [(2, 5), (3, 5), (4, 5)],
    "B_approach_upper_2": [(2, 9), (3, 9), (4, 9)],
    "C_approach_2":       [(7, 5), (8, 5), (9, 5)],
    "D_approach_2":       [(7, 9), (8, 9), (9, 9)],

    # 중간 가로줄 (행5, 행6) — ML/MR/C/D 근처
    "ML_approach": [(5, 1), (5, 2), (5, 3), (6, 1), (6, 2), (6, 3)],
    "MR_approach": [(5, 9), (5, 10), (5, 11), (6, 9), (6, 10), (6, 11)],
    "mid_mid":     [(5, 5), (5, 6), (5, 7), (6, 5), (6, 6), (6, 7)],

    # 아래쪽 바깥 경계 (행10, 행11)
    "BL_approach": [(10, 0), (10, 1), (10, 2), (10, 3), (11, 0), (11, 1), (11, 2), (11, 3)],
    "BR_approach": [(10, 9), (10, 10), (10, 11), (10, 12), (11, 9), (11, 10), (11, 11), (11, 12)],
    "bottom_mid":  [(10, 5), (10, 6), (10, 7), (11, 5), (11, 6), (11, 7)],
}

# ── 12개 노드의 대표 좌표 ────────────────────────────
# 출구/도착점은 정수 좌표 확정. 교차로는 잠정치(★ 재확인 필요).
NODES = {
    "TL": (1, 0),      # 8/19 수정 -- 최북단+동쪽 응시라 오른쪽(남쪽)이 안쪽(행1)으로 들어옴
    "TR": (0, 12),     # 최북단+서쪽 응시라 오른쪽(북쪽)이 지도 밖 -- 자기 행(0) 그대로
    "BL": (11, 0),     # 최남단+동쪽 응시라 오른쪽(남쪽)이 지도 밖 -- 자기 행(11) 그대로
    "BR": (10, 12),    # 8/19 수정 -- 최남단+서쪽 응시라 오른쪽(북쪽)이 안쪽(행10)으로 들어옴
    "ML": (5, 1),      # 또는 (6,1) — 칠판 확정값
    "MR": (5, 11),     # 또는 (6,11) — 칠판 확정값
    "A":  (0.5, 4),    # 확정 — 레일 교차점(행0~1 경계), 근처 QR(0,4) 실측 확인됨
    "B":  (0.5, 8),    # 확정 — 레일 교차점(행0~1 경계), 근처 QR(0,8) 실측 확인됨
    "C":  (5.5, 4),    # 확정 — 레일 교차점(행5~6 경계). 해당 열엔 QR 없음(양옆 3/5열만 있음)
    "D":  (5.5, 8),    # 확정 — 레일 교차점(행5~6 경계). 해당 열엔 QR 없음(양옆 7/9열만 있음)
    "E":  (10.5, 4),   # 확정 — 레일 교차점(행10~11 경계), 근처 QR(11,4) 실측 확인됨
    "F":  (10.5, 8),   # 확정 — 레일 교차점(행10~11 경계), 근처 QR(11,8) 실측 확인됨
}

# ── 노드 간 연결 (인접 그래프) ────────────────────────
# 각 간선은 그 구간에서 실제로 지나가는 QR 좌표 순서를 담는다.
# 라파가 이 순서대로 QR을 읽으며 목표까지의 벡터를 계산하는 데 쓴다.
EDGES = {
    ("TL", "A"): [(0, 0), (0, 1), (0, 2), (0, 3)],
    ("A", "B"):  [(0, 5), (0, 6), (0, 7)],
    ("B", "TR"): [(0, 9), (0, 10), (0, 11), (0, 12)],

    ("ML", "C"): [(5, 1), (5, 2), (5, 3)],
    ("C", "D"):  [(5, 5), (5, 6), (5, 7)],
    ("D", "MR"): [(5, 9), (5, 10), (5, 11)],

    ("BL", "E"): [(11, 0), (11, 1), (11, 2), (11, 3)],
    ("E", "F"):  [(11, 5), (11, 6), (11, 7)],
    ("F", "BR"): [(11, 9), (11, 10), (11, 11), (11, 12)],

    ("A", "C"): [(2, 3), (3, 3), (4, 3)],   # 8/19: 축(row) 기준 매칭이라 어느 벽이든 무관 -- 대표값
    ("C", "E"): [(7, 3), (8, 3), (9, 3)],
    ("B", "D"): [(2, 7), (3, 7), (4, 7)],
    ("D", "F"): [(7, 7), (8, 7), (9, 7)],
}


def edge_axis(start_node, end_node):
    """
    8/19 추가 -- 두 노드 사이 구간이 세로(행 변화, 'row')인지
    가로(열 변화, 'col')인지 판별한다.
    카메라가 로봇 오른쪽에 고정되어 있어서, 진행 방향에 따라 어느 쪽
    벽(예: 열3 vs 열5)이 보일지가 달라진다. 로봇은 항상 그 QR의 복도
    중심 쪽에 있으므로, 진행 방향 축만 비교하면 어느 벽을 보든 맞는다.
    행/열 중 변화폭이 더 큰 쪽을 "진행 축"으로 본다 (정확히 같은지가
    아니라 크기 비교 -- 교차로 좌표가 0.5 오프셋을 쓰기 때문에 정확히
    같은 값 비교로는 TL-A 같은 가로 구간이 오판될 수 있음).
    """
    a, b = NODES[start_node], NODES[end_node]
    d_row = abs(a[0] - b[0])
    d_col = abs(a[1] - b[1])
    return "row" if d_row > d_col else "col"


def get_route(node_sequence):
    """
    ['TL', 'A', 'B', 'TR'] 같은 노드 이름 리스트를 받아서,
    그 구간들의 실제 QR 좌표를 순서대로 이어붙인 ROUTE 리스트를 만든다.
    rpi_qr_navigator.py 의 ROUTE 에 그대로 대입해서 쓸 것.
    axes 는 route 와 길이가 같은 병렬 리스트로, 각 좌표를 'row' 축만
    비교할지 'col' 축만 비교할지 알려준다 (좌/우 벽 QR 무관하게 매칭).
    """
    route = []
    axes = []
    for i in range(len(node_sequence) - 1):
        a, b = node_sequence[i], node_sequence[i + 1]
        seg = EDGES.get((a, b)) or list(reversed(EDGES.get((b, a), [])))
        if not seg:
            raise ValueError(f"{a} -> {b} 구간의 QR 경로가 정의되어 있지 않음")
        axis = edge_axis(a, b)
        route.extend(seg)
        axes.extend([axis] * len(seg))
    return route, axes


# ── 그래프 인접 정보 (경로탐색용) ──────────────────────
# EDGES 의 키(노드 쌍)로부터 양방향 인접 리스트를 만든다.
_ADJACENCY = {}
for (a, b) in EDGES.keys():
    _ADJACENCY.setdefault(a, set()).add(b)
    _ADJACENCY.setdefault(b, set()).add(a)

EXIT_NODES = ["TL", "TR", "BL", "BR"]


def find_path(start, goal):
    """
    두 노드 이름 사이의 최단 경로(변 개수 기준, BFS)를 노드 이름 리스트로 반환.
    예: find_path('C', 'TL') -> ['C', 'A', 'TL']
    경로가 없으면 None.
    """
    if start == goal:
        return [start]
    from collections import deque
    visited = {start}
    queue = deque([[start]])
    while queue:
        path = queue.popleft()
        node = path[-1]
        for nxt in _ADJACENCY.get(node, ()):
            if nxt == goal:
                return path + [nxt]
            if nxt not in visited:
                visited.add(nxt)
                queue.append(path + [nxt])
    return None


def nearest_exit(start):
    """
    start 노드에서 가장 가까운(변 개수 기준 최단) 출구(TL/TR/BL/BR)를 찾아
    (출구이름, 경로리스트) 를 반환. start 자체가 출구면 [start] 를 바로 반환.
    """
    if start in EXIT_NODES:
        return start, [start]
    best_exit, best_path = None, None
    for ex in EXIT_NODES:
        path = find_path(start, ex)
        if path is None:
            continue
        if best_path is None or len(path) < len(best_path):
            best_exit, best_path = ex, path
    if best_path is None:
        raise ValueError(f"{start} 에서 도달 가능한 출구가 없음 (EDGES 연결 확인 필요)")
    return best_exit, best_path


def nearest_node_to_coord(coord, max_dist=1.5):
    """
    (행,열) 좌표와 가장 가까운 노드 이름을 찾는다. max_dist 이내가 없으면 None.
    초기 위치 확인(모서리 QR 2개 동시 인식) 시, 읽은 좌표가 어느 노드인지
    식별하는 데 쓴다.
    """
    best_name, best_d = None, max_dist
    for name, nc in NODES.items():
        d = ((coord[0]-nc[0])**2 + (coord[1]-nc[1])**2) ** 0.5
        if d < best_d:
            best_name, best_d = name, d
    return best_name


# ── 가스 구역 코드 (고정형 센서가 MQTT로 알려주는 그 이름) ──────────
# A1~A3=윗줄 3구간, B1~B3=중간줄 3구간, C1~C3=아랫줄 3구간.
# 각 구역은 EDGES 의 한 구간(두 노드 사이)에 대응된다.
ZONES = {
    "A1": ("TL", "A"), "A2": ("A", "B"), "A3": ("B", "TR"),
    "B1": ("ML", "C"), "B2": ("C", "D"), "B3": ("D", "MR"),
    "C1": ("BL", "E"), "C2": ("E", "F"), "C3": ("F", "BR"),
}


def zone_info(zone_code, entry_node=None):
    """
    'A2' -> (진입 노드, 진입 방향 QR좌표 리스트(진입노드->중간),
             그 좌표들의 축('row'/'col'), 중간 정지 좌표, 그 구간의 절대 방위)
    entry_node 를 지정하면 그 노드에서부터 진입하는 것으로 계산한다
    (구역의 두 끝점 중 로봇 입장에서 더 가까운 쪽으로 들어가기 위함).
    지정하지 않으면 ZONES 에 정의된 기본 시작 노드를 그대로 쓴다(하위호환).
    """
    if zone_code not in ZONES:
        raise ValueError(f"알 수 없는 구역 코드: {zone_code}")
    start, end = ZONES[zone_code]
    if entry_node is None:
        entry_node = start
    if entry_node not in (start, end):
        raise ValueError(f"{zone_code} 는 {start}/{end} 로만 진입 가능 (요청: {entry_node})")
    other = end if entry_node == start else start

    full_qr = EDGES.get((entry_node, other)) or list(reversed(EDGES.get((other, entry_node), [])))
    if not full_qr:
        raise ValueError(f"{zone_code} ({entry_node}->{other}) 의 QR 경로가 정의되어 있지 않음")

    axis = edge_axis(entry_node, other)
    mid_idx = len(full_qr) // 2   # 홀수개 QR 기준 정가운데. 짝수면 뒤쪽 중앙.
    enter_qr = full_qr[:mid_idx + 1]   # 진입 지점부터 중간 지점까지만
    stop_coord = full_qr[mid_idx]
    heading = heading_between(NODES[entry_node], NODES[other])
    return entry_node, enter_qr, axis, stop_coord, heading


def heading_between(from_rc, to_rc):
    d_row = to_rc[0] - from_rc[0]
    d_col = to_rc[1] - from_rc[1]
    return math.degrees(math.atan2(d_row, d_col))


if __name__ == "__main__":
    example, example_axes = get_route(["TL", "A", "C", "ML"])
    print("예시 경로:", example)
    print("축(row/col):", example_axes)
    print("C -> 가장 가까운 출구:", nearest_exit("C"))
    print("경로 TL->F:", find_path("TL", "F"))
    print("구역 A2 정보:", zone_info("A2"))
    print("구역 B1 정보:", zone_info("B1"))