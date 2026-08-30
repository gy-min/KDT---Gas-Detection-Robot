# evacuation.py — 다익스트라 기반 대피 경로 계산
#
# 8/20 전면 재작성 -- 기존 버전은 x=0/4/8 "사다리형" 좌표계로 따로 만들어져
# 있었는데, 실제 로봇이 쓰는 지도(map_data.py: TL/TR/BL/BR/ML/MR/A~F 노드 +
# EDGES)와 전혀 다른 공간이었다. 그래서 로봇/센서가 보내는 실제 좌표를 그대로
# 넣으면 엉뚱한 경로가 나왔다. 이제 map_data.py를 그대로 가져와 같은 지도
# 위에서 계산한다 (라파가 쓰는 지도 = 서버가 대피경로 계산에 쓰는 지도, 동일).
#
# 그래프: map_data.EDGES의 노드 쌍 = 정점, 그 구간 QR 좌표 개수 = 가중치
# (실제 물리적 거리에 비례). 위험 구역(zone_A1 등)이 뜨면 그 구역이 걸쳐있는
# EDGES 구간을 통째로 그래프에서 제거해 우회시킨다.

import heapq

from map_data import NODES, EDGES, EXIT_NODES, ZONES, nearest_node_to_coord

Node = str


def _edge_key(a: Node, b: Node):
    return frozenset((a, b))


def resolve_danger_edges(danger_zone_codes: list[str]) -> set[frozenset]:
    """['A2', 'zone_C1', ...] 같은 위험 구역 코드 목록을 실제 차단할 간선
    집합으로 바꾼다. 'zone_' 접두사가 있으면 떼고, 대문자로 맞춘다.
    ZONES에 없는 코드(레거시 zone_A/B/C/D 등)는 무시한다."""
    blocked: set[frozenset] = set()
    for raw in danger_zone_codes:
        code = raw[len("zone_"):] if raw.lower().startswith("zone_") else raw
        code = code.upper()
        pair = ZONES.get(code)
        if pair:
            blocked.add(_edge_key(*pair))
    return blocked


def build_graph(blocked_edges: set = frozenset()) -> dict[Node, list[tuple[Node, int]]]:
    """map_data.EDGES로부터 가중치 그래프를 만든다. 가중치 = 그 구간 QR 좌표
    개수(실제 이동 거리에 비례). blocked_edges에 걸린 구간은 통째로 뺀다."""
    graph: dict[Node, list[tuple[Node, int]]] = {n: [] for n in NODES}
    for (a, b), qr_coords in EDGES.items():
        if _edge_key(a, b) in blocked_edges:
            continue
        weight = len(qr_coords)
        graph[a].append((b, weight))
        graph[b].append((a, weight))
    return graph


def dijkstra_nearest_exit(
    graph: dict[Node, list[tuple[Node, int]]],
    start: Node,
    exits: list[Node] = EXIT_NODES,
) -> list[Node] | None:
    """start에서 exits 중 가장 가까운(가중치 합 최소) 곳까지의 경로(노드 이름 리스트)."""
    if start not in graph:
        return None

    distances: dict[Node, float] = {n: float("inf") for n in graph}
    distances[start] = 0
    previous: dict[Node, Node | None] = {n: None for n in graph}
    visited: set[Node] = set()
    queue: list[tuple[float, Node]] = [(0, start)]

    while queue:
        dist, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        for neighbor, weight in graph.get(current, []):
            new_dist = dist + weight
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                previous[neighbor] = current
                heapq.heappush(queue, (new_dist, neighbor))

    reachable = [e for e in exits if e in distances and distances[e] < float("inf")]
    if not reachable:
        return None
    best_exit = min(reachable, key=lambda e: distances[e])

    path: list[Node] = []
    node: Node | None = best_exit
    while node is not None:
        path.append(node)
        node = previous[node]
    path.reverse()
    return path


def resolve_start_node(current_location: str) -> Node | None:
    """current_location이 노드 이름(예: "C")이면 그대로, QR 좌표 문자열
    (예: "(5,5)")이면 가장 가까운 노드로 변환한다. 어느 쪽도 아니면 None."""
    text = current_location.strip()
    if text in NODES:
        return text

    import re
    m = re.match(r"^\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)$", text)
    if not m:
        return None
    coord = (float(m.group(1)), float(m.group(2)))
    return nearest_node_to_coord(coord, max_dist=3.0)


def compute_route(current_location: str, danger_zone_codes: list[str]) -> list[str] | None:
    """현재 위치(노드 이름 또는 "(row,col)" QR 좌표)와 위험 구역 코드 목록을
    받아, 위험 구간을 피해 가장 가까운 출구까지의 경로(노드 이름 리스트)를
    반환한다. 경로가 없으면(고립됨) None."""
    start = resolve_start_node(current_location)
    if start is None:
        raise ValueError(f"위치를 노드로 변환할 수 없습니다: {current_location}")

    blocked = resolve_danger_edges(danger_zone_codes)
    graph = build_graph(blocked)
    return dijkstra_nearest_exit(graph, start)
