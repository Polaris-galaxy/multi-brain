from heapq import heappop, heappush

nodes = {
    "A": (0, 0),
    "B": (2, 0),
    "C": (4, 0),
    "D": (0, 2),
    "E": (2, 2),
}

edges = [
    ("A", "B", 2.0),
    ("B", "C", 2.0),
    ("A", "D", 2.0),
    ("D", "E", 2.0),
    ("E", "B", 2.0),
]

PENDING = "PENDING"
CLAIMED = "CLAIMED"
COVERED = "COVERED"


def edge_key(a, b):
    """无向边统一成按名称排序的键，E-B 和 B-E 视为同一条路线。"""
    return tuple(sorted((a, b)))


def build_graph(edges):
    """邻接表保存 (邻居, 路线长度)。"""
    graph = {}
    for a, b, length in edges:
        if a == b:
            raise ValueError(f"不允许自环: {a}")
        if length <= 0:
            raise ValueError(f"边长必须大于 0: {a}-{b}")
        graph.setdefault(a, [])
        graph.setdefault(b, [])
        if not any(neighbor == b for neighbor, _length in graph[a]):
            graph[a].append((b, length))
        if not any(neighbor == a for neighbor, _length in graph[b]):
            graph[b].append((a, length))
    for neighbors in graph.values():
        neighbors.sort(key=lambda item: item[0])
    return graph


def dijkstra(graph, start):
    """按路线长度计算最短距离，并记录前驱。"""
    distance = {start: 0.0}
    parent = {start: None}
    heap = [(0.0, start)]
    while heap:
        dist, current = heappop(heap)
        if dist > distance[current]:
            continue
        for neighbor, length in graph[current]:
            new_dist = dist + length
            if neighbor not in distance or new_dist < distance[neighbor]:
                distance[neighbor] = new_dist
                parent[neighbor] = current
                heappush(heap, (new_dist, neighbor))
    return distance, parent


def validate_graph(graph, nodes):
    """检查坐标点都有路线、没有孤立点，并且整张图连通。"""
    if len(graph) < 2:
        raise ValueError("至少需要两个节点")
    missing_nodes = set(nodes) - set(graph)
    if missing_nodes:
        raise ValueError(f"坐标节点没有路线: {sorted(missing_nodes)}")
    isolated = [node for node, neighbors in graph.items() if not neighbors]
    if isolated:
        raise ValueError(f"存在没有连接的节点: {isolated}")
    start = next(iter(graph))
    distance, _parent = dijkstra(graph, start)
    missing = set(graph) - set(distance)
    if missing:
        raise ValueError(f"图不连通，到不了: {sorted(missing)}")


def shortest_path(parent, target):
    """从目标沿前驱回到起点，再反转成实际路线。"""
    path = []
    while target is not None:
        path.append(target)
        target = parent[target]
    path.reverse()
    return path


def assign_nearest(graph, edge_state, edge_length, robots):
    """给空闲机器人分配代价最小的未覆盖路线。"""
    assignments = {}
    for robot_id in sorted(robots):
        robot = robots[robot_id]
        if robot["target_edge"] or robot["status"] == "BLOCKED":
            continue

        distance, _parent = dijkstra(graph, robot["current"])
        candidates = []
        for edge, state in edge_state.items():
            if state != PENDING:
                continue
            travel = min(distance[edge[0]], distance[edge[1]])
            cost = travel + edge_length[edge]
            candidates.append((cost, edge))

        if not candidates:
            assignments[robot_id] = None
            continue

        _cost, edge = min(candidates)
        edge_state[edge] = CLAIMED
        robot["target_edge"] = edge
        robot["edge_progress"] = 0.0
        robot["finished_edge"] = None
        assignments[robot_id] = edge
    return assignments


def one_step(graph, edge_state, robot):
    """在目标路线上时直接覆盖；否则只朝较近端点移动一跳。"""
    edge = robot["target_edge"]
    if edge is None:
        return "IDLE"

    start, end = edge
    if robot["current"] not in edge:
        distance, parent = dijkstra(graph, robot["current"])
        goal = start if distance[start] <= distance[end] else end
        path = shortest_path(parent, goal)
        robot["current"] = path[1]
        robot["edge_progress"] = 0.0
        return "MOVE"

    other = end if robot["current"] == start else start
    robot["current"] = other
    robot["edge_progress"] = 1.0
    robot["finished_edge"] = edge
    robot["target_edge"] = None
    edge_state[edge] = COVERED
    return "COVER"


graph = build_graph(edges)
validate_graph(graph, nodes)
print("节点数:", len(graph))
print(graph)

edge_length = {edge_key(a, b): length for a, b, length in edges}
edge_state = {edge: PENDING for edge in edge_length}
robots = {
    "robot_1": {
        "current": "A",
        "target_edge": None,
        "status": "IDLE",
        "edge_progress": 0.0,
        "finished_edge": None,
    },
    "robot_2": {
        "current": "C",
        "target_edge": None,
        "status": "IDLE",
        "edge_progress": 0.0,
        "finished_edge": None,
    },
}

round_id = 0
while any(state != COVERED for state in edge_state.values()):
    round_id += 1
    assignments = assign_nearest(graph, edge_state, edge_length, robots)
    print(f"第{round_id}轮分配:", assignments)
    for robot_id in sorted(robots):
        robot = robots[robot_id]
        action = one_step(graph, edge_state, robot)
        print(
            robot_id,
            action,
            "当前位置", robot["current"],
            "目标路线", robot["target_edge"],
            "进度", robot["edge_progress"],
            "刚完成", robot["finished_edge"],
        )

print(edge_state)