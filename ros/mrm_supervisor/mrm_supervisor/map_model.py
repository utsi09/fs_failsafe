"""MRM 후보 선택을 위한 Lanelet2 .osm 지도 모델.

[이 파일이 하는 일]
  lanelet2 형식(.osm, XML)의 지도를 라이브러리 없이 직접 파싱해서
  (local_x/local_y 좌표 사용),
  1) 도로 lanelet들 위에 가벼운 라우팅(경로 탐색) 그래프를 만들고,
  2) 지도 태그에서 안전 정차 후보를 뽑고
       - road_shoulder(갓길) lanelet
       - parking_lot(주차장) 폴리곤
  3) "여기서 저기까지 가면 거리 몇 m, 시간 몇 초, 신호등 몇 개?"라는
     경로 질의에 답한다 (supervisor.py의 비용함수가 이 답을 쓴다).

[lanelet2 용어 미니 사전]
  - node: 지도 위의 점 하나 (좌표).
  - way: 점들을 이은 선 (차선 경계선, 주차장 외곽선 등).
  - relation: way/node들을 묶은 것. type=lanelet이면 "차선 한 칸"
    (왼쪽 경계선 + 오른쪽 경계선 한 쌍으로 정의됨).
  - regulatory_element: 신호등/표지판 같은 교통 규제 요소.
"""
import heapq                          # 우선순위 큐 — 다익스트라 알고리즘에 사용
import math
import xml.etree.ElementTree as ET    # 파이썬 표준 XML 파서


def _tags(elem):
    """XML 요소 안의 <tag k="키" v="값"/> 들을 파이썬 dict로 변환하는 헬퍼."""
    return {t.get("k"): t.get("v") for t in elem.findall("tag")}


class Candidate:
    """안전 정차 후보 한 곳을 담는 단순 데이터 묶음 (구조체 역할의 클래스)."""

    def __init__(self, kind, name, x, y, yaw, target_lanelet, required_sectors):
        self.kind = kind                    # "shoulder"(갓길) | "parking_lot"(주차장)
        self.name = name                    # 로그/블랙리스트에 쓰는 고유 이름
        self.x, self.y, self.yaw = x, y, yaw  # 정차 목표 위치와 방향(라디안)
        self.target_lanelet = target_lanelet  # 라우팅 목적지로 쓸 인접 도로 lanelet id
        self.required_sectors = required_sectors  # 이 정차 기동에 필요한 감지 섹터들


class MapModel:
    # 두 lanelet의 중심선 끝점이 이 거리(m) 안이면 "이어져 있다"고 판정.
    # 지도 데이터가 완벽히 딱 맞붙어 있지 않아도 연결로 인식하기 위한 여유값.
    SUCC_EPS = 1.5

    def __init__(self, osm_path):
        root = ET.parse(osm_path).getroot()

        # --- 1단계: 점(node) 읽기. local_x/local_y가 있는 점만 사용 ---
        # (lanelet2 .osm은 위경도 대신 지역 평면 좌표를 태그로 넣어두기도 한다)
        nodes = {}
        for n in root.findall("node"):
            tg = _tags(n)
            if "local_x" in tg and "local_y" in tg:
                nodes[n.get("id")] = (float(tg["local_x"]), float(tg["local_y"]))

        # --- 2단계: 선(way) 읽기. way = 점들의 나열 → 좌표 리스트로 변환 ---
        self._ways = {}
        way_tags = {}
        for w in root.findall("way"):
            self._ways[w.get("id")] = [nodes[nd.get("ref")] for nd in w.findall("nd")
                                       if nd.get("ref") in nodes]
            way_tags[w.get("id")] = _tags(w)

        # --- 3단계: 신호등인 규제 요소(relation)들의 id를 모아둔다 ---
        tl_rel_ids = set()
        for r in root.findall("relation"):
            tg = _tags(r)
            if tg.get("type") == "regulatory_element" and tg.get("subtype") == "traffic_light":
                tl_rel_ids.add(r.get("id"))

        # --- 4단계: lanelet(차선 한 칸) 읽기 ---
        # lanelet = 왼쪽 경계선(way) + 오른쪽 경계선(way) 한 쌍.
        # 여기서 두 경계선의 "가운데"를 이어 중심선(center)을 만든다.
        self.lanelets = {}
        for r in root.findall("relation"):
            tg = _tags(r)
            if tg.get("type") != "lanelet":
                continue
            left = right = None
            has_tl = False
            for m in r.findall("member"):
                if m.get("type") == "way" and m.get("role") == "left":
                    left = m.get("ref")
                elif m.get("type") == "way" and m.get("role") == "right":
                    right = m.get("ref")
                elif m.get("type") == "relation" and m.get("ref") in tl_rel_ids:
                    has_tl = True   # 이 차선에 걸린 신호등이 있다
            if not left or not right:
                continue   # 경계선이 없는 불완전한 lanelet은 건너뜀
            lp, rp = self._ways.get(left), self._ways.get(right)
            if not lp or not rp:
                continue
            # 중심선 계산: 왼쪽 선과 오른쪽 선의 점 개수가 다를 수 있으므로,
            # 짧은 쪽 개수(n)에 맞춰 양쪽에서 "비율상 같은 위치"의 점을 골라
            # (인덱스를 i*(len-1)/(n-1)로 늘려 매핑) 둘의 중점을 취한다.
            n = min(len(lp), len(rp))
            center = [((lp[int(i * (len(lp) - 1) / max(1, n - 1))][0] +
                        rp[int(i * (len(rp) - 1) / max(1, n - 1))][0]) / 2.0,
                       (lp[int(i * (len(lp) - 1) / max(1, n - 1))][1] +
                        rp[int(i * (len(rp) - 1) / max(1, n - 1))][1]) / 2.0)
                      for i in range(n)]
            # 중심선을 따라 인접 점 사이 거리를 다 더하면 차선 길이
            length = sum(math.dist(center[i], center[i + 1]) for i in range(len(center) - 1))
            try:
                # 제한속도 태그(km/h) → m/s. 최소 1 m/s로 막아 0 나누기 방지.
                speed = max(float(tg.get("speed_limit", 50.0)) / 3.6, 1.0)  # km/h -> m/s
            except ValueError:
                speed = 50.0 / 3.6
            self.lanelets[r.get("id")] = {
                "subtype": tg.get("subtype", "road"),  # road / road_shoulder 등
                "left": left, "right": right,          # 경계선 way id (인접 판정에 사용)
                "center": center, "length": length, "has_tl": has_tl,
                "speed": speed,
            }

        # --- 5단계: 주차장 폴리곤. type=parking_lot 태그가 붙은 way 중
        # 점이 3개 이상(폴리곤이 되려면 최소 삼각형)인 것만 채택 ---
        self.parking_lots = {
            wid: pts for wid, pts in self._ways.items()
            if way_tags[wid].get("type") == "parking_lot" and len(pts) >= 3
        }

        self._build_graph()

    # ------------------------------------------------------------------ 그래프
    def _build_graph(self):
        """도로 lanelet들을 노드로 하는 방향 그래프를 만든다.

        간선(edge)은 두 종류:
          1) 후행(successor) 간선: A의 중심선 끝점 ≈ B의 중심선 시작점이면
             "A 다음에 B로 이어진다" (주행 방향이 지켜지는 연결).
          2) 인접(lane-change) 간선: 경계선 way를 공유하는 옆 차선끼리
             차선 변경으로 건너갈 수 있다.
        간선 가중치는 (거리 m, 시간 s) 두 가지를 같이 들고 다닌다 —
        라우팅은 시간으로 최적화하되, 비용함수는 거리도 필요하기 때문.
        """
        roads = {lid: ll for lid, ll in self.lanelets.items() if ll["subtype"] == "road"}
        self.graph = {lid: [] for lid in roads}   # lanelet id -> [(이웃, 거리, 시간)]

        # 경계선 way id → 그 선을 경계로 쓰는 lanelet 목록 (인접 차선 찾기용)
        by_bounds = {}
        for lid, ll in roads.items():
            by_bounds.setdefault(ll["left"], []).append(lid)
            by_bounds.setdefault(ll["right"], []).append(lid)

        # 모든 lanelet 쌍을 검사해 후행 간선 연결 (끝점-시작점 거리 < SUCC_EPS)
        lids = list(roads)
        for i, a in enumerate(lids):
            ca = roads[a]["center"]
            for b in lids[i + 1:]:
                cb = roads[b]["center"]
                # a의 끝 → b의 시작: a 다음이 b (a를 통과하는 비용을 간선에 싣는다)
                if math.dist(ca[-1], cb[0]) < self.SUCC_EPS:
                    self.graph[a].append((b, roads[a]["length"],
                                          roads[a]["length"] / roads[a]["speed"]))
                # 반대 방향도 마찬가지로 검사
                if math.dist(cb[-1], ca[0]) < self.SUCC_EPS:
                    self.graph[b].append((a, roads[b]["length"],
                                          roads[b]["length"] / roads[b]["speed"]))

        # 인접(차선 변경) 간선: 같은 경계선을 공유하는 차선들 사이.
        # 차선 변경 자체의 비용을 거리 5 m(와 그만큼의 시간)로 근사한다.
        for _, lids_shared in by_bounds.items():
            for i, a in enumerate(lids_shared):
                for b in lids_shared[i + 1:]:
                    lc_t = 5.0 / roads[a]["speed"]
                    self.graph[a].append((b, 5.0, lc_t))
                    self.graph[b].append((a, 5.0, lc_t))

    def nearest_road_lanelet(self, x, y):
        """(x, y)에서 가장 가까운 '도로' lanelet의 id를 반환.

        중심선 점들을 하나 걸러 하나만([::2]) 검사해 계산량을 절반으로 줄이고,
        비교만 하면 되므로 제곱근을 생략한 거리 제곱을 쓴다 (흔한 최적화).
        """
        best, best_d = None, float("inf")
        for lid, ll in self.lanelets.items():
            if ll["subtype"] != "road":
                continue
            for px, py in ll["center"][::2]:
                d = (px - x) ** 2 + (py - y) ** 2
                if d < best_d:
                    best, best_d = lid, d
        return best

    def route(self, src, dst):
        """도로 lanelet 그래프에서 다익스트라로 '시간 최소' 경로를 찾는다.
        반환: (거리 m, 시간 s, 경로상 신호등 개수) 또는 도달 불가 시 None."""
        if src not in self.graph or dst not in self.graph:
            return None
        best = {src: (0.0, 0.0)}          # lanelet id -> (누적 시간 s, 누적 거리 m)
        prev = {}                         # 경로 역추적용: 어느 노드에서 왔는지
        pq = [(0.0, src)]                 # (누적 시간, 노드) 우선순위 큐
        while pq:
            t, u = heapq.heappop(pq)      # 지금까지 중 가장 빨리 도착한 노드를 꺼냄
            if u == dst:
                break                     # 목적지 도착 — 다익스트라는 여기서 최적 보장
            if t > best.get(u, (float("inf"),))[0]:
                continue                  # 이미 더 좋은 기록이 있는 낡은 큐 항목은 스킵
            for v, w_len, w_time in self.graph[u]:
                nt = t + w_time
                if nt < best.get(v, (float("inf"),))[0]:
                    best[v] = (nt, best[u][1] + w_len)
                    prev[v] = u
                    heapq.heappush(pq, (nt, v))
        if dst not in best:
            return None                   # 큐가 다 비도록 목적지에 못 갔음 = 도달 불가
        # prev를 거꾸로 따라가며 경로를 복원하고, 경로 위 신호등 개수를 센다
        path = [dst]
        while path[-1] != src:
            path.append(prev[path[-1]])
        n_tl = sum(1 for lid in path if self.lanelets[lid]["has_tl"])
        return best[dst][1], best[dst][0], n_tl

    # ------------------------------------------------------------- 후보 추출
    def extract_candidates(self, shoulder_sectors, parking_sectors):
        """지도에서 안전 정차 후보(갓길 + 주차장) 목록을 만든다.

        supervisor 시작 시 한 번만 호출된다. 인자는 각 후보 종류에
        "필요한 감지 섹터" 목록 (Candidate에 그대로 실어 보낸다).
        """
        cands = []

        # ---- 갓길 후보: subtype == "road_shoulder" 인 lanelet ----
        for lid, ll in self.lanelets.items():
            if ll["subtype"] != "road_shoulder":
                continue
            c = ll["center"]
            # [7 m 길이 필터의 이유] 달리면서 옆으로 붙이는(lateral-shift)
            # 정차 기동에는 "차 길이 + 붙는 데 필요한 진입 구간"만큼의 포켓이
            # 필요하다. 실측: 6.3 m 포켓 → goal_planner가 "정차 경로 후보 0개"를
            # 영원히 반복 / 7.8 m 포켓 → 정상 주차. 그 사이인 7 m를 최소로 잡음.
            pocket_len = sum(math.dist(c[i], c[i + 1]) for i in range(len(c) - 1))
            if pocket_len < 7.0:
                continue
            # 목표 위치: 갓길 lanelet의 진짜 "가운데" 점.
            # 목표 방향(yaw): 중심선 전체의 시작→끝 방향.
            # (가운데의 인접 두 점(mid, mid+1)으로 방향을 재면 점이 2개뿐인
            #  중심선에서 계산이 무너지므로, 전체 시작→끝으로 잰다)
            if len(c) > 2:
                x, y = c[len(c) // 2]
            else:
                # 점이 2개뿐이면 두 점의 중점을 쓴다
                x = (c[0][0] + c[-1][0]) / 2.0
                y = (c[0][1] + c[-1][1]) / 2.0
            yaw = math.atan2(c[-1][1] - c[0][1], c[-1][0] - c[0][0])
            # 라우팅 목적지: 갓길과 경계선 way를 공유하는 "옆 도로 lanelet".
            # (갓길 자체는 라우팅 그래프에 없으므로, 옆 도로까지 가는 경로를 잰다)
            # 집합 교집합(&)이 비어있지 않으면 = 경계선 하나를 공유한다는 뜻.
            target = None
            for rid, rl in self.lanelets.items():
                if rid != lid and rl["subtype"] == "road" and \
                        {rl["left"], rl["right"]} & {ll["left"], ll["right"]}:
                    target = rid
                    break
            if target:
                cands.append(Candidate("shoulder", f"shoulder_{lid}", x, y, yaw,
                                       target, shoulder_sectors))

        # ---- 주차장 후보: parking_lot 폴리곤 ----
        for wid, pts in self.parking_lots.items():
            # 폴리곤의 무게중심(centroid) = 꼭짓점 좌표들의 평균
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            # [왜 무게중심을 그대로 안 쓰나] 주차장 폴리곤이 도로와 "겹쳐"
            # 그려진 지도가 많아서, 무게중심이 차도 위에 떨어질 수 있다
            # (실측: "주차 완료" 상태인데 차선의 51%를 막고 있었음).
            # 대신 "도로 중심선에서 가장 먼 내부 점"을 목표로 쓰되,
            # 폴리곤 밖으로 나가지 않도록 각 꼭짓점을 무게중심 쪽으로
            # 50% 당긴 점들 중에서 고른다.
            def road_dist(p):
                """점 p에서 가장 가까운 도로 중심선까지의 거리."""
                best = float("inf")
                for ll in self.lanelets.values():
                    if ll["subtype"] != "road":
                        continue
                    for q in ll["center"][::2]:
                        d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
                        if d < best:
                            best = d
                return math.sqrt(best)
            # 후보 점들 = 각 꼭짓점과 무게중심의 중점 → 그중 도로에서 가장 먼 점
            gx, gy = max(
                ((0.5 * (px + cx), 0.5 * (py + cy)) for px, py in pts),
                key=road_dist)
            # 라우팅 목적지는 주차장에서 가장 가까운 도로 lanelet
            target = self.nearest_road_lanelet(cx, cy)
            if target is None:
                continue
            # 목표 방향은 그 도로 lanelet의 진행 방향과 맞춘다
            tc = self.lanelets[target]["center"]
            yaw = math.atan2(tc[-1][1] - tc[0][1], tc[-1][0] - tc[0][0])
            cands.append(Candidate("parking_lot", f"parking_{wid}", gx, gy, yaw,
                                   target, parking_sectors))

        return cands
