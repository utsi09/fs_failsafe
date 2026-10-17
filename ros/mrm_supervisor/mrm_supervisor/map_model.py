"""Lanelet2 .osm map model for MRM candidate selection.

Parses the map directly (local_x/local_y coordinates), builds a lightweight
routing graph over road lanelets, extracts safe-stop candidates from map tags
(road_shoulder lanelets, parking_lot polygons), and answers route queries with
distance + number of traffic lights along the way.
"""
import heapq
import math
import xml.etree.ElementTree as ET


def _tags(elem):
    return {t.get("k"): t.get("v") for t in elem.findall("tag")}


class Candidate:
    def __init__(self, kind, name, x, y, yaw, target_lanelet, required_sectors):
        self.kind = kind                    # "shoulder" | "parking_lot"
        self.name = name
        self.x, self.y, self.yaw = x, y, yaw
        self.target_lanelet = target_lanelet
        self.required_sectors = required_sectors


class MapModel:
    SUCC_EPS = 1.5   # [m] centerline endpoint gap that still counts as connected

    def __init__(self, osm_path):
        root = ET.parse(osm_path).getroot()

        nodes = {}
        for n in root.findall("node"):
            tg = _tags(n)
            if "local_x" in tg and "local_y" in tg:
                nodes[n.get("id")] = (float(tg["local_x"]), float(tg["local_y"]))

        self._ways = {}
        way_tags = {}
        for w in root.findall("way"):
            self._ways[w.get("id")] = [nodes[nd.get("ref")] for nd in w.findall("nd")
                                       if nd.get("ref") in nodes]
            way_tags[w.get("id")] = _tags(w)

        # regulatory elements that are traffic lights (+ their stop lines)
        # [v4.8] 태그만 믿으면 안 된다: Town07 맵은 같은 방향 병행 차선 중
        # 한쪽에만 regelem이 붙어 있어(예: 1229에만 있고 이웃 8328/1222 누락)
        # Dijkstra 경로가 무태그 차선을 타면 n_tl이 과소집계된다 (0813 실측:
        # cn 케이스가 "tl 0"으로 판정한 3003 경로가 실제로는 신호교차로 통과).
        # 해법: 정지선(ref_line) 기하와 중심선 교차로 판정한다. 태그는 보조.
        tl_rel_ids = set()
        tl_stoplines = {}                     # rel_id -> [(x, y), ...]
        for r in root.findall("relation"):
            tg = _tags(r)
            if tg.get("type") == "regulatory_element" and tg.get("subtype") == "traffic_light":
                tl_rel_ids.add(r.get("id"))
                for m in r.findall("member"):
                    if m.get("role") == "ref_line" and m.get("ref") in self._ways:
                        tl_stoplines[r.get("id")] = self._ways[m.get("ref")]
                        break

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
                    has_tl = True
            if not left or not right:
                continue
            lp, rp = self._ways.get(left), self._ways.get(right)
            if not lp or not rp:
                continue
            # 좌/우 경계 way 그리기 방향이 반대면(수제 회랑) i-i 짝이 교차로
            # 묶여 center가 한 점으로 붕괴 → 길이 0, successor 매칭 오염.
            # 우측 점열을 좌측 진행 방향에 정렬한다.
            if math.dist(lp[0], rp[0]) > math.dist(lp[0], rp[-1]):
                rp = list(reversed(rp))
            n = min(len(lp), len(rp))
            center = [((lp[int(i * (len(lp) - 1) / max(1, n - 1))][0] +
                        rp[int(i * (len(rp) - 1) / max(1, n - 1))][0]) / 2.0,
                       (lp[int(i * (len(lp) - 1) / max(1, n - 1))][1] +
                        rp[int(i * (len(rp) - 1) / max(1, n - 1))][1]) / 2.0)
                      for i in range(n)]
            length = sum(math.dist(center[i], center[i + 1]) for i in range(len(center) - 1))
            try:
                speed = max(float(tg.get("speed_limit", 50.0)) / 3.6, 1.0)  # km/h -> m/s
            except ValueError:
                speed = 50.0 / 3.6
            self.lanelets[r.get("id")] = {
                "subtype": tg.get("subtype", "road"),
                "left": left, "right": right,
                "center": center, "length": length, "has_tl": has_tl,
                "speed": speed,
                # 이 lanelet에 태그로 붙은 신호등 regelem id들 (기하 판정은 아래서 합류)
                "tl_ids": {m.get("ref") for m in r.findall("member")
                           if m.get("type") == "relation" and m.get("ref") in tl_rel_ids},
            }

        # [v4.8] 신호 교차로를 "정지선 중심 35m 클러스터"로 묶어 교차로
        # 중심점 목록을 만든다. lanelet 단위 태그 귀속은 병행 차선 누락
        # (예: regelem 9430이 1229에만 붙고 이웃 차선 8328 누락)과 접근로별
        # regelem 중복 계산 때문에 신뢰 불가 — 경로 기하 vs 교차로 중심
        # 거리로 세는 방식(route 참조)으로 전환한다.
        sl_centroid = {rid: (sum(p[0] for p in pts) / len(pts),
                             sum(p[1] for p in pts) / len(pts))
                       for rid, pts in tl_stoplines.items()}
        self.tl_junctions = []             # [(cx, cy)] 신호 교차로 중심들
        for cx, cy in sl_centroid.values():
            if not any(math.hypot(cx - gx, cy - gy) < 35.0
                       for gx, gy in self.tl_junctions):
                self.tl_junctions.append((cx, cy))

        self.parking_lots = {
            wid: pts for wid, pts in self._ways.items()
            if way_tags[wid].get("type") == "parking_lot" and len(pts) >= 3
        }
        # parking_space(칸 중심선 2점 way): 맵에서 칸을 옮기면 goal이 따라가도록
        # 후보 생성 시 lot 폴리곤 안 최근접 칸으로 골을 스냅하는 데 쓴다
        self.parking_spaces = {
            wid: pts for wid, pts in self._ways.items()
            if way_tags[wid].get("type") == "parking_space" and len(pts) >= 2
        }

        self._build_graph()

    # ------------------------------------------------------------------ graph
    def _build_graph(self):
        # 경계 way 그리기 방향이 제각각이라 center 방향 추정이 틀리는 lanelet이
        # 있다(8/11 실측: 8479 등 9개가 successor 0인 '싱크'가 되어 그 위에서
        # fault 시 전 후보 unreachable). 1차 빌드 후 출구 없는 도로 lanelet의
        # center를 뒤집어 재빌드하는 교정 패스를 최대 2회 돈다.
        for _pass in range(3):
            self._build_graph_once()
            sinks = [lid for lid, edges in self.graph.items() if not edges]
            if not sinks or _pass == 2:
                if sinks:
                    print(f"[map_model] warning: {len(sinks)} sink lanelets remain: {sinks[:6]}")
                break
            flipped = 0
            for lid in sinks:
                self.lanelets[lid]["center"] = list(reversed(self.lanelets[lid]["center"]))
                flipped += 1
            if not flipped:
                break

    def _build_graph_once(self):
        roads = {lid: ll for lid, ll in self.lanelets.items() if ll["subtype"] == "road"}
        self.graph = {lid: [] for lid in roads}

        by_bounds = {}
        for lid, ll in roads.items():
            by_bounds.setdefault(ll["left"], []).append(lid)
            by_bounds.setdefault(ll["right"], []).append(lid)

        lids = list(roads)
        for i, a in enumerate(lids):
            ca = roads[a]["center"]
            for b in lids[i + 1:]:
                cb = roads[b]["center"]
                # successor edges (driving direction respected); weight carries
                # both length [m] and travel time [s] at the lanelet speed limit
                if math.dist(ca[-1], cb[0]) < self.SUCC_EPS:
                    self.graph[a].append((b, roads[a]["length"],
                                          roads[a]["length"] / roads[a]["speed"]))
                if math.dist(cb[-1], ca[0]) < self.SUCC_EPS:
                    self.graph[b].append((a, roads[b]["length"],
                                          roads[b]["length"] / roads[b]["speed"]))

        # adjacent (lane-change) edges: lanelets sharing a bound linestring
        for _, lids_shared in by_bounds.items():
            for i, a in enumerate(lids_shared):
                for b in lids_shared[i + 1:]:
                    lc_t = 5.0 / roads[a]["speed"]
                    self.graph[a].append((b, 5.0, lc_t))
                    self.graph[b].append((a, 5.0, lc_t))

    def nearest_road_lanelet(self, x, y):
        best, best_d = None, float("inf")
        for lid, ll in self.lanelets.items():
            if ll["subtype"] != "road":
                continue
            for px, py in ll["center"][::2]:
                d = (px - x) ** 2 + (py - y) ** 2
                if d < best_d:
                    best, best_d = lid, d
        return best

    def nearest_road_lanelets(self, x, y, k=3):
        """최근접 도로 lanelet k개 (거리순). 교차로 내부 lanelet처럼 그래프
        출구가 없는 소스에 스냅됐을 때의 폴백용 (8/11 agg 케이스 실측:
        lanelet 8576에서 전 후보 unreachable -> 즉시 DEFER)."""
        scored = []
        for lid, ll in self.lanelets.items():
            if ll["subtype"] != "road":
                continue
            d = min((px - x) ** 2 + (py - y) ** 2 for px, py in ll["center"][::2])
            scored.append((d, lid))
        return [lid for _, lid in sorted(scored)[:k]]

    def route(self, src, dst):
        """Dijkstra over road lanelets, minimizing travel time.
        -> (length_m, time_s, traffic_light_count) or None."""
        if src not in self.graph or dst not in self.graph:
            return None
        best = {src: (0.0, 0.0)}          # lid -> (time_s, length_m)
        prev = {}
        pq = [(0.0, src)]
        while pq:
            t, u = heapq.heappop(pq)
            if u == dst:
                break
            if t > best.get(u, (float("inf"),))[0]:
                continue
            for v, w_len, w_time in self.graph[u]:
                nt = t + w_time
                if nt < best.get(v, (float("inf"),))[0]:
                    best[v] = (nt, best[u][1] + w_len)
                    prev[v] = u
                    heapq.heappush(pq, (nt, v))
        if dst not in best:
            return None
        path = [dst]
        while path[-1] != src:
            path.append(prev[path[-1]])
        # [v4.8] 태그 기반 lanelet 합산은 병행 차선 태그 누락(과소)과 접근로별
        # regelem 중복(과대)을 모두 앓는다 — 경로 중심선이 신호 교차로 중심
        # 25m 이내를 지나는 교차로 개수로 센다 (기하 기반, 주석 무관)
        n_tl = 0
        for gx, gy in self.tl_junctions:
            for lid in path:
                if any(math.hypot(px - gx, py - gy) < 25.0
                       for px, py in self.lanelets[lid]["center"]):
                    n_tl += 1
                    break
        return best[dst][1], best[dst][0], n_tl

    # ------------------------------------------------------------- candidates
    def extract_candidates(self, shoulder_sectors, parking_sectors):
        cands = []

        for lid, ll in self.lanelets.items():
            if ll["subtype"] != "road_shoulder":
                continue
            if str(lid) == "1271":
                # 헛간 회랑 서향 차선 — 라우팅 고립 문제로 road_shoulder로
                # 리태그된 도로임(8/10). 역방향 차선 위 정차는 유효한 노측
                # 대피가 아니므로 후보에서 제외 (유령 타깃 방지)
                continue
            c = ll["center"]
            # feasibility: a moving lateral-shift pull-over cannot fit into a
            # pocket shorter than ~car + shift run-in (observed: 6.3 m pocket
            # -> goal_planner "generated 0 pull over path candidates" forever,
            # 7.8 m pocket -> parks fine)
            pocket_len = sum(math.dist(c[i], c[i + 1]) for i in range(len(c) - 1))
            if pocket_len < 7.0:
                continue
            # position: true middle of the lanelet; yaw: overall start->end
            # direction (a mid/mid+1 segment degenerates on 2-point centerlines)
            if len(c) > 2:
                x, y = c[len(c) // 2]
            else:
                x = (c[0][0] + c[-1][0]) / 2.0
                y = (c[0][1] + c[-1][1]) / 2.0
            yaw = math.atan2(c[-1][1] - c[0][1], c[-1][0] - c[0][0])
            # adjacent road lanelet shares one of the bound linestrings
            target = None
            for rid, rl in self.lanelets.items():
                if rid != lid and rl["subtype"] == "road" and \
                        {rl["left"], rl["right"]} & {ll["left"], ll["right"]}:
                    target = rid
                    break
            if target:
                cands.append(Candidate("shoulder", f"shoulder_{lid}", x, y, yaw,
                                       target, shoulder_sectors))

        for wid, pts in self.parking_lots.items():
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            # lot polygons often OVERLAP the road, so the centroid can land on
            # the carriageway (observed: "parked" blocking 51% of the lane).
            # Use the interior point farthest from any road centerline instead,
            # blended toward the centroid to stay inside the polygon.
            def road_dist(p):
                best = float("inf")
                for ll in self.lanelets.values():
                    if ll["subtype"] != "road":
                        continue
                    for q in ll["center"][::2]:
                        d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
                        if d < best:
                            best = d
                return math.sqrt(best)
            gx, gy = max(
                ((0.5 * (px + cx), 0.5 * (py + cy)) for px, py in pts),
                key=road_dist)
            # 목표 차선은 중심점이 아니라 "도로에 가장 가까운 경계점" 기준으로
            # 배정 — 중심점 기준은 도로 반대편(역방향) 차선을 고를 수 있어
            # 합법 경로가 수백 m 우회로 계산된다 (900000 실측: 346m/신호등 2)
            n_pts = len(pts)
            edge_mids = [((pts[i][0] + pts[(i + 1) % n_pts][0]) / 2.0,
                          (pts[i][1] + pts[(i + 1) % n_pts][1]) / 2.0)
                         for i in range(n_pts)]
            anchor = min(list(pts) + edge_mids, key=road_dist)
            target = self.nearest_road_lanelet(*anchor)
            if target is None:
                continue
            # 목표 자세: 도로에 정확히 수직(진입 방향) + 폴리곤 가로 중앙 정렬.
            # 어긋난 yaw는 footprint 대각선이 소형 패드 경계를 넘겨
            # freespace goal invalid를 유발한다 (실측: 113도에서 0.23m 초과)
            tc = self.lanelets[target]["center"]
            road_yaw = math.atan2(tc[-1][1] - tc[0][1], tc[-1][0] - tc[0][0])
            perp = road_yaw + math.pi / 2.0
            dx, dy = math.cos(perp), math.sin(perp)
            if dx * (gx - cx) + dy * (gy - cy) < 0:   # 깊은 쪽(도로 반대)으로
                perp += math.pi
                dx, dy = -dx, -dy
            depth = max(dx * (px - cx) + dy * (py - cy) for px, py in pts)
            # 위치: 깊이 30% 지점(접근 단계 goal_planner가 폴리곤 안 정지하는
            # 실측 배치). yaw: 도로 쪽(nose-out) — base_link 기준 전방 돌출이
            # 남쪽(도로)으로 향해 벽 여유가 양방향 2.5m+ (오프라인 A* 검증: 9.7m 경로)
            # 최종 자세: 순수 수직 nose-in(perp) + 얕은 깊이(0.15).
            # 대각 45°는 코너가 3.5m 뻗어 폭 5m 패드에 해가 공집합(격자 실측),
            # 수직은 가로폭 ±1.07m로 여유 확보. 결속은 89.45°<90° 통과
            # (케이스 11~14 직진 접근 실증). 깊이 0.15는 북쪽 벽(코 전방 3.9m)
            # 회피 — 코 y≈11.5 vs 벽 12.1 (여유 0.6m+)
            gx2, gy2 = cx + dx * depth * 0.3, cy + dy * depth * 0.3
            yaw_goal = perp + math.pi   # [900000 촬영용: 성공 당시(nose-out) 레시피]
            if str(wid) == "900000":
                # [남측 영상 데모: 0809_0037 PARKED_SUCCESS 실측 주차 포즈를
                #  골로 직접 지정 — freespace가 이미 한 번 도달한 검증된 포즈]
                gx2, gy2, yaw_goal = -40.0, -5.9, -2.86
            elif str(wid) == "1002367":
                # [우측 실주차장 데모: 헛간 앞 포장 앞마당 중앙, 북향 nose-in]
                gx2, gy2, yaw_goal = -17.0, 14.0, -1.5708  # 빈 슬롯, 이웃차 이격 2.0m
            elif str(wid) == "1003001":
                # [북쪽 헛간 14칸 라인: 레벨에 구워진 정적 주차차 3대(BMW x-70,
                #  Jeep x-45.8, Leon x-32.3, env_objects 실측)와 겹치는 칸은 A*
                #  "Invalid goal". 확실히 빈 중동부 칸(spot 1002542, x=-51.8),
                #  북향 nose-in. y는 68.3: 앞범퍼(base_link+3.83m)가 폴리곤
                #  북변(73.0) 안에 들어야 함 — 69.2는 앞코 73.03으로 3cm 초과
                #  -> wayarea 밖 셀 충돌로 A* Invalid goal (8/10 실측)]
                gx2, gy2, yaw_goal = -51.75, 68.3, 1.5708
            elif str(wid) == "1003003":
                # [북서 세로 패드: spot 중심(109.1)은 북쪽 덤불 군락(y 111.5+,
                #  env_objects 실측)에 footprint가 물려 A* Invalid — 남동쪽으로
                #  당겨 여유 확보. 남향 nose, 폴리곤(남변 104 확장) 안 수용]
                gx2, gy2, yaw_goal = -140.9, 108.3, -1.5708
            elif str(wid) == "1003002":
                # [북서 로터리: 섬 중앙 지향 — 중앙 조형물(반경~0.8m)만 피해
                #  2.2m 동쪽, 북향. 연석은 costmap min_lidar_height 0.3으로 무시.
                #  폴리곤은 섬+동측 진입 링까지 커버(in_lot 전환용)]
                gx2, gy2, yaw_goal = -147.5, 91.9, 1.5708
            gx2, gy2, yaw_goal = self._space_goal(pts, gx2, gy2, yaw_goal)
            cands.append(Candidate("parking_lot", f"parking_{wid}", gx2, gy2,
                                   yaw_goal, target, parking_sectors))

        return cands

    @staticmethod
    def _point_in_poly(x, y, pts):
        inside = False
        j = len(pts) - 1
        for i in range(len(pts)):
            xi, yi = pts[i]
            xj, yj = pts[j]
            if (yi > y) != (yj > y) and \
                    x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                inside = not inside
            j = i
        return inside

    def _space_goal(self, poly, gx, gy, yaw):
        """lot 폴리곤 안의 최근접 parking_space 칸 중심선으로 goal 스냅.

        맵에서 칸을 옮기면 goal이 자동으로 따라간다. yaw는 칸 축의 두 방향 중
        기존(검증된 nose 관례) yaw에 가까운 쪽. 위치는 footprint(base_link
        전방 3.83/후방 1.05 m)가 폴리곤의 축 투영 구간 안에 들도록 클램프
        (여유 0.3 m). 적합 칸이 없거나 클램프 구간이 공집합이면 기존 goal 유지.
        """
        best, best_d = None, float("inf")
        for spts in self.parking_spaces.values():
            mx = sum(p[0] for p in spts) / len(spts)
            my = sum(p[1] for p in spts) / len(spts)
            if not self._point_in_poly(mx, my, poly):
                continue
            d = math.hypot(mx - gx, my - gy)
            if d < best_d:
                best, best_d = (spts, mx, my), d
        if best is None:
            return gx, gy, yaw
        spts, mx, my = best
        ax = math.atan2(spts[-1][1] - spts[0][1], spts[-1][0] - spts[0][0])

        def angdiff(a, b):
            return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)

        yaw2 = ax if angdiff(ax, yaw) <= angdiff(ax + math.pi, yaw) else ax + math.pi
        hx, hy = math.cos(yaw2), math.sin(yaw2)
        # 가용 깊이는 goal 축이 폴리곤 경계와 만나는 실제 교차 구간(코드)으로
        # 잰다 — 꼭짓점 투영은 비직사각 폴리곤에서 깊이를 과대평가해 앞코가
        # 경계를 수 cm 넘는 골을 만든다 (1003001 y=69.2 리젝트 실측)
        t_neg, t_pos = -float("inf"), float("inf")
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            ex, ey = x2 - x1, y2 - y1
            det = ex * hy - ey * hx
            if abs(det) < 1e-9:
                continue
            dx, dy = x1 - mx, y1 - my
            t = (ex * dy - ey * dx) / det
            s = (hx * dy - hy * dx) / det
            if 0.0 <= s <= 1.0:
                if t <= 0.0:
                    t_neg = max(t_neg, t)
                else:
                    t_pos = min(t_pos, t)
        if not math.isfinite(t_neg) or not math.isfinite(t_pos):
            return gx, gy, yaw
        fmin = t_neg + 1.05 + 0.35
        fmax = t_pos - 3.83 - 0.35
        if fmin > fmax:
            # 폴리곤 깊이가 근소하게(<0.5 m) 부족하면 구간 중점으로
            # best-effort 스냅 — 기존 골로의 폴백은 폴리곤이 옮겨진 뒤엔
            # 오히려 완전히 밖일 수 있다 (1003003 실측). 크게 부족하면 유지.
            if fmin - fmax < 0.5:
                off = (fmin + fmax) / 2.0
                return mx + hx * off, my + hy * off, yaw2
            return gx, gy, yaw
        off = min(max(0.0, fmin), fmax)
        return mx + hx * off, my + hy * off, yaw2
