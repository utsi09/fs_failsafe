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

        # regulatory elements that are traffic lights
        tl_rel_ids = set()
        for r in root.findall("relation"):
            tg = _tags(r)
            if tg.get("type") == "regulatory_element" and tg.get("subtype") == "traffic_light":
                tl_rel_ids.add(r.get("id"))

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
            }

        self.parking_lots = {
            wid: pts for wid, pts in self._ways.items()
            if way_tags[wid].get("type") == "parking_lot" and len(pts) >= 3
        }

        self._build_graph()

    # ------------------------------------------------------------------ graph
    def _build_graph(self):
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
        n_tl = sum(1 for lid in path if self.lanelets[lid]["has_tl"])
        return best[dst][1], best[dst][0], n_tl

    # ------------------------------------------------------------- candidates
    def extract_candidates(self, shoulder_sectors, parking_sectors):
        cands = []

        for lid, ll in self.lanelets.items():
            if ll["subtype"] != "road_shoulder":
                continue
            c = ll["center"]
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
            target = self.nearest_road_lanelet(cx, cy)
            if target is None:
                continue
            tc = self.lanelets[target]["center"]
            yaw = math.atan2(tc[-1][1] - tc[0][1], tc[-1][0] - tc[0][0])
            cands.append(Candidate("parking_lot", f"parking_{wid}", cx, cy, yaw,
                                   target, parking_sectors))

        return cands
