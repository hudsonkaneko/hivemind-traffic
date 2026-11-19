"""Tiny helper for agents: load the lane graph, plan routes, get waypoints.

Works from lanegraph.json (no Isaac needed) so it can live in a training loop.

    from lanegraph_api import LaneGraph
    g = LaneGraph("la_10_110/lanegraph.json")
    route = g.route(g.entry_lanes()[0], target_road="I 110")   # list of lane ids
    pts = g.waypoints(route, spacing=2.0)                      # [(x, y, z), ...]
"""
import heapq
import json
import math


class LaneGraph:
    def __init__(self, path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        self.origin = data["origin"]
        self.lanes = {L["id"]: L for L in data["lanes"]}
        for L in self.lanes.values():
            L["length"] = sum(math.dist(a, b) for a, b in zip(L["pts"], L["pts"][1:]))

    def entry_lanes(self):
        return sorted(k for k, L in self.lanes.items() if not L["pred"])

    def exit_lanes(self):
        return sorted(k for k, L in self.lanes.items() if not L["succ"])

    def nearest_lane(self, x, y, z=None):
        best = (1e18, None, 0)
        for k, L in self.lanes.items():
            for i, p in enumerate(L["pts"]):
                d = (p[0] - x) ** 2 + (p[1] - y) ** 2 + (0 if z is None else 4 * (p[2] - z) ** 2)
                if d < best[0]:
                    best = (d, k, i)
        return best[1], best[2]

    LANE_CHANGE_COST = 60.0   # metres-equivalent penalty per lane change

    def route(self, start, goal=None, target_road=None, target_destination=None):
        """Shortest lane sequence to `goal` lane, any lane on `target_road`, or any lane
        whose signed destination contains `target_destination` (e.g. "Pasadena").
        Uses successors (drive on) and left/right neighbours (lane change)."""
        dist = {start: 0.0}
        prev = {}
        heap = [(0.0, start)]
        while heap:
            d, k = heapq.heappop(heap)
            if d > dist[k]:
                continue
            L = self.lanes[k]
            hit = (k == goal or (target_road and k != start and target_road in L["road"])
                   or (target_destination and k != start and target_destination in L.get("destination", "")))
            if hit:
                path = [k]
                while path[-1] in prev:
                    path.append(prev[path[-1]])
                return path[::-1]
            moves = [(s, L["length"]) for s in L["succ"]]
            moves += [(n, self.LANE_CHANGE_COST) for n in (L.get("left"), L.get("right")) if n]
            for s, c in moves:
                nd = d + c
                if nd < dist.get(s, 1e18):
                    dist[s] = nd
                    prev[s] = k
                    heapq.heappush(heap, (nd, s))
        return None

    def loop_lanes(self, corner=None):
        """Lane ids on the simulation loops (optionally one corner: NW/NE/SE/SW)."""
        return sorted(k for k, L in self.lanes.items() if L.get("simLoop") and (corner is None or L["simLoop"] == corner))

    def signs_for_lane(self, lane_id, signs):
        """Overhead sign panels that apply to a lane (from lanegraph.json['signs'])."""
        L = self.lanes[lane_id]
        return [s for s in signs if s["roadway"] == L["way"] and L["index"] in s["lanes"]]

    def waypoints(self, route, spacing=2.0):
        pts = []
        for k in route:
            for p in self.lanes[k]["pts"]:
                if not pts or math.dist(pts[-1], p) >= spacing:
                    pts.append(tuple(p))
        return pts

    def speed_limit(self, lane_id):
        return self.lanes[lane_id]["speed"]


if __name__ == "__main__":
    import os
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "lanegraph.json")
    g = LaneGraph(path)
    e = g.entry_lanes()
    print(len(g.lanes), "lanes,", len(e), "entries,", len(g.exit_lanes()), "exits,", len(g.loop_lanes()), "sim-loop lanes")
    # a full lap: SW loop -> NB 110 -> onto the I-10 -> ... -> back onto the same SW loop lane
    start = g.loop_lanes("SW")[0]
    r = g.route(start, target_road="I 10")
    back = g.route(r[-1], goal=start)
    lap = r + back[1:]
    wp = g.waypoints(lap)
    length = sum(math.dist(a, b) for a, b in zip(wp, wp[1:]))
    roads = []
    for k in lap:
        nm = g.lanes[k]["road"]
        if not roads or roads[-1] != nm:
            roads.append(nm)
    print(f"closed lap from {start}: {len(lap)} lanes, {length / 1000:.1f} km, z {min(p[2] for p in wp):.1f}..{max(p[2] for p in wp):.1f} m")
    print("  via: " + " -> ".join(roads))
