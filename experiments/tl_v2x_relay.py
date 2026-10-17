#!/usr/bin/env python3
"""[v4.6g] V2X 신호등 릴레이 — lb 컨테이너에서 독립 프로세스로 실행.

aw_e2e(리더보드 agent) 프로세스의 rclpy는 TrafficLightGroupArray 발행이 wire에
안 실리는 심층 문제가 있다(0812 새벽 실측: 블록 실행+publisher 등록+QoS 정합인데
수신 0, 동일 컨테이너 독립 rclpy 자가테스트는 정상). 검증된 조건(독립 프로세스)
으로 분리: CARLA에서 ego 유효 신호 상태를 읽어 맵의 모든 신호등 규제요소에
동일 상태를 발행한다 (aw_converter.get_current_traffic_light_state 로직 미러).

카메라-폴트 케이스(cf/cn/cfl/cfl2/mix)에서는 러너가 이 릴레이를 띄우지 않는다
(신호등 인식이 실험 변인 — 카메라 경로 유지).
"""
import os
import sys
import time
import xml.etree.ElementTree as ET

import carla
import rclpy
from rclpy.node import Node
from autoware_perception_msgs.msg import (TrafficLightGroupArray,
                                          TrafficLightGroup,
                                          TrafficLightElement)
from rosgraph_msgs.msg import Clock

MAP_OSM = ("/home/taewook/autoware_carla_leaderboard/experiments/results/"
           "lanelet2_map_for_snapshot.osm")

CARLA_TO_AW = {
    carla.TrafficLightState.Red: TrafficLightElement.RED,
    carla.TrafficLightState.Yellow: TrafficLightElement.AMBER,
    carla.TrafficLightState.Green: TrafficLightElement.GREEN,
    carla.TrafficLightState.Off: TrafficLightElement.UNKNOWN,
    carla.TrafficLightState.Unknown: TrafficLightElement.UNKNOWN,
}


def tl_ids_from_osm(path):
    ids = []
    for rel in ET.parse(path).getroot().findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if (tags.get("type") == "regulatory_element"
                and tags.get("subtype") == "traffic_light"):
            ids.append(int(rel.get("id")))
    return ids


class TlRelay(Node):
    def __init__(self, ids):
        super().__init__("tl_v2x_relay")
        self.ids = ids
        self.pub_main = self.create_publisher(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/traffic_signals", 10)
        self.pub_ext = self.create_publisher(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/external/traffic_signals", 10)
        self.clock = None
        self.create_subscription(Clock, "/clock", self._clock_cb, 10)
        # aw_converter의 green-hold 로직 미러: CARLA 신호 영역 밖에서 마지막
        # green을 30틱 유지, 그 외 RED 보수 기본
        self.last = TrafficLightElement.RED
        self.hold = 0

    def _clock_cb(self, msg):
        self.clock = msg.clock

    def state_for(self, ego):
        if ego.is_at_traffic_light():
            self.hold = 0
            st = ego.get_traffic_light_state()
            self.last = CARLA_TO_AW.get(st, TrafficLightElement.UNKNOWN)
            return self.last
        if self.last != TrafficLightElement.GREEN:
            self.last = TrafficLightElement.RED
            return TrafficLightElement.RED
        self.hold += 1
        if self.hold > 30:
            self.hold = 0
            self.last = TrafficLightElement.RED
            return TrafficLightElement.RED
        return TrafficLightElement.GREEN

    def publish(self, color):
        msg = TrafficLightGroupArray()
        if self.clock is not None:
            msg.stamp = self.clock
        for tid in self.ids:
            g = TrafficLightGroup()
            g.traffic_light_group_id = tid
            e = TrafficLightElement()
            e.color = color
            e.shape = TrafficLightElement.CIRCLE
            e.status = TrafficLightElement.SOLID_ON
            e.confidence = 1.0
            g.elements.append(e)
            msg.traffic_light_groups.append(g)
        self.pub_main.publish(msg)
        self.pub_ext.publish(msg)


def find_ego(world):
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") in ("hero", "ego_vehicle"):
            return a
    return None


def main():
    ids = tl_ids_from_osm(MAP_OSM)
    print(f"[tl_relay] {len(ids)} traffic-light reg elems", flush=True)
    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    rclpy.init()
    node = TlRelay(ids)

    ego = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        try:
            if ego is None or not ego.is_alive:
                ego = find_ego(world)
                if ego is None:
                    time.sleep(0.5)
                    continue
            color = node.state_for(ego)
        except Exception as e:
            print(f"[tl_relay] carla error: {e} — retrying", flush=True)
            ego = None
            time.sleep(1.0)
            continue
        node.publish(color)
        time.sleep(0.1)


if __name__ == "__main__":
    main()
