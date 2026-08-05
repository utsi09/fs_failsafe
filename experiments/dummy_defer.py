#!/usr/bin/env python3
"""--mrm-off 전용: supervisor 없이 진단 그래프의 defer_request 유닛만 OK로
채워주는 더미 노드. (v2 배터리 1차에서 supervisor 부재 -> 유닛 STALE ->
autonomous 잠금 -> 전 케이스 출발 불가였던 문제의 해결책)"""
import time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

rclpy.init()
n = Node("mrm_supervisor")          # 그래프가 기대하는 노드명 그대로
pub = n.create_publisher(DiagnosticArray, "/diagnostics", 1)
while rclpy.ok():
    m = DiagnosticArray()
    m.header.stamp = n.get_clock().now().to_msg()
    s = DiagnosticStatus(level=DiagnosticStatus.OK,
                         name="mrm_supervisor: defer_request",
                         message="mrm-off dummy", hardware_id="")
    m.status = [s]
    pub.publish(m)
    time.sleep(0.1)
