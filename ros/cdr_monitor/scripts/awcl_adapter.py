#!/usr/bin/env python3
"""Adapter from Autoware (awcl) topics to the cdr_monitor input topics.

/localization/kinematic_state (nav_msgs/Odometry)
    -> /vel     (geometry_msgs/TwistStamped)
    -> /heading (geometry_msgs/QuaternionStamped)
/perception/object_recognition/objects (autoware_perception_msgs/PredictedObjects)
    -> /hesai_zed/obstacles (hesai_zed_fusion/ObstacleArray, nearest distance)
    -> /monitor/ttc_ms (std_msgs/Float64) spatio-temporal TTC: first time the
       planned ego trajectory and a predicted object path come within a safety
       radius. -1 when no encounter is predicted within the horizon.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Trajectory
from hesai_zed_fusion.msg import ObstacleArray, ObstacleInfo

TTC_HORIZON_S = 6.0    # how far into the future we look
TTC_STEP_S = 0.25      # time grid resolution
TTC_RADIUS_M = 2.5     # ego-object distance treated as an encounter


class AwclAdapter(Node):
    def __init__(self):
        super().__init__("awcl_adapter")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)

        self._vel_pub = self.create_publisher(TwistStamped, "/vel", qos)
        self._heading_pub = self.create_publisher(QuaternionStamped, "/heading", qos)
        self._obstacle_pub = self.create_publisher(ObstacleArray, "/hesai_zed/obstacles", qos)

        self._ttc_pub = self.create_publisher(Float64, "/monitor/ttc_ms", qos)

        self._ego_xy = None
        self._traj = None
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._odom_cb, qos)
        self.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                                 self._objects_cb, qos)
        self.create_subscription(Trajectory, "/planning/scenario_planning/trajectory",
                                 self._traj_cb, qos)

    def _traj_cb(self, msg):
        self._traj = msg

    def _odom_cb(self, msg):
        self._ego_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        vel = TwistStamped()
        vel.header = msg.header
        vel.twist = msg.twist.twist
        self._vel_pub.publish(vel)

        heading = QuaternionStamped()
        heading.header = msg.header
        heading.quaternion = msg.pose.pose.orientation
        self._heading_pub.publish(heading)

    def _objects_cb(self, msg):
        if self._ego_xy is None:
            return
        ex, ey = self._ego_xy
        min_dist = float("inf")
        for obj in msg.objects:
            p = obj.kinematics.initial_pose_with_covariance.pose.position
            min_dist = min(min_dist, math.hypot(p.x - ex, p.y - ey))

        out = ObstacleArray()
        out.header = msg.header
        ob = ObstacleInfo()
        ob.label = "nearest"
        # no detected objects -> report a far-but-sane distance so the FTTI
        # stays at its base value instead of jumping to DBL_MAX downstream
        ob.distance = min_dist if math.isfinite(min_dist) else 1000.0
        out.obstacles.append(ob)
        self._obstacle_pub.publish(out)

        self._ttc_pub.publish(Float64(data=self._spacetime_ttc_ms(msg)))

    # ------------------------------------------------- spatio-temporal TTC
    def _ego_samples(self):
        """[(t, x, y)] along the planned trajectory; time integrated as
        segment_length / commanded_velocity."""
        if self._traj is None or len(self._traj.points) < 2:
            return None
        out, t = [], 0.0
        prev = self._traj.points[0].pose.position
        out.append((0.0, prev.x, prev.y))
        for p in self._traj.points[1:]:
            pos = p.pose.position
            ds = math.hypot(pos.x - prev.x, pos.y - prev.y)
            v = max(abs(p.longitudinal_velocity_mps), 0.5)
            t += ds / v
            out.append((t, pos.x, pos.y))
            prev = pos
            if t > TTC_HORIZON_S:
                break
        return out

    @staticmethod
    def _interp(samples, t):
        """Linear interpolation over [(t, x, y)]; None past the end."""
        if t > samples[-1][0]:
            return None
        for i in range(1, len(samples)):
            t1, x1, y1 = samples[i]
            if t <= t1:
                t0, x0, y0 = samples[i - 1]
                a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return (x0 + a * (x1 - x0), y0 + a * (y1 - y0))
        return None

    def _spacetime_ttc_ms(self, msg):
        """First time [ms] the ego trajectory and any predicted object path
        are within TTC_RADIUS_M of each other. -1 if none within horizon."""
        ego = self._ego_samples()
        if ego is None:
            return -1.0
        best = -1.0
        for obj in msg.objects:
            paths = obj.kinematics.predicted_paths
            if not paths:
                continue
            path = max(paths, key=lambda p: p.confidence)
            step = path.time_step.sec + path.time_step.nanosec * 1e-9
            if step <= 0.0 or not path.path:
                continue
            t = TTC_STEP_S
            while t <= TTC_HORIZON_S:
                e = self._interp(ego, t)
                if e is None:
                    break
                idx = min(int(t / step), len(path.path) - 1)
                o = path.path[idx].position
                if math.hypot(o.x - e[0], o.y - e[1]) < TTC_RADIUS_M:
                    best = t if best < 0 else min(best, t)
                    break
                t += TTC_STEP_S
        return best * 1000.0 if best > 0 else -1.0


def main():
    rclpy.init()
    node = AwclAdapter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
