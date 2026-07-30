"""Score-based MRM supervisor.

Consumes /monitor/scores (cdr_monitor) and adds its own localization-jump
check, then runs the state machine:

  NORMAL      perception score healthy
  MRM_MOVE    degraded band (0.3 < s <= 0.7): pick the cheapest safe-stop
              candidate (shoulder / parking lot) by cost function and reroute
              Autoware to it via the AD API
  MRM_PARKED  arrived and stopped at the chosen spot
  MRM_DEFER   score below the hard floor (or localization too bad to drive):
              do nothing — the stock Autoware mrm_handler performs
              comfortable/emergency stop

eta(c)  = route_time(speed limits) + n_traffic_lights * tl_wait_s
          -- exposure time in the degraded state (risk accumulates per second,
             not per meter: ISO 26262 time-to-safe-state framing)

cost(c) = w_time * eta
        + w_tl * n_traffic_lights * g(cap_traffic_light)      [inf if camera dead]
        + w_sec * sum over required sectors of (1 - sector_score)
        + w_loc * eta * (1 - localization_score)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import SetRoutePoints

from mrm_supervisor.map_model import MapModel


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class Supervisor(Node):
    def __init__(self):
        super().__init__("mrm_supervisor")

        self.declare_parameter("map_path", "/home/taewook/awcl_maps/Town07/lanelet2_map.osm")
        # baseline mode: identical trigger condition, but instead of the
        # shoulder/parking reroute hand the vehicle straight to the stock
        # Autoware mrm_handler (comfortable/emergency stop in lane). Used as
        # the comparison arm in the thesis experiments.
        self.declare_parameter("baseline", False)
        self.declare_parameter("upper_threshold", 0.7)
        self.declare_parameter("lower_threshold", 0.3)
        self.declare_parameter("min_loc_to_move", 0.5)
        self.declare_parameter("hold_ticks", 10)          # 1 s at 10 Hz
        self.declare_parameter("w_time", 7.0)     # cost per second of degraded driving
        self.declare_parameter("tl_wait_s", 15.0) # expected wait per traffic light
        self.declare_parameter("w_tl", 200.0)
        self.declare_parameter("w_sector", 500.0)
        self.declare_parameter("w_loc", 14.0)     # per second, scaled by (1 - loc)
        self.declare_parameter("arrive_dist", 4.0)
        # a pull-over needs approach distance (decel + lateral shift);
        # candidates closer than this are infeasible, not "cheap"
        self.declare_parameter("min_approach_dist", 25.0)
        self.declare_parameter("shoulder_sectors", ["F", "FR"])
        self.declare_parameter("parking_sectors", ["F", "FL", "FR", "B", "RL", "RR"])
        # traffic-light observability: cameras (mount yaw / FOV, deg) that could
        # cover a signal ahead. A TL route is allowed if ANY of them is healthy.
        self.declare_parameter("tl_camera_names", ["CAMERA_front"])
        self.declare_parameter("tl_camera_yaws_deg", [0.0])
        self.declare_parameter("tl_camera_fovs_deg", [90.0])
        self.declare_parameter("tl_visible_min_score", 0.5)
        self.declare_parameter("tl_bearing_deg", 30.0)  # signals sit within +-30 deg ahead

        map_path = self.get_parameter("map_path").value
        self.map = MapModel(map_path)
        self.candidates = self.map.extract_candidates(
            list(self.get_parameter("shoulder_sectors").value),
            list(self.get_parameter("parking_sectors").value))
        self.get_logger().info(
            f"map loaded: {len(self.map.lanelets)} lanelets, "
            f"{len(self.candidates)} MRM candidates "
            f"({sum(1 for c in self.candidates if c.kind == 'shoulder')} shoulder, "
            f"{sum(1 for c in self.candidates if c.kind == 'parking_lot')} parking)")

        self.scores = {}           # name -> score from /monitor/scores
        self.pose = None           # (x, y)
        self.speed = 0.0
        self.loc_jump_score = 1.0
        self.loc_meas_score = 1.0  # raw GNSS/NDT pose vs EKF output residual
        self._meas_base = 0.0      # static frame offset learned at boot
        self._meas_n = 0
        self._last_odom = None

        self.state = "NORMAL"
        # armed only after real scores show a healthy system once — startup
        # transients (leaderboard still spawning sensors) must not trigger MRM
        self._armed = False
        self._armed_ticks = 0
        self._band_ticks = 0
        self._floor_ticks = 0   # consecutive ticks at/below the hard floor
        self._chosen = None
        self._goal_sent = False
        self._goal_sent_time = None
        self._goal_accepted = False
        self._noresp = 0
        self._select_failures = 0
        self._blacklist = set()      # candidates goal_planner failed to park at
        self._stuck_since = None

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(DiagnosticArray, "/monitor/scores", self._scores_cb, qos)
        self.create_subscription(Odometry, "/localization/kinematic_state", self._odom_cb, qos)
        # Autoware's own arrival signal: goal_planner may refine the pull-over
        # pose tens of meters away from our candidate midpoint, so a pure
        # euclidean arrive_dist check can never fire (observed 19 m offset)
        self.route_state = RouteState.UNKNOWN
        self.create_subscription(
            RouteState, "/api/routing/state", self._route_state_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # GNSS/NDT measurement quality: the EKF smooths noisy pose inputs so
        # its OUTPUT looks clean (loc_jump misses gn faults entirely) — the
        # input-vs-output residual is where measurement noise is visible
        self.create_subscription(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance",
            self._meas_pose_cb, qos)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/mrm_supervisor/state", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/mrm_supervisor/candidates", qos)
        self.route_cli = self.create_client(SetRoutePoints, "/api/routing/change_route_points")
        # DEFER hand-over to the stock Autoware MRM: the diagnostic graph links
        # "mrm_supervisor: defer_request" into /autoware/modes/autonomous, so
        # ERROR here makes autonomous unavailable -> mrm_handler performs
        # comfortable/emergency stop. Covers DEFER causes that the perception
        # score alone does not reach (TL gate, localization too low to move).
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_timer(0.1, self._tick)
        self._publish_state()

    # ------------------------------------------------------------- callbacks
    def _scores_cb(self, msg):
        for st in msg.status:
            for kv in st.values:
                if kv.key == "score":
                    try:
                        self.scores[st.name] = float(kv.value)
                    except ValueError:
                        pass

    def _route_state_cb(self, msg):
        self.route_state = msg.state

    def _meas_pose_cb(self, msg):
        if self.pose is None:
            return
        # residual between the raw localization measurement and the EKF pose.
        # There is a STATIC offset between the two (vehicle-center vs base_link,
        # ~1.4 m) — learn it during boot (faults only fire after driving
        # starts) and score only the EXCESS over that baseline.
        r = math.dist(self.pose, (msg.pose.pose.position.x, msg.pose.pose.position.y))
        if self._meas_n < 60:                     # ~3 s calibration window
            self._meas_base = r if self._meas_n == 0 else \
                0.9 * self._meas_base + 0.1 * r
            self._meas_n += 1
            return
        excess = max(0.0, r - self._meas_base - 0.2)
        # gn mid (sigma 1 m, excess ~0.3) -> ~0.58: degraded band, still above
        # min_loc_to_move -> shoulder pull-over; gn high (sigma 2.5, excess
        # ~1.6) -> ~0.05: floor -> DEFER
        inst = math.exp(-excess / 0.55)
        self.loc_meas_score = 0.9 * self.loc_meas_score + 0.1 * inst

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy)
        now = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        if self._last_odom is not None:
            px, py, pt, pv = self._last_odom
            dt = now - pt
            if 0.0 < dt < 1.0:
                # innovation: actual displacement vs what the twist predicts
                jump = abs(math.dist((x, y), (px, py)) - pv * dt)
                inst = math.exp(-max(0.0, jump - 0.3) / 0.5)
                self.loc_jump_score = 0.9 * self.loc_jump_score + 0.1 * inst
        self._last_odom = (x, y, now, self.speed)
        self.pose = (x, y)

    def _publish_defer_diag(self):
        st = DiagnosticStatus()
        st.name = "mrm_supervisor: defer_request"
        st.hardware_id = "mrm_supervisor"
        st.level = (DiagnosticStatus.ERROR if self.state == "MRM_DEFER"
                    else DiagnosticStatus.OK)
        st.message = self.state
        st.values.append(KeyValue(key="state", value=self.state))
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self.diag_pub.publish(arr)

    # ------------------------------------------------------------------ tick
    def _tick(self):
        self._publish_defer_diag()
        perception = self.scores.get("capability/perception", 1.0)
        cap_loc = min(self.scores.get("capability/localization", 1.0),
                      self.loc_jump_score, self.loc_meas_score)

        upper = self.get_parameter("upper_threshold").value
        lower = self.get_parameter("lower_threshold").value

        # trigger on the WORST capability: perception faults (lidar/camera)
        # and localization faults (IMU/GNSS -> EKF wobble, caught by the
        # odometry-innovation loc_jump score) must both be able to fire MRM
        health = min(perception, cap_loc)

        if not self._armed:
            # require sustained health (3 s), not one blip: a transient 0.9
            # during boot armed the supervisor right into the boot dip, firing
            # a false MRM seconds after engagement
            if "capability/perception" in self.scores and perception >= upper:
                self._armed_ticks += 1
                if self._armed_ticks >= 30:
                    self._armed = True
                    self.get_logger().info(f"armed: perception {perception:.2f} healthy for 3 s")
            else:
                self._armed_ticks = 0
            return

        # MRM_DEFER is terminal, so a single bad tick must not trigger it
        self._floor_ticks = self._floor_ticks + 1 if health <= lower else 0

        if self.state == "NORMAL":
            if self._floor_ticks >= 5:
                self._transition("MRM_DEFER", f"health {health:.2f} below floor "
                                 f"(perception {perception:.2f}, loc {cap_loc:.2f})")
            elif health < upper:
                self._band_ticks += 1
                if self._band_ticks == 1 or self._band_ticks % 10 == 0:
                    self.get_logger().info(
                        f"degraded band: health {health:.2f} (p {perception:.2f}/l {cap_loc:.2f}) "
                        f"({self._band_ticks}/{self.get_parameter('hold_ticks').value} ticks)")
                if self._band_ticks >= self.get_parameter("hold_ticks").value:
                    if self.get_parameter("baseline").value:
                        self._transition("MRM_DEFER",
                                         f"baseline: health {health:.2f}, stock MRM")
                    elif cap_loc < self.get_parameter("min_loc_to_move").value:
                        self._transition("MRM_DEFER",
                                         f"degraded but localization {cap_loc:.2f} too low to move")
                    else:
                        self._enter_mrm_move(cap_loc)
            else:
                # decay instead of reset so a borderline score oscillating
                # around the threshold still accumulates
                self._band_ticks = max(0, self._band_ticks - 1)

        elif self.state == "MRM_MOVE":
            if self._floor_ticks >= 5:
                self._transition("MRM_DEFER", f"health {health:.2f} collapsed en route")
                return
            # route-change watchdog: near its own goal Autoware's goal_planner
            # takes over and the change service can go unanswered — the car
            # then silently finishes the OLD route. Retry, then hand over to
            # the stock MRM instead of driving on as if nothing happened.
            if (self._goal_sent and not self._goal_accepted
                    and self._goal_sent_time is not None
                    and (self.get_clock().now() - self._goal_sent_time).nanoseconds > 5e9):
                self._noresp += 1
                self.get_logger().error(
                    f"route change unanswered for 5 s (try {self._noresp}/3) — retrying")
                self._goal_sent = False
                if self._noresp >= 3:
                    self._transition("MRM_DEFER", "route change unanswered — stock MRM")
                    return
            if not self._goal_sent:
                self._send_goal()
            elif self._chosen and self.pose:
                d = math.dist(self.pose, (self._chosen.x, self._chosen.y))
                # decompose into along-shoulder / across-shoulder: a plain
                # radius check declared "parked" while stopped IN THE LANE
                # 3.5 m abeam of the shoulder marker
                dx = self.pose[0] - self._chosen.x
                dy = self.pose[1] - self._chosen.y
                cy, sy = math.cos(self._chosen.yaw), math.sin(self._chosen.yaw)
                lon = dx * cy + dy * sy
                lat = -dx * sy + dy * cy
                on_shoulder = (abs(lon) < self.get_parameter("arrive_dist").value
                               and abs(lat) < 1.5)
                arrived = on_shoulder or self.route_state == RouteState.ARRIVED
                if arrived and self.speed < 0.2:
                    self._transition("MRM_PARKED",
                                     f"stopped at {self._chosen.name} "
                                     f"(d={d:.1f}m, lat={lat:.1f}m, "
                                     f"route_state={self.route_state})")
                    return
                # goal_planner can end up with "no path candidate" (too-narrow
                # or overshot shoulder): ego stalls without ever reaching the
                # shoulder. Blacklist that candidate and fall back to the next.
                # 45 s > a full Town07 red phase, so light waits don't trigger.
                if self._goal_accepted and self.speed < 0.2:
                    now = self.get_clock().now()
                    if self._stuck_since is None:
                        self._stuck_since = now
                    elif (now - self._stuck_since).nanoseconds > 45e9:
                        self._blacklist.add(self._chosen.name)
                        self.get_logger().error(
                            f"stalled 45 s off-shoulder near {self._chosen.name} "
                            f"(d={d:.0f}m, lat={lat:.1f}m) — trying next candidate")
                        self._stuck_since = None
                        self._goal_sent = False
                        self._goal_accepted = False
                        best = self._select_candidate(cap_loc)
                        if best is None:
                            self._transition("MRM_DEFER",
                                             "no viable candidate left — stock MRM")
                            return
                        self._chosen = best
                        self._publish_state()
                        self.get_logger().warn(f"fallback target: {best.name}")
                else:
                    self._stuck_since = None

        # MRM_PARKED / MRM_DEFER are terminal for this run

    # ---------------------------------------------------------- MRM planning
    def _enter_mrm_move(self, cap_loc):
        if self.pose is None:
            # keep waiting in the degraded band until localization arrives
            self.get_logger().warn("degraded, but no /localization/kinematic_state yet — waiting")
            return
        best = self._select_candidate(cap_loc)
        if best is None:
            self._select_failures += 1
            if self._select_failures >= 30:   # ~3 s of consecutive failures
                self._transition("MRM_DEFER", "no reachable MRM candidate (after retries)")
            return
        self._select_failures = 0
        self._chosen = best
        self._goal_sent = False
        self._transition("MRM_MOVE", f"target {best.name}")

    def _tl_observable(self):
        """A traffic light ahead is observable if any camera whose FOV covers
        the forward bearing band is still healthy. Function-level availability:
        the front camera dying is fine when another camera sees the signal."""
        names = list(self.get_parameter("tl_camera_names").value)
        yaws = list(self.get_parameter("tl_camera_yaws_deg").value)
        fovs = list(self.get_parameter("tl_camera_fovs_deg").value)
        th = self.get_parameter("tl_visible_min_score").value
        bearing = self.get_parameter("tl_bearing_deg").value
        for name, yaw, fov in zip(names, yaws, fovs):
            covers = abs(yaw) - fov / 2.0 <= bearing   # FOV overlaps [-bearing, bearing]
            healthy = self.scores.get(f"sensor/{name}", 1.0) >= th
            if covers and healthy:
                return True
        return False

    def _select_candidate(self, cap_loc):
        if self.pose is None:
            return None
        src = self.map.nearest_road_lanelet(*self.pose)
        self.get_logger().info(f"selecting from pose ({self.pose[0]:.1f}, {self.pose[1]:.1f}), "
                               f"current lanelet {src}")
        cap_tl = self.scores.get("capability/traffic_light", 1.0)
        w_t = self.get_parameter("w_time").value
        tl_wait = self.get_parameter("tl_wait_s").value
        w_tl = self.get_parameter("w_tl").value
        w_sec = self.get_parameter("w_sector").value
        w_loc = self.get_parameter("w_loc").value

        best, best_cost, evaluated = None, float("inf"), []
        for c in self.candidates:
            if c.name in self._blacklist:
                continue     # goal_planner already failed to park there
            r = self.map.route(src, c.target_lanelet)
            if r is None:
                evaluated.append((c, float("inf"), "unreachable"))
                continue
            length, time_s, n_tl = r
            approach = max(length, math.dist(self.pose, (c.x, c.y)))
            if approach < self.get_parameter("min_approach_dist").value:
                evaluated.append((c, float("inf"), f"too close ({approach:.0f}m)"))
                continue
            if n_tl > 0 and not self._tl_observable():
                evaluated.append((c, float("inf"),
                                  f"{n_tl} traffic lights, no healthy camera covers them"))
                continue
            g = (1.0 / max(cap_tl, 0.05)) - 1.0
            sector_pen = sum(1.0 - self.scores.get(f"sector/{k}", 1.0)
                             for k in c.required_sectors)
            # exposure time in the degraded state, incl. expected signal waits
            eta = time_s + n_tl * tl_wait
            cost = (w_t * eta
                    + w_tl * n_tl * g
                    + w_sec * sector_pen
                    + w_loc * eta * (1.0 - cap_loc))
            evaluated.append((c, cost,
                              f"len {length:.0f}m eta {eta:.0f}s tl {n_tl} sec {sector_pen:.2f}"))
            if cost < best_cost:
                best, best_cost = c, cost

        for c, cost, why in sorted(evaluated, key=lambda e: e[1]):
            self.get_logger().info(f"  candidate {c.name} [{c.kind}]: cost "
                                   f"{'INF' if math.isinf(cost) else f'{cost:.0f}'} ({why})")
        self._publish_markers(evaluated, best)
        return best

    def _send_goal(self):
        if not self.route_cli.service_is_ready():
            self.get_logger().warn("route service not ready, retrying...")
            return
        req = SetRoutePoints.Request()
        req.header.frame_id = "map"
        req.option.allow_goal_modification = True
        # not present in older autoware_adapi_v1_msgs versions
        if hasattr(req.option, "allow_while_using_route"):
            req.option.allow_while_using_route = True
        req.goal = Pose(position=Point(x=self._chosen.x, y=self._chosen.y, z=0.0),
                        orientation=yaw_to_quat(self._chosen.yaw))
        self.route_cli.call_async(req).add_done_callback(self._goal_response)
        self._goal_sent = True
        self._goal_sent_time = self.get_clock().now()
        self._goal_accepted = False
        self.get_logger().warn(f"MRM goal sent: {self._chosen.name} "
                               f"({self._chosen.x:.1f}, {self._chosen.y:.1f})")

    def _goal_response(self, future):
        try:
            res = future.result()
            if not res.status.success:
                self.get_logger().error(f"route change rejected: {res.status.message} — retrying")
                self._goal_sent = False
            else:
                self._goal_accepted = True
                self._noresp = 0
                self.get_logger().warn("route change ACCEPTED by Autoware")
        except Exception as e:
            self.get_logger().error(f"route change failed: {e} — retrying")
            self._goal_sent = False

    # ------------------------------------------------------------- reporting
    def _transition(self, state, why):
        if state != self.state:
            self.get_logger().warn(f"state {self.state} -> {state}: {why}")
            self.state = state
            self._publish_state()

    def _publish_state(self):
        detail = f":{self._chosen.name}" if self._chosen else ""
        self.state_pub.publish(String(data=self.state + detail))

    def _publish_markers(self, evaluated, best):
        arr = MarkerArray()
        for i, (c, cost, _) in enumerate(evaluated):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "mrm_candidates"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=c.x, y=c.y, z=1.0)
            m.pose.orientation = yaw_to_quat(c.yaw)
            m.scale.x = m.scale.y = m.scale.z = 2.5
            if c is best:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
            elif math.isinf(cost):
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.6
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.7
            arr.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = "mrm_costs"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position = Point(x=c.x, y=c.y, z=3.5)
            t.scale.z = 1.5
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{c.name}\n{'INF' if math.isinf(cost) else f'{cost:.0f}'}"
            arr.markers.append(t)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = Supervisor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
