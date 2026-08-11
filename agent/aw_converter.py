import numpy as np
from collections import deque

from scipy.spatial.transform import Rotation
from scipy.signal import butter, lfilter
from collections import deque
import uuid

import carla

from builtin_interfaces.msg import Duration
from unique_identifier_msgs.msg import UUID
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, Point32, Vector3, Quaternion, PoseWithCovariance, TwistWithCovariance, AccelWithCovariance, Pose

from autoware_agent.tum_ros_base_agent import BridgeHelper


from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import PredictedObjects, PredictedObject, ObjectClassification, PredictedPath, Shape, TrafficLightElement


class AutowareToCarlaControl:
    def __init__(self):
        # Lateral Conversion Map
        self._ackermann_angle = np.array([0, 6.7780, 13.1744,
                                          19.2750, 25.1526,
                                          30.8688, 36.4756,
                                          42.0187, 47.5399,
                                          53.0797, 58.6798])
        self._carla_angle = np.array([0.0, -0.1, -0.2, -0.3, -0.4, -0.5,
                                       -0.6, -0.7, -0.8, -0.9, -1.0])
        
        # Longitudinal P-Control
        self._Kp = 0.5
        self._Ki = 0.1
        self._Kd = 0.0
        self._max_throttle = 0.6
        self._max_brake = 0.2
        self._dt = 0.05
        self._integral_error = 0.0
        self._prev_error = 0.0

        #self._grid_data = np.load("accel_map_grid.npz")
        #self._grid_vel = self._grid_data["grid_vel"]
        #self._grid_throttle = self._grid_data["grid_throttle"]
        #self._grid_acc = self._grid_data["grid_acc"]
    
    def convertLatAwToCarla(self, ackermann_angle):
        # Convert the Ackermann angle to the corresponding CARLA angle
        ackermann_angle = np.clip(ackermann_angle, -self._ackermann_angle[-1], self._ackermann_angle[-1])
        sign = np.sign(ackermann_angle)
        carla_angle = np.interp(abs(ackermann_angle), self._ackermann_angle, self._carla_angle)
        return sign * carla_angle
    
    def updateThrottleBrake(self, target_acceleration, current_acceleration, velocity):
        #print('Current Acceleration', current_acceleration)
        #print('Target Acceleration', target_acceleration)

        error = target_acceleration - current_acceleration
        self._integral_error += error * self._dt
        derivate_error = (error - self._prev_error) / self._dt

        if velocity > 2.0:
            control_output = self._Kp * error + self._Ki * self._integral_error + self._Kd * derivate_error
        else:
            control_output = self._Kp * error + self._Ki * self._integral_error 
        #print('P', self._Kp * error)
        #print('I', self._Ki * self._integral_error)

        if control_output > self._max_throttle:
            control_output = self._max_throttle
            self._integral_error -= error * self._dt  # Undo last integration step
        elif control_output < -self._max_brake:
            control_output = -self._max_brake
            self._integral_error -= error * self._dt
        self._prev_error = error

        throttle = max(0.0, min(self._max_throttle, control_output)) 
        brake = max(0.0, min(self._max_brake, -control_output))      
        return throttle, brake
    
    def mapThrottleBrake(self, current_acc,target_acceleration, current_velocity):
        if current_velocity < 0.01:
            current_velocity = 0.01
        velocity_idx = np.abs(self._grid_vel[0, :] - current_velocity).argmin()
        accs_at_target_velocity = self._grid_acc[:, velocity_idx]
        throttle_idx = np.abs(accs_at_target_velocity - target_acceleration).argmin()
        throttle = self._grid_throttle[throttle_idx,0]
        print(f"Throttle for current_acc {current_acc} target acceleration {target_acceleration} at velocity {current_velocity}: {throttle:.2f}")

        if throttle < 0.0:
            brake = -throttle
            throttle = 0.0
        else:   
            throttle = throttle
            brake = 0.0
        
        return throttle, brake

class CarlaObjectsToAutoware:
    def __init__(self, ego_vehicle, world):
        self._ego_vehicle = ego_vehicle
        self._world = world
        self._timestep = self._world.get_settings().fixed_delta_seconds
        self._ros_duration = Duration()
        self._ros_duration.sec = int(self._timestep)
        self._ros_duration.nanosec = int((self._timestep - int(self._timestep)) * 1e9)
        self._exclude_actors = [
            "brokentile",
            "calibrator",
            "mesh",
            "friction",
            "dirtdebris",
            "sensor",
            "traffic_light",
            "unknown",
            "spectator"
        ]

        # TODO (GEMB): Instead of using default wheelbase, use the bounding box of the vehicle
        self._wheelbase = 2.86 # Lincoln MKZ2020
        self._prediction_horizon = 2.5
        self._detection_radius = 300.0

        # Save prev steering since we need to smooth the steering
        self._steering_angle_history = {}
        self._moving_average_window =7

        # Debugging
        self._first_actor = None

    def get_predicted_objects_msg(self):
        predicted_objects_msg = PredictedObjects()
        predicted_objects_msg.header.frame_id = "map"

        actors = self._world.get_actors()
        ego_location = self._ego_vehicle.get_location()
        # Filter out actors that are too far away and by keywords
        nearby_actors = [
            actor
            for actor in actors
            if actor.id != self._ego_vehicle.id and 
            actor.get_location().distance(ego_location) < self._detection_radius and
            not any(keyword in actor.type_id.lower() for keyword in self._exclude_actors)
            and actor.get_location().z > -10.0 # CARLA places all actors necessary for the scenario somewhere underground
        ]

        for actor in nearby_actors:
            pred_obj_msg = PredictedObject()
            pred_obj_msg.object_id = UUID()
            pred_obj_msg.object_id.uuid = list(uuid.uuid5(uuid.NAMESPACE_DNS, str(actor.id)).bytes)
            pred_obj_msg.existence_probability = 1.0

            obj_position_ros = BridgeHelper.carla2ros_pose(
            actor.get_location().x, actor.get_location().y, actor.get_location().z,
            np.deg2rad(actor.get_transform().rotation.roll), 
            np.deg2rad(actor.get_transform().rotation.pitch),
            np.deg2rad(actor.get_transform().rotation.yaw),
            to_quat=True
            )
            
            # For efficiency we do not transform the velocity to the vehicle coordinate system
            vel = np.array([actor.get_velocity().x, actor.get_velocity().y])
            vel = np.linalg.norm(vel)
            #actor_velocity = self._from_map_to_vehicle(actor, actor_velocity)

            obj_velocity_ros = BridgeHelper.carla2ros_pose(
            actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z,
            np.deg2rad(actor.get_angular_velocity().x), 
            np.deg2rad(actor.get_angular_velocity().y),
            np.deg2rad(actor.get_angular_velocity().z),
            to_quat=False
            )

            obj_acc_ros = BridgeHelper.carla2ros_pose(
            actor.get_acceleration().x, actor.get_acceleration().y, actor.get_acceleration().z,
            0.0, 0.0, 0.0,
            to_quat=False
            )

            obj_yaw = -np.deg2rad(actor.get_transform().rotation.yaw)

            pred_obj_msg.kinematics.initial_pose_with_covariance.pose.position = Point(**obj_position_ros["position"])
            pred_obj_msg.kinematics.initial_pose_with_covariance.pose.orientation = Quaternion(**obj_position_ros["orientation"])

            pred_obj_msg.kinematics.initial_twist_with_covariance.twist.linear.x = vel
            pred_obj_msg.kinematics.initial_twist_with_covariance.twist.angular = Vector3(
                x=obj_velocity_ros["orientation"]["roll"],
                y=obj_velocity_ros["orientation"]["pitch"],
                z=obj_velocity_ros["orientation"]["yaw"],
            )
            
            pred_obj_msg.kinematics.initial_acceleration_with_covariance.accel.linear = Vector3(**obj_acc_ros["position"])


            type_id = actor.type_id
            if "vehicle" in type_id:
                classification = ObjectClassification()
                classification.label = ObjectClassification.CAR
                classification.probability = 1.0
                pred_obj_msg.classification.append(classification)
                pred_obj_msg.kinematics.predicted_paths.append(self._predict_vehicle_bycicle(actor, obj_position_ros, vel, obj_yaw))
                pred_obj_msg.shape = self.get_bbox(actor, Shape.BOUNDING_BOX)
            elif "walker" in type_id:
                classification = ObjectClassification()
                classification.label = ObjectClassification.PEDESTRIAN
                classification.probability = 1.0
                pred_obj_msg.classification.append(classification)
                pred_obj_msg.kinematics.predicted_paths.append(self._predict_pedestrian(actor, obj_position_ros, vel, obj_yaw))
                pred_obj_msg.shape = self.get_bbox(actor, Shape.CYLINDER)
            
            else:
                classification = ObjectClassification()
                classification.label = ObjectClassification.UNKNOWN
                classification.probability = 1.0
                pred_obj_msg.classification.append(classification)
                pred_obj_msg.kinematics.predicted_paths.append(self._predict_pedestrian(actor, obj_position_ros, vel, obj_yaw))
                pred_obj_msg.shape = self.get_bbox(actor, Shape.BOUNDING_BOX)
            
            predicted_objects_msg.objects.append(pred_obj_msg)
        
        return predicted_objects_msg
    
    
    def get_bbox(self, actor, shape):
        shape_msg = Shape()
        shape_msg.type = shape
        
        bbox_vertices_global = actor.bounding_box.get_world_vertices(actor.get_transform())
        vertices_np = np.array([[vert.x, vert.y, vert.z] for vert in bbox_vertices_global])
        footprint = vertices_np[vertices_np[:, 2].argsort()][:4]

        ros_points = []
        for x, y, z in footprint:
            transformed = BridgeHelper.carla2ros_pose(x, y, z, 0.0, 0.0, 0.0)  # No rotation needed for just points
            pos = transformed["position"]
            ros_points.append(Point32(x=pos["x"], y=pos["y"], z=pos["z"]))
        shape_msg.footprint.points = ros_points
        shape_msg.dimensions = Vector3(x=actor.bounding_box.extent.x * 2, y=actor.bounding_box.extent.y * 2, z=actor.bounding_box.extent.z * 2)
        return shape_msg
    
    # Bycyclemodel with simplified heading update
    # This does not really work since we get the velocity in map coordinate system
    def _predict_vehicle_bycicle(self, actor, pos, vel, yaw):
        predicted_path = PredictedPath()
        predicted_path.time_step = self._ros_duration
        predicted_path.confidence = 1.0
        predicted_path.path = []

        timestep = 0

        #vel = vel["position"]["x"]
        #vel = np.linalg.norm(np.array([vel["position"]["x"], vel["position"]["y"]]))

        wheel_steering_L = actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        wheel_steering_R = actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        wheel_steering = np.deg2rad(-(wheel_steering_L + wheel_steering_R) / 2.0)

        # Smooth the steering
        if actor.id not in self._steering_angle_history:
            self._steering_angle_history[actor.id] = deque(maxlen=self._moving_average_window)
            self._steering_angle_history[actor.id].append(wheel_steering)
        else:
            # Apply a moving average filter to smooth the steering angle
            self._steering_angle_history[actor.id].append(wheel_steering)
            wheel_steering = sum(self._steering_angle_history[actor.id]) / len(self._steering_angle_history[actor.id])

                # Debugging
        #if self._first_actor is None:
        #    self._first_actor = actor
        #elif actor.id == self._first_actor.id:
        #    print('First Actor', actor.id, yaw, wheel_steering, actor.get_control().steer)
        #    print(f"L: {wheel_steering_L:.2f}°, R: {wheel_steering_R:.2f}°, avg: {(wheel_steering_L + wheel_steering_R) / 2.0:.2f}°")

        #if yaw < 0.05: yaw = 0.0
        position = pos.copy()

        while timestep < self._prediction_horizon:
            
            
            pos["position"]["x"] +=  vel * np.cos(yaw) * self._timestep
            pos["position"]["y"] +=  vel * np.sin(yaw) * self._timestep
            yaw += vel / self._wheelbase * np.tan(wheel_steering) * self._timestep
            

            predicted_pose = Pose()
            predicted_pose.position = Point(**pos["position"])
            quat = Rotation.from_euler('z', yaw).as_quat()
            predicted_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            predicted_path.path.append(predicted_pose)

            timestep += self._timestep

        return predicted_path
    

    # Point mass
    def _predict_pedestrian(self, actor, pos, vel, yaw):
        predicted_path = PredictedPath()
        predicted_path.time_step = self._ros_duration
        predicted_path.confidence = 1.0
        predicted_path.path = []

        timestep = 0

        while timestep < self._prediction_horizon:
            

            pos["position"]["x"] +=  vel * np.cos(yaw) * self._timestep
            pos["position"]["y"] +=  vel * np.sin(yaw) * self._timestep

            predicted_pose = Pose()
            predicted_pose.position = Point(**pos["position"])
            predicted_pose.orientation = Quaternion(**pos["orientation"])
            predicted_path.path.append(predicted_pose)

            timestep += self._timestep

        return predicted_path
    
    def _from_map_to_vehicle(self, actor, vel):
        cos_roll = np.cos(np.deg2rad(actor.get_transform().rotation.roll))
        sin_roll = np.sin(np.deg2rad(actor.get_transform().rotation.roll))
        cos_pitch = np.cos(np.deg2rad(actor.get_transform().rotation.pitch))
        sin_pitch = np.sin(np.deg2rad(actor.get_transform().rotation.pitch))
        cos_yaw = np.cos(np.deg2rad(actor.get_transform().rotation.yaw))
        sin_yaw = np.sin(np.deg2rad(actor.get_transform().rotation.yaw))
        R_x = np.array([
            [1, 0, 0],
            [0, cos_roll, -sin_roll],
            [0, sin_roll, cos_roll]
        ])

        R_y = np.array([
            [cos_pitch, 0, sin_pitch],
            [0, 1, 0],
            [-sin_pitch, 0, cos_pitch]
        ])

        R_z = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw, cos_yaw, 0],
            [0, 0, 1]
        ])  
    
        R = np.dot(R_z, np.dot(R_y, R_x))
        R_inv = R.T

        return np.dot(R_inv, vel)


class AutowareConverter:
    def __init__(self, ego_vehicle: carla.Vehicle, world: carla.World):
        self._ego_vehicle = ego_vehicle
        self._world = world
        self._vector_carlavehicle_to_autowarebase = np.array([0.0, 0.0, 0.0])
        self._control_converter = AutowareToCarlaControl()
        self._carla_objects_converter = CarlaObjectsToAutoware(self._ego_vehicle, self._world)

        # ACC butter filter + low pass filter
        self._acc_buffer_size = 20
        self._acc_cutoff = 1.0
        self._acc_fs = 20.0
        self._acc_filter_order = 2

        self._b_acc, self._a_acc = butter(self._acc_filter_order, self._acc_cutoff / (0.5 * self._acc_fs), btype='low')
        self._acc_x = deque(maxlen=self._acc_buffer_size)
        self._acc_y = deque(maxlen=self._acc_buffer_size)
        self._acc_z = deque(maxlen=self._acc_buffer_size)
        
        self._last_acc = 0.0

        # ACC butter filter + low pass filter
        self._vel_buffer_size = 20
        self._vel_cutoff = 1.0
        self._vel_fs = 20.0
        self._vel_filter_order = 2

        self._b_vel, self._a_vel = butter(self._vel_filter_order, self._vel_cutoff / (0.5 * self._vel_fs), btype='low')
        self._vel_x = deque(maxlen=self._vel_buffer_size)
        self._vel_y = deque(maxlen=self._vel_buffer_size)
        self._vel_z = deque(maxlen=self._vel_buffer_size)


        self._carla_to_autoware_traffic_light_map = {
            carla.TrafficLightState.Red: TrafficLightElement.RED,
            carla.TrafficLightState.Yellow: TrafficLightElement.AMBER,
            carla.TrafficLightState.Green: TrafficLightElement.GREEN}
        
        # Traffic lights
        self._last_tl = TrafficLightElement.RED
        self._steps_since_last_tl = 0
    
    def _get_localization(self):
        ego_position = np.array([
            self._ego_vehicle.get_location().x,
            self._ego_vehicle.get_location().y,
            self._ego_vehicle.get_location().z,
        ])
        # TODO (Gemb): What is this forward_vector?
        ego_forward_vector = self._ego_vehicle.get_transform().rotation.get_forward_vector()

        # Shift from carla middle of the vehicle to the autoware rear axle
        ego_position_shifted = ego_position + self._vector_carlavehicle_to_autowarebase

        # Convert CARLA coordinates to ROS2-compatible coordinates
        # First adding the roll pith yaw angles from the ego_forward_vector_np
        ego_position_ros = BridgeHelper.carla2ros_pose(
            ego_position_shifted[0], ego_position_shifted[1], ego_position_shifted[2],
            np.deg2rad(self._ego_vehicle.get_transform().rotation.roll), 
            np.deg2rad(self._ego_vehicle.get_transform().rotation.pitch),
            np.deg2rad(self._ego_vehicle.get_transform().rotation.yaw),
            to_quat=True
        )
        return ego_position_ros
    
    # Velocity and accelerationa are given in the  global coordinate_system
    def _from_map_to_vehicle(self, map_pos):
        cos_roll = np.cos(np.deg2rad(self._ego_vehicle.get_transform().rotation.roll))
        sin_roll = np.sin(np.deg2rad(self._ego_vehicle.get_transform().rotation.roll))
        cos_pitch = np.cos(np.deg2rad(self._ego_vehicle.get_transform().rotation.pitch))
        sin_pitch = np.sin(np.deg2rad(self._ego_vehicle.get_transform().rotation.pitch))
        cos_yaw = np.cos(np.deg2rad(self._ego_vehicle.get_transform().rotation.yaw))
        sin_yaw = np.sin(np.deg2rad(self._ego_vehicle.get_transform().rotation.yaw))
        R_x = np.array([
            [1, 0, 0],
            [0, cos_roll, -sin_roll],
            [0, sin_roll, cos_roll]
        ])

        R_y = np.array([
            [cos_pitch, 0, sin_pitch],
            [0, 1, 0],
            [-sin_pitch, 0, cos_pitch]
        ])

        R_z = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw, cos_yaw, 0],
            [0, 0, 1]
        ])  
    
        R = np.dot(R_z, np.dot(R_y, R_x))
        R_inv = R.T

        return np.dot(R_inv, map_pos)

    
    def _get_twist(self):
        ego_velocity = np.array([
            self._ego_vehicle.get_velocity().x,
            self._ego_vehicle.get_velocity().y,
            self._ego_vehicle.get_velocity().z
        ])

        ego_velocity = self._from_map_to_vehicle(ego_velocity)
        
        ego_angular_velocity = np.array([
            self._ego_vehicle.get_angular_velocity().x,
            self._ego_vehicle.get_angular_velocity().y,
            self._ego_vehicle.get_angular_velocity().z,
        ])
        
        # Convert CARLA coordinates to ROS2-compatible coordinates
        # First adding the roll pith yaw angles from the ego_forward_vector_np
        ego_twist_ros = BridgeHelper.carla2ros_pose(
            ego_velocity[0], ego_velocity[1], ego_velocity[2],
            np.deg2rad(ego_angular_velocity[0]), 
            np.deg2rad(ego_angular_velocity[1]),
            np.deg2rad(ego_angular_velocity[2]),
            to_quat=False
        )

        self._vel_x.append(ego_twist_ros["position"]["x"])
        self._vel_y.append(ego_twist_ros["position"]["y"])
        self._vel_z.append(ego_twist_ros["position"]["z"])
        
        # lp
        if len(self._vel_x) < self._vel_buffer_size:
            pass  # just return raw
        else:
            pass
            #ego_twist_ros["position"]["x"] = lfilter(self._b_vel, self._a_vel, list(self._vel_x))[-1]
            #ego_twist_ros["position"]["y"] = lfilter(self._b_vel, self._a_vel, list(self._vel_y))[-1]
            #ego_twist_ros["position"]["z"] = lfilter(self._b_vel, self._a_vel, list(self._vel_z))[-1]


        return ego_twist_ros
    
    def _get_acceleration(self):
        ego_acceleration = np.array([
            self._ego_vehicle.get_acceleration().x,
            self._ego_vehicle.get_acceleration().y,
            self._ego_vehicle.get_acceleration().z,
        ])
        
        ego_acceleration = self._from_map_to_vehicle(ego_acceleration)

        # CARLA hat no api to read the angular acceleration
        ego_acceleration_ros = BridgeHelper.carla2ros_pose(
            ego_acceleration[0], ego_acceleration[1], ego_acceleration[2],
            0.0, 0.0, 0.0,
            to_quat=False
        )

        self._acc_x.append(ego_acceleration_ros["position"]["x"])
        self._acc_y.append(ego_acceleration_ros["position"]["y"])
        self._acc_z.append(ego_acceleration_ros["position"]["z"])
        
        # lp
        if len(self._acc_x) < self._acc_buffer_size:
            pass  # just return raw
        else:
            ego_acceleration_ros["position"]["x"] = lfilter(self._b_acc, self._a_acc, list(self._acc_x))[-1]
            ego_acceleration_ros["position"]["y"] = lfilter(self._b_acc, self._a_acc, list(self._acc_y))[-1]
            ego_acceleration_ros["position"]["z"] = lfilter(self._b_acc, self._a_acc, list(self._acc_z))[-1]

        return ego_acceleration_ros
    
    def _get_steering(self):
        # TODO (Gemb): Calculate the ackermann angle instead of taking the verage
        ego_wheel_steering_L = self._ego_vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        ego_wheel_steering_R = self._ego_vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        # Desired angle of the steering tire in radians left (positive)"
        # or right (negative) of center (0.0)
        ego_wheel_steering = -(ego_wheel_steering_L + ego_wheel_steering_R) / 2.0
        return np.deg2rad(ego_wheel_steering)
    
    def convert_control(self, aw_control_msg):
        carla_steering_angle = self._control_converter.convertLatAwToCarla(np.rad2deg(aw_control_msg.lateral.steering_tire_angle))
        # 후진 지원: Autoware는 후진 구간에서 목표 속도를 음수로 주지만,
        # 가속도는 "항상 진행방향 기준"이다 (+ = 증속, - = 제동; pid_longitudinal_
        # controller applyVelocityFeedback의 NOTE 참조). 그래서 가속도는 부호
        # 그대로 쓰고, 진행방향 현재속도만 미러링한다.
        # (기존 acc_cmd = -lon.acceleration 미러링이 후진 가속(+0.5)을 브레이크로
        # 뒤집어 주차 후진이 영구 정지하던 원인 — 8/10 게이트 프로브 실측)
        lon = aw_control_msg.longitudinal
        is_reverse = lon.velocity < -0.01
        acc_cmd = lon.acceleration
        cur_speed = self._get_twist()["position"]["x"]
        if is_reverse:
            cur_speed = -cur_speed
        carla_throttle, carla_brake = self._control_converter.updateThrottleBrake(
            acc_cmd, self._last_acc, cur_speed
        )
        #carla_throttle, carla_brake = self._control_converter.mapThrottleBrake(
        #    self._get_acceleration()['position']['x'], aw_control_msg.longitudinal.acceleration, self._get_twist()["position"]["x"]
        #)
        control_timestamp = aw_control_msg.stamp.sec + aw_control_msg.stamp.nanosec * 1e-9
        control = carla.VehicleControl(
            steer=carla_steering_angle,
            throttle=carla_throttle,
            brake=carla_brake,
            hand_brake=False,
            reverse=is_reverse,
            manual_gear_shift=False,
            gear=0
        )

        return control_timestamp, control
    
    def convert_lidar_data(self, lidar_numpy):

        lidar_numpy[:, 1] *= -1

        lidar_numpy[:, 3] *= 255
        np.clip(lidar_numpy[:, 3], 0, 255, out=lidar_numpy[:, 3])

        num_channels = 64  # Set this to match your CARLA config

        # Trim lidar_numpy to the nearest full channel set
        total_points = lidar_numpy.shape[0]
        points_per_channel = total_points // num_channels
        valid_point_count = points_per_channel * num_channels

        lidar_numpy = lidar_numpy[:valid_point_count]

        # Simulate channel ID (C) and return type (R)
        channel_ids = np.tile(np.arange(num_channels), points_per_channel)
        return_types = np.zeros(valid_point_count, dtype=np.uint8)  # All single return

        #  x(float32), y(float32), z(float32), intensity(uint8), padding(3 bytes)
        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.uint8),
            ('return_type', np.uint8),      
            ('channel', np.uint16) 
        ])

        structured_points = np.zeros(lidar_numpy.shape[0], dtype=dtype)

        structured_points['x'] = lidar_numpy[:, 0]
        structured_points['y'] = lidar_numpy[:, 1]
        structured_points['z'] = lidar_numpy[:, 2]
        structured_points['intensity'] = lidar_numpy[:, 3].astype(np.uint8)
        structured_points['return_type'] = return_types
        structured_points['channel'] = channel_ids

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
        ]

        msg = PointCloud2()
        msg.height = 1
        msg.width = structured_points.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 16 bytes per point (float32*3 + uint8 + 3 padding)
        msg.row_step = msg.point_step * structured_points.shape[0]
        msg.is_dense = True
        msg.data = structured_points.tobytes()  
        return msg  
    
    def getAcc(self, loc_msg):
        self._last_acc = loc_msg.accel.accel.linear.x
    
    def create_predicted_object_message(self):
        return self._carla_objects_converter.get_predicted_objects_msg()
    
    def get_current_traffic_light_state(self):
        if self._ego_vehicle.is_at_traffic_light():
            self._steps_since_last_tl = 0
            tl_state = self._ego_vehicle.get_traffic_light_state()
            self._last_tl = self._carla_to_autoware_traffic_light_map[tl_state]
            return self._carla_to_autoware_traffic_light_map[tl_state]
        else:
            # Sometimes the carla traffic light areas are no set correctly
            # thats why we will keep the last tl state green for a while
            if self._last_tl != TrafficLightElement.GREEN:
                self._last_tl = TrafficLightElement.RED
                return TrafficLightElement.RED
            self._steps_since_last_tl += 1
            if self._steps_since_last_tl > 30:
                self._steps_since_last_tl = 0
                self._last_tl = TrafficLightElement.RED
                return TrafficLightElement.RED
            
            else:
                return TrafficLightElement.GREEN
        