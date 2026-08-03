import carla
import time
import math
import numpy as np

# ==========================================
# 1. 基础控制算法 (PID) - 已优化
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        # 增加积分限幅，防止积分饱和
        _ie = np.clip(_ie, -2.0, 2.0)
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

class PIDLateralController:
    def __init__(self, K_P=1.0, K_I=0.01, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint_loc, vehicle_transform):
        # 【优化】使用航向角(Yaw)差值代替原先的点积acos运算，极大地减小了数值抖动
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)

        # 计算目标点相对于车辆的方位角
        target_vector = np.array([waypoint_loc.x - v_loc.x, waypoint_loc.y - v_loc.y])
        norm = np.linalg.norm(target_vector)
        if norm < 0.1: return 0.0

        target_yaw = math.atan2(target_vector[1], target_vector[0])

        # 计算角度差，并归一化到 [-pi, pi]
        error = target_yaw - v_yaw
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi

        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed, target_wp_loc):
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed, speed)
    steer_output = pid_lat.run_step(target_wp_loc, tf)

    # 【优化保留】抑制微小抖动，因为算法优化，死区可以从0.1大幅降低到0.02
    if abs(steer_output) < 0.02:
        steer_output = 0.0

    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)
    vehicle.apply_control(control)

# ==========================================
# 2. Helper functions: road cruising and trajectory tracking
# ==========================================
def get_straightest_waypoint(current_wp, distance=5.0):
    """【增强】绝对直行逻辑，通过比对Yaw角度差确保直行"""
    next_wps = current_wp.next(distance)
    if not next_wps: return None
    if len(next_wps) == 1: return next_wps[0]

    best_wp = next_wps[0]
    min_yaw_diff = float('inf')
    curr_yaw = current_wp.transform.rotation.yaw

    for wp in next_wps:
        wp_yaw = wp.transform.rotation.yaw
        # 计算角度差
        diff = abs(curr_yaw - wp_yaw)
        while diff > 180.0: diff = abs(diff - 360.0)

        if diff < min_yaw_diff:
            min_yaw_diff = diff
            best_wp = wp
    return best_wp

def get_dynamic_right_turn_waypoint(current_wp, distance=3.0):
    next_wps = current_wp.next(distance)
    if not next_wps: return None
    if len(next_wps) == 1: return next_wps[0]

    best_wp = next_wps[0]
    max_right_dot = -1.0
    right_vec = current_wp.transform.get_right_vector()

    for wp in next_wps:
        wp_fwd = wp.transform.get_forward_vector()
        dot = right_vec.x * wp_fwd.x + right_vec.y * wp_fwd.y
        if dot > max_right_dot:
            max_right_dot = dot
            best_wp = wp
    return best_wp

EGO_TRAJECTORY = [
    (-39.949, 2.286, -1.640), (-39.949, 2.286, -1.640), (-39.949, 2.286, -1.640), (-39.949, 2.286, -1.640),
    (-39.949, 2.286, -1.640), (-39.949, 2.286, -1.640), (-39.636, 2.277, -1.640), (-38.261, 2.238, -1.640),
    (-34.887, 2.141, -1.640), (-32.263, 2.066, -1.640), (-28.452, 1.958, -1.500), (-24.704, 1.860, -1.640),
    (-20.893, 1.740, -1.920), (-17.146, 1.614, -1.920), (-14.398, 1.522, -1.920), (-13.848, 1.503, -1.920),
    (-13.340, 1.486, -1.920), (-12.840, 1.470, -1.920), (-12.332, 1.453, -1.920), (-11.832, 1.433, -2.569),
    (-11.325, 1.410, -2.569), (-10.193, 1.359, -2.569), (-8.923, 1.302, -2.779), (-7.677, 1.209, -5.411),
    (-6.433, 1.089, -5.904), (-5.170, 0.948, -6.473), (-3.923, 0.876, 2.339), (-2.674, 1.089, 13.774),
    (-1.473, 1.492, 23.988), (-0.369, 2.112, 35.208), (-0.083, 2.320, 36.280), (-0.083, 2.320, 36.280),
    (-0.083, 2.320, 36.280), (-0.083, 2.320, 36.280), (0.377, 2.677, 38.763), (1.326, 3.488, 42.808),
    (2.202, 4.378, 50.340), (2.969, 5.416, 54.571), (3.672, 6.448, 59.121), (4.250, 7.579, 66.575),
    (4.737, 8.753, 67.567), (5.193, 9.917, 70.501), (5.582, 11.104, 74.898), (5.897, 12.336, 78.921),
    (6.058, 13.596, 85.949), (6.107, 14.842, 88.293), (6.167, 16.862, 88.293), (6.135, 19.360, 95.520),
    (5.740, 21.870, 101.986), (5.151, 24.642, 101.986), (4.359, 28.371, 101.986), (3.568, 32.101, 101.986),
    (2.781, 35.832, 101.426), (2.126, 39.525, 97.593), (1.736, 43.254, 94.394), (1.448, 47.056, 94.324),
    (1.185, 50.796, 93.764), (1.077, 54.543, 90.213), (1.063, 58.355, 90.213), (1.063, 62.105, 89.793),
    (1.080, 65.856, 89.722), (1.098, 69.668, 89.722), (1.116, 73.418, 89.722), (1.135, 77.293, 89.722),
    (1.139, 81.106, 90.212), (1.117, 84.856, 90.422), (1.071, 88.668, 90.772), (1.021, 92.417, 90.772),
    (1.018, 92.605, 90.772), (1.018, 92.605, 90.772), (1.018, 92.605, 90.772),
]

def clean_xy_yaw_trajectory(points, min_dist=0.1):
    cleaned = []
    for pt in points:
        if not cleaned or math.hypot(pt[0] - cleaned[-1][0], pt[1] - cleaned[-1][1]) >= min_dist:
            cleaned.append(pt)
    return cleaned

def interpolate_xy_yaw_trajectory(points, interval=0.5):
    if not points:
        return []

    dense = []
    for i in range(len(points) - 1):
        x1, y1, yaw1 = points[i]
        x2, y2, yaw2 = points[i + 1]
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(1, int(dist / interval))
        for step in range(steps):
            ratio = step / steps
            dense.append((
                x1 + (x2 - x1) * ratio,
                y1 + (y2 - y1) * ratio,
                yaw1 + (yaw2 - yaw1) * ratio,
            ))
    dense.append(points[-1])
    return dense

def trajectory_locations(points, z=0.3):
    return [carla.Location(x=pt[0], y=pt[1], z=z) for pt in points]

def get_target_waypoint(vehicle_loc, path_points, current_index, speed_kmh,
                        min_lookahead=3.0, lookahead_ratio=0.35,
                        max_search_ahead=35, fallback_dist=20.0):
    if not path_points:
        return None, current_index

    vx, vy = vehicle_loc.x, vehicle_loc.y
    closest_index = current_index
    min_dist_sq = float('inf')
    search_end = min(current_index + max_search_ahead, len(path_points))

    for i in range(current_index, search_end):
        p = path_points[i]
        dist_sq = (p.x - vx) ** 2 + (p.y - vy) ** 2
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = i

    if min_dist_sq > fallback_dist ** 2:
        for i, p in enumerate(path_points):
            dist_sq = (p.x - vx) ** 2 + (p.y - vy) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_index = i

    lookahead_dist = max(min_lookahead, (speed_kmh / 3.6) * lookahead_ratio)
    target_index = closest_index
    travel = 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1, p2 = path_points[i], path_points[i + 1]
        travel += p1.distance(p2)
        target_index = i + 1
        if travel >= lookahead_dist:
            break

    return path_points[target_index], closest_index

def speed_kmh(vehicle):
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

def check_vehicle_out_of_bounds(vehicle, carla_map, threshold_dist=8.0, auto_destroy=True):
    if not vehicle or not vehicle.is_alive:
        return True

    loc = vehicle.get_location()
    waypoint = carla_map.get_waypoint(
        loc,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    if not waypoint:
        print(f"[OutOfBounds] Vehicle [{vehicle.id}] cannot be projected to a driving lane.")
        if auto_destroy and vehicle.is_alive:
            vehicle.destroy()
        return True

    dist_to_road = loc.distance(waypoint.transform.location)
    if dist_to_road > threshold_dist:
        print(f"[OutOfBounds] Vehicle [{vehicle.id}] is {dist_to_road:.1f}m from road center; destroying.")
        if auto_destroy and vehicle.is_alive:
            vehicle.destroy()
        return True

    return False

class EgoSpeedStateMachine:
    CRUISE = "CRUISE"
    SLOW = "SLOW"
    STOP = "STOP"
    RESUME = "RESUME"

    def __init__(self, stop_point, stop_radius=3.0, stop_seconds=8.0):
        self.state = self.CRUISE
        self.stop_point = stop_point
        self.stop_radius = stop_radius
        self.stop_seconds = stop_seconds
        self.stop_started_at = None

    def tick(self, ego_loc):
        now = time.time()

        if self.state == self.CRUISE and ego_loc.x >= -15.0:
            self.state = self.SLOW
            print("[EgoSM] CRUISE -> SLOW: target 20 km/h")

        if self.state == self.SLOW and ego_loc.distance(self.stop_point) <= self.stop_radius:
            self.state = self.STOP
            self.stop_started_at = now
            print("[EgoSM] SLOW -> STOP: hold brake for 4 wall-clock seconds")

        if self.state == self.STOP:
            if now - self.stop_started_at < self.stop_seconds:
                return 0.0, True
            self.state = self.RESUME
            print("[EgoSM] STOP -> RESUME: target 30 km/h")

        if self.state == self.SLOW:
            return 20.0, False
        return 30.0, False

# ==========================================
# 3. 主程序
# ==========================================

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (1.018, 92.605)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.lincoln.mkz_2020'
_RTB_OPT_EGO_ROLE_NAMES = ['ego', 'hero']
_RTB_OPT_GOAL_RADIUS_M = 5.0
_RTB_OPT_GOAL_HITS = 0

def _rtb_opt_is_alive(actor):
    return bool(actor is not None and hasattr(actor, 'is_alive') and actor.is_alive)

def _rtb_opt_iter_actor_values(value, seen=None):
    if seen is None:
        seen = set()
    obj_id = id(value)
    if obj_id in seen:
        return
    seen.add(obj_id)
    if _rtb_opt_is_alive(value) and hasattr(value, 'get_location'):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from _rtb_opt_iter_actor_values(item, seen)
    elif isinstance(value, (list, tuple, set)):
        for item in value:
            yield from _rtb_opt_iter_actor_values(item, seen)

def _rtb_opt_actor_matches_ego(actor):
    if not _rtb_opt_is_alive(actor):
        return False
    try:
        role_name = actor.attributes.get('role_name', '')
        if role_name in _RTB_OPT_EGO_ROLE_NAMES:
            return True
    except Exception:
        pass
    try:
        if _RTB_OPT_EGO_TYPE_ID and actor.type_id == _RTB_OPT_EGO_TYPE_ID:
            return True
    except Exception:
        pass
    return False

def _rtb_opt_find_ego(local_vars):
    preferred_names = ('ego', 'ego_vehicle', 'vehicle_ego', 'v3_ego', 'v2_ego', 'agent_ego', 'audi', 'tesla', 'moto', 'truck', 'firetruck')
    for name in preferred_names:
        if name in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[name]):
                if _rtb_opt_actor_matches_ego(actor) or 'ego' in name.lower():
                    return actor
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            if _rtb_opt_actor_matches_ego(actor):
                return actor
    return None

def _rtb_opt_collect_scene_actors(local_vars, world):
    actors = []
    seen = set()

    def add(actor):
        if not _rtb_opt_is_alive(actor):
            return
        try:
            actor_id = actor.id
        except Exception:
            actor_id = id(actor)
        if actor_id in seen:
            return
        seen.add(actor_id)
        actors.append(actor)

    for key in ('actor_list', 'actors', 'vehicles', 'spawned_actors'):
        if key in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[key]):
                add(actor)
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            add(actor)
    try:
        world_actors = world.get_actors()
        for pattern in ('vehicle.*', 'walker.*', 'sensor.*', 'controller.*', 'static.prop.*', 'static.trigger.*'):
            for actor in world_actors.filter(pattern):
                add(actor)
    except Exception:
        pass
    return actors

def _rtb_opt_cleanup_scene(local_vars, client, world):
    actors = _rtb_opt_collect_scene_actors(local_vars, world)
    try:
        commands = [carla.command.DestroyActor(actor.id) for actor in actors if _rtb_opt_is_alive(actor)]
        if commands:
            client.apply_batch(commands)
        return
    except Exception:
        pass
    for actor in actors:
        try:
            if _rtb_opt_is_alive(actor):
                actor.destroy()
        except Exception:
            pass

def _rtb_opt_goal_guard(local_vars, client, world):
    global _RTB_OPT_GOAL_HITS
    if _RTB_OPT_EGO_GOAL_XY is None:
        _RTB_OPT_GOAL_HITS = 0
        return False
    ego_actor = _rtb_opt_find_ego(local_vars)
    if not _rtb_opt_is_alive(ego_actor):
        _RTB_OPT_GOAL_HITS = 0
        return False
    try:
        loc = ego_actor.get_location()
        dist = ((loc.x - _RTB_OPT_EGO_GOAL_XY[0]) ** 2 + (loc.y - _RTB_OPT_EGO_GOAL_XY[1]) ** 2) ** 0.5
    except Exception:
        _RTB_OPT_GOAL_HITS = 0
        return False
    if dist <= _RTB_OPT_GOAL_RADIUS_M:
        _RTB_OPT_GOAL_HITS += 1
    else:
        _RTB_OPT_GOAL_HITS = 0
    if _RTB_OPT_GOAL_HITS >= 2:
        print('[RoadTailBench Opt] Ego reached trajectory endpoint; cleaning all scene actors and ending simulation.')
        _rtb_opt_cleanup_scene(local_vars, client, world)
        return True
    return False
# === End RoadTailBench Opt guard ===

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 设置精准天气
    weather = carla.WeatherParameters(
        cloudiness=20.0, precipitation=0.0, precipitation_deposits=5.0, wind_intensity=5.0,
        sun_azimuth_angle=240.0, sun_altitude_angle=22.0, fog_density=4.0, fog_distance=0.0,
        fog_falloff=0.0, wetness=5.0, scattering_intensity=0.5, mie_scattering_scale=0.1,
        rayleigh_scattering_scale=0.3, dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    # 车辆存活标志字典，方便随时释放车辆
    active_vehicles = {'v1': False, 'v2': False, 'v3': False, 'ego': False}

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pids = {
            'v2': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'v3': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'ego': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        }

        LIGHT_LOW_BEAM = carla.VehicleLightState.LowBeam
        LIGHT_HIGH_BEAM = carla.VehicleLightState.HighBeam
        LIGHT_BLINKER_RIGHT = carla.VehicleLightState.RightBlinker
        LIGHT_HAZARD = carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.Position
        LIGHT_POSITION = carla.VehicleLightState.Position

        # ==========================================
        # 车辆 1: 特斯拉 Model 3 (静止)
        # ==========================================
        bp_v1 = bp_lib.find('vehicle.tesla.model3')
        loc_v1 = carla.Location(x=1.0, y=10.975, z=0.3)
        v1 = world.try_spawn_actor(bp_v1, carla.Transform(loc_v1, carla_map.get_waypoint(loc_v1).transform.rotation))
        if v1:
            actor_list.append(v1)
            active_vehicles['v1'] = True
            v1.set_light_state(carla.VehicleLightState(LIGHT_HAZARD))
            v1.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            print("V1 (Tesla) 生成成功: 静止打双闪")

        # ==========================================
        # 车辆 2: 奥迪 TT (右转)
        # ==========================================
        bp_v2 = bp_lib.find('vehicle.audi.tt')
        if bp_v2.has_attribute('color'): bp_v2.set_attribute('color', '255,165,0')
        loc_v2 = carla.Location(x=5.089, y=66.621, z=0.3)
        v2_wp = carla_map.get_waypoint(loc_v2)
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(loc_v2, v2_wp.transform.rotation))
        if v2:
            actor_list.append(v2)
            active_vehicles['v2'] = True
            v2.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_LOW_BEAM))
            print("V2 (Audi TT) 生成成功: 橙色，PID控制准备右转")

        # ==========================================
        # 车辆 3: 警车 (严格直行)
        # ==========================================
        bp_v3 = bp_lib.find('vehicle.dodge.charger_police')
        loc_v3 = carla.Location(x=61.362, y=-1.89, z=0.3)
        v3_wp = carla_map.get_waypoint(loc_v3)
        v3 = world.try_spawn_actor(bp_v3, carla.Transform(loc_v3, v3_wp.transform.rotation))
        if v3:
            actor_list.append(v3)
            active_vehicles['v3'] = True
            v3.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_LOW_BEAM))
            print("V3 (Police) 生成成功: 警车，严格直行，远光灯闪烁")

        # ==========================================
        # Ego vehicle: Lincoln MKZ 2020, fixed trajectory control
        # ==========================================
        bp_ego = bp_lib.find('vehicle.lincoln.mkz_2020')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '230,230,230')
        if bp_ego.has_attribute('role_name'):
            bp_ego.set_attribute('role_name', 'ego')

        ego_raw_traj = clean_xy_yaw_trajectory(EGO_TRAJECTORY, min_dist=0.1)
        ego_traj = interpolate_xy_yaw_trajectory(ego_raw_traj, interval=0.5)
        ego_path = trajectory_locations(ego_traj, z=0.3)
        loc_ego_start = ego_path[0]
        ego_start_yaw = ego_raw_traj[0][2]
        ego = world.try_spawn_actor(
            bp_ego,
            carla.Transform(loc_ego_start, carla.Rotation(yaw=ego_start_yaw))
        )
        ego_route_idx = 0
        ego_speed_sm = EgoSpeedStateMachine(carla.Location(x=-0.083, y=2.320, z=0.3))
        if ego:
            actor_list.append(ego)
            active_vehicles['ego'] = True
            ego.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_HIGH_BEAM))
            ego.set_target_velocity(carla.Vector3D(
                x=(30.0 / 3.6) * math.cos(math.radians(ego_start_yaw)),
                y=(30.0 / 3.6) * math.sin(math.radians(ego_start_yaw)),
                z=0.0
            ))
            print("Ego (Lincoln) spawned: silver-white, fixed trajectory PID control")
        print("\n车辆加载完毕，等待物理稳定...")
        for _ in range(20): world.tick()
        print("仿真正式开始...")

        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            for key, vehicle in (('v1', v1), ('v2', v2), ('v3', v3), ('ego', ego)):
                if active_vehicles[key] and vehicle and vehicle.is_alive:
                    if check_vehicle_out_of_bounds(vehicle, carla_map, threshold_dist=8.0, auto_destroy=True):
                        active_vehicles[key] = False

            # ==========================
            # V2 控制: 动态寻迹 + 自动右转打灯 + 【出界销毁】
            # ==========================
            if active_vehicles['v2'] and v2.is_alive:
                lookahead_wps = v2_wp.next(8.0)
                if not lookahead_wps:
                    print("V2 驶出路网，自动销毁释放资源")
                    v2.destroy()
                    active_vehicles['v2'] = False
                else:
                    lookahead_wp = lookahead_wps[0]
                    next_options = lookahead_wp.next(8.0)

                    if next_options and len(next_options) > 1:
                        target_wp = get_dynamic_right_turn_waypoint(lookahead_wp)
                        v2.set_light_state(
                            carla.VehicleLightState(LIGHT_POSITION | LIGHT_LOW_BEAM | LIGHT_BLINKER_RIGHT))
                    else:
                        target_wp = next_options[0] if next_options else lookahead_wp

                    v2_wp = carla_map.get_waypoint(v2.get_location())
                    apply_pid_control(v2, pids['v2']['lon'], pids['v2']['lat'], 25.0, target_wp.transform.location)

            # ==========================
            # V3 控制: 绝对直行 + 闪灯 + 【出界销毁】
            # ==========================
            if active_vehicles['v3'] and v3.is_alive:
                target_wp = get_straightest_waypoint(v3_wp, distance=8.0)
                if not target_wp:
                    print("V3 驶出路网，自动销毁释放资源")
                    v3.destroy()
                    active_vehicles['v3'] = False
                else:
                    v3_wp = carla_map.get_waypoint(v3.get_location())
                    apply_pid_control(v3, pids['v3']['lon'], pids['v3']['lat'], 30.0, target_wp.transform.location)

                    # 闪灯逻辑
                    cycle = sim_time % 2.0
                    is_flashing = True if (cycle < 0.2 or 0.4 <= cycle < 0.6 or 0.8 <= cycle < 1.0) else False
                    if is_flashing:
                        v3.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_HIGH_BEAM))
                    else:
                        v3.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_LOW_BEAM))

            # ==========================
            # Ego control: state-machine speed script + fixed trajectory tracking
            # ==========================
            if active_vehicles['ego'] and ego.is_alive:
                ego_loc = ego.get_location()
                current_speed = speed_kmh(ego)
                target_speed, hold_brake = ego_speed_sm.tick(ego_loc)

                if hold_brake:
                    ego.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_HIGH_BEAM))
                    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0, hand_brake=False))
                else:
                    target_wp_loc, ego_route_idx = get_target_waypoint(
                        ego_loc,
                        ego_path,
                        ego_route_idx,
                        max(current_speed, target_speed),
                    )
                    ego.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_HIGH_BEAM))
                    if target_wp_loc:
                        apply_pid_control(ego, pids['ego']['lon'], pids['ego']['lat'], target_speed, target_wp_loc)
                    else:
                        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0, hand_brake=False))

            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()
        print("清理完毕。")

if __name__ == '__main__':
    main()
