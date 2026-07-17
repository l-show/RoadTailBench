# -*- coding: utf-8 -*-

import carla
import time
import math
import numpy as np
import random


# ==========================================
# 轨迹数据清洗 (自动去重)
# ==========================================
def clean_path_points(raw_points):
    cleaned_points = []
    if raw_points:
        cleaned_points.append(raw_points[0])
        for i in range(1, len(raw_points)):
            if raw_points[i] != raw_points[i - 1]:
                cleaned_points.append(raw_points[i])
    return cleaned_points


# 蓝车 (Nissan/Tesla) 轨迹数据
RAW_VEHICLE_PATH_POINTS = [
    (-23.595, 52.2, -89.396), (-23.595, 47.2, -89.096), (-23.595, 41, -94.096),
    (-23.595, 37, -94.096), (-23.595, 35, -94.166), (-23.595, 34.2, -94.339),
    (-23.861, 30.619, -93.826), (-24.094, 26.773, -92.652), (-24.234, 22.964, -91.925),
    (-24.363, 19.108, -91.558), (-24.429, 15.196, -90.109), (-24.424, 11.391, -89.872),
    (-24.343, 7.504, -87.562), (-24.101, 3.688, -85.596), (-23.711, -0.089, -80.814),
    (-22.989, -3.799, -78.13), (-22.136, -7.544, -74.397), (-21.118, -11.173, -74.327),
    (-20.075, -14.828, -73.327), (-18.929, -18.442, -71.683), (-17.423, -22.063, -61.694),
    (-15.515, -25.403, -60.861), (-13.752, -28.779, -63.729), (-12.018, -32.277, -63.008),
    (-10.252, -35.606, -60.477), (-8.232, -38.899, -57.583), (-6.121, -42.164, -55.897),
    (-3.834, -45.271, -52.278), (-1.462, -48.201, -50.749), (1.192, -51.064, -44.477),
    (3.933, -53.743, -43.158), (6.832, -56.316, -39.974), (9.749, -58.582, -36.317),
    (12.883, -60.707, -31.933), (16.175, -62.679, -29.216), (19.59, -64.437, -26.253),
    (23.074, -66.12, -25.071), (26.531, -67.711, -24.467), (29.935, -69.229, -23.54),
    (33.486, -70.707, -21.49), (37.073, -72.073, -19.433), (40.756, -73.319, -17.164),
    (44.466, -74.321, -13.442), (48.27, -75.125, -11.051), (51.909, -75.909, -14.959),
    (55.627, -77.08, -21.985), (58.993, -78.838, -33.343), (61.97, -81.177, -41.304),
    (64.633, -83.835, -48.421), (66.96, -86.87, -55.11), (69.103, -89.947, -55.809),
    (70.894, -93.377, -66.295), (72.347, -96.969, -68.358), (73.776, -100.571, -68.358),
    (75.101, -104.145, -71.47), (76.226, -107.787, -74.617), (77.245, -111.525, -74.756),
    (78.237, -115.206, -75.595), (79.118, -118.915, -78.008), (79.887, -122.649, -79.267),
    (80.603, -126.459, -79.408), (81.293, -130.146, -79.408), (81.983, -133.833, -79.408),
    (82.626, -137.275, -79.408), (82.626, -137.275, -79.408), (82.626, -137.275, -79.408),
    (82.626, -137.275, -79.408), (82.626, -137.275, -79.408)
]
VEHICLE_PATH_POINTS = clean_path_points(RAW_VEHICLE_PATH_POINTS)

RAW_EGO_PATH_POINTS = [
    (-23.859, 25.471, -89.768), (-23.859, 25.471, -89.768), (-23.859, 25.471, -89.768),
    (-23.859, 25.471, -89.768), (-23.859, 25.471, -89.768), (-23.859, 25.471, -89.768),
    (-23.859, 25.471, -89.768), (-23.859, 25.471, -89.768), (-23.859, 25.471, -89.349),
    (-23.834, 23.300, -89.349), (-23.816, 19.474, -90.589), (-23.874, 15.684, -91.009),
    (-23.941, 11.886, -91.009), (-23.992, 8.161, -90.449), (-23.967, 4.322, -88.211),
    (-23.813, 0.503, -86.656), (-23.436, -3.287, -80.761), (-22.606, -7.046, -76.016),
    (-21.655, -10.674, -74.516), (-20.550, -14.367, -72.595), (-19.337, -17.950, -69.817),
    (-17.869, -21.519, -63.790), (-16.174, -24.915, -62.945), (-14.460, -28.270, -62.945),
    (-12.673, -31.720, -62.035), (-10.838, -35.004, -60.633), (-8.936, -38.339, -59.588),
    (-6.973, -41.626, -57.828), (-4.872, -44.767, -54.640), (-2.640, -47.819, -52.008),
    (-0.156, -50.778, -47.568), (2.484, -53.492, -44.421), (5.251, -56.080, -42.301),
    (8.125, -58.563, -39.226), (11.128, -60.886, -35.951), (14.338, -63.018, -31.211),
    (17.686, -64.944, -28.450), (21.073, -66.615, -24.897), (24.528, -68.219, -24.897),
    (28.039, -69.774, -23.545), (31.346, -71.215, -23.545), (35.062, -72.751, -21.263),
    (38.607, -74.101, -19.894), (42.219, -75.265, -16.469), (45.862, -76.233, -13.047),
    (49.626, -76.902, -8.568), (53.414, -77.526, -12.470), (56.978, -78.680, -25.558),
    (60.193, -80.813, -38.604), (63.040, -83.312, -45.389), (65.461, -86.160, -53.056),
    (67.608, -89.254, -56.173), (69.522, -92.493, -61.806), (71.288, -95.880, -63.314),
    (72.857, -99.285, -67.636), (74.217, -102.846, -70.870), (75.401, -106.470, -72.884),
    (76.425, -110.143, -75.231), (77.339, -113.843, -77.397), (78.124, -117.509, -77.674),
    (78.937, -121.233, -77.674), (79.787, -124.950, -77.079), (80.717, -128.649, -74.885),
    (81.698, -132.332, -75.630), (82.267, -134.573, -75.758), (82.267, -134.573, -75.758),
    (82.267, -134.573, -75.758), (82.267, -134.573, -75.758), (82.267, -134.573, -75.758)
]
EGO_PATH_POINTS = clean_path_points(RAW_EGO_PATH_POINTS)

# 行人随机漫游坐标点
PEDESTRIAN_LOCATIONS = [
    carla.Location(x=-19.568, y=-27.066, z=1.0),
    carla.Location(x=-17.682, y=-30.062, z=1.0),
    carla.Location(x=-16.603, y=-34.192, z=1.0),
    carla.Location(x=-20.078, y=-32.118, z=1.0)
]


# ==========================================
# PID 控制器类
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController2:
    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint, vehicle_transform):
        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint[0] - v_begin.x, waypoint[1] - v_begin.y, 0.0])
        norm_w = np.linalg.norm(w_vec)
        if norm_w < 0.1:
            return 0.0
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0
        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


# ==========================================
# 辅助函数
# ==========================================
def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )


def get_target_waypoint(actor_loc, path_points, lookahead_dist=4.0):
    min_dist = float('inf')
    closest_index = 0
    for i, p in enumerate(path_points):
        dist = math.sqrt((p[0] - actor_loc.x) ** 2 + (p[1] - actor_loc.y) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_index = i

    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1 = path_points[i]
        p2 = path_points[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist:
            break
    return path_points[target_index]


def clamp(v, a, b):
    return max(a, min(b, v))


class BlueVehicleSpeedStateMachine:
    CRUISE_60 = "CRUISE_60"
    HOLD_40 = "HOLD_40"

    def __init__(self, initial_speed_kmh=60.0, hold_speed_kmh=40.0, trigger_y=-5.0):
        self.initial_speed_kmh = initial_speed_kmh
        self.hold_speed_kmh = hold_speed_kmh
        self.trigger_y = trigger_y
        self.state = self.CRUISE_60

    def tick(self, location):
        if self.state == self.CRUISE_60 and location.y <= self.trigger_y:
            self.state = self.HOLD_40
            print("[RoadTailBench] Blue tesla.model3 reached y=-5; state -> HOLD_40.")
        if self.state == self.HOLD_40:
            return self.hold_speed_kmh
        return self.initial_speed_kmh


# 【新增】出界判定及销毁函数
def check_and_handle_out_of_bounds(actor, carla_map, threshold=6.0):
    """
    检查车辆是否垂直投影脱离了道路，如果超过距离阈值(如6米)则直接销毁该 actor。
    """
    if actor is None or not actor.is_alive:
        return True

    loc = actor.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)

    # 获取不到投影路点直接销毁
    if wp_nearest is None:
        print(f"[{actor.type_id} {actor.attributes.get('role_name', 'None')}] 无法投影到道路，判定出界被销毁！")
        actor.destroy()
        return True

    distance = wp_nearest.transform.location.distance(loc)
    # 大于允许的出轨阈值时销毁
    if distance > threshold:
        print(
            f"[{actor.type_id} {actor.attributes.get('role_name', 'None')}] 偏离道路中心 {distance:.2f} 米，判定出界被销毁！")
        actor.destroy()
        return True

    return False


# ==========================================
# 主程序
# ==========================================


# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = EGO_PATH_POINTS[-1][:2]
_RTB_OPT_EGO_TYPE_ID = 'vehicle.audi.tt'
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
    ego_actor = local_vars.get('orange_audi') or _rtb_opt_find_ego(local_vars)
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


def _rtb_ego_destroyed(local_vars):
    return not _rtb_opt_is_alive(local_vars.get('orange_audi'))
# === End RoadTailBench Opt guard ===

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # 严格保持天气参数
    weather = carla.WeatherParameters(
        cloudiness=15.0, precipitation=100.0, precipitation_deposits=100.0,
        wind_intensity=10.0, sun_azimuth_angle=85.0, sun_altitude_angle=-90.0,
        fog_density=15.0, fog_distance=5.0, fog_falloff=0.0, wetness=60.0,
        scattering_intensity=8.0, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.10,
        dust_storm=0.0
    )
    world.set_weather(weather)
    print("天气参数已更新。")

    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        # 交通管理器 (Traffic Manager) 设置
        tm_port = 8000
        tm = client.get_trafficmanager(tm_port)
        tm.set_synchronous_mode(True)
        tm.set_hybrid_physics_mode(True)

        # ---------------------------------------------------------
        # 1. 生成 蓝车 (tesla.model3)
        # ---------------------------------------------------------
        bp_vehicle = bp_lib.find('vehicle.tesla.model3')
        bp_vehicle.set_attribute('color', '0,0,255')
        initial_vehicle_point = VEHICLE_PATH_POINTS[0]
        trans_vehicle = get_transform(x=initial_vehicle_point[0], y=initial_vehicle_point[1], z=1.0,
                                      yaw=initial_vehicle_point[2])
        vehicle = world.try_spawn_actor(bp_vehicle, trans_vehicle)
        if vehicle:
            actor_list.append(vehicle)
            vehicle.set_simulate_physics(True)
            print("tesla.model3 生成成功 (跟随蓝车)")

        lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0, dt=settings.fixed_delta_seconds)
        lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2, dt=settings.fixed_delta_seconds)

        # ---------------------------------------------------------
        # 2. 生成 Agent 自动驾驶车辆 (红色的Audi)
        # ---------------------------------------------------------
        bp_agent_car = bp_lib.find('vehicle.audi.tt')
        bp_agent_car.set_attribute('color', '255,0,0')

        agent_spawn_loc = carla.Location(x=-27.166, y=40.632, z=0.0)
        agent_spawn_wp = carla_map.get_waypoint(agent_spawn_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

        agent_spawn_transform = agent_spawn_wp.transform
        agent_spawn_transform.location.z += 1.0

        agent_vehicle = world.try_spawn_actor(bp_agent_car, agent_spawn_transform)
        if agent_vehicle:
            actor_list.append(agent_vehicle)
            agent_vehicle.set_simulate_physics(True)
            agent_vehicle.set_autopilot(True, tm_port)

            tm.auto_lane_change(agent_vehicle, False)
            tm.vehicle_percentage_speed_difference(agent_vehicle, -56.25)

            # 【关键防御】彻底禁止 TM 控制 Agent 车辆的灯光
            tm.ignore_lights_percentage(agent_vehicle, 100.0)
            try:
                tm.update_vehicle_lights(agent_vehicle, False)
            except Exception:
                pass
            print("Agent 车辆 (自动保持车道红车) 生成成功")

        # ---------------------------------------------------------
        # 3. [优化点] 生成橙色 Audi TT，设定为主控 Ego
        # ---------------------------------------------------------
        bp_orange_audi = bp_lib.find('vehicle.audi.tt')
        bp_orange_audi.set_attribute('color', '255,128,0')  # 橙色
        bp_orange_audi.set_attribute('role_name', 'ego')  # 【关键修改】设置actor名为ego

        # 目标位置，z轴设为0让API自动去贴近地面寻找
        initial_ego_point = EGO_PATH_POINTS[0]
        orange_audi_loc = carla.Location(x=initial_ego_point[0], y=initial_ego_point[1], z=20.0)
        # 自动获取道路锚点 (project_to_road=True 会把坐标映射到合法的道路中心或车道上)
        orange_audi_wp = carla_map.get_waypoint(orange_audi_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

        orange_audi_transform = orange_audi_wp.transform
        orange_audi_transform.rotation.yaw = initial_ego_point[2]
        orange_audi_transform.location.z += 0.5  # 略微抬高避免碰撞地面

        orange_audi = world.try_spawn_actor(bp_orange_audi, orange_audi_transform)
        if orange_audi:
            actor_list.append(orange_audi)
            orange_audi.set_simulate_physics(True)
            print(f"EGO 橙色 Audi TT 生成成功，吸附位置: {orange_audi_transform.location}")

            # 为 Ego 橙色 Audi 初始化独立的PID控制器
            orange_lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0,
                                                              dt=settings.fixed_delta_seconds)
            orange_lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2, dt=settings.fixed_delta_seconds)

        # ---------------------------------------------------------
        # 4. 生成 行人
        # ---------------------------------------------------------
        walker_bps = bp_lib.filter('walker.pedestrian.*')
        bp_walker = random.choice(walker_bps)

        initial_ped_loc = random.choice(PEDESTRIAN_LOCATIONS)
        trans_walker = get_transform(x=initial_ped_loc.x, y=initial_ped_loc.y, z=initial_ped_loc.z, yaw=0.0)
        walker = world.try_spawn_actor(bp_walker, trans_walker)
        if walker:
            actor_list.append(walker)
            print("行人 生成成功")

        # ==========================================
        # 等待物理稳定后，一次性设定车灯
        # ==========================================
        print("等待物理系统初始化...")
        for _ in range(5):
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            time.sleep(settings.fixed_delta_seconds)

        # 物理稳定后，强制一次性打开远光灯+位置灯
        base_light_state = carla.VehicleLightState.Position | carla.VehicleLightState.HighBeam
        if vehicle and vehicle.is_alive:
            vehicle.set_light_state(carla.VehicleLightState(base_light_state))
        if agent_vehicle and agent_vehicle.is_alive:
            agent_vehicle.set_light_state(carla.VehicleLightState(base_light_state))
        if orange_audi and orange_audi.is_alive:
            orange_audi.set_light_state(carla.VehicleLightState(base_light_state))

        print("\n=> 物理系统稳定，已下发车灯常亮指令！场景运行中...")

        # --- 车辆参数 ---
        blue_speed_sm = BlueVehicleSpeedStateMachine(
            initial_speed_kmh=52.0,
            hold_speed_kmh=40.0,
            trigger_y=-5.0
        )
        if vehicle and vehicle.is_alive:
            yaw_rad = math.radians(initial_vehicle_point[2])
            speed_ms = blue_speed_sm.initial_speed_kmh / 3.6
            vehicle.set_target_velocity(carla.Vector3D(
                x=speed_ms * math.cos(yaw_rad),
                y=speed_ms * math.sin(yaw_rad),
                z=0.0
            ))
            print(f"[RoadTailBench] Blue tesla.model3 initial speed set to {blue_speed_sm.initial_speed_kmh:.1f} km/h.")

        # 行人控制参数
        PED_SPEED_MPS = 5.0 / 3.6  # 5 km/h
        PED_ARRIVAL_DIST = 0.8
        current_ped_target = random.choice([loc for loc in PEDESTRIAN_LOCATIONS if loc != initial_ped_loc])
        ped_last_pos = walker.get_location() if walker else None
        ped_last_progress_time = time.time()

        # 主循环
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            if _rtb_ego_destroyed(locals()):
                print("[RoadTailBench] RTB004check ego orange Audi destroyed; ending simulation.")
                break

            # ==============================
            # 1. 蓝车 (tesla.model3) PID 控制逻辑
            # ==============================
            if vehicle and vehicle.is_alive:
                # 【新增出界判定】
                if check_and_handle_out_of_bounds(vehicle, carla_map):
                    vehicle = None  # 防止报错，标记已销毁
                else:
                    tf = vehicle.get_transform()
                    vel = vehicle.get_velocity()
                    current_vehicle_speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                    target_vehicle_speed = blue_speed_sm.tick(tf.location)

                    target_wp = get_target_waypoint(tf.location, VEHICLE_PATH_POINTS, lookahead_dist=5.0)
                    throttle_output = lon_controller.run_step(target_vehicle_speed, current_vehicle_speed)
                    steer_output = lat_controller.run_step(target_wp, tf)

                    control = carla.VehicleControl()
                    control.steer = steer_output
                    if throttle_output >= 0.0:
                        control.throttle = throttle_output
                        control.brake = 0.0
                    else:
                        control.throttle = 0.0
                        control.brake = abs(throttle_output)

                    vehicle.apply_control(control)

            # ==============================
            # 2. TM 自动驾驶车辆 (红色 Audi)出界判定
            # ==============================
            if agent_vehicle and agent_vehicle.is_alive:
                if check_and_handle_out_of_bounds(agent_vehicle, carla_map):
                    agent_vehicle = None

            # ==============================
            # 3. Ego 橙色 Audi TT 控制逻辑
            # ==============================
            if orange_audi and orange_audi.is_alive:
                # 【新增出界判定】
                if check_and_handle_out_of_bounds(orange_audi, carla_map):
                    orange_audi = None
                    print("[RoadTailBench] RTB004check ego orange Audi destroyed; ending simulation.")
                    break
                else:
                    o_tf = orange_audi.get_transform()
                    o_vel = orange_audi.get_velocity()
                    o_current_speed = 3.6 * math.sqrt(o_vel.x ** 2 + o_vel.y ** 2 + o_vel.z ** 2)

                    # 速度策略：初始70km/h，在y=-5减速
                    if o_tf.location.y > -5.0:
                        o_target_speed = 70.0
                    elif -30.0 < o_tf.location.y <= -5.0:
                        o_target_speed = 40.0
                    else:  # y <= -30.0
                        o_target_speed = 80.0

                    o_target_wp = get_target_waypoint(o_tf.location, EGO_PATH_POINTS, lookahead_dist=4.0)
                    o_steer_output = orange_lat_controller.run_step(o_target_wp, o_tf)

                    # 纵向PID控制计算
                    o_throttle_output = orange_lon_controller.run_step(o_target_speed, o_current_speed)

                    o_control = carla.VehicleControl()
                    o_control.steer = o_steer_output
                    if o_throttle_output >= 0.0:
                        o_control.throttle = o_throttle_output
                        o_control.brake = 0.0
                    else:
                        o_control.throttle = 0.0
                        o_control.brake = abs(o_throttle_output)

                    orange_audi.apply_control(o_control)

            # ==============================
            # 4. 行人随机漫游控制逻辑
            # ==============================
            if walker and walker.is_alive:
                ped_loc = walker.get_location()
                dx = current_ped_target.x - ped_loc.x
                dy = current_ped_target.y - ped_loc.y
                dist = math.hypot(dx, dy)

                if dist <= PED_ARRIVAL_DIST:
                    available_targets = [loc for loc in PEDESTRIAN_LOCATIONS if loc != current_ped_target]
                    current_ped_target = random.choice(available_targets)
                    ped_last_progress_time = time.time()
                    continue

                ctrl = carla.WalkerControl()
                if dist > 1e-6:
                    nx, ny = dx / dist, dy / dist
                    ctrl.direction = carla.Vector3D(nx, ny, 0.0)
                    slow_radius = 2.0
                    if dist < slow_radius:
                        desired_speed = PED_SPEED_MPS * (dist / slow_radius)
                        ctrl.speed = clamp(desired_speed, 0.25, PED_SPEED_MPS)
                    else:
                        ctrl.speed = PED_SPEED_MPS
                else:
                    ctrl.speed = 0.0
                    ctrl.direction = carla.Vector3D(0.0, 0.0, 0.0)

                walker.apply_control(ctrl)

                # 防卡死检测
                current_time = time.time()
                if math.hypot(ped_loc.x - ped_last_pos.x, ped_loc.y - ped_last_pos.y) > 0.05:
                    ped_last_progress_time = current_time
                    ped_last_pos = ped_loc

                if current_time - ped_last_progress_time > 8.0:
                    available_targets = [loc for loc in PEDESTRIAN_LOCATIONS if loc != current_ped_target]
                    current_ped_target = random.choice(available_targets)
                    ped_last_progress_time = current_time

            # ==============================
            # 严格保留的时间同步逻辑
            # ==============================
            compute_time = time.time() - start_time
            if compute_time < settings.fixed_delta_seconds:
                time.sleep(settings.fixed_delta_seconds - compute_time)

    except Exception as e:
        print(f"发生异常: {e}")
    except KeyboardInterrupt:
        print("\n用户停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(False)

        # 【优化】判断 actor 是否存活再释放，防止释放已经被出界函数销毁的实体报错
        if actor_list:
            actors_to_destroy = [a for a in actor_list if a is not None and a.is_alive]
            client.apply_batch([carla.command.DestroyActor(a) for a in actors_to_destroy])

        print("清理完成，Carla 已恢复正常。")


if __name__ == '__main__':
    main()
