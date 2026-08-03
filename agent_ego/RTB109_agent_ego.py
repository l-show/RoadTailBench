# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

场景主题：
城市主干道交叉口前右变道超车 + 高出路面检修井室限界侵入
+ 行人群体遮挡 + 三辆两轮车干扰 + 新增动态车辆轨迹交互

本版修改：
1. 三辆自行车/电动车轨迹替换为用户最新轨迹。
2. 新增一辆动态车辆，沿用户提供轨迹行驶。
3. 保留 Ego、慢速前车、8名行人。
4. 严格同步模式。
5. 所有运动车辆独立 PID。
"""

import sys
import os
import glob
import time
import math
import random
import traceback

# ============================================================
# 0. 动态引入 CARLA PythonAPI 与标准化函数库
# ============================================================

LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

CARLA_ROOT = r"D:\Carla"

CARLA_API_PATH = os.path.join(CARLA_ROOT, "PythonAPI", "carla")
if os.path.isdir(CARLA_API_PATH) and CARLA_API_PATH not in sys.path:
    sys.path.append(CARLA_API_PATH)

egg_pattern = os.path.join(
    CARLA_ROOT,
    "PythonAPI",
    "carla",
    "dist",
    "carla-*%d.%d-%s.egg" % (
        sys.version_info.major,
        sys.version_info.minor,
        "win-amd64" if os.name == "nt" else "linux-x86_64"
    )
)

for egg_path in glob.glob(egg_pattern):
    if egg_path not in sys.path:
        sys.path.append(egg_path)

import carla
import RoadTailBenchInitV9 as RTB

# ============================================================
# 1. 全局可调接口：仿真配置
# ============================================================

HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

DT = 0.05
SCENARIO_DURATION = 60.0
KEEP_ACTORS_AFTER_SCRIPT = False

ROUTE_RESOLUTION_M = 2.0
TRAJ_INTERVAL_M = 0.8

USE_LINEAR_FALLBACK_WHEN_ROUTE_FAIL = True

# ============================================================
# 2. 全局可调接口：车辆速度与状态机
# ============================================================

UTURN_INITIAL_SPEED_KMH = 60.0
UTURN_SLOW_SPEED_KMH = 30.0
UTURN_TURN_SPEED_KMH = 5.0
UTURN_RESTORE_SPEED_KMH = 60.0
UTURN_MAX_SPEED_KMH = 70.0
UTURN_SLOW_TRIGGER_X = 52.0
UTURN_TURN_TRIGGER_X = 14.0
UTURN_TURN_DURATION_S = 4.0

STRAIGHT_SPEED_KMH = 25.0
STRAIGHT_MAX_SPEED_KMH = 35.0

EGO_INITIAL_SPEED_KMH = 60.0
EGO_SLOW_SPEED_KMH = 30.0
EGO_TURN_SPEED_KMH = 20.0
EGO_RESTORE_SPEED_KMH = 60.0
EGO_MAX_SPEED_KMH = 70.0
EGO_SLOW_TRIGGER_X = 100.0
EGO_TURN_TRIGGER_X = 60.0
EGO_RESTORE_TRIGGER_X = -35.0

# ============================================================
# 3. 天气参数：ClearSunset
# ============================================================

cloudiness = 5.0
precipitation = 0.0
precipitation_deposits = 0.0
wind_intensity = 10.0
sun_azimuth_angle = 0.0
sun_altitude_angle = 9.0
fog_density = 2.0
fog_distance = 1.750
fog_falloff = 0.100
wetness = 0.0
scattering_intensity = 2.500
mie_scattering_scale = 0.030
rayleigh_scattering_scale = 0.1331
dust_storm = 0.0

# ============================================================
# 4. 车辆蓝图候选
# ============================================================

UTURN_BP_CANDIDATES = [
    "vehicle.bmw.grandtourer",
]

STRAIGHT_BP_CANDIDATES = [
    "vehicle.mercedes.sprinter",
]

EGO_BP_CANDIDATES = [
    "vehicle.audi.tt",
]

# ============================================================
# 5. 掉头背景车轨迹，格式为 (x, y, yaw)
# ============================================================

UTURN_RAW_TRAJECTORY = [
    (124.174, -8.806, 173.654), (124.174, -8.806, 173.654), (124.174, -8.806, 173.654), (123.444, -8.725, 173.654),
    (119.665, -8.404, 176.253), (115.727, -8.092, 174.944), (111.967, -7.705, 173.209), (108.232, -7.187, 171.146),
    (104.554, -6.569, 170.210), (100.844, -5.926, 170.000), (97.042, -5.240, 169.748), (93.477, -4.637, 171.735),
    (89.610, -4.139, 174.234), (85.945, -3.825, 175.291), (82.030, -3.502, 175.291), (78.270, -3.257, 178.054),
    (74.619, -3.206, 179.431), (70.645, -3.164, 179.221), (66.904, -3.105, 178.801), (63.179, -3.016, 178.591),
    (59.335, -2.921, 178.591), (55.568, -2.828, 178.591), (52.574, -2.755, 178.591), (52.574, -2.755, 178.591),
    (52.574, -2.755, 178.591), (52.574, -2.755, 178.591), (52.574, -2.755, 178.591), (52.574, -2.755, 178.591),
    (52.574, -2.755, 178.591), (52.574, -2.755, 178.591), (52.574, -2.755, 178.591), (52.574, -2.755, 178.591),
    (50.528, -2.738, -179.417), (46.738, -2.835, -178.377), (44.348, -2.903, -178.377), (43.133, -2.937, -178.377),
    (41.811, -2.974, -178.377), (40.589, -3.005, -178.673), (39.300, -3.032, -178.813), (38.090, -3.057, -178.813),
    (36.818, -3.083, -178.813), (35.545, -3.110, -178.813), (34.277, -3.136, -178.813), (33.003, -3.157, -179.488),
    (31.803, -3.168, -179.488), (30.517, -3.179, -179.488), (29.243, -3.191, -179.488), (27.966, -3.198, -179.993),
    (26.669, -3.194, 179.797), (25.458, -3.190, 179.797), (24.217, -3.185, 179.797), (22.894, -3.181, 179.797),
    (21.630, -3.176, 179.797), (20.382, -3.172, 179.797), (19.141, -3.167, 179.797), (17.919, -3.163, 179.797),
    (16.599, -3.158, 179.797), (15.381, -3.154, 179.797), (14.067, -3.150, 179.797), (14.067, -3.150, 179.797),
    (14.067, -3.150, 179.797), (14.067, -3.150, 179.797), (14.067, -3.150, 179.797), (14.067, -3.150, 179.797),
    (14.067, -3.150, 179.797), (14.067, -3.150, 179.797), (14.067, -3.150, 179.797), (13.452, -3.147, 179.797),
    (12.139, -3.143, 179.797), (10.915, -3.138, 179.797), (10.067, -3.135, 179.797), (9.572, -3.134, 179.797),
    (9.046, -3.132, 179.797), (8.551, -3.130, 179.797), (8.056, -3.128, 179.797), (7.561, -3.127, 179.797),
    (7.029, -3.125, 179.797), (6.531, -3.123, 179.797), (6.035, -3.121, 179.797), (5.536, -3.102, 174.656),
    (5.040, -3.043, 168.742), (4.553, -2.855, 149.747), (4.141, -2.589, 143.343), (3.752, -2.237, 134.050),
    (3.435, -1.857, 126.981), (3.163, -1.435, 117.771), (2.951, -0.962, 113.140), (2.796, -0.484, 100.253),
    (2.731, 0.002, 93.089), (2.819, 0.497, 76.952), (2.936, 0.986, 73.620), (3.127, 1.461, 63.516),
    (3.369, 1.897, 56.535), (3.681, 2.300, 47.376), (4.062, 2.625, 34.047), (4.513, 2.869, 17.941),
    (5.016, 2.952, 4.177), (5.518, 2.988, 3.279), (6.015, 2.992, -2.745), (6.541, 2.964, -3.070),
    (7.040, 2.937, -3.070), (7.539, 2.911, -3.070), (8.035, 2.882, -4.103), (8.531, 2.846, -4.173),
    (9.999, 2.739, -4.173), (13.640, 2.473, -4.173), (17.455, 2.262, -0.040), (21.096, 2.368, 1.867),
    (24.954, 2.494, 1.867), (28.704, 2.589, 1.272), (32.652, 2.669, 0.596), (36.315, 2.687, -0.196),
    (40.282, 2.648, -0.661), (43.859, 2.606, -0.521), (47.702, 2.579, -0.311), (51.507, 2.559, -0.311),
    (54.652, 2.541, -0.311), (54.652, 2.541, -0.311), (54.652, 2.541, -0.311), (54.652, 2.541, -0.311),
]

# ============================================================
# 6. 直行车轨迹，格式为 (x, y, yaw)
# ============================================================

STRAIGHT_RAW_TRAJECTORY = [
    (102.276, -9.050, 171.880), (101.338, -8.916, 171.880), (100.100, -8.739, 171.880), (97.955, -8.433, 171.880),
    (95.447, -8.091, 172.875), (92.918, -7.812, 173.868), (90.349, -7.571, 175.851), (87.575, -7.369, 175.851),
    (83.015, -7.040, 176.546), (77.965, -6.814, 178.126), (72.670, -6.641, 178.126), (67.663, -6.485, 178.984),
    (62.629, -6.426, 179.389), (57.496, -6.371, 179.389), (52.400, -6.317, 179.389), (47.297, -6.262, 179.389),
    (42.467, -6.211, 179.389), (37.264, -6.159, 179.459), (32.146, -6.120, 179.669), (27.326, -6.104, -179.981),
    (22.107, -6.109, -179.911), (17.056, -6.152, -179.246), (12.074, -6.227, -179.036), (7.071, -6.311, -179.036),
    (2.022, -6.396, -179.036), (-3.020, -6.481, -179.036), (-8.391, -6.563, -179.599), (-13.519, -6.573, 179.865),
    (-18.293, -6.562, 179.865), (-23.494, -6.550, 179.865), (-28.522, -6.538, 179.865), (-33.721, -6.525, 179.865),
    (-38.587, -6.514, 179.865), (-43.961, -6.501, 179.865), (-48.824, -6.490, 179.865), (-54.042, -6.489, -179.925),
    (-58.934, -6.501, -178.482), (-63.923, -6.816, -174.282), (-69.206, -7.426, -172.716), (-74.273, -8.074, -172.786),
    (-79.120, -8.646, -173.753), (-83.994, -9.145, -175.498), (-89.135, -9.434, -177.895), (-94.301, -9.592, -178.617),
    (-99.121, -9.695, -178.827), (-105.687, -9.829, -178.827), (-113.370, -9.932, -179.769), (-120.584, -9.950, -179.979),
    (-128.377, -9.953, -179.979), (-135.996, -9.955, -179.979), (-143.654, -9.947, 179.671), (-151.273, -9.902, 179.600),
    (-158.796, -9.841, 179.460), (-166.443, -9.769, 179.460), (-173.843, -9.726, 179.692), (-181.595, -9.708, -179.686),
    (-188.881, -9.757, -179.476), (-196.649, -9.835, -179.162), (-200.206, -9.887, -179.162), (-200.206, -9.887, -179.162),
    (-200.206, -9.887, -179.162), (-200.206, -9.887, -179.162),
]

# ============================================================
# 7. Ego 轨迹，格式为 (x, y, yaw)
# ============================================================

EGO_RAW_TRAJECTORY = [
    (141.298, -17.067, 164.434), (141.298, -17.067, 164.434), (141.001, -16.984, 164.223), (136.282, -15.617, 163.764),
    (131.270, -14.251, 165.478), (126.560, -13.065, 167.045), (121.377, -11.944, 168.835), (116.458, -11.039, 170.022),
    (111.590, -10.205, 170.664), (106.670, -9.446, 171.561), (101.657, -8.748, 173.404), (96.613, -8.308, 176.539),
    (91.547, -8.173, -179.111), (86.390, -8.420, -176.173), (81.525, -8.708, -178.076), (76.540, -8.537, 173.677),
    (71.378, -7.671, 169.021), (70.733, -7.545, 169.021), (70.733, -7.545, 169.021), (70.733, -7.545, 169.021),
    (70.733, -7.545, 169.021), (70.733, -7.545, 169.021), (70.733, -7.545, 169.021), (70.733, -7.545, 169.021),
    (70.733, -7.545, 169.021), (67.211, -6.920, 170.694), (62.306, -6.358, 174.880), (57.275, -5.965, 176.974),
    (52.239, -5.764, 178.059), (47.202, -5.667, 179.542), (42.092, -5.631, 179.612), (36.899, -5.617, -179.617),
    (31.938, -5.686, -179.027), (26.860, -5.772, -179.027), (21.998, -5.854, -179.027), (16.755, -5.958, -178.607),
    (11.881, -6.086, -178.467), (6.622, -6.227, -178.467), (1.648, -6.346, -179.069), (-3.318, -6.305, 178.881),
    (-8.614, -6.241, -179.833), (-13.619, -6.255, -179.833), (-18.724, -6.056, 174.854), (-23.404, -5.425, 170.479),
    (-28.649, -4.489, 169.857), (-33.659, -3.851, 175.795), (-38.480, -3.562, 177.155), (-43.763, -3.356, 178.271),
    (-48.539, -3.259, 179.702), (-53.634, -3.237, 179.772), (-58.850, -3.217, 179.772), (-63.781, -3.197, 179.772),
    (-69.012, -3.176, 179.772), (-74.098, -3.156, 179.772), (-78.926, -3.137, 179.702), (-84.153, -3.109, 179.702),
    (-89.278, -3.083, 179.702), (-94.356, -3.056, 179.772), (-97.604, -3.053, 179.971), (-97.604, -3.053, 179.971),
    (-97.604, -3.053, 179.971), (-97.604, -3.053, 179.971),
]

# ============================================================
# 7. 行人参数
# ============================================================

ENABLE_PEDESTRIANS = True
PEDESTRIAN_COUNT = 8

PEDESTRIAN_START_TF = carla.Transform(
    carla.Location(x=57.248, y=-13.075, z=1.132),
    carla.Rotation(pitch=0.000, yaw=-173.388, roll=0.000)
)

PEDESTRIAN_TARGET_TF = carla.Transform(
    carla.Location(x=25.334, y=-13.269, z=0.949),
    carla.Rotation(pitch=-0.195, yaw=174.176, roll=0.000)
)

PEDESTRIAN_START_DELAY_S = 0.0
PEDESTRIAN_WALK_SPEED_MPS = 1.25
PEDESTRIAN_STOP_DISTANCE_M = 0.45
PEDESTRIAN_SPAWN_Z_OFFSET = 0.20

PEDESTRIAN_LONGITUDINAL_SPACING_M = 0.95
PEDESTRIAN_LATERAL_SPACING_M = 0.75
PEDESTRIAN_MIN_DISTANCE_M = 0.60

# ============================================================
# 10. 基础工具函数
# ============================================================

def validate_user_inputs():
    if PEDESTRIAN_COUNT != 8:
        raise RuntimeError("本场景要求 PEDESTRIAN_COUNT = 8。")
    if len(UTURN_RAW_TRAJECTORY) < 2:
        raise RuntimeError("UTURN_RAW_TRAJECTORY 锚点不足。")
    if len(STRAIGHT_RAW_TRAJECTORY) < 2:
        raise RuntimeError("STRAIGHT_RAW_TRAJECTORY 锚点不足。")
    if len(EGO_RAW_TRAJECTORY) < 2:
        raise RuntimeError("EGO_RAW_TRAJECTORY 锚点不足。")

def enable_sync(world, dt=0.05):
    RTB.enable_synchronous_mode(world, dt=dt)

def disable_sync(world):
    RTB.disable_synchronous_mode(world)

def print_world_sync_state(world):
    settings = world.get_settings()
    print(
        "[同步检查] synchronous_mode={} | fixed_delta_seconds={} | substepping={} | max_substeps={} | max_substep_delta_time={}".format(
            settings.synchronous_mode,
            settings.fixed_delta_seconds,
            getattr(settings, "substepping", None),
            settings.max_substeps,
            settings.max_substep_delta_time
        )
    )

def cleanup_actors(client, actor_list):
    RTB.cleanup_actors(client, actor_list)

def warmup_map_cache(world):
    try:
        carla_map = world.get_map()
        _ = carla_map.name
        _ = carla_map.get_topology()
        _ = carla_map.generate_waypoints(20.0)
        print("[地图缓存] 已提前预热 map topology / waypoint cache。")
    except Exception as e:
        print("[地图缓存警告] 预热失败，但不影响场景继续运行：", e)

def choose_existing_blueprint(bp_lib, candidates):
    for bp_name in candidates:
        try:
            bp_lib.find(bp_name)
            return bp_name
        except Exception:
            continue
    raise RuntimeError("候选蓝图均不存在：{}".format(candidates))

def get_speed_kmh(vehicle):
    if not vehicle or not vehicle.is_alive:
        return 0.0
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

def distance_2d(loc_a, loc_b):
    return math.hypot(loc_a.x - loc_b.x, loc_a.y - loc_b.y)

def soft_hold_vehicle(vehicle, hand_brake=False):
    if vehicle and vehicle.is_alive:
        vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=0.0,
                hand_brake=hand_brake
            )
        )

def get_path_end_location(path):
    p = path[-1]
    return carla.Location(x=p[0], y=p[1], z=p[2])

def reached_path_end(vehicle, path, threshold=6.0):
    if not vehicle or not vehicle.is_alive or not path:
        return True
    return distance_2d(vehicle.get_location(), get_path_end_location(path)) <= threshold

def set_vehicle_lights(vehicle, brake=False, hazard=False, low_beam=True, high_beam=False, fog=False):
    if not vehicle or not vehicle.is_alive:
        return

    try:
        mask = 0
        mask |= int(carla.VehicleLightState.Position)

        if low_beam:
            mask |= int(carla.VehicleLightState.LowBeam)
        if high_beam:
            mask |= int(carla.VehicleLightState.HighBeam)
        if fog:
            mask |= int(carla.VehicleLightState.Fog)
        if brake:
            mask |= int(carla.VehicleLightState.Brake)
        if hazard:
            mask |= int(carla.VehicleLightState.LeftBlinker)
            mask |= int(carla.VehicleLightState.RightBlinker)

        vehicle.set_light_state(carla.VehicleLightState(mask))
    except Exception as e:
        print("[灯光警告] 设置车辆灯光失败：", e)

def make_transform_from_xy_yaw(carla_map, point, z_offset=0.0):
    x, y, yaw = point
    loc = carla.Location(x=x, y=y, z=0.5)

    try:
        wp = carla_map.get_waypoint(
            loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        if wp is not None:
            loc.z = wp.transform.location.z
    except Exception:
        pass

    loc.z += z_offset
    return carla.Transform(
        loc,
        carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0)
    )

def build_path_from_xy_yaw(carla_map, raw_traj, interval=1.0):
    raw_points = []
    for x, y, yaw in raw_traj:
        loc = carla.Location(x=x, y=y, z=0.5)
        try:
            wp = carla_map.get_waypoint(
                loc,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            if wp is not None:
                loc.z = wp.transform.location.z
        except Exception:
            pass
        raw_points.append((x, y, loc.z))

    raw_points = RTB.clean_trajectory(raw_points, min_dist=1e-5)
    dense = RTB.interpolate_trajectory(raw_points, interval=interval)
    return RTB.clean_trajectory(dense, min_dist=0.5)

def get_uturn_target_speed(vehicle, sim_time, state):
    if not vehicle or not vehicle.is_alive:
        return UTURN_TURN_SPEED_KMH

    vehicle_x = vehicle.get_location().x

    if state["phase"] == "initial" and vehicle_x <= UTURN_SLOW_TRIGGER_X:
        state["phase"] = "slow"
        print("[事件触发] 掉头背景车第一次到达 x<= {:.1f}，减速到 {} km/h | t={:.2f}s".format(
            UTURN_SLOW_TRIGGER_X,
            UTURN_SLOW_SPEED_KMH,
            sim_time
        ))

    if state["phase"] == "slow" and vehicle_x <= UTURN_TURN_TRIGGER_X:
        state["phase"] = "turn_slow"
        state["turn_start_time"] = sim_time
        print("[事件触发] 掉头背景车第一次到达 x<= {:.1f}，减速到 {} km/h | t={:.2f}s".format(
            UTURN_TURN_TRIGGER_X,
            UTURN_TURN_SPEED_KMH,
            sim_time
        ))

    if state["phase"] == "turn_slow":
        if sim_time - state["turn_start_time"] >= UTURN_TURN_DURATION_S:
            state["phase"] = "restored"
            print("[事件触发] 掉头背景车低速等待结束，恢复到 {} km/h | t={:.2f}s".format(
                UTURN_RESTORE_SPEED_KMH,
                sim_time
            ))
        else:
            return UTURN_TURN_SPEED_KMH

    if state["phase"] == "initial":
        return UTURN_INITIAL_SPEED_KMH
    if state["phase"] == "slow":
        return UTURN_SLOW_SPEED_KMH
    return UTURN_RESTORE_SPEED_KMH

def get_ego_target_speed(ego, sim_time, state):
    if not ego or not ego.is_alive:
        return EGO_TURN_SPEED_KMH

    if state["phase"] == "initial" and ego_x <= EGO_SLOW_TRIGGER_X:
        state["phase"] = "slow"
        print("[事件触发] Ego 第一次到达 x<= {:.1f}，减速到 {} km/h | t={:.2f}s".format(
            EGO_SLOW_TRIGGER_X,
            EGO_SLOW_SPEED_KMH,
            sim_time
        ))

    if state["phase"] == "slow" and ego_x <= EGO_TURN_TRIGGER_X:
        state["phase"] = "turn_slow"
        print("[事件触发] Ego 第一次到达 x<= {:.1f}，减速到 {} km/h | t={:.2f}s".format(
            EGO_TURN_TRIGGER_X,
            EGO_TURN_SPEED_KMH,
            sim_time
        ))

    if state["phase"] == "turn_slow" and ego_x <= EGO_RESTORE_TRIGGER_X:
        state["phase"] = "restored"
        print("[事件触发] Ego 到达 x<= {:.1f}，恢复到 {} km/h | t={:.2f}s".format(
            EGO_RESTORE_TRIGGER_X,
            EGO_RESTORE_SPEED_KMH,
            sim_time
        ))

    if state["phase"] == "initial":
        return EGO_INITIAL_SPEED_KMH
    if state["phase"] == "slow":
        return EGO_SLOW_SPEED_KMH
    if state["phase"] == "turn_slow":
        return EGO_TURN_SPEED_KMH
    return EGO_RESTORE_SPEED_KMH

def spawn_vehicle_by_tf(
    world,
    candidates,
    tf,
    color=None,
    role_name="background",
    z_offset=0.7,
    extra_z_offsets=None,
    xy_retry_offsets=None,
    yaw_retry_offsets=None
):
    """
    鲁棒车辆生成函数。
    支持多 z、xy、yaw 重试，避免坡道、碰撞盒、路缘导致生成失败。
    """
    bp_lib = world.get_blueprint_library()
    bp_name = choose_existing_blueprint(bp_lib, candidates)

    if extra_z_offsets is None:
        extra_z_offsets = [z_offset, 1.0, 1.2, 1.5, 1.8, 2.0]

    if xy_retry_offsets is None:
        xy_retry_offsets = [
            (0.0, 0.0),
            (0.4, 0.0),
            (-0.4, 0.0),
            (0.0, 0.4),
            (0.0, -0.4),
            (0.8, 0.0),
            (-0.8, 0.0),
            (0.0, 0.8),
            (0.0, -0.8),
        ]

    if yaw_retry_offsets is None:
        yaw_retry_offsets = [0.0, 2.0, -2.0, 4.0, -4.0]

    for dz in extra_z_offsets:
        for dx, dy in xy_retry_offsets:
            for dyaw in yaw_retry_offsets:
                try_tf = carla.Transform(
                    carla.Location(
                        x=tf.location.x + dx,
                        y=tf.location.y + dy,
                        z=tf.location.z
                    ),
                    carla.Rotation(
                        pitch=tf.rotation.pitch,
                        yaw=tf.rotation.yaw + dyaw,
                        roll=tf.rotation.roll
                    )
                )

                actor = RTB.spawn_vehicle(
                    world=world,
                    bp_name=bp_name,
                    x=try_tf.location.x,
                    y=try_tf.location.y,
                    z=try_tf.location.z,
                    yaw=try_tf.rotation.yaw,
                    color=color,
                    role_name=role_name,
                    z_offset=dz
                )

                if actor:
                    exact_tf = carla.Transform(
                        carla.Location(
                            x=try_tf.location.x,
                            y=try_tf.location.y,
                            z=try_tf.location.z + dz
                        ),
                        try_tf.rotation
                    )
                    actor.set_transform(exact_tf)
                    actor.set_autopilot(False)

                    print(
                        "[生成成功] {} | role={} | loc=({:.3f}, {:.3f}, {:.3f}) | yaw={:.2f} | z_offset={:.2f} | xy_offset=({:.2f},{:.2f}) | yaw_offset={:.2f}".format(
                            bp_name,
                            role_name,
                            exact_tf.location.x,
                            exact_tf.location.y,
                            exact_tf.location.z,
                            exact_tf.rotation.yaw,
                            dz,
                            dx,
                            dy,
                            dyaw
                        )
                    )
                    return actor

                try:
                    world.tick()
                except Exception:
                    pass

    print(
        "[生成失败] {} | role={} | base_loc=({:.3f}, {:.3f}, {:.3f})".format(
            bp_name,
            role_name,
            tf.location.x,
            tf.location.y,
            tf.location.z
        )
    )
    return None

def get_forward_and_left_from_transform(tf):
    yaw_rad = math.radians(tf.rotation.yaw)
    fx = math.cos(yaw_rad)
    fy = math.sin(yaw_rad)
    lx = -math.sin(yaw_rad)
    ly = math.cos(yaw_rad)
    return (fx, fy), (lx, ly)

def build_pedestrian_spawn_and_target_pairs():
    (fx, fy), (lx, ly) = get_forward_and_left_from_transform(PEDESTRIAN_START_TF)

    offsets = []
    for row in range(2):
        for col in range(4):
            longitudinal = -col * PEDESTRIAN_LONGITUDINAL_SPACING_M
            lateral = (row - 0.5) * PEDESTRIAN_LATERAL_SPACING_M
            offsets.append((longitudinal, lateral))

    pairs = []
    for longitudinal, lateral in offsets:
        start_loc = carla.Location(
            x=PEDESTRIAN_START_TF.location.x + fx * longitudinal + lx * lateral,
            y=PEDESTRIAN_START_TF.location.y + fy * longitudinal + ly * lateral,
            z=PEDESTRIAN_START_TF.location.z
        )

        target_loc = carla.Location(
            x=PEDESTRIAN_TARGET_TF.location.x + fx * longitudinal + lx * lateral,
            y=PEDESTRIAN_TARGET_TF.location.y + fy * longitudinal + ly * lateral,
            z=PEDESTRIAN_TARGET_TF.location.z
        )

        pairs.append((start_loc, target_loc))

    for i in range(len(pairs)):
        for j in range(i + 1, len(pairs)):
            d = distance_2d(pairs[i][0], pairs[j][0])
            if d < PEDESTRIAN_MIN_DISTANCE_M:
                raise RuntimeError(
                    "行人错位距离过近：{} 和 {} 距离 {:.2f}m".format(i, j, d)
                )

    return pairs

def get_walker_blueprints(world):
    walkers = list(world.get_blueprint_library().filter("walker.pedestrian.*"))
    if not walkers:
        raise RuntimeError("未找到 walker.pedestrian.* 行人蓝图。")
    return walkers

def configure_walker_blueprint(bp):
    if bp.has_attribute("is_invincible"):
        bp.set_attribute("is_invincible", "false")
    return bp

def spawn_pedestrians(world, actor_list):
    walker_bps = get_walker_blueprints(world)
    pairs = build_pedestrian_spawn_and_target_pairs()
    pedestrians = []

    for idx, (start_loc, target_loc) in enumerate(pairs):
        spawned = None
        attempts = [
            (0.0, 0.0),
            (0.12, 0.0),
            (-0.12, 0.0),
            (0.0, 0.12),
            (0.0, -0.12),
            (0.18, 0.18),
            (-0.18, -0.18),
        ]

        for dx, dy in attempts:
            walker_bp = configure_walker_blueprint(random.choice(walker_bps))

            spawn_tf = carla.Transform(
                carla.Location(
                    x=start_loc.x + dx,
                    y=start_loc.y + dy,
                    z=start_loc.z + PEDESTRIAN_SPAWN_Z_OFFSET
                ),
                carla.Rotation(
                    pitch=0.0,
                    yaw=PEDESTRIAN_START_TF.rotation.yaw,
                    roll=0.0
                )
            )

            walker = world.try_spawn_actor(walker_bp, spawn_tf)

            if walker:
                spawned = {
                    "actor": walker,
                    "target": carla.Location(
                        x=target_loc.x + dx,
                        y=target_loc.y + dy,
                        z=target_loc.z
                    ),
                    "index": idx
                }
                actor_list.append(walker)

                print(
                    "[行人生成成功] idx={} | loc=({:.3f}, {:.3f}, {:.3f}) | target=({:.3f}, {:.3f}, {:.3f})".format(
                        idx,
                        spawn_tf.location.x,
                        spawn_tf.location.y,
                        spawn_tf.location.z,
                        spawned["target"].x,
                        spawned["target"].y,
                        spawned["target"].z
                    )
                )
                break

        if not spawned:
            raise RuntimeError("第 {} 个行人生成失败。".format(idx))

        pedestrians.append(spawned)

    return pedestrians

def stop_pedestrian(walker):
    if walker and walker.is_alive:
        walker.apply_control(
            carla.WalkerControl(
                direction=carla.Vector3D(0.0, 0.0, 0.0),
                speed=0.0,
                jump=False
            )
        )

def update_pedestrians(pedestrians, sim_time):
    for item in pedestrians:
        walker = item["actor"]
        target_loc = item["target"]

        if not walker or not walker.is_alive:
            continue

        if sim_time < PEDESTRIAN_START_DELAY_S:
            stop_pedestrian(walker)
            continue

        current_loc = walker.get_location()
        dx = target_loc.x - current_loc.x
        dy = target_loc.y - current_loc.y
        dist = math.hypot(dx, dy)

        if dist <= PEDESTRIAN_STOP_DISTANCE_M:
            stop_pedestrian(walker)
            continue

        walker.apply_control(
            carla.WalkerControl(
                direction=carla.Vector3D(
                    x=dx / max(dist, 1e-6),
                    y=dy / max(dist, 1e-6),
                    z=0.0
                ),
                speed=PEDESTRIAN_WALK_SPEED_MPS,
                jump=False
            )
        )

def apply_static_weather(world):
    RTB.set_static_weather(
        world,
        cloudiness=cloudiness,
        precipitation=precipitation,
        precipitation_deposits=precipitation_deposits,
        wind_intensity=wind_intensity,
        sun_azimuth_angle=sun_azimuth_angle,
        sun_altitude_angle=sun_altitude_angle,
        fog_density=fog_density,
        fog_distance=fog_distance,
        fog_falloff=fog_falloff,
        wetness=wetness,
        scattering_intensity=scattering_intensity,
        mie_scattering_scale=mie_scattering_scale,
        rayleigh_scattering_scale=rayleigh_scattering_scale,
        dust_storm=dust_storm
    )

    print(
        "[天气设置] Clouds={} | Rain={} | Puddles={} | Wind={} | SunAzim={} | SunAlt={} | FogDens={} | FogDist={} | Wetness={} | Scatter={} | Mie={} | Rayleigh={} | Dust={}".format(
            cloudiness,
            precipitation,
            precipitation_deposits,
            wind_intensity,
            sun_azimuth_angle,
            sun_altitude_angle,
            fog_density,
            fog_distance,
            wetness,
            scattering_intensity,
            mie_scattering_scale,
            rayleigh_scattering_scale,
            dust_storm
        )
    )

def follow_path_constant_speed(
    vehicle,
    path,
    path_index,
    pid_lon,
    pid_lat,
    target_speed_kmh,
    max_speed_kmh,
    min_lookahead=5.5,
    lookahead_ratio=0.42,
    max_search_ahead=65,
    fallback_dist=50.0
):
    if not vehicle or not vehicle.is_alive or not path:
        return path_index

    current_speed = get_speed_kmh(vehicle)

    if current_speed > max_speed_kmh + 1.0:
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=0.40,
                steer=0.0,
                hand_brake=False
            )
        )
        return path_index

    target_wp, path_index = RTB.get_target_waypoint(
        vehicle_loc=vehicle.get_location(),
        path_points=path,
        current_index=path_index,
        speed_kmh=current_speed,
        min_lookahead=min_lookahead,
        lookahead_ratio=lookahead_ratio,
        max_search_ahead=max_search_ahead,
        fallback_dist=fallback_dist
    )

    if target_wp is None or target_speed_kmh <= 0.1:
        soft_hold_vehicle(vehicle)
        return path_index

    RTB.apply_pid_control(
        vehicle=vehicle,
        pid_lon=pid_lon,
        pid_lat=pid_lat,
        target_speed_kmh=target_speed_kmh,
        target_wp=target_wp
    )

    return path_index

# ============================================================
# 11. 主函数
# ============================================================

def main():
    actor_list = []
    world = None
    pedestrians = []

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    try:
        validate_user_inputs()

        world = client.get_world()
        carla_map = world.get_map()

        enable_sync(world, dt=DT)
        print_world_sync_state(world)
        warmup_map_cache(world)
        apply_static_weather(world)

        print("[预热] 同步模式预热中。")
        for _ in range(15):
            world.tick()
            time.sleep(DT)

        print("\n================ 路径构建阶段 ================")
        uturn_path = build_path_from_xy_yaw(carla_map, UTURN_RAW_TRAJECTORY, interval=TRAJ_INTERVAL_M)
        straight_path = build_path_from_xy_yaw(carla_map, STRAIGHT_RAW_TRAJECTORY, interval=TRAJ_INTERVAL_M)
        ego_path = build_path_from_xy_yaw(carla_map, EGO_RAW_TRAJECTORY, interval=TRAJ_INTERVAL_M)

        uturn_start_tf = make_transform_from_xy_yaw(carla_map, UTURN_RAW_TRAJECTORY[0])
        straight_start_tf = make_transform_from_xy_yaw(carla_map, STRAIGHT_RAW_TRAJECTORY[0])
        ego_start_tf = make_transform_from_xy_yaw(carla_map, EGO_RAW_TRAJECTORY[0])

        print("[路径结果] UTurn       : manual_xy_yaw | points={}".format(len(uturn_path)))
        print("[路径结果] Straight    : manual_xy_yaw | points={}".format(len(straight_path)))
        print("[路径结果] Ego         : manual_xy_yaw | points={}".format(len(ego_path)))
        print("================================================\n")

        uturn_vehicle = spawn_vehicle_by_tf(
            world=world,
            candidates=UTURN_BP_CANDIDATES,
            tf=uturn_start_tf,
            color="0,60,180",
            role_name="uturn_background_vehicle",
            z_offset=1.0,
            extra_z_offsets=[0.90, 1.10, 1.30, 1.50]
        )
        if not uturn_vehicle:
            raise RuntimeError("掉头背景车生成失败。")
        actor_list.append(uturn_vehicle)

        straight_vehicle = spawn_vehicle_by_tf(
            world=world,
            candidates=STRAIGHT_BP_CANDIDATES,
            tf=straight_start_tf,
            color="255,255,255",
            role_name="straight_vehicle",
            z_offset=1.0,
            extra_z_offsets=[0.90, 1.10, 1.30, 1.50]
        )
        if not straight_vehicle:
            raise RuntimeError("直行车生成失败。")
        actor_list.append(straight_vehicle)

        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        if not ego:
            raise RuntimeError("Ego 生成失败。")

        if ENABLE_PEDESTRIANS:
            pedestrians = spawn_pedestrians(world, actor_list)

        set_vehicle_lights(uturn_vehicle, brake=False, hazard=False, low_beam=True)
        set_vehicle_lights(straight_vehicle, brake=False, hazard=False, low_beam=True)
        set_vehicle_lights(ego, brake=False, hazard=False, low_beam=True)

        uturn_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.65),
            i_clip=(-1.0, 1.0)
        )
        uturn_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.50, 0.50),
            i_clip=(-1.0, 1.0)
        )

        straight_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.85, 0.45),
            i_clip=(-1.0, 1.0)
        )
        straight_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.45, 0.45),
            i_clip=(-1.0, 1.0)
        )

        uturn_idx = 0
        straight_idx = 0
        ego_idx = 0
        uturn_state = {"phase": "initial", "turn_start_time": None}
        ego_state = {"phase": "initial"}

        print("[预热] Actor 生成后同步预热。")
        sim_time = 0.0

        for _ in range(20):
            update_pedestrians(pedestrians, sim_time)
            world.tick()
            sim_time += DT
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            uturn_vehicle,
            target_speed_kmh=UTURN_INITIAL_SPEED_KMH,
            yaw_deg=UTURN_RAW_TRAJECTORY[0][2]
        )
        RTB.set_vehicle_initial_speed(
            straight_vehicle,
            target_speed_kmh=STRAIGHT_SPEED_KMH,
            yaw_deg=STRAIGHT_RAW_TRAJECTORY[0][2]
        )

        for _ in range(5):
            update_pedestrians(pedestrians, sim_time)
            world.tick()
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print(
            "[速度设置] UTurn初始={} km/h | x<={} 后 {} km/h | x<={} 后 {} km/h 持续 {}s，再恢复 {} km/h".format(
                UTURN_INITIAL_SPEED_KMH,
                UTURN_SLOW_TRIGGER_X,
                UTURN_SLOW_SPEED_KMH,
                UTURN_TURN_TRIGGER_X,
                UTURN_TURN_SPEED_KMH,
                UTURN_TURN_DURATION_S,
                UTURN_RESTORE_SPEED_KMH
            )
        )
        print(
            "[速度设置] Straight={} km/h | Ego初始={} km/h | x<={} 后 {} km/h | x<={} 后 {} km/h | x<={} 后恢复 {} km/h".format(
                STRAIGHT_SPEED_KMH,
                EGO_INITIAL_SPEED_KMH,
                EGO_SLOW_TRIGGER_X,
                EGO_SLOW_SPEED_KMH,
                EGO_TURN_TRIGGER_X,
                EGO_TURN_SPEED_KMH,
                EGO_RESTORE_TRIGGER_X,
                EGO_RESTORE_SPEED_KMH
            )
        )

        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            sim_time += DT
            frame_count += 1

            update_pedestrians(pedestrians, sim_time)

            if reached_path_end(uturn_vehicle, uturn_path, threshold=4.5):
                soft_hold_vehicle(uturn_vehicle)
                set_vehicle_lights(uturn_vehicle, brake=True, low_beam=True)
            else:
                uturn_target_speed = get_uturn_target_speed(uturn_vehicle, sim_time, uturn_state)
                uturn_idx = follow_path_constant_speed(
                    vehicle=uturn_vehicle,
                    path=uturn_path,
                    path_index=uturn_idx,
                    pid_lon=uturn_pid_lon,
                    pid_lat=uturn_pid_lat,
                    target_speed_kmh=uturn_target_speed,
                    max_speed_kmh=UTURN_MAX_SPEED_KMH,
                    min_lookahead=5.5,
                    lookahead_ratio=0.40,
                    max_search_ahead=75,
                    fallback_dist=55.0
                )
                set_vehicle_lights(uturn_vehicle, brake=False, low_beam=True)

            if reached_path_end(straight_vehicle, straight_path, threshold=5.0):
                soft_hold_vehicle(straight_vehicle)
                set_vehicle_lights(straight_vehicle, brake=True, low_beam=True)
            else:
                straight_idx = follow_path_constant_speed(
                    vehicle=straight_vehicle,
                    path=straight_path,
                    path_index=straight_idx,
                    pid_lon=straight_pid_lon,
                    pid_lat=straight_pid_lat,
                    target_speed_kmh=STRAIGHT_SPEED_KMH,
                    max_speed_kmh=STRAIGHT_MAX_SPEED_KMH,
                    min_lookahead=4.5,
                    lookahead_ratio=0.36,
                    max_search_ahead=65,
                    fallback_dist=45.0
                )
                set_vehicle_lights(straight_vehicle, brake=False, low_beam=True)

            if reached_path_end(ego, ego_path, threshold=5.0):
                soft_hold_vehicle(ego)
                set_vehicle_lights(ego, brake=True, low_beam=True)
                print("[场景结束] Ego 已到达轨迹终点附近。")
                break
            else:
                ego_target_speed = get_ego_target_speed(ego, sim_time, ego_state)
                ego_idx = follow_path_constant_speed(
                    vehicle=ego,
                    path=ego_path,
                    path_index=ego_idx,
                    pid_lon=ego_pid_lon,
                    pid_lat=ego_pid_lat,
                    target_speed_kmh=ego_target_speed,
                    max_speed_kmh=EGO_MAX_SPEED_KMH,
                    min_lookahead=5.5,
                    lookahead_ratio=0.40,
                    max_search_ahead=75,
                    fallback_dist=55.0
                )
                set_vehicle_lights(ego, brake=False, low_beam=True)

            if frame_count % int(2.0 / DT) == 0:
                ped_state = "WAIT" if sim_time < PEDESTRIAN_START_DELAY_S else "MOVE"
                print(
                    "[t={:05.2f}s | frame={:04d}] UTurn={:05.1f} | Straight={:05.1f} | Ego={:05.1f} | idx(U/S/E)=({}/{}/{}) | State(U/E)=({}/{}) | Ped={}".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(uturn_vehicle),
                        get_speed_kmh(straight_vehicle),
                        get_speed_kmh(ego),
                        uturn_idx,
                        straight_idx,
                        ego_idx,
                        uturn_state["phase"],
                        ego_state["phase"],
                        ped_state
                    )
                )

            compute_time = time.time() - loop_t0
            if compute_time < DT:
                time.sleep(DT - compute_time)

        print("[场景结束] 主循环正常结束。")

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动终止。")

    except Exception as e:
        print("[错误] 场景运行异常：", e)
        traceback.print_exc()

    finally:
        try:
            for item in pedestrians:
                stop_pedestrian(item["actor"])
        except Exception:
            pass

        if world is not None:
            if KEEP_ACTORS_AFTER_SCRIPT:
                print("[清理策略] KEEP_ACTORS_AFTER_SCRIPT=True，保留 Actor。")
            else:
                try:
                    cleanup_actors(client, actor_list)
                except Exception as e:
                    print("[清理警告] cleanup_actors 失败：", e)

            try:
                disable_sync(world)
            except Exception as e:
                print("[同步恢复警告] disable_sync 失败：", e)

        print("[脚本退出] 已清理 Actor 并恢复异步模式。")

if __name__ == "__main__":
    main()
