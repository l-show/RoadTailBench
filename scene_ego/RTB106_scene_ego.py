# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

场景主题：
城市道路限界侵入隐患 + Ego 正常通行 + 对向来车会车约束 + 路侧行人干扰

本版内容：
1. Ego 按给定起点和终点，通过 GlobalRoutePlanner 自动生成路线，固定速度循迹。
2. 对向来车按给定起点和终点，通过 GlobalRoutePlanner 自动生成路线，固定 45 km/h 行驶。
3. 对向来车支持开始运动前静止等待时间 ONCOMING_START_DELAY_S。
4. 生成 2 个行人，起点和终点由用户指定，行人开始运动前支持等待时间 PEDESTRIAN_START_DELAY_S。
5. 行人不使用 AI Controller，直接使用 WalkerControl 驱动，避免 NavMesh 不可用导致原地不动。
6. 行人生成使用 try_spawn_actor + 最小间距检查，避免生成碰撞箱冲突。
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

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except Exception as e:
    GlobalRoutePlanner = None
    print("[导入失败] GlobalRoutePlanner 导入失败：", e)

# ============================================================
# 1. 全局可调接口：仿真配置
# ============================================================

HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

DT = 0.05
SCENARIO_DURATION = 60.0
KEEP_ACTORS_AFTER_SCRIPT = False

# GlobalRoutePlanner 自动导航锚点间隔
ROUTE_RESOLUTION_M = 2.0

# ============================================================
# 2. 全局可调接口：车辆速度与启动延迟
# ============================================================

# Ego 速度状态机
EGO_INITIAL_SPEED_KMH = 35.0
EGO_SLOW_SPEED_KMH = 10.0
EGO_POST_SLOW_SPEED_KMH = 30.0
EGO_MAX_SPEED_KMH = 42.0
EGO_SLOW_TRIGGER_X = 70.0
EGO_SLOW_DURATION_S = 2.0

# 前车固定 10 km/h，沿给定轨迹循迹，终点停车
FRONT_VEHICLE_SPEED_KMH = 6.0
FRONT_VEHICLE_MAX_SPEED_KMH = 32.0

# 对向来车固定 42 km/h
ONCOMING_SPEED_KMH = 42.0
ONCOMING_MAX_SPEED_KMH = 52.0

# 对向来车开始运动前静止等待时间，单位秒。
ONCOMING_START_DELAY_S = 1.0

# ============================================================
# 3. 全局可调接口：天气参数
# ============================================================
# 限界侵入场景建议使用清晰天气，突出固定构造物侵限问题。
# 如需复用你前面截图天气，可以直接改这里。

cloudiness = 10.0
precipitation = 0.0
precipitation_deposits = 0.0
wind_intensity = 5.0
sun_azimuth_angle = 0.0
sun_altitude_angle = 45.0
fog_density = 0.0
fog_distance = 0.75
fog_falloff = 0.1
wetness = 0.0
scattering_intensity = 1.0
mie_scattering_scale = 0.03
rayleigh_scattering_scale = 0.0331
dust_storm = 0.0

# ============================================================
# 4. 全局可调接口：车辆蓝图候选
# ============================================================

EGO_BP_CANDIDATES = [
    "vehicle.tesla.model3"
]

ONCOMING_BP_CANDIDATES = [
    "vehicle.dodge.charger_2020",
    "vehicle.audi.tt",
    "vehicle.lincoln.mkz_2020",
    "vehicle.tesla.model3",
]

FRONT_VEHICLE_BP_CANDIDATES = [
    "vehicle.mini.cooper_s",
    "vehicle.mini.cooper_s_2021",
]

# ============================================================
# 5. 全局可调接口：Ego 完整轨迹，格式为 (x, y, yaw)
# ============================================================

EGO_RAW_TRAJECTORY = [
    (140.252, -0.622, 178.130), (138.267, -0.566, 178.498), (135.735, -0.554, -178.541), (133.236, -0.637, -178.049),
    (130.698, -0.748, -178.184), (128.205, -0.814, -178.553), (125.667, -0.904, -177.445), (123.170, -1.016, -177.691),
    (120.630, -1.118, -177.691), (118.051, -1.256, -176.269), (115.473, -1.428, -175.899), (112.981, -1.620, -174.850),
    (110.493, -1.860, -173.835), (107.972, -2.181, -172.586), (105.462, -2.572, -169.551), (103.010, -3.051, -168.453),
    (100.523, -3.576, -169.219), (98.056, -3.979, -171.711), (95.497, -4.336, -172.083), (93.021, -4.680, -172.206),
    (90.500, -5.010, -172.549), (88.020, -5.330, -173.168), (85.536, -5.613, -174.259), (82.966, -5.866, -174.382),
    (80.438, -6.111, -175.002), (77.864, -6.336, -175.002), (75.291, -6.557, -175.126), (72.218, -6.819, -175.126),
    (68.420, -7.144, -174.878), (64.691, -7.535, -172.793), (60.977, -8.054, -170.989), (57.226, -8.735, -169.050),
    (53.483, -9.460, -169.050), (49.795, -10.133, -170.788), (46.083, -10.662, -173.844), (42.290, -11.049, -174.217),
    (38.497, -11.435, -173.968), (34.777, -11.905, -171.693), (31.010, -12.474, -171.692), (27.292, -12.944, -174.329),
    (23.493, -13.265, -175.339), (19.756, -13.570, -175.339), (16.018, -13.878, -175.215), (12.281, -14.191, -175.215),
    (8.482, -14.509, -175.215), (4.745, -14.818, -175.339), (0.880, -15.099, -177.113), (-2.865, -15.287, -177.113),
    (-6.735, -15.483, -177.113), (-10.481, -15.663, -177.736), (-14.290, -15.813, -177.861), (-18.100, -15.955, -177.861),
    (-21.847, -16.093, -178.234), (-25.659, -16.142, -179.678), (-29.409, -16.127, 179.119), (-33.158, -16.070, 179.119),
    (-36.970, -16.011, 179.119), (-40.722, -15.966, 179.368), (-44.350, -15.926, 179.368), (-48.387, -15.882, 179.368),
    (-52.199, -15.840, 179.368), (-55.949, -15.798, 179.368), (-59.698, -15.745, 179.119), (-63.510, -15.663, 177.607),
    (-67.379, -15.463, 176.076), (-71.118, -15.169, 175.078), (-74.977, -14.824, 174.703), (-78.775, -14.480, 175.077),
    (-82.644, -14.279, 178.264), (-84.552, -14.392, -179.615), (-84.552, -14.392, -179.615), (-84.552, -14.392, -179.615),
    (-84.552, -14.392, -179.615), (-84.552, -14.392, -179.615), (-84.702, -14.393, -179.615), (-85.202, -14.401, -176.673),
    (-85.707, -14.454, -170.843), (-86.198, -14.553, -168.300), (-86.671, -14.733, -152.463), (-87.093, -15.001, -144.789),
    (-87.496, -15.295, -140.783), (-87.874, -15.636, -137.067), (-88.225, -15.991, -131.920), (-88.551, -16.381, -128.759),
    (-88.858, -16.775, -127.675), (-89.168, -17.177, -127.675), (-89.473, -17.573, -126.759), (-89.763, -17.990, -122.659),
    (-90.011, -18.424, -116.344), (-90.210, -18.892, -109.297), (-90.359, -19.369, -107.078), (-90.490, -19.861, -101.154),
    (-90.579, -20.363, -100.020), (-90.667, -20.863, -99.409), (-90.749, -21.357, -99.409), (-90.830, -21.850, -99.409),
    (-90.913, -22.351, -99.409), (-90.994, -22.844, -99.409), (-91.077, -23.344, -99.409), (-91.158, -23.838, -99.409),
    (-91.242, -24.348, -99.278), (-91.323, -24.841, -99.278), (-91.406, -25.343, -99.278), (-91.486, -25.837, -98.883),
    (-91.560, -26.339, -97.067), (-91.609, -26.845, -94.644), (-91.647, -27.344, -91.756), (-91.639, -27.852, -88.616),
    (-91.627, -28.352, -88.616), (-91.614, -28.860, -88.616), (-91.602, -29.360, -88.616), (-91.591, -29.868, -89.561),
    (-91.591, -30.368, -90.019), (-91.589, -30.868, -89.756), (-91.587, -31.376, -89.756), (-91.585, -31.876, -89.756),
    (-91.582, -32.376, -89.756), (-91.580, -32.885, -89.756), (-91.578, -33.385, -89.756), (-91.575, -33.893, -89.756),
    (-91.573, -34.392, -89.756), (-91.572, -34.609, -89.756), (-91.572, -34.609, -89.756), (-91.572, -34.609, -89.756),
    (-91.572, -34.609, -89.756),
]

# ============================================================
# 6. 全局可调接口：前车完整轨迹，格式为 (x, y, yaw)
# ============================================================

FRONT_VEHICLE_RAW_TRAJECTORY = [
    (67.638, -7.784, -179.412), (67.638, -7.784, -179.412), (67.638, -7.784, -179.412), (67.638, -7.784, -179.412),
    (67.538, -7.785, -179.412), (67.039, -7.790, -179.272), (66.531, -7.797, -179.272), (66.023, -7.803, -179.272),
    (65.515, -7.810, -179.272), (65.016, -7.816, -179.272), (64.516, -7.822, -179.482), (63.941, -7.827, -179.482),
    (61.524, -7.849, -179.482), (59.082, -7.379, 160.700), (56.871, -6.229, 144.707), (55.520, -5.261, 144.392),
    (55.108, -4.963, 144.042), (54.709, -4.662, 140.619), (54.329, -4.324, 136.182), (53.969, -3.978, 136.182),
    (53.625, -3.616, 128.491), (53.346, -3.192, 119.831), (53.093, -2.751, 119.831), (52.842, -2.309, 118.103),
    (52.644, -1.842, 110.183), (52.466, -1.358, 110.183), (51.804, 0.441, 110.183), (51.417, 2.942, 92.610),
    (51.553, 5.511, 74.131), (52.258, 7.953, 73.900), (53.342, 10.188, 54.714), (54.999, 12.051, 41.106),
    (57.115, 13.430, 23.863), (59.461, 14.269, 12.451), (61.907, 14.786, 11.872), (64.353, 15.300, 11.872),
    (66.840, 15.823, 11.872), (69.304, 16.244, 8.457), (71.831, 16.512, 3.215), (74.329, 16.599, 0.811),
    (76.829, 16.632, 0.741), (76.870, 16.632, 0.741), (76.870, 16.632, 0.741), (76.870, 16.632, 0.741),
    (76.870, 16.632, 0.741), (76.870, 16.632, 0.741),
]

# ============================================================
# 7. 全局可调接口：对向来车起终点
# ============================================================

ONCOMING_START_TF = carla.Transform(
    carla.Location(x=-70.445, y=-9.699, z=1.259),
    carla.Rotation(pitch=0.033, yaw=1.099, roll=0.000)
)

ONCOMING_END_TF = carla.Transform(
    carla.Location(x=171.694, y=2.798, z=1.470),
    carla.Rotation(pitch=-11.631, yaw=4.351, roll=0.000)
)

# ============================================================
# 7. 全局可调接口：行人
# ============================================================

ENABLE_PEDESTRIANS = True
PEDESTRIAN_COUNT = 2

PEDESTRIAN_START_TF = carla.Transform(
    carla.Location(x=13.509, y=-15.917, z=0.700),
    carla.Rotation(pitch=0.000, yaw=91.087, roll=0.000)
)

PEDESTRIAN_TARGET_TF = carla.Transform(
    carla.Location(x=13.873, y=-5.275, z=0.922),
    carla.Rotation(pitch=-6.029, yaw=82.542, roll=0.000)
)

# 行人开始运动前等待时间，单位秒。
PEDESTRIAN_START_DELAY_S = 14.0

# 行人速度
PEDESTRIAN_WALK_SPEED_MPS = 1.20

# 行人距离目标点小于该距离后停止
PEDESTRIAN_STOP_DISTANCE_M = 0.45

# 两名行人生成时的紧凑间距，不建议小于 0.7，否则容易碰撞箱冲突
PEDESTRIAN_PAIR_SPACING_M = 0.85

# 行人之间最小生成距离
PEDESTRIAN_MIN_DISTANCE_M = 0.65

# 行人生成 z 抬高，避免卡地面
PEDESTRIAN_SPAWN_Z_OFFSET = 0.20

# 若初始紧凑点生成失败，允许在小范围内寻找备用点
PEDESTRIAN_FALLBACK_RADIUS_M = 0.80
PEDESTRIAN_MAX_ATTEMPTS_PER_PERSON = 25

# ============================================================
# 8. 基础工具函数
# ============================================================

def validate_user_inputs():
    if GlobalRoutePlanner is None:
        raise RuntimeError(
            "无法导入 CARLA GlobalRoutePlanner。请检查 CARLA_ROOT 是否正确，"
            "或者确认 PythonAPI\\carla\\agents\\navigation\\global_route_planner.py 存在。"
        )

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

def get_path_end_location(path):
    p = path[-1]
    return carla.Location(x=p[0], y=p[1], z=p[2])

def reached_path_end(vehicle, path, threshold=5.0):
    if not vehicle or not vehicle.is_alive or not path:
        return True

    return distance_2d(vehicle.get_location(), get_path_end_location(path)) <= threshold

def soft_hold_vehicle(vehicle):
    if vehicle and vehicle.is_alive:
        vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=0.0,
                hand_brake=False
            )
        )

def hold_vehicle_before_release(vehicle):
    if not vehicle or not vehicle.is_alive:
        return

    vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    vehicle.apply_control(
        carla.VehicleControl(
            throttle=0.0,
            brake=1.0,
            steer=0.0,
            hand_brake=True
        )
    )

def release_vehicle(vehicle):
    if not vehicle or not vehicle.is_alive:
        return

    vehicle.apply_control(
        carla.VehicleControl(
            throttle=0.0,
            brake=0.0,
            steer=0.0,
            hand_brake=False
        )
    )

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

def spawn_vehicle_by_tf(world, candidates, tf, color=None, role_name="background", z_offset=0.7):
    bp_lib = world.get_blueprint_library()
    bp_name = choose_existing_blueprint(bp_lib, candidates)

    actor = RTB.spawn_vehicle(
        world=world,
        bp_name=bp_name,
        x=tf.location.x,
        y=tf.location.y,
        z=tf.location.z,
        yaw=tf.rotation.yaw,
        color=color,
        role_name=role_name,
        z_offset=z_offset
    )

    if actor:
        exact_tf = carla.Transform(
            carla.Location(
                x=tf.location.x,
                y=tf.location.y,
                z=tf.location.z + z_offset
            ),
            tf.rotation
        )
        actor.set_transform(exact_tf)
        actor.set_autopilot(False)

        print(
            "[生成成功] {} | role={} | loc=({:.3f}, {:.3f}, {:.3f}) | yaw={:.2f}".format(
                bp_name,
                role_name,
                exact_tf.location.x,
                exact_tf.location.y,
                exact_tf.location.z,
                exact_tf.rotation.yaw
            )
        )

    return actor

# ============================================================
# 9. 天气设置
# ============================================================

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
        "[天气设置] Clouds={} | Rain={} | Puddles={} | Wind={} | SunAzim={} | SunAlt={} | FogDens={} | FogDist={} | Wetness={}".format(
            cloudiness,
            precipitation,
            precipitation_deposits,
            wind_intensity,
            sun_azimuth_angle,
            sun_altitude_angle,
            fog_density,
            fog_distance,
            wetness
        )
    )

# ============================================================
# 10. 路径生成函数
# ============================================================

def build_route_by_global_planner(carla_map, start_loc, end_loc, resolution=2.0):
    grp = GlobalRoutePlanner(carla_map, sampling_resolution=resolution)
    route = grp.trace_route(start_loc, end_loc)

    if not route:
        raise RuntimeError("GlobalRoutePlanner 未生成路线，请检查起终点是否在同一可达道路网络上。")

    path = []
    for wp, road_option in route:
        loc = wp.transform.location
        path.append((loc.x, loc.y, loc.z))

    path = RTB.clean_trajectory(path, min_dist=0.5)
    return path

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

def get_ego_target_speed(ego, sim_time, state):
    if state["phase"] == "initial":
        if ego and ego.is_alive and ego.get_location().x <= EGO_SLOW_TRIGGER_X:
            state["phase"] = "slow"
            state["slow_start_time"] = sim_time
            print("[事件触发] Ego 到达 x<= {:.1f}，减速到 {} km/h | t={:.2f}s".format(
                EGO_SLOW_TRIGGER_X,
                EGO_SLOW_SPEED_KMH,
                sim_time
            ))
        else:
            return EGO_INITIAL_SPEED_KMH

    if state["phase"] == "slow":
        if sim_time - state["slow_start_time"] >= EGO_SLOW_DURATION_S:
            state["phase"] = "post_slow"
            print("[事件触发] Ego 低速阶段结束，恢复到 {} km/h | t={:.2f}s".format(
                EGO_POST_SLOW_SPEED_KMH,
                sim_time
            ))
        else:
            return EGO_SLOW_SPEED_KMH

    return EGO_POST_SLOW_SPEED_KMH

# ============================================================
# 11. 车辆循迹控制函数
# ============================================================

def follow_path_constant_speed(
    vehicle,
    path,
    path_index,
    pid_lon,
    pid_lat,
    target_speed_kmh,
    max_speed_kmh,
    min_lookahead=5.5,
    lookahead_ratio=0.42
):
    if not vehicle or not vehicle.is_alive or not path:
        return path_index

    current_speed = get_speed_kmh(vehicle)

    if current_speed > max_speed_kmh + 1.0:
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=0.45,
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
        max_search_ahead=55,
        fallback_dist=45.0
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
# 12. 行人工具函数
# ============================================================

def rotate_offset_by_yaw(offset_x, offset_y, yaw_deg):
    yaw = math.radians(yaw_deg)
    rx = offset_x * math.cos(yaw) - offset_y * math.sin(yaw)
    ry = offset_x * math.sin(yaw) + offset_y * math.cos(yaw)
    return rx, ry

def make_location_with_offset(center_tf, offset_x, offset_y, z_offset=0.0):
    rx, ry = rotate_offset_by_yaw(offset_x, offset_y, center_tf.rotation.yaw)
    return carla.Location(
        x=center_tf.location.x + rx,
        y=center_tf.location.y + ry,
        z=center_tf.location.z + z_offset
    )

def is_far_enough_from_existing(candidate_loc, accepted_locs, min_dist):
    for loc in accepted_locs:
        if distance_2d(candidate_loc, loc) < min_dist:
            return False
    return True

def get_walker_blueprints(world):
    bp_lib = world.get_blueprint_library()
    walkers = list(bp_lib.filter("walker.pedestrian.*"))

    if not walkers:
        raise RuntimeError("当前 CARLA blueprint library 中未找到 walker.pedestrian.* 行人蓝图。")

    return walkers

def configure_walker_blueprint(bp):
    if bp.has_attribute("is_invincible"):
        bp.set_attribute("is_invincible", "false")
    return bp

def get_pedestrian_base_offsets():
    """
    2 个行人使用紧凑横向队形，既不会太远，也避免碰撞箱重叠。
    """
    half = PEDESTRIAN_PAIR_SPACING_M * 0.5
    return [
        (-half, 0.0),
        (half, 0.0),
    ]

def spawn_pedestrians(world):
    """
    生成 2 个不会碰撞箱冲突的行人。
    行人不使用 AI Controller，由主循环 WalkerControl 直接驱动。
    """
    walker_bps = get_walker_blueprints(world)

    walkers = []
    target_locs = []
    accepted_spawn_locs = []

    base_offsets = get_pedestrian_base_offsets()

    for i in range(PEDESTRIAN_COUNT):
        spawned_walker = None

        candidate_offsets = [base_offsets[i]]

        for _ in range(PEDESTRIAN_MAX_ATTEMPTS_PER_PERSON - 1):
            angle = random.uniform(0.0, 2.0 * math.pi)
            radius = random.uniform(0.0, PEDESTRIAN_FALLBACK_RADIUS_M)
            jitter_x = math.cos(angle) * radius
            jitter_y = math.sin(angle) * radius
            candidate_offsets.append((base_offsets[i][0] + jitter_x, base_offsets[i][1] + jitter_y))

        for offset_x, offset_y in candidate_offsets:
            spawn_loc = make_location_with_offset(
                PEDESTRIAN_START_TF,
                offset_x,
                offset_y,
                z_offset=PEDESTRIAN_SPAWN_Z_OFFSET
            )

            if not is_far_enough_from_existing(
                spawn_loc,
                accepted_spawn_locs,
                PEDESTRIAN_MIN_DISTANCE_M
            ):
                continue

            walker_bp = random.choice(walker_bps)
            walker_bp = configure_walker_blueprint(walker_bp)

            spawn_tf = carla.Transform(
                spawn_loc,
                carla.Rotation(
                    pitch=0.0,
                    yaw=PEDESTRIAN_START_TF.rotation.yaw,
                    roll=0.0
                )
            )

            spawned_walker = world.try_spawn_actor(walker_bp, spawn_tf)

            if spawned_walker is not None:
                walkers.append(spawned_walker)
                accepted_spawn_locs.append(spawn_loc)
                break

        if spawned_walker is None:
            raise RuntimeError(
                "第 {} 个行人生成失败：候选点存在碰撞或非法位置，已停止生成以避免碰撞箱冲突。".format(i + 1)
            )

    for i in range(PEDESTRIAN_COUNT):
        target_offset_x, target_offset_y = base_offsets[i]
        target_loc = make_location_with_offset(
            PEDESTRIAN_TARGET_TF,
            target_offset_x,
            target_offset_y,
            z_offset=0.0
        )
        target_locs.append(target_loc)

    world.tick()

    print("[行人生成] 已成功生成 {} 个行人，未检测到生成碰撞箱冲突。".format(len(walkers)))
    return walkers, target_locs

def stop_pedestrians(walkers):
    for walker in walkers:
        if not walker or not walker.is_alive:
            continue

        walker.apply_control(
            carla.WalkerControl(
                direction=carla.Vector3D(0.0, 0.0, 0.0),
                speed=0.0,
                jump=False
            )
        )

def update_pedestrians(walkers, target_locs, sim_time):
    """
    行人开始等待 PEDESTRIAN_START_DELAY_S 后，直接朝目标点移动。
    不依赖 NavMesh，也不使用 controller.ai.walker。
    """
    if not walkers:
        return

    if sim_time < PEDESTRIAN_START_DELAY_S:
        stop_pedestrians(walkers)
        return

    for walker, target_loc in zip(walkers, target_locs):
        if not walker or not walker.is_alive:
            continue

        current_loc = walker.get_location()
        dx = target_loc.x - current_loc.x
        dy = target_loc.y - current_loc.y
        dist = math.hypot(dx, dy)

        if dist <= PEDESTRIAN_STOP_DISTANCE_M:
            walker.apply_control(
                carla.WalkerControl(
                    direction=carla.Vector3D(0.0, 0.0, 0.0),
                    speed=0.0,
                    jump=False
                )
            )
            continue

        direction = carla.Vector3D(
            x=dx / max(dist, 1e-6),
            y=dy / max(dist, 1e-6),
            z=0.0
        )

        walker.apply_control(
            carla.WalkerControl(
                direction=direction,
                speed=PEDESTRIAN_WALK_SPEED_MPS,
                jump=False
            )
        )

# ============================================================
# 13. 主函数
# ============================================================

def main():
    actor_list = []
    walkers = []
    pedestrian_target_locs = []
    world = None

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    try:
        validate_user_inputs()

        world = client.get_world()
        carla_map = world.get_map()

        # ====================================================
        # 13.1 同步模式 + 地图缓存 + 天气
        # ====================================================
        enable_sync(world, dt=DT)
        print_world_sync_state(world)
        warmup_map_cache(world)
        apply_static_weather(world)

        print("[预热] 同步模式预热中。")
        for _ in range(15):
            world.tick()
            time.sleep(DT)

        # ====================================================
        # 13.2 构建 Ego、前车固定轨迹与对向车自动路线
        # ====================================================
        ego_path = build_path_from_xy_yaw(
            carla_map=carla_map,
            raw_traj=EGO_RAW_TRAJECTORY,
            interval=1.0
        )
        front_vehicle_path = build_path_from_xy_yaw(
            carla_map=carla_map,
            raw_traj=FRONT_VEHICLE_RAW_TRAJECTORY,
            interval=0.8
        )

        oncoming_path = build_route_by_global_planner(
            carla_map=carla_map,
            start_loc=ONCOMING_START_TF.location,
            end_loc=ONCOMING_END_TF.location,
            resolution=ROUTE_RESOLUTION_M
        )

        ego_start_tf = make_transform_from_xy_yaw(carla_map, EGO_RAW_TRAJECTORY[0])
        front_vehicle_start_tf = make_transform_from_xy_yaw(carla_map, FRONT_VEHICLE_RAW_TRAJECTORY[0])

        print("[路径检查] ego_path points:", len(ego_path))
        print("[路径检查] front_vehicle_path points:", len(front_vehicle_path))
        print("[路径检查] oncoming_path points:", len(oncoming_path))
        print("[控制策略] Ego 给定轨迹 PID 循迹 + 速度状态机；Mini 前车 25 km/h 给定轨迹循迹；对向来车保留原逻辑。")

        # ====================================================
        # 13.3 生成 Ego
        # ====================================================
        ego = spawn_vehicle_by_tf(
            world=world,
            candidates=EGO_BP_CANDIDATES,
            tf=ego_start_tf,
            color="0,80,255",
            role_name="ego",
            z_offset=0.75
        )
        if not ego:
            raise RuntimeError("Ego 生成失败。")
        actor_list.append(ego)

        set_vehicle_lights(
            ego,
            brake=False,
            hazard=False,
            low_beam=True
        )

        # ====================================================
        # 13.4 生成 Mini 前车
        # ====================================================
        front_vehicle = spawn_vehicle_by_tf(
            world=world,
            candidates=FRONT_VEHICLE_BP_CANDIDATES,
            tf=front_vehicle_start_tf,
            color="255,255,255",
            role_name="front_mini_vehicle",
            z_offset=0.75
        )
        if not front_vehicle:
            raise RuntimeError("Mini 前车生成失败。")
        actor_list.append(front_vehicle)

        set_vehicle_lights(
            front_vehicle,
            brake=False,
            hazard=False,
            low_beam=True
        )

        # ====================================================
        # 13.5 生成对向来车
        # ====================================================
        oncoming = spawn_vehicle_by_tf(
            world=world,
            candidates=ONCOMING_BP_CANDIDATES,
            tf=ONCOMING_START_TF,
            color="255,255,255",
            role_name="oncoming_vehicle",
            z_offset=0.75
        )
        if not oncoming:
            raise RuntimeError("对向来车生成失败。")
        actor_list.append(oncoming)

        hold_vehicle_before_release(oncoming)
        set_vehicle_lights(
            oncoming,
            brake=True,
            hazard=False,
            low_beam=True
        )

        # ====================================================
        # 13.6 生成 2 个行人
        # ====================================================
        if ENABLE_PEDESTRIANS:
            walkers, pedestrian_target_locs = spawn_pedestrians(world)
            actor_list.extend(walkers)

        # ====================================================
        # 13.7 独立 PID
        # ====================================================
        ego_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.55),
            i_clip=(-1.0, 1.0)
        )
        ego_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.45, 0.45),
            i_clip=(-1.0, 1.0)
        )

        front_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.55),
            i_clip=(-1.0, 1.0)
        )
        front_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.45, 0.45),
            i_clip=(-1.0, 1.0)
        )

        oncoming_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.60),
            i_clip=(-1.0, 1.0)
        )
        oncoming_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.45, 0.45),
            i_clip=(-1.0, 1.0)
        )

        ego_idx = 0
        front_vehicle_idx = 0
        front_vehicle_finished = False
        oncoming_idx = 0
        oncoming_released = False
        ego_speed_state = {
            "phase": "initial",
            "slow_start_time": None,
        }

        # ====================================================
        # 13.8 Actor 生成后预热与初速度注入
        # ====================================================
        print("[预热] Actor 生成后同步预热。")
        sim_time = 0.0

        for _ in range(20):
            hold_vehicle_before_release(oncoming)
            update_pedestrians(walkers, pedestrian_target_locs, sim_time)

            world.tick()
            sim_time += DT
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_INITIAL_SPEED_KMH,
            yaw_deg=EGO_RAW_TRAJECTORY[0][2]
        )
        RTB.set_vehicle_initial_speed(
            front_vehicle,
            target_speed_kmh=FRONT_VEHICLE_SPEED_KMH,
            yaw_deg=FRONT_VEHICLE_RAW_TRAJECTORY[0][2]
        )

        for _ in range(5):
            hold_vehicle_before_release(oncoming)
            update_pedestrians(walkers, pedestrian_target_locs, sim_time)

            world.tick()
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print("[速度设置] Ego初始={} km/h | x<={} 后 {} km/h 持续 {}s，再恢复 {} km/h | Mini前车={} km/h | Oncoming={} km/h".format(
            EGO_INITIAL_SPEED_KMH,
            EGO_SLOW_TRIGGER_X,
            EGO_SLOW_SPEED_KMH,
            EGO_SLOW_DURATION_S,
            EGO_POST_SLOW_SPEED_KMH,
            FRONT_VEHICLE_SPEED_KMH,
            ONCOMING_SPEED_KMH
        ))
        print("[启动延迟] 对向来车等待={}s | 行人等待={}s".format(
            ONCOMING_START_DELAY_S,
            PEDESTRIAN_START_DELAY_S
        ))

        # ====================================================
        # 13.8 主循环
        # ====================================================
        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            sim_time += DT
            frame_count += 1

            # -----------------------------
            # 行人延迟启动后移动
            # -----------------------------
            update_pedestrians(walkers, pedestrian_target_locs, sim_time)

            # -----------------------------
            # Ego：给定轨迹 PID 循迹 + 速度状态机
            # -----------------------------
            if reached_path_end(ego, ego_path, threshold=6.0):
                soft_hold_vehicle(ego)
                set_vehicle_lights(
                    ego,
                    brake=True,
                    low_beam=True
                )
                print("[场景结束] Ego 已到达轨迹终点附近。")
                break
            else:
                ego_target_speed = get_ego_target_speed(ego, sim_time, ego_speed_state)
                ego_idx = follow_path_constant_speed(
                    vehicle=ego,
                    path=ego_path,
                    path_index=ego_idx,
                    pid_lon=ego_pid_lon,
                    pid_lat=ego_pid_lat,
                    target_speed_kmh=ego_target_speed,
                    max_speed_kmh=EGO_MAX_SPEED_KMH,
                    min_lookahead=5.0,
                    lookahead_ratio=0.36
                )
                set_vehicle_lights(
                    ego,
                    brake=False,
                    low_beam=True
                )

            # -----------------------------
            # Mini 前车：固定 25 km/h 给定轨迹循迹，到达终点后停车
            # -----------------------------
            if front_vehicle_finished or reached_path_end(front_vehicle, front_vehicle_path, threshold=4.0):
                front_vehicle_finished = True
                soft_hold_vehicle(front_vehicle)
                set_vehicle_lights(
                    front_vehicle,
                    brake=True,
                    low_beam=True
                )
            else:
                front_vehicle_idx = follow_path_constant_speed(
                    vehicle=front_vehicle,
                    path=front_vehicle_path,
                    path_index=front_vehicle_idx,
                    pid_lon=front_pid_lon,
                    pid_lat=front_pid_lat,
                    target_speed_kmh=FRONT_VEHICLE_SPEED_KMH,
                    max_speed_kmh=FRONT_VEHICLE_MAX_SPEED_KMH,
                    min_lookahead=4.0,
                    lookahead_ratio=0.34
                )
                set_vehicle_lights(
                    front_vehicle,
                    brake=False,
                    low_beam=True
                )

            # -----------------------------
            # 对向来车：开始前静止，延迟后固定 45 km/h 循迹
            # -----------------------------
            if sim_time < ONCOMING_START_DELAY_S:
                hold_vehicle_before_release(oncoming)
                set_vehicle_lights(
                    oncoming,
                    brake=True,
                    low_beam=True
                )
            else:
                if not oncoming_released:
                    oncoming_released = True
                    release_vehicle(oncoming)
                    RTB.set_vehicle_initial_speed(
                        oncoming,
                        target_speed_kmh=ONCOMING_SPEED_KMH,
                        yaw_deg=ONCOMING_START_TF.rotation.yaw
                    )
                    print("[事件触发] 对向来车开始运动 | t={:.2f}s".format(sim_time))

                if reached_path_end(oncoming, oncoming_path, threshold=6.0):
                    soft_hold_vehicle(oncoming)
                    set_vehicle_lights(
                        oncoming,
                        brake=True,
                        low_beam=True
                    )
                else:
                    oncoming_idx = follow_path_constant_speed(
                        vehicle=oncoming,
                        path=oncoming_path,
                        path_index=oncoming_idx,
                        pid_lon=oncoming_pid_lon,
                        pid_lat=oncoming_pid_lat,
                        target_speed_kmh=ONCOMING_SPEED_KMH,
                        max_speed_kmh=ONCOMING_MAX_SPEED_KMH,
                        min_lookahead=5.5,
                        lookahead_ratio=0.42
                    )
                    set_vehicle_lights(
                        oncoming,
                        brake=False,
                        low_beam=True
                    )

            if frame_count % int(2.0 / DT) == 0:
                ped_state = "WAIT" if sim_time < PEDESTRIAN_START_DELAY_S else "MOVE"
                oncoming_state = "WAIT" if sim_time < ONCOMING_START_DELAY_S else "MOVE"
                print(
                    "[t={:05.2f}s | frame={:04d}] Ego={:05.1f}km/h | Front={:05.1f}km/h | Oncoming={:05.1f}km/h | idx(E/F/O)=({}/{}/{}) | EgoState={} | Ped={} | Oncoming={}".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(ego),
                        get_speed_kmh(front_vehicle),
                        get_speed_kmh(oncoming),
                        ego_idx,
                        front_vehicle_idx,
                        oncoming_idx,
                        ego_speed_state["phase"],
                        ped_state,
                        oncoming_state
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
            stop_pedestrians(walkers)
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
