# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

场景主题：
高速主路多车道行驶 + Ego 高速直行 + 右侧匝道汇入车辆加速并行冲突

元素：
1. 第一车道/最左车道车辆：
   - 使用用户给定轨迹锚点列表
   - 固定速度 110 km/h

2. Ego：
   - 起终点自动路径
   - 固定速度 120 km/h
   - 起点已更新为：
     Location: x=100.868, y=158.956, z=1.962
     Rotation: pitch=-5.273, yaw=-79.161, roll=0.000

3. 汇入车辆：
   - 起终点自动路径
   - 开始前静止等待 MERGE_START_DELAY_S 秒
   - 释放后从 30 km/h 逐渐加速到 80 km/h
   - 加速时长由 MERGE_ACCEL_DURATION_S 控制

要求：
- 严格同步模式
- 每辆车独立 PID
- 固定/规划速度循迹
- 无 debug 可视化绘制
"""

import sys
import os
import glob
import time
import math
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

ROUTE_RESOLUTION_M = 2.0
TRAJ_INTERVAL_M = 1.0

USE_LINEAR_FALLBACK_WHEN_ROUTE_FAIL = True


# ============================================================
# 2. 全局可调接口：车辆速度
# ============================================================

# 第一车道/最左车道车辆
LEFT_LANE_SPEED_KMH = 122.0
LEFT_LANE_MAX_SPEED_KMH = 125.0

# Ego
EGO_SPEED_KMH = 118.0
EGO_MAX_SPEED_KMH = 132.0

# 汇入车速度规划：
# 先等待，再从 30 km/h 逐渐加速到 80 km/h
MERGE_START_DELAY_S = 2.0
MERGE_INITIAL_SPEED_KMH = 30.0
MERGE_TARGET_SPEED_KMH = 80.0
MERGE_ACCEL_DURATION_S = 6.0
MERGE_MAX_SPEED_KMH = 92.0


# ============================================================
# 3. 全局可调接口：天气参数
# ============================================================

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
# 4. 全局可调接口：车辆蓝图
# ============================================================

LEFT_LANE_BP_CANDIDATES = [
    "vehicle.audi.etron",
    "vehicle.lincoln.mkz_2020",
    "vehicle.dodge.charger_2020",
    "vehicle.tesla.model3",
]

EGO_BP_CANDIDATES = [
    "vehicle.tesla.model3"
]

MERGE_BP_CANDIDATES = [
    "vehicle.toyota.prius",
    "vehicle.audi.tt",
    "vehicle.lincoln.mkz_2020",
    "vehicle.tesla.model3",
]


# ============================================================
# 5. Ego 起终点
# ============================================================

# 已按你的最新要求修改
EGO_START_TF = carla.Transform(
    carla.Location(x=100.455, y=162.456, z=1.962),
    carla.Rotation(pitch=0, yaw=-79.109, roll=0.000)
)

EGO_END_TF = carla.Transform(
    carla.Location(x=111.689, y=-389.262, z=1.711),
    carla.Rotation(pitch=0, yaw=-102.041, roll=0.000)
)


# ============================================================
# 6. 汇入车起终点
# ============================================================

MERGE_START_TF = carla.Transform(
    carla.Location(x=171.066, y=-57.086, z=1.782),
    carla.Rotation(pitch=-0.537, yaw=-93.465, roll=0.000)
)

MERGE_END_TF = carla.Transform(
    carla.Location(x=122.914, y=-355.315, z=1.410),
    carla.Rotation(pitch=-0.417, yaw=-103.502, roll=0.000)
)


# ============================================================
# 7. 第一车道/最左车道车辆轨迹锚点
# ============================================================

LEFT_LANE_RAW_TRAJ = [
    (95.696, 147.391, 0.649, -0.081, -82.671, 0.000),
    (95.696, 147.391, 0.649, 0.129, -83.580, 0.000),
    (95.792, 146.229, 0.683, 1.669, -85.260, 0.000),
    (97.735, 130.194, 1.027, 0.549, -80.149, 0.000),
    (100.249, 114.225, 1.061, -0.151, -81.339, 0.000),
    (103.080, 98.485, 1.166, 1.249, -74.967, 0.000),
    (107.800, 83.028, 1.085, -0.641, -73.775, 0.000),
    (112.453, 67.551, 1.012, 0.269, -72.514, 0.000),
    (117.260, 52.290, 1.088, 0.269, -72.514, 0.000),
    (122.314, 36.934, 1.163, 0.269, -71.534, 0.000),
    (127.961, 21.969, 1.253, 0.339, -67.195, 0.000),
    (134.009, 6.801, 1.241, -0.431, -69.553, 0.000),
    (139.888, -8.078, 1.275, 0.409, -67.384, 0.000),
    (144.977, -23.224, 1.245, -0.501, -75.782, 0.000),
    (147.191, -39.215, 1.101, -0.431, -84.724, 0.000),
    (148.781, -55.304, 1.049, -0.011, -84.094, 0.000),
    (150.620, -71.366, 1.108, 0.199, -83.533, 0.000),
    (152.047, -87.469, 1.093, -0.081, -85.563, 0.000),
    (153.139, -103.599, 1.055, -0.151, -85.772, 0.000),
    (155.227, -119.455, 1.023, -0.151, -80.872, 0.000),
    (156.291, -135.526, 0.987, -0.412, -94.699, 0.000),
    (154.129, -151.542, 0.860, -0.482, -99.809, 0.000),
    (151.389, -167.306, 0.726, -0.482, -99.739, 0.000),
    (148.909, -183.279, 0.876, 0.988, -99.110, 0.000),
    (146.089, -199.197, 0.989, -0.412, -101.070, 0.000),
    (142.926, -214.881, 0.901, 0.008, -101.281, 0.000),
    (139.534, -230.684, 0.719, 0.226, -102.396, 0.000),
    (138.799, -234.106, 0.764, 1.276, -101.837, 0.000),
    (138.799, -234.106, 0.764, 1.276, -101.837, 0.000),
]


# ============================================================
# 8. 基础工具函数
# ============================================================

def validate_user_inputs():
    if GlobalRoutePlanner is None:
        raise RuntimeError(
            "无法导入 CARLA GlobalRoutePlanner。请检查 CARLA_ROOT 是否正确，"
            "或者确认 PythonAPI\\carla\\agents\\navigation\\global_route_planner.py 存在。"
        )

    if len(LEFT_LANE_RAW_TRAJ) < 2:
        raise RuntimeError("LEFT_LANE_RAW_TRAJ 轨迹锚点不足。")

    if MERGE_ACCEL_DURATION_S <= 0.0:
        raise RuntimeError("MERGE_ACCEL_DURATION_S 必须大于 0。")


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


def make_transform_from_raw_point(p):
    return carla.Transform(
        carla.Location(x=p[0], y=p[1], z=p[2]),
        carla.Rotation(pitch=p[3], yaw=p[4], roll=p[5])
    )


def raw_traj_to_xyz(raw_traj):
    return [(p[0], p[1], p[2]) for p in raw_traj]


def build_dense_path_from_raw(raw_traj, interval=1.0):
    raw_points = raw_traj_to_xyz(raw_traj)
    raw_points = RTB.clean_trajectory(raw_points, min_dist=1e-5)
    dense = RTB.interpolate_trajectory(raw_points, interval=interval)
    dense = RTB.clean_trajectory(dense, min_dist=0.5)
    return dense


def build_linear_path_from_transforms(start_tf, end_tf, interval=1.0):
    raw_path = [
        (start_tf.location.x, start_tf.location.y, start_tf.location.z),
        (end_tf.location.x, end_tf.location.y, end_tf.location.z),
    ]
    dense = RTB.interpolate_trajectory(raw_path, interval=interval)
    dense = RTB.clean_trajectory(dense, min_dist=0.5)
    return dense


def get_path_end_location(path):
    p = path[-1]
    return carla.Location(x=p[0], y=p[1], z=p[2])


def reached_path_end(vehicle, path, threshold=8.0):
    if not vehicle or not vehicle.is_alive or not path:
        return True

    return distance_2d(vehicle.get_location(), get_path_end_location(path)) <= threshold


def describe_nearest_waypoint(carla_map, loc, name):
    try:
        wp = carla_map.get_waypoint(
            loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if wp is None:
            print("[路径诊断] {} nearest waypoint: None".format(name))
            return

        print(
            "[路径诊断] {} nearest waypoint | road_id={} | section_id={} | lane_id={} | s={:.3f} | loc=({:.3f}, {:.3f}, {:.3f})".format(
                name,
                wp.road_id,
                wp.section_id,
                wp.lane_id,
                wp.s,
                wp.transform.location.x,
                wp.transform.location.y,
                wp.transform.location.z
            )
        )
    except Exception as e:
        print("[路径诊断] {} waypoint 查询失败：{}".format(name, e))


def build_route_by_global_planner(carla_map, start_loc, end_loc, resolution=2.0):
    grp = GlobalRoutePlanner(carla_map, sampling_resolution=resolution)
    route = grp.trace_route(start_loc, end_loc)

    if not route:
        raise RuntimeError("GlobalRoutePlanner 未生成路线，请检查起终点是否在同一可达道路网络上。")

    path = []
    for wp, _ in route:
        loc = wp.transform.location
        path.append((loc.x, loc.y, loc.z))

    path = RTB.clean_trajectory(path, min_dist=0.5)
    return path


def build_route_with_diagnosis(
    route_name,
    carla_map,
    start_tf,
    end_tf,
    resolution=2.0,
    fallback_interval=1.0
):
    print("\n[路径构建] 开始构建 {} 路径。".format(route_name))
    print(
        "[路径构建] {} start=({:.3f}, {:.3f}, {:.3f}) yaw={:.3f}".format(
            route_name,
            start_tf.location.x,
            start_tf.location.y,
            start_tf.location.z,
            start_tf.rotation.yaw
        )
    )
    print(
        "[路径构建] {} end  =({:.3f}, {:.3f}, {:.3f}) yaw={:.3f}".format(
            route_name,
            end_tf.location.x,
            end_tf.location.y,
            end_tf.location.z,
            end_tf.rotation.yaw
        )
    )

    describe_nearest_waypoint(carla_map, start_tf.location, "{} 起点".format(route_name))
    describe_nearest_waypoint(carla_map, end_tf.location, "{} 终点".format(route_name))

    try:
        path = build_route_by_global_planner(
            carla_map=carla_map,
            start_loc=start_tf.location,
            end_loc=end_tf.location,
            resolution=resolution
        )

        print("[路径构建成功] {} | GlobalRoutePlanner path points={}".format(route_name, len(path)))
        return path, "global_route"

    except Exception as e:
        print("\n[路径构建失败] {} | GlobalRoutePlanner 无法连通该起终点。".format(route_name))
        print("[路径构建失败] 异常类型：{}".format(type(e).__name__))
        print("[路径构建失败] 异常信息：{}".format(e))
        traceback.print_exc()

        if USE_LINEAR_FALLBACK_WHEN_ROUTE_FAIL:
            print("[路径兜底] {} 使用起终点直线插值路径继续运行。".format(route_name))
            path = build_linear_path_from_transforms(
                start_tf,
                end_tf,
                interval=fallback_interval
            )
            print("[路径兜底成功] {} | linear path points={}".format(route_name, len(path)))
            return path, "linear_fallback"

        raise


def spawn_vehicle_by_tf(
    world,
    candidates,
    tf,
    color=None,
    role_name="background",
    z_offset=0.8,
    extra_z_offsets=None,
    xy_retry_offsets=None,
    yaw_retry_offsets=None
):
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


def get_merge_target_speed(sim_time):
    """
    汇入车速度规划：
    1. sim_time < MERGE_START_DELAY_S：静止等待；
    2. 释放后从 MERGE_INITIAL_SPEED_KMH 线性加速到 MERGE_TARGET_SPEED_KMH；
    3. 加速结束后保持 MERGE_TARGET_SPEED_KMH。
    """
    if sim_time < MERGE_START_DELAY_S:
        return 0.0

    t = sim_time - MERGE_START_DELAY_S

    if t >= MERGE_ACCEL_DURATION_S:
        return MERGE_TARGET_SPEED_KMH

    ratio = t / MERGE_ACCEL_DURATION_S
    return MERGE_INITIAL_SPEED_KMH + (
        MERGE_TARGET_SPEED_KMH - MERGE_INITIAL_SPEED_KMH
    ) * ratio


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
        "[天气设置] Clouds={} | Rain={} | Wetness={}".format(
            cloudiness,
            precipitation,
            wetness
        )
    )


# ============================================================
# 10. 车辆循迹控制函数
# ============================================================

def follow_path_constant_speed(
    vehicle,
    path,
    path_index,
    pid_lon,
    pid_lat,
    target_speed_kmh,
    max_speed_kmh,
    min_lookahead=6.0,
    lookahead_ratio=0.42,
    max_search_ahead=80,
    fallback_dist=60.0
):
    if not vehicle or not vehicle.is_alive or not path:
        return path_index

    current_speed = get_speed_kmh(vehicle)

    if current_speed > max_speed_kmh + 1.0:
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=0.35,
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

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    try:
        validate_user_inputs()

        world = client.get_world()
        carla_map = world.get_map()

        # ====================================================
        # 11.1 同步模式 + 地图缓存 + 天气
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
        # 11.2 构建路径
        # ====================================================
        print("\n================ 路径构建阶段 ================")

        left_lane_path = build_dense_path_from_raw(
            LEFT_LANE_RAW_TRAJ,
            interval=TRAJ_INTERVAL_M
        )
        print("[路径构建成功] LeftLane | manual_anchor | points={}".format(len(left_lane_path)))

        ego_path, ego_path_type = build_route_with_diagnosis(
            route_name="Ego",
            carla_map=carla_map,
            start_tf=EGO_START_TF,
            end_tf=EGO_END_TF,
            resolution=ROUTE_RESOLUTION_M,
            fallback_interval=TRAJ_INTERVAL_M
        )

        merge_path, merge_path_type = build_route_with_diagnosis(
            route_name="MergeVehicle",
            carla_map=carla_map,
            start_tf=MERGE_START_TF,
            end_tf=MERGE_END_TF,
            resolution=ROUTE_RESOLUTION_M,
            fallback_interval=TRAJ_INTERVAL_M
        )

        print("\n================ 路径构建结果 ================")
        print("[路径结果] LeftLane     : manual_anchor | points={}".format(len(left_lane_path)))
        print("[路径结果] Ego          : {} | points={}".format(ego_path_type, len(ego_path)))
        print("[路径结果] MergeVehicle : {} | points={}".format(merge_path_type, len(merge_path)))
        print("================================================\n")

        # ====================================================
        # 11.3 生成车辆
        # ====================================================
        left_lane_start_tf = make_transform_from_raw_point(LEFT_LANE_RAW_TRAJ[0])

        left_lane_vehicle = spawn_vehicle_by_tf(
            world=world,
            candidates=LEFT_LANE_BP_CANDIDATES,
            tf=left_lane_start_tf,
            color="255,255,255",
            role_name="left_lane_fast_vehicle",
            z_offset=0.90,
            extra_z_offsets=[0.80, 0.90, 1.10, 1.30, 1.50],
        )
        if not left_lane_vehicle:
            raise RuntimeError("第一车道车辆生成失败。")
        actor_list.append(left_lane_vehicle)

        ego = spawn_vehicle_by_tf(
            world=world,
            candidates=EGO_BP_CANDIDATES,
            tf=EGO_START_TF,
            color="255,80,255",
            role_name="ego",
            z_offset=0.90,
            extra_z_offsets=[0.80, 0.90, 1.10, 1.30, 1.50],
        )
        if not ego:
            raise RuntimeError("Ego 生成失败。")
        actor_list.append(ego)

        merge_vehicle = spawn_vehicle_by_tf(
            world=world,
            candidates=MERGE_BP_CANDIDATES,
            tf=MERGE_START_TF,
            color="40,40,40",
            role_name="merging_vehicle",
            z_offset=0.90,
            extra_z_offsets=[0.80, 0.90, 1.10, 1.30, 1.50],
        )
        if not merge_vehicle:
            raise RuntimeError("汇入车辆生成失败。")
        actor_list.append(merge_vehicle)

        # ====================================================
        # 11.4 灯光
        # ====================================================
        set_vehicle_lights(left_lane_vehicle, brake=False, low_beam=True)
        set_vehicle_lights(ego, brake=False, low_beam=True)
        set_vehicle_lights(merge_vehicle, brake=True, low_beam=True)

        # ====================================================
        # 11.5 每辆车独立 PID
        # ====================================================
        left_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.75),
            i_clip=(-1.0, 1.0)
        )
        left_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.55, 0.55),
            i_clip=(-1.0, 1.0)
        )

        ego_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.95, 0.78),
            i_clip=(-1.0, 1.0)
        )
        ego_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.55, 0.55),
            i_clip=(-1.0, 1.0)
        )

        merge_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.65),
            i_clip=(-1.0, 1.0)
        )
        merge_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.50, 0.50),
            i_clip=(-1.0, 1.0)
        )

        left_idx = 0
        ego_idx = 0
        merge_idx = 0
        merge_released = False

        # ====================================================
        # 11.6 Actor 生成后预热与初速度注入
        # ====================================================
        print("[预热] Actor 生成后同步预热。")
        sim_time = 0.0

        for _ in range(20):
            soft_hold_vehicle(merge_vehicle, hand_brake=True)
            world.tick()
            sim_time += DT
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            left_lane_vehicle,
            target_speed_kmh=LEFT_LANE_SPEED_KMH,
            yaw_deg=left_lane_vehicle.get_transform().rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_SPEED_KMH,
            yaw_deg=ego.get_transform().rotation.yaw
        )

        # 汇入车不在这里注入速度，先保持静止等待
        soft_hold_vehicle(merge_vehicle, hand_brake=True)

        for _ in range(5):
            soft_hold_vehicle(merge_vehicle, hand_brake=True)
            world.tick()
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print(
            "[速度设置] LeftLane={} km/h | Ego={} km/h | Merge={}→{} km/h | MergeDelay={}s | AccelDuration={}s".format(
                LEFT_LANE_SPEED_KMH,
                EGO_SPEED_KMH,
                MERGE_INITIAL_SPEED_KMH,
                MERGE_TARGET_SPEED_KMH,
                MERGE_START_DELAY_S,
                MERGE_ACCEL_DURATION_S
            )
        )

        # ====================================================
        # 11.7 主循环
        # ====================================================
        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            sim_time += DT
            frame_count += 1

            # 第一车道车辆：固定 110 km/h
            if reached_path_end(left_lane_vehicle, left_lane_path, threshold=8.0):
                soft_hold_vehicle(left_lane_vehicle)
                set_vehicle_lights(left_lane_vehicle, brake=True, low_beam=True)
            else:
                left_idx = follow_path_constant_speed(
                    vehicle=left_lane_vehicle,
                    path=left_lane_path,
                    path_index=left_idx,
                    pid_lon=left_pid_lon,
                    pid_lat=left_pid_lat,
                    target_speed_kmh=LEFT_LANE_SPEED_KMH,
                    max_speed_kmh=LEFT_LANE_MAX_SPEED_KMH,
                    min_lookahead=8.0,
                    lookahead_ratio=0.50,
                    max_search_ahead=90,
                    fallback_dist=70.0
                )
                set_vehicle_lights(left_lane_vehicle, brake=False, low_beam=True)

            # Ego：固定 120 km/h
            if reached_path_end(ego, ego_path, threshold=10.0):
                soft_hold_vehicle(ego)
                set_vehicle_lights(ego, brake=True, low_beam=True)
                print("[场景结束] Ego 已到达终点附近。")
                break
            else:
                ego_idx = follow_path_constant_speed(
                    vehicle=ego,
                    path=ego_path,
                    path_index=ego_idx,
                    pid_lon=ego_pid_lon,
                    pid_lat=ego_pid_lat,
                    target_speed_kmh=EGO_SPEED_KMH,
                    max_speed_kmh=EGO_MAX_SPEED_KMH,
                    min_lookahead=9.0,
                    lookahead_ratio=0.52,
                    max_search_ahead=100,
                    fallback_dist=80.0
                )
                set_vehicle_lights(ego, brake=False, low_beam=True)

            # 汇入车：先静止等待，然后 30 km/h 逐渐加速到 80 km/h
            merge_target_speed = get_merge_target_speed(sim_time)

            if sim_time < MERGE_START_DELAY_S:
                soft_hold_vehicle(merge_vehicle, hand_brake=True)
                set_vehicle_lights(merge_vehicle, brake=True, low_beam=True)

            else:
                if not merge_released:
                    merge_released = True

                    merge_vehicle.apply_control(
                        carla.VehicleControl(
                            throttle=0.0,
                            brake=0.0,
                            steer=0.0,
                            hand_brake=False
                        )
                    )

                    RTB.set_vehicle_initial_speed(
                        merge_vehicle,
                        target_speed_kmh=MERGE_INITIAL_SPEED_KMH,
                        yaw_deg=merge_vehicle.get_transform().rotation.yaw
                    )

                    print(
                        "[事件触发] 汇入车释放 | delay={:.2f}s | initial_speed={} km/h | target_speed={} km/h | accel_duration={}s".format(
                            MERGE_START_DELAY_S,
                            MERGE_INITIAL_SPEED_KMH,
                            MERGE_TARGET_SPEED_KMH,
                            MERGE_ACCEL_DURATION_S
                        )
                    )

                if reached_path_end(merge_vehicle, merge_path, threshold=8.0):
                    soft_hold_vehicle(merge_vehicle)
                    set_vehicle_lights(merge_vehicle, brake=True, low_beam=True)
                else:
                    merge_idx = follow_path_constant_speed(
                        vehicle=merge_vehicle,
                        path=merge_path,
                        path_index=merge_idx,
                        pid_lon=merge_pid_lon,
                        pid_lat=merge_pid_lat,
                        target_speed_kmh=merge_target_speed,
                        max_speed_kmh=MERGE_MAX_SPEED_KMH,
                        min_lookahead=7.0,
                        lookahead_ratio=0.45,
                        max_search_ahead=80,
                        fallback_dist=60.0
                    )
                    set_vehicle_lights(merge_vehicle, brake=False, low_beam=True)

            if frame_count % int(2.0 / DT) == 0:
                print(
                    "[t={:05.2f}s | frame={:04d}] Left={:05.1f}km/h | Ego={:05.1f}km/h | Merge={:05.1f}km/h(target={:05.1f}) | idx(L/E/M)=({}/{}/{})".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(left_lane_vehicle),
                        get_speed_kmh(ego),
                        get_speed_kmh(merge_vehicle),
                        merge_target_speed,
                        left_idx,
                        ego_idx,
                        merge_idx
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
