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
6. 无 debug 可视化绘制。
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

ROUTE_RESOLUTION_M = 2.0
TRAJ_INTERVAL_M = 0.8

USE_LINEAR_FALLBACK_WHEN_ROUTE_FAIL = True


# ============================================================
# 2. 全局可调接口：速度
# ============================================================

EGO_SPEED_KMH = 40.0
EGO_MAX_SPEED_KMH = 58.0

FRONT_SLOW_SPEED_KMH = 20.0
FRONT_SLOW_MAX_SPEED_KMH = 32.0

BICYCLE_SPEED_KMH = 20.0
BICYCLE_MAX_SPEED_KMH = 22.0

EXTRA_VEHICLE_SPEED_KMH = 35.0
EXTRA_VEHICLE_MAX_SPEED_KMH = 45.0


# ============================================================
# 3. 天气参数：ClearSunset
# ============================================================

WEATHER_PRESET = "ClearSunset"

WEATHER_CLOUDINESS = 5.0
WEATHER_PRECIPITATION = 0.0
WEATHER_PUDDLES = 0.0
WEATHER_WIND = 10.0

WEATHER_SUN_AZIMUTH = 0.0
WEATHER_SUN_ALTITUDE = 9.0

WEATHER_FOG_DENSITY = 2.0
WEATHER_FOG_DISTANCE = 1.750
WEATHER_FOG_FALLOFF = 0.100

WEATHER_WETNESS = 0.0

WEATHER_SCATTERING = 2.500
WEATHER_MIE = 0.030
WEATHER_RAYLEIGH = 0.1331
WEATHER_DUST = 0.0


# ============================================================
# 4. 蓝图候选
# ============================================================

EGO_BP_CANDIDATES = [
    "vehicle.nissan.patrol",
    "vehicle.audi.etron",
    "vehicle.tesla.model3",
    "vehicle.lincoln.mkz_2020",
]

FRONT_SLOW_BP_CANDIDATES = [
    "vehicle.toyota.prius",
    "vehicle.lincoln.mkz_2020",
    "vehicle.audi.tt",
    "vehicle.tesla.model3",
]

EXTRA_VEHICLE_BP_CANDIDATES = [
    "vehicle.audi.tt",
    "vehicle.lincoln.mkz_2020",
    "vehicle.toyota.prius",
    "vehicle.tesla.model3",
]

BICYCLE_BP_CANDIDATES_GROUP = [
    [
        "vehicle.bh.crossbike",
        "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets",
        "vehicle.yamaha.yzf",
    ],
    [
        "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets",
        "vehicle.bh.crossbike",
        "vehicle.yamaha.yzf",
    ],
    [
        "vehicle.gazelle.omafiets",
        "vehicle.bh.crossbike",
        "vehicle.diamondback.century",
        "vehicle.yamaha.yzf",
    ],
]


# ============================================================
# 5. Ego 起终点
# ============================================================

EGO_START_TF = carla.Transform(
    carla.Location(x=132.767, y=-10.725, z=1.547),
    carla.Rotation(pitch=-9.958, yaw=168.114, roll=0.000)
)

EGO_END_TF = carla.Transform(
    carla.Location(x=-61.967, y=-2.817, z=2.056),
    carla.Rotation(pitch=-1.800, yaw=178.767, roll=0.000)
)


# ============================================================
# 6. 慢速前车起终点
# ============================================================

FRONT_SLOW_START_TF = carla.Transform(
    carla.Location(x=97.901, y=-4.977, z=1.032),
    carla.Rotation(pitch=7.945, yaw=163.451, roll=0.000)
)

FRONT_SLOW_END_TF = carla.Transform(
    carla.Location(x=-39.216, y=-2.283, z=1.588),
    carla.Rotation(pitch=0.372, yaw=-177.648, roll=0.000)
)


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
# 8. 三辆自行车/电动车参数
# ============================================================

ENABLE_BICYCLES = False
BICYCLE_COUNT = 0

# 建议主要通过纵向拉开，避免互相碰撞
BICYCLE_LATERAL_OFFSETS_M = [ 0.00, 1.20]
BICYCLE_LONGITUDINAL_OFFSETS_M = [-4.00, -0.00]
BICYCLE_SPAWN_Z_OFFSET = 1.20

BICYCLE_RAW_TRAJ = [
    (81.171, -10.119, 2.783, -19.383, 178.855, 0.000),
    (79.108, -10.131, 1.833, -22.017, -179.937, 0.000),
    (77.967, -10.106, 1.753, -3.978, 178.751, 0.000),
    (74.624, -10.034, 1.545, -3.208, 178.751, 0.000),
    (70.254, -9.950, 1.410, -0.548, 179.031, 0.000),
    (63.579, -9.837, 1.346, 0.432, 177.631, 0.000),
    (59.099, -9.710, 1.326, -1.038, 179.870, 0.000),
    (55.722, -9.702, 1.265, -1.038, 179.800, 0.000),
    (50.085, -9.683, 1.163, -1.038, 179.730, 0.000),
    (50.085, -9.683, 1.163, -0.461, 164.816, 0.000),
    (50.085, -9.683, 1.163, -1.090, 159.076, 0.000),
    (50.085, -9.683, 1.163, -1.090, 159.076, 0.000),
    (50.085, -9.683, 1.163, -1.090, 159.076, 0.000),
]


# ============================================================
# 9. 新增车辆轨迹
# ============================================================

EXTRA_VEHICLE_RAW_TRAJ = [
    (80.134, -2.806, 0.895, 3.390, -178.425, 0.000),
    (80.134, -2.806, 0.895, 3.390, -178.425, 0.000),
    (80.134, -2.806, 0.895, 3.530, -178.565, 0.000),
    (72.375, -2.800, 1.429, 3.460, -178.705, 0.000),
    (59.827, -2.679, 1.803, 1.080, 179.475, 0.000),
    (48.405, -2.721, 1.889, 0.100, -178.985, 0.000),
    (32.028, -2.983, 1.846, -0.390, -179.755, 0.000),
    (17.633, -3.033, 1.740, -0.530, -179.965, 0.000),
    (11.697, -2.945, 1.560, -1.738, 179.150, 0.000),
    (9.194, -2.866, 1.473, -3.336, 171.105, 0.000),
    (4.817, -1.183, 1.211, -3.125, 151.073, 0.000),
    (2.946, 1.642, 1.069, -1.866, 114.393, 0.000),
    (2.566, 3.719, 1.022, 1.485, 55.598, 0.000),
    (6.081, 5.448, 1.228, 2.112, 4.935, 0.000),
    (13.331, 3.976, 1.340, 0.611, -7.570, 0.000),
    (19.697, 3.607, 1.349, -0.673, -7.224, 0.000),
    (29.374, 2.585, 1.352, 2.273, 0.291, 0.000),
    (37.743, 2.894, 1.637, 0.243, 3.160, 0.000),
    (37.743, 2.894, 1.637, 0.243, 3.160, 0.000),
    (37.743, 2.894, 1.637, 0.243, 3.160, 0.000),
    (37.743, 2.894, 1.637, 0.243, 3.160, 0.000),
    (37.743, 2.894, 1.637, 0.243, 3.160, 0.000),
    (37.743, 2.894, 1.637, 0.243, 3.160, 0.000),
]


# ============================================================
# 10. 基础工具函数
# ============================================================

def validate_user_inputs():
    if GlobalRoutePlanner is None:
        raise RuntimeError(
            "无法导入 CARLA GlobalRoutePlanner。请检查 CARLA_ROOT 是否正确。"
        )

    if PEDESTRIAN_COUNT != 8:
        raise RuntimeError("本场景要求 PEDESTRIAN_COUNT = 8。")

    if ENABLE_BICYCLES:
        if BICYCLE_COUNT < 1:
            raise RuntimeError("启用两轮车时，BICYCLE_COUNT 至少为 1。")

        if len(BICYCLE_RAW_TRAJ) < 2:
            raise RuntimeError("BICYCLE_RAW_TRAJ 锚点不足。")

    if len(EXTRA_VEHICLE_RAW_TRAJ) < 2:
        raise RuntimeError("EXTRA_VEHICLE_RAW_TRAJ 锚点不足。")


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
        raise RuntimeError("GlobalRoutePlanner 未生成路线。")

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
                    if _rtb_opt_goal_guard(locals(), client, world):
                        break
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


def offset_raw_traj_by_lateral_and_longitudinal(raw_traj, lateral_offset, longitudinal_offset):
    result = []
    for p in raw_traj:
        x, y, z, pitch, yaw, roll = p

        yaw_rad = math.radians(yaw)
        forward_x = math.cos(yaw_rad)
        forward_y = math.sin(yaw_rad)

        left_x = -math.sin(yaw_rad)
        left_y = math.cos(yaw_rad)

        ox = x + left_x * lateral_offset + forward_x * longitudinal_offset
        oy = y + left_y * lateral_offset + forward_y * longitudinal_offset

        result.append((ox, oy, z, pitch, yaw, roll))

    return result


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
        preset=WEATHER_PRESET,
        cloudiness=WEATHER_CLOUDINESS,
        precipitation=WEATHER_PRECIPITATION,
        precipitation_deposits=WEATHER_PUDDLES,
        wind_intensity=WEATHER_WIND,
        sun_azimuth_angle=WEATHER_SUN_AZIMUTH,
        sun_altitude_angle=WEATHER_SUN_ALTITUDE,
        fog_density=WEATHER_FOG_DENSITY,
        fog_distance=WEATHER_FOG_DISTANCE,
        fog_falloff=WEATHER_FOG_FALLOFF,
        wetness=WEATHER_WETNESS,
        scattering_intensity=WEATHER_SCATTERING,
        mie_scattering_scale=WEATHER_MIE,
        rayleigh_scattering_scale=WEATHER_RAYLEIGH,
        dust_storm=WEATHER_DUST
    )

    print(
        "[天气设置] {} | Clouds={} | Rain={} | Puddles={} | Wind={} | SunAzim={} | SunAlt={} | FogDens={} | FogDist={} | Wetness={} | Scatter={} | Mie={} | Rayleigh={} | Dust={}".format(
            WEATHER_PRESET,
            WEATHER_CLOUDINESS,
            WEATHER_PRECIPITATION,
            WEATHER_PUDDLES,
            WEATHER_WIND,
            WEATHER_SUN_AZIMUTH,
            WEATHER_SUN_ALTITUDE,
            WEATHER_FOG_DENSITY,
            WEATHER_FOG_DISTANCE,
            WEATHER_WETNESS,
            WEATHER_SCATTERING,
            WEATHER_MIE,
            WEATHER_RAYLEIGH,
            WEATHER_DUST
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



# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-61.967, -2.817)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.nissan.patrol'
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
    actor_list = []
    world = None

    pedestrians = []

    bicycle_actors = []
    bicycle_paths = []
    bicycle_indices = []
    bicycle_pid_lons = []
    bicycle_pid_lats = []

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
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            time.sleep(DT)

        # ====================================================
        # 路径构建
        # ====================================================
        print("\n================ 路径构建阶段 ================")

        ego_path, ego_path_type = build_route_with_diagnosis(
            route_name="Ego",
            carla_map=carla_map,
            start_tf=EGO_START_TF,
            end_tf=EGO_END_TF,
            resolution=ROUTE_RESOLUTION_M,
            fallback_interval=TRAJ_INTERVAL_M
        )

        front_slow_path, front_slow_path_type = build_route_with_diagnosis(
            route_name="FrontSlow",
            carla_map=carla_map,
            start_tf=FRONT_SLOW_START_TF,
            end_tf=FRONT_SLOW_END_TF,
            resolution=ROUTE_RESOLUTION_M,
            fallback_interval=TRAJ_INTERVAL_M
        )

        extra_vehicle_path = build_dense_path_from_raw(EXTRA_VEHICLE_RAW_TRAJ, interval=TRAJ_INTERVAL_M)
        print("[路径构建成功] ExtraVehicle | manual path points={}".format(len(extra_vehicle_path)))

        base_bicycle_paths = []

        if ENABLE_BICYCLES:
            for i in range(BICYCLE_COUNT):
                raw_offset = offset_raw_traj_by_lateral_and_longitudinal(
                    BICYCLE_RAW_TRAJ,
                    lateral_offset=BICYCLE_LATERAL_OFFSETS_M[i],
                    longitudinal_offset=BICYCLE_LONGITUDINAL_OFFSETS_M[i]
                )
                path = build_dense_path_from_raw(raw_offset, interval=0.6)
                base_bicycle_paths.append((raw_offset, path))
                print("[路径构建成功] Bicycle{} | manual path points={}".format(i + 1, len(path)))
        else:
            print("[两轮车配置] ENABLE_BICYCLES=False，跳过两轮车路径构建。")
        print("\n================ 路径构建结果 ================")
        print("[路径结果] Ego         : {} | points={}".format(ego_path_type, len(ego_path)))
        print("[路径结果] FrontSlow   : {} | points={}".format(front_slow_path_type, len(front_slow_path)))
        print("[路径结果] ExtraVehicle: manual_anchor | points={}".format(len(extra_vehicle_path)))
        if ENABLE_BICYCLES:
            for i, (_, p) in enumerate(base_bicycle_paths):
                print("[路径结果] Bicycle{}    : manual_anchor | points={}".format(i + 1, len(p)))
        else:
            print("[路径结果] Bicycles    : disabled")
        print("================================================\n")

        # ====================================================
        # 车辆生成
        # ====================================================
        ego = spawn_vehicle_by_tf(
            world=world,
            candidates=EGO_BP_CANDIDATES,
            tf=EGO_START_TF,
            color="0,80,255",
            role_name="ego",
            z_offset=1.20,
            extra_z_offsets=[1.00, 1.20, 1.40, 1.60, 1.80, 2.00],
        )
        if not ego:
            raise RuntimeError("Ego 生成失败。")
        actor_list.append(ego)

        front_slow = spawn_vehicle_by_tf(
            world=world,
            candidates=FRONT_SLOW_BP_CANDIDATES,
            tf=FRONT_SLOW_START_TF,
            color="255,255,255",
            role_name="front_slow_vehicle",
            z_offset=1.10,
            extra_z_offsets=[0.90, 1.10, 1.30, 1.50, 1.70],
        )
        if not front_slow:
            raise RuntimeError("慢速前车生成失败。")
        actor_list.append(front_slow)

        extra_start_tf = make_transform_from_raw_point(EXTRA_VEHICLE_RAW_TRAJ[0])
        extra_vehicle = spawn_vehicle_by_tf(
            world=world,
            candidates=EXTRA_VEHICLE_BP_CANDIDATES,
            tf=extra_start_tf,
            color="80,80,80",
            role_name="extra_dynamic_vehicle",
            z_offset=1.00,
            extra_z_offsets=[0.90, 1.10, 1.30, 1.50],
        )
        if not extra_vehicle:
            raise RuntimeError("新增动态车辆生成失败。")
        actor_list.append(extra_vehicle)

        # 三辆两轮车生成
        for i in range(BICYCLE_COUNT):
            raw_offset, path = base_bicycle_paths[i]
            start_tf = make_transform_from_raw_point(raw_offset[0])

            bike = spawn_vehicle_by_tf(
                world=world,
                candidates=BICYCLE_BP_CANDIDATES_GROUP[i],
                tf=start_tf,
                color=None,
                role_name="bicycle_or_ebike_{}".format(i + 1),
                z_offset=BICYCLE_SPAWN_Z_OFFSET,
                extra_z_offsets=[1.00, 1.20, 1.40, 1.60],
                xy_retry_offsets=[
                    (0.0, 0.0),
                    (0.3, 0.0),
                    (-0.3, 0.0),
                    (0.0, 0.3),
                    (0.0, -0.3),
                    (0.6, 0.0),
                    (-0.6, 0.0),
                    (0.0, 0.6),
                    (0.0, -0.6),
                ],
                yaw_retry_offsets=[0.0, 2.0, -2.0]
            )

            if not bike:
                raise RuntimeError("第 {} 辆自行车/电动车生成失败。".format(i + 1))

            actor_list.append(bike)
            bicycle_actors.append(bike)
            bicycle_paths.append(path)
            bicycle_indices.append(0)

            bicycle_pid_lons.append(
                RTB.PIDLongitudinalController(
                    dt=DT,
                    preset="default_car",
                    output_clip=(-0.70, 0.35),
                    i_clip=(-0.6, 0.6)
                )
            )
            bicycle_pid_lats.append(
                RTB.PIDLateralController(
                    dt=DT,
                    preset="default_car",
                    output_clip=(-0.32, 0.32),
                    i_clip=(-0.6, 0.6)
                )
            )

        if ENABLE_PEDESTRIANS:
            pedestrians = spawn_pedestrians(world, actor_list)

        # ====================================================
        # 灯光
        # ====================================================
        set_vehicle_lights(ego, brake=False, hazard=False, low_beam=True)
        set_vehicle_lights(front_slow, brake=False, hazard=False, low_beam=True)
        set_vehicle_lights(extra_vehicle, brake=False, hazard=False, low_beam=True)

        for bike in bicycle_actors:
            set_vehicle_lights(bike, brake=False, hazard=False, low_beam=False)

        # ====================================================
        # PID
        # ====================================================
        ego_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.65),
            i_clip=(-1.0, 1.0)
        )
        ego_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.50, 0.50),
            i_clip=(-1.0, 1.0)
        )

        front_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.85, 0.55),
            i_clip=(-1.0, 1.0)
        )
        front_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.45, 0.45),
            i_clip=(-1.0, 1.0)
        )

        extra_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.85, 0.55),
            i_clip=(-1.0, 1.0)
        )
        extra_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.45, 0.45),
            i_clip=(-1.0, 1.0)
        )

        ego_idx = 0
        front_idx = 0
        extra_idx = 0

        # ====================================================
        # 预热与初速度
        # ====================================================
        print("[预热] Actor 生成后同步预热。")
        sim_time = 0.0

        for _ in range(20):
            update_pedestrians(pedestrians, sim_time)
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += DT
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_SPEED_KMH,
            yaw_deg=ego.get_transform().rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            front_slow,
            target_speed_kmh=FRONT_SLOW_SPEED_KMH,
            yaw_deg=front_slow.get_transform().rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            extra_vehicle,
            target_speed_kmh=EXTRA_VEHICLE_SPEED_KMH,
            yaw_deg=extra_vehicle.get_transform().rotation.yaw
        )

        for bike in bicycle_actors:
            RTB.set_vehicle_initial_speed(
                bike,
                target_speed_kmh=BICYCLE_SPEED_KMH,
                yaw_deg=bike.get_transform().rotation.yaw
            )

        for _ in range(5):
            update_pedestrians(pedestrians, sim_time)
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print("[速度设置] Ego={} km/h | FrontSlow={} km/h | ExtraVehicle={} km/h | Bicycle={} km/h".format(
            EGO_SPEED_KMH,
            FRONT_SLOW_SPEED_KMH,
            EXTRA_VEHICLE_SPEED_KMH,
            BICYCLE_SPEED_KMH
        ))

        # ====================================================
        # 主循环
        # ====================================================
        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += DT
            frame_count += 1

            update_pedestrians(pedestrians, sim_time)

            # 慢速前车
            if reached_path_end(front_slow, front_slow_path, threshold=6.0):
                soft_hold_vehicle(front_slow)
                set_vehicle_lights(front_slow, brake=True, low_beam=True)
            else:
                front_idx = follow_path_constant_speed(
                    vehicle=front_slow,
                    path=front_slow_path,
                    path_index=front_idx,
                    pid_lon=front_pid_lon,
                    pid_lat=front_pid_lat,
                    target_speed_kmh=FRONT_SLOW_SPEED_KMH,
                    max_speed_kmh=FRONT_SLOW_MAX_SPEED_KMH,
                    min_lookahead=4.5,
                    lookahead_ratio=0.36,
                    max_search_ahead=55,
                    fallback_dist=40.0
                )
                set_vehicle_lights(front_slow, brake=False, low_beam=True)

            # Ego
            if reached_path_end(ego, ego_path, threshold=7.0):
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
                    min_lookahead=6.5,
                    lookahead_ratio=0.42,
                    max_search_ahead=75,
                    fallback_dist=55.0
                )
                set_vehicle_lights(ego, brake=False, low_beam=True)

            # 新增动态车辆
            if reached_path_end(extra_vehicle, extra_vehicle_path, threshold=5.0):
                soft_hold_vehicle(extra_vehicle)
                set_vehicle_lights(extra_vehicle, brake=True, low_beam=True)
            else:
                extra_idx = follow_path_constant_speed(
                    vehicle=extra_vehicle,
                    path=extra_vehicle_path,
                    path_index=extra_idx,
                    pid_lon=extra_pid_lon,
                    pid_lat=extra_pid_lat,
                    target_speed_kmh=EXTRA_VEHICLE_SPEED_KMH,
                    max_speed_kmh=EXTRA_VEHICLE_MAX_SPEED_KMH,
                    min_lookahead=5.0,
                    lookahead_ratio=0.38,
                    max_search_ahead=55,
                    fallback_dist=40.0
                )
                set_vehicle_lights(extra_vehicle, brake=False, low_beam=True)

            # 三辆两轮车
            for i, bike in enumerate(bicycle_actors):
                if not bike or not bike.is_alive:
                    continue

                if reached_path_end(bike, bicycle_paths[i], threshold=2.5):
                    soft_hold_vehicle(bike)
                    continue

                bicycle_indices[i] = follow_path_constant_speed(
                    vehicle=bike,
                    path=bicycle_paths[i],
                    path_index=bicycle_indices[i],
                    pid_lon=bicycle_pid_lons[i],
                    pid_lat=bicycle_pid_lats[i],
                    target_speed_kmh=BICYCLE_SPEED_KMH,
                    max_speed_kmh=BICYCLE_MAX_SPEED_KMH,
                    min_lookahead=2.5,
                    lookahead_ratio=0.32,
                    max_search_ahead=40,
                    fallback_dist=28.0
                )

            if frame_count % int(2.0 / DT) == 0:
                ped_state = "WAIT" if sim_time < PEDESTRIAN_START_DELAY_S else "MOVE"
                bike_speeds = [get_speed_kmh(b) for b in bicycle_actors]

                print(
                    "[t={:05.2f}s | frame={:04d}] Ego={:05.1f} | Front={:05.1f} | Extra={:05.1f} | Bikes=({:04.1f},{:04.1f},{:04.1f}) | idx(E/F/X/B)=({}/{}/{}/{}) | Ped={}".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(ego),
                        get_speed_kmh(front_slow),
                        get_speed_kmh(extra_vehicle),
                        bike_speeds[0] if len(bike_speeds) > 0 else 0.0,
                        bike_speeds[1] if len(bike_speeds) > 1 else 0.0,
                        bike_speeds[2] if len(bike_speeds) > 2 else 0.0,
                        ego_idx,
                        front_idx,
                        extra_idx,
                        bicycle_indices,
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