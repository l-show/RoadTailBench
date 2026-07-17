# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

场景主题：
限界侵入：滨水乡村弯道右侧构筑物侵限 + 前车左偏诱导 + Ego 跟随误判 + 行人干扰

场景元素：
1. 前车：沿用户给定锚点轨迹行驶，固定 60 km/h。
2. Ego：从用户给定起点到终点行驶，固定 60 km/h。
3. 行人：带可调等待时间，等待后从起点移动至终点。
4. 严格同步模式。
5. 独立 PID。
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
SCENARIO_DURATION = 55.0
KEEP_ACTORS_AFTER_SCRIPT = False

ROUTE_RESOLUTION_M = 2.0
TRAJ_INTERVAL_M = 1.0

# 如果 GlobalRoutePlanner 路径构建失败，是否自动退回为起终点直线插值路径
USE_LINEAR_FALLBACK_WHEN_ROUTE_FAIL = True


# ============================================================
# 2. 全局可调接口：车辆速度
# ============================================================

FRONT_VEHICLE_SPEED_KMH = 60.0
FRONT_VEHICLE_MAX_SPEED_KMH = 70.0

EGO_SPEED_KMH = 60.0
EGO_MAX_SPEED_KMH = 70.0


# ============================================================
# 3. 全局可调接口：天气参数
# ============================================================

WEATHER_PRESET = "ClearNoon"

WEATHER_CLOUDINESS = 20.0
WEATHER_PRECIPITATION = 0.0
WEATHER_PUDDLES = 0.0
WEATHER_WIND = 5.0

WEATHER_SUN_AZIMUTH = 260.0
WEATHER_SUN_ALTITUDE = 20.0

WEATHER_FOG_DENSITY = 0.0
WEATHER_FOG_DISTANCE = 0.75
WEATHER_FOG_FALLOFF = 0.1

WEATHER_WETNESS = 0.0

WEATHER_SCATTERING = 1.0
WEATHER_MIE = 0.03
WEATHER_RAYLEIGH = 0.0331
WEATHER_DUST = 0.0


# ============================================================
# 4. 全局可调接口：车辆蓝图候选
# ============================================================

EGO_BP_CANDIDATES = [
    "vehicle.nissan.patrol",
    "vehicle.audi.etron",
    "vehicle.tesla.model3",
    "vehicle.lincoln.mkz_2020",
]

# 前车建议用小型 MPV / SUV / 轿车模拟本地慢车
FRONT_VEHICLE_BP_CANDIDATES = [
    "vehicle.nissan.patrol",
    "vehicle.lincoln.mkz_2020",
    "vehicle.audi.etron",
    "vehicle.tesla.model3",
    "vehicle.dodge.charger_2020",
]


# ============================================================
# 5. 前车轨迹锚点
# 格式：(x, y, z, pitch, yaw, roll)
# ============================================================

FRONT_VEHICLE_RAW_TRAJ = [
    (39.251, -13.671, 3.989, -0.947, -163.224, -0.000),
    (39.251, -13.671, 3.989, -0.947, -163.224, -0.000),
    (39.251, -13.671, 3.989, -0.947, -163.224, -0.000),
    (35.901, -14.684, 3.940, 0.524, -162.483, -0.000),
    (34.318, -15.206, 3.975, 1.204, -161.732, 0.000),
    (28.491, -17.218, 4.097, 0.994, -159.906, 0.000),
    (17.631, -21.440, 4.200, -0.322, -157.609, 0.000),
    (5.895, -26.311, 4.034, -0.911, -157.650, -0.000),
    (-3.995, -30.193, 3.965, 0.069, -158.938, 0.000),
    (-14.027, -34.056, 3.978, 0.069, -158.938, 0.000),
    (-23.953, -37.847, 3.993, 0.139, -159.524, 0.000),
    (-33.907, -41.564, 4.019, 0.139, -159.524, 0.000),
    (-43.674, -45.739, 3.918, -1.071, -165.999, 0.000),
    (-54.234, -47.650, 3.766, -0.163, -176.714, 0.000),
    (-64.853, -47.594, 3.743, 0.047, -176.532, 0.000),
    (-75.399, -49.575, 3.709, -0.303, -166.005, 0.000),
    (-85.832, -51.565, 3.681, -0.023, -170.933, 0.000),
    (-96.421, -52.214, 3.853, 1.476, 179.177, 0.000),
    (-107.119, -51.296, 4.100, 1.056, 172.073, 0.000),
    (-117.629, -49.750, 4.064, -1.749, 170.129, 0.000),
    (-125.776, -47.832, 3.932, -0.617, 164.566, 0.000),
]


# ============================================================
# 6. Ego 起终点
# ============================================================

EGO_START_TF = carla.Transform(
    carla.Location(x=47.577, y=-9.826, z=4.622),
    carla.Rotation(pitch=-1.751, yaw=-159.090, roll=0.000)
)

EGO_END_TF = carla.Transform(
    carla.Location(x=-179.721, y=-28.794, z=4.206),
    carla.Rotation(pitch=-8.121, yaw=154.640, roll=0.000)
)


# ============================================================
# 7. 行人参数
# ============================================================

ENABLE_PEDESTRIAN = True

PEDESTRIAN_START_TF = carla.Transform(
    carla.Location(x=-75.824, y=-55.982, z=4.526),
    carla.Rotation(pitch=-4.658, yaw=46.550, roll=0.000)
)

PEDESTRIAN_TARGET_TF = carla.Transform(
    carla.Location(x=-70.374, y=-52.681, z=3.674),
    carla.Rotation(pitch=-0.108, yaw=26.156, roll=0.000)
)

# 行人开始运动前静止等待时间，单位秒
PEDESTRIAN_START_DELAY_S = 5.0

PEDESTRIAN_WALK_SPEED_MPS = 1.20
PEDESTRIAN_STOP_DISTANCE_M = 0.35
PEDESTRIAN_SPAWN_Z_OFFSET = 0.20


# ============================================================
# 8. 基础工具函数
# ============================================================

def validate_user_inputs():
    if GlobalRoutePlanner is None:
        raise RuntimeError(
            "无法导入 CARLA GlobalRoutePlanner。请检查 CARLA_ROOT 是否正确，"
            "或者确认 PythonAPI\\carla\\agents\\navigation\\global_route_planner.py 存在。"
        )

    if len(FRONT_VEHICLE_RAW_TRAJ) < 2:
        raise RuntimeError("FRONT_VEHICLE_RAW_TRAJ 锚点不足。")


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
        raise RuntimeError("GlobalRoutePlanner 未生成路线，请检查起终点是否在同一可达道路网络上。")

    path = []
    for wp, road_option in route:
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
        print("[路径构建失败] 详细堆栈：")
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

        raise RuntimeError(
            "{} 路径构建失败，并且 USE_LINEAR_FALLBACK_WHEN_ROUTE_FAIL=False。".format(route_name)
        )


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
        "[天气设置] Preset={} | Clouds={} | Rain={} | SunAzim={} | SunAlt={} | Wetness={}".format(
            WEATHER_PRESET,
            WEATHER_CLOUDINESS,
            WEATHER_PRECIPITATION,
            WEATHER_SUN_AZIMUTH,
            WEATHER_SUN_ALTITUDE,
            WEATHER_WETNESS
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
# 11. 行人控制
# ============================================================

def get_walker_blueprints(world):
    walkers = list(world.get_blueprint_library().filter("walker.pedestrian.*"))
    if not walkers:
        raise RuntimeError("当前 CARLA blueprint library 中未找到 walker.pedestrian.* 行人蓝图。")
    return walkers


def configure_walker_blueprint(bp):
    if bp.has_attribute("is_invincible"):
        bp.set_attribute("is_invincible", "false")
    return bp


def spawn_single_pedestrian(world):
    walker_bp = configure_walker_blueprint(random.choice(get_walker_blueprints(world)))

    spawn_tf = carla.Transform(
        carla.Location(
            x=PEDESTRIAN_START_TF.location.x,
            y=PEDESTRIAN_START_TF.location.y,
            z=PEDESTRIAN_START_TF.location.z + PEDESTRIAN_SPAWN_Z_OFFSET
        ),
        carla.Rotation(
            pitch=0.0,
            yaw=PEDESTRIAN_START_TF.rotation.yaw,
            roll=0.0
        )
    )

    walker = world.try_spawn_actor(walker_bp, spawn_tf)

    if walker:
        print(
            "[行人生成] loc=({:.3f}, {:.3f}, {:.3f})".format(
                spawn_tf.location.x,
                spawn_tf.location.y,
                spawn_tf.location.z
            )
        )
    else:
        print("[行人警告] 行人生成失败，可能与碰撞或位置非法有关。")

    return walker


def stop_pedestrian(walker):
    if walker and walker.is_alive:
        walker.apply_control(
            carla.WalkerControl(
                direction=carla.Vector3D(0.0, 0.0, 0.0),
                speed=0.0,
                jump=False
            )
        )


def update_pedestrian(walker, sim_time):
    if not walker or not walker.is_alive:
        return

    if sim_time < PEDESTRIAN_START_DELAY_S:
        stop_pedestrian(walker)
        return

    current_loc = walker.get_location()
    target_loc = PEDESTRIAN_TARGET_TF.location

    dx = target_loc.x - current_loc.x
    dy = target_loc.y - current_loc.y
    dist = math.hypot(dx, dy)

    if dist <= PEDESTRIAN_STOP_DISTANCE_M:
        stop_pedestrian(walker)
        return

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


# ============================================================
# 12. 主函数
# ============================================================



# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-179.721, -28.794)
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
    pedestrian = None

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    try:
        validate_user_inputs()

        world = client.get_world()
        carla_map = world.get_map()

        # ====================================================
        # 12.1 同步模式 + 地图缓存 + 天气
        # ====================================================
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
        # 12.2 构建路径
        # ====================================================
        print("\n================ 路径构建阶段 ================")

        front_vehicle_path = build_dense_path_from_raw(
            FRONT_VEHICLE_RAW_TRAJ,
            interval=TRAJ_INTERVAL_M
        )
        print("[路径构建成功] FrontVehicle | manual anchor path points={}".format(len(front_vehicle_path)))

        ego_path, ego_path_type = build_route_with_diagnosis(
            route_name="Ego",
            carla_map=carla_map,
            start_tf=EGO_START_TF,
            end_tf=EGO_END_TF,
            resolution=ROUTE_RESOLUTION_M,
            fallback_interval=TRAJ_INTERVAL_M
        )

        print("\n================ 路径构建结果 ================")
        print("[路径结果] FrontVehicle : manual_anchor | points={}".format(len(front_vehicle_path)))
        print("[路径结果] Ego          : {} | points={}".format(ego_path_type, len(ego_path)))
        print("================================================\n")

        # ====================================================
        # 12.3 生成 Actor
        # ====================================================
        front_vehicle_start_tf = make_transform_from_raw_point(FRONT_VEHICLE_RAW_TRAJ[0])

        front_vehicle = spawn_vehicle_by_tf(
            world=world,
            candidates=FRONT_VEHICLE_BP_CANDIDATES,
            tf=front_vehicle_start_tf,
            color="180,180,180",
            role_name="front_local_vehicle",
            z_offset=0.75
        )
        if not front_vehicle:
            raise RuntimeError("前车生成失败。")
        actor_list.append(front_vehicle)

        ego = spawn_vehicle_by_tf(
            world=world,
            candidates=EGO_BP_CANDIDATES,
            tf=EGO_START_TF,
            color="0,80,255",
            role_name="ego",
            z_offset=0.75
        )
        if not ego:
            raise RuntimeError("Ego 生成失败。")
        actor_list.append(ego)

        if ENABLE_PEDESTRIAN:
            pedestrian = spawn_single_pedestrian(world)
            if pedestrian:
                actor_list.append(pedestrian)

        # ====================================================
        # 12.4 灯光
        # ====================================================
        set_vehicle_lights(front_vehicle, brake=False, hazard=False, low_beam=True)
        set_vehicle_lights(ego, brake=False, hazard=False, low_beam=True)

        # ====================================================
        # 12.5 独立 PID
        # ====================================================
        front_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.85, 0.65),
            i_clip=(-1.0, 1.0)
        )
        front_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.50, 0.50),
            i_clip=(-1.0, 1.0)
        )

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

        front_idx = 0
        ego_idx = 0

        # ====================================================
        # 12.6 Actor 生成后预热与初速度注入
        # ====================================================
        print("[预热] Actor 生成后同步预热。")
        sim_time = 0.0

        for _ in range(20):
            update_pedestrian(pedestrian, sim_time)
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += DT
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            front_vehicle,
            target_speed_kmh=FRONT_VEHICLE_SPEED_KMH,
            yaw_deg=front_vehicle_start_tf.rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_SPEED_KMH,
            yaw_deg=EGO_START_TF.rotation.yaw
        )

        for _ in range(5):
            update_pedestrian(pedestrian, sim_time)
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print(
            "[速度设置] FrontVehicle={} km/h | Ego={} km/h".format(
                FRONT_VEHICLE_SPEED_KMH,
                EGO_SPEED_KMH
            )
        )
        print("[行人设置] start_delay={}s | speed={}m/s".format(
            PEDESTRIAN_START_DELAY_S,
            PEDESTRIAN_WALK_SPEED_MPS
        ))

        # ====================================================
        # 12.7 主循环
        # ====================================================
        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += DT
            frame_count += 1

            update_pedestrian(pedestrian, sim_time)

            # -----------------------------
            # 前车：固定 60 km/h，沿给定轨迹
            # -----------------------------
            if reached_path_end(front_vehicle, front_vehicle_path, threshold=7.0):
                soft_hold_vehicle(front_vehicle)
                set_vehicle_lights(front_vehicle, brake=True, low_beam=True)
            else:
                front_idx = follow_path_constant_speed(
                    vehicle=front_vehicle,
                    path=front_vehicle_path,
                    path_index=front_idx,
                    pid_lon=front_pid_lon,
                    pid_lat=front_pid_lat,
                    target_speed_kmh=FRONT_VEHICLE_SPEED_KMH,
                    max_speed_kmh=FRONT_VEHICLE_MAX_SPEED_KMH,
                    min_lookahead=6.5,
                    lookahead_ratio=0.43,
                    max_search_ahead=70,
                    fallback_dist=55.0
                )
                set_vehicle_lights(front_vehicle, brake=False, low_beam=True)

            # -----------------------------
            # Ego：固定 60 km/h
            # -----------------------------
            if reached_path_end(ego, ego_path, threshold=8.0):
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
                    min_lookahead=7.0,
                    lookahead_ratio=0.45,
                    max_search_ahead=80,
                    fallback_dist=60.0
                )
                set_vehicle_lights(ego, brake=False, low_beam=True)

            if frame_count % int(2.0 / DT) == 0:
                ped_state = "WAIT"
                if pedestrian and pedestrian.is_alive and sim_time >= PEDESTRIAN_START_DELAY_S:
                    ped_state = "MOVE"

                print(
                    "[t={:05.2f}s | frame={:04d}] Front={:05.1f}km/h | Ego={:05.1f}km/h | idx(F/E)=({}/{}) | Path(E)={} | Ped={}".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(front_vehicle),
                        get_speed_kmh(ego),
                        front_idx,
                        ego_idx,
                        ego_path_type,
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
            stop_pedestrian(pedestrian)
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