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
7. 无 debug 可视化绘制。
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

# Ego 速度。用户未指定时，默认设置为城市道路较合理的 35 km/h。
EGO_SPEED_KMH = 45.0
EGO_MAX_SPEED_KMH = 50.0

# 对向来车固定 45 km/h
ONCOMING_SPEED_KMH = 42.0
ONCOMING_MAX_SPEED_KMH = 52.0

# 对向来车开始运动前静止等待时间，单位秒。
ONCOMING_START_DELAY_S = 1.0


# ============================================================
# 3. 全局可调接口：天气参数
# ============================================================
# 限界侵入场景建议使用清晰天气，突出固定构造物侵限问题。
# 如需复用你前面截图天气，可以直接改这里。

WEATHER_PRESET = "ClearNoon"

WEATHER_CLOUDINESS = 10.0
WEATHER_PRECIPITATION = 0.0
WEATHER_PUDDLES = 0.0
WEATHER_WIND = 5.0

WEATHER_SUN_AZIMUTH = 0.0
WEATHER_SUN_ALTITUDE = 45.0

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

ONCOMING_BP_CANDIDATES = [
    "vehicle.dodge.charger_2020",
    "vehicle.audi.tt",
    "vehicle.lincoln.mkz_2020",
    "vehicle.tesla.model3",
]


# ============================================================
# 5. 全局可调接口：Ego 起终点
# ============================================================

EGO_START_TF = carla.Transform(
    carla.Location(x=130.692, y=-0.722, z=1.707),
    carla.Rotation(pitch=0.000, yaw=-175.223, roll=0.000)
)

EGO_END_TF = carla.Transform(
    carla.Location(x=-69.643, y=-13.991, z=1.166),
    carla.Rotation(pitch=1.136, yaw=175.752, roll=0.000)
)


# ============================================================
# 6. 全局可调接口：对向来车起终点
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
PEDESTRIAN_START_DELAY_S = 8.0

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
        "[天气设置] Preset={} | Clouds={} | Rain={} | Puddles={} | Wind={} | SunAzim={} | SunAlt={} | FogDens={} | FogDist={} | Wetness={}".format(
            WEATHER_PRESET,
            WEATHER_CLOUDINESS,
            WEATHER_PRECIPITATION,
            WEATHER_PUDDLES,
            WEATHER_WIND,
            WEATHER_SUN_AZIMUTH,
            WEATHER_SUN_ALTITUDE,
            WEATHER_FOG_DENSITY,
            WEATHER_FOG_DISTANCE,
            WEATHER_WETNESS
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



# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-69.643, -13.991)
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
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            time.sleep(DT)

        # ====================================================
        # 13.2 构建 Ego 与对向车自动路线
        # ====================================================
        ego_path = build_route_by_global_planner(
            carla_map=carla_map,
            start_loc=EGO_START_TF.location,
            end_loc=EGO_END_TF.location,
            resolution=ROUTE_RESOLUTION_M
        )

        oncoming_path = build_route_by_global_planner(
            carla_map=carla_map,
            start_loc=ONCOMING_START_TF.location,
            end_loc=ONCOMING_END_TF.location,
            resolution=ROUTE_RESOLUTION_M
        )

        print("[路径检查] ego_path points:", len(ego_path))
        print("[路径检查] oncoming_path points:", len(oncoming_path))
        print("[控制策略] Ego 固定速度循迹；对向来车延迟启动后固定 45 km/h；行人延迟启动后移动。")

        # ====================================================
        # 13.3 生成 Ego
        # ====================================================
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

        set_vehicle_lights(
            ego,
            brake=False,
            hazard=False,
            low_beam=True
        )

        # ====================================================
        # 13.4 生成对向来车
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
        # 13.5 生成 2 个行人
        # ====================================================
        if ENABLE_PEDESTRIANS:
            walkers, pedestrian_target_locs = spawn_pedestrians(world)
            actor_list.extend(walkers)

        # ====================================================
        # 13.6 独立 PID
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
        oncoming_idx = 0
        oncoming_released = False

        # ====================================================
        # 13.7 Actor 生成后预热与 Ego 初速度注入
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
            target_speed_kmh=EGO_SPEED_KMH,
            yaw_deg=EGO_START_TF.rotation.yaw
        )

        for _ in range(5):
            hold_vehicle_before_release(oncoming)
            update_pedestrians(walkers, pedestrian_target_locs, sim_time)

            world.tick()
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print("[速度设置] Ego={} km/h | Oncoming={} km/h".format(
            EGO_SPEED_KMH,
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
            # Ego：固定速度循迹
            # -----------------------------
            if reached_path_end(ego, ego_path, threshold=6.0):
                soft_hold_vehicle(ego)
                set_vehicle_lights(
                    ego,
                    brake=True,
                    low_beam=True
                )
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
                    min_lookahead=5.0,
                    lookahead_ratio=0.36
                )
                set_vehicle_lights(
                    ego,
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
                    "[t={:05.2f}s | frame={:04d}] Ego={:05.1f}km/h | Oncoming={:05.1f}km/h | idx(E/O)=({}/{}) | Ped={} | Oncoming={}".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(ego),
                        get_speed_kmh(oncoming),
                        ego_idx,
                        oncoming_idx,
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