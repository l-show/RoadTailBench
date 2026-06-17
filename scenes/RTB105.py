# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

场景主题：
城市交叉口非标黄闪警示灯视认失效 + 静止洒水车突然喷水水雾遮挡
+ 静止大型白色厢式货车遮挡 + Ego 匀速通过 + 紧凑行人群体间歇式移动遮挡

本版修改：
1. 新增一辆大型白色厢式货车，生成在指定位置并保持静止不动。
2. 行人不使用 AI Controller，改为 WalkerControl 直接控制，避免 NavMesh 导致原地不动。
3. 行人运动改为“初始静止 → 移动 → 静止 → 移动”的间歇式行为。
4. 行人静止时间、移动时间、初始静止时间均提供全局参数接口。
5. 保持无 debug 可视化绘制。
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
SCENARIO_DURATION = 45.0
KEEP_ACTORS_AFTER_SCRIPT = False

ROUTE_RESOLUTION_M = 2.0


# ============================================================
# 2. 全局可调接口：Ego 速度
# ============================================================

EGO_SPEED_KMH = 60.0
EGO_MAX_SPEED_KMH = 68.0


# ============================================================
# 3. 全局可调接口：天气参数
# ============================================================

WEATHER_PRESET = "ClearSunset"

WEATHER_CLOUDINESS = 0.0
WEATHER_PRECIPITATION = 0.0
WEATHER_PUDDLES = 0.0
WEATHER_WIND = 5.0

WEATHER_SUN_AZIMUTH = 0.0
WEATHER_SUN_ALTITUDE = 37.0

WEATHER_FOG_DENSITY = 2.0
WEATHER_FOG_DISTANCE = 0.750
WEATHER_FOG_FALLOFF = 0.100

WEATHER_WETNESS = 0.0

WEATHER_SCATTERING = 1.000
WEATHER_MIE = 0.0300
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

SPRINKLER_TRUCK_BP_CANDIDATES = [
    "vehicle.carlamotors.firetruck",
]

STATIC_BOX_TRUCK_BP_CANDIDATES = [
    "vehicle.mercedes.sprinter",
    "vehicle.carlamotors.carlacola",
    "vehicle.carlamotors.firetruck",
]


# ============================================================
# 5. 全局可调接口：洒水车静止生成位置
# ============================================================

SPRINKLER_TRUCK_TF = carla.Transform(
    carla.Location(x=52.247, y=-12.384, z=0.1),
    carla.Rotation(pitch=-1.261, yaw=178.499, roll=0.000)
)


# ============================================================
# 6. 全局可调接口：静止大型白色厢式货车位置
# ============================================================

STATIC_BOX_TRUCK_TF = carla.Transform(
    carla.Location(x=31.232, y=-15.211, z=0.1),
    carla.Rotation(pitch=1.105, yaw=179.628, roll=0.001)
)


# ============================================================
# 7. 全局可调接口：Ego 起终点
# ============================================================

EGO_START_TF = carla.Transform(
    carla.Location(x=179.316, y=-10.443, z=1.491),
    carla.Rotation(pitch=-0.244, yaw=178.496, roll=0.0010)
)


EGO_END_TF = carla.Transform(
    carla.Location(x=-117.743, y=-6.200, z=1.208),
    carla.Rotation(pitch=-0.332, yaw=-177.713, roll=0.000)
)


# ============================================================
# 8. 全局可调接口：行人群体
# ============================================================

ENABLE_PEDESTRIAN_GROUP = True
PEDESTRIAN_COUNT = 8

PEDESTRIAN_START_TF = carla.Transform(
    carla.Location(x=18.126, y=-15.381, z=1.548),
    carla.Rotation(pitch=-12.125, yaw=91.192, roll=0.001)
)

PEDESTRIAN_TARGET_TF = carla.Transform(
    carla.Location(x=17.814, y=7.814, z=1.281),
    carla.Rotation(pitch=-5.677, yaw=67.583, roll=0.001)
)

# 紧凑队形参数
PEDESTRIAN_CLUSTER_SPACING_M = 0.78
PEDESTRIAN_MIN_DISTANCE_M = 0.62
PEDESTRIAN_SPAWN_Z_OFFSET = 0.20
PEDESTRIAN_FALLBACK_RADIUS_M = 1.35
PEDESTRIAN_MAX_ATTEMPTS_PER_PERSON = 30

# 行人移动参数
PEDESTRIAN_WALK_SPEED_MPS = 1.25
PEDESTRIAN_STOP_DISTANCE_M = 0.45

# 行人间歇式运动参数
# 行人生成后先静止一段时间，再开始“移动-静止-移动”的循环
PEDESTRIAN_INITIAL_STATIC_TIME_S = 8

# 每轮连续移动时间
PEDESTRIAN_MOVE_PHASE_TIME_S = 5.0

# 每轮移动后的静止时间
PEDESTRIAN_STATIC_PHASE_TIME_S = 0

# 是否启用间歇式移动；False 时行人持续移动到目标点
ENABLE_PEDESTRIAN_INTERMITTENT_MOTION = True


# ============================================================
# 9. 基础工具函数
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


def force_static_vehicle(vehicle, hand_brake=True):
    if not vehicle or not vehicle.is_alive:
        return

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
# 10. 行人工具函数
# ============================================================

def make_cluster_offsets(count, spacing):
    base_offsets = [
        (-1.5, -0.5), (-0.5, -0.5), (0.5, -0.5), (1.5, -0.5),
        (-1.5, 0.5),  (-0.5, 0.5),  (0.5, 0.5),  (1.5, 0.5),
    ]

    offsets = []
    for i in range(count):
        ox, oy = base_offsets[i % len(base_offsets)]
        offsets.append((ox * spacing, oy * spacing))

    return offsets


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


def spawn_pedestrian_group(world):
    """
    生成紧凑 8 人行人群体，并避免生成碰撞箱冲突。

    不生成 controller.ai.walker。
    行人移动由主循环中的 WalkerControl 直接驱动。
    """
    walker_bps = get_walker_blueprints(world)

    walkers = []
    accepted_spawn_locs = []
    target_locs = []

    start_offsets = make_cluster_offsets(PEDESTRIAN_COUNT, PEDESTRIAN_CLUSTER_SPACING_M)
    target_offsets = make_cluster_offsets(PEDESTRIAN_COUNT, PEDESTRIAN_CLUSTER_SPACING_M)

    for i in range(PEDESTRIAN_COUNT):
        spawned_walker = None

        base_offset = start_offsets[i]
        candidate_offsets = [base_offset]

        for _ in range(PEDESTRIAN_MAX_ATTEMPTS_PER_PERSON - 1):
            angle = random.uniform(0.0, 2.0 * math.pi)
            radius = random.uniform(0.0, PEDESTRIAN_FALLBACK_RADIUS_M)
            jitter_x = math.cos(angle) * radius
            jitter_y = math.sin(angle) * radius
            candidate_offsets.append((base_offset[0] + jitter_x, base_offset[1] + jitter_y))

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
                "第 {} 个行人生成失败：周围紧凑候选点存在碰撞或非法位置。"
                "已停止生成，避免碰撞箱冲突。".format(i + 1)
            )

    for i in range(PEDESTRIAN_COUNT):
        target_offset_x, target_offset_y = target_offsets[i]
        target_loc = make_location_with_offset(
            PEDESTRIAN_TARGET_TF,
            target_offset_x,
            target_offset_y,
            z_offset=0.0
        )
        target_locs.append(target_loc)

    world.tick()

    print("[行人生成] 已成功生成 {} 个紧凑行人。行人移动由 WalkerControl 直接驱动。".format(len(walkers)))

    return walkers, target_locs


def should_pedestrians_move(sim_time):
    """
    行人间歇式运动逻辑：
    1. 场景开始后先静止 PEDESTRIAN_INITIAL_STATIC_TIME_S。
    2. 之后进入周期：
       - 移动 PEDESTRIAN_MOVE_PHASE_TIME_S
       - 静止 PEDESTRIAN_STATIC_PHASE_TIME_S
       - 循环
    """
    if not ENABLE_PEDESTRIAN_INTERMITTENT_MOTION:
        return True

    if sim_time < PEDESTRIAN_INITIAL_STATIC_TIME_S:
        return False

    phase_time = sim_time - PEDESTRIAN_INITIAL_STATIC_TIME_S
    cycle_time = PEDESTRIAN_MOVE_PHASE_TIME_S + PEDESTRIAN_STATIC_PHASE_TIME_S

    if cycle_time <= 1e-6:
        return True

    cycle_pos = phase_time % cycle_time

    return cycle_pos < PEDESTRIAN_MOVE_PHASE_TIME_S


def stop_pedestrian_group(walkers):
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


def update_pedestrian_group(walkers, target_locs, sim_time):
    """
    每帧直接驱动行人朝目标点移动。
    支持间歇式静止时间。
    不依赖 NavMesh，不依赖 AI Controller。
    """
    if not walkers:
        return

    if not should_pedestrians_move(sim_time):
        stop_pedestrian_group(walkers)
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
# 11. 路径生成函数
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
# 12. 车辆循迹控制函数
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
# 13. 天气设置
# ============================================================

def apply_clear_sunset_weather(world):
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
# 14. 主函数
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
        # 14.1 同步模式 + 地图缓存 + 天气
        # ====================================================
        enable_sync(world, dt=DT)
        print_world_sync_state(world)
        warmup_map_cache(world)
        apply_clear_sunset_weather(world)

        print("[预热] 同步模式预热中。")
        for _ in range(15):
            world.tick()
            time.sleep(DT)

        # ====================================================
        # 14.2 构建 Ego 自动路线
        # ====================================================
        ego_path = build_route_by_global_planner(
            carla_map=carla_map,
            start_loc=EGO_START_TF.location,
            end_loc=EGO_END_TF.location,
            resolution=ROUTE_RESOLUTION_M
        )

        print("[路径检查] ego_path points:", len(ego_path))
        print(
            "[控制策略] Ego 固定速度循迹；洒水车静止；白色厢式货车静止；"
            "行人由 WalkerControl 直接间歇移动。"
        )

        # ====================================================
        # 14.3 生成静止洒水车
        # ====================================================
        sprinkler_truck = spawn_vehicle_by_tf(
            world=world,
            candidates=SPRINKLER_TRUCK_BP_CANDIDATES,
            tf=SPRINKLER_TRUCK_TF,
            color="220,220,220",
            role_name="static_sprinkler_truck",
            z_offset=1.0
        )
        if not sprinkler_truck:
            raise RuntimeError("洒水车/消防车生成失败。")
        actor_list.append(sprinkler_truck)

        force_static_vehicle(sprinkler_truck, hand_brake=True)
        set_vehicle_lights(
            sprinkler_truck,
            brake=True,
            hazard=True,
            low_beam=True
        )

        # ====================================================
        # 14.4 生成静止大型白色厢式货车
        # ====================================================
        static_box_truck = spawn_vehicle_by_tf(
            world=world,
            candidates=STATIC_BOX_TRUCK_BP_CANDIDATES,
            tf=STATIC_BOX_TRUCK_TF,
            color="255,255,255",
            role_name="static_white_box_truck",
            z_offset=1.0
        )
        if not static_box_truck:
            raise RuntimeError("大型白色厢式货车生成失败。")
        actor_list.append(static_box_truck)

        force_static_vehicle(static_box_truck, hand_brake=True)
        set_vehicle_lights(
            static_box_truck,
            brake=True,
            hazard=False,
            low_beam=True
        )

        # ====================================================
        # 14.5 生成 Ego 主车
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
            low_beam=True,
            fog=False
        )

        # ====================================================
        # 14.6 生成紧凑行人群体
        # ====================================================
        if ENABLE_PEDESTRIAN_GROUP:
            walkers, pedestrian_target_locs = spawn_pedestrian_group(world)
            actor_list.extend(walkers)

        # ====================================================
        # 14.7 Ego 独立 PID
        # ====================================================
        ego_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.60),
            i_clip=(-1.0, 1.0)
        )
        ego_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.45, 0.45),
            i_clip=(-1.0, 1.0)
        )

        ego_idx = 0

        # ====================================================
        # 14.8 Actor 生成后预热与初速度注入
        # ====================================================
        print("[预热] Actor 生成后同步预热。")
        sim_time = 0.0

        for _ in range(20):
            force_static_vehicle(sprinkler_truck, hand_brake=True)
            force_static_vehicle(static_box_truck, hand_brake=True)
            update_pedestrian_group(walkers, pedestrian_target_locs, sim_time)

            set_vehicle_lights(
                sprinkler_truck,
                brake=True,
                hazard=True,
                low_beam=True
            )
            set_vehicle_lights(
                static_box_truck,
                brake=True,
                hazard=False,
                low_beam=True
            )

            world.tick()
            sim_time += DT
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_SPEED_KMH,
            yaw_deg=EGO_START_TF.rotation.yaw
        )

        for _ in range(5):
            force_static_vehicle(sprinkler_truck, hand_brake=True)
            force_static_vehicle(static_box_truck, hand_brake=True)
            update_pedestrian_group(walkers, pedestrian_target_locs, sim_time)

            world.tick()
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print(
            "[速度设置] Ego={} km/h | 洒水车静止 | 白色厢式货车静止 | 行人速度={} m/s".format(
                EGO_SPEED_KMH,
                PEDESTRIAN_WALK_SPEED_MPS
            )
        )
        print(
            "[行人间歇] 初始静止={}s | 移动段={}s | 静止段={}s | 启用={}".format(
                PEDESTRIAN_INITIAL_STATIC_TIME_S,
                PEDESTRIAN_MOVE_PHASE_TIME_S,
                PEDESTRIAN_STATIC_PHASE_TIME_S,
                ENABLE_PEDESTRIAN_INTERMITTENT_MOTION
            )
        )

        # ====================================================
        # 14.9 主循环
        # ====================================================
        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            sim_time += DT
            frame_count += 1

            # 洒水车持续保持静止
            force_static_vehicle(sprinkler_truck, hand_brake=True)
            set_vehicle_lights(
                sprinkler_truck,
                brake=True,
                hazard=True,
                low_beam=True
            )

            # 大型白色厢式货车持续保持静止
            force_static_vehicle(static_box_truck, hand_brake=True)
            set_vehicle_lights(
                static_box_truck,
                brake=True,
                hazard=False,
                low_beam=True
            )

            # 行人间歇式移动到目标点附近
            update_pedestrian_group(walkers, pedestrian_target_locs, sim_time)

            # Ego：固定 60 km/h，沿起终点自动路径行驶
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
                    min_lookahead=5.5,
                    lookahead_ratio=0.42
                )
                set_vehicle_lights(
                    ego,
                    brake=False,
                    low_beam=True
                )

            if frame_count % int(2.0 / DT) == 0:
                ped_motion_state = "MOVE" if should_pedestrians_move(sim_time) else "STOP"
                print(
                    "[t={:05.2f}s | frame={:04d}] Ego={:05.1f}km/h | idx(E)={} | walkers={} | PedState={}".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(ego),
                        ego_idx,
                        len(walkers),
                        ped_motion_state
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
            stop_pedestrian_group(walkers)
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