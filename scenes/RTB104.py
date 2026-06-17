# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

场景主题：
山区浓雾弯道后半段低速重卡 + Ego正常循迹行驶 + 对向高速来车冲突

本版内容：
1. 完全删除 Ego 的紧急制动、TTC触发、左避让、横向偏移控制。
2. Ego 只按照给定起点和终点，通过 GlobalRoutePlanner 自动生成路径并固定 50 km/h 行驶。
3. 对向来车终点：
   Location: x=114.005, y=0.548, z=18.965
   Rotation: pitch=2.754, yaw=17.023, roll=0.000
4. 重卡蓝图强制使用 vehicle.carlamotors.firetruck，不使用可口可乐车。
5. 已删除所有 debug 可视化绘制内容。
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
SCENARIO_DURATION = 70.0
KEEP_ACTORS_AFTER_SCRIPT = False

# 自动导航锚点间隔
ROUTE_RESOLUTION_M = 2.0


# ============================================================
# 2. 全局可调接口：车辆速度
# ============================================================

HEAVY_TRUCK_SPEED_KMH = 10.0
HEAVY_TRUCK_MAX_SPEED_KMH = 16.0

ONCOMING_SPEED_KMH = 60.0
ONCOMING_MAX_SPEED_KMH = 68.0

EGO_SPEED_KMH = 50.0
EGO_MAX_SPEED_KMH = 58.0


# ============================================================
# 3. 全局可调接口：天气参数
# ============================================================

WEATHER_PRESET = "ClearNoon"

WEATHER_CLOUDINESS = 100.0
WEATHER_PRECIPITATION = 0.0
WEATHER_PUDDLES = 70.0
WEATHER_WIND = 60.0

WEATHER_SUN_AZIMUTH = 0.0
WEATHER_SUN_ALTITUDE = 6.0

WEATHER_FOG_DENSITY = 90.0
WEATHER_FOG_DISTANCE = 50.750
WEATHER_FOG_FALLOFF = 0.100

WEATHER_WETNESS = 0.0

WEATHER_SCATTERING = 1.500
WEATHER_MIE = 0.210
WEATHER_RAYLEIGH = 0.070
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

HEAVY_TRUCK_BP_CANDIDATES = [
    "vehicle.carlamotors.firetruck",
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
    carla.Location(x=148.837, y=4.789, z=19.913),
    carla.Rotation(pitch=2.447, yaw=-166.496, roll=0.000)
)

EGO_END_TF = carla.Transform(
    carla.Location(x=-149.286, y=61.583, z=8.560),
    carla.Rotation(pitch=-4.096, yaw=118.011, roll=0.000)
)


# ============================================================
# 6. 全局可调接口：对向来车起终点
# ============================================================

ONCOMING_START_TF = carla.Transform(
    carla.Location(x=-179.880, y=221.894, z=2.463),
    carla.Rotation(pitch=4.823, yaw=-81.046, roll=0.000)
)

ONCOMING_END_TF = carla.Transform(
    carla.Location(x=114.005, y=0.548, z=18.965),
    carla.Rotation(pitch=2.754, yaw=17.023, roll=0.000)
)


# ============================================================
# 7. 全局可调接口：重卡轨迹锚点
# ============================================================

HEAVY_TRUCK_RAW_TRAJ = [
    (-38.702, -11.747, 13.802, -3.951, 163.121, 0.000),
    (-40.945, -11.111, 13.712, -2.201, 164.169, 0.000),
    (-42.906, -10.547, 13.636, -2.131, 163.959, 0.000),
    (-42.906, -10.547, 13.636, -2.341, 159.923, 0.000),
    (-46.442, -9.311, 13.470, -3.740, 160.552, 0.000),
    (-48.468, -8.570, 13.275, -5.910, 160.552, 0.000),
    (-49.277, -8.284, 13.186, -3.880, 159.572, 0.000),
    (-50.083, -7.978, 13.155, -1.710, 159.082, 0.000),
    (-53.375, -6.679, 13.007, -4.045, 157.176, 0.000),
    (-56.692, -5.348, 12.755, -2.925, 159.276, 0.000),
    (-59.983, -4.048, 12.614, -2.015, 157.666, 0.000),
    (-63.286, -2.664, 12.490, -1.595, 157.247, 0.000),
    (-66.538, -1.263, 12.489, 0.575, 155.148, 0.000),
    (-69.681, 0.366, 12.503, -3.205, 153.539, 0.000),
    (-72.835, 2.002, 12.057, -9.644, 152.140, 0.000),
    (-75.945, 3.642, 11.647, -3.274, 152.280, 0.000),
    (-79.099, 5.340, 11.647, 1.695, 151.510, 0.000),
    (-82.211, 7.028, 11.752, 1.555, 151.440, 0.000),
    (-85.317, 8.728, 11.691, -2.084, 151.300, 0.000),
    (-88.435, 10.398, 11.514, -3.484, 152.140, 0.000),
    (-91.582, 12.097, 11.300, -3.274, 150.460, 0.000),
    (-94.655, 13.847, 11.102, -3.204, 150.320, 0.000),
    (-97.727, 15.597, 10.904, -3.134, 150.320, 0.000),
    (-101.191, 17.594, 10.792, -1.244, 149.900, 0.000),
    (-107.430, 21.328, 10.627, -1.664, 147.099, 0.000),
    (-114.861, 26.324, 10.359, -1.594, 146.118, 0.000),
    (-120.814, 30.768, 10.080, -2.504, 141.848, 0.000),
    (-126.394, 35.111, 9.699, -5.464, 139.298, 0.000),
    (-131.612, 39.870, 9.188, -1.334, 137.056, 0.000),
    (-136.458, 45.028, 9.047, -1.614, 129.986, 0.000),
    (-141.127, 50.462, 8.877, -1.194, 131.106, 0.000),
    (-145.521, 56.009, 8.653, -2.454, 125.504, 0.000),
    (-149.580, 61.908, 8.381, -1.964, 123.545, 0.000),
    (-153.297, 68.026, 8.096, -2.734, 118.716, 0.000),
    (-156.552, 74.309, 7.787, -0.844, 116.615, 0.000),
    (-159.626, 80.783, 7.782, -0.154, 115.272, 0.000),
    (-162.422, 87.298, 7.662, -1.764, 110.369, 0.000),
    (-164.827, 93.956, 7.404, -3.094, 108.549, 0.000),
    (-166.802, 100.728, 6.792, -6.383, 103.931, 0.000),
    (-168.499, 107.661, 6.174, -3.093, 103.652, 0.000),
    (-170.003, 114.575, 5.857, -1.973, 99.943, 0.000),
]


# ============================================================
# 8. 基础工具函数
# ============================================================

def validate_user_inputs():
    if len(HEAVY_TRUCK_RAW_TRAJ) < 2:
        raise RuntimeError("HEAVY_TRUCK_RAW_TRAJ 锚点不足。")

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


def make_transform_from_raw_point(p):
    return carla.Transform(
        carla.Location(x=p[0], y=p[1], z=p[2]),
        carla.Rotation(pitch=p[3], yaw=p[4], roll=p[5])
    )


def raw_traj_to_xyz(raw_traj):
    return [(p[0], p[1], p[2]) for p in raw_traj]


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
# 9. 路径生成函数
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


def build_heavy_truck_path():
    raw_points = raw_traj_to_xyz(HEAVY_TRUCK_RAW_TRAJ)
    raw_points = RTB.clean_trajectory(raw_points, min_dist=1e-5)
    dense = RTB.interpolate_trajectory(raw_points, interval=1.0)
    dense = RTB.clean_trajectory(dense, min_dist=0.5)
    return dense


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
    lookahead_ratio=0.42
):
    """
    固定速度循迹。
    Ego、重卡、对向车都只使用这个函数。
    不包含任何避让、不包含任何横向偏移、不包含任何 TTC 逻辑。
    """
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
# 11. 天气设置
# ============================================================

def apply_mountain_fog_weather(world):
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
        "[天气设置] Clouds={} | Rain={} | Puddles={} | Wind={} | SunAzim={} | SunAlt={} | FogDens={} | FogDist={} | Wetness={}".format(
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
# 12. 主函数
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
        # 12.1 同步模式 + 地图缓存 + 天气
        # ====================================================
        enable_sync(world, dt=DT)
        print_world_sync_state(world)
        warmup_map_cache(world)
        apply_mountain_fog_weather(world)

        print("[预热] 同步模式预热中。")
        for _ in range(15):
            world.tick()
            time.sleep(DT)

        # ====================================================
        # 12.2 构建三辆车路径
        # ====================================================
        heavy_truck_path = build_heavy_truck_path()

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

        print("[路径检查] heavy_truck_path points:", len(heavy_truck_path))
        print("[路径检查] ego_path points:", len(ego_path))
        print("[路径检查] oncoming_path points:", len(oncoming_path))
        print("[控制策略] Ego 仅固定速度循迹，不进行避让、不制动、不横向偏移。")

        # ====================================================
        # 12.3 生成三辆车
        # ====================================================
        heavy_truck_start_tf = make_transform_from_raw_point(HEAVY_TRUCK_RAW_TRAJ[0])

        heavy_truck = spawn_vehicle_by_tf(
            world=world,
            candidates=HEAVY_TRUCK_BP_CANDIDATES,
            tf=heavy_truck_start_tf,
            color="180,180,180",
            role_name="slow_heavy_truck",
            z_offset=1.3
        )
        if not heavy_truck:
            raise RuntimeError("重卡生成失败。")
        actor_list.append(heavy_truck)

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

        oncoming = spawn_vehicle_by_tf(
            world=world,
            candidates=ONCOMING_BP_CANDIDATES,
            tf=ONCOMING_START_TF,
            color="255,255,255",
            role_name="oncoming_high_speed_vehicle",
            z_offset=0.75
        )
        if not oncoming:
            raise RuntimeError("对向来车生成失败。")
        actor_list.append(oncoming)

        set_vehicle_lights(heavy_truck, brake=False, hazard=False, low_beam=True)
        set_vehicle_lights(ego, brake=False, hazard=False, low_beam=True, fog=True)
        set_vehicle_lights(oncoming, brake=False, hazard=False, low_beam=True)

        # ====================================================
        # 12.4 每辆车独立 PID
        # ====================================================
        truck_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="truck",
            output_clip=(-0.80, 0.45),
            i_clip=(-1.0, 1.0)
        )
        truck_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="truck",
            output_clip=(-0.35, 0.35),
            i_clip=(-1.0, 1.0)
        )

        ego_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="wet_road",
            output_clip=(-0.90, 0.55),
            i_clip=(-1.0, 1.0)
        )
        ego_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="wet_road",
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

        truck_idx = 0
        ego_idx = 0
        oncoming_idx = 0

        # ====================================================
        # 12.5 Actor 生成后预热与初速度注入
        # ====================================================
        print("[预热] Actor 生成后同步预热。")
        for _ in range(20):
            world.tick()
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            heavy_truck,
            target_speed_kmh=HEAVY_TRUCK_SPEED_KMH,
            yaw_deg=heavy_truck_start_tf.rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_SPEED_KMH,
            yaw_deg=EGO_START_TF.rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            oncoming,
            target_speed_kmh=ONCOMING_SPEED_KMH,
            yaw_deg=ONCOMING_START_TF.rotation.yaw
        )

        for _ in range(5):
            world.tick()
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print("[速度设置] 重卡={} km/h | Ego={} km/h | 对向来车={} km/h".format(
            HEAVY_TRUCK_SPEED_KMH,
            EGO_SPEED_KMH,
            ONCOMING_SPEED_KMH
        ))

        # ====================================================
        # 12.6 主循环
        # ====================================================
        sim_time = 0.0
        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            sim_time += DT
            frame_count += 1

            # -----------------------------
            # 重卡：固定 10 km/h，沿指定锚点行驶
            # -----------------------------
            if reached_path_end(heavy_truck, heavy_truck_path, threshold=5.0):
                soft_hold_vehicle(heavy_truck)
                set_vehicle_lights(heavy_truck, brake=True, low_beam=True)
            else:
                truck_idx = follow_path_constant_speed(
                    vehicle=heavy_truck,
                    path=heavy_truck_path,
                    path_index=truck_idx,
                    pid_lon=truck_pid_lon,
                    pid_lat=truck_pid_lat,
                    target_speed_kmh=HEAVY_TRUCK_SPEED_KMH,
                    max_speed_kmh=HEAVY_TRUCK_MAX_SPEED_KMH,
                    min_lookahead=4.0,
                    lookahead_ratio=0.35
                )
                set_vehicle_lights(heavy_truck, brake=False, low_beam=True)

            # -----------------------------
            # 对向车：固定 60 km/h，沿自动路径行驶
            # -----------------------------
            if reached_path_end(oncoming, oncoming_path, threshold=5.0):
                soft_hold_vehicle(oncoming)
                set_vehicle_lights(oncoming, brake=True, low_beam=True)
            else:
                oncoming_idx = follow_path_constant_speed(
                    vehicle=oncoming,
                    path=oncoming_path,
                    path_index=oncoming_idx,
                    pid_lon=oncoming_pid_lon,
                    pid_lat=oncoming_pid_lat,
                    target_speed_kmh=ONCOMING_SPEED_KMH,
                    max_speed_kmh=ONCOMING_MAX_SPEED_KMH,
                    min_lookahead=6.0,
                    lookahead_ratio=0.45
                )
                set_vehicle_lights(oncoming, brake=False, low_beam=True)

            # -----------------------------
            # Ego：固定 50 km/h，沿起终点自动路径行驶
            # 无避让、无TTC、无制动、无左偏移
            # -----------------------------
            if reached_path_end(ego, ego_path, threshold=6.0):
                soft_hold_vehicle(ego)
                set_vehicle_lights(ego, brake=True, low_beam=True, fog=True)
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
                    lookahead_ratio=0.35
                )
                set_vehicle_lights(ego, brake=False, low_beam=True, fog=True)

            if frame_count % int(2.0 / DT) == 0:
                print(
                    "[t={:05.2f}s | frame={:04d}] Ego={:05.1f}km/h | Truck={:05.1f}km/h | Oncoming={:05.1f}km/h | idx(E/T/O)=({}/{}/{})".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(ego),
                        get_speed_kmh(heavy_truck),
                        get_speed_kmh(oncoming),
                        ego_idx,
                        truck_idx,
                        oncoming_idx
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