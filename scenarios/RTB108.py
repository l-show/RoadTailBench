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

EGO_INITIAL_SPEED_KMH = 60.0
EGO_SLOW_SPEED_KMH = 40.0
EGO_ACCEL_SPEED_KMH = 65.0
EGO_MAX_SPEED_KMH = 75.0
EGO_SLOW_TRIGGER_X = -39.0
EGO_SLOW_DURATION_S = 2.0


# ============================================================
# 3. 全局可调接口：天气参数
# ============================================================

cloudiness = 20.0
precipitation = 0.0
precipitation_deposits = 0.0
wind_intensity = 5.0
sun_azimuth_angle = 260.0
sun_altitude_angle = 20.0
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
    "vehicle.lincoln.mkz_2020"
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
    (-114.034, -50.748, 162.280), (-114.034, -50.748, 162.280), (-114.034, -50.748, 162.280), (-114.034, -50.748, 162.280),
    (-119.912, -48.895, 162.839), (-129.732, -46.275, 166.784), (-139.465, -43.986, 166.574), (-149.167, -41.558, 165.374),
    (-158.707, -38.584, 158.665), (-168.005, -34.888, 157.404), (-177.263, -30.668, 153.383), (-186.042, -25.876, 150.149),
    (-194.610, -20.409, 145.245), (-202.713, -14.270, 141.449), (-210.313, -7.774, 137.412), (-217.525, -0.848, 134.584),
    (-224.295, 6.731, 129.317), (-229.972, 13.714, 129.037), (-229.972, 13.714, 129.037), (-229.972, 13.714, 129.037),
    (-229.972, 13.714, 129.037)
]

# ============================================================
# 6. Ego 完整轨迹，格式为 (x, y, yaw)
# ============================================================

EGO_RAW_TRAJECTORY = [
    (59.143, -6.051, -163.701), (59.143, -6.051, -163.841), (58.864, -6.135, -162.881), (58.388, -6.285, -162.439),
    (57.904, -6.438, -162.439), (57.428, -6.589, -161.460), (56.949, -6.760, -159.979), (55.294, -7.363, -159.979),
    (51.778, -8.665, -158.676), (48.245, -10.099, -156.897), (44.796, -11.569, -157.116), (41.265, -13.007, -158.650),
    (37.773, -14.373, -158.321), (34.289, -15.758, -158.321), (30.689, -17.189, -158.321), (27.201, -18.575, -158.321),
    (23.659, -19.983, -158.321), (20.173, -21.369, -158.321), (16.688, -22.754, -158.321), (13.088, -24.185, -158.321),
    (9.545, -25.594, -158.321), (6.047, -26.946, -159.323), (2.545, -28.286, -158.994), (-1.015, -29.653, -158.994),
    (-4.573, -31.020, -158.994), (-8.072, -32.363, -159.104), (-11.697, -33.736, -158.994), (-15.198, -35.081, -158.994),
    (-18.698, -36.425, -158.994), (-22.257, -37.791, -158.994), (-25.758, -39.136, -158.994), (-29.329, -40.471, -159.561),
    (-32.901, -41.803, -159.561), (-36.433, -43.062, -161.422), (-39.573, -44.117, -161.422), (-39.573, -44.117, -161.422),
    (-39.573, -44.117, -161.422), (-39.573, -44.117, -161.422), (-39.573, -44.117, -161.422), (-39.573, -44.117, -161.422),
    (-39.573, -44.117, -161.422), (-39.573, -44.117, -161.422), (-39.573, -44.117, -161.422), (-39.573, -44.117, -161.422),
    (-39.573, -44.117, -161.422), (-39.573, -44.117, -161.422), (-39.575, -44.118, -161.905), (-40.053, -44.270, -163.311),
    (-40.536, -44.397, -167.673), (-41.029, -44.481, -171.260), (-42.062, -44.639, -171.702), (-43.488, -44.813, -173.334),
    (-49.156, -45.478, -172.808), (-55.442, -46.404, -171.524), (-61.624, -47.325, -171.524), (-67.908, -48.261, -171.752),
    (-74.209, -49.084, -171.851), (-80.482, -50.092, -170.181), (-86.643, -51.144, -171.215), (-92.857, -51.798, -175.907),
    (-99.207, -51.942, 179.184), (-105.451, -51.706, 175.810), (-111.764, -50.997, 171.344), (-118.121, -49.857, 168.326),
    (-118.529, -49.772, 168.326), (-118.529, -49.772, 168.326), (-118.529, -49.772, 168.326), (-118.529, -49.772, 168.326),
]


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
    if len(FRONT_VEHICLE_RAW_TRAJ) < 2:
        raise RuntimeError("FRONT_VEHICLE_RAW_TRAJ 锚点不足。")
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
    if not ego or not ego.is_alive:
        return EGO_SLOW_SPEED_KMH

    if state["phase"] == "initial" and ego.get_location().x <= EGO_SLOW_TRIGGER_X:
        state["phase"] = "slow"
        state["slow_start_time"] = sim_time
        print("[事件触发] Ego 到达 x<= {:.1f}，减速到 {} km/h | t={:.2f}s".format(
            EGO_SLOW_TRIGGER_X,
            EGO_SLOW_SPEED_KMH,
            sim_time
        ))

    if state["phase"] == "slow":
        if sim_time - state["slow_start_time"] >= EGO_SLOW_DURATION_S:
            state["phase"] = "accelerated"
            print("[事件触发] Ego 低速阶段结束，加速到 {} km/h | t={:.2f}s".format(
                EGO_ACCEL_SPEED_KMH,
                sim_time
            ))
        else:
            return EGO_SLOW_SPEED_KMH

    if state["phase"] == "initial":
        return EGO_INITIAL_SPEED_KMH
    return EGO_ACCEL_SPEED_KMH


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
        "[天气设置] Clouds={} | Rain={} | SunAzim={} | SunAlt={} | Wetness={}".format(
            cloudiness,
            precipitation,
            sun_azimuth_angle,
            sun_altitude_angle,
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

        ego_path = build_path_from_xy_yaw(
            carla_map=carla_map,
            raw_traj=EGO_RAW_TRAJECTORY,
            interval=TRAJ_INTERVAL_M
        )
        ego_path_type = "manual_xy_yaw"
        ego_start_tf = make_transform_from_xy_yaw(carla_map, EGO_RAW_TRAJECTORY[0])
        print("[路径构建成功] Ego | manual xy-yaw path points={}".format(len(ego_path)))

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
            tf=ego_start_tf,
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
        ego_speed_state = {
            "phase": "initial",
            "slow_start_time": None,
        }

        # ====================================================
        # 12.6 Actor 生成后预热与初速度注入
        # ====================================================
        print("[预热] Actor 生成后同步预热。")
        sim_time = 0.0

        for _ in range(20):
            update_pedestrian(pedestrian, sim_time)
            world.tick()
            sim_time += DT
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            front_vehicle,
            target_speed_kmh=FRONT_VEHICLE_SPEED_KMH,
            yaw_deg=front_vehicle_start_tf.rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_INITIAL_SPEED_KMH,
            yaw_deg=EGO_RAW_TRAJECTORY[0][2]
        )

        for _ in range(5):
            update_pedestrian(pedestrian, sim_time)
            world.tick()
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print(
            "[速度设置] FrontVehicle={} km/h | Ego初始={} km/h | x<={} 后 {} km/h 持续 {}s，再加速到 {} km/h".format(
                FRONT_VEHICLE_SPEED_KMH,
                EGO_INITIAL_SPEED_KMH,
                EGO_SLOW_TRIGGER_X,
                EGO_SLOW_SPEED_KMH,
                EGO_SLOW_DURATION_S,
                EGO_ACCEL_SPEED_KMH
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
            # Ego：给定轨迹 PID 循迹 + 60/25/65 km/h 状态机
            # -----------------------------
            if reached_path_end(ego, ego_path, threshold=8.0):
                soft_hold_vehicle(ego)
                set_vehicle_lights(ego, brake=True, low_beam=True)
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
