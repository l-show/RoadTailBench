# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

场景主题：
限界侵入：前方大卡车遮挡风险区域并提前左变道，
Ego 跟随前方大卡车行驶，左侧车道后方存在快速逼近车辆，
行人在风险区域附近带开始静止时间后移动。

本版修正：
1. 删除掉落物全部代码：
   - 删除 ENABLE_DROPPED_OBJECT
   - 删除 DROPPED_OBJECT_TF
   - 删除 DROPPED_OBJECT_BP_CANDIDATES
   - 删除 spawn_static_prop_by_tf()
   - 删除主函数中的掉落物生成与打印
2. 删除自车道后方跟驰车全部代码：
   - 删除 REAR_FOLLOW_SPEED_KMH / REAR_FOLLOW_MAX_SPEED_KMH
   - 删除 REAR_FOLLOW_BP_CANDIDATES
   - 删除 REAR_FOLLOW_START_TF / REAR_FOLLOW_END_TF
   - 删除 rear_follow_path
   - 删除 rear_follow actor
   - 删除 rear PID
   - 删除 rear 控制循环
   - 删除 rear 速度注入与控制台输出
3. 保留：
   - 前方大卡车：锚点轨迹，80 km/h
   - Ego：起终点路径，95 km/h
   - 左侧车道快速车：起终点路径，120 km/h
   - 行人：带开始静止时间
   - GlobalRoutePlanner 路径诊断与直线兜底
   - 同步模式
   - 独立 PID
   - 无 debug 可视化绘制
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
SCENARIO_DURATION = 65.0
KEEP_ACTORS_AFTER_SCRIPT = False

ROUTE_RESOLUTION_M = 2.0
TRAJ_INTERVAL_M = 1.0

# 如果 GlobalRoutePlanner 路径构建失败，是否自动退回为起终点直线插值路径
USE_LINEAR_FALLBACK_WHEN_ROUTE_FAIL = True


# ============================================================
# 2. 全局可调接口：车辆速度
# ============================================================

FRONT_TRUCK_SPEED_KMH = 80.0
FRONT_TRUCK_MAX_SPEED_KMH = 88.0

EGO_INITIAL_SPEED_KMH = 90.0
EGO_SLOW_SPEED_KMH = 60.0
EGO_RESTORE_SPEED_KMH = 90.0
EGO_MAX_SPEED_KMH = 105.0
EGO_SLOW_TRIGGER_X = -40.0
EGO_RESTORE_TRIGGER_X = -130.0

LEFT_REAR_FAST_SPEED_KMH = 120.0
LEFT_REAR_FAST_MAX_SPEED_KMH = 130.0


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
# 4. 全局可调接口：车辆蓝图候选
# ============================================================

EGO_BP_CANDIDATES = [
    "vehicle.tesla.model3"
]

FRONT_TRUCK_BP_CANDIDATES = [
    "vehicle.carlamotors.firetruck",
    "vehicle.mercedes.sprinter",
    "vehicle.tesla.cybertruck",
]

LEFT_REAR_FAST_BP_CANDIDATES = [
    "vehicle.audi.etron",
    "vehicle.dodge.charger_2020",
    "vehicle.lincoln.mkz_2020",
    "vehicle.tesla.model3",
]


# ============================================================
# 5. 前方大卡车轨迹锚点
# 格式：(x, y, z, pitch, yaw, roll)
# ============================================================

FRONT_TRUCK_RAW_TRAJ = [
    (65.219, -9.265, 8.406, 0.000, 174.578, 0.000),
    (65.219, -9.265, 8.406, 0.000, 174.578, 0.000),
    (65.219, -9.265, 8.406, 0.000, 174.578, 0.000),
    (65.219, -9.265, 8.406, 1.207, 174.028, -0.000),
    (63.254, -8.924, 8.568, 4.663, 170.148, 0.000),
    (57.164, -8.102, 9.053, 4.103, 175.549, 0.000),
    (52.212, -8.011, 9.284, 2.587, 179.301, 0.000),
    (42.246, -7.822, 9.745, 3.359, 178.461, 0.000),
    (32.558, -7.638, 10.348, 3.639, 179.231, 0.000),
    (21.942, -7.590, 10.718, 0.551, 179.580, 0.000),
    (11.192, -7.508, 10.836, 0.691, 179.580, 0.000),
    (0.568, -7.429, 10.960, 0.481, 179.510, 0.000),
    (-10.516, -7.399, 11.030, 0.341, 179.930, 0.000),
    (-22.912, -7.374, 11.077, -0.289, 179.650, 0.000),
    (-35.307, -7.351, 10.981, -0.287, -179.474, 0.000),
    (-47.701, -7.544, 10.998, -0.147, 179.419, 0.000),
    (-60.205, -6.663, 10.901, -0.567, 173.504, 0.000),
    (-72.528, -5.342, 10.875, 0.413, 176.118, 0.000),
    (-84.907, -4.718, 10.897, -0.217, 179.354, 0.000),
    (-97.445, -4.943, 10.841, -0.147, -176.952, 0.000),
    (-109.933, -6.085, 10.823, -0.077, -173.843, 0.000),
    (-122.286, -7.068, 10.846, 0.553, -178.423, 0.000),
    (-130.396, -8.136, 176.733), (-131.107, -8.097, 176.838), (-133.601, -7.980, 178.494), (-136.100, -7.935, 179.788),
    (-138.641, -7.927, 179.825), (-143.432, -7.912, 179.825), (-148.515, -7.896, 179.578), (-153.509, -7.849, 179.506),
    (-158.498, -7.855, -179.619), (-163.634, -7.909, -179.349), (-171.350, -7.997, -179.349),(-176.603, -8.610, 177.090), (-181.701, -8.357, 177.454), (-188.051, -8.155, 179.260), (-194.295, -8.134, 179.910),
    (-200.641, -8.130, -179.982), (-206.870, -8.133, -179.695), (-213.197, -8.209, -179.272), (-219.428, -8.288, -179.272),
    (-225.768, -8.367, -179.521), (-232.011, -8.333, 179.140), (-238.358, -8.238, 179.140), (-244.601, -8.159, 179.465),
    (-250.843, -8.101, 179.682), (-257.187, -8.114, -179.667), (-263.426, -8.150, -179.667), (-269.665, -8.186, -179.667),
    (-276.009, -8.229, -179.341), (-282.349, -8.302, -179.341), (-288.586, -8.361, -179.883), (-294.822, -8.373, -179.883),
    (-301.165, -8.324, 179.141), (-307.406, -8.230, 179.141), (-313.750, -8.160, 179.862), (-319.989, -8.263, -178.015),
    (-326.430, -8.535, -177.303), (-332.765, -8.923, -175.538), (-338.986, -9.473, -174.203), (-345.295, -10.224, -171.982),
    (-351.483, -11.105, -171.762), (-357.651, -12.100, -169.503), (-363.773, -13.297, -168.591), (-369.881, -14.529, -168.591),
    (-375.987, -15.777, -167.931), (-381.958, -17.196, -165.839), (-381.958, -17.196, -165.839), (-381.958, -17.196, -165.839),
    (-381.958, -17.196, -165.839)
]

# ============================================================
# 6. Ego 完整轨迹，格式为 (x, y, yaw)
# ============================================================

EGO_RAW_TRAJECTORY = [
    (111.701, -13.475, 169.397), (111.701, -13.475, 169.397), (111.701, -13.475, 170.038), (111.216, -13.392, 170.598),
    (109.275, -13.071, 170.598), (105.435, -12.483, 171.843), (101.724, -11.975, 171.996), (98.009, -11.473, 172.502),
    (94.226, -11.035, 174.303), (90.443, -10.695, 175.110), (86.726, -10.377, 175.110), (82.946, -10.054, 175.110),
    (79.165, -9.743, 175.711), (75.382, -9.460, 175.711), (71.659, -9.182, 175.832), (67.865, -8.941, 176.794),
    (64.064, -8.728, 176.794), (60.325, -8.519, 176.794), (56.522, -8.310, 177.154), (52.714, -8.145, 177.799),
    (48.906, -8.003, 177.920), (45.160, -7.884, 178.522), (41.351, -7.790, 178.883), (37.544, -7.763, 179.966),
    (33.799, -7.761, 179.966), (29.989, -7.759, 179.966), (26.244, -7.757, 179.966), (22.436, -7.754, 179.966),
    (18.627, -7.752, 179.966), (14.878, -7.741, 179.725), (11.066, -7.723, 179.725), (7.315, -7.705, 179.725),
    (3.502, -7.687, 179.725), (-0.248, -7.672, 179.846), (-4.060, -7.662, 179.846), (-7.810, -7.652, 179.846),
    (-11.560, -7.642, 179.846), (-15.372, -7.631, 179.846), (-19.122, -7.658, -179.497), (-22.935, -7.691, -179.497),
    (-26.685, -7.724, -179.497), (-30.434, -7.757, -179.497), (-34.248, -7.736, 178.830), (-37.995, -7.612, 177.002),
    (-41.737, -7.365, 176.031), (-45.477, -7.105, 176.031), (-49.277, -6.802, 175.284), (-50.897, -6.669, 175.284),
    (-50.897, -6.669, 175.284), (-50.897, -6.669, 175.284), (-50.897, -6.669, 175.284), (-50.897, -6.669, 175.284),
    (-50.897, -6.669, 175.284), (-50.897, -6.669, 175.284), (-50.897, -6.669, 175.284), (-50.897, -6.669, 175.284),
    (-50.897, -6.669, 175.284), (-52.952, -6.499, 175.214), (-56.686, -6.153, 174.496), (-60.480, -5.779, 174.369),
    (-64.213, -5.425, 175.239), (-68.016, -5.151, 176.879), (-71.762, -4.974, 177.339), (-75.509, -4.825, 178.240),
    (-79.383, -4.736, 179.216), (-83.258, -4.704, -179.698), (-87.010, -4.756, -178.982), (-90.759, -4.822, -178.982),
    (-94.508, -4.889, -178.982), (-98.320, -4.957, -178.982), (-102.070, -5.023, -178.982), (-105.883, -5.068, -179.945),
    (-109.633, -5.071, -179.945), (-113.383, -5.077, -179.808), (-117.196, -5.090, -179.808), (-121.071, -5.103, -179.808),
    (-124.821, -5.115, -179.808), (-129.758, -5.132, -179.808), (-134.842, -5.154, -179.395), (-139.923, -5.211, -179.258),
    (-144.922, -5.276, -179.258), (-150.774, -5.352, -179.258), (-158.269, -5.417, 179.873), (-165.753, -5.336, 178.839),
    (-173.351, -5.131, 178.528), (-180.949, -4.978, 179.426), (-188.430, -4.957, -179.505), (-195.917, -5.022, -179.505),
    (-203.526, -5.178, -178.273), (-208.390, -5.324, -178.273), (-208.390, -5.324, -178.273), (-208.390, -5.324, -178.273),
    (-208.390, -5.324, -178.273),
]


# ============================================================
# 7. 左侧车道快速车起终点
# ============================================================

LEFT_REAR_FAST_START_TF = carla.Transform(
    carla.Location(x=232.597, y=-24.785, z=0.337),
    carla.Rotation(pitch=5.941, yaw=178.381, roll=0.003)
)

LEFT_REAR_FAST_END_TF = carla.Transform(
    carla.Location(x=-286.823, y=-4.724, z=1.972),
    carla.Rotation(pitch=0.657, yaw=177.778, roll=0.000)
)


# ============================================================
# 8. 行人
# ============================================================

ENABLE_PEDESTRIAN = True

PEDESTRIAN_START_TF = carla.Transform(
    carla.Location(x=-88.322, y=-10.942, z=10.948),
    carla.Rotation(pitch=-22.052, yaw=88.528, roll=0.000)
)

PEDESTRIAN_TARGET_TF = carla.Transform(
    carla.Location(x=-88.353, y=-8.786, z=10.607),
    carla.Rotation(pitch=-6.233, yaw=90.403, roll=0.000)
)

PEDESTRIAN_START_DELAY_S = 6.0
PEDESTRIAN_WALK_SPEED_MPS = 1.20
PEDESTRIAN_STOP_DISTANCE_M = 0.35
PEDESTRIAN_SPAWN_Z_OFFSET = 0.20


# ============================================================
# 9. 基础工具函数
# ============================================================

def validate_user_inputs():
    if GlobalRoutePlanner is None:
        raise RuntimeError(
            "无法导入 CARLA GlobalRoutePlanner。请检查 CARLA_ROOT 是否正确，"
            "或者确认 PythonAPI\\carla\\agents\\navigation\\global_route_planner.py 存在。"
        )
    if len(FRONT_TRUCK_RAW_TRAJ) < 2:
        raise RuntimeError("FRONT_TRUCK_RAW_TRAJ 锚点不足。")


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


def get_ego_target_speed(ego, state):
    if not ego or not ego.is_alive:
        return EGO_SLOW_SPEED_KMH

    ego_x = ego.get_location().x

    if state["phase"] == "initial" and ego_x <= EGO_SLOW_TRIGGER_X:
        state["phase"] = "slow"
        print("[事件触发] Ego 到达 x<= {:.1f}，减速到 {} km/h".format(
            EGO_SLOW_TRIGGER_X,
            EGO_SLOW_SPEED_KMH
        ))

    if state["phase"] == "slow" and ego_x <= EGO_RESTORE_TRIGGER_X:
        state["phase"] = "restored"
        print("[事件触发] Ego 到达 x<= {:.1f}，恢复到 {} km/h".format(
            EGO_RESTORE_TRIGGER_X,
            EGO_RESTORE_SPEED_KMH
        ))

    if state["phase"] == "initial":
        return EGO_INITIAL_SPEED_KMH
    if state["phase"] == "slow":
        return EGO_SLOW_SPEED_KMH
    return EGO_RESTORE_SPEED_KMH


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
# 10. 天气设置
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
# 12. 行人控制
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
# 13. 主函数
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
        # 13.2 构建路径
        # ====================================================
        print("\n================ 路径构建阶段 ================")

        front_truck_path = build_dense_path_from_raw(
            FRONT_TRUCK_RAW_TRAJ,
            interval=TRAJ_INTERVAL_M
        )
        print("[路径构建成功] FrontTruck | manual anchor path points={}".format(len(front_truck_path)))

        ego_path = build_path_from_xy_yaw(
            carla_map=carla_map,
            raw_traj=EGO_RAW_TRAJECTORY,
            interval=TRAJ_INTERVAL_M
        )
        ego_path_type = "manual_xy_yaw"
        ego_start_tf = make_transform_from_xy_yaw(carla_map, EGO_RAW_TRAJECTORY[0])
        print("[路径构建成功] Ego | manual xy-yaw path points={}".format(len(ego_path)))

        left_rear_fast_path, left_rear_fast_path_type = build_route_with_diagnosis(
            route_name="LeftRearFast",
            carla_map=carla_map,
            start_tf=LEFT_REAR_FAST_START_TF,
            end_tf=LEFT_REAR_FAST_END_TF,
            resolution=ROUTE_RESOLUTION_M,
            fallback_interval=TRAJ_INTERVAL_M
        )

        print("\n================ 路径构建结果 ================")
        print("[路径结果] FrontTruck   : manual_anchor | points={}".format(len(front_truck_path)))
        print("[路径结果] Ego          : {} | points={}".format(ego_path_type, len(ego_path)))
        print("[路径结果] LeftRearFast : {} | points={}".format(left_rear_fast_path_type, len(left_rear_fast_path)))
        print("================================================\n")

        # ====================================================
        # 13.3 生成 Actor
        # ====================================================
        front_truck_start_tf = make_transform_from_raw_point(FRONT_TRUCK_RAW_TRAJ[0])

        front_truck = spawn_vehicle_by_tf(
            world=world,
            candidates=FRONT_TRUCK_BP_CANDIDATES,
            tf=front_truck_start_tf,
            color="220,220,220",
            role_name="front_occlusion_truck",
            z_offset=1.35
        )
        if not front_truck:
            raise RuntimeError("前方大卡车生成失败。")
        actor_list.append(front_truck)

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

        left_rear_fast = spawn_vehicle_by_tf(
            world=world,
            candidates=LEFT_REAR_FAST_BP_CANDIDATES,
            tf=LEFT_REAR_FAST_START_TF,
            color="40,180,80",
            role_name="left_rear_fast_vehicle",
            z_offset=0.75
        )
        if not left_rear_fast:
            raise RuntimeError("左侧车道快速车生成失败。")
        actor_list.append(left_rear_fast)

        if ENABLE_PEDESTRIAN:
            pedestrian = spawn_single_pedestrian(world)
            if pedestrian:
                actor_list.append(pedestrian)

        # ====================================================
        # 13.4 灯光
        # ====================================================
        set_vehicle_lights(front_truck, brake=False, hazard=False, low_beam=True)
        set_vehicle_lights(ego, brake=False, hazard=False, low_beam=True)
        set_vehicle_lights(left_rear_fast, brake=False, hazard=False, low_beam=True)

        # ====================================================
        # 13.5 独立 PID
        # ====================================================
        front_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="truck",
            output_clip=(-0.85, 0.70),
            i_clip=(-1.0, 1.0)
        )
        front_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="truck",
            output_clip=(-0.45, 0.45),
            i_clip=(-1.0, 1.0)
        )

        ego_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.75),
            i_clip=(-1.0, 1.0)
        )
        ego_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.55, 0.55),
            i_clip=(-1.0, 1.0)
        )

        left_fast_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.90, 0.80),
            i_clip=(-1.0, 1.0)
        )
        left_fast_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="default_car",
            output_clip=(-0.55, 0.55),
            i_clip=(-1.0, 1.0)
        )

        front_idx = 0
        ego_idx = 0
        left_fast_idx = 0
        ego_speed_state = {"phase": "initial"}

        # ====================================================
        # 13.6 Actor 生成后预热与初速度注入
        # ====================================================
        print("[预热] Actor 生成后同步预热。")
        sim_time = 0.0

        for _ in range(20):
            update_pedestrian(pedestrian, sim_time)
            world.tick()
            sim_time += DT
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            front_truck,
            target_speed_kmh=FRONT_TRUCK_SPEED_KMH,
            yaw_deg=front_truck_start_tf.rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_INITIAL_SPEED_KMH,
            yaw_deg=EGO_RAW_TRAJECTORY[0][2]
        )

        RTB.set_vehicle_initial_speed(
            left_rear_fast,
            target_speed_kmh=LEFT_REAR_FAST_SPEED_KMH,
            yaw_deg=LEFT_REAR_FAST_START_TF.rotation.yaw
        )

        for _ in range(5):
            update_pedestrian(pedestrian, sim_time)
            world.tick()
            sim_time += DT
            time.sleep(DT)

        print("[场景启动] 所有元素配置完成。")
        print(
            "[速度设置] FrontTruck={} km/h | Ego初始={} km/h | x<={} 后 {} km/h | x<={} 后恢复 {} km/h | LeftRearFast={} km/h".format(
                FRONT_TRUCK_SPEED_KMH,
                EGO_INITIAL_SPEED_KMH,
                EGO_SLOW_TRIGGER_X,
                EGO_SLOW_SPEED_KMH,
                EGO_RESTORE_TRIGGER_X,
                EGO_RESTORE_SPEED_KMH,
                LEFT_REAR_FAST_SPEED_KMH
            )
        )
        print("[行人设置] start_delay={}s | speed={}m/s".format(
            PEDESTRIAN_START_DELAY_S,
            PEDESTRIAN_WALK_SPEED_MPS
        ))

        # ====================================================
        # 13.7 主循环
        # ====================================================
        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            sim_time += DT
            frame_count += 1

            update_pedestrian(pedestrian, sim_time)

            # -----------------------------
            # 前方大卡车：固定 80 km/h，沿给定轨迹
            # -----------------------------
            if reached_path_end(front_truck, front_truck_path, threshold=8.0):
                soft_hold_vehicle(front_truck)
                set_vehicle_lights(front_truck, brake=True, low_beam=True)
            else:
                front_idx = follow_path_constant_speed(
                    vehicle=front_truck,
                    path=front_truck_path,
                    path_index=front_idx,
                    pid_lon=front_pid_lon,
                    pid_lat=front_pid_lat,
                    target_speed_kmh=FRONT_TRUCK_SPEED_KMH,
                    max_speed_kmh=FRONT_TRUCK_MAX_SPEED_KMH,
                    min_lookahead=7.0,
                    lookahead_ratio=0.45,
                    max_search_ahead=70,
                    fallback_dist=55.0
                )
                set_vehicle_lights(front_truck, brake=False, low_beam=True)

            # -----------------------------
            # Ego：给定轨迹 PID 循迹 + 90/60/90 km/h 状态机
            # -----------------------------
            if reached_path_end(ego, ego_path, threshold=8.0):
                soft_hold_vehicle(ego)
                set_vehicle_lights(ego, brake=True, low_beam=True)
                print("[场景结束] Ego 已到达轨迹终点附近。")
                break
            else:
                ego_target_speed = get_ego_target_speed(ego, ego_speed_state)
                ego_idx = follow_path_constant_speed(
                    vehicle=ego,
                    path=ego_path,
                    path_index=ego_idx,
                    pid_lon=ego_pid_lon,
                    pid_lat=ego_pid_lat,
                    target_speed_kmh=ego_target_speed,
                    max_speed_kmh=EGO_MAX_SPEED_KMH,
                    min_lookahead=8.0,
                    lookahead_ratio=0.48,
                    max_search_ahead=80,
                    fallback_dist=60.0
                )
                set_vehicle_lights(ego, brake=False, low_beam=True)

            # -----------------------------
            # 左侧车道快速车：固定 120 km/h
            # -----------------------------
            if reached_path_end(left_rear_fast, left_rear_fast_path, threshold=8.0):
                soft_hold_vehicle(left_rear_fast)
                set_vehicle_lights(left_rear_fast, brake=True, low_beam=True)
            else:
                left_fast_idx = follow_path_constant_speed(
                    vehicle=left_rear_fast,
                    path=left_rear_fast_path,
                    path_index=left_fast_idx,
                    pid_lon=left_fast_pid_lon,
                    pid_lat=left_fast_pid_lat,
                    target_speed_kmh=LEFT_REAR_FAST_SPEED_KMH,
                    max_speed_kmh=LEFT_REAR_FAST_MAX_SPEED_KMH,
                    min_lookahead=8.0,
                    lookahead_ratio=0.50,
                    max_search_ahead=85,
                    fallback_dist=65.0
                )
                set_vehicle_lights(left_rear_fast, brake=False, low_beam=True)

            if frame_count % int(2.0 / DT) == 0:
                ped_state = "WAIT"
                if pedestrian and pedestrian.is_alive and sim_time >= PEDESTRIAN_START_DELAY_S:
                    ped_state = "MOVE"

                print(
                    "[t={:05.2f}s | frame={:04d}] Truck={:05.1f} | Ego={:05.1f} | LeftFast={:05.1f} | idx(T/E/L)=({}/{}/{}) | Path(E/L)=({}/{}) | Ped={}".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(front_truck),
                        get_speed_kmh(ego),
                        get_speed_kmh(left_rear_fast),
                        front_idx,
                        ego_idx,
                        left_fast_idx,
                        ego_path_type,
                        left_rear_fast_path_type,
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
