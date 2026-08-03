# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

场景主题：
山区浓雾弯道后半段低速重卡 + Ego正常循迹行驶 + 对向高速来车冲突

本版内容：
1. 完全删除 Ego 的紧急制动、TTC触发、左避让、横向偏移控制。
2. Ego 使用固定轨迹进行 PID 控制，并按 x 坐标触发 60/30/70/60 km/h 状态机车速。
3. 对向来车终点：
   Location: x=114.005, y=0.548, z=18.965
   Rotation: pitch=2.754, yaw=17.023, roll=0.000
4. 重卡蓝图强制使用 vehicle.carlamotors.firetruck，不使用可口可乐车。
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

HEAVY_TRUCK_SPEED_KMH = 15.0
HEAVY_TRUCK_MAX_SPEED_KMH = 16.0

ONCOMING_SPEED_KMH = 60.0
ONCOMING_MAX_SPEED_KMH = 68.0

EGO_CRUISE_SPEED_KMH = 60.0
EGO_SLOW_SPEED_KMH = 30.0
EGO_BOOST_SPEED_KMH = 70.0
EGO_BOOST_DURATION_S = 3.0
EGO_MAX_SPEED_KMH = 78.0
EGO_TRAJECTORY_START_Z_HINT = 20.0
EGO_SPAWN_Z_OFFSET = 0.75

# ============================================================
# 3. 全局可调接口：天气参数
# ============================================================

cloudiness = 45.0
precipitation = 5.0
precipitation_deposits = 25.0
wind_intensity = 20.0
sun_azimuth_angle = 90.0
sun_altitude_angle = 0.001
fog_density = 1
fog_distance = 1.0
fog_falloff = 0.1
wetness = 75.0
scattering_intensity = 1
mie_scattering_scale = 0.0300
rayleigh_scattering_scale = 0.0331
dust_storm = 0.0

# ============================================================
# 4. 全局可调接口：车辆蓝图候选
# ============================================================

EGO_BP_CANDIDATES = [
    "vehicle.lincoln.mkz_2020"
]

HEAVY_TRUCK_BP_CANDIDATES = [
    "vehicle.carlamotors.firetruck",
]

ONCOMING_BP_CANDIDATES = [
    "vehicle.dodge.charger_2020",
    "vehicle.audi.tt",
    "vehicle.tesla.model3"
]

# ============================================================
# 5. 全局可调接口：Ego 固定轨迹
# ============================================================

EGO_TRAJECTORY = [
    (165.803, 8.202, -172.011), (164.644, 8.027, -170.647), (163.409, 7.800, -169.265), (162.166, 7.564, -169.005),
    (160.926, 7.299, -167.290), (159.674, 6.984, -164.576), (158.468, 6.656, -165.414), (156.512, 6.147, -165.414),
    (154.052, 5.507, -165.414), (150.624, 4.625, -166.259), (146.937, 3.728, -166.739), (143.285, 2.878, -166.978),
    (139.620, 2.086, -168.416), (135.945, 1.333, -168.416), (132.209, 0.547, -167.556), (128.550, -0.283, -167.216),
    (124.815, -1.096, -168.174), (121.141, -1.841, -168.773), (117.401, -2.583, -168.773), (113.717, -3.314, -168.773),
    (109.135, -4.224, -168.773), (104.149, -5.214, -168.773), (99.243, -6.187, -168.773), (94.326, -7.123, -169.373),
    (89.329, -8.054, -169.372), (84.337, -9.008, -169.132), (79.428, -9.932, -169.371), (74.518, -10.855, -169.131),
    (69.619, -11.827, -168.651), (64.640, -12.826, -168.651), (59.743, -13.809, -168.721), (54.757, -14.768, -170.140),
    (49.747, -15.591, -170.859), (44.816, -16.384, -170.859), (39.796, -17.145, -172.588), (34.757, -17.753, -173.220),
    (29.713, -18.320, -174.181), (24.741, -18.766, -175.349), (19.674, -19.065, -178.167), (14.598, -19.202, -178.939),
    (9.602, -19.216, 179.214), (4.527, -18.979, 176.361), (-0.462, -18.661, 176.361), (-5.527, -18.263, 174.025),
    (-6.355, -18.174, 173.831), (-6.355, -18.174, 173.831), (-6.355, -18.174, 173.831), (-6.355, -18.174, 173.831),
    (-6.355, -18.174, 173.831), (-6.355, -18.174, 173.831), (-6.355, -18.174, 173.831), (-6.355, -18.174, 173.831),
    (-6.355, -18.174, 173.831), (-6.355, -18.174, 173.831), (-6.355, -18.174, 173.831), (-6.603, -18.148, 173.831),
    (-11.642, -17.500, 171.182), (-16.632, -16.725, 171.182), (-21.652, -15.947, 171.182), (-26.580, -15.123, 169.611),
    (-31.529, -14.063, 165.385), (-36.398, -12.622, 162.960), (-41.173, -11.158, 162.960), (-46.029, -9.670, 162.960),
    (-50.813, -8.173, 161.296), (-55.566, -6.378, 157.960), (-60.172, -4.438, 155.891), (-64.702, -2.332, 153.954),
    (-69.225, -0.023, 152.688), (-73.233, 2.014, 153.361), (-73.233, 2.014, 153.361), (-73.233, 2.014, 153.361),
    (-73.233, 2.014, 153.361), (-73.233, 2.014, 153.361), (-73.233, 2.014, 153.361), (-74.571, 2.688, 152.941),
    (-79.026, 5.125, 147.830), (-83.129, 7.976, 142.215), (-87.114, 11.130, 141.514), (-91.098, 14.284, 142.360),
    (-95.139, 17.219, 146.635), (-99.371, 19.871, 148.259), (-103.692, 22.544, 148.259), (-107.910, 25.224, 145.926),
    (-112.021, 28.064, 145.224), (-116.093, 30.959, 143.398), (-120.118, 33.996, 141.918), (-123.980, 37.291, 135.735),
    (-127.428, 40.907, 133.235), (-130.926, 44.475, 134.658), (-134.489, 48.096, 134.374), (-137.939, 51.709, 132.188),
    (-141.193, 55.500, 129.061), (-144.276, 59.643, 123.383), (-147.054, 63.998, 122.463), (-149.808, 68.364, 121.483),
    (-152.385, 72.740, 119.085), (-154.744, 77.237, 115.547), (-156.883, 81.753, 115.337), (-158.787, 86.369, 111.016),
    (-160.606, 91.110, 109.954), (-162.130, 95.867, 106.985), (-163.544, 100.830, 102.660), (-164.590, 105.715, 101.883),
    (-165.789, 110.563, 106.366), (-167.239, 115.344, 106.997), (-168.724, 120.202, 106.997), (-170.184, 124.980, 106.927),
    (-171.507, 129.883, 102.470), (-172.365, 134.804, 98.731), (-172.962, 139.763, 97.020), (-173.573, 144.721, 97.020),
    (-174.270, 149.763, 98.224), (-174.999, 154.705, 98.364), (-175.727, 159.651, 98.224), (-176.408, 164.611, 97.734),
    (-177.092, 169.646, 97.734), (-177.764, 174.599, 97.734), (-178.451, 179.650, 97.734), (-179.134, 184.771, 97.594),
    (-179.795, 189.725, 97.594), (-180.467, 194.763, 97.594), (-181.128, 199.735, 97.454), (-181.776, 204.693, 97.454),
    (-182.436, 209.742, 97.454), (-183.081, 214.785, 96.894), (-183.691, 219.832, 96.894), (-183.801, 220.742, 96.894),
    (-183.801, 220.742, 96.894), (-183.801, 220.742, 96.894), (-183.801, 220.742, 96.894)
]

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

    if len(EGO_TRAJECTORY) < 2:
        raise RuntimeError("EGO_TRAJECTORY 锚点不足。")

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

def cleanup_all_vehicle_actors(client, world, actor_list):
    vehicles_by_id = {}
    for actor in actor_list:
        if actor and actor.is_alive:
            vehicles_by_id[actor.id] = actor

    try:
        for actor in world.get_actors().filter("vehicle.*"):
            if actor and actor.is_alive:
                vehicles_by_id[actor.id] = actor
    except Exception as e:
        print("[RTB104] Failed to enumerate vehicle actors during cleanup:", e)

    if vehicles_by_id:
        commands = [carla.command.DestroyActor(actor.id) for actor in vehicles_by_id.values()]
        try:
            client.apply_batch_sync(commands, True)
        except Exception:
            client.apply_batch(commands)
        try:
            world.tick()
        except Exception:
            pass
    actor_list.clear()
    print("[RTB104] Destroyed {} vehicle actors.".format(len(vehicles_by_id)))

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

def get_driving_waypoint_at_xy(carla_map, x, y, search_z):
    loc = carla.Location(x=x, y=y, z=search_z)
    waypoint = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if waypoint is None:
        raise RuntimeError("No driving waypoint near ({:.3f}, {:.3f}, z_hint={:.3f})".format(x, y, search_z))
    return waypoint

def make_transform_from_xy_yaw(carla_map, p, search_z=EGO_TRAJECTORY_START_Z_HINT):
    waypoint = get_driving_waypoint_at_xy(carla_map, p[0], p[1], search_z)
    z = waypoint.transform.location.z
    return carla.Transform(
        carla.Location(x=p[0], y=p[1], z=z),
        carla.Rotation(yaw=p[2])
    )

def raw_traj_to_xyz(raw_traj):
    return [(p[0], p[1], p[2]) for p in raw_traj]

def xy_yaw_traj_to_xyz(raw_traj, carla_map, z_offset=EGO_SPAWN_Z_OFFSET, start_z_hint=EGO_TRAJECTORY_START_Z_HINT):
    path = []
    search_z = start_z_hint
    for x, y, _yaw in raw_traj:
        waypoint = get_driving_waypoint_at_xy(carla_map, x, y, search_z)
        search_z = waypoint.transform.location.z
        z = search_z + z_offset
        path.append((x, y, z))
    return path

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

def build_ego_path(carla_map):
    raw_points = xy_yaw_traj_to_xyz(
        EGO_TRAJECTORY,
        carla_map,
        z_offset=EGO_SPAWN_Z_OFFSET,
        start_z_hint=EGO_TRAJECTORY_START_Z_HINT
    )
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

        ego_path = build_ego_path(carla_map)

        oncoming_path = build_route_by_global_planner(
            carla_map=carla_map,
            start_loc=ONCOMING_START_TF.location,
            end_loc=ONCOMING_END_TF.location,
            resolution=ROUTE_RESOLUTION_M
        )

        print("[路径检查] heavy_truck_path points:", len(heavy_truck_path))
        print("[路径检查] ego_path points:", len(ego_path))
        print("[路径检查] oncoming_path points:", len(oncoming_path))
        print("[控制策略] Ego 使用固定轨迹 PID 循迹，并按状态机控制目标速度。")

        # ====================================================
        # 12.3 生成三辆车
        # ====================================================
        heavy_truck_start_tf = make_transform_from_raw_point(HEAVY_TRUCK_RAW_TRAJ[0])
        ego_start_tf = make_transform_from_xy_yaw(carla_map, EGO_TRAJECTORY[0], EGO_TRAJECTORY_START_Z_HINT)
        print(
            "[RTB104] Ego spawn base z={:.3f}, final z={:.3f}, z_hint={:.3f}".format(
                ego_start_tf.location.z,
                ego_start_tf.location.z + EGO_SPAWN_Z_OFFSET,
                EGO_TRAJECTORY_START_Z_HINT
            )
        )

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
            tf=ego_start_tf,
            color="0,80,255",
            role_name="ego",
            z_offset=EGO_SPAWN_Z_OFFSET
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
            target_speed_kmh=EGO_CRUISE_SPEED_KMH,
            yaw_deg=ego_start_tf.rotation.yaw
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
            EGO_CRUISE_SPEED_KMH,
            ONCOMING_SPEED_KMH
        ))

        # ====================================================
        # 12.6 主循环
        # ====================================================
        sim_time = 0.0
        frame_count = 0
        ego_speed_state = "CRUISE"
        ego_boost_start_time = None

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
            # Ego: PID follows EGO_TRAJECTORY with speed-state targets.
            # -----------------------------
            if reached_path_end(ego, ego_path, threshold=6.0):
                print("[RTB104] Ego reached trajectory end; destroying all vehicle actors.")
                cleanup_all_vehicle_actors(client, world, actor_list)
                break
            else:
                ego_loc = ego.get_location()
                if ego_speed_state == "CRUISE" and ego_loc.x <= -6.0:
                    ego_speed_state = "SLOW"
                    print("[RTB104] Ego speed state -> SLOW (30 km/h).")
                if ego_speed_state == "SLOW" and ego_loc.x <= -73.0:
                    ego_speed_state = "BOOST"
                    ego_boost_start_time = sim_time
                    print("[RTB104] Ego speed state -> BOOST (70 km/h).")
                if (
                    ego_speed_state == "BOOST"
                    and ego_boost_start_time is not None
                    and sim_time - ego_boost_start_time >= EGO_BOOST_DURATION_S
                ):
                    ego_speed_state = "RECOVER"
                    print("[RTB104] Ego speed state -> RECOVER (60 km/h).")

                if ego_speed_state == "SLOW":
                    ego_target_speed = EGO_SLOW_SPEED_KMH
                elif ego_speed_state == "BOOST":
                    ego_target_speed = EGO_BOOST_SPEED_KMH
                else:
                    ego_target_speed = EGO_CRUISE_SPEED_KMH

                ego_idx = follow_path_constant_speed(
                    vehicle=ego,
                    path=ego_path,
                    path_index=ego_idx,
                    pid_lon=ego_pid_lon,
                    pid_lat=ego_pid_lat,
                    target_speed_kmh=ego_target_speed,
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
