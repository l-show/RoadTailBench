# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本

本版修改点：
1. 动态白色厢式作业车：
   - 起点保持用户指定起点不变；
   - 删除原终点直线路径；
   - 改为严格按照用户提供的锚点列表行驶；
   - 匀速 20 km/h。

2. 前车：
   - 使用用户原始前车锚点轨迹；
   - 先 100 km/h 匀速 T1 秒；
   - 再以 A1 m/s² 减速度减速 T2 秒；
   - 减速结束后保持最终速度匀速行驶。

3. Ego：
   - 起点、终点使用用户指定位置；
   - 目标速度 80 km/h；
   - 不使用前车轨迹；
   - 使用当前慢车道 waypoint 拓扑；
   - 在第一个道路分流点强制选择右侧匝道分支，避免继续走主路。

4. Debug 可视化：
   - Ego 路线：黄色线；
   - 前车路线：蓝色线；
   - 作业车路线：绿色线；
   - 三辆车当前下一个目标点实时绘制；
   - Ego 当前目标点：红色；
   - 前车当前目标点：青色；
   - 作业车当前目标点：绿色。
"""

import sys
import time
import math
import traceback
import carla


# ============================================================
# 0. 动态引入标准化函数库
# ============================================================

LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

import RoadTailBenchInitV9 as RTB


# ============================================================
# 1. 基础配置
# ============================================================

HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

DT = 0.05
SCENARIO_DURATION = 45.0
TRAJ_INTERVAL = 0.8
KEEP_ACTORS_AFTER_SCRIPT = False

# 是否绘制 debug 路线
DRAW_DEBUG_PATH = True
DRAW_NEXT_TARGET_DEBUG = True

# ============================================================
# 1.1 前车速度模型
# ============================================================

FRONT_INITIAL_SPEED_KMH = 90.0
T1_SECONDS = 7
A1_DECEL_MPS2 = 8
T2_SECONDS = 2
FRONT_MAX_SPEED_KMH = 92.0

# ============================================================
# 1.2 Ego 速度模型
# ============================================================

EGO_TARGET_SPEED_KMH = 90.0
EGO_MAX_SPEED_KMH = 92.0

# ============================================================
# 1.3 动态作业车速度
# ============================================================

WORK_VAN_TARGET_SPEED_KMH = 12.0
WORK_VAN_MAX_SPEED_KMH = 14.0

# ============================================================
# 1.4 湿滑低摩擦区域
# ============================================================

FRICTION_VALUE = 0.42
DRAW_FRICTION_DEBUG = True


# ============================================================
# 2. 用户指定车辆位置与轨迹
# ============================================================

# 2.1 动态白色厢式作业车起点，保持不变
WORK_VAN_START_TF = carla.Transform(
    carla.Location(x=-106.270, y=-550.000, z=0.100),
    carla.Rotation(pitch=0.700, yaw=-144.411, roll=0.000)
)

# 2.2 动态白色厢式作业车锚点轨迹
WORK_VAN_RAW_TRAJ = [
    (-106.834, -550.293, 0.530, 0.985, -148.096, 0.000),
    (-111.363, -553.413, 0.604, 0.775, -145.435, 0.000),
    (-111.775, -553.697, 0.611, 0.775, -145.155, 0.000),
    (-111.775, -553.697, 0.611, 0.775, -145.155, 0.000),
    (-111.775, -553.697, 0.611, 0.775, -145.155, 0.000),
    (-112.785, -554.432, 0.623, 0.565, -143.964, 0.000),
    (-114.819, -555.885, 0.643, -0.345, -149.143, 0.000),
    (-118.111, -557.930, 0.653, 0.355, -146.761, 0.000),
    (-121.130, -559.935, 0.676, 0.355, -145.711, 0.000),
    (-124.658, -562.305, 0.703, 0.425, -146.550, 0.000),
    (-127.063, -563.880, 0.727, 0.495, -147.179, 0.000),
    (-128.961, -565.088, 0.750, 0.565, -148.019, 0.000),
    (-130.985, -566.332, 0.777, 0.775, -149.418, 0.000),
    (-133.847, -567.867, 0.823, 0.845, -153.127, 0.000),
    (-136.632, -569.284, 0.867, 0.635, -152.567, 0.000),
    (-139.591, -570.906, 0.902, 0.565, -149.977, 0.000),
    (-142.718, -572.740, 0.939, 0.845, -147.526, 0.000),
    (-145.211, -574.408, 0.995, 1.195, -144.375, 0.000),
    (-147.938, -576.598, 1.066, 1.055, -138.006, 0.000),
    (-151.411, -579.837, 1.144, 0.845, -136.535, 0.000),
    (-153.588, -581.901, 1.184, 0.775, -136.535, 0.000),
    (-156.307, -584.484, 1.235, 0.775, -136.185, 0.000),
    (-158.735, -586.827, 1.282, 1.055, -135.066, 0.000),
    (-160.096, -588.291, 1.323, 1.195, -130.048, 0.000),
    (-162.831, -591.705, 1.415, 1.195, -128.367, 0.000),
    (-165.153, -594.647, 1.494, 1.405, -127.667, 0.000),
    (-167.402, -597.646, 1.600, 1.685, -126.336, 0.000),
    (-171.784, -603.870, 1.463, 2.913, -124.031, 0.000),
    (-178.042, -614.776, 2.020, 1.439, -113.001, 0.000),
    (-182.470, -626.456, 2.368, 1.999, -108.450, 0.000),
    (-185.766, -638.713, 2.963, 3.679, -103.599, 0.000),
    (-187.918, -651.089, 3.664, 1.667, -91.015, 0.000),
    (-185.116, -663.336, 3.947, 0.967, -61.646, 0.000),
    (-177.329, -673.173, 4.265, 1.667, -40.845, 0.000),
    (-173.487, -676.030, 4.384, 1.247, -34.676, 0.000),
]

# 2.3 动态前车轨迹，格式：(x, y, z, pitch, yaw, roll)
FRONT_RAW_TRAJ = [
    (15.044, -452.578, 0.591, 0.491, -146.313, 0.000),
    (15.044, -452.578, 0.591, 0.911, -147.152, 0.000),
    (7.193, -457.622, 0.757, 1.471, -148.061, 0.000),
    (-2.131, -463.454, 0.909, -1.609, -147.639, 0.000),
    (-6.219, -466.025, 0.718, -1.818, -148.057, 0.000),
    (-10.758, -468.823, 0.606, -0.768, -148.756, 0.000),
    (-17.250, -473.051, 0.732, 1.471, -144.904, 0.000),
    (-24.219, -478.093, 0.717, -2.168, -143.434, 0.000),
    (-33.597, -484.709, 0.710, 0.071, -144.974, 0.000),
    (-41.716, -490.687, 0.654, 0.282, -142.594, 0.000),
    (-50.764, -497.683, 0.710, 0.282, -141.824, 0.000),
    (-60.777, -505.337, 0.828, 0.981, -144.414, 0.000),
    (-70.686, -513.122, 1.005, 0.701, -141.554, 0.000),
    (-80.680, -520.963, 0.705, -2.378, -142.045, 0.000),
    (-90.590, -528.746, 0.470, 0.142, -141.626, 0.000),
    (-100.384, -536.676, 0.562, 0.702, -139.037, 0.000),
    (-110.028, -544.778, 0.618, -0.278, -135.631, 0.000),
    (-119.873, -552.611, 0.586, -0.050, -154.504, 0.000),
    (-129.868, -560.205, 0.510, -0.230, -136.823, 0.000),
    (-139.969, -567.685, 0.618, 0.385, -141.538, 0.000),
    (-146.435, -575.959, 0.628, -0.026, -138.743, 0.000),
    (-155.547, -584.813, 0.730, 0.744, -134.682, 0.000),
    (-164.202, -593.827, 0.924, 1.444, -130.542, 0.000),
    (-171.784, -603.870, 1.463, 2.913, -124.031, 0.000),
    (-178.042, -614.776, 2.020, 1.439, -113.001, 0.000),
    (-182.470, -626.456, 2.368, 1.999, -108.450, 0.000),
    (-185.766, -638.713, 2.963, 3.679, -103.599, 0.000),
    (-187.918, -651.089, 3.664, 1.667, -91.015, 0.000),
    (-185.116, -663.336, 3.947, 0.967, -61.646, 0.000),
    (-177.329, -673.173, 4.265, 1.667, -40.845, 0.000),
    (-173.487, -676.030, 4.384, 1.247, -34.676, 0.000),
]

# 2.4 Ego 起点与终点
EGO_START_TF = carla.Transform(
    carla.Location(x=40.157, y=-443.206, z=0.793),
    carla.Rotation(pitch=0.326, yaw=-156.765, roll=0.000)
)

EGO_END_TF = carla.Transform(
    carla.Location(x=-188.726, y=-656.154, z=3.900),
    carla.Rotation(pitch=-4.671, yaw=-68.915, roll=0.000)
)


# ============================================================
# 3. 右侧汇出匝道湿滑区域
# ============================================================

RAMP_FRICTION_TF = carla.Transform(
    carla.Location(x=-169.0, y=-615.0, z=2.8),
    carla.Rotation(pitch=0.0, yaw=-113.0, roll=0.0)
)

RAMP_FRICTION_EXTENT = (68.0, 18.0, 4.0)


# ============================================================
# 4. 基础工具函数
# ============================================================

def enable_sync(world, dt=0.05):
    if hasattr(RTB, "enable_synchronous_mode"):
        RTB.enable_synchronous_mode(world, dt=dt)
    else:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        settings.substepping = True
        settings.max_substep_delta_time = min(0.01, dt)
        settings.max_substeps = max(10, int(dt / settings.max_substep_delta_time))
        world.apply_settings(settings)


def disable_sync(world):
    if hasattr(RTB, "disable_synchronous_mode"):
        RTB.disable_synchronous_mode(world)
    else:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)


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


def warmup_map_cache(world):
    try:
        carla_map = world.get_map()
        _ = carla_map.name
        _ = carla_map.get_topology()
        _ = carla_map.generate_waypoints(20.0)
        print("[地图缓存] 已提前预热 map topology / waypoint cache。")
    except Exception as e:
        print("[地图缓存警告] 预热失败，但不影响场景继续运行：", e)


def cleanup_actors(client, actor_list):
    if hasattr(RTB, "cleanup_actors"):
        RTB.cleanup_actors(client, actor_list)
    else:
        for actor in actor_list:
            if actor and actor.is_alive:
                try:
                    actor.destroy()
                except Exception:
                    pass


def choose_existing_blueprint(bp_lib, candidates):
    for bp_name in candidates:
        try:
            bp_lib.find(bp_name)
            return bp_name
        except Exception:
            continue
    raise RuntimeError("候选蓝图均不存在：{}".format(candidates))


def make_transform_from_raw_point(p):
    return carla.Transform(
        carla.Location(x=p[0], y=p[1], z=p[2]),
        carla.Rotation(pitch=p[3], yaw=p[4], roll=p[5])
    )


def get_speed_kmh(vehicle):
    if not vehicle or not vehicle.is_alive:
        return 0.0
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)


def distance_2d(loc_a, loc_b):
    return math.hypot(loc_a.x - loc_b.x, loc_a.y - loc_b.y)


def reached_end(vehicle, end_loc, threshold=6.0):
    if not vehicle or not vehicle.is_alive:
        return True
    return distance_2d(vehicle.get_location(), end_loc) <= threshold


def soft_hold_vehicle(vehicle):
    if vehicle and vehicle.is_alive:
        vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        vehicle.apply_control(carla.VehicleControl(
            throttle=0.0,
            brake=1.0,
            steer=0.0,
            hand_brake=False
        ))


def set_vehicle_lights(vehicle, brake=False, hazard=False, low_beam=True):
    if not vehicle or not vehicle.is_alive:
        return

    try:
        mask = 0
        mask |= int(carla.VehicleLightState.Position)

        if low_beam:
            mask |= int(carla.VehicleLightState.LowBeam)

        if brake:
            mask |= int(carla.VehicleLightState.Brake)

        if hazard:
            mask |= int(carla.VehicleLightState.LeftBlinker)
            mask |= int(carla.VehicleLightState.RightBlinker)

        vehicle.set_light_state(carla.VehicleLightState(mask))

    except Exception as e:
        print("[灯光警告] 设置车辆灯光失败：", e)


def make_dense_traj_from_raw(raw_traj, interval=0.8):
    raw_points = [(p[0], p[1], p[2]) for p in raw_traj]
    raw_points = RTB.clean_trajectory(raw_points, min_dist=1e-5)
    dense = RTB.interpolate_trajectory(raw_points, interval=interval)
    dense = RTB.clean_trajectory(dense, min_dist=0.5)
    return dense


def build_front_path():
    return make_dense_traj_from_raw(FRONT_RAW_TRAJ, interval=TRAJ_INTERVAL)


def build_work_van_path():
    return make_dense_traj_from_raw(WORK_VAN_RAW_TRAJ, interval=0.5)


def get_angle_diff_signed(yaw_target, yaw_current):
    """
    返回 [-180, 180] 的有符号角度差。
    与 RTB.generate_topological_route 中的分支判断保持一致：
    diff 越大，越偏右；diff 越小，越偏左。
    """
    return (yaw_target - yaw_current + 180.0) % 360.0 - 180.0


def build_ego_lane_keep_path_force_ramp(world, start_tf, end_tf, step=1.0, max_distance=420.0):
    """
    Ego 慢车道到匝道路径。

    当前版本为了解决“Ego 不往匝道汇入”的问题：
    1. 从 Ego 起点投影到当前 Driving waypoint；
    2. 沿当前车道拓扑向前行驶；
    3. 遇到第一个自然分流点时，强制选择“右侧分支”；
    4. 之后继续选择更靠近终点的分支；
    5. 不使用 lane_change，不主动横向变道。
    """

    carla_map = world.get_map()

    current_wp = carla_map.get_waypoint(
        start_tf.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    if current_wp is None:
        raise RuntimeError("无法从 Ego 起点投影到合法 Driving waypoint。")

    path = []
    traveled = 0.0
    end_loc = end_tf.location

    forced_first_ramp_branch = False
    branch_count = 0

    while traveled < max_distance:
        loc = current_wp.transform.location
        path.append((loc.x, loc.y, loc.z))

        if distance_2d(loc, end_loc) < 7.0:
            break

        next_wps = current_wp.next(step)
        if not next_wps:
            break

        if len(next_wps) == 1:
            current_wp = next_wps[0]
        else:
            branch_count += 1
            current_yaw = current_wp.transform.rotation.yaw
            current_loc = current_wp.transform.location

            if not forced_first_ramp_branch:
                # 第一个分流点强制选择右侧分支
                best_wp = None
                best_score = -999999.0

                for wp in next_wps:
                    wp_loc = wp.transform.location
                    geom_yaw = math.degrees(
                        math.atan2(
                            wp_loc.y - current_loc.y,
                            wp_loc.x - current_loc.x
                        )
                    )
                    diff = get_angle_diff_signed(geom_yaw, current_yaw)

                    # 右转优先，同时稍微考虑离终点距离
                    dist_score = -0.01 * distance_2d(wp_loc, end_loc)
                    score = diff + dist_score

                    if score > best_score:
                        best_score = score
                        best_wp = wp

                current_wp = best_wp
                forced_first_ramp_branch = True

                print(
                    "[Ego路径] 第一个道路分流点已强制选择右侧匝道分支 | "
                    "branch_count={} | loc=({:.2f}, {:.2f})".format(
                        branch_count,
                        current_loc.x,
                        current_loc.y
                    )
                )

            else:
                # 后续分支选择更接近终点且方向平顺的分支
                def branch_score(wp):
                    wp_loc = wp.transform.location
                    dist_score = distance_2d(wp_loc, end_loc)
                    yaw_diff = abs(
                        (wp.transform.rotation.yaw - current_yaw + 180.0) % 360.0 - 180.0
                    )
                    return dist_score + 0.08 * yaw_diff

                current_wp = min(next_wps, key=branch_score)

        traveled += step

    path.append((end_tf.location.x, end_tf.location.y, end_tf.location.z))
    path = RTB.clean_trajectory(path, min_dist=0.5)

    print(
        "[Ego路径] 生成完成 | points={} | forced_first_ramp_branch={} | branch_count={}".format(
            len(path),
            forced_first_ramp_branch,
            branch_count
        )
    )

    return path


def safe_spawn_vehicle(world, candidates, tf, color=None, role_name="background", z_offset=0.6):
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

        print(
            "[生成成功] {} | role={} | loc=({:.3f}, {:.3f}, {:.3f})".format(
                bp_name,
                role_name,
                exact_tf.location.x,
                exact_tf.location.y,
                exact_tf.location.z
            )
        )

    return actor


def safe_follow_path(vehicle, path, path_index, pid_lon, pid_lat,
                     target_speed_kmh, max_speed_kmh,
                     max_throttle=0.55, max_brake=0.90,
                     max_steer=0.50, lookahead_ratio=0.42):
    """
    返回：
    - path_index
    - 当前目标点 target_wp
    """
    if not vehicle or not vehicle.is_alive or not path:
        return path_index, None

    current_speed = get_speed_kmh(vehicle)

    if current_speed > max_speed_kmh + 1.0:
        vehicle.apply_control(carla.VehicleControl(
            throttle=0.0,
            brake=min(0.45, max_brake),
            steer=0.0,
            hand_brake=False
        ))
        return path_index, None

    target_wp, path_index = RTB.get_target_waypoint(
        vehicle_loc=vehicle.get_location(),
        path_points=path,
        current_index=path_index,
        speed_kmh=current_speed,
        min_lookahead=5.5,
        lookahead_ratio=lookahead_ratio,
        max_search_ahead=50,
        fallback_dist=40.0
    )

    if target_wp is None or target_speed_kmh <= 0.1:
        soft_hold_vehicle(vehicle)
        return path_index, None

    RTB.apply_pid_control(
        vehicle=vehicle,
        pid_lon=pid_lon,
        pid_lat=pid_lat,
        target_speed_kmh=target_speed_kmh,
        target_wp=target_wp
    )

    return path_index, target_wp


def get_front_target_speed_kmh(sim_time):
    v0_mps = FRONT_INITIAL_SPEED_KMH / 3.6

    if sim_time < T1_SECONDS:
        return FRONT_INITIAL_SPEED_KMH

    decel_elapsed = sim_time - T1_SECONDS

    if decel_elapsed < T2_SECONDS:
        v_mps = max(0.0, v0_mps - A1_DECEL_MPS2 * decel_elapsed)
        return v_mps * 3.6

    v_final_mps = max(0.0, v0_mps - A1_DECEL_MPS2 * T2_SECONDS)
    return v_final_mps * 3.6


# ============================================================
# 5. Debug 绘制函数
# ============================================================

def draw_debug_path(world, path, color, label="", z_offset=0.45, life_time=999.0):
    if not path or len(path) < 2:
        return

    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]

        loc1 = carla.Location(x=p1[0], y=p1[1], z=p1[2] + z_offset)
        loc2 = carla.Location(x=p2[0], y=p2[1], z=p2[2] + z_offset)

        world.debug.draw_line(
            loc1,
            loc2,
            thickness=0.10,
            color=color,
            life_time=life_time
        )

    if label:
        first = path[0]
        world.debug.draw_string(
            carla.Location(x=first[0], y=first[1], z=first[2] + 2.2),
            label,
            draw_shadow=True,
            color=color,
            life_time=life_time
        )


def draw_debug_target(world, target_wp, color, label="", z_offset=1.2, life_time=0.08):
    if target_wp is None:
        return

    loc = carla.Location(
        x=target_wp[0],
        y=target_wp[1],
        z=target_wp[2] + z_offset
    )

    world.debug.draw_point(
        loc,
        size=0.22,
        color=color,
        life_time=life_time
    )

    if label:
        world.debug.draw_string(
            loc + carla.Location(z=0.4),
            label,
            draw_shadow=True,
            color=color,
            life_time=life_time
        )


# ============================================================
# 6. 天气设置：严格参考截图参数
# ============================================================

def apply_weather_from_user_image(world):
    RTB.set_static_weather(
        world,
        preset="ClearNoon",

        cloudiness=40.0,
        precipitation=100.0,
        precipitation_deposits=100.0,
        wind_intensity=100.0,

        sun_azimuth_angle=40.0,
        sun_altitude_angle=6.0,

        fog_density=10.0,
        fog_distance=0.750,
        fog_falloff=0.100,

        wetness=100.0,

        scattering_intensity=1.500,
        mie_scattering_scale=0.210,
        rayleigh_scattering_scale=0.070,

        dust_storm=0.0
    )

    print("[天气设置] 已按截图设置。")


# ============================================================
# 7. 主函数
# ============================================================

def main():
    actor_list = []

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    world = None

    try:
        world = client.get_world()
        carla_map = world.get_map()
        bp_lib = world.get_blueprint_library()

        # ====================================================
        # 7.1 同步模式 + 地图缓存预热 + 天气
        # ====================================================
        enable_sync(world, dt=DT)
        print_world_sync_state(world)

        warmup_map_cache(world)

        apply_weather_from_user_image(world)

        print("[预热] 同步模式预热中。")
        for _ in range(15):
            world.tick()
            time.sleep(DT)

        # ====================================================
        # 7.2 构建三辆车路径
        # ====================================================
        front_path = build_front_path()
        work_van_path = build_work_van_path()

        ego_path = build_ego_lane_keep_path_force_ramp(
            world=world,
            start_tf=EGO_START_TF,
            end_tf=EGO_END_TF,
            step=1.0,
            max_distance=420.0
        )

        print("[路径检查] front_path points:", len(front_path))
        print("[路径检查] work_van_path points:", len(work_van_path))
        print("[路径检查] ego_path points:", len(ego_path))
        print("[路径策略] Ego 不使用前车轨迹，在第一个道路分流点强制选择右侧匝道分支。")

        if DRAW_DEBUG_PATH:
            draw_debug_path(
                world,
                ego_path,
                carla.Color(255, 220, 0),
                label="EGO_PATH",
                z_offset=0.65,
                life_time=999.0
            )
            draw_debug_path(
                world,
                front_path,
                carla.Color(0, 120, 255),
                label="FRONT_PATH",
                z_offset=0.65,
                life_time=999.0
            )
            draw_debug_path(
                world,
                work_van_path,
                carla.Color(0, 255, 80),
                label="WORK_VAN_PATH",
                z_offset=0.65,
                life_time=999.0
            )

        # ====================================================
        # 7.3 生成动态白色厢式作业车
        # ====================================================
        work_van = safe_spawn_vehicle(
            world=world,
            candidates=[
                "vehicle.mercedes.sprinter",
                "vehicle.volkswagen.t2",
                "vehicle.ford.ambulance"
            ],
            tf=WORK_VAN_START_TF,
            color="255,255,255",
            role_name="moving_work_van",
            z_offset=0.85
        )

        if not work_van:
            raise RuntimeError("动态白色厢式作业车生成失败。")

        actor_list.append(work_van)
        work_van.set_autopilot(False)
        set_vehicle_lights(work_van, brake=False, hazard=True, low_beam=True)

        # ====================================================
        # 7.4 生成动态前车
        # ====================================================
        front_start_tf = make_transform_from_raw_point(FRONT_RAW_TRAJ[0])

        front_vehicle = safe_spawn_vehicle(
            world=world,
            candidates=[
                "vehicle.audi.etron",
                "vehicle.nissan.patrol",
                "vehicle.lincoln.mkz_2020",
                "vehicle.tesla.model3",
                "vehicle.audi.tt"
            ],
            tf=front_start_tf,
            color="245,245,245",
            role_name="front_vehicle_middle_lane",
            z_offset=0.65
        )

        if not front_vehicle:
            raise RuntimeError("动态前车生成失败。")

        actor_list.append(front_vehicle)
        front_vehicle.set_autopilot(False)
        set_vehicle_lights(front_vehicle, brake=False, hazard=False, low_beam=True)

        # ====================================================
        # 7.5 生成 Ego 主车
        # ====================================================
        ego = safe_spawn_vehicle(
            world=world,
            candidates=[
                "vehicle.tesla.model3",
                "vehicle.lincoln.mkz_2020",
                "vehicle.audi.tt",
                "vehicle.dodge.charger_2020"
            ],
            tf=EGO_START_TF,
            color="0,80,255",
            role_name="ego",
            z_offset=0.65
        )

        if not ego:
            raise RuntimeError("Ego 生成失败。")

        actor_list.append(ego)
        ego.set_autopilot(False)
        set_vehicle_lights(ego, brake=False, hazard=False, low_beam=True)

        # ====================================================
        # 7.6 只生成右侧汇出匝道湿滑区域
        # ====================================================
        friction_actor = RTB.spawn_friction_region(
            world=world,
            bp_lib=bp_lib,
            center_loc=RAMP_FRICTION_TF,
            friction=FRICTION_VALUE,
            extent=RAMP_FRICTION_EXTENT,
            draw_debug=DRAW_FRICTION_DEBUG,
            debug_life=999.0
        )

        if friction_actor:
            actor_list.append(friction_actor)
            print("[湿滑区域] 已生成右侧汇出匝道低摩擦区。")
        else:
            print("[警告] 湿滑区域生成失败。")

        # ====================================================
        # 7.7 每辆车独立 PID
        # ====================================================
        front_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="wet_road",
            output_clip=(-0.95, 0.65),
            i_clip=(-1.0, 1.0)
        )
        front_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="wet_road",
            output_clip=(-0.50, 0.50),
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

        work_van_pid_lon = RTB.PIDLongitudinalController(
            dt=DT,
            preset="truck",
            output_clip=(-0.80, 0.45),
            i_clip=(-1.0, 1.0)
        )
        work_van_pid_lat = RTB.PIDLateralController(
            dt=DT,
            preset="truck",
            output_clip=(-0.35, 0.35),
            i_clip=(-1.0, 1.0)
        )

        front_idx = 0
        ego_idx = 0
        work_van_idx = 0

        # ====================================================
        # 7.8 Actor 生成后同步预热与初速度注入
        # ====================================================
        print("[预热] Actor 生成后同步预热。")

        for _ in range(20):
            set_vehicle_lights(work_van, brake=False, hazard=True, low_beam=True)
            set_vehicle_lights(front_vehicle, brake=False, hazard=False, low_beam=True)
            set_vehicle_lights(ego, brake=False, hazard=False, low_beam=True)
            world.tick()
            time.sleep(DT)

        RTB.set_vehicle_initial_speed(
            work_van,
            target_speed_kmh=WORK_VAN_TARGET_SPEED_KMH,
            yaw_deg=WORK_VAN_START_TF.rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            front_vehicle,
            target_speed_kmh=FRONT_INITIAL_SPEED_KMH,
            yaw_deg=front_start_tf.rotation.yaw
        )

        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_TARGET_SPEED_KMH,
            yaw_deg=EGO_START_TF.rotation.yaw
        )

        for _ in range(5):
            world.tick()
            time.sleep(DT)

        v_final = max(
            0.0,
            FRONT_INITIAL_SPEED_KMH / 3.6 - A1_DECEL_MPS2 * T2_SECONDS
        ) * 3.6

        print("[场景启动] 所有元素配置完成。")
        print("[作业车速度模型] 白色厢式作业车匀速 {:.1f} km/h，按锚点列表行驶。".format(WORK_VAN_TARGET_SPEED_KMH))
        print("[前车速度模型] 100km/h 匀速 {:.1f}s -> 以 {:.2f}m/s² 减速 {:.1f}s -> 最终匀速 {:.1f}km/h".format(
            T1_SECONDS,
            A1_DECEL_MPS2,
            T2_SECONDS,
            v_final
        ))
        print("[Ego速度模型] Ego 始终目标速度 {:.1f}km/h，强制第一个分流点进入右侧匝道。".format(
            EGO_TARGET_SPEED_KMH
        ))

        # ====================================================
        # 7.9 仿真主循环
        # ====================================================
        sim_time = 0.0
        frame_count = 0

        work_van_end_loc = carla.Location(
            x=WORK_VAN_RAW_TRAJ[-1][0],
            y=WORK_VAN_RAW_TRAJ[-1][1],
            z=WORK_VAN_RAW_TRAJ[-1][2]
        )

        front_end_loc = carla.Location(
            x=FRONT_RAW_TRAJ[-1][0],
            y=FRONT_RAW_TRAJ[-1][1],
            z=FRONT_RAW_TRAJ[-1][2]
        )

        while sim_time < SCENARIO_DURATION:
            loop_t0 = time.time()

            world.tick()
            sim_time += DT
            frame_count += 1

            # -----------------------------
            # 动态作业车：匀速 20km/h，锚点轨迹
            # -----------------------------
            if reached_end(work_van, work_van_end_loc, threshold=4.5):
                soft_hold_vehicle(work_van)
                set_vehicle_lights(work_van, brake=True, hazard=True, low_beam=True)
                work_van_target_wp = None
            else:
                work_van_idx, work_van_target_wp = safe_follow_path(
                    vehicle=work_van,
                    path=work_van_path,
                    path_index=work_van_idx,
                    pid_lon=work_van_pid_lon,
                    pid_lat=work_van_pid_lat,
                    target_speed_kmh=WORK_VAN_TARGET_SPEED_KMH,
                    max_speed_kmh=WORK_VAN_MAX_SPEED_KMH,
                    max_throttle=0.42,
                    max_brake=0.85,
                    max_steer=0.35,
                    lookahead_ratio=0.35
                )
                set_vehicle_lights(work_van, brake=False, hazard=True, low_beam=True)

            # -----------------------------
            # 前车控制：t1/a1/t2 速度模型
            # -----------------------------
            front_target_speed = get_front_target_speed_kmh(sim_time)

            if reached_end(front_vehicle, front_end_loc, threshold=6.0):
                soft_hold_vehicle(front_vehicle)
                set_vehicle_lights(front_vehicle, brake=True, hazard=False, low_beam=True)
                front_target_wp = None
            else:
                front_idx, front_target_wp = safe_follow_path(
                    vehicle=front_vehicle,
                    path=front_path,
                    path_index=front_idx,
                    pid_lon=front_pid_lon,
                    pid_lat=front_pid_lat,
                    target_speed_kmh=front_target_speed,
                    max_speed_kmh=FRONT_MAX_SPEED_KMH,
                    max_throttle=0.65,
                    max_brake=0.95,
                    max_steer=0.50,
                    lookahead_ratio=0.40
                )

                if sim_time >= T1_SECONDS and sim_time < T1_SECONDS + T2_SECONDS:
                    set_vehicle_lights(front_vehicle, brake=True, hazard=False, low_beam=True)
                else:
                    set_vehicle_lights(front_vehicle, brake=False, hazard=False, low_beam=True)

            # -----------------------------
            # Ego 控制：始终 80km/h，强制右侧匝道分支
            # -----------------------------
            if reached_end(ego, EGO_END_TF.location, threshold=8.0):
                soft_hold_vehicle(ego)
                set_vehicle_lights(ego, brake=True, hazard=False, low_beam=True)
                ego_target_wp = None
                print("[场景结束] Ego 已到达终点附近。")
                break
            else:
                ego_idx, ego_target_wp = safe_follow_path(
                    vehicle=ego,
                    path=ego_path,
                    path_index=ego_idx,
                    pid_lon=ego_pid_lon,
                    pid_lat=ego_pid_lat,
                    target_speed_kmh=EGO_TARGET_SPEED_KMH,
                    max_speed_kmh=EGO_MAX_SPEED_KMH,
                    max_throttle=0.55,
                    max_brake=0.90,
                    max_steer=0.45,
                    lookahead_ratio=0.42
                )

                set_vehicle_lights(ego, brake=False, hazard=False, low_beam=True)

            # -----------------------------
            # 实时绘制三辆车当前目标点
            # -----------------------------
            if DRAW_NEXT_TARGET_DEBUG:
                draw_debug_target(
                    world,
                    ego_target_wp,
                    carla.Color(255, 0, 0),
                    label="EGO_NEXT",
                    life_time=0.08
                )
                draw_debug_target(
                    world,
                    front_target_wp,
                    carla.Color(0, 255, 255),
                    label="FRONT_NEXT",
                    life_time=0.08
                )
                draw_debug_target(
                    world,
                    work_van_target_wp,
                    carla.Color(0, 255, 80),
                    label="WORK_NEXT",
                    life_time=0.08
                )

            # 可选：车辆偏离道路检测
            if hasattr(RTB, "check_vehicle_out_of_road"):
                for veh in [ego, front_vehicle, work_van]:
                    try:
                        RTB.check_vehicle_out_of_road(
                            veh,
                            carla_map,
                            threshold_dist=7.0,
                            auto_destroy=False
                        )
                    except Exception:
                        pass

            # 控制台低频输出
            if frame_count % int(2.0 / DT) == 0:
                print(
                    "[t={:05.2f}s | frame={:04d}] Ego={:05.1f}km/h | Front={:05.1f}/{:05.1f}km/h | WorkVan={:05.1f}/{:05.1f}km/h | idx(E/F/W)=({}/{}/{})".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(ego),
                        get_speed_kmh(front_vehicle),
                        front_target_speed,
                        get_speed_kmh(work_van),
                        WORK_VAN_TARGET_SPEED_KMH,
                        ego_idx,
                        front_idx,
                        work_van_idx
                    )
                )

            # 硬件时钟补齐
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