# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 RoadTailBench 场景脚本
主题：信号设施问题 + 主路两辆白色厢式货车遮挡 + 支路静止白色厢式货车 + 支路车辆汇入

本版本修改点：
1. 删除所有状态机逻辑，不再使用 RTB.MultiStageBehaviorMachine。
2. 所有车辆速度独立设置：
   - Ego 独立速度
   - Truck1 独立速度
   - Truck2 独立速度
   - Merge 独立速度
3. 支路汇入车通过 MERGE_START_TIME_S 控制启动时间。
4. 场景中所有运动车辆均使用轨迹列表寻迹。
5. Ego 使用用户提供的主车轨迹列表。
6. 两辆主路货车基于主车轨迹派生，保持类似轨迹。
7. 支路汇入车使用用户提供的完整轨迹列表。
8. 所有车辆均开启 Position + LowBeam，静止车额外 Brake。
9. 每辆车使用独立 PID，禁止共用。
"""

import sys
import time
import math
import traceback
import carla

# =========================================================
# 0. 动态引入标准化函数库路径
# =========================================================
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

import RoadTailBenchInitV9 as RTB

# =========================================================
# 1. 基础配置
# =========================================================
HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

DT = 0.05
SCENARIO_DURATION = 80.0
TRAJ_INTERVAL = 0.5
KEEP_ACTORS_AFTER_SCRIPT = False

# =========================================================
# 1.1 所有车辆独立速度设置
# =========================================================
EGO_TARGET_SPEED_KMH = 40.0
TRUCK1_TARGET_SPEED_KMH = 45.0
TRUCK2_TARGET_SPEED_KMH = 45.0
MERGE_TARGET_SPEED_KMH = 45.0

# 支路汇入车开始行驶时间，单位秒。
MERGE_START_TIME_S = 3.2

# 每辆车速度硬上限，防止 PID 超调
EGO_MAX_SPEED_KMH = 45.0
TRUCK1_MAX_SPEED_KMH = 50.0
TRUCK2_MAX_SPEED_KMH = 50.0
MERGE_MAX_SPEED_KMH = 50.0

# 按你当前最新位置要求换算出的沿轨迹前移距离
TRUCK1_ADVANCE_M = 8
TRUCK2_ADVANCE_M = 18
BACKGROUND_TRAJ_EXTEND_M = 30.0

# =========================================================
# 2. 用户提供轨迹列表
# =========================================================
# 格式：(x, y, z, pitch, yaw, roll)
EGO_RAW_TRAJ = [
    (5.105, 88.198, 1.560, -2.153, -94.246, 0.000),
    (5.105, 88.198, 1.560, -2.223, -95.156, 0.000),
    (5.105, 88.198, 1.560, -2.153, -94.736, 0.000),
    (3.827, 70.758, 1.403, 1.277, -90.816, 0.000),
    (4.011, 35.485, 0.937, -2.292, -88.575, 0.000),
    (4.560, 8.375, 0.850, 0.087, -87.664, 0.000),
    (5.053, -5.616, 0.695, 0.017, -86.474, 0.000),
    (6.930, -20.645, 0.728, 0.087, -79.143, 0.000),
    (7.254, -33.679, 0.981, 2.805, -93.233, 0.000),
    (7.420, -53.209, 1.303, 0.006, -88.403, 0.000),
    (8.224, -83.234, 0.783, -1.744, -89.243, 0.000),
    (8.631, -102.476, 0.551, 0.076, -88.963, 0.000),
    (9.046, -130.472, 0.698, 0.006, -90.642, 0.000)
]

MERGE_RAW_TRAJ = [
    (78.630, -50.815, 1.193, -1.041, 178.028, 0.000),
    (77.672, -50.794, 1.175, -1.041, 179.497, 0.000),
    (72.631, -50.735, 1.126, -0.131, 178.937, 0.000),
    (67.616, -50.686, 1.095, -0.848, -179.498, 0.000),
    (62.388, -50.641, 1.055, 0.062, 178.710, 0.000),
    (54.838, -50.954, 0.914, -1.338, -176.600, 0.000),
    (47.285, -51.266, 0.815, -0.638, -174.709, 0.000),
    (39.849, -52.562, 0.768, 0.062, -163.577, 0.000),
    (32.968, -55.373, 0.803, -0.008, -155.021, 0.000),
    (26.290, -59.372, 0.793, 0.342, -143.400, 0.000),
    (20.699, -64.441, 0.951, 1.602, -131.501, 0.000),
    (16.531, -70.734, 1.107, 0.342, -118.290, 0.000),
    (13.259, -77.619, 1.069, -0.708, -112.338, 0.000),
    (10.885, -84.791, 1.086, 0.622, -104.988, 0.000),
    (9.589, -92.293, 1.177, 0.762, -94.701, 0.000),
    (9.430, -99.850, 1.280, -0.708, -89.451, 0.000),
    (9.384, -107.407, 1.025, -1.337, -90.360, 0.000),
    (9.341, -115.031, 1.075, 0.972, -90.850, 0.000),
    (9.032, -122.586, 1.184, 0.762, -92.599, 0.000),
    (8.870, -126.145, 1.231, 0.762, -92.599, 0.000),
    (9.241, -130.099, 1.245, -0.007, -90.327, 0.000),
    (9.192, -136.349, 1.244, -0.007, -90.465, 0.000),
    (9.141, -142.598, 1.244, -0.007, -90.465, 0.000),
    (9.091, -148.848, 1.243, -0.007, -90.465, 0.000),
    (9.040, -155.098, 1.242, -0.007, -90.465, 0.000),
    (8.988, -161.429, 1.241, -0.007, -90.465, 0.000),
]

STATIC_TRUCK_TF = carla.Transform(
    carla.Location(x=22.285, y=-51.274, z=1.0),
    carla.Rotation(pitch=-22.019, yaw=-178.130, roll=0.000)
)

# =========================================================
# 3. 辅助函数
# =========================================================
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

def make_dense_traj_from_raw(raw_traj, interval=0.5):
    """
    将用户提供的轨迹列表转换为 PID 可用的稠密路径点。
    """
    raw_points = [(p[0], p[1], p[2]) for p in raw_traj]
    raw_points = RTB.clean_trajectory(raw_points, min_dist=1e-5)
    dense = RTB.interpolate_trajectory(raw_points, interval=interval)
    clean = RTB.clean_trajectory(dense, min_dist=0.5)
    return clean

def get_cumulative_distances(raw_traj):
    """
    计算轨迹累计弧长。
    """
    points = [(p[0], p[1], p[2], p[4]) for p in raw_traj]
    cum = [0.0]

    for i in range(1, len(points)):
        x1, y1 = points[i - 1][0], points[i - 1][1]
        x2, y2 = points[i][0], points[i][1]
        cum.append(cum[-1] + math.hypot(x2 - x1, y2 - y1))

    return cum

def sample_raw_traj_at_s(raw_traj, cum_dist, s):
    """
    按累计弧长 s 从 raw_traj 中采样一个点。
    超出终点时，沿最后一个点的 yaw 方向继续延伸。
    """
    if not raw_traj:
        return None

    if s <= 0.0:
        return raw_traj[0]

    total_len = cum_dist[-1]

    if s >= total_len:
        last = raw_traj[-1]
        extra = s - total_len
        yaw_rad = math.radians(last[4])
        x = last[0] + math.cos(yaw_rad) * extra
        y = last[1] + math.sin(yaw_rad) * extra
        z = last[2]
        return (x, y, z, last[3], last[4], last[5])

    for i in range(1, len(raw_traj)):
        if cum_dist[i] >= s:
            s0 = cum_dist[i - 1]
            s1 = cum_dist[i]
            ratio = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)

            p0 = raw_traj[i - 1]
            p1 = raw_traj[i]

            x = p0[0] + (p1[0] - p0[0]) * ratio
            y = p0[1] + (p1[1] - p0[1]) * ratio
            z = p0[2] + (p1[2] - p0[2]) * ratio

            geom_yaw = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
            pitch = p0[3] + (p1[3] - p0[3]) * ratio
            roll = p0[5] + (p1[5] - p0[5]) * ratio

            return (x, y, z, pitch, geom_yaw, roll)

    return raw_traj[-1]

def advance_raw_traj_along_path(raw_traj, advance_m):
    """
    生成前方车辆轨迹：
    基于 Ego 原始轨迹，整体沿路径弧长提前 advance_m。
    """
    clean_raw = []
    last_xy = None

    for p in raw_traj:
        xy = (p[0], p[1])
        if last_xy is None or math.hypot(xy[0] - last_xy[0], xy[1] - last_xy[1]) > 1e-4:
            clean_raw.append(p)
            last_xy = xy

    cum = get_cumulative_distances(clean_raw)
    advanced = []

    for s in cum:
        advanced.append(sample_raw_traj_at_s(clean_raw, cum, s + advance_m))

    return advanced

def extend_raw_traj_forward(raw_traj, extend_m):
    """
    Extend a raw trajectory by adding one extra point ahead of its current end.
    The extension follows the last point's yaw direction and preserves pitch/roll.
    """
    if not raw_traj or extend_m <= 0.0:
        return list(raw_traj)

    clean_raw = []
    last_xy = None
    for p in raw_traj:
        xy = (p[0], p[1])
        if last_xy is None or math.hypot(xy[0] - last_xy[0], xy[1] - last_xy[1]) > 1e-4:
            clean_raw.append(p)
            last_xy = xy

    cum = get_cumulative_distances(clean_raw)
    extended_end = sample_raw_traj_at_s(clean_raw, cum, cum[-1] + extend_m)
    return list(raw_traj) + [extended_end]

def get_start_tf_from_raw(raw_traj):
    return make_transform_from_raw_point(raw_traj[0])

def get_end_tf_from_raw(raw_traj):
    return make_transform_from_raw_point(raw_traj[-1])

def set_vehicle_lights(vehicle, brake=False, high_beam=False):
    """
    所有车辆开启：
    - Position 位置灯
    - LowBeam 近光灯

    静止车额外 Brake。
    """
    if not vehicle or not vehicle.is_alive:
        return

    try:
        mask = 0
        mask |= int(carla.VehicleLightState.Position)
        mask |= int(carla.VehicleLightState.LowBeam)

        if high_beam:
            mask |= int(carla.VehicleLightState.HighBeam)

        if brake:
            mask |= int(carla.VehicleLightState.Brake)

        vehicle.set_light_state(carla.VehicleLightState(mask))

    except Exception as e:
        print("[灯光警告] 设置车辆灯光失败：", e)

def get_speed_kmh(vehicle):
    if not vehicle or not vehicle.is_alive:
        return 0.0

    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

def distance_2d(loc_a, loc_b):
    return math.hypot(loc_a.x - loc_b.x, loc_a.y - loc_b.y)

def reached_end(vehicle, end_tf, threshold=4.0):
    if not vehicle or not vehicle.is_alive:
        return True

    return distance_2d(vehicle.get_location(), end_tf.location) <= threshold

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

def safe_follow_path(vehicle, path, path_index, pid_lon, pid_lat, target_speed_kmh,
                     max_speed_kmh, max_throttle=0.55, max_brake=0.85,
                     max_steer=0.45, lookahead_ratio=0.40):
    """
    使用车辆自己的路径 + 自己的 PID 进行轨迹跟踪。
    """
    if not vehicle or not vehicle.is_alive or not path:
        return path_index

    current_speed = get_speed_kmh(vehicle)

    if current_speed > max_speed_kmh + 1.0:
        vehicle.apply_control(carla.VehicleControl(
            throttle=0.0,
            brake=min(0.45, max_brake),
            steer=0.0,
            hand_brake=False
        ))
        return path_index

    target_wp, path_index = RTB.get_target_waypoint(
        vehicle.get_location(),
        path,
        path_index,
        speed_kmh=current_speed,
        min_lookahead=5.0,
        lookahead_ratio=lookahead_ratio,
        max_search_ahead=40,
        fallback_dist=35.0
    )

    if target_wp is None or target_speed_kmh <= 0.1:
        soft_hold_vehicle(vehicle)
        return path_index

    tf = vehicle.get_transform()

    lon_output = pid_lon.run_step(target_speed_kmh / 3.6, current_speed / 3.6)
    steer_output = pid_lat.run_step(target_wp, tf)

    control = carla.VehicleControl()
    control.steer = float(steer_output) if abs(steer_output) >= 0.02 else 0.0
    control.steer = max(-max_steer, min(max_steer, control.steer))

    if lon_output >= 0.0:
        control.throttle = min(float(lon_output), max_throttle)
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = min(abs(float(lon_output)), max_brake)

    control.hand_brake = False
    vehicle.apply_control(control)

    return path_index

def print_world_sync_state(world):
    settings = world.get_settings()
    print(
        "[同步检查] synchronous_mode={} | fixed_delta_seconds={} | max_substeps={} | max_substep_delta_time={}".format(
            settings.synchronous_mode,
            settings.fixed_delta_seconds,
            settings.max_substeps,
            settings.max_substep_delta_time
        )
    )

# =========================================================
# 4. 主函数
# =========================================================

def main():
    actor_list = []

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        bp_lib = world.get_blueprint_library()

        # =================================================
        # 4.1 环境初始化：同步模式 + 固定步长 + 天气
        # =================================================
        RTB.enable_synchronous_mode(world, dt=DT)
        print_world_sync_state(world)

        RTB.set_static_weather(
            world,
            preset=None,

            cloudiness=45.0,
            precipitation=5.0,
            precipitation_deposits=25.0,
            wind_intensity=20.0,

            sun_azimuth_angle=90.0,
            sun_altitude_angle=0.001,

            fog_density=1,
            fog_distance=1.0,
            fog_falloff=0.1,
            wetness=75.0,

            scattering_intensity=1,
            mie_scattering_scale=0.0300,
            rayleigh_scattering_scale=0.0331,

            dust_storm=0.0
        )
        print("[场景配置] 同步模式与天气系统已设置。")

        # =================================================
        # 4.2 轨迹列表处理
        # =================================================
        TRUCK1_RAW_TRAJ = advance_raw_traj_along_path(EGO_RAW_TRAJ, TRUCK1_ADVANCE_M)
        TRUCK2_RAW_TRAJ = advance_raw_traj_along_path(EGO_RAW_TRAJ, TRUCK2_ADVANCE_M)
        TRUCK1_RAW_TRAJ = extend_raw_traj_forward(TRUCK1_RAW_TRAJ, BACKGROUND_TRAJ_EXTEND_M)
        TRUCK2_RAW_TRAJ = extend_raw_traj_forward(TRUCK2_RAW_TRAJ, BACKGROUND_TRAJ_EXTEND_M)
        MERGE_RUNTIME_RAW_TRAJ = extend_raw_traj_forward(MERGE_RAW_TRAJ, BACKGROUND_TRAJ_EXTEND_M)

        traj_ego = make_dense_traj_from_raw(EGO_RAW_TRAJ, interval=TRAJ_INTERVAL)
        traj_truck1 = make_dense_traj_from_raw(TRUCK1_RAW_TRAJ, interval=TRAJ_INTERVAL)
        traj_truck2 = make_dense_traj_from_raw(TRUCK2_RAW_TRAJ, interval=TRAJ_INTERVAL)
        traj_merge = make_dense_traj_from_raw(MERGE_RUNTIME_RAW_TRAJ, interval=TRAJ_INTERVAL)

        EGO_START_TF = get_start_tf_from_raw(EGO_RAW_TRAJ)
        EGO_END_TF = get_end_tf_from_raw(EGO_RAW_TRAJ)

        TRUCK1_START_TF = get_start_tf_from_raw(TRUCK1_RAW_TRAJ)
        TRUCK1_END_TF = get_end_tf_from_raw(TRUCK1_RAW_TRAJ)

        TRUCK2_START_TF = get_start_tf_from_raw(TRUCK2_RAW_TRAJ)
        TRUCK2_END_TF = get_end_tf_from_raw(TRUCK2_RAW_TRAJ)

        MERGE_START_TF = get_start_tf_from_raw(MERGE_RUNTIME_RAW_TRAJ)
        MERGE_END_TF = get_end_tf_from_raw(MERGE_RUNTIME_RAW_TRAJ)

        print("[轨迹配置] Ego 点数：", len(traj_ego))
        print("[轨迹配置] Truck1 点数：", len(traj_truck1))
        print("[轨迹配置] Truck2 点数：", len(traj_truck2))
        print("[轨迹配置] Merge 点数：", len(traj_merge))

        # =================================================
        # 4.3 车辆实体安全生成
        # =================================================
        ego_bp = choose_existing_blueprint(bp_lib, [
            "vehicle.tesla.model3"
        ])

        box_truck_bp = choose_existing_blueprint(bp_lib, [
            "vehicle.mercedes.sprinter",
            "vehicle.carlamotors.carlacola",
            "vehicle.carlamotors.firetruck"
        ])

        merge_bp = choose_existing_blueprint(bp_lib, [
            "vehicle.audi.etron",
            "vehicle.nissan.patrol",
            "vehicle.lincoln.mkz_2020",
            "vehicle.tesla.model3"
        ])

        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        if ego is None:
            raise RuntimeError("Ego 生成失败。")
        ego.set_transform(EGO_START_TF)

        truck1 = RTB.spawn_vehicle(
            world, box_truck_bp,
            x=TRUCK1_START_TF.location.x,
            y=TRUCK1_START_TF.location.y,
            z=TRUCK1_START_TF.location.z,
            yaw=TRUCK1_START_TF.rotation.yaw,
            color="255,255,255",
            role_name="occlusion_truck_1",
            z_offset=1.0
        )
        if truck1 is None:
            raise RuntimeError("Truck1 生成失败。")
        actor_list.append(truck1)
        truck1.set_transform(TRUCK1_START_TF)

        truck2 = RTB.spawn_vehicle(
            world, box_truck_bp,
            x=TRUCK2_START_TF.location.x,
            y=TRUCK2_START_TF.location.y,
            z=TRUCK2_START_TF.location.z,
            yaw=TRUCK2_START_TF.rotation.yaw,
            color="255,255,255",
            role_name="occlusion_truck_2",
            z_offset=1.0
        )
        if truck2 is None:
            raise RuntimeError("Truck2 生成失败。")
        actor_list.append(truck2)
        truck2.set_transform(TRUCK2_START_TF)

        static_truck = RTB.spawn_vehicle(
            world, box_truck_bp,
            x=STATIC_TRUCK_TF.location.x,
            y=STATIC_TRUCK_TF.location.y,
            z=STATIC_TRUCK_TF.location.z,
            yaw=STATIC_TRUCK_TF.rotation.yaw,
            color="255,255,255",
            role_name="static_waiting_box_truck",
            z_offset=1.0
        )
        if static_truck is None:
            raise RuntimeError("支路静止白色厢式货车生成失败。")
        actor_list.append(static_truck)
        static_truck.set_transform(STATIC_TRUCK_TF)

        merge_car = RTB.spawn_vehicle(
            world, merge_bp,
            x=MERGE_START_TF.location.x,
            y=MERGE_START_TF.location.y,
            z=MERGE_START_TF.location.z,
            yaw=MERGE_START_TF.rotation.yaw,
            color="40,40,40",
            role_name="merging_vehicle",
            z_offset=0.7
        )
        if merge_car is None:
            raise RuntimeError("支路汇入车生成失败。")
        actor_list.append(merge_car)
        merge_car.set_transform(MERGE_START_TF)

        RTB.force_vehicle_stop(static_truck)
        soft_hold_vehicle(merge_car)

        # =================================================
        # 4.4 每辆车独立 PID
        # =================================================

        truck1_pid_lon = RTB.PIDLongitudinalController(
            dt=DT, preset="truck", output_clip=(-0.85, 0.60), i_clip=(-0.8, 0.8)
        )
        truck1_pid_lat = RTB.PIDLateralController(
            dt=DT, preset="truck", output_clip=(-0.38, 0.38), i_clip=(-0.8, 0.8)
        )

        truck2_pid_lon = RTB.PIDLongitudinalController(
            dt=DT, preset="truck", output_clip=(-0.85, 0.60), i_clip=(-0.8, 0.8)
        )
        truck2_pid_lat = RTB.PIDLateralController(
            dt=DT, preset="truck", output_clip=(-0.38, 0.38), i_clip=(-0.8, 0.8)
        )

        merge_pid_lon = RTB.PIDLongitudinalController(
            dt=DT, preset="wet_road", output_clip=(-0.80, 0.50), i_clip=(-0.8, 0.8)
        )
        merge_pid_lat = RTB.PIDLateralController(
            dt=DT, preset="wet_road", output_clip=(-0.45, 0.45), i_clip=(-0.8, 0.8)
        )

        ego_idx = 0
        truck1_idx = 0
        truck2_idx = 0
        merge_idx = 0

        # =================================================
        # 4.5 车辆灯光
        # =================================================
        set_vehicle_lights(ego, brake=False)
        set_vehicle_lights(truck1, brake=False)
        set_vehicle_lights(truck2, brake=False)
        set_vehicle_lights(merge_car, brake=False)
        set_vehicle_lights(static_truck, brake=True)

        # =================================================
        # 4.6 同步预热与初速度注入
        # =================================================
        print("[预热] 开始同步预热。")
        for _ in range(20):
            RTB.force_vehicle_stop(static_truck)
            soft_hold_vehicle(merge_car)

            set_vehicle_lights(ego, brake=False)
            set_vehicle_lights(truck1, brake=False)
            set_vehicle_lights(truck2, brake=False)
            set_vehicle_lights(merge_car, brake=False)
            set_vehicle_lights(static_truck, brake=True)

            world.tick()

        RTB.set_vehicle_initial_speed(
            truck1,
            target_speed_kmh=TRUCK1_TARGET_SPEED_KMH,
            yaw_deg=TRUCK1_START_TF.rotation.yaw
        )
        RTB.set_vehicle_initial_speed(
            truck2,
            target_speed_kmh=TRUCK2_TARGET_SPEED_KMH,
            yaw_deg=TRUCK2_START_TF.rotation.yaw
        )

        soft_hold_vehicle(merge_car)
        RTB.force_vehicle_stop(static_truck)

        for _ in range(5):
            RTB.force_vehicle_stop(static_truck)
            soft_hold_vehicle(merge_car)

            set_vehicle_lights(ego, brake=False)
            set_vehicle_lights(truck1, brake=False)
            set_vehicle_lights(truck2, brake=False)
            set_vehicle_lights(merge_car, brake=False)
            set_vehicle_lights(static_truck, brake=True)

            world.tick()

        print("[场景启动] 所有车辆、独立 PID、灯光、轨迹列表已配置。")

        # =================================================
        # 4.7 仿真主循环：严格 V4 同步 + 硬件时钟补齐
        # =================================================
        sim_time = 0.0
        frame_count = 0

        while sim_time < SCENARIO_DURATION:
            start_time = time.time()

            world.tick()
            sim_time += DT
            frame_count += 1

            RTB.force_vehicle_stop(static_truck)

            # =================================================
            # 不再使用状态机，所有目标速度直接独立设置
            # =================================================
            ego_target = EGO_TARGET_SPEED_KMH
            truck1_target = TRUCK1_TARGET_SPEED_KMH
            truck2_target = TRUCK2_TARGET_SPEED_KMH

            if sim_time < MERGE_START_TIME_S:
                merge_target = 0.0
            else:
                merge_target = MERGE_TARGET_SPEED_KMH

            if reached_end(ego, EGO_END_TF, threshold=5.0):
                soft_hold_vehicle(ego)
            else:
                ego_idx = safe_follow_path(
                    ego,
                    traj_ego,
                    ego_idx,
                    ego_pid_lon,
                    ego_pid_lat,
                    target_speed_kmh=ego_target,
                    max_speed_kmh=EGO_MAX_SPEED_KMH,
                    max_throttle=0.55,
                    max_brake=0.85,
                    max_steer=0.45,
                    lookahead_ratio=0.42
                )

            if reached_end(truck1, TRUCK1_END_TF, threshold=5.0):
                soft_hold_vehicle(truck1)
            else:
                truck1_idx = safe_follow_path(
                    truck1,
                    traj_truck1,
                    truck1_idx,
                    truck1_pid_lon,
                    truck1_pid_lat,
                    target_speed_kmh=truck1_target,
                    max_speed_kmh=TRUCK1_MAX_SPEED_KMH,
                    max_throttle=0.60,
                    max_brake=0.85,
                    max_steer=0.38,
                    lookahead_ratio=0.42
                )

            if reached_end(truck2, TRUCK2_END_TF, threshold=5.0):
                soft_hold_vehicle(truck2)
            else:
                truck2_idx = safe_follow_path(
                    truck2,
                    traj_truck2,
                    truck2_idx,
                    truck2_pid_lon,
                    truck2_pid_lat,
                    target_speed_kmh=truck2_target,
                    max_speed_kmh=TRUCK2_MAX_SPEED_KMH,
                    max_throttle=0.60,
                    max_brake=0.85,
                    max_steer=0.38,
                    lookahead_ratio=0.42
                )

            if merge_target <= 0.1:
                soft_hold_vehicle(merge_car)
            elif reached_end(merge_car, MERGE_END_TF, threshold=4.0):
                soft_hold_vehicle(merge_car)
            else:
                merge_idx = safe_follow_path(
                    merge_car,
                    traj_merge,
                    merge_idx,
                    merge_pid_lon,
                    merge_pid_lat,
                    target_speed_kmh=merge_target,
                    max_speed_kmh=MERGE_MAX_SPEED_KMH,
                    max_throttle=0.55,
                    max_brake=0.85,
                    max_steer=0.45,
                    lookahead_ratio=0.40
                )

            # 控制后重新设置灯光
            set_vehicle_lights(ego, brake=False)
            set_vehicle_lights(truck1, brake=False)
            set_vehicle_lights(truck2, brake=False)
            set_vehicle_lights(merge_car, brake=False)
            set_vehicle_lights(static_truck, brake=True)

            if hasattr(RTB, "check_vehicle_out_of_road"):
                for veh in [ego, truck1, truck2, merge_car]:
                    try:
                        RTB.check_vehicle_out_of_road(
                            veh,
                            carla_map,
                            threshold_dist=6.0,
                            auto_destroy=False
                        )
                    except Exception:
                        pass

            if frame_count % int(2.0 / DT) == 0:
                print(
                    "[t={:05.2f}s | frame={:04d}] Ego={:05.1f}km/h | Truck1={:05.1f} | Truck2={:05.1f} | Merge={:05.1f} | EgoY={:07.2f}".format(
                        sim_time,
                        frame_count,
                        get_speed_kmh(ego),
                        get_speed_kmh(truck1),
                        get_speed_kmh(truck2),
                        get_speed_kmh(merge_car),
                        ego.get_location().y
                    )
                )

            compute_time = time.time() - start_time
            if compute_time < DT:
                time.sleep(DT - compute_time)

        print("[场景结束] 同步仿真主循环正常结束。")

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断。")

    except Exception as e:
        print("[错误] 场景脚本发生异常：", e)
        traceback.print_exc()

    finally:
        if KEEP_ACTORS_AFTER_SCRIPT:
            print("[清理策略] KEEP_ACTORS_AFTER_SCRIPT=True，保留 Actor。")
        else:
            try:
                RTB.cleanup_actors(client, actor_list)
            except Exception as e:
                print("[清理警告] cleanup_actors 失败：", e)

        try:
            RTB.disable_synchronous_mode(world)
        except Exception:
            pass

        print("[脚本退出] 已恢复异步模式并完成清理。")

if __name__ == "__main__":
    main()
