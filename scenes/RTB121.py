import carla
import time
import math
import numpy as np

# ==========================================
# 1. 轨迹数据定义
# ==========================================
RAW_EGO_TRAJ = [
    (81.48, 43.313, -88.683), (81.48, 43.313, -88.683), (81.48, 43.313, -88.753), (81.634, 36.815, -89.327),
    (81.559, 26.481, -90.549), (81.434, 16.486, -90.482), (81.354, 6.493, -90.412), (81.318, -3.503, -90.129),
    (81.255, -13.845, -90.629), (81.141, -24.18, -90.629), (81.021, -34.344, -90.769), (80.992, -44.668, -88.915),
    (81.242, -54.599, -88.489), (81.441, -64.825, -89.34), (81.408, -74.848, -90.61), (81.223, -83.316, -91.676),
    (81.134, -87.315, -91.391), (81.086, -89.315, -91.391), (81.086, -89.315, -91.391), (81.086, -89.315, -91.391),
    (81.013, -92.312, -91.391), (80.815, -100.47, -91.391), (80.791, -101.469, -91.391), (80.791, -101.469, -91.391),
    (80.791, -101.469, -91.391), (80.791, -101.469, -91.391), (80.791, -101.469, -91.391), (80.791, -101.469, -91.391),
    (80.791, -101.469, -91.391), (80.791, -101.469, -91.391), (80.621, -108.298, -91.603), (80.421, -118.293, -89.815),
    (80.594, -128.292, -88.74), (80.825, -138.289, -88.39), (81.116, -148.618, -88.39), (81.357, -158.615, -89.167),
    (81.482, -168.947, -89.731), (81.529, -178.948, -89.731), (81.577, -189.281, -89.731), (81.629, -199.281, -89.381),
    (81.792, -209.613, -88.887), (81.993, -219.945, -88.887), (82.157, -229.945, -89.309), (82.274, -239.943, -89.449),
    (82.341, -250.275, -89.874), (82.353, -260.275, -90.084), (82.319, -270.276, -90.224), (82.279, -280.277, -90.294),
    (82.225, -290.61, -90.294), (82.419, -300.603, -82.174), (85.628, -310.172, -58.934), (93.494, -316.492, -17.311),
    (103.664, -318.15, -2.883), (113.988, -318.4, -0.034), (124.317, -318.36, 0.596), (134.483, -318.253, 0.596),
    (138.149, -318.215, 0.596), (138.149, -318.215, 0.596), (138.149, -318.215, 0.596)
]

RAW_V2_TRAJ = [
    (79.586, -309.055, 87.987), (79.586, -309.055, 88.197), (79.591, -308.888, 88.407), (79.857, -298.893, 88.975),
    (79.937, -288.556, 89.825), (79.967, -278.557, 89.825), (79.615, -268.568, 94.995), (78.854, -258.593, 93.489),
    (78.746, -248.593, 89.862), (78.728, -238.422, 90.43), (78.618, -228.256, 90.998), (78.368, -218.092, 91.639),
    (78.125, -207.928, 91.289), (77.976, -197.929, 90.228), (77.936, -187.843, 90.228), (77.894, -177.507, 90.228),
    (77.853, -167.173, 90.228), (77.812, -156.84, 90.228), (77.816, -146.841, 89.59), (77.899, -136.506, 89.87),
    (77.794, -126.339, 91.003), (77.573, -116.01, 91.428), (77.385, -105.839, 90.58), (77.381, -95.506, 89.667),
    (77.432, -85.34, 89.807), (77.371, -75.178, 90.797), (77.264, -64.88, 90.159), (77.323, -55.019, 89.519),
    (77.447, -44.802, 88.668), (77.466, -34.808, 90.481), (77.267, -24.818, 91.702), (77.054, -14.491, 89.697),
    (77.106, -4.491, 89.697), (76.956, 5.504, 91.421), (76.883, 15.669, 89.743), (76.948, 25.823, 89.741),
    (76.973, 35.977, 89.953), (76.934, 46.133, 90.513), (76.841, 56.459, 90.513), (76.756, 66.451, 90.233),
    (76.714, 76.775, 90.233), (76.672, 87.101, 90.233), (76.63, 97.259, 90.233), (76.588, 107.584, 90.233),
    (76.528, 117.743, 90.518), (76.147, 127.892, 93.42), (75.392, 138.024, 94.921), (74.376, 147.968, 97.362),
    (72.337, 158.086, 106.556), (68.768, 167.593, 113.929), (63.996, 176.746, 121.703), (58.326, 185.382, 125.249),
    (51.849, 193.203, 134.543), (44.497, 199.976, 140.312), (36.246, 206.189, 146.12), (27.61, 211.217, 153.463),
    (18.414, 215.128, 158.965), (8.949, 218.325, 164.244), (-0.852, 220.2, 174.063), (-9.812, 220.867, 177.054),
    (-9.812, 220.867, 177.054), (-9.812, 220.867, 177.054)
]

RAW_PED1_TRAJ = [(101.741, -129.613, -177.626), (101.741, -129.613, -177.696), (101.741, -129.613, -178.046),
                 (101.741, -129.613, -178.046), (101.741, -129.613, -176.603), (99.249, -129.792, -175.898),
                 (91.621, -130.494, -179.031), (84.627, -130.671, 178.556), (80.467, -130.5, 175.295),
                 (72.025, -129.575, 170.972), (68.254, -128.888, 169.184), (61.143, -128.123, -178.887),
                 (57.187, -128.673, -170.632), (53.735, -129.243, -170.632), (53.735, -129.243, -170.632),
                 (53.735, -129.243, -170.632)]
RAW_PED2_TRAJ = [(54.794, -146.364, 3.777), (54.794, -146.364, 3.777), (54.794, -146.364, 3.777),
                 (59.106, -146.08, 3.777), (64.593, -145.876, -0.098), (73.41, -147.422, -20.759),
                 (79.449, -150.232, -25.452), (83.482, -151.81, -24.466), (86.681, -153.223, -23.343),
                 (87.292, -153.488, -44.683), (89.565, -155.926, -45.445), (89.8, -156.163, -2.096),
                 (95.128, -156.346, -2.248), (99.097, -156.777, -4.768), (101.664, -158.845, -49.058),
                 (101.664, -158.845, -68.386), (101.664, -158.845, -68.532), (101.834, -161.167, -86.58),
                 (101.912, -162.663, -87.961), (101.912, -162.663, -87.961), (101.912, -162.663, -87.961)]
RAW_PED3_TRAJ = [(54.697, -183.68, 0.61), (54.697, -183.68, 0.251), (54.697, -183.68, 0.251), (60.178, -183.656, 0.251),
                 (65.339, -183.816, -1.997), (68.667, -183.944, -3.007), (78.763, -185.669, -27.831),
                 (80.605, -187.664, -67.125), (82.605, -192.961, -69.242), (84.347, -197.469, -53.282),
                 (84.347, -197.469, 3.008), (85.449, -197.089, 19.675), (87.798, -196.244, 11.393),
                 (90.595, -195.804, 5.02), (90.595, -195.804, -3.544), (90.595, -195.804, -21.967),
                 (92.506, -196.387, -12.573), (93.158, -196.523, 17.077), (93.158, -196.523, 17.077),
                 (93.158, -196.523, 17.077)]


# ==========================================
# 2. 核心辅助函数
# ==========================================
def remove_duplicate_points(trajectory, min_dist=0.5):
    """去除重复或距离过近的轨迹点，防止车辆/行人PID卡顿"""
    if not trajectory: return []
    cleaned = [trajectory[0]]
    for pt in trajectory[1:]:
        last_pt = cleaned[-1]
        dist = math.hypot(pt[0] - last_pt[0], pt[1] - last_pt[1])
        if dist >= min_dist:
            cleaned.append(pt)
    return cleaned


# 清洗数据
EGO_TRAJECTORY = remove_duplicate_points(RAW_EGO_TRAJ)
V2_TRAJECTORY = remove_duplicate_points(RAW_V2_TRAJ)
PED1_TRAJECTORY = remove_duplicate_points(RAW_PED1_TRAJ)
PED2_TRAJECTORY = remove_duplicate_points(RAW_PED2_TRAJ)
PED3_TRAJECTORY = remove_duplicate_points(RAW_PED3_TRAJ)


def get_vehicle_speed_kmh(vehicle):
    """获取车辆当前速度(km/h)用于打印"""
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


# 基础控制算法 (PID)
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        _ie = np.clip(_ie, -2.0, 2.0)
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 0.8)


class PIDLateralController:
    def __init__(self, K_P=1.0, K_I=0.01, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint_loc, vehicle_transform):
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)
        target_vector = np.array([waypoint_loc.x - v_loc.x, waypoint_loc.y - v_loc.y])
        norm = np.linalg.norm(target_vector)
        if norm < 0.1: return 0.0
        target_yaw = math.atan2(target_vector[1], target_vector[0])
        error = target_yaw - v_yaw
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.7, 0.7)


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed_kmh, target_loc):
    target_speed_ms = target_speed_kmh / 3.6
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    current_speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed_ms, current_speed_ms)
    steer_output = pid_lat.run_step(target_loc, tf)
    if abs(steer_output) < 0.02: steer_output = 0.0

    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)
    vehicle.apply_control(control)


def walker_move_step(walker, target_loc, target_speed):
    """简单的行人走向目标点逻辑"""
    loc = walker.get_location()
    vec_x = target_loc.x - loc.x
    vec_y = target_loc.y - loc.y
    distance = math.hypot(vec_x, vec_y)

    control = carla.WalkerControl()
    if distance > 0.5:
        # 归一化方向向量
        control.direction = carla.Vector3D(vec_x / distance, vec_y / distance, 0)
        control.speed = target_speed
    else:
        control.speed = 0.0  # 到达节点时停止
    walker.apply_control(control)


# ==========================================
# 3. 主程序
# ==========================================
def main():
    print(">>> 正在连接 CARLA 仿真器...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    print(">>> 连接成功！正在设置环境...")

    # 【设置精确天气】 严格按照截图的参数配置
    weather = carla.WeatherParameters(
        cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
        wind_intensity=10.0, sun_azimuth_angle=-1.0, sun_altitude_angle=15.0,
        fog_density=2.0, fog_distance=0.7500, fog_falloff=0.1000, wetness=0.0,
        scattering_intensity=1.0, mie_scattering_scale=0.0300, rayleigh_scattering_scale=0.0331,
        dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_v2 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        print("---------------------------------------")
        print(">>> 开始生成实体...")
        # ================= 生成 Actor =================
        # 1. Ego (Chevrolet Impala)
        bp_ego = bp_lib.find('vehicle.chevrolet.impala')
        ego_loc = carla.Location(x=EGO_TRAJECTORY[0][0], y=EGO_TRAJECTORY[0][1], z=0.5)
        ego_loc.z = carla_map.get_waypoint(ego_loc).transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego, carla.Transform(ego_loc, carla.Rotation(yaw=EGO_TRAJECTORY[0][2])))
        if ego:
            actor_list.append(ego)
            print("✅ 成功: Ego车辆 (Chevrolet Impala)")
        else:
            print("❌ 失败: Ego车辆生成受阻 (可能存在碰撞)")

        # 2. V2 (BMW Grandtourer)
        bp_v2 = bp_lib.find('vehicle.bmw.grandtourer')
        v2_loc = carla.Location(x=V2_TRAJECTORY[0][0], y=V2_TRAJECTORY[0][1], z=0.5)
        v2_loc.z = carla_map.get_waypoint(v2_loc).transform.location.z + 0.5
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(v2_loc, carla.Rotation(yaw=V2_TRAJECTORY[0][2])))
        if v2:
            actor_list.append(v2)
            print("✅ 成功: V2车辆 (BMW Grandtourer)")
        else:
            print("❌ 失败: V2车辆生成受阻")

        # 3. 行人生成
        bp_walker = bp_lib.filter('walker.pedestrian.*')[0]

        # Ped 1
        p1_loc = carla.Location(x=PED1_TRAJECTORY[0][0], y=PED1_TRAJECTORY[0][1], z=1.0)
        p1_loc.z = carla_map.get_waypoint(p1_loc).transform.location.z + 0.5
        ped1 = world.try_spawn_actor(bp_walker, carla.Transform(p1_loc, carla.Rotation(yaw=PED1_TRAJECTORY[0][2])))
        if ped1:
            actor_list.append(ped1)
            print("✅ 成功: 行人1")
        else:
            print("❌ 失败: 行人1生成受阻")

        # Ped 2
        p2_loc = carla.Location(x=PED2_TRAJECTORY[0][0], y=PED2_TRAJECTORY[0][1], z=1.0)
        p2_loc.z = carla_map.get_waypoint(p2_loc).transform.location.z + 1
        ped2 = world.try_spawn_actor(bp_walker, carla.Transform(p2_loc, carla.Rotation(yaw=PED2_TRAJECTORY[0][2])))
        if ped2:
            actor_list.append(ped2)
            print("✅ 成功: 行人2")
        else:
            print("❌ 失败: 行人2生成受阻")

        # Ped 3
        p3_loc = carla.Location(x=PED3_TRAJECTORY[0][0], y=PED3_TRAJECTORY[0][1], z=1.0)
        p3_loc.z = carla_map.get_waypoint(p3_loc).transform.location.z + 1
        ped3 = world.try_spawn_actor(bp_walker, carla.Transform(p3_loc, carla.Rotation(yaw=PED3_TRAJECTORY[0][2])))
        if ped3:
            actor_list.append(ped3)
            print("✅ 成功: 行人3")
        else:
            print("❌ 失败: 行人3生成受阻")
        print("---------------------------------------")

        print(">>> 等待物理引擎预热贴地 (防止瞬移)...")
        for _ in range(10):
            world.tick()

        print(">>> 预热完毕，正在赋予车辆初始速度 (60 km/h)...")
        init_speed_ms = 60.0 / 3.6
        if ego:
            ego_yaw_rad = math.radians(EGO_TRAJECTORY[0][2])
            ego.set_target_velocity(
                carla.Vector3D(init_speed_ms * math.cos(ego_yaw_rad), init_speed_ms * math.sin(ego_yaw_rad), 0.0))
        if v2:
            v2_yaw_rad = math.radians(V2_TRAJECTORY[0][2])
            v2.set_target_velocity(
                carla.Vector3D(init_speed_ms * math.cos(v2_yaw_rad), init_speed_ms * math.sin(v2_yaw_rad), 0.0))

        # 状态变量初始化
        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds
        ego_idx = v2_idx = ped1_idx = ped2_idx = ped3_idx = 0
        ego_active = bool(ego)
        v2_active = bool(v2)

        # 用于确保单次播报的Flag
        p1_started = p2_started = p3_started = False
        p1_done = p2_done = p3_done = False

        frame_count = 0

        print("\n🚀 仿真正式开始！\n")
        while True:
            start_time = time.time()
            world.tick()
            frame_count += 1

            # 当前相对时间(秒)
            sim_time = world.get_snapshot().timestamp.elapsed_seconds - start_sim_time

            # ================= 每秒播报一次仿真进度 =================
            if frame_count % 20 == 0:
                ego_spd = get_vehicle_speed_kmh(ego) if ego_active else 0.0
                v2_spd = get_vehicle_speed_kmh(v2) if v2_active else 0.0
                print(
                    f"[Time: {sim_time:.1f}s] | Ego 进度: {ego_idx}/{len(EGO_TRAJECTORY)} (速: {ego_spd:.1f}km/h) | V2 进度: {v2_idx}/{len(V2_TRAJECTORY)} (速: {v2_spd:.1f}km/h)")

            # =============== 车辆逻辑 (目标速度均为60km/h) ===============
            if ego_active and ego.is_alive:
                if ego_idx < len(EGO_TRAJECTORY):
                    tx, ty, tyaw = EGO_TRAJECTORY[ego_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=ego.get_location().z)
                    if ego.get_location().distance(target_loc) < 3.5 and ego_idx < len(EGO_TRAJECTORY) - 1:
                        ego_idx += 1
                    apply_pid_control(ego, pid_ego['lon'], pid_ego['lat'], 60.0, target_loc)
                else:
                    ego.apply_control(carla.VehicleControl(brake=1.0))
                    print("🏁 Ego 车辆已到达轨迹终点。")
                    ego_active = False

            if v2_active and v2.is_alive:
                if v2_idx < len(V2_TRAJECTORY):
                    tx, ty, tyaw = V2_TRAJECTORY[v2_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v2.get_location().z)
                    if v2.get_location().distance(target_loc) < 3.5 and v2_idx < len(V2_TRAJECTORY) - 1:
                        v2_idx += 1
                    apply_pid_control(v2, pid_v2['lon'], pid_v2['lat'], 60.0, target_loc)
                else:
                    v2.apply_control(carla.VehicleControl(brake=1.0))
                    print("🏁 V2 车辆已到达轨迹终点。")
                    v2_active = False

            # =============== 行人逻辑 (按时间触发) ===============
            # Pedestrian 1: 3s 开始行走 (提升为 2.5 m/s 约9km/h 快走)
            if ped1 and ped1.is_alive and not p1_done:
                if sim_time >= 3.0:
                    if not p1_started:
                        print(f"⏰ [Time: {sim_time:.1f}s] 行人1 开始行走！")
                        p1_started = True

                    if ped1_idx < len(PED1_TRAJECTORY):
                        tx, ty, tyaw = PED1_TRAJECTORY[ped1_idx]
                        target_loc = carla.Location(x=tx, y=ty, z=ped1.get_location().z)
                        if ped1.get_location().distance(target_loc) < 1.0 and ped1_idx < len(PED1_TRAJECTORY) - 1:
                            ped1_idx += 1
                        walker_move_step(ped1, target_loc, target_speed=2.5)
                    else:
                        ped1.apply_control(carla.WalkerControl(speed=0.0))  # 静止
                        print("🏁 行人1 已走完设定轨迹。")
                        p1_done = True

            # Pedestrian 2: 4s 开始跑步 (提升为 6.0 m/s 约21km/h 冲刺跑)
            if ped2 and ped2.is_alive and not p2_done:
                if sim_time >= 4.0:
                    if not p2_started:
                        print(f"⏰ [Time: {sim_time:.1f}s] 行人2 开始跑步！")
                        p2_started = True

                    if ped2_idx < len(PED2_TRAJECTORY):
                        tx, ty, tyaw = PED2_TRAJECTORY[ped2_idx]
                        target_loc = carla.Location(x=tx, y=ty, z=ped2.get_location().z)
                        if ped2.get_location().distance(target_loc) < 1.0 and ped2_idx < len(PED2_TRAJECTORY) - 1:
                            ped2_idx += 1
                        walker_move_step(ped2, target_loc, target_speed=6.0)
                    else:
                        ped2.apply_control(carla.WalkerControl(speed=0.0))
                        print("🏁 行人2 已跑完设定轨迹。")
                        p2_done = True

            # Pedestrian 3: 5s 开始跑步 (提升为 6.0 m/s 约21km/h 冲刺跑)
            if ped3 and ped3.is_alive and not p3_done:
                if sim_time >= 5.0:
                    if not p3_started:
                        print(f"⏰ [Time: {sim_time:.1f}s] 行人3 开始跑步！")
                        p3_started = True

                    if ped3_idx < len(PED3_TRAJECTORY):
                        tx, ty, tyaw = PED3_TRAJECTORY[ped3_idx]
                        target_loc = carla.Location(x=tx, y=ty, z=ped3.get_location().z)
                        if ped3.get_location().distance(target_loc) < 1.0 and ped3_idx < len(PED3_TRAJECTORY) - 1:
                            ped3_idx += 1
                        walker_move_step(ped3, target_loc, target_speed=6.0)
                    else:
                        ped3.apply_control(carla.WalkerControl(speed=0.0))
                        print("🏁 行人3 已跑完设定轨迹。")
                        p3_done = True

            # 帧率同步控制
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n⚠️ 键盘中断，终止运行。")
    finally:
        print("\n>>> 正在清理环境并恢复异步设置...")
        for actor in actor_list:
            if actor and actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print(">>> 清理完毕，程序退出。")


if __name__ == '__main__':
    main()