import carla
import time
import math
import numpy as np

# ==========================================
# 基础控制算法 (PID) - 保持原样保留
# ==========================================
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

def check_and_handle_out_of_bounds(vehicle, carla_map):
    loc = vehicle.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    if wp_nearest is None:
        print(f"[{vehicle.type_id}] 彻底脱离地图，被销毁！")
        vehicle.destroy()
        return True
    distance = wp_nearest.transform.location.distance(loc)
    if distance > 10.0:  # 偏离投影路面6米即销毁
        print(f"[{vehicle.type_id}] 偏离道路中心 {distance:.2f} 米，判定出界被销毁！")
        vehicle.destroy()
        return True
    return False

# ==========================================
# 轨迹数据定义与去重处理
# ==========================================
RAW_V1_TRAJECTORY = [
    (56.882, 64.333, -50.55), (56.882, 64.333, -50.55), (56.882, 64.333, -50.55), (56.882, 64.333, -50.55),
    (56.908, 64.3, -51.268), (57.675, 63.317, -52.684), (58.449, 62.312, -52.124), (59.688, 60.719, -52.124),
    (62, 57.772, -51.13), (64.366, 54.863, -50.36), (66.895, 51.931, -48.579), (69.37, 49.119, -49.001),
    (71.886, 46.173, -49.705), (74.36, 43.191, -50.762), (76.735, 40.289, -50.622), (79.195, 37.295, -50.552),
    (82.345, 33.466, -50.552), (86.322, 28.645, -50.343), (90.307, 23.831, -50.693), (94.388, 18.825, -50.766),
    (98.368, 14.007, -50.342), (102.489, 9.036, -50.342), (106.41, 4.305, -50.342), (106.971, 3.629, -50.342),
    (107.593, 2.878, -50.342), (109.867, 0.136, -50.342),
    (110.048, -0.175, -51.738), (110.048, -0.175, -51.738), (110.048, -0.175, -51.738), (110.048, -0.175, -51.738),
    (110.292, -0.483, -51.878), (111.051, -1.470, -52.377), (111.834, -2.471, -52.112), (112.600, -3.460, -52.331),
    (113.377, -4.466, -52.331), (114.145, -5.481, -53.097), (114.880, -6.517, -56.397), (115.557, -7.593, -59.288),
    (116.170, -8.684, -61.742), (116.768, -9.781, -60.045), (117.443, -10.858, -55.224), (118.229, -11.855, -47.829),
    (119.126, -12.755, -43.320), (120.039, -13.609, -42.976), (120.973, -14.501, -45.301), (121.844, -15.398, -45.886),
    (122.714, -16.295, -45.886), (123.572, -17.233, -49.645), (124.355, -18.206, -52.045), (125.099, -19.211, -55.391),
    (125.814, -20.261, -55.817), (126.516, -21.295, -55.817), (127.222, -22.352, -57.381), (127.875, -23.418, -60.147),
    (128.449, -24.529, -64.588), (128.940, -25.701, -69.942), (129.297, -26.899, -76.739), (129.517, -28.151, -83.496),
    (129.584, -29.399, -89.573), (129.594, -30.649, -89.573), (129.505, -31.936, -97.368), (129.288, -33.166, -102.663),
    (128.996, -34.381, -106.081), (128.588, -35.607, -110.576), (128.113, -36.807, -113.297), (127.580, -37.961, -115.333),
    (126.988, -39.061, -119.749), (126.336, -40.153, -121.791), (125.646, -41.244, -123.867), (124.928, -42.268, -125.265),
    (124.161, -43.256, -128.245), (123.374, -44.254, -128.245), (122.584, -45.249, -129.576), (121.745, -46.175, -135.437),
    (120.830, -47.056, -136.368), (119.875, -47.862, -142.739), (118.856, -48.622, -145.775), (117.802, -49.295, -147.815),
    (116.696, -49.921, -151.326), (115.597, -50.516, -153.028), (114.462, -51.041, -155.987), (113.258, -51.508, -160.356),
    (112.042, -51.872, -166.592), (110.822, -52.145, -169.680), (109.588, -52.340, -173.121), (108.324, -52.476, -175.853),
    (107.076, -52.546, -178.540), (105.806, -52.570, -179.354), (104.556, -52.550, 177.335), (103.308, -52.487, 176.958),
    (101.290, -52.379, 176.958), (98.443, -52.180, 174.752), (94.708, -51.837, 174.752), (90.913, -51.474, 173.998),
    (87.185, -51.075, 173.872), (83.450, -50.674, 173.872), (79.655, -50.305, 175.027), (77.785, -50.169, 176.431),
    (77.785, -50.169, 176.431), (77.785, -50.169, 176.431), (77.785, -50.169, 176.431), (73.627, -49.873, 175.758),
    (68.953, -49.526, 175.758), (62.726, -49.001, 174.186), (56.513, -48.326, 173.624), (48.896, -47.475, 173.624),
    (37.517, -46.335, 174.845), (26.127, -45.292, 174.633), (14.553, -44.205, 174.633), (4.866, -43.295, 174.633),
    (-1.025, -42.741, 174.633), (-3.584, -42.402, 169.934), (-6.066, -41.863, 165.881), (-7.879, -41.391, 165.173),
    (-7.879, -41.391, 165.173), (-7.879, -41.391, 165.173)
]

RAW_V2_TRAJECTORY = [
    (0.705, -39.09, -6.755), (0.705, -39.09, -6.755), (0.705, -39.09, -6.755), (1.945, -39.244, -7.109),
    (8.145, -40.01, -6.472), (14.363, -40.626, -4.772), (20.795, -41.164, -4.194), (27.029, -41.603, -3.842),
    (33.371, -41.986, -3.408), (39.81, -42.433, -4.472), (46.137, -43, -5.534), (52.358, -43.609, -5.534),
    (58.685, -44.196, -4.969), (64.911, -44.737, -4.969), (71.346, -45.297, -4.969), (77.675, -45.852, -5.249),
    (83.898, -46.435, -5.462), (90.12, -47.03, -5.462), (94.475, -47.446, -5.462), (97.005, -47.688, -5.462),
    (99.576, -47.934, -5.462), (102.122, -48.177, -5.462), (104.611, -48.409, -4.022), (106.672, -48.478, -1.529),
    (107.181, -48.489, 0.56), (107.68, -48.468, 2.513), (108.192, -48.446, 2.513), (109.394, -48.385, 5.827),
    (110.645, -48.166, 10.522), (111.9, -47.864, 16.33), (113.082, -47.459, 20.869), (114.221, -46.946, 26.716),
    (115.35, -46.321, 31.948), (116.415, -45.592, 36.483), (117.42, -44.815, 38.766), (118.38, -43.953, 45.525),
    (119.21, -42.965, 54.643), (119.923, -41.914, 56.953), (120.633, -40.836, 56.135), (121.352, -39.764, 56.135),
    (122.072, -38.691, 56.135), (122.773, -37.632, 57.777), (123.419, -36.562, 59.881), (124.043, -35.431, 63),
    (124.558, -34.293, 68.313), (124.989, -33.076, 73.509), (125.277, -31.817, 80.457), (125.412, -30.554, 87.885),
    (125.421, -29.284, 90.757), (125.341, -28.037, 96.325), (125.135, -26.783, 102.512), (124.811, -25.534, 107.824),
    (124.376, -24.362, 112.814), (123.804, -23.205, 119.697), (123.15, -22.14, 123.367), (122.436, -21.115, 126.234),
    (121.677, -20.12, 127.671), (120.898, -19.143, 130.479), (120.044, -18.175, 132.42), (119.215, -17.24, 128.96),
    (118.43, -16.268, 128.96), (117.081, -14.6, 128.96), (114.647, -11.586, 128.173), (112.24, -8.551, 129.812),
    (109.808, -5.615, 129.46), (105.356, -0.187, 128.687), (100.491, 5.845, 129.683), (95.542, 11.809, 129.968),
    (90.729, 17.56, 129.615), (85.885, 23.448, 129.404), (80.965, 29.435, 129.474), (76.185, 35.212, 129.826),
    (71.199, 41.145, 130.675), (66.229, 46.928, 130.675), (61.181, 52.808, 130.465), (56.284, 58.653, 129.83),
    (51.508, 64.435, 129.267), (46.799, 70.276, 128.772), (42.102, 76.123, 128.772), (37.385, 81.953, 129.053),
    (32.581, 87.874, 129.053), (27.756, 93.779, 130.052), (22.891, 99.489, 130.839), (17.836, 105.367, 130.556),
    (17.267, 106.031, 130.556), (17.267, 106.031, 130.556), (17.267, 106.031, 130.556), (17.267, 106.031, 130.556)
]

def clean_trajectory(raw_trajectory, min_dist=0.1):
    """去除轨迹中的重复点（距离过近的点）"""
    cleaned = []
    for pt in raw_trajectory:
        if not cleaned:
            cleaned.append(pt)
        else:
            # 计算与上一个点的距离
            dist = math.hypot(pt[0] - cleaned[-1][0], pt[1] - cleaned[-1][1])
            if dist > min_dist:
                cleaned.append(pt)
    return cleaned

V1_TRAJECTORY = clean_trajectory(RAW_V1_TRAJECTORY)
V2_TRAJECTORY = clean_trajectory(RAW_V2_TRAJECTORY)

# ==========================================
# 主程序
# ==========================================

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 【天气系统配置】严格依据截图参数
    weather = carla.WeatherParameters(
        cloudiness=15.0,
        precipitation=0.0,
        precipitation_deposits=50.0,  # Puddles
        wind_intensity=100.0,
        sun_azimuth_angle=105.0,
        sun_altitude_angle=10.0,
        fog_density=40.0,
        fog_distance=0.75,
        fog_falloff=0.1,
        wetness=50.0,
        scattering_intensity=10.0,
        mie_scattering_scale=0.0200,
        rayleigh_scattering_scale=0.0500,
        dust_storm=0.0  # Dust
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

        pid_v1 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_v2 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= Actor 1：V1车 (Audi TT) =================
        bp_v1 = bp_lib.find('vehicle.audi.tt')
        if bp_v1.has_attribute('role_name'):
            pass
        if bp_v1.has_attribute('color'):
            bp_v1.set_attribute('color', '128,0,128')
        v1_start_x, v1_start_y, v1_start_yaw = V1_TRAJECTORY[0]
        v1_loc = carla.Location(x=v1_start_x, y=v1_start_y, z=0.5)
        v1_loc.z = carla_map.get_waypoint(v1_loc).transform.location.z + 52  # 防止卡地
        v1 = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        if v1:
            print("生成 V1 (Audi TT) 成功。")

        # ================= Actor 2：V2车 (Citroen C3) =================
        bp_v2 = bp_lib.find('vehicle.citroen.c3')
        v2_start_x, v2_start_y, v2_start_yaw = V2_TRAJECTORY[0]
        v2_loc = carla.Location(x=v2_start_x, y=v2_start_y, z=0.5)
        v2_loc.z = carla_map.get_waypoint(v2_loc).transform.location.z + 52
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(v2_loc, carla.Rotation(yaw=v2_start_yaw)))

        if v2:
            actor_list.append(v2)
            print("生成 V2 (Citroen C3) 成功。")
            v2.set_light_state(
                carla.VehicleLightState(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))

        # 【防飞天机制】先让物理引擎预热，让车辆重力生效贴近地面
        for _ in range(15):
            if v2: v2.apply_control(carla.VehicleControl(brake=1.0))
            world.tick()

        # 【初始物理速度赋予】避免从0加速
        initial_speed_ms = 30.0 / 3.6
        if v2:
            yaw_rad = math.radians(v2_start_yaw)
            v2.set_target_velocity(
                carla.Vector3D(initial_speed_ms * math.cos(yaw_rad), initial_speed_ms * math.sin(yaw_rad), 0.0))

        print("已赋予车辆 30 km/h 初始速度。仿真正式开始！\n")

        v1_traj_idx, v2_traj_idx = 0, 0
        v1_active, v2_active = bool(v1), bool(v2)

        # 状态机参数初始化
        v1_state = 'NORMAL'  # 可选: NORMAL, BRAKING, WAITING, RECOVERING
        v2_state = 'NORMAL'
        v1_wait_start_time = 0
        v2_wait_start_time = 0

        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds

        while True:
            start_time = time.time()
            world.tick()
            current_snap_time = world.get_snapshot().timestamp.elapsed_seconds
            sim_time = current_snap_time - start_sim_time

            # ==========================
            # V1 (Audi TT-EGO) 控制逻辑
            # 正常30 -> x=105急刹至15 -> 等3s -> 恢复30
            # ==========================

            # ==========================
            # V2 (Citroen C3) 控制逻辑
            # 正常30 -> x=100慢刹至15 -> 等3s -> 恢复30
            # ==========================
            if v2_active and v2.is_alive:
                if check_and_handle_out_of_bounds(v2, carla_map):
                    v2_active = False
                elif v2_traj_idx < len(V2_TRAJECTORY):
                    v2_loc_current = v2.get_location()
                    tx, ty, tyaw = V2_TRAJECTORY[v2_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v2_loc_current.z)

                    if v2_loc_current.distance(target_loc) < 3.5 and v2_traj_idx < len(V2_TRAJECTORY) - 1:
                        v2_traj_idx += 1

                    # --- 状态机判断 ---
                    current_speed_v2 = math.sqrt(v2.get_velocity().x ** 2 + v2.get_velocity().y ** 2) * 3.6
                    target_speed_v2 = 30.0  # 默认

                    if v2_state == 'NORMAL' and v2_loc_current.x >= 100.0:
                        print(f"[{sim_time:.1f}s] V2 到达 x=100 (急转弯前)，开始缓慢减速！")
                        v2_state = 'BRAKING'

                    if v2_state == 'BRAKING':
                        target_speed_v2 = 15.0
                        if current_speed_v2 <= 17.0:
                            print(f"[{sim_time:.1f}s] V2 降速完毕，维持15km/h过弯并等待3秒...")
                            v2_state = 'WAITING'
                            v2_wait_start_time = sim_time

                    elif v2_state == 'WAITING':
                        target_speed_v2 = 15.0
                        if sim_time - v2_wait_start_time >= 3.0:
                            print(f"[{sim_time:.1f}s] V2 过弯完毕，缓慢恢复30km/h！")
                            v2_state = 'RECOVERING'

                    elif v2_state == 'RECOVERING':
                        target_speed_v2 = 30.0

                    apply_pid_control(v2, pid_v2['lon'], pid_v2['lat'], target_speed_v2, target_loc)
                else:
                    v2.apply_control(carla.VehicleControl(brake=1.0))
                    v2_active = False
                    print("\nV2 (Citroen C3) 已到达轨迹终点。")

            # 结束判断
            if not v1_active and not v2_active:
                print("所有车辆已完成测试。")
                break

            # 帧率同步控制
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("清理完毕。")

if __name__ == '__main__':
    main()
