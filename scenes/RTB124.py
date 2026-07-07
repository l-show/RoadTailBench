import carla
import time
import math
import numpy as np


# ==========================================
# 基础控制算法 (PID) - 优化油门上限
# ==========================================
class PIDLongitudinalController:
    # 新增 max_throttle 参数，默认0.8，可为特定车辆放宽到1.0
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.1, dt=0.05, max_throttle=0.8):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._max_throttle = max_throttle
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        _ie = np.clip(_ie, -2.0, 2.0)

        # 允许输出配置的最大油门
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, self._max_throttle)


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


def get_speed_kmh(vehicle):
    """辅助函数：获取车辆当前速度(km/h)"""
    if vehicle is None or not vehicle.is_alive: return 0.0
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


# ==========================================
# 辅助函数：去重、出界判定、赋初速度
# ==========================================
def clean_trajectory(raw_trajectory, dist_threshold=0.5):
    cleaned = []
    for pt in raw_trajectory:
        if not cleaned:
            cleaned.append(pt)
        else:
            last_pt = cleaned[-1]
            dist = math.hypot(pt[0] - last_pt[0], pt[1] - last_pt[1])
            if dist > dist_threshold:
                cleaned.append(pt)
    return cleaned


def check_and_handle_out_of_bounds(actor, carla_map, threshold=6.0):
    loc = actor.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    if wp_nearest is None:
        actor.destroy()
        return True
    distance = wp_nearest.transform.location.distance(loc)
    if distance > threshold:
        print(f"[{actor.type_id}] 偏离道路中心 {distance:.2f} 米，判定出界被销毁！")
        actor.destroy()
        return True
    return False


def apply_initial_velocity(vehicle, speed_kmh, yaw_deg):
    speed_ms = speed_kmh / 3.6
    yaw_rad = math.radians(yaw_deg)
    vehicle.set_target_velocity(carla.Vector3D(
        speed_ms * math.cos(yaw_rad),
        speed_ms * math.sin(yaw_rad),
        0.0
    ))


# ==========================================
# 原始轨迹数据提取 (保持不变)
# ==========================================
RAW_EGO_TRAJ = [(3.78, 317.128, -90.558), (3.78, 317.128, -90.558), (3.78, 317.128, -90.558), (3.776, 316.616, -90.412),
                (3.748, 307.941, -90.919), (3.572, 299.009, -90.919), (3.433, 290.347, -90.919),
                (3.315, 281.636, -90.709), (3.204, 272.647, -90.709), (3.006, 263.38, -90.354),
                (3.087, 254.857, -89.001), (3.242, 246.001, -89.071), (3.312, 237.03, -90.19), (3.281, 227.874, -90.33),
                (3.158, 219.387, -91.174), (3.089, 210.25, -89.463), (3.171, 201.448, -89.463),
                (3.253, 192.785, -89.463), (3.337, 183.725, -89.463), (3.42, 174.6, -89.743), (3.381, 166.034, -91.54),
                (2.975, 156.789, -93.151), (2.607, 148.209, -91.236), (2.603, 139.071, -89.322),
                (2.765, 130.507, -88.789), (2.924, 121.31, -89.489), (2.852, 112.669, -91.466),
                (2.632, 103.696, -91.186), (2.47, 94.594, -90.976), (2.365, 85.899, -90.486), (2.305, 77.465, -90.276),
                (2.28, 68.409, -90.136), (2.258, 59.305, -90.136), (2.237, 50.269, -90.136), (2.216, 41.575, -90.136),
                (2.194, 32.418, -90.136), (2.193, 23.684, -89.996), (2.193, 15.046, -89.996), (2.194, 5.933, -89.996),
                (2.195, -3.032, -89.996), (2.195, -12.018, -89.996), (2.196, -20.963, -89.996),
                (2.113, -29.942, -90.808), (1.997, -38.641, -90.738), (1.912, -47.817, -90.057),
                (1.905, -56.338, -89.987), (1.908, -65.435, -89.987), (1.91, -74.108, -89.987),
                (1.927, -83.164, -89.567), (1.993, -92.257, -89.637), (2.049, -101.073, -89.637),
                (2.106, -109.973, -89.637), (2.162, -118.877, -89.637), (2.2, -127.879, -90.177),
                (2.033, -136.471, -91.653), (1.771, -145.563, -91.653), (1.586, -154.153, -90.883),
                (1.504, -163.297, -90.463), (1.435, -171.89, -90.463), (1.363, -180.78, -90.463),
                (1.288, -190.089, -90.463), (1.219, -198.637, -90.463), (1.179, -207.597, -89.506),
                (1.435, -216.252, -82.09), (5.935, -223.972, -26.668), (14.474, -226.39, -7.649),
                (23.46, -226.773, 0.357), (32.303, -226.726, 0.077), (41.221, -226.639, 1.29),
                (49.741, -226.379, 2.012), (58.967, -226.103, 1.586), (67.818, -225.955, 0.606),
                (76.711, -225.875, 0.19), (85.396, -225.918, -0.362), (94.381, -225.975, -0.222),
                (103.326, -225.989, -0.082), (112.262, -225.945, 0.536), (121.133, -225.834, 0.746),
                (129.887, -225.715, 0.931), (138.711, -225.575, 0.757), (147.629, -225.457, 0.757),
                (156.611, -225.344, 0.327), (165.577, -225.292, 0.327), (174.621, -225.241, 0.327),
                (183.546, -225.19, 0.327), (192.314, -225.172, -0.7), (200.47, -225.325, -1.112),
                (200.47, -225.325, -1.112), (200.47, -225.325, -1.112)]
RAW_V2_TRAJ = [(-2.195, -71.183, 90.458),
               (-2.257, -62.394, 90.178), (-2.282, -53.444, 90.258), (-2.321, -44.692, 90.013),
               (-2.239, -36.107, 89.314), (-2.119, -27.28, 89.104), (-2.019, -17.988, 90.01), (-2.061, -8.953, 90.338),
               (-2.126, -0.259, 90.478), (-2.177, 8.556, 90.198), (-2.143, 17.464, 89.498), (-2.064, 26.524, 89.498),
               (-1.986, 35.255, 89.428), (-2.013, 44.114, 90.454), (-2.081, 52.778, 90.454), (-2.153, 61.779, 90.454),
               (-2.139, 70.5, 89.446), (-2.043, 79.507, 89.376), (-1.947, 88.289, 89.376), (-1.848, 97.164, 89.306),
               (-1.739, 106.132, 89.306), (-1.631, 115.05, 89.306), (-1.523, 123.977, 89.376),
               (-1.516, 133.096, 90.342), (-1.567, 141.682, 90.342), (-1.637, 150.668, 90.762),
               (-1.754, 159.492, 90.762), (-1.874, 168.503, 90.762), (-1.981, 177.289, 90.622),
               (-2.062, 186.22, 89.734), (-2.01, 194.966, 89.246), (-1.868, 203.559, 88.966), (-1.676, 212.657, 88.688),
               (-1.531, 221.82, 89.783), (-1.57, 230.584, 89.974), (-1.566, 239.731, 89.974), (-1.57, 248.353, 90.044),
               (-1.58, 257.361, 90.114), (-1.615, 266.272, 90.254), (-1.639, 275.068, 90.114),
               (-1.657, 284.282, 90.114), (-1.635, 293.063, 89.575), (-1.568, 302.048, 89.575),
               (-1.503, 310.782, 89.575), (-1.437, 319.66, 89.575), (-1.371, 328.626, 89.575),
               (-1.309, 337.699, 89.715), (-1.265, 346.678, 89.715), (-1.226, 355.402, 89.785),
               (-1.193, 364.291, 89.785), (-1.155, 373.333, 89.527), (-1.043, 382.377, 89.27), (-0.93, 391.272, 89.27),
               (-0.814, 400.337, 89.27), (-0.722, 409.378, 89.942), (-0.712, 418.421, 89.942),
               (-0.704, 427.171, 89.942), (-0.694, 436.212, 89.942), (-0.685, 445.107, 89.942),
               (-0.683, 447.732, 89.942), (-0.683, 447.732, 89.942), (-0.683, 447.732, 89.942),
               (-0.683, 447.732, 89.942)]
RAW_V3_TRAJ = [(5.418, 368.866, -89.47), (5.503, 360.807, -89.12), (5.612, 353.335, -89.675), (5.649, 345.668, -90.025),
               (5.614, 338.15, -90.305), (5.574, 330.547, -90.305), (5.532, 322.777, -90.305),
               (5.497, 315.105, -90.025), (5.488, 307.681, -90.234), (5.418, 299.597, -90.584),
               (5.343, 292.198, -90.584), (5.267, 284.735, -90.514), (5.266, 276.699, -89.669), (5.3, 269.406, -89.879),
               (5.292, 261.354, -90.229), (5.239, 253.95, -90.439), (5.18, 246.339, -90.439), (5.117, 238.653, -90.579),
               (5.037, 230.808, -90.579), (4.919, 223.509, -91.805), (4.448, 215.615, -95.887),
               (3.411, 207.978, -101.882), (0.175, 201.448, -131.612), (-5.497, 196.094, -140.612),
               (-11.797, 191.553, -147.047), (-18.09, 187.473, -147.047), (-24.491, 183.584, -154.032),
               (-31.824, 181.155, -171.003), (-39.341, 180.516, -176.631), (-46.661, 180.085, -176.771),
               (-54.347, 179.788, -178.858), (-61.908, 179.621, -178.358), (-69.529, 179.404, -178.568),
               (-77.426, 179.257, -179.25), (-84.784, 179.242, 179.597), (-92.489, 179.306, 179.898),
               (-100.19, 179.314, -179.822), (-108.009, 179.478, 175.85), (-115.649, 180.307, 173.032),
               (-123.44, 181.136, 176.78), (-131.153, 181.25, -179.665), (-138.761, 181.197, -179.594),
               (-146.315, 181.079, -178.921), (-154.088, 180.933, -178.921), (-161.641, 180.79, -178.921),
               (-169.265, 180.637, -178.663), (-177.016, 180.439, -178.277), (-184.762, 180.206, -178.277),
               (-192.259, 179.981, -178.277), (-199.755, 179.756, -178.277), (-202.629, 179.67, -178.277),
               (-202.629, 179.67, -178.277)]
RAW_V4_TRAJ = [(4.521, 150.128, -92.955), (4.521, 150.128, -92.955), (4.521, 150.128, -92.955),
               (4.521, 150.128, -92.955), (4.347, 146.685, -92.675), (4.287, 144.955, -89.56),
               (4.347, 142.501, -89.121), (4.287, 139.067, -90.751), (4.249, 134.177, -89.194),
               (4.361, 128.856, -88.722), (4.479, 123.585, -88.722), (4.542, 118.72, -91.289),
               (4.274, 113.561, -94.225), (3.876, 108.439, -93.563), (3.694, 103.352, -91.006),
               (3.655, 98.174, -89.635), (3.728, 93.3, -88.326), (3.872, 88.158, -88.466), (3.969, 82.891, -89.069),
               (4.051, 77.825, -89.139), (4.097, 72.723, -89.839), (4.068, 67.767, -90.539), (4.018, 62.412, -90.539),
               (3.991, 57.357, -90.119), (3.983, 52.159, -89.839), (3.997, 47.02, -89.839), (3.996, 41.979, -90.189),
               (3.978, 36.694, -90.189), (3.962, 31.74, -90.189), (3.953, 26.697, -89.909), (3.962, 21.348, -89.909),
               (3.97, 16.325, -89.909), (3.976, 11.349, -89.979), (4.006, 6.142, -89.096), (4.097, 1.279, -88.495),
               (4.248, -4.049, -88.495), (4.325, -9.182, -90.433), (4.162, -14.098, -93.381), (3.9, -19.194, -91.937),
               (3.789, -24.269, -90.908), (3.732, -29.426, -90.254), (3.746, -34.381, -89.649),
               (3.826, -39.348, -88.671), (3.95, -44.58, -88.601), (4.046, -49.474, -89.244), (4.088, -54.589, -89.804),
               (4.042, -59.742, -91.279), (3.96, -64.948, -90.558), (3.906, -69.885, -90.628), (3.86, -74.833, -90.488),
               (3.815, -80.011, -90.628), (3.759, -85.123, -90.628), (3.702, -90.317, -90.628),
               (3.641, -95.32, -90.768), (3.573, -100.349, -90.768), (3.54, -105.408, -90.37),
               (3.509, -110.245, -90.37), (3.475, -115.589, -90.37), (3.428, -122.865, -90.37),
               (3.378, -130.59, -90.37), (3.376, -137.98, -89.23), (3.499, -145.729, -89.09), (3.59, -153.479, -89.701),
               (3.576, -160.88, -90.829), (3.433, -168.239, -91.249), (3.266, -175.901, -91.249),
               (3.119, -183.448, -91.039), (3.127, -190.913, -88.653), (3.208, -194.033, -88.443),
               (3.208, -194.033, -88.443), (3.208, -194.033, -88.443), (3.208, -194.033, -88.443)]
RAW_PED_TRAJ = [(-17.012, 213.464, -52.023), (-17.012, 213.464, -52.023), (-17.012, 213.464, -51.814),
                (-14.519, 210.372, -50.904), (-11.348, 206.469, -50.904), (-7.98, 202.325, -50.834),
                (-4.887, 198.516, -50.974), (-1.469, 194.299, -50.974), (1.591, 190.524, -50.974),
                (4.849, 186.504, -50.974), (8.25, 182.718, -42.389), (12.091, 179.59, -37.281),
                (16.154, 176.497, -37.281), (20.343, 173.5, -28.614), (24.894, 171.227, -26.13),
                (29.699, 169.346, -25.914), (33.867, 166.764, -32.994), (38.126, 163.999, -33.064),
                (42.331, 161.308, -30.076), (46.956, 159.907, -3.824), (50.857, 159.727, -2.33),
                (50.857, 159.727, -2.33), (50.857, 159.727, -2.33), (50.857, 159.727, -2.33)]

EGO_TRAJ = clean_trajectory(RAW_EGO_TRAJ)
V2_TRAJ = clean_trajectory(RAW_V2_TRAJ)
V3_TRAJ = clean_trajectory(RAW_V3_TRAJ)
V4_TRAJ = clean_trajectory(RAW_V4_TRAJ)
PED_TRAJ = clean_trajectory(RAW_PED_TRAJ)


# ==========================================
# 主程序逻辑
# ==========================================
def main():
    print("\n--- 初始化 CARLA 客户端 ---")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    print("设置极端暴雨黑夜天气 (Wetness=100)...")
    weather = carla.WeatherParameters(
        cloudiness=0.0, precipitation=100.0, precipitation_deposits=90.0,
        wind_intensity=100.0, sun_azimuth_angle=0.0, sun_altitude_angle=-90.0,
        fog_density=0.0, fog_distance=0.0, fog_falloff=0.0, wetness=100.0,
        scattering_intensity=0.0, mie_scattering_scale=0.0, rayleigh_scattering_scale=0.0, dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    # 控制器与状态机：为摩托车赋予满油门权限 1.0
    pid_ego = {'lon': PIDLongitudinalController(dt=dt, max_throttle=0.8), 'lat': PIDLateralController(dt=dt)}
    pid_v2 = {'lon': PIDLongitudinalController(dt=dt, max_throttle=0.8), 'lat': PIDLateralController(dt=dt)}
    pid_v3 = {'lon': PIDLongitudinalController(dt=dt, max_throttle=1.0),
              'lat': PIDLateralController(dt=dt)}  # 摩托车解开油门限制
    pid_v4 = {'lon': PIDLongitudinalController(dt=dt, max_throttle=0.6), 'lat': PIDLateralController(dt=dt)}

    ego_state, ego_timer = 'NORMAL', 0.0
    v2_state, v2_timer = 'NORMAL', 0.0

    # 【新增】：摩托车失控状态追踪标志
    v3_crashed = False

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        print("\n--- 开始生成参与者 ---")
        # 1. Ego (Audi TT)
        bp_ego = bp_lib.find('vehicle.audi.tt')
        bp_ego.set_attribute('role_name', 'ego')
        e_x, e_y, e_yaw = EGO_TRAJ[0]
        ego_loc = carla.Location(x=e_x, y=e_y,
                                 z=carla_map.get_waypoint(carla.Location(e_x, e_y, 0)).transform.location.z + 0.5)
        ego = world.try_spawn_actor(bp_ego, carla.Transform(ego_loc, carla.Rotation(yaw=e_yaw)))
        if ego: actor_list.append(ego); print(">> Ego (Audi TT) 生成成功")

        # 2. V2 (雪佛兰)
        bp_v2 = bp_lib.find('vehicle.chevrolet.impala')
        v2_x, v2_y, v2_yaw = V2_TRAJ[0]
        v2_loc = carla.Location(x=v2_x, y=v2_y,
                                z=carla_map.get_waypoint(carla.Location(v2_x, v2_y, 0)).transform.location.z + 0.5)
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(v2_loc, carla.Rotation(yaw=v2_yaw)))
        if v2: actor_list.append(v2); print(">> V2 (雪佛兰) 生成成功")

        # 3. V3 (Yamaha YZF 摩托车)
        bp_v3 = bp_lib.find('vehicle.yamaha.yzf')
        v3_x, v3_y, v3_yaw = V3_TRAJ[0]
        v3_loc = carla.Location(x=v3_x, y=v3_y,
                                z=carla_map.get_waypoint(carla.Location(v3_x, v3_y, 0)).transform.location.z + 0.5)
        v3 = world.try_spawn_actor(bp_v3, carla.Transform(v3_loc, carla.Rotation(yaw=v3_yaw)))
        if v3:
            actor_list.append(v3)
            print(">> V3 (Yamaha YZF 摩托车) 生成成功")

            # 【核心修复】：物理外挂，大幅增加摩托车轮胎摩擦力，克服 wetness=100 带来的疯狂打滑
            physics_control = v3.get_physics_control()
            wheels = physics_control.wheels
            for w in wheels:
                w.tire_friction = 5.0  # 正常路面一般是 2.0-3.5，这里调高确保雨天抓地力
            physics_control.wheels = wheels
            v3.apply_physics_control(physics_control)
            print("   -> 已为摩托车(V3)加载高摩擦力轮胎(物理外挂)，防止雨夜打滑脱轨。")

        # 4. V4 (Sprinter 乌龟车)
        bp_v4 = bp_lib.find('vehicle.mercedes.sprinter')
        v4_x, v4_y, v4_yaw = V4_TRAJ[0]
        v4_loc = carla.Location(x=v4_x, y=v4_y,
                                z=carla_map.get_waypoint(carla.Location(v4_x, v4_y, 0)).transform.location.z + 0.5)
        v4 = world.try_spawn_actor(bp_v4, carla.Transform(v4_loc, carla.Rotation(yaw=v4_yaw)))
        if v4: actor_list.append(v4); print(">> V4 (Sprinter 乌龟车) 生成成功")

        # 5. 行人
        bp_ped = bp_lib.filter("walker.pedestrian.*")[0]
        if bp_ped.has_attribute('is_invincible'): bp_ped.set_attribute('is_invincible', 'false')
        p_x, p_y, p_yaw = PED_TRAJ[0]
        ped_loc = carla.Location(x=p_x, y=p_y,
                                 z=carla_map.get_waypoint(carla.Location(p_x, p_y, 0)).transform.location.z + 1.0)
        ped = world.try_spawn_actor(bp_ped, carla.Transform(ped_loc, carla.Rotation(yaw=p_yaw)))
        if ped: actor_list.append(ped); print(">> 行人 生成成功")

        # 【核心新增】：雨夜统一开启所有车辆的车灯
        print("\n--- 尝试开启所有车辆灯光 ---")
        for act in actor_list:
            if 'vehicle' in act.type_id:
                try:
                    act.set_light_state(carla.VehicleLightState(
                        carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))
                    print(f"[{act.type_id}] 尾灯与近光灯已开启。")
                except Exception as e:
                    print(f"[{act.type_id}] 灯光开启失败: {e}")

        print("预热物理引擎 10 帧...")
        for _ in range(10): world.tick()

        # 赋予初始物理速度
        if ego: apply_initial_velocity(ego, 60.0, EGO_TRAJ[0][2])
        if v2: apply_initial_velocity(v2, 60.0, V2_TRAJ[0][2])
        if v3: apply_initial_velocity(v3, 140.0, V3_TRAJ[0][2])  # 摩托车初速设定为140
        if v4: apply_initial_velocity(v4, 5.0, V4_TRAJ[0][2])

        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds
        idx_ego, idx_v2, idx_v3, idx_v4, idx_ped = 0, 0, 0, 0, 0
        tick_counter = 0

        print("\n================ 仿真正式开始 ================\n")

        while True:
            start_time = time.time()
            world.tick()
            tick_counter += 1
            sim_time = world.get_snapshot().timestamp.elapsed_seconds - start_sim_time

            # ==========================
            # Ego 车控制逻辑
            # ==========================
            if ego and ego.is_alive:
                if check_and_handle_out_of_bounds(ego, carla_map):
                    ego = None
                elif idx_ego < len(EGO_TRAJ):
                    tx, ty, tyaw = EGO_TRAJ[idx_ego]
                    target_loc = carla.Location(x=tx, y=ty, z=ego.get_location().z)
                    if ego.get_location().distance(target_loc) < 3.5 and idx_ego < len(EGO_TRAJ) - 1:
                        idx_ego += 1

                    target_speed = 60.0
                    curr_y = ego.get_location().y
                    curr_v = get_speed_kmh(ego)

                    # 状态机输出Debug
                    if ego_state == 'NORMAL' and curr_y <= 225.0:
                        ego_state = 'BRAKING'
                        print(f"[{sim_time:.1f}s] EGO 触发紧急制动！当前坐标 Y: {curr_y:.1f}")
                    elif ego_state == 'BRAKING':
                        target_speed = 30.0
                        if curr_v <= 32.0:
                            ego_state = 'WAITING'
                            ego_timer = sim_time
                            print(f"[{sim_time:.1f}s] EGO 已降速至30km/h，进入等待状态...")
                    elif ego_state == 'WAITING':
                        target_speed = 30.0
                        if sim_time - ego_timer >= 3.0:
                            ego_state = 'RECOVERING'
                            print(f"[{sim_time:.1f}s] EGO 等待结束，开始恢复 60km/h 巡航。")

                    apply_pid_control(ego, pid_ego['lon'], pid_ego['lat'], target_speed, target_loc)
                else:
                    ego.apply_control(carla.VehicleControl(brake=1.0))

            # ==========================
            # V2 (雪佛兰) 控制逻辑
            # ==========================
            if v2 and v2.is_alive:
                if check_and_handle_out_of_bounds(v2, carla_map):
                    v2 = None
                elif idx_v2 < len(V2_TRAJ):
                    tx, ty, tyaw = V2_TRAJ[idx_v2]
                    target_loc = carla.Location(x=tx, y=ty, z=v2.get_location().z)
                    if v2.get_location().distance(target_loc) < 3.5 and idx_v2 < len(V2_TRAJ) - 1:
                        idx_v2 += 1

                    target_speed = 60.0
                    curr_y = v2.get_location().y
                    curr_v = get_speed_kmh(v2)

                    if v2_state == 'NORMAL' and curr_y >= 145.0:
                        v2_state = 'BRAKING'
                        print(f"[{sim_time:.1f}s] V2(雪佛兰) 触发制动。")
                    elif v2_state == 'BRAKING':
                        target_speed = 30.0
                        if curr_v <= 32.0:
                            v2_state = 'WAITING'
                            v2_timer = sim_time
                    elif v2_state == 'WAITING':
                        target_speed = 30.0
                        if sim_time - v2_timer >= 3.0:
                            v2_state = 'RECOVERING'
                            print(f"[{sim_time:.1f}s] V2(雪佛兰) 恢复提速。")

                    apply_pid_control(v2, pid_v2['lon'], pid_v2['lat'], target_speed, target_loc)

                    # 爆闪远光灯
                    base_lights = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
                    if int(sim_time * 10) % 2 == 0:
                        v2.set_light_state(carla.VehicleLightState(base_lights | carla.VehicleLightState.HighBeam))
                    else:
                        v2.set_light_state(carla.VehicleLightState(base_lights))
                else:
                    v2.apply_control(carla.VehicleControl(brake=1.0))

            # ==========================
            # V3 (Yamaha 摩托车) 控制逻辑
            # ==========================
            if v3 and v3.is_alive:
                if v3_crashed:
                    # 【新增】：如果已经失控翻车，取消其PID，下发完全放开控制的指令，让其在物理引擎下真实滑行倒地
                    neutral_control = carla.VehicleControl()
                    neutral_control.throttle = 0.0
                    neutral_control.steer = 0.0
                    neutral_control.brake = 0.0
                    v3.apply_control(neutral_control)
                else:
                    # 注意这里去掉了 check_and_handle_out_of_bounds，防止它摔出去就立马原地被销毁
                    v3_loc = v3.get_location()
                    v3_transform = v3.get_transform()

                    if idx_v3 < len(V3_TRAJ):
                        tx, ty, tyaw = V3_TRAJ[idx_v3]
                        target_loc = carla.Location(x=tx, y=ty, z=v3_loc.z)
                        dist_to_target = v3_loc.distance(target_loc)
                        roll_angle = abs(v3_transform.rotation.roll)

                        # 【新增】：判定失控 - 偏离目标点>15米 或 车身发生严重侧倾(>50度)
                        if dist_to_target > 15.0 or roll_angle > 50.0:
                            print(
                                f"[{sim_time:.1f}s] 💥 V3(摩托车) 飙车失控飞出！(偏离:{dist_to_target:.1f}m, 侧倾角:{roll_angle:.1f}°)")
                            print("   -> 已永久取消其 PID 强制寻迹，交由纯物理引擎接管模拟真实翻滚倒地。")
                            v3_crashed = True
                        else:
                            if dist_to_target < 5.0 and idx_v3 < len(V3_TRAJ) - 1:
                                idx_v3 += 1
                            apply_pid_control(v3, pid_v3['lon'], pid_v3['lat'], 140.0, target_loc)
                    else:
                        v3.apply_control(carla.VehicleControl(brake=1.0))

            # ==========================
            # V4 (Sprinter 乌龟车) 控制逻辑
            # ==========================
            if v4 and v4.is_alive:
                if check_and_handle_out_of_bounds(v4, carla_map):
                    v4 = None
                elif idx_v4 < len(V4_TRAJ):
                    tx, ty, tyaw = V4_TRAJ[idx_v4]
                    target_loc = carla.Location(x=tx, y=ty, z=v4.get_location().z)
                    if v4.get_location().distance(target_loc) < 2.0 and idx_v4 < len(V4_TRAJ) - 1:
                        idx_v4 += 1
                    apply_pid_control(v4, pid_v4['lon'], pid_v4['lat'], 5.0, target_loc)
                else:
                    v4.apply_control(carla.VehicleControl(brake=1.0))

            # ==========================
            # 行人：跑步穿过街道
            # ==========================
            if ped and ped.is_alive:
                if idx_ped < len(PED_TRAJ):
                    tx, ty, tyaw = PED_TRAJ[idx_ped]
                    ped_loc = ped.get_location()
                    target_vec = np.array([tx - ped_loc.x, ty - ped_loc.y])
                    dist = np.linalg.norm(target_vec)
                    if dist < 1.0 and idx_ped < len(PED_TRAJ) - 1: idx_ped += 1
                    if dist > 0.1:
                        direction = carla.Vector3D(target_vec[0] / dist, target_vec[1] / dist, 0.0)
                        ped.apply_control(carla.WalkerControl(direction=direction, speed=4.0, jump=False))
                else:
                    ped.apply_control(carla.WalkerControl(direction=carla.Vector3D(0, 0, 0), speed=0.0))

            # ==========================
            # Debug: 每隔 1 秒打印一次全场车辆实时速度
            # ==========================
            if tick_counter % 20 == 0:  # dt=0.05, 20 ticks = 1s
                speed_str = f"[{sim_time:04.1f}s] 实速监视 -> "
                if ego: speed_str += f"Ego:{get_speed_kmh(ego):.0f} | "
                if v2: speed_str += f"V2:{get_speed_kmh(v2):.0f} | "
                if v3: speed_str += f"MOTO(V3):{get_speed_kmh(v3):.0f} | "
                if v4: speed_str += f"V4:{get_speed_kmh(v4):.0f}"
                print(speed_str)

            # 帧率同步控制
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n检测到键盘中断，准备退出...")
    finally:
        print("\n--- 清理环境并恢复异步设置 ---")
        for actor in actor_list:
            if actor is not None and actor.is_alive:
                actor.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("清理完毕，退出程序。")


if __name__ == '__main__':
    main()
