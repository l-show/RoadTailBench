# -*- coding: utf-8 -*-

import carla
import time
import math
import numpy as np
# 尝试导入 GlobalRoutePlanner
try:
    # 这里的路径通常在 carla 安装包里，如果报错提示找不到 agents，
    # 你可能需要手动 sys.path.append('你的carla/PythonAPI/carla路径')
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except ImportError:
    print("警告: 无法导入 GlobalRoutePlanner，请确保 'agents' 模块在 Python 路径中。")
    print("通常位于: .../CARLA_0.9.15/PythonAPI/carla/agents/navigation/")

# ==========================================
# 轨迹数据清洗 (自动去重)
# ==========================================
def clean_path_points(raw_points):
    cleaned_points = []
    if raw_points:
        cleaned_points.append(raw_points[0])
        for i in range(1, len(raw_points)):
            # 简单的去重逻辑，防止重叠点导致PID计算异常
            if raw_points[i] != raw_points[i - 1]:
                cleaned_points.append(raw_points[i])
    return cleaned_points


# ==========================================
# 目标轨迹数据 (Location_x, Location_y, Rotation_yaw)
# ==========================================
RAW_PATH_POINTS = [
    (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.678),
    (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.678),
    (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.224), (-11.632, -33.337, -66.315),
    (-11.505, -33.624, -65.696), (-11.293, -34.094, -65.697), (-11.081, -34.562, -65.627),
    (-10.867, -35.029, -65.277), (-10.649, -35.504, -65.277), (-10.438, -35.956, -64.786),
    (-10.221, -36.416, -64.786), (-10.0, -36.885, -64.786), (-9.788, -37.33, -63.726),
    (-9.545, -37.793, -61.744), (-9.312, -38.226, -61.744), (-9.066, -38.682, -61.741),
    (-8.832, -39.134, -63.374), (-8.599, -39.599, -63.374), (-8.371, -40.054, -63.374),
    (-8.147, -40.501, -63.374), (-7.916, -40.962, -63.374), (-7.679, -41.427, -61.995),
    (-7.438, -41.875, -61.234), (-7.172, -42.315, -55.895), (-6.865, -42.725, -51.569),
    (-6.541, -43.108, -48.166), (-6.203, -43.478, -47.34), (-5.863, -43.847, -46.788),
    (-5.514, -44.206, -44.813), (-5.146, -44.564, -44.123), (-4.775, -44.922, -44.123),
    (-4.415, -45.272, -44.532), (-4.062, -45.626, -45.431), (-3.718, -45.986, -46.957),
    (-3.366, -46.364, -47.224), (-3.015, -46.745, -47.706), (-2.687, -47.119, -49.057),
    (-2.381, -47.513, -53.703), (-2.079, -47.925, -53.843), (-1.772, -48.333, -52.466),
    (-1.462, -48.736, -52.466), (-1.149, -49.136, -51.24), (-0.829, -49.528, -49.54),
    (-0.494, -49.918, -49.264), (-0.166, -50.299, -49.264), (0.171, -50.683, -48.032),
    (0.524, -51.068, -47.351), (0.871, -51.435, -45.648), (1.237, -51.801, -44.951),
    (1.603, -52.166, -44.814), (1.963, -52.524, -44.814), (2.329, -52.888, -44.814),
    (2.694, -53.251, -44.814), (3.064, -53.613, -44.4), (3.432, -53.974, -44.4),
    (3.794, -54.328, -44.4), (4.147, -54.674, -44.4), (4.515, -55.034, -44.4),
    (4.883, -55.395, -44.4), (5.247, -55.751, -44.4), (5.607, -56.102, -44.128),
    (5.974, -56.455, -43.717), (6.338, -56.797, -43.059), (6.713, -57.146, -43.059),
    (7.083, -57.492, -42.447), (7.48, -57.83, -39.352), (7.873, -58.152, -39.352),
    (8.26, -58.469, -39.352), (8.647, -58.785, -37.599), (9.064, -59.078, -32.134),
    (9.491, -59.339, -31.03), (9.927, -59.601, -30.546), (10.37, -59.859, -29.717),
    (10.823, -60.114, -29.302), (11.271, -60.367, -30.148), (11.712, -60.633, -31.952),
    (12.134, -60.896, -31.952), (12.572, -61.169, -31.952), (13.001, -61.438, -32.232),
    (13.438, -61.714, -32.232), (13.875, -61.989, -32.232), (14.31, -62.263, -32.232),
    (14.746, -62.538, -32.232), (15.181, -62.812, -32.232), (15.605, -63.08, -32.232),
    (16.03, -63.348, -32.232), (16.463, -63.621, -32.232), (16.902, -63.898, -32.232),
    (17.334, -64.17, -32.232), (17.768, -64.444, -32.232), (18.2, -64.716, -32.232),
    (18.626, -64.985, -32.232), (19.052, -65.269, -34.781), (19.464, -65.554, -34.781),
    (19.884, -65.846, -34.781), (20.312, -66.139, -34.095), (20.728, -66.416, -32.506),
    (21.165, -66.687, -30.232), (21.618, -66.937, -27.513), (22.077, -67.174, -27.234),
    (22.528, -67.407, -27.722), (22.977, -67.644, -27.862), (23.414, -67.875, -27.862),
    (23.871, -68.116, -27.862), (24.328, -68.358, -27.862), (24.767, -68.589, -26.297),
    (25.238, -68.811, -23.905), (25.701, -69.013, -23.349), (26.173, -69.216, -23.349),
    (26.646, -69.421, -23.349), (27.115, -69.621, -22.86), (27.593, -69.822, -22.72),
    (28.052, -70.015, -23.137), (28.519, -70.214, -23.137), (28.99, -70.416, -23.137),
    (29.463, -70.618, -23.137), (29.923, -70.815, -23.137), (30.395, -71.016, -23.137),
    (30.866, -71.217, -23.137), (31.34, -71.42, -22.792), (31.805, -71.61, -21.598),
    (32.278, -71.791, -20.489), (32.748, -71.966, -20.419), (33.219, -72.145, -21.536),
    (33.697, -72.337, -22.228), (34.176, -72.533, -22.228), (34.635, -72.72, -22.228),
    (35.101, -72.911, -22.228), (35.579, -73.106, -22.228), (36.056, -73.276, -16.659),
    (36.528, -73.415, -15.573), (37.028, -73.554, -15.573), (37.527, -73.693, -15.573),
    (38.02, -73.83, -15.573), (38.509, -73.968, -16.335), (38.986, -74.125, -19.439),
    (39.47, -74.301, -21.446), (39.953, -74.491, -22.067), (40.416, -74.697, -24.31),
    (40.876, -74.91, -25.714), (41.336, -75.138, -26.363), (41.805, -75.349, -22.485),
    (42.269, -75.535, -18.419), (42.752, -75.685, -17.273), (43.249, -75.84, -17.273),
    (43.727, -75.986, -15.713), (44.226, -76.117, -12.549), (44.729, -76.22, -9.507),
    (45.221, -76.3, -8.454), (45.728, -76.373, -7.765), (46.224, -76.441, -7.765),
    (46.727, -76.509, -7.765), (47.24, -76.576, -6.603), (47.743, -76.634, -6.603),
    (48.236, -76.691, -6.477), (48.736, -76.747, -6.477), (49.239, -76.804, -6.477),
    (49.731, -76.86, -6.477), (50.247, -76.919, -6.477), (50.742, -76.976, -7.104),
    (51.255, -77.057, -13.106), (51.736, -77.184, -15.056), (52.236, -77.31, -13.475),
    (52.741, -77.422, -11.649), (53.238, -77.52, -11.161), (53.743, -77.622, -12.876),
    (54.243, -77.753, -15.305), (54.721, -77.884, -15.376), (55.221, -78.021, -15.25),
    (55.703, -78.152, -15.25), (56.181, -78.293, -19.389), (56.664, -78.463, -19.389),
    (57.146, -78.637, -19.897), (57.616, -78.807, -19.897), (58.089, -78.978, -19.897),
    (58.53, -79.235, -39.786), (58.918, -79.573, -41.137), (59.297, -79.904, -41.137),
    (59.681, -80.24, -41.137), (60.071, -80.581, -41.137), (60.456, -80.917, -41.137),
    (60.834, -81.247, -41.137), (61.206, -81.605, -46.419), (61.548, -81.964, -46.291),
    (61.905, -82.338, -46.291), (62.262, -82.711, -46.291), (62.613, -83.079, -46.291),
    (62.958, -83.44, -46.291), (63.309, -83.808, -46.291), (63.66, -84.175, -46.291),
    (64.007, -84.539, -46.421), (64.357, -84.911, -47.005), (64.704, -85.294, -48.311),
    (65.036, -85.668, -48.311), (65.369, -86.041, -48.311), (65.71, -86.428, -48.694),
    (66.045, -86.811, -48.95), (66.378, -87.194, -48.95), (66.703, -87.596, -54.247),
    (67.006, -88.014, -53.862), (67.306, -88.424, -53.862), (67.606, -88.835, -53.862),
    (67.91, -89.252, -53.862), (68.205, -89.656, -54.13), (68.496, -90.072, -55.754),
    (68.757, -90.499, -59.032), (69.019, -90.935, -59.032), (69.276, -91.364, -59.032),
    (69.542, -91.807, -59.032), (69.798, -92.236, -59.288), (70.05, -92.668, -59.93),
    (70.303, -93.118, -61.243), (70.542, -93.557, -61.5), (70.788, -94.01, -61.5),
    (71.031, -94.456, -61.5), (71.268, -94.914, -64.726), (71.484, -95.375, -64.984),
    (71.695, -95.828, -65.113), (71.912, -96.297, -65.499), (72.126, -96.767, -65.499),
    (72.327, -97.225, -66.756), (72.531, -97.699, -66.756), (72.732, -98.166, -66.756),
    (72.936, -98.641, -66.756), (73.14, -99.116, -66.756), (73.344, -99.59, -66.756),
    (73.548, -100.065, -66.756), (73.752, -100.539, -66.756), (73.945, -101.009, -68.173),
    (74.135, -101.49, -68.688), (74.317, -101.955, -68.688), (74.504, -102.437, -69.774),
    (74.664, -102.919, -71.9), (74.819, -103.395, -71.9), (74.98, -103.886, -71.9),
    (75.14, -104.377, -71.9), (75.298, -104.861, -71.9), (75.459, -105.352, -71.9),
    (75.613, -105.828, -72.158), (75.766, -106.304, -72.158), (75.917, -106.789, -73.388),
    (76.062, -107.275, -73.447), (76.201, -107.755, -74.221), (76.338, -108.253, -74.865),
    (76.473, -108.752, -74.865), (76.605, -109.242, -74.865), (76.699, -109.587, -74.865),
    (76.699, -109.587, -74.865), (76.699, -109.587, -74.865), (76.699, -109.587, -74.865),
    (76.699, -109.587, -74.865), (76.699, -109.587, -74.865), (76.699, -109.587, -74.865),
    (76.699, -109.587, -74.865)
]
VEHICLE_PATH_POINTS = clean_path_points(RAW_PATH_POINTS)


# ==========================================
# PID 控制器类 (保持原逻辑)
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController2:
    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint, vehicle_transform):
        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint[0] - v_begin.x, waypoint[1] - v_begin.y, 0.0])
        norm_w = np.linalg.norm(w_vec)
        if norm_w < 0.1:
            return 0.0
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0
        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


# ==========================================
# 辅助函数
# ==========================================
def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )


def get_target_waypoint(actor_loc, path_points, lookahead_dist=4.0):
    min_dist = float('inf')
    closest_index = 0
    # 找到最近的点
    for i, p in enumerate(path_points):
        dist = math.sqrt((p[0] - actor_loc.x) ** 2 + (p[1] - actor_loc.y) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_index = i

    # 向前寻找 lookahead_dist 距离的点
    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1 = path_points[i]
        p2 = path_points[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist:
            break
    return path_points[target_index]


# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 引入路径规划器 (放在函数内引用防止没安装报错时还能跑前面)
    try:
        from agents.navigation.global_route_planner import GlobalRoutePlanner
    except ImportError:
        print("Error: 缺少 agents 模块，请检查 Carla PythonAPI 环境配置")
        return

    # ---------------------------------------------------------
    # 设置 Traffic Manager (TM)
    # ---------------------------------------------------------
    tm_port = 8000
    tm = client.get_trafficmanager(tm_port)
    # 重要：为了和世界同步模式配合，TM 也要设为同步
    tm.set_synchronous_mode(True)
    # 设置全局混合物理模式 (可选，减少计算量，半径内物理全开)
    # tm.set_hybrid_physics_mode(True)

    # ---------------------------------------------------------
    # 设置天气参数
    # ---------------------------------------------------------
    weather = carla.WeatherParameters(
        cloudiness=25.0, precipitation=40.0, precipitation_deposits=70.0,
        wind_intensity=10.0, sun_azimuth_angle=115.0, sun_altitude_angle=14.0,
        fog_density=2.0, fog_distance=0.0, fog_falloff=0.0, wetness=40.0,
        scattering_intensity=5.0, mie_scattering_scale=0.0, rayleigh_scattering_scale=0.3,
        dust_storm=0.0
    )
    world.set_weather(weather)
    print("天气参数已更新。")

    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        # ---------------------------------------------------------
        # 1. 生成 原始车辆 (vehicle.volkswagen.t2_2021)
        # ---------------------------------------------------------
        vehicle_bp_name = 'vehicle.volkswagen.t2_2021'
        bp_vehicle = bp_lib.find(vehicle_bp_name)
        initial_point = VEHICLE_PATH_POINTS[0]
        trans_vehicle = get_transform(x=initial_point[0], y=initial_point[1], z=0.5,
                                      yaw=initial_point[2])
        vehicle = world.try_spawn_actor(bp_vehicle, trans_vehicle)
        if vehicle:
            actor_list.append(vehicle)
            vehicle.set_simulate_physics(True)
            print(f"{vehicle_bp_name} 生成成功")

            # 初始化 VW 控制器
            lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0, dt=settings.fixed_delta_seconds)
            lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2, dt=settings.fixed_delta_seconds)

            # 开启车灯
            base_light_state = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
            vehicle.set_light_state(carla.VehicleLightState(base_light_state))
        else:
            print(f"无法生成车辆 {vehicle_bp_name}")

        # ---------------------------------------------------------
        # 2. 生成 Ego 车辆 (vehicle.chevrolet.impala)
        # ---------------------------------------------------------
        ego_bp = bp_lib.find('vehicle.chevrolet.impala')
        ego_bp.set_attribute('role_name', 'hero')

        # 初始位置 x=-23.700, y=47.344, z=0.5, yaw=-90.0 (向下行驶)
        ego_spawn_trans = get_transform(x=-23.700, y=47.344, z=0.5, yaw=-90.0)

        ego_vehicle = world.try_spawn_actor(ego_bp, ego_spawn_trans)

        if ego_vehicle:
            actor_list.append(ego_vehicle)
            print("Ego Vehicle 生成成功，正在等待物理落地...")

            # 【关键修改 1】: 刚生成时不要给速度，也不要开自动驾驶
            # 此时车是悬空的 (z=0.5)，让它先掉下来并在原地停稳
        else:
            print("无法生成 Ego Vehicle，请检查位置是否冲突。")

        # ==========================================
        # 等待物理系统稳定 (原地落地)
        # ==========================================
        # 这 10-20 帧用于让车从 z=0.5 掉到地面并停稳悬挂
        for _ in range(20):
            world.tick()
            # 在这里不需要 sleep，因为我们在同步模式下只关心物理计算步数
            # 如果为了看清落地过程，可以加一点 sleep，但实际运行不需要

        print("物理系统已稳定，开始施加初始速度并接管...")

        # ==========================================
        # 【核心修复】：物理稳定后，统一强制开启车灯
        # ==========================================
        # 定义灯光状态：位置灯 + 远光灯 (HighBeam 亮度更高)
        target_light_state = carla.VehicleLightState(
            carla.VehicleLightState.Position | carla.VehicleLightState.HighBeam
        )

        # 1. 设置 VW T2 车灯
        if vehicle and vehicle.is_alive:
            vehicle.set_light_state(target_light_state)
            print("VW T2 车灯已开启 (Position + HighBeam)")

        # 2. 设置 Ego Impala 车灯
        if ego_vehicle and ego_vehicle.is_alive:
            ego_vehicle.set_light_state(target_light_state)
            print("Ego Impala 车灯已开启 (Position + HighBeam)")

            # (可选) 告诉 TM 不要接管这辆车的灯光，防止 TM 觉得白天不需要开灯而自动关掉
            # 注意：如果 TM 自动驾驶接管了灯光，它可能会覆盖你的设置。
            # 下面这行指令在 0.9.15 中可以禁止 TM 更改灯光状态：
            tm.update_vehicle_lights(ego_vehicle, False)

        # ==========================================
        # 【关键修改 2】: 落地后再设置速度和自动驾驶
        # ==========================================
        # 前提：此时代码刚结束了 20 帧的物理稳定等待期 (world.tick)，车辆已从悬空状态落地并稳定悬挂
        if ego_vehicle:
            # -----------------------------------------------------
            # 1. 物理层面的速度初始化
            # -----------------------------------------------------
            # 设定目标初始速度为 110 km/h
            initial_speed_kmh = 110.0
            # Carla 物理引擎使用国际单位制 (m/s)，需要转换：110 / 3.6 ≈ 30.56 m/s
            initial_speed_mps = initial_speed_kmh / 3.6

            # 将角度转换为弧度。Carla 中 yaw=-90 通常指向地图坐标系的负 Y 轴方向
            yaw_rad = math.radians(-90.0)

            # 根据三角函数分解速度向量
            # vx = 速度 * cos(角度)，vy = 速度 * sin(角度)
            vx = initial_speed_mps * math.cos(yaw_rad)
            vy = initial_speed_mps * math.sin(yaw_rad)

            # 【物理注入】：直接修改车辆刚体的线性速度
            # z=0.0 表示不给垂直方向的速度，让车辆紧贴地面平滑滑行
            # 这行代码让车辆在这一帧瞬间获得 110km/h 的动能
            ego_vehicle.set_target_velocity(carla.Vector3D(x=vx, y=vy, z=0.0))

            # -----------------------------------------------------
            # 2. 行为决策层 (Traffic Manager) 初始化
            # -----------------------------------------------------
            # 开启自动驾驶模式。
            # 参数 True：开启；参数 tm_port：指定由哪个端口的 Traffic Manager 来接管
            # 如果不指定端口，默认会使用 8000
            ego_vehicle.set_autopilot(True, tm_port)

            # -----------------------------------------------------
            # 3. 设置 TM 的具体驾驶风格 (针对 ego_vehicle 个体)
            # -----------------------------------------------------
            # CarlaTM的底线：Traffic Manager的算法写死了一条规则——只要开启避障，必须保证物理上能刹停
            # TM内部有一个强制的纵向安全模型，它的刹车逻辑是基于碰撞时间(TTC)和物理刹车距离计算的
            # 【闯红灯设置】：100% 的概率忽略红绿灯
            # 效果：遇到红灯完全不减速，直接通过
            tm.ignore_lights_percentage(ego_vehicle, 100)

            #  规则破坏：无视停车牌
            tm.ignore_signs_percentage(ego_vehicle, 100)

            # 【碰撞忽略设置】：100% 的概率忽略前车 跟车也会失效
            # 效果：TM 的纵向控制逻辑将不再考虑“避免碰撞”。
            # 即便前方有障碍物，TM 也不会为了避让而踩刹车（除非物理碰撞发生）
            # *注意：这是导致车不减速直接撞上去的关键参数*
            tm.ignore_vehicles_percentage(ego_vehicle, 100)

            # 避障关闭：无视行人 (直接撞)
            tm.ignore_walkers_percentage(ego_vehicle, 100)

            # 【超速设置】：设置速度与路段限速的百分比差值
            # 公式：目标速度 = 当前路段限速 * (1 - percentage/100)
            # 填 -20.0 意味着：目标速度 = 限速 * 1.2 (即超速 20%)
            # 如果想让它跑得飞快，可以设为 -50% 甚至更低，或者设为 -200%
            tm.vehicle_percentage_speed_difference(ego_vehicle, -20.0)

            # 【变道设置】：False 表示禁止自动变道
            # 效果：车辆将死死地保持在当前车道中心，除非车道结束，否则不会为了超车而变道
            tm.auto_lane_change(ego_vehicle, False)

            # 【跟车距离设置】：设置与前车的最小保持距离 (单位：米)
            # 这里你用的是 set_global...，这意味着这个设置会影响 TM 管理的 *所有* 车辆
            # 设为 0.5 米意味着只有贴到 0.5 米才会尝试刹车（当然，配合上面的 ignore_vehicles 100%，这行其实失效了）
            tm.set_global_distance_to_leading_vehicle(0.5)
            # # 建议使用针对单个车的 API
            # tm.distance_to_leading_vehicle(ego_vehicle, 0.5)

        # ==========================================
        # 【新增逻辑】: 强制 Ego 车辆进入匝道
        # 【核心修改】：计算并锁定匝道路径
        # ==========================================
            print("正在计算强制进入匝道的路径...")

            # 1. 获取地图和路由规划器
            amap = world.get_map()
            # 采样分辨率 1.0 米，意味着生成的路径点每隔 1 米一个，非常顺滑
            grp = GlobalRoutePlanner(amap, sampling_resolution=1.0)

            # 2. 设定起点和终点
            # 起点：Ego 车当前位置
            start_loc = ego_vehicle.get_location()

            # 终点：你提供的匝道最深处的坐标 (x=80.721, y=-131.473)
            # 规划器会自动计算如何从高速并入这里
            end_loc = carla.Location(x=80.721, y=-131.473, z=0.0)

            # 3. 计算路径
            # trace_route 返回的是列表 [(waypoint, road_option), ...]
            route = grp.trace_route(start_loc, end_loc)

            # 4. 提取 Location 列表传给 Traffic Manager
            # TM 的 set_path 需要的是 carla.Location 的列表
            path_locations = []
            for snapshot in route:
                # snapshot[0] 是 waypoint
                path_locations.append(snapshot[0].transform.location)

            # 5. 强制应用路径
            if path_locations:
                # 这一步是关键：告诉 TM "别随机跑了，严格按这个坐标列表跑"
                tm.set_path(ego_vehicle, path_locations)
                print(f"路径设置成功！已锁定匝道路线，全长 {len(path_locations)} 个路径点。")
            else:
                print("错误：无法计算到匝道的路径，请检查终点坐标是否在路面上。")

        print("场景运行中...")
        target_speed_vw_kmh = 25.0

        # 控制开关：是否使用外部模型控制 Ego 车辆
        # 如果为 True，则关闭 Autopilot 并应用下方计算的 Control
        # 如果为 False，则继续使用 Traffic Manager 的车道保持
        ENABLE_EXTERNAL_CONTROL = False

        # 主循环
        while True:
            start_time = time.time()
            world.tick()

            # -------------------------------------------------
            # A. 控制 VW T2 (PID 轨迹跟随)
            # -------------------------------------------------
            if vehicle and vehicle.is_alive:
                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                current_speed_kmh = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
                target_wp = get_target_waypoint(tf.location, VEHICLE_PATH_POINTS, lookahead_dist=5.0)

                throttle_output = lon_controller.run_step(target_speed_vw_kmh, current_speed_kmh)
                steer_output = lat_controller.run_step(target_wp, tf)

                control = carla.VehicleControl()
                control.steer = steer_output
                if throttle_output >= 0.0:
                    control.throttle = throttle_output
                    control.brake = 0.0
                else:
                    control.throttle = 0.0
                    control.brake = abs(throttle_output)
                vehicle.apply_control(control)

            # -------------------------------------------------
            # B. 控制 Ego Impala (预留接口)
            # -------------------------------------------------
            if ego_vehicle and ego_vehicle.is_alive:
                # 可以在这里获取传感器数据、图像等传给 UniAD
                # ego_transform = ego_vehicle.get_transform()
                # ego_velocity = ego_vehicle.get_velocity()

                # ==================================================
                # TODO: UniAD / External Model Interface
                # ==================================================
                if ENABLE_EXTERNAL_CONTROL:
                    # 1. 确保关闭 TM 自动驾驶
                    # (如果在循环外已经关闭，这里可以省略判断，但为了安全建议检查)
                    # ego_vehicle.set_autopilot(False, tm_port)

                    # 2. 接收模型输出的控制量
                    # 假设模型输出为: model_steer, model_throttle, model_brake
                    model_steer = 0.0  # [-1, 1]
                    model_throttle = 0.5  # [0, 1]
                    model_brake = 0.0  # [0, 1]

                    # 3. 应用控制
                    ego_control = carla.VehicleControl()
                    ego_control.steer = model_steer
                    ego_control.throttle = model_throttle
                    ego_control.brake = model_brake
                    ego_control.manual_gear_shift = False
                    ego_vehicle.apply_control(ego_control)

                else:
                    # 保持 Traffic Manager 控制 (车道保持 + 高速)
                    # 如果需要强制设置每一帧的速度而不受物理引擎阻力影响(不推荐，物理不真实)，
                    # 可以用 ego_vehicle.set_target_velocity()，但通常 autopilo 足够。
                    pass

            # ==============================
            # 同步时间控制
            # ==============================
            compute_time = time.time() - start_time
            if compute_time < settings.fixed_delta_seconds:
                time.sleep(settings.fixed_delta_seconds - compute_time)

    except Exception as e:
        print(f"发生异常: {e}")
    except KeyboardInterrupt:
        print("\n用户停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        # 恢复异步模式
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        # 恢复 TM 模式 (虽然 destroy 后通常不需要，但为了保险)
        if 'tm' in locals():
            tm.set_synchronous_mode(False)

        # 清理车辆
        if actor_list:
            client.apply_batch([carla.command.DestroyActor(a) for a in actor_list])
        print("清理完成。")


if __name__ == '__main__':
    main()