import carla
import time
import math
import numpy as np

# ==========================================
# 轨迹数据清洗 (自动去重) - 只保留 (x, y, yaw)
# ==========================================

# 现有的大货车轨迹
RAW_TRUCK_PATH_POINTS_WITH_YAW = [
    (-84.753, -529.298, -142.046), (-87.759, -531.643, -142.046),
    (-90.815, -534.025, -142.186), (-93.881, -536.396, -142.536),
    (-96.900, -538.723, -142.399), (-99.874, -541.002, -142.539),
    (-102.892, -543.325, -142.189), (-105.851, -545.624, -142.120),
    (-108.816, -547.920, -142.330), (-111.785, -550.211, -142.680),
    (-114.837, -552.495, -143.803), (-117.885, -554.679, -144.513),
    (-120.989, -556.780, -146.173), (-124.102, -558.869, -146.103),
    (-127.318, -561.028, -146.388), (-130.543, -563.057, -148.475),
    (-133.866, -565.053, -149.753), (-137.139, -566.882, -152.044),
    (-140.472, -568.599, -153.042), (-143.949, -570.310, -154.220),
    (-147.461, -571.947, -155.292), (-150.896, -573.454, -157.150),
    (-154.438, -574.934, -157.430), (-157.964, -576.372, -159.078),
    (-161.538, -577.689, -159.931), (-165.052, -579.005, -159.372),
    (-168.571, -580.330, -159.372), (-172.197, -581.694, -159.442),
    (-175.771, -583.018, -160.432), (-179.430, -584.293, -160.855),
    (-183.092, -585.555, -160.995), (-186.647, -586.742, -163.488),
    (-188.446, -587.268, -165.442), (-191.971, -588.183, -165.442),
    (-195.620, -589.037, -168.765), (-199.363, -589.762, -169.115),
    (-203.069, -590.321, -173.504), (-205.184, -590.526, -174.799),
    (-205.184, -590.526, -174.799), (-205.868, -590.588, -174.656),
    (-208.674, -590.768, -179.392), (-212.424, -590.786, 178.255),
    (-216.287, -590.505, 173.735), (-220.072, -590.044, 173.008),
    (-223.853, -589.562, 170.766), (-227.452, -588.539, 157.610),
    (-230.817, -586.890, 151.595), (-234.167, -584.944, 147.989),
    (-237.346, -582.956, 145.519), (-241.926, -578.880, 144.742),
    (-247.355, -575.050, 146.988), (-247.120, -574.609, 156.047),
    (-248.456, -572.745, 166.850), (-250.397, -572.291, 167.915),
    (-250.722, -575.313, 157.399), (-254.212, -572.947, 164.108),
    (-257.880, -572.875, 169.545), (-261.586, -572.610, 173.274),
    (-265.377, -572.212, 174.620), (-269.240, -571.865, 175.115),
    (-272.984, -571.546, 175.115), (-276.843, -571.216, 175.115),
    (-280.578, -570.897, 175.115), (-284.374, -570.571, 175.045),
    (-288.108, -570.247, 175.045), (-291.862, -569.922, 175.045),
    (-295.596, -569.599, 175.045), (-299.429, -568.951, 175.045),
    (-303.146, -568.441, 175.045), (-306.900, -568.116, 175.045),
    (-310.760, -567.792, 175.397), (-314.496, -567.503, 175.823),
    (-318.359, -567.222, 175.823), (-322.161, -566.952, 176.105),
    (-325.900, -566.696, 176.105), (-327.521, -566.585, 176.105)
]

# 现有的小轿车轨迹
RAW_CAR_PATH_POINTS_WITH_YAW = [
(-15.056, -471.325, -143.757),(-15.056, -471.325, -143.757),
(-17.279, -472.979, -142.719),(-24.146, -478.402, -141.821),(-31.042, -483.790, -142.308),
(-37.992, -489.101, -142.654),(-44.896, -494.472, -141.616),
(-51.794, -499.856, -142.171),(-58.707, -505.220, -142.518),
(-64.988, -509.971, -143.492),(-67.983, -512.050, -145.642),
(-68.344, -512.297, -145.642),(-68.344, -512.297, -145.642),
(-68.344, -512.297, -145.642),(-68.344, -512.297, -145.642),(-68.344, -512.297, -145.642),
    (-73.324, -515.593, -145.037), (-76.721, -518.040, -143.319),
    (-80.521, -521.435, -145.829), (-84.531, -524.586, -145.146),
    (-88.571, -527.774, -146.701), (-92.706, -530.731, -145.399),
    (-96.812, -533.775, -144.231), (-100.833, -536.780, -142.104),
    (-104.822, -539.861, -143.137), (-108.833, -543.016, -141.158),
    (-112.686, -546.333, -143.742), (-116.829, -549.241, -145.248),
    (-120.928, -552.177, -143.673), (-125.109, -555.141, -146.006),
    (-129.406, -557.853, -149.284), (-133.847, -560.324, -150.172),
    (-138.262, -562.845, -150.376), (-142.781, -565.254, -157.835),
    (-147.659, -566.733, -165.254), (-152.615, -568.006, -165.595),
    (-157.551, -569.299, -164.907), (-162.388, -570.733, -160.451),
    (-167.236, -572.503, -159.832), (-172.022, -574.263, -159.762),
    (-176.754, -576.075, -159.074), (-181.535, -577.795, -161.322),
    (-186.347, -579.297, -163.041), (-191.262, -580.707, -165.425),
    (-196.209, -581.914, -166.380), (-201.154, -582.985, -168.125),
    (-206.173, -583.934, -170.380), (-211.211, -584.570, -174.469),
    (-216.292, -585.008, -177.069), (-221.376, -585.226, -179.695),
    (-226.433, -585.154, 175.927), (-231.458, -584.291, 158.169),
    (-236.017, -581.926, 150.665), (-240.426, -579.485, 151.008),
    (-244.840, -577.053, 151.904), (-249.325, -574.848, 156.633),
    (-254.051, -572.001, 164.843), (-259.022, -571.801, 172.302),
    (-264.038, -571.759, 176.373), (-269.134, -571.337, 174.485),
    (-274.217, -570.917, 175.718), (-279.283, -570.553, 176.065),
    (-284.396, -570.192, 175.084), (-289.443, -569.658, 174.608),
    (-294.497, -569.193, 174.951), (-299.555, -568.793, 175.643),
    (-304.684, -568.436, 176.126), (-309.814, -568.084, 175.237),
    (-314.851, -567.513, 174.556), (-319.929, -567.021, 175.593),
    (-325.061, -566.645, 176.625), (-330.155, -566.341, 176.210),
    (-335.206, -565.484, 161.248), (-340.797, -563.012, 154.169),
]

# 新增的 Mini Cooper 轨迹
RAW_MINI_PATH_POINTS_WITH_YAW = [
    (-109.333, -547.938, -140.085), (-109.333, -547.938, -139.545), (-109.333, -547.938, -138.322),
    (-109.333, -547.938, -138.871), (-110.139, -548.654, -138.394),
    (-112.027, -550.279, -140.138), (-116.065, -553.49, -143.921), (-120.173, -556.338, -146.179),
    (-124.368, -559.057, -148.402), (-128.818, -561.682, -149.384),
    (-133.232, -564.367, -148.371), (-137.49, -566.99, -148.371), (-140.445, -569.012, -143.139),
    (-140.445, -569.012, -143.139), (-140.445, -569.012, -143.139),
    (-140.445, -569.012, -143.139), (-140.717, -569.232, -137.226), (-141.08, -569.588, -133.781),
    (-141.435, -569.962, -132.967), (-141.785, -570.343, -132.423),
    (-141.926, -570.496, -132.423), (-141.926, -570.496, -132.423), (-141.926, -570.496, -132.423),
    (-141.926, -570.496, -132.423), (-141.926, -570.496, -132.423),
    (-141.926, -570.496, -132.423), (-141.926, -570.496, -132.423), (-141.926, -570.496, -132.423),
    (-141.926, -570.496, -132.423), (-141.926, -570.496, -132.015),
    (-142.147, -570.767, -130.625), (-142.462, -571.154, -127.494), (-142.771, -571.557, -127.494),
    (-143.08, -571.96, -127.494), (-143.39, -572.362, -127.494),
    (-143.704, -572.772, -127.494), (-144.028, -573.173, -131.893), (-144.38, -573.54, -133.991),
    (-144.734, -573.904, -134.878), (-145.08, -574.241, -136.641),
    (-145.463, -574.598, -137.191), (-145.848, -574.944, -138.344), (-146.223, -575.274, -138.89),
    (-146.612, -575.613, -138.89), (-146.989, -575.941, -138.89),
    (-147.365, -576.271, -138.41), (-147.738, -576.604, -138.41), (-148.126, -576.946, -139.231),
    (-148.512, -577.277, -139.369), (-148.899, -577.606, -139.914),
    (-149.282, -577.927, -139.914), (-149.671, -578.255, -139.776), (-150.065, -578.589, -139.776),
    (-150.46, -578.923, -139.776), (-150.855, -579.258, -139.776),
    (-151.243, -579.586, -139.776), (-151.638, -579.921, -139.776), (-152.02, -580.244, -139.776),
    (-152.415, -580.578, -139.776), (-152.803, -580.907, -139.776),
    (-153.198, -581.241, -139.776), (-153.593, -581.575, -139.914), (-153.976, -581.896, -140.052),
    (-154.372, -582.228, -140.052), (-154.755, -582.549, -140.052),
    (-155.151, -582.881, -140.052), (-155.541, -583.208, -139.777), (-155.922, -583.531, -139.573),
    (-156.301, -583.856, -139.227), (-156.693, -584.193, -139.227),
    (-157.084, -584.53, -139.227), (-157.469, -584.861, -139.227), (-157.86, -585.198, -139.227),
    (-158.253, -585.533, -139.569), (-158.647, -585.866, -139.983),
    (-159.03, -586.188, -139.983), (-159.421, -586.513, -140.6), (-159.824, -586.836, -141.83),
    (-160.234, -587.151, -143.387), (-160.64, -587.442, -145.306),
    (-161.058, -587.717, -147.893), (-161.482, -587.981, -148.235), (-161.921, -588.253, -147.959),
    (-162.339, -588.526, -145.651), (-162.76, -588.826, -144.159),
    (-163.175, -589.135, -141.783), (-163.565, -589.462, -138.656), (-163.946, -589.811, -136.135),
    (-164.307, -590.181, -133.123), (-164.649, -590.546, -133.123),
    (-164.991, -590.912, -133.123), (-165.339, -591.284, -132.847), (-165.687, -591.666, -132.299),
    (-166.016, -592.042, -130.518), (-166.341, -592.421, -130.518),
    (-166.677, -592.814, -130.518), (-167.012, -593.206, -130.518), (-167.336, -593.587, -130.176),
    (-167.658, -593.969, -130.176), (-167.991, -594.364, -130.176),
    (-168.324, -594.759, -130.176), (-168.652, -595.147, -130.176), (-168.984, -595.542, -130.038),
    (-169.315, -595.928, -131.344), (-169.651, -596.309, -131.412),
    (-169.986, -596.702, -129.224), (-170.3, -597.092, -128.535), (-170.622, -597.497, -128.535),
    (-170.939, -597.895, -128.535), (-171.257, -598.302, -126.693),
    (-171.545, -598.71, -124.754), (-171.839, -599.134, -124.754), (-172.123, -599.545, -124.07),
    (-172.408, -599.976, -123.047), (-172.686, -600.403, -123.047),
    (-172.961, -600.824, -123.185), (-173.236, -601.242, -123.528), (-173.522, -601.673, -123.528),
    (-173.808, -602.105, -123.528), (-174.092, -602.536, -122.909),
    (-174.358, -602.97, -120.093), (-174.609, -603.411, -119.678), (-174.857, -603.844, -119.678),
    (-175.102, -604.28, -118.993), (-175.343, -604.718, -118.855),
    (-175.585, -605.157, -118.855), (-175.834, -605.61, -118.925), (-176.084, -606.053, -119.678),
    (-176.339, -606.501, -119.678), (-176.586, -606.935, -118.52),
    (-176.824, -607.394, -117.158), (-177.059, -607.854, -116.305), (-177.28, -608.302, -116.305),
    (-177.502, -608.75, -116.305), (-177.73, -609.212, -115.757),
    (-177.943, -609.665, -115.145), (-178.159, -610.125, -115.145), (-178.378, -610.592, -115.145),
    (-178.594, -611.052, -115.145), (-178.809, -611.512, -115.145),
    (-179.028, -611.979, -115.145), (-179.241, -612.432, -115.145), (-179.458, -612.9, -114.159),
    (-179.663, -613.357, -114.159), (-179.871, -613.826, -113.815),
    (-180.079, -614.298, -113.815), (-180.261, -614.709, -113.815), (-180.261, -614.709, -113.815),
    (-180.571, -615.415, -113.472), (-181.058, -616.589, -112.173),
    (-181.508, -617.755, -110.673), (-181.936, -618.974, -107.887), (-182.292, -620.193, -104.35),
    (-182.591, -621.407, -103.123), (-182.882, -622.667, -102.988),
    (-183.164, -623.928, -102.164), (-183.436, -625.147, -102.717), (-183.716, -626.386, -102.717),
    (-184.001, -627.646, -102.717), (-184.267, -628.825, -102.717),
    (-184.561, -630.127, -102.717), (-184.846, -631.386, -102.717), (-184.869, -631.488, -102.717),
    (-185.447, -633.983, -103.607), (-186.328, -637.756, -101.148),
    (-186.963, -641.579, -99.038), (-187.514, -645.415, -96.852), (-187.864, -649.211, -93.238),
    (-187.918, -653.086, -88.39), (-187.486, -656.929, -77.183),
    (-186.441, -660.595, -70.362), (-184.946, -664.029, -63.487), (-183.043, -667.317, -54.805)
]


def clean_trajectory_data(raw_path_points_with_yaw):
    """
    去除连续重复的路径点，仅基于XY坐标进行去重。
    """
    cleaned_points = []
    if raw_path_points_with_yaw:
        cleaned_points.append(raw_path_points_with_yaw[0])
        for i in range(1, len(raw_path_points_with_yaw)):
            if raw_path_points_with_yaw[i][:2] != raw_path_points_with_yaw[i - 1][:2]:
                cleaned_points.append(raw_path_points_with_yaw[i])
    return cleaned_points


TRUCK_PATH_POINTS = clean_trajectory_data(RAW_TRUCK_PATH_POINTS_WITH_YAW)
CAR_PATH_POINTS = clean_trajectory_data(RAW_CAR_PATH_POINTS_WITH_YAW)
MINI_PATH_POINTS = clean_trajectory_data(RAW_MINI_PATH_POINTS_WITH_YAW)


# ==========================================
# PID 控制器类
# ==========================================
class PIDLongitudinalController:
    """ 纵向控制 (油门/刹车) """

    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
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


class PIDLateralController:
    """ 横向控制 (转向) """

    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_waypoint_xy, vehicle_transform):
        v_begin = vehicle_transform.location
        v_vec = np.array([
            math.cos(math.radians(vehicle_transform.rotation.yaw)),
            math.sin(math.radians(vehicle_transform.rotation.yaw)),
            0.0
        ])
        w_vec = np.array([
            target_waypoint_xy[0] - v_begin.x,
            target_waypoint_xy[1] - v_begin.y,
            0.0
        ])

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


def get_target_waypoint(vehicle_loc, path_points, lookahead_dist=4.0):
    min_dist = float('inf')
    closest_index = 0

    for i, p_with_yaw in enumerate(path_points):
        p_x, p_y, _ = p_with_yaw
        dist = math.sqrt((p_x - vehicle_loc.x) ** 2 + (p_y - vehicle_loc.y) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_index = i

    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1_x, p1_y, _ = path_points[i]
        p2_x, p2_y, _ = path_points[i + 1]
        d = math.sqrt((p1_x - p2_x) ** 2 + (p1_y - p2_y) ** 2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist:
            break

    if target_index >= len(path_points):
        target_index = len(path_points) - 1

    return path_points[target_index][:2]


# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 保持天气设置
    weather = carla.WeatherParameters(cloudiness=25.0, precipitation=0.0,
                                      precipitation_deposits=0.0, wind_intensity=25.0,
                                      sun_azimuth_angle=40.0, sun_altitude_angle=20.0,
                                      fog_density=0.0, fog_distance=0.0, fog_falloff=0.0,
                                      wetness=0.0, scattering_intensity=20.0,
                                      mie_scattering_scale=0.10, rayleigh_scattering_scale=0.05, dust_storm=0.0)
    world.set_weather(weather)

    bp_lib = world.get_blueprint_library()
    actor_list = []  # 用于存储所有生成的Actor
    original_settings = world.get_settings()  # 保存原始世界设置

    try:
        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(100.0)

        # --- 1. 生成 大货车 (主控车辆) ---
        bp_truck = bp_lib.find('vehicle.carlamotors.carlacola')
        bp_truck.set_attribute('color', '0,0,255')

        initial_point_truck = TRUCK_PATH_POINTS[0]
        spawn_z_truck = 2
        trans_truck = get_transform(x=initial_point_truck[0], y=initial_point_truck[1], z=spawn_z_truck,
                                    yaw=initial_point_truck[2])

        truck = world.try_spawn_actor(bp_truck, trans_truck)
        if truck:
            actor_list.append(truck)
            truck.set_simulate_physics(True)
            print(
                f"大货车 (carlacola) 生成成功，初始位置: ({initial_point_truck[0]}, {initial_point_truck[1]}, {spawn_z_truck})")
            truck_lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0)
            truck_lat_controller = PIDLateralController(K_P=1.95, K_I=0.05, K_D=0.2)
        else:
            print("错误：无法生成大货车。")

        # --- 2. 生成 小轿车 ---
        bp_car = bp_lib.find('vehicle.tesla.model3')
        bp_car.set_attribute('color', '255,0,0')

        initial_point_car = CAR_PATH_POINTS[0]
        spawn_z_car = 2.0
        trans_car = get_transform(x=initial_point_car[0], y=initial_point_car[1], z=spawn_z_car,
                                  yaw=initial_point_car[2])

        car = world.try_spawn_actor(bp_car, trans_car)
        if car:
            actor_list.append(car)
            car.set_simulate_physics(True)
            print(
                f"小轿车 (tesla.model3) 生成成功，初始位置: ({initial_point_car[0]}, {initial_point_car[1]}, {spawn_z_car})")
            car_lon_controller = PIDLongitudinalController(K_P=0.8, K_I=0.03, K_D=0.1)
            car_lat_controller = PIDLateralController(K_P=2.2, K_I=0.08, K_D=0.3)
            car_target_speed = 100
            car_slowdown_x_threshold = -160
            car_slowdown_speed = 40
        else:
            print("错误：无法生成小轿车。")

        # --- 3. 生成 Mini Cooper (新增车辆) ---
        bp_mini = bp_lib.find('vehicle.mini.cooper_s_2021')
        bp_mini.set_attribute('color', '0,0,255')  # 蓝色

        initial_point_mini = MINI_PATH_POINTS[0]
        spawn_z_mini = 2.0
        trans_mini = get_transform(x=initial_point_mini[0], y=initial_point_mini[1], z=spawn_z_mini,
                                   yaw=initial_point_mini[2])

        mini = world.try_spawn_actor(bp_mini, trans_mini)
        if mini:
            actor_list.append(mini)
            mini.set_simulate_physics(True)
            print(f"Mini Cooper 生成成功，初始位置: ({initial_point_mini[0]}, {initial_point_mini[1]}, {spawn_z_mini})")

            # 为 Mini 初始化PID及速度状态变量
            mini_lon_controller = PIDLongitudinalController(K_P=0.8, K_I=0.03, K_D=0.1)
            mini_lat_controller = PIDLateralController(K_P=2.2, K_I=0.08, K_D=0.3)
            mini_target_speed = 60  # 初始速度 60km/h

            # 使用状态机来管理Mini的速度，避免在路口折返时重复触发：
            # 0: 初始状态 (60km/h)
            # 1: 穿过 Y = -555，已减速 (10km/h)
            # 2: 穿过 Y = -660，已加速 (40km/h)
            mini_speed_state = 0
        else:
            print("错误：无法生成 Mini Cooper。")

        # 等待物理稳定
        print("等待 1 秒，物理稳定中...")
        for _ in range(20):
            world.tick()
            time.sleep(0.05)

        print("\n场景运行中... (三辆车均已采用PID控制行驶)")

        # 主循环：仿真核心逻辑
        while True:
            start_time = time.time()
            world.tick()

            # --- 大货车 PID 控制逻辑 ---
            if truck and truck.is_alive:
                tf_truck = truck.get_transform()
                vel_truck = truck.get_velocity()
                speed_truck = 3.6 * math.sqrt(vel_truck.x ** 2 + vel_truck.y ** 2 + vel_truck.z ** 2)

                target_wp_xy_truck = get_target_waypoint(tf_truck.location, TRUCK_PATH_POINTS, lookahead_dist=10.0)
                throttle_output_truck = truck_lon_controller.run_step(80, speed_truck)  # 目标80km/h
                steer_output_truck = truck_lat_controller.run_step(target_wp_xy_truck, tf_truck)

                control_truck = carla.VehicleControl()
                control_truck.steer = steer_output_truck
                if throttle_output_truck >= 0.0:
                    control_truck.throttle = throttle_output_truck
                    control_truck.brake = 0.0
                else:
                    control_truck.throttle = 0.0
                    control_truck.brake = abs(throttle_output_truck)
                truck.apply_control(control_truck)

            # --- 小轿车 PID 控制逻辑 ---
            if car and car.is_alive:
                tf_car = car.get_transform()
                vel_car = car.get_velocity()
                speed_car = 3.6 * math.sqrt(vel_car.x ** 2 + vel_car.y ** 2 + vel_car.z ** 2)

                if tf_car.location.x < car_slowdown_x_threshold and car_target_speed > car_slowdown_speed:
                    car_target_speed = car_slowdown_speed
                    print(f"小轿车在 X={tf_car.location.x:.2f} 处减速，目标速度变为 {car_target_speed} km/h")

                target_wp_xy_car = get_target_waypoint(tf_car.location, CAR_PATH_POINTS, lookahead_dist=8.0)
                throttle_output_car = car_lon_controller.run_step(car_target_speed, speed_car)
                steer_output_car = car_lat_controller.run_step(target_wp_xy_car, tf_car)

                control_car = carla.VehicleControl()
                control_car.steer = steer_output_car
                if throttle_output_car >= 0.0:
                    control_car.throttle = throttle_output_car
                    control_car.brake = 0.0
                else:
                    control_car.throttle = 0.0
                    control_car.brake = abs(throttle_output_car)
                car.apply_control(control_car)

            # --- Mini Cooper PID 控制逻辑 (新增) ---
            if mini and mini.is_alive:
                tf_mini = mini.get_transform()
                vel_mini = mini.get_velocity()
                speed_mini = 3.6 * math.sqrt(vel_mini.x ** 2 + vel_mini.y ** 2 + vel_mini.z ** 2)
                current_y = tf_mini.location.y

                # Mini 状态机逻辑判定 (因为车一开始往负Y方向行驶)
                if mini_speed_state == 0 and current_y < -555:
                    mini_target_speed = 10
                    mini_speed_state = 1
                    print(f"Mini Cooper在 Y={current_y:.2f} 处达到阈值，减速至 {mini_target_speed} km/h")
                elif mini_speed_state == 1 and current_y < -660:
                    mini_target_speed = 40
                    mini_speed_state = 2
                    print(f"Mini Cooper在 Y={current_y:.2f} 处达到阈值，重新加速至 {mini_target_speed} km/h")

                target_wp_xy_mini = get_target_waypoint(tf_mini.location, MINI_PATH_POINTS, lookahead_dist=8.0)
                throttle_output_mini = mini_lon_controller.run_step(mini_target_speed, speed_mini)
                steer_output_mini = mini_lat_controller.run_step(target_wp_xy_mini, tf_mini)

                control_mini = carla.VehicleControl()
                control_mini.steer = steer_output_mini
                if throttle_output_mini >= 0.0:
                    control_mini.throttle = throttle_output_mini
                    control_mini.brake = 0.0
                else:
                    control_mini.throttle = 0.0
                    control_mini.brake = abs(throttle_output_mini)
                mini.apply_control(control_mini)

            # --- 时间同步 ---
            compute_time = time.time() - start_time
            if compute_time < settings.fixed_delta_seconds:
                time.sleep(settings.fixed_delta_seconds - compute_time)

    except Exception as e:
        print(f"发生异常：{e}")
    except KeyboardInterrupt:
        print("\n用户停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        if actor_list:
            client.apply_batch([carla.command.DestroyActor(a) for a in actor_list])
            print(f"已销毁 {len(actor_list)} 个 Actor。")
        world.apply_settings(original_settings)
        print("清理完成，Carla 已恢复正常。")


if __name__ == '__main__':
    main()