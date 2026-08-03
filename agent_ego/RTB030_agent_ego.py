import carla
import time
import math
import numpy as np

# ==========================================
# 基础控制算法 (PID) - 保持原样
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
    if distance > 6.0:
        print(f"[{vehicle.type_id}] 偏离道路中心 {distance:.2f} 米，判定出界被销毁！")
        vehicle.destroy()
        return True
    return False

def destroy_actor(actor):
    if actor and actor.is_alive:
        actor.destroy()

def destroy_all_actors(actor_list):
    for actor in actor_list:
        destroy_actor(actor)

# ==========================================
# 轨迹数据定义 & 核心逻辑函数
# ==========================================

# 原始轨迹数据 (您提供的两辆车的数据)
RAW_EGO_TRAJECTORY = [
    (-307.424, 5.362, -7.695), (-307.424, 5.362, -7.695), (-307.424, 5.362, -7.695), (-307.005, 5.305, -7.486),
    (-305.772, 5.169, -5.98), (-305.61, 5.152, -5.98), (-303.215, 4.907, -5.625), (-300.679, 4.672, -5.2),
    (-298.217, 4.458, -4.637), (-295.663, 4.274, -3.337), (-293.188, 4.184, -1.463), (-290.67, 4.129, -1.035),
    (-288.108, 4.127, 0.976), (-285.546, 4.201, 3.194), (-282.994, 4.458, 8.693), (-280.518, 4.948, 12.918),
    (-278.125, 5.608, 16.705), (-275.773, 6.4, 20.372), (-273.485, 7.362, 25.001), (-271.264, 8.557, 32.246),
    (-269.188, 9.991, 37.277), (-267.247, 11.604, 41.525), (-265.439, 13.305, 44.748), (-263.668, 15.158, 46.791),
    (-261.941, 16.997, 46.791), (-260.187, 18.865, 46.791), (-258.431, 20.734, 46.861), (-256.773, 22.579, 49.875),
    (-255.184, 24.483, 50.162), (-253.568, 26.419, 50.162), (-251.979, 28.324, 50.162), (-250.348, 30.304, 50.868),
    (-248.753, 32.263, 50.655), (-247.118, 34.247, 50.235), (-245.468, 36.221, 49.883), (-243.821, 38.146, 49.036),
    (-242.082, 40.045, 45.006), (-240.185, 41.724, 37.833), (-238.163, 43.252, 35.896), (-236.076, 44.69, 32.284),
    (-233.871, 46.021, 30.279), (-231.718, 47.278, 30.279), (-229.495, 48.576, 30.209), (-227.257, 49.845, 29.284),
    (-225.006, 51.089, 28.722), (-222.824, 52.285, 28.722), (-220.605, 53.499, 28.652), (-218.419, 54.688, 28.23),
    (-216.153, 55.899, 28.09), (-213.881, 57.099, 27.09), (-211.595, 58.268, 27.09), (-209.344, 59.412, 26.233),
    (-207.039, 60.521, 23.726), (-204.696, 61.525, 22.731), (-202.403, 62.395, 17.169), (-200.073, 63.003, 13.565),
    (-197.7, 63.538, 10.072), (-195.293, 63.882, 5.964), (-192.825, 64.014, 0.72), (-190.347, 64.01, -3.175),
    (-187.965, 63.68, -13.523), (-185.63, 63.018, -17.165), (-183.242, 62.209, -22.813), (-181.054, 61.01, -33.019),
    (-178.944, 59.586, -35.735), (-177.035, 58.016, -42.594), (-175.195, 56.216, -44.72), (-173.379, 54.44, -43.66),
    (-171.506, 52.661, -43.445), (-169.631, 50.884, -43.445), (-167.753, 49.109, -43.165), (-165.858, 47.426, -38.186),
    (-163.721, 46.051, -23.116), (-161.268, 45.638, 2.899), (-158.943, 46.39, 27.964), (-156.848, 47.786, 37.846),
    (-155.016, 49.534, 51.379), (-153.81, 51.78, 68.4), (-153.177, 54.175, 79.482), (-152.838, 56.674, 86.43),
    (-152.851, 59.239, 91.135), (-152.865, 59.942, 91.135), (-153.3, 62.412, 109.214), (-154.424, 64.663, 124.172),
    (-156.162, 66.539, 140.516), (-158.218, 67.999, 146.853), (-160.332, 69.375, 147.825), (-162.551, 70.661, 151.205),
    (-164.819, 71.861, 153.135), (-167.064, 72.919, 155.373), (-169.32, 73.952, 155.655), (-171.574, 74.916, 157.585),
    (-173.932, 75.861, 158.225), (-176.218, 76.774, 158.225), (-178.545, 77.704, 158.225), (-180.914, 78.659, 157.803),
    (-183.205, 79.601, 157.453), (-185.565, 80.597, 155.954), (-187.847, 81.646, 154.463), (-190.149, 82.744, 155.04),
    (-192.472, 83.796, 155.893), (-194.724, 84.804, 155.893), (-196.978, 85.809, 156.32), (-199.322, 86.834, 156.671),
    (-201.678, 87.844, 156.811), (-203.996, 88.838, 156.671), (-206.314, 89.853, 155.969), (-208.626, 90.898, 155.48),
    (-210.899, 91.938, 154.765), (-213.222, 93.064, 153.409), (-215.52, 94.241, 152.767), (-217.779, 95.404, 152.767),
    (-220.056, 96.619, 150.337), (-222.156, 97.972, 144.033), (-224.134, 99.634, 134.856), (-225.701, 101.677, 120.222),
    (-226.674, 104.056, 103.379), (-226.821, 106.571, 78.998), (-226.805, 106.653, 77.041), (-226.805, 106.653, 77.041),
    (-225.898, 108.687, 51.83), (-223.891, 110.189, 16.685), (-221.373, 110.489, 0.066), (-218.914, 110.08, -14.574),
    (-216.519, 109.369, -17.463), (-214.613, 108.769, -17.463), (-214.613, 108.769, -17.463),
    (-214.613, 108.769, -17.463), (-214.613, 108.769, -17.463), (-213.549, 108.402, -19.087), (-211.109, 107.558, -19.157),
    (-208.677, 106.695, -19.649), (-206.254, 105.81, -20.139), (-203.91, 104.951, -20.139), (-201.57, 104.082, -20.699), (-199.161, 103.171, -20.769),
    (-196.792, 102.27, -20.839), (-194.462, 101.384, -20.839), (-192.132, 100.497, -20.839), (-189.81, 99.591, -22.045),
    (-187.427, 98.598, -22.82), (-185.125, 97.629, -22.82), (-182.745, 96.628, -22.82), (-180.365, 95.626, -22.82),
    (-178.062, 94.656, -22.89), (-175.757, 93.696, -22.539), (-173.375, 92.707, -22.539), (-173.144, 92.611, -22.539),
    (-169.974, 91.296, -22.539), (-164.019, 88.824, -22.539), (-161.380, 87.820, -25.813), (-161.059, 87.665, -25.813), (-160.614, 87.450, -25.442), (-160.165, 87.238, -25.054),
    (-159.723, 87.032, -25.054), (-159.275, 86.816, -26.227), (-158.828, 86.594, -25.982), (-158.385, 86.379, -25.917),
    (-157.935, 86.160, -25.917), (-157.485, 85.943, -25.299), (-157.030, 85.732, -24.583), (-156.571, 85.527, -23.945),
    (-155.065, 84.858, -23.945), (-151.619, 83.328, -23.945), (-148.392, 81.895, -23.944), (-147.259, 81.390, -24.066),
    (-146.119, 80.890, -23.151), (-144.953, 80.392, -23.150), (-143.786, 79.893, -23.149), (-142.637, 79.401, -23.148),
    (-138.634, 77.690, -23.148), (-133.963, 75.687, -22.938), (-129.343, 73.775, -22.314), (-124.654, 71.812, -23.312),
    (-120.058, 69.842, -22.855), (-115.360, 67.911, -21.993), (-110.733, 66.032, -21.736), (-105.943, 64.116, -21.901),
    (-101.315, 62.256, -21.901), (-101.315, 62.256, -21.901), (-101.315, 62.256, -21.901), (-101.315, 62.256, -21.901),
    (-101.315, 62.256, -21.901), (-101.315, 62.256, -21.901)
]

RAW_V2_TRAJECTORY = [
    (-156.481, 115.872, -35.327), (-156.481, 115.872, -35.327), (-154.835, 114.739, -34.335),
    (-150.632, 112.035, -31.698),
    (-146.356, 109.445, -30.336), (-142.057, 106.888, -31.184), (-137.79, 104.282, -31.604),
    (-133.396, 101.564, -31.744),
    (-129.001, 98.845, -31.744), (-124.611, 96.12, -32.096), (-120.396, 93.429, -32.734), (-116.124, 90.674, -32.597),
    (-111.911, 87.981, -32.597), (-107.678, 85.32, -31.385), (-103.327, 82.692, -30.96), (-99.031, 80.133, -30.748),
    (-94.649, 77.556, -30.253), (-90.336, 75.043, -30.183), (-85.954, 72.494, -30.183), (-81.578, 69.937, -30.393),
    (-77.153, 67.303, -31.028), (-72.746, 64.654, -30.958), (-68.338, 62.006, -31.526), (-64.03, 59.342, -31.739),
    (-59.821, 56.673, -33.809), (-55.768, 53.506, -46.81), (-54.774, 52.25, -58.699), (-54.439, 51.674, -61.416),
    (-53.77, 49.891, -86.576), (-53.77, 49.891, -86.576), (-53.88, 48.175, -128.401), (-55.578, 46.666, -153.416),
    (-55.652, 46.629, -153.778), (-58.038, 46.154, 179.87), (-58.038, 46.154, -177.162), (-62.298, 45.615, -176.433),
    (-67.206, 46.3, 164.675), (-71.982, 47.754, 160.221), (-76.756, 49.535, 158.436), (-81.513, 51.545, 156.366),
    (-86.233, 53.634, 156.654), (-90.919, 55.59, 157.504), (-95.691, 57.556, 157.644), (-100.387, 59.487, 157.644),
    (-105.006, 61.387, 157.644), (-109.779, 63.35, 157.644), (-114.467, 65.303, 156.934), (-119.063, 67.26, 156.934),
    (-123.659, 69.217, 156.934), (-128.263, 71.158, 157.287), (-133.041, 73.113, 158.067), (-137.695, 74.927, 158.85),
    (-142.507, 76.793, 158.71), (-147.239, 78.637, 158.71), (-152.048, 80.511, 158.71), (-156.859, 82.386, 158.71),
    (-161.591, 84.229, 158.71), (-166.237, 86.054, 158.497), (-171.036, 87.944, 158.637), (-175.861, 89.763, 159.43),
    (-180.668, 91.63, 157.79), (-185.341, 93.608, 156.51), (-190.064, 95.68, 156.016), (-194.774, 97.784, 155.876),
    (-199.343, 99.795, 156.298), (-203.899, 101.835, 155.801), (-208.521, 103.932, 155.379),
    (-212.984, 106.168, 151.329),
    (-217.438, 108.603, 151.329), (-221.219, 109.98, -160.172), (-221.747, 109.574, -141.505),
    (-223.13, 108.372, -110.648),
    (-223.559, 107.198, -107.828), (-224.039, 105.345, -97.469), (-224.158, 102.357, -63.863),
    (-223.442, 101.436, -51.806),
    (-219.663, 98.243, -28.057), (-214.936, 96.179, -23.657), (-210.259, 94.017, -25.571), (-205.793, 91.836, -25.347),
    (-201.135, 89.687, -24.069), (-196.511, 87.662, -23.149), (-191.945, 85.71, -23.149), (-187.227, 83.692, -23.009),
    (-182.522, 81.67, -24.093), (-177.949, 79.646, -23.737), (-173.339, 77.573, -24.586), (-168.686, 75.472, -24.16),
    (-164.098, 73.37, -25.581), (-159.554, 71.149, -26.791), (-155.354, 68.495, -37.489), (-152.234, 65.422, -53.116),
    (-152.234, 65.422, -66.788), (-152.18, 65.266, -71.02), (-151.89, 64.41, -78.584), (-151.432, 62.238, -78.509),
    (-151.389, 60.436, -91.35), (-151.945, 55.389, -106.147), (-153.779, 50.647, -121.371), (-157, 46.941, -143.737),
    (-157.67, 46.453, -144.025), (-157.67, 46.453, -144.025), (-157.872, 46.308, -144.89), (-162.663, 45.015, 170.716),
    (-167.202, 47.077, 131.254), (-170.446, 50.857, 135.784), (-174.156, 54.135, 141.678), (-178.228, 56.942, 148.956),
    (-182.802, 59.209, 159.039), (-187.599, 60.57, 170.301), (-192.587, 61.275, 175.498), (-197.556, 61.035, -170.701),
    (-202.182, 59.624, -158.674), (-206.729, 57.573, -154.41), (-211.245, 55.177, -150.843),
    (-215.69, 52.649, -150.205),
    (-219.991, 50.171, -149.855), (-224.311, 47.703, -150.86), (-228.611, 45.174, -149.339),
    (-233.043, 42.536, -148.476),
    (-237.367, 39.736, -145.112), (-241.282, 36.667, -140.136), (-245.032, 33.281, -134.284),
    (-248.304, 29.54, -130.189),
    (-251.62, 25.615, -130.119), (-254.912, 21.672, -129.764), (-258.195, 17.722, -129.694),
    (-261.377, 13.904, -130.882),
    (-265.032, 10.429, -142.487), (-269.316, 7.637, -150.67), (-273.866, 5.299, -157.26), (-278.634, 3.439, -162.095),
    (-283.617, 2.292, -171.956), (-288.552, 1.808, -178.27), (-290.457, 1.776, -179.423), (-290.457, 1.776, -179.423),
    (-290.457, 1.776, -179.423), (-290.457, 1.776, -179.423)
]

def clean_trajectory(traj, min_distance=0.5):
    """去除轨迹中的重复点或距离过近的点，防止PID计算瞬间出错"""
    if not traj: return []
    cleaned_traj = [traj[0]]
    for pt in traj[1:]:
        dist = math.hypot(pt[0] - cleaned_traj[-1][0], pt[1] - cleaned_traj[-1][1])
        if dist >= min_distance:
            cleaned_traj.append(pt)
    return cleaned_traj

def get_dynamic_target_speed(trajectory, current_idx, lookahead_distance=20.0, max_angle=60.0):
    """
    使用滑动窗口逻辑：往前方搜索固定距离（而不是锚点个数），
    如果这段距离内累积转角变化超过 max_angle，则降速到 5km/h，否则 30km/h
    """
    if current_idx >= len(trajectory) - 1:
        return 30.0

    start_yaw = trajectory[current_idx][2]
    accumulated_distance = 0.0
    max_yaw_diff = 0.0

    # 向前方滑动窗口搜索
    for i in range(current_idx + 1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i - 1][0]
        dy = trajectory[i][1] - trajectory[i - 1][1]
        dist = math.hypot(dx, dy)
        accumulated_distance += dist

        # 超过观测距离则停止搜索
        if accumulated_distance > lookahead_distance:
            break

        current_yaw = trajectory[i][2]
        # 计算角度差并归一化到 0-180 度之间
        diff = abs(current_yaw - start_yaw)
        while diff > 180.0: diff -= 360.0
        while diff < -180.0: diff += 360.0
        diff = abs(diff)

        if diff > max_yaw_diff:
            max_yaw_diff = diff

    # 如果前方视野内出现大于60度的转角，视为急弯减速
    if max_yaw_diff >= max_angle:
        return 10.0
    return 30.0

def apply_initial_speed(vehicle, speed_kmh, yaw_deg):
    """根据朝向，利用物理引擎赋予车辆平稳初速度"""
    speed_ms = speed_kmh / 3.6
    yaw_rad = math.radians(yaw_deg)
    vec = carla.Vector3D(x=speed_ms * math.cos(yaw_rad), y=speed_ms * math.sin(yaw_rad), z=0.0)
    vehicle.set_target_velocity(vec)

# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 严格按照需求图片设定天气参数
    weather = carla.WeatherParameters(
        cloudiness=100.0,
        precipitation=50.0,
        precipitation_deposits=100.0,
        wind_intensity=60.0,
        sun_azimuth_angle=-1.0,
        sun_altitude_angle=-90.0,  # 夜晚
        fog_density=70.0,
        fog_distance=75.0,
        fog_falloff=1.0,
        wetness=100.0,
        scattering_intensity=1.0,
        mie_scattering_scale=0.0300,
        rayleigh_scattering_scale=0.0331
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    ego_active = False
    v2_active = False

    # 轨迹去重清洗
    EGO_TRAJECTORY = clean_trajectory(RAW_EGO_TRAJECTORY)
    V2_TRAJECTORY = clean_trajectory(RAW_V2_TRAJECTORY)

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_v2 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # 黑夜灯光掩码：示宽灯(1) | 近光灯(2) | 远光灯(4) = 7
        dark_light_state = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam | carla.VehicleLightState.HighBeam

        # ================= Actor 1：Ego车 (Lincoln MKZ 2017) =================
        bp_ego = bp_lib.find('vehicle.lincoln.mkz_2017')
        if bp_ego.has_attribute('color'): bp_ego.set_attribute('color', '255,0,0')
        ego_start_x, ego_start_y, ego_start_yaw = EGO_TRAJECTORY[0]
        ego_loc = carla.Location(x=ego_start_x, y=ego_start_y, z=1.0)  # 稍微抬高避免穿模
        ego_loc.z = carla_map.get_waypoint(ego_loc, project_to_road=True).transform.location.z + 0.5
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        if ego:
            ego_active = True
            print("生成 Ego (Lincoln MKZ 2017) 成功。")

        # ================= Actor 2：NPC车 (yamaha) =================
        bp_v2 = bp_lib.find('vehicle.yamaha.yzf')
        v2_start_x, v2_start_y, v2_start_yaw = V2_TRAJECTORY[0]
        v2_loc = carla.Location(x=v2_start_x, y=v2_start_y, z=1.0)
        v2_loc.z = carla_map.get_waypoint(v2_loc, project_to_road=True).transform.location.z + 0.5
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(v2_loc, carla.Rotation(yaw=v2_start_yaw)))
        if v2:
            actor_list.append(v2)
            v2_active = True
            v2.set_light_state(carla.VehicleLightState(dark_light_state))
            print("生成 NPC (yamaha) 成功。")

        # 核心修改：让物理引擎预热15帧，让车辆安稳落地，防止因为悬空状态加上速度导致车辆乱飞
        for _ in range(15):
            world.tick()

        # 稳定落地后，赋予初始物理速度 (30km/h)
        if ego_active: apply_initial_speed(ego, 30.0, ego_start_yaw)
        if v2_active: apply_initial_speed(v2, 30.0, v2_start_yaw)
        print("已平稳赋予两车初始速度 30 km/h。")

        ego_traj_idx = 0
        v2_traj_idx = 0

        print("\n仿真正式开始！")

        while True:
            start_time = time.time()
            world.tick()

            # ==========================
            # Ego 车：循迹与动态弯道控速
            # ==========================

            # ==========================
            # V2 车辆 (yamaha)：循迹与动态弯道控速
            # ==========================
            if v2_active and v2.is_alive:
                if check_and_handle_out_of_bounds(v2, carla_map):
                    v2_active = False
                elif v2_traj_idx < len(V2_TRAJECTORY):
                    tx, ty, tyaw = V2_TRAJECTORY[v2_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v2.get_location().z)
                    target_distance = v2.get_location().distance(target_loc)

                    if target_distance < 3.0 and v2_traj_idx < len(V2_TRAJECTORY) - 1:
                        v2_traj_idx += 1
                    elif target_distance < 3.0 and v2_traj_idx >= len(V2_TRAJECTORY) - 1:
                        print("\nV2 reached the trajectory endpoint; destroying actor.")
                        destroy_actor(v2)
                        v2_active = False
                        v2 = None
                        continue

                    v2_target_speed = get_dynamic_target_speed(V2_TRAJECTORY, v2_traj_idx, lookahead_distance=20.0,
                                                               max_angle=60.0)
                    apply_pid_control(v2, pid_v2['lon'], pid_v2['lat'], v2_target_speed, target_loc)
                else:
                    v2.apply_control(carla.VehicleControl(brake=1.0))
                    v2_active = False
                    print("\nV2 已到达轨迹终点。")

            # 两辆车都走完了退出
            if not ego_active and not v2_active:
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
        destroy_all_actors(actor_list)

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("清理完毕。")

if __name__ == '__main__':
    main()
