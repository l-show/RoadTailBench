import carla
import time
import math
import numpy as np


# ==========================================
# 基础控制算法 (PID) - 保留
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
        # 加速上限设为0.8，保证加速足够合理
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


# ==========================================
# 轨迹去重工具函数
# ==========================================
def remove_duplicate_points(trajectory, distance_threshold=0.1):
    """剔除轨迹中距离过近的重复点"""
    if not trajectory: return []
    cleaned_traj = [trajectory[0]]
    for pt in trajectory[1:]:
        last_pt = cleaned_traj[-1]
        dist = math.sqrt((pt[0] - last_pt[0]) ** 2 + (pt[1] - last_pt[1]) ** 2)
        if dist > distance_threshold:
            cleaned_traj.append(pt)
    return cleaned_traj


# ==========================================
# 原始轨迹数据定义
# ==========================================
TRAJ_1_FOLLOWER = [
    (1.599, 192.866, -89.446), (1.599, 192.866, -89.446), (1.599, 192.866, -89.446), (1.599, 192.866, -89.446),
    (1.589, 190.242, -90.216), (1.426, 178.629, -92.176), (1.367, 167.503, -87.696), (1.769, 156.333, -90.846),
    (1.589, 145.183, -90.285), (1.523, 133.987, -90.705), (1.364, 122.825, -90.845), (1.279, 111.296, -90.075),
    (1.41, 99.949, -89.235), (1.745, 88.373, -87.765), (1.964, 76.979, -90.215), (1.923, 65.448, -90.075),
    (1.918, 53.87, -89.934), (1.907, 42.268, -90.424), (1.803, 30.834, -90.704), (1.645, 19.213, -90.914),
    (1.465, 7.966, -90.914), (1.286, -3.283, -90.914), (1.122, -14.907, -90.774), (0.999, -26.156, -90.564),
    (0.937, -37.592, -89.794), (1.062, -49.015, -89.164), (1.229, -60.426, -89.164), (1.474, -71.846, -87.764),
    (1.927, -83.441, -87.764), (2.34, -94.656, -88.604), (2.258, -106.235, -91.194), (2.017, -117.8, -91.054),
    (1.85, -129.354, -90.634), (1.724, -140.762, -90.634), (1.595, -152.426, -90.634), (1.5, -163.835, -90.424),
    (1.415, -175.437, -90.424), (1.329, -187.038, -90.424), (1.273, -198.461, -90.144), (1.334, -209.704, -89.514),
    (1.458, -221.323, -89.374), (1.597, -232.948, -89.304), (1.734, -244.199, -89.304), (1.827, -255.628, -89.584),
    (1.91, -267.053, -89.584), (1.891, -278.298, -90.354), (1.811, -289.547, -90.424), (1.727, -300.984, -90.424),
    (1.641, -312.611, -90.424), (1.558, -323.86, -90.424), (1.475, -335.108, -90.424), (1.389, -346.622, -90.424),
    (1.306, -357.869, -90.424), (1.252, -365.18, -90.424), (1.252, -365.18, -90.424), (1.252, -365.18, -90.424)
]

TRAJ_2_OPPOSITE = [
    (-2.564, -407.423, 94.36), (-2.564, -407.423, 94.36), (-2.564, -407.423, 94.36), (-2.564, -407.423, 93.94),
    (-2.564, -407.423, 93.45), (-3.007, -398.997, 92.61), (-3.325, -387.403, 90.997), (-3.244, -376.002, 87.428),
    (-3.232, -364.408, 92.187), (-3.605, -353.169, 91.347), (-3.292, -341.621, 87.707), (-2.926, -330.256, 89.177),
    (-2.894, -319.006, 89.877), (-2.881, -307.37, 90.087), (-2.899, -295.735, 90.087), (-2.916, -284.298, 90.087),
    (-3.003, -272.679, 90.507), (-3.1, -261.247, 90.297), (-3.16, -249.621, 90.297), (-3.221, -237.995, 90.297),
    (-3.28, -226.558, 90.297), (-3.239, -214.94, 89.597), (-3.157, -203.326, 89.597), (-3.075, -191.711, 89.597),
    (-2.995, -180.289, 89.597), (-2.913, -168.684, 89.597), (-2.832, -157.119, 89.597), (-2.752, -145.761, 89.596),
    (-2.701, -134.541, 90.156), (-2.805, -123.237, 90.576), (-2.92, -111.816, 90.576), (-3.059, -100.217, 90.366),
    (-3.064, -89.016, 90.016), (-3.04, -77.417, 89.736), (-2.911, -66.189, 89.316), (-2.824, -54.772, 89.876),
    (-2.8, -43.556, 89.876), (-2.878, -32.132, 91.136), (-3.066, -20.698, 90.576), (-3.161, -9.449, 90.156),
    (-3.138, 2.166, 89.526), (-3.06, 13.417, 89.666), (-2.993, 24.854, 89.666), (-2.928, 36.103, 89.666),
    (-2.862, 47.539, 89.946), (-2.879, 58.944, 90.506), (-3.003, 70.374, 90.645), (-3.132, 81.797, 90.645),
    (-3.224, 93.2, 89.945), (-3.12, 104.729, 89.385), (-3.001, 115.826, 89.385), (-2.967, 127.314, 89.945),
    (-2.943, 138.864, 89.665), (-2.878, 150.044, 89.665), (-2.813, 161.221, 90.085), (-2.918, 172.414, 90.505),
    (-3.003, 184.032, 90.365), (-3.069, 194.53, 90.365), (-3.069, 194.53, 90.365)
]

TRAJ_3_EGO = [
    (1.17, 158.353, -81.191), (1.17, 158.353, -81.191), (1.17, 158.353, -81.191), (1.17, 158.353, -81.191),
    (1.58, 155.243, -83.99), (1.927, 151.665, -86.509), (2.045, 147.981, -88.609), (2.134, 144.308, -88.609),
    (2.228, 140.445, -88.609), (2.349, 135.492, -88.609), (2.462, 130.539, -89.728), (2.387, 125.666, -92.108),
    (2.116, 120.86, -93.507), (1.822, 116.058, -93.507), (1.517, 111.102, -93.927), (1.094, 106.321, -95.607),
    (0.683, 101.378, -91.478), (0.796, 96.48, -87.418), (0.986, 91.68, -88.258), (1.137, 86.73, -88.258),
    (1.247, 81.938, -89.097), (1.31, 76.98, -89.517), (1.292, 72.01, -90.917), (1.198, 67.03, -91.197),
    (1.096, 62.129, -91.197), (1.005, 57.145, -90.777), (0.968, 52.314, -90.217), (0.951, 47.4, -89.937),
    (0.956, 42.567, -89.937), (0.962, 37.655, -89.937), (0.977, 32.663, -89.377), (1.024, 27.67, -89.937),
    (1.008, 22.676, -90.217), (1.005, 17.763, -89.377), (1.091, 12.75, -88.957), (1.205, 6.544, -88.957),
    (1.336, -1.033, -89.587), (1.25, -8.366, -90.986), (1.118, -15.99, -90.986), (0.966, -24.859, -90.986),
    (0.805, -33.584, -91.406), (0.519, -42.445, -91.826), (0.489, -51.279, -87.832), (0.824, -60.141, -87.972),
    (1.127, -68.713, -88.252), (1.292, -77.282, -89.092), (1.358, -85.836, -89.932), (1.323, -94.664, -90.631),
    (1.229, -103.209, -90.631), (1.131, -112.054, -90.631), (1.035, -120.759, -90.631), (0.928, -130.443, -90.631),
    (0.809, -141.288, -90.701), (0.83, -152.718, -89.931), (0.75, -164.142, -90.491), (1.019, -175.533, -84.333),
    (2.048, -186.692, -87.692), (1.588, -198.117, -93.43), (1.08, -209.551, -90.701), (1.056, -220.817, -89.581),
    (0.936, -232.267, -91.401), (0.84, -243.71, -89.301), (1.067, -255.154, -88.601), (1.342, -266.428, -88.601),
    (1.634, -277.692, -88.461), (1.884, -288.957, -89.161), (1.903, -300.408, -90.561), (1.77, -311.489, -90.701),
    (1.606, -322.939, -90.98), (1.56, -334.02, -89.721), (1.632, -345.29, -89.441), (1.705, -356.554, -89.721),
    (1.759, -367.633, -89.721), (1.815, -379.081, -89.721), (1.817, -379.45, -89.721)
]

TRAJ_4_OVERTAKE = [
    (5.791, 154.445, -88.736), (5.791, 154.445, -88.736), (5.791, 154.445, -88.736), (5.791, 154.445, -88.736),
    (5.791, 154.445, -93.635), (5.638, 149.511, -91.325), (5.571, 142.377, -90.205), (5.577, 135.469, -89.925),
    (5.584, 128.505, -89.505), (5.698, 121.3, -89.015), (5.813, 114.361, -89.155), (5.909, 107.452, -89.995),
    (5.668, 100.261, -95.454), (4.544, 93.276, -100.073), (3.28, 86.206, -100.213), (2.273, 79.091, -94.615),
    (1.859, 71.895, -91.185), (1.724, 64.907, -90.205), (1.767, 57.811, -89.366), (1.847, 50.6, -89.366),
    (1.929, 43.144, -89.366), (2.047, 32.528, -89.366), (1.937, 21.44, -90.135), (2.18, 10.514, -88.455),
    (2.382, -0.227, -89.155), (2.501, -11.306, -89.435), (2.622, -22.41, -89.435), (2.665, -33.339, -90.555),
    (2.555, -44.267, -90.835), (2.3, -55.175, -91.675), (1.976, -66.232, -91.675), (1.679, -76.931, -91.395),
    (1.426, -87.812, -90.835), (1.533, -98.891, -88.455), (1.839, -109.982, -88.315), (2.099, -120.704, -88.735),
    (2.31, -131.78, -90.274), (2.078, -142.508, -91.394), (1.93, -153.433, -90.694), (1.796, -164.536, -90.694),
    (1.615, -175.454, -91.394), (1.48, -186.148, -89.784), (1.562, -197.194, -88.944), (1.83, -207.939, -88.524),
    (2.057, -218.687, -89.994), (1.953, -229.616, -90.553), (1.875, -240.545, -90.133), (1.944, -251.293, -89.573),
    (2.168, -262.39, -86.494), (2.976, -273.104, -85.234), (3.897, -284.171, -85.514), (4.565, -295.253, -90.203),
    (4.299, -306.356, -91.602), (4.052, -317.281, -90.902), (4.006, -328.209, -88.803), (4.244, -338.955, -88.733),
    (4.462, -348.807, -88.733)
]


# ==========================================
# 主程序
# ==========================================


# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (1.815, -379.081)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.chevrolet.impala'
_RTB_OPT_EGO_ROLE_NAMES = ['ego', 'hero']
_RTB_OPT_GOAL_RADIUS_M = 5.0
_RTB_OPT_GOAL_HITS = 0


def _rtb_opt_is_alive(actor):
    return bool(actor is not None and hasattr(actor, 'is_alive') and actor.is_alive)


def _rtb_opt_iter_actor_values(value, seen=None):
    if seen is None:
        seen = set()
    obj_id = id(value)
    if obj_id in seen:
        return
    seen.add(obj_id)
    if _rtb_opt_is_alive(value) and hasattr(value, 'get_location'):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from _rtb_opt_iter_actor_values(item, seen)
    elif isinstance(value, (list, tuple, set)):
        for item in value:
            yield from _rtb_opt_iter_actor_values(item, seen)


def _rtb_opt_actor_matches_ego(actor):
    if not _rtb_opt_is_alive(actor):
        return False
    try:
        role_name = actor.attributes.get('role_name', '')
        if role_name in _RTB_OPT_EGO_ROLE_NAMES:
            return True
    except Exception:
        pass
    try:
        if _RTB_OPT_EGO_TYPE_ID and actor.type_id == _RTB_OPT_EGO_TYPE_ID:
            return True
    except Exception:
        pass
    return False


def _rtb_opt_find_ego(local_vars):
    preferred_names = ('ego', 'ego_vehicle', 'vehicle_ego', 'v3_ego', 'v2_ego', 'agent_ego', 'audi', 'tesla', 'moto', 'truck', 'firetruck')
    for name in preferred_names:
        if name in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[name]):
                if _rtb_opt_actor_matches_ego(actor) or 'ego' in name.lower():
                    return actor
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            if _rtb_opt_actor_matches_ego(actor):
                return actor
    return None


def _rtb_opt_collect_scene_actors(local_vars, world):
    actors = []
    seen = set()

    def add(actor):
        if not _rtb_opt_is_alive(actor):
            return
        try:
            actor_id = actor.id
        except Exception:
            actor_id = id(actor)
        if actor_id in seen:
            return
        seen.add(actor_id)
        actors.append(actor)

    for key in ('actor_list', 'actors', 'vehicles', 'spawned_actors'):
        if key in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[key]):
                add(actor)
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            add(actor)
    try:
        world_actors = world.get_actors()
        for pattern in ('vehicle.*', 'walker.*', 'sensor.*', 'controller.*', 'static.prop.*', 'static.trigger.*'):
            for actor in world_actors.filter(pattern):
                add(actor)
    except Exception:
        pass
    return actors


def _rtb_opt_cleanup_scene(local_vars, client, world):
    actors = _rtb_opt_collect_scene_actors(local_vars, world)
    try:
        commands = [carla.command.DestroyActor(actor.id) for actor in actors if _rtb_opt_is_alive(actor)]
        if commands:
            client.apply_batch(commands)
        return
    except Exception:
        pass
    for actor in actors:
        try:
            if _rtb_opt_is_alive(actor):
                actor.destroy()
        except Exception:
            pass


def _rtb_opt_goal_guard(local_vars, client, world):
    global _RTB_OPT_GOAL_HITS
    if _RTB_OPT_EGO_GOAL_XY is None:
        _RTB_OPT_GOAL_HITS = 0
        return False
    ego_actor = _rtb_opt_find_ego(local_vars)
    if not _rtb_opt_is_alive(ego_actor):
        _RTB_OPT_GOAL_HITS = 0
        return False
    try:
        loc = ego_actor.get_location()
        dist = ((loc.x - _RTB_OPT_EGO_GOAL_XY[0]) ** 2 + (loc.y - _RTB_OPT_EGO_GOAL_XY[1]) ** 2) ** 0.5
    except Exception:
        _RTB_OPT_GOAL_HITS = 0
        return False
    if dist <= _RTB_OPT_GOAL_RADIUS_M:
        _RTB_OPT_GOAL_HITS += 1
    else:
        _RTB_OPT_GOAL_HITS = 0
    if _RTB_OPT_GOAL_HITS >= 2:
        print('[RoadTailBench Opt] Ego reached trajectory endpoint; cleaning all scene actors and ending simulation.')
        _rtb_opt_cleanup_scene(local_vars, client, world)
        return True
    return False
# === End RoadTailBench Opt guard ===

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 【天气系统】：完全按照截图数据进行配置
    weather = carla.WeatherParameters(
        cloudiness=85.0000,
        precipitation=100.0000,
        precipitation_deposits=70.0000,
        wind_intensity=100.0000,
        sun_azimuth_angle=99.0000,
        sun_altitude_angle=40.0000,
        fog_density=3.0000,
        fog_distance=0.7500,
        fog_falloff=0.1000,
        wetness=75.0000,
        scattering_intensity=1.0000,
        mie_scattering_scale=0.0300,
        rayleigh_scattering_scale=0.0331,
        dust_storm=0.0000
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    # 定义车辆管理字典
    vehicles_config = [
        {
            'name': 'Follower', 'bp': 'vehicle.chevrolet.impala',
            'traj': remove_duplicate_points(TRAJ_1_FOLLOWER), 'target_speed': 60.0,
            'lights': carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
        },
        {
            'name': 'Opposite', 'bp': 'vehicle.chevrolet.impala',
            'traj': remove_duplicate_points(TRAJ_2_OPPOSITE), 'target_speed': 60.0,
            'lights': carla.VehicleLightState.Position | carla.VehicleLightState.HighBeam  # 对向车开远光灯
        },
        {
            'name': 'Ego', 'bp': 'vehicle.audi.tt',
            'traj': remove_duplicate_points(TRAJ_3_EGO), 'target_speed': 60.0,
            'lights': carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
        },
        {
            'name': 'Overtake', 'bp': 'vehicle.jeep.wrangler_rubicon',
            'traj': remove_duplicate_points(TRAJ_4_OVERTAKE), 'target_speed': 80.0,
            'lights': carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
        }
    ]

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        # 1. 批量生成车辆
        for v_config in vehicles_config:
            bp = bp_lib.find(v_config['bp'])
            start_x, start_y, start_yaw = v_config['traj'][0]

            # 生成位置Z轴略微抬高，防止卡模
            loc = carla.Location(x=start_x, y=start_y, z=1.0)
            wp = carla_map.get_waypoint(loc)
            if wp: loc.z = wp.transform.location.z + 0.5

            transform = carla.Transform(loc, carla.Rotation(yaw=start_yaw))
            actor = world.try_spawn_actor(bp, transform)

            if actor:
                actor_list.append(actor)
                v_config['actor'] = actor
                v_config['active'] = True
                v_config['traj_idx'] = 0
                v_config['pid_lon'] = PIDLongitudinalController(dt=dt)
                v_config['pid_lat'] = PIDLateralController(dt=dt)

                # 开启对应的车灯
                actor.set_light_state(carla.VehicleLightState(v_config['lights']))
                print(f"[{v_config['name']}] 生成成功。")
            else:
                v_config['active'] = False
                print(f"[{v_config['name']}] 生成失败！")

        # 2. 物理引擎预热：让车辆先稳定落在地上
        print("\n正在预热物理引擎，确保车辆贴地...")
        for _ in range(20):
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break

        # 3. 赋予所有车辆初始物理速度 (防止从 0 加速导致失控或瞬移)
        print("\n赋予初始速度...")
        for v in vehicles_config:
            if v['active']:
                init_speed_ms = v['target_speed'] / 3.6
                start_yaw_rad = math.radians(v['traj'][0][2])
                v['actor'].set_target_velocity(carla.Vector3D(
                    init_speed_ms * math.cos(start_yaw_rad),
                    init_speed_ms * math.sin(start_yaw_rad),
                    0.0
                ))

        # 再次跑两帧让速度矢量生效
        world.tick()
        world.tick()

        # 4. 进入正式仿真循环
        print("\n仿真正式开始！")
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break

            any_vehicle_active = False

            for v in vehicles_config:
                if not v['active']: continue

                actor = v['actor']
                if not actor.is_alive or check_and_handle_out_of_bounds(actor, carla_map):
                    v['active'] = False
                    continue

                any_vehicle_active = True
                traj = v['traj']
                idx = v['traj_idx']

                if idx < len(traj):
                    # 获取当前目标航点
                    tx, ty, tyaw = traj[idx]
                    target_loc = carla.Location(x=tx, y=ty, z=actor.get_location().z)

                    # 距离判定 (距离 < 3.5m 则切换到下一个轨迹点)
                    if actor.get_location().distance(target_loc) < 3.5 and idx < len(traj) - 1:
                        v['traj_idx'] += 1

                    # 执行 PID 控制
                    apply_pid_control(actor, v['pid_lon'], v['pid_lat'], v['target_speed'], target_loc)
                else:
                    # 到达终点，踩满刹车
                    actor.apply_control(carla.VehicleControl(brake=1.0))
                    v['active'] = False
                    print(f"\n[{v['name']}] 已安全到达轨迹终点。")

            if not any_vehicle_active:
                print("\n所有车辆均已跑完轨迹，仿真结束。")
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