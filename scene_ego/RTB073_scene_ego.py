import carla
import time
import math
import numpy as np

# ==========================================
# 轨迹数据清洗 (自动去重)
# ==========================================
# 1. 消防车原有轨迹
RAW_PATH_POINTS = [
    (34.197, 125.424, 0.974), (33.337, 114.480, 0.910), (33.337, 114.480, 0.910),
    (33.337, 114.480, 0.910), (33.337, 114.480, 0.910), (33.357, 113.814, 0.922),
    (33.517, 103.581, 1.183), (33.271, 101.014, 1.194), (32.974, 98.043, 1.166),
    (32.974, 98.043, 1.166), (32.480, 89.345, 1.109), (32.373, 84.241, 1.100),
    (32.373, 84.241, 1.100), (32.276, 81.759, 1.094), (31.991, 74.906, 1.072),
    (31.964, 74.540, 1.075), (31.257, 64.414, 1.158), (31.246, 64.248, 1.159),
    (31.200, 63.551, 1.165), (29.802, 53.440, 1.134), (25.685, 44.355, 0.896),
    (24.932, 42.869, 0.916), (23.802, 40.639, 0.960), (21.617, 36.328, 1.044),
    (20.560, 34.249, 1.085), (18.225, 30.387, 1.155), (15.451, 27.200, 1.229),
    (12.245, 23.969, 1.293), (9.214, 21.908, 1.285), (5.220, 20.304, 1.211),
    (2.909, 19.991, 1.152), (-1.740, 20.312, 1.101), (-6.570, 20.459, 1.078),
    (-12.404, 20.486, 1.063), (-15.063, 21.657, 1.027), (-18.396, 21.637, 0.969),
    (-21.395, 21.695, 0.910)
]

# 2. 新增的 Impala 轨迹数据 (Location_x, Location_y, Rotation_yaw)
RAW_IMPALA_PATH_POINTS = [
    (41.942, 210.443, -94.294), (41.942, 210.443, -94.294), (41.942, 210.443, -94.154),
    (41.404, 201.806, -93.524), (40.41, 185.322, -93.384), (39.745, 168.824, -91.387),
    (39.238, 152.583, -92.024), (38.722, 137.988, -92.024), (38.052, 124.025, -92.963),
    (37.341, 110.293, -92.963), (36.693, 97.769, -92.963), (36.262, 89.446, -92.963),
    (36.262, 89.446, -92.963), (36.262, 89.446, -92.963), (36.262, 89.446, -92.963),
    (36.262, 89.446, -92.963), (36.262, 89.446, -92.963), (36.262, 89.446, -92.963),
    (36.262, 89.446, -92.963), (36.064, 85.618, -92.963), (35.801, 80.541, -92.963),
    (35.543, 75.464, -92.47), (35.332, 70.469, -92.26), (35.132, 65.391, -92.26),
    (34.941, 60.312, -91.91), (34.829, 55.314, -89.529), (35.112, 50.242, -82.006),
    (35.842, 46.225, -79.393), (36.076, 44.976, -79.393), (36.312, 43.728, -78.383),
    (36.635, 42.521, -71.743), (37.077, 41.307, -68.645), (37.586, 40.143, -62.897),
    (38.207, 39.059, -57.913), (38.922, 38.009, -52.783), (39.764, 37.059, -47.314),
    (40.658, 36.157, -40.356), (41.655, 35.369, -36.699), (42.685, 34.661, -32.854),
    (43.812, 34.031, -27.501), (46.701, 32.543, -24.305), (50.225, 31.265, -17.832),
    (53.92, 30.344, -9.656), (57.695, 29.809, -7.003), (61.421, 29.396, -5.737),
    (65.222, 29.113, -3.081), (69.028, 28.917, -2.941), (72.835, 28.719, -3.011),
    (76.641, 28.519, -3.011), (78.077, 28.444, -3.011), (78.077, 28.444, -3.011),
    (78.077, 28.444, -3.011), (81.197, 28.27, -3.221), (85.0, 28.056, -3.221)
]

# 消防车轨迹去重
PATH_POINTS = []
if RAW_PATH_POINTS:
    PATH_POINTS.append(RAW_PATH_POINTS[0])
    for i in range(1, len(RAW_PATH_POINTS)):
        if RAW_PATH_POINTS[i] != RAW_PATH_POINTS[i - 1]:
            PATH_POINTS.append(RAW_PATH_POINTS[i])

# Impala 轨迹去重
IMPALA_PATH_POINTS = []
if RAW_IMPALA_PATH_POINTS:
    IMPALA_PATH_POINTS.append(RAW_IMPALA_PATH_POINTS[0])
    for i in range(1, len(RAW_IMPALA_PATH_POINTS)):
        if RAW_IMPALA_PATH_POINTS[i] != RAW_IMPALA_PATH_POINTS[i - 1]:
            IMPALA_PATH_POINTS.append(RAW_IMPALA_PATH_POINTS[i])

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

class PIDLateralController2:
    """ 横向控制 (转向) """

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
        if norm_w < 0.1: return 0.0

        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0: _dot *= -1.0

        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
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

def calculate_velocity_vector(speed, rotation):
    pitch_rad = math.radians(rotation.pitch)
    yaw_rad = math.radians(rotation.yaw)
    x = speed * math.cos(yaw_rad) * math.cos(pitch_rad)
    y = speed * math.sin(yaw_rad) * math.cos(pitch_rad)
    z = speed * math.sin(pitch_rad)
    return carla.Vector3D(x=x, y=y, z=z)

def get_target_waypoint(vehicle_loc, path_points, lookahead_dist=4.0):
    min_dist = float('inf')
    closest_index = 0

    for i, p in enumerate(path_points):
        dist = math.sqrt((p[0] - vehicle_loc.x) ** 2 + (p[1] - vehicle_loc.y) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_index = i

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

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (85.0, 28.056)
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
    preferred_names = ('ego', 'ego_vehicle', 'vehicle_ego', 'v3_ego', 'v2_ego', 'agent_ego', 'impala', 'audi', 'tesla', 'moto', 'truck', 'firetruck')
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
    weather = carla.WeatherParameters(cloudiness=40.0, precipitation=100.0,
                                      precipitation_deposits=100.0, wind_intensity=100.0,
                                      sun_azimuth_angle=90.0, sun_altitude_angle=10.0,
                                      fog_density=10.0, fog_distance=0.75, fog_falloff=0.10,
                                      wetness=100.0, scattering_intensity=11.5,
                                      mie_scattering_scale=0.21, rayleigh_scattering_scale=0.07, dust_storm=0.0)
    world.set_weather(weather)
    bp_lib = world.get_blueprint_library()

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(100.0)

        actor_list = []

        try:
            # 1. 生成 Sprinter
            bp_sprinter = bp_lib.find('vehicle.mercedes.sprinter')
            bp_sprinter.set_attribute('color', '255,255,0')
            trans_sprinter = get_transform(x=38.3, y=72.7, z=0.4, yaw=-91.0)
            sprinter = world.try_spawn_actor(bp_sprinter, trans_sprinter)
            if sprinter:
                actor_list.append(sprinter)
                control = carla.VehicleControl(hand_brake=True, brake=1.0, manual_gear_shift=True, gear=0)
                sprinter.apply_control(control)
                print("Sprinter 生成成功 (绝对静止)")

            # 2. 生成 自行车
            bp_bike = bp_lib.find('vehicle.bh.crossbike')
            trans_bike = get_transform(x=38.3, y=78.7, z=0.5, yaw=-121.0)
            bike = world.try_spawn_actor(bp_bike, trans_bike)
            if bike:
                actor_list.append(bike)
                bike.set_simulate_physics(True)
                print("自行车 生成成功")

            # 3. 生成 消防车
            bp_firetruck = bp_lib.find('vehicle.carlamotors.firetruck')
            bp_firetruck.set_attribute('color', '255,255,255')
            trans_firetruck = get_transform(x=34.197, y=125.424, z=1.0, pitch=0, yaw=-91.038)
            firetruck = world.try_spawn_actor(bp_firetruck, trans_firetruck)
            if firetruck:
                actor_list.append(firetruck)
                firetruck.set_simulate_physics(True)
                print("消防车 生成成功")
                ft_lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0)
                ft_lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2)

            # 4. 🌟 生成新增车辆 Impala
            bp_impala = bp_lib.find('vehicle.chevrolet.impala')
            bp_impala.set_attribute('color', '255,255,255')
            if bp_impala.has_attribute('role_name'):
                bp_impala.set_attribute('role_name', 'ego')
            # 提取轨迹第一个点作为初始坐标 (Z轴默认给个0.5防穿模)
            trans_impala = get_transform(x=RAW_IMPALA_PATH_POINTS[0][0], y=RAW_IMPALA_PATH_POINTS[0][1], z=0.5,
                                         yaw=RAW_IMPALA_PATH_POINTS[0][2])
            impala = world.try_spawn_actor(bp_impala, trans_impala)
            if impala:
                actor_list.append(impala)
                impala.set_simulate_physics(True)
                # 开启行车灯 (Position)
                impala.set_light_state(carla.VehicleLightState.Position)

                # 赋予初始物理速度 70km/h (转换单位为 m/s 送入引擎)
                init_speed_ms = 70.0 / 3.6
                impala.set_target_velocity(calculate_velocity_vector(init_speed_ms, trans_impala.rotation))
                print("Impala 生成成功，已开启行车灯并注入 70km/h 初速度")

                # 为 Impala 创建专属PID控制器
                im_lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0)
                im_lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2)

            # 5. 生成 足球
            bp_football = bp_lib.find('static.prop.mesh')
            bp_football.set_attribute('mesh_path', '/Game/Carla/Static/RoadTailModel/soccerball_Solid.soccerball_Solid')
            bp_football.set_attribute('mass', '50')
            bp_football.set_attribute('scale', '0.2')
            trans_football = get_transform(x=40.882, y=66.285, z=0.984)
            football = world.try_spawn_actor(bp_football, trans_football)
            if football:
                actor_list.append(football)
                football.set_simulate_physics(True)
                try:
                    football.set_linear_damping(0.5)
                    football.set_angular_damping(0.5)
                except:
                    pass
                force_vector = calculate_velocity_vector(100.0, carla.Rotation(pitch=2.045, yaw=175.270))
                football.add_impulse(force_vector)
                print("足球 生成成功，已施加冲量")

            # 6. 生成 行人
            bp_walker = bp_lib.find('walker.pedestrian.0012')
            trans_walker = get_transform(x=41.876, y=66.161, z=0.930, pitch=1.835, yaw=173.521)
            walker = world.try_spawn_actor(bp_walker, trans_walker)
            if walker:
                actor_list.append(walker)
                print("行人 生成成功")

            print("等待 1 秒，物理稳定中...")
            for _ in range(20):
                world.tick()
                if _rtb_opt_goal_guard(locals(), client, world):
                    return
                time.sleep(0.05)

            if bike:
                bike.apply_control(carla.VehicleControl(throttle=0.1))

            walker_ctrl = carla.WalkerControl()
            if walker:
                walker_ctrl.speed = 1.5
                walker_ctrl.direction = walker.get_transform().rotation.get_forward_vector()
                walker_ctrl.jump = False

            print("\n场景运行中... (多车辆协同剧本执行)")

            # --- 自行车状态变量 ---
            bike_start_frames = 100
            bike_frame_count = 0
            bike_target_speed = 1.0
            bike_current_speed = 0.0
            bike_acceleration = 0.05

            # --- 🌟 Impala 状态机变量 ---
            # 状态 0: 初始保持 70km/h
            # 状态 1: 触发减速到 35km/h，并开始计时 5 秒
            # 状态 2: 计时结束，重新加速恢复 70km/h
            impala_state = 0
            impala_timer = 0.0
            impala_target_speed = 70.0  # 控制器要追逐的目标速度
            impala_current_speed = 70.0  # 平滑过渡的中间速度

            # 主循环
            while True:
                start_time = time.time()
                world.tick()
                if _rtb_opt_goal_guard(locals(), client, world):
                    return

                # ==========================================
                # 消防车 控制流
                # ==========================================
                if firetruck:
                    tf = firetruck.get_transform()
                    vel = firetruck.get_velocity()
                    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                    target_wp = get_target_waypoint(tf.location, PATH_POINTS, lookahead_dist=5.0)
                    target_speed = 30
                    throttle_output = ft_lon_controller.run_step(target_speed, speed)
                    steer_output = ft_lat_controller.run_step(target_wp, tf)

                    control = carla.VehicleControl()
                    control.steer = steer_output
                    if throttle_output >= 0.0:
                        control.throttle = throttle_output
                        control.brake = 0.0
                    else:
                        control.throttle = 0.0
                        control.brake = abs(throttle_output)
                    firetruck.apply_control(control)

                # ==========================================
                # 🌟 Impala 控制流 (带加减速剧本)
                # ==========================================
                if impala:
                    tf = impala.get_transform()
                    vel = impala.get_velocity()
                    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                    # 1. 剧本状态机逻辑
                    if impala_state == 0 and tf.location.y <= 90.0:
                        impala_state = 1
                        impala_target_speed = 35.0  # 触发减速
                        impala_timer = 0.0
                    elif impala_state == 1:
                        impala_timer += 0.05  # 累计等待时间 (每次 tick 是 0.05s)
                        if impala_timer >= 5.0:
                            impala_state = 2
                            impala_target_speed = 70.0  # 等待完毕，触发加速

                    # 2. 速度平滑过渡 (防止 PID 直接收到突变目标导致急刹抱死)
                    # 每秒 20km/h 的加速度/减速度限制
                    if impala_current_speed < impala_target_speed:
                        impala_current_speed = min(impala_current_speed + 20.0 * 0.05, impala_target_speed)
                    elif impala_current_speed > impala_target_speed:
                        impala_current_speed = max(impala_current_speed - 20.0 * 0.05, impala_target_speed)

                    # 3. 预瞄点获取 (车速较快，预瞄距离稍微放大到 8.0 米，防止画龙)
                    target_wp = get_target_waypoint(tf.location, IMPALA_PATH_POINTS, lookahead_dist=8.0)

                    # 4. PID 结算下发
                    throttle_output = im_lon_controller.run_step(impala_current_speed, speed)
                    steer_output = im_lat_controller.run_step(target_wp, tf)

                    control = carla.VehicleControl()
                    control.steer = steer_output
                    if throttle_output >= 0.0:
                        control.throttle = throttle_output
                        control.brake = 0.0
                    else:
                        control.throttle = 0.0
                        control.brake = abs(throttle_output)
                    impala.apply_control(control)

                # ==========================================
                # 其他杂项 控制流
                # ==========================================
                if bike:
                    bike_frame_count += 1
                    if bike_frame_count <= bike_start_frames:
                        bike.set_target_velocity(carla.Vector3D(0, 0, 0))
                    else:
                        if bike_current_speed == 0.0: bike_current_speed = 0.1
                        if bike_current_speed < bike_target_speed:
                            bike_current_speed = min(bike_current_speed + bike_acceleration, bike_target_speed)
                        bike_rot = bike.get_transform().rotation
                        vel_vec = calculate_velocity_vector(bike_current_speed, bike_rot)
                        bike.set_target_velocity(vel_vec)

                if walker:
                    walker.apply_control(walker_ctrl)

                # 时间同步
                compute_time = time.time() - start_time
                if compute_time < 0.05:
                    time.sleep(0.05 - compute_time)

        except Exception as e:
            print(f"发生异常！{e}")

    except KeyboardInterrupt:
        print("\n用户停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        tm.set_synchronous_mode(False)

        if actor_list:
            client.apply_batch([carla.command.DestroyActor(a) for a in actor_list])
        print("清理完成，Carla 已恢复正常。")

if __name__ == '__main__':
    main()
