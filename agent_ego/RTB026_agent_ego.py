import carla
import time
import math
import numpy as np
import random

# =================基础控制算法 (PID 引入横滑安全限速补偿限幅机制) =================
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

class PIDLateralController:
    def __init__(self, K_P=1.0, K_I=0.01, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_waypoint_loc, vehicle_transform):
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)
        target_vector = np.array([target_waypoint_loc.x - v_loc.x, target_waypoint_loc.y - v_loc.y])
        norm = np.linalg.norm(target_vector)
        if norm < 0.1: return 0.0

        target_yaw = math.atan2(target_vector[1], target_vector[0])
        error = target_yaw - v_yaw

        # 使夹角规范在最短侧规转(-pi to pi区间) 防止过度偏移绕转圈的情况
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi

        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)

        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        # 限值给低是因为 CARLA 在 wet 赛道(湿路面)容易直接大幅甩尾(即你所指的滑坡入界情况)
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.5, 0.5)

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

# =================所有要求生成的预先静态轨道路径位置序列=============
TESLA_TRAJECTORY = [(0.11, -42.484, 3.51), (0.11, -42.484, 2.945), (0.11, -42.484, 2.24), (1.98, -42.397, 2.662),
                    (9.59, -42.036, 2.732), (17.207, -41.673, 2.732), (23.702, -41.388, 2.237),
                    (28.076, -41.228, 2.097), (35.694, -40.89, 2.668), (43.429, -40.356, 7.879),
                    (51.004, -38.769, 18.658), (58.12, -35.756, 32.019), (63.751, -31.416, 46.036),
                    (64.856, -29.752, 55.742), (68.227, -22.863, 73.213), (69.451, -17.153, 88.69),
                    (69.45, -17.028, 90.349), (69.382, -10.927, 90.844), (70.622, -3.384, 62.28),
                    (73.046, -0.788, 9.218), (78.013, 0.366, 10.792), (85.467, 0.912, 2.569), (93.171, 1.598, 5.565),
                    (100.686, 1.824, -1.191), (108.143, 1.613, -1.253), (115.793, 1.592, 0.159),
                    (123.535, 1.567, -0.334), (131.268, 1.516, -0.474), (138.766, 1.454, -0.544),
                    (146.268, 1.383, -0.544), (153.847, 1.382, 0.573), (161.395, 1.498, 0.934), (169.018, 1.646, 1.572),
                    (176.784, 1.815, 0.797), (184.533, 1.883, 0.234), (192.033, 1.783, -3.284), (199.64, 1.273, -2.355),
                    (207.133, 0.965, -2.285), (214.756, 0.879, 1.005), (222.504, 1.049, 1.288), (230.128, 1.304, 2.07),
                    (237.875, 1.584, 2.07), (245.497, 1.71, 0.014), (253.12, 1.585, -1.128), (260.74, 1.43, -1.198),
                    (268.362, 1.27, -1.198), (276.111, 1.119, -0.985), (283.735, 0.988, -0.985),
                    (291.484, 0.856, -0.772), (299.233, 0.893, 1.43), (306.854, 1.101, 1.57), (314.6, 1.318, 1.57),
                    (322.095, 1.425, 0.507), (326.092, 1.442, 0.017), (326.092, 1.442, 0.017), (326.092, 1.442, 0.017)]
IMPALA_TRAJECTORY = [
    (187.882, -2.948, 179.005), (187.882, -2.948, 179.005), (187.882, -2.948, 179.005), (187.882, -2.948, 179.005),
    (187.882, -2.948, 179.005), (187.882, -2.948, 179.005), (186.894, -2.931, 179.005), (185.646, -2.909, 178.725),
    (184.380, -2.876, 178.375), (183.124, -2.840, 178.375), (181.875, -2.810, 178.794), (180.612, -2.792, 179.424),
    (179.348, -2.783, 179.914), (178.102, -2.783, -179.946), (176.044, -2.785, -179.946), (172.553, -2.789, -179.946),
    (168.765, -2.792, -179.946), (164.949, -2.791, 179.494), (161.163, -2.754, 179.424), (157.411, -2.733, 179.844),
    (153.599, -2.722, 179.844), (149.849, -2.712, 179.844), (146.099, -2.705, 179.984), (142.286, -2.710, -179.876),
    (138.536, -2.718, -179.876), (134.723, -2.720, 179.914), (130.911, -2.715, 179.914), (127.162, -2.698, 179.704),
    (123.351, -2.679, 179.704), (119.602, -2.659, 179.704), (115.853, -2.634, 179.564), (112.032, -2.605, 179.564),
    (108.282, -2.576, 179.564), (104.469, -2.547, 179.564), (100.718, -2.526, 179.774), (96.905, -2.525, -179.876),
    (93.097, -2.539, -179.666), (90.290, -2.555, -179.736), (90.290, -2.555, -179.736), (90.290, -2.555, -179.736),
    (90.290, -2.555, -179.736), (90.290, -2.555, -179.736), (90.290, -2.555, -179.736), (90.290, -2.555, -179.736),
    (90.290, -2.555, -179.736), (90.290, -2.555, -179.736), (90.290, -2.555, -179.736), (86.552, -2.573, -179.736),
    (84.330, -2.583, -179.736), (83.083, -2.589, -179.736), (82.340, -2.592, -179.736), (81.833, -2.594, -179.736),
    (80.699, -2.599, -179.736), (79.453, -2.605, -179.736), (78.186, -2.611, -179.736), (76.920, -2.580, 173.741),
    (75.699, -2.339, 164.232), (74.518, -1.941, 159.195), (73.339, -1.471, 158.104), (72.182, -1.006, 158.104),
    (70.983, -0.531, 158.664), (69.814, -0.092, 159.924), (68.637, 0.327, 161.113), (67.419, 0.690, 165.639),
    (66.204, 0.980, 167.673), (64.957, 1.226, 170.041), (63.722, 1.419, 172.465), (62.459, 1.558, 174.587),
    (61.213, 1.654, 176.607), (59.945, 1.728, 177.318), (58.675, 1.750, 179.762), (57.425, 1.756, 179.762),
    (56.155, 1.754, -179.608), (54.884, 1.746, -179.608), (53.634, 1.737, -179.608), (52.389, 1.656, -171.182),
    (51.116, 1.434, -169.355), (49.888, 1.203, -169.355), (48.641, 0.959, -168.585), (47.416, 0.707, -168.304),
    (46.192, 0.454, -168.304), (44.947, 0.200, -168.794), (43.700, -0.041, -169.144), (42.472, -0.276, -169.144),
    (41.244, -0.511, -169.144), (39.996, -0.750, -169.214), (38.767, -0.971, -171.138), (37.509, -1.138, -173.067),
    (36.250, -1.284, -173.837), (34.990, -1.420, -173.837), (33.751, -1.551, -174.256), (32.512, -1.672, -174.466),
    (31.253, -1.794, -174.466), (30.015, -1.910, -175.236), (28.757, -1.999, -176.364), (27.500, -2.079, -176.364),
    (26.243, -2.146, -177.834), (25.010, -2.173, -179.102), (23.757, -2.192, -179.102), (22.524, -2.211, -179.242),
    (21.264, -2.224, -179.522), (20.023, -2.234, -179.732), (18.780, -2.239, -179.732), (17.102, -2.247, -179.732),
    (14.613, -2.232, 178.463), (12.078, -2.162, 178.813), (9.582, -2.113, 178.883), (7.084, -2.064, 178.883),
    (3.940, -2.003, 178.883), (0.192, -1.933, 179.163), (-3.683, -1.885, 179.303), (-7.910, -1.834, 179.303),
    (-12.910, -1.781, 179.723), (-17.993, -1.756, 179.723), (-22.994, -1.732, 179.653), (-27.994, -1.701, 179.653),
    (-33.077, -1.671, 179.653), (-38.163, -1.640, 179.653), (-43.247, -1.609, 179.653), (-48.246, -1.579, 179.653),
    (-53.246, -1.546, 179.443), (-58.329, -1.497, 179.443), (-63.413, -1.438, 179.233), (-68.412, -1.365, 179.163),
    (-73.410, -1.281, 178.883), (-78.490, -1.185, 178.953), (-83.486, -1.122, 179.583), (-88.478, -1.092, 179.723)
]
PED_TRAJECTORY = [(53.246, -10.436, 81.453), (53.246, -10.436, 80.375), (53.264, -10.337, 79.594),
                  (53.358, -9.829, 79.594), (53.449, -9.33, 79.594), (53.54, -8.838, 79.594), (53.633, -8.331, 79.737),
                  (53.718, -7.838, 80.586), (53.802, -7.337, 80.586), (53.945, -6.852, 63.138), (54.265, -6.449, 42.52),
                  (54.632, -6.11, 55.069), (54.817, -5.63, 74.696), (54.947, -5.138, 77.524), (55.041, -4.639, 79.557),
                  (55.213, -4.175, 46.94), (55.455, -3.524, 14.37), (55.372, -3.184, 12.665), (55.574, -3.14, 12.24),
                  (55.574, -3.14, 15.387), (55.574, -3.14, 15.387), (55.574, -3.14, 15.387), (55.574, -3.14, 15.387),
                  (55.574, -3.14, 15.387), (55.574, -3.14, 15.387), (55.574, -3.14, 15.387), (55.574, -3.14, 46.681),
                  (55.574, -3.14, 90.761), (55.528, -2.802, 101.42), (55.435, -2.311, 100.341),
                  (55.345, -1.803, 99.419), (55.273, -1.292, 96.13), (55.219, -0.796, 96.13), (55.164, -0.284, 96.13),
                  (55.111, 0.213, 96.13), (55.057, 0.725, 94.684), (55.031, 1.232, 92.453), (54.917, 1.728, 118.116),
                  (54.591, 2.102, 140.275), (54.204, 2.443, 135.206), (53.871, 2.827, 124.104),
                  (53.629, 3.282, 114.316), (53.432, 3.741, 111.516), (53.265, 4.212, 106.095),
                  (53.156, 4.709, 100.419), (53.078, 5.211, 95.908), (53.048, 5.709, 92.466), (53.039, 6.225, 89.861),
                  (53.04, 6.725, 89.861), (53.041, 6.95, 89.861), (53.041, 6.95, 89.861)]

def get_vel_vector_at_yaw(speed_kmh, yaw_degree):
    yaw_rad = math.radians(yaw_degree)
    v_ms = speed_kmh / 3.6
    return carla.Vector3D(x=v_ms * math.cos(yaw_rad), y=v_ms * math.sin(yaw_rad), z=0.0)

# (修复1的关键机制): 用于代替原来固定 index+=1 ，使巡轨算法成为前驱探针，解决刚启动因为偏差产生的 甩尾、跑飞、以及冲界等
def extract_dynamic_lookahead(actor, trajectory, curr_idx, speed_req_ms):
    # 防止因贴身间隙（1米甚至等距散点），转动PID输出疯狂转弯指令！始终拉直往前开一小截(最少确保跟踪距离5米之外平滑的轨迹线段)。
    v_loc = actor.get_location()
    f_vec = actor.get_transform().get_forward_vector()
    lookahead = max(5.0, speed_req_ms * 0.4)

    target_idx = curr_idx
    while target_idx < len(trajectory) - 1:
        tx, ty, _ = trajectory[target_idx]
        t_loc = carla.Location(tx, ty, v_loc.z)
        dist = v_loc.distance(t_loc)

        # 判断坐标是在车前面还是后方（或者过短跟不上处理反应）防止“死亡后甩回追点现象”。如果满足继续遍历找远点：
        vec_front = carla.Vector3D(tx - v_loc.x, ty - v_loc.y, 0)
        dot_projection = (vec_front.x * f_vec.x + vec_front.y * f_vec.y)

        if dot_projection < 0 or dist < lookahead:
            target_idx += 1
        else:
            break

    # 返参目标远看接引点的同时携带 index 同步主逻辑流
    tx, ty, _ = trajectory[target_idx]
    return target_idx, carla.Location(x=tx, y=ty, z=v_loc.z)

# ======================== 主循环场长尾剧本流执行区域 ========================

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    tm = client.get_trafficmanager(8000)

    # =============== 天气设置要求解析 ===============
    weather = carla.WeatherParameters(
        cloudiness=5.0, precipitation=0.0, precipitation_deposits=75.0, wind_intensity=10.0,
        sun_azimuth_angle=-1.0, sun_altitude_angle=15.0, fog_density=2.0, fog_distance=0.75,
        fog_falloff=0.1000, wetness=50.0, scattering_intensity=1.0, mie_scattering_scale=0.0300,
        rayleigh_scattering_scale=0.0331
    )
    world.set_weather(weather)
    dt = 0.05
    actor_list = []

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

        pid_params = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.1, 'dt': dt}

        # ========== 创生生成区域 ==========
        bp_tesla = bp_lib.find('vehicle.tesla.model3')
        if bp_tesla.has_attribute('color'): bp_tesla.set_attribute('color', '0,0,0')  # 规定要求的黑色车
        t_yaw = TESLA_TRAJECTORY[0][2]
        # 使用确定的安全路面映射高度确保下坠平坦 (0.5为中心半车身起悬偏距）
        tz_offset = carla_map.get_waypoint(carla.Location(TESLA_TRAJECTORY[0][0], TESLA_TRAJECTORY[0][1], 0),
                                           project_to_road=True).transform.location.z
        t_loc = carla.Transform(carla.Location(TESLA_TRAJECTORY[0][0], TESLA_TRAJECTORY[0][1], tz_offset + 0.5),
                                carla.Rotation(yaw=t_yaw))
        tesla = world.try_spawn_actor(bp_tesla, t_loc)
        if tesla: actor_list.append(tesla)

        bp_impala = bp_lib.find('vehicle.chevrolet.impala')
        if bp_impala.has_attribute('color'): bp_impala.set_attribute('color', '255,255,255')  # 规定白色
        i_yaw = IMPALA_TRAJECTORY[0][2]
        iz_offset = carla_map.get_waypoint(carla.Location(IMPALA_TRAJECTORY[0][0], IMPALA_TRAJECTORY[0][1], 0),
                                           project_to_road=True).transform.location.z
        i_loc = carla.Transform(carla.Location(IMPALA_TRAJECTORY[0][0], IMPALA_TRAJECTORY[0][1], iz_offset + 0.5),
                                carla.Rotation(yaw=i_yaw))
        impala = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        walker_bpd = bp_lib.filter('walker.pedestrian.*')
        bp_ped = random.choice([bp for bp in walker_bpd if bp.id != 'walker.pedestrian.child'])  # 成年行人选取
        ped_z = carla_map.get_waypoint(
            carla.Location(PED_TRAJECTORY[0][0], PED_TRAJECTORY[0][1], 0)).transform.location.z
        p_loc = carla.Transform(carla.Location(PED_TRAJECTORY[0][0], PED_TRAJECTORY[0][1], ped_z + 0.5),
                                carla.Rotation(yaw=PED_TRAJECTORY[0][2]))
        pedestrian = world.try_spawn_actor(bp_ped, p_loc)
        if pedestrian: actor_list.append(pedestrian)

        # ----------- 第一层重要抗出圈滑动防护 ：使悬挂从空气落下来直至自然贴合（约1秒引擎流跑完全）完全抛离PID起落点失衡偏移 ----
        for _ in range(25):
            world.tick()

        # ----------- 直接注入规定满转初始初速度域(不用等待爬坡时间、避免油门0速失滑！) ----
        if tesla: tesla.set_target_velocity(get_vel_vector_at_yaw(60.0, TESLA_TRAJECTORY[0][2]))

        # 小放两步步让动力系统的初级转矩顺出并抓着轮胎接合并更新实际的初定位不漂：
        for _ in range(3): world.tick()

        # 分立 PID 控制实例化接引
        pid_tesla = {'lon': PIDLongitudinalController(**pid_params), 'lat': PIDLateralController(**pid_params)}
        pid_impala = {'lon': PIDLongitudinalController(**pid_params), 'lat': PIDLateralController(**pid_params)}

        print("\n维稳初构结体完毕！正式带匀常态启跑接控路轨追切..")

        tesla_traj_idx, impala_traj_idx, ped_traj_idx = 0, 0, 0
        ped_pause_timer, ped_state = 0.0, "walking"
        ped_finished = False
        impala_state, impala_stop_start_time = "cruise_50", None
        trash_thrown = False

        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            # ======================== Tesla 控制链：完全规矩切线法 ======================
            if tesla and tesla.is_alive:
                if tesla_traj_idx < len(TESLA_TRAJECTORY) - 1:
                    tesla_target_vel = 45.0  # Tesla 限速全45
                    # 执行 LookAhead (动态看前算法平稳切分轨步): 防止被前排近点回抽跑野转轴失稳冲外路外道。
                    tesla_traj_idx, tgt_loc = extract_dynamic_lookahead(tesla, TESLA_TRAJECTORY, tesla_traj_idx,
                                                                        tesla_target_vel / 3.6)
                    apply_pid_control(tesla, pid_tesla['lon'], pid_tesla['lat'], tesla_target_vel, tgt_loc)
                else:
                    tesla.apply_control(carla.VehicleControl(brake=1.0))

            # ======================== Impala 控制链：附带动能跳挡位流 =====================

            # ======================= Pedestrian 走路、留候断点停格 + 发长尾物理向心投垃圾(参数均值调低) ======
            if pedestrian and pedestrian.is_alive and not ped_finished:
                px, py, pyaw = PED_TRAJECTORY[ped_traj_idx]
                ped_loc = pedestrian.get_location()
                ped_target = carla.Location(px, py, ped_loc.z)

                dist_to_tar = math.sqrt((ped_target.x - ped_loc.x) ** 2 + (ped_target.y - ped_loc.y) ** 2)

                # 行人的特殊时段事件要求在 55.574, -3.14  处切开静息时针等位逻辑：
                if ped_state == "walking" and (55.45 <= px <= 55.65) and (-3.25 <= py <= -3.05):
                    # 判断如果靠近要求坐标触发则启动倒点时停
                    if math.sqrt((ped_loc.x - 55.574) ** 2 + (ped_loc.y - -3.14) ** 2) < 0.65 and not trash_thrown:
                        ped_state = "waiting"
                        ped_pause_timer = sim_time

                if ped_state == "waiting":
                    # 控制步行控制器强制归为定桩态断速度输出保持滞步模型
                    pedestrian.apply_control(carla.WalkerControl(direction=carla.Vector3D(0, 0, 0), speed=0.0))

                    if (sim_time - ped_pause_timer) >= 1.0:  # 留 3秒满足发牌时长后起向！
                        # 执行网格要求：静态引用和投空垃圾件计算触发!
                        bp_mesh = bp_lib.find('static.prop.mesh')
                        bp_mesh.set_attribute('mesh_path',
                                              "StaticMesh'/Game/Carla/Static/Dynamic/Trash/SM_TrasdhBag.SM_TrasdhBag'")
                        bp_mesh.set_attribute('mass', '2.0')

                        p_tf = pedestrian.get_transform()
                        # 高度抛投定位微上置
                        spawn_tr = carla.Transform(p_tf.location + carla.Location(x=-0.2, z=1.8), carla.Rotation())
                        trash_bag = world.try_spawn_actor(bp_mesh, spawn_tr)

                        if trash_bag:
                            trash_bag.set_simulate_physics(True)
                            actor_list.append(trash_bag)
                            # ---- 修复2关键 ----
                            # [根据您的要求已经全系数重调折损降半物理落力幅参数比值, 削水平抛冲与垂直下行以使得不会抛出到远缘地外边界！]
                            vx_target = (68.072 - 54.958) / 1.7  # 以基础常秒比基 估划长算大概7-8的X向原力常冲抛
                            # 将抛出距(横距X偏移冲量)缩短至之前计算值力矩一半 （*0.485折实效向抵半偏距）, 原前侧平偏移量重砍到 (* 0.5)：
                            adjusted_vX = vx_target * 0.485
                            adjusted_vY = p_tf.get_forward_vector().y * 0.5
                            adjusted_vZ = 4.2  # 垂直落空升腾势能亦削对半（原来是大概要求3.8米跳高折出的8.5向量流升重）现定约为高度不足1.5米的软式微侧轻抛发位

                            trash_bag.set_target_velocity(carla.Vector3D(x=adjusted_vX, y=adjusted_vY, z=adjusted_vZ))
                            print(
                                f"| {sim_time:.2f} 长尾件模拟触发: 行李垃圾平侧推抛释放完成,投距下调生效一半距发限比态!! |")

                        ped_state = "resuming"  # 断截出卡控继续行走发卡
                        trash_thrown = True

                elif ped_state in ["walking", "resuming"]:
                    if ped_traj_idx >= len(PED_TRAJECTORY) - 1 and dist_to_tar < 0.6:
                        pedestrian.apply_control(carla.WalkerControl(direction=carla.Vector3D(0, 0, 0), speed=0.0))
                        ped_finished = True
                        continue

                    if dist_to_tar < 0.6 and ped_traj_idx < len(PED_TRAJECTORY) - 1:
                        ped_traj_idx += 1
                        px, py, pyaw = PED_TRAJECTORY[ped_traj_idx]
                        ped_target = carla.Location(px, py, ped_loc.z)

                    p_dir = carla.Vector3D(ped_target.x - ped_loc.x, ped_target.y - ped_loc.y, 0)
                    p_norm = math.sqrt(p_dir.x ** 2 + p_dir.y ** 2)
                    if p_norm > 0: p_dir.x, p_dir.y = p_dir.x / p_norm, p_dir.y / p_norm

                    pedestrian.apply_control(carla.WalkerControl(direction=p_dir, speed=1.5, jump=False))

            # 按同步刻算时率FPS补流帧定帧率
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n捕获中止退出环境。")
    finally:
        print("清理车辆等物件引用解包恢复环境原调制锁帧.....")
        for actor in actor_list:
            if actor and actor.is_alive: actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        if tm: tm.set_synchronous_mode(False)

if __name__ == '__main__':
    main()
