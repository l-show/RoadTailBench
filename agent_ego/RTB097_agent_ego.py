import carla
import time
import math
import numpy as np

# ================= 基础控制算法 (PID) =================
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

# ================= 辅助工具函数 =================
def check_and_handle_out_of_bounds(actor, carla_map):
    loc = actor.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    if wp_nearest is None: return True
    distance = wp_nearest.transform.location.distance(loc)
    if distance > 6.0: return True
    return False

def remove_duplicate_points(trajectory, threshold=0.1):
    """去除轨迹中的重复或距离极近的点，防止循迹瞬间跳跃计算"""
    if not trajectory: return []
    cleaned_traj = [trajectory[0]]
    for pt in trajectory[1:]:
        last_pt = cleaned_traj[-1]
        dist = math.hypot(pt[0] - last_pt[0], pt[1] - last_pt[1])
        if dist > threshold:
            cleaned_traj.append(pt)
    return cleaned_traj

def set_initial_velocity(actor, speed_kmh, yaw_deg):
    """根据航向角赋予物理初速度向量，防止车辆从0加速"""
    speed_ms = speed_kmh / 3.6
    yaw_rad = math.radians(yaw_deg)
    actor.set_target_velocity(carla.Vector3D(
        speed_ms * math.cos(yaw_rad),
        speed_ms * math.sin(yaw_rad),
        0.0))

def calculate_velocity_vector(magnitude, rotation):
    """计算冲量/速度的3D向量 (传入 magnitude 大小和 carla.Rotation 旋转角度)"""
    rad = math.radians(rotation.yaw)
    return carla.Vector3D(magnitude * math.cos(rad), magnitude * math.sin(rad), 0)

def cleanup_actors(client, actors):
    alive_actors = [actor for actor in actors if actor and actor.is_alive]
    if not alive_actors:
        return
    try:
        client.apply_batch([carla.command.DestroyActor(actor.id) for actor in alive_actors])
    except Exception:
        for actor in alive_actors:
            try:
                if actor and actor.is_alive:
                    actor.destroy()
            except Exception:
                pass

def ego_reached_goal(ego, goal_xy, radius=5.0):
    if not ego or not ego.is_alive:
        return False
    loc = ego.get_location()
    return math.hypot(loc.x - goal_xy[0], loc.y - goal_xy[1]) <= radius

# ================= 原始轨迹数据 =================
RAW_SPRINTER_TRAJECTORY = [
    (177.508, -1.476, -178), (177.508, -1.476, -178), (177.508, -1.476, -178), (177.508, -1.476, -178.14),
    (174.45, -1.593, -177.719), (170.702, -1.67, -179.648), (166.952, -1.693, -179.648), (163.202, -1.706, -179.861),
    (159.327, -1.716, -179.718), (155.452, -1.731, -179.861), (151.641, -1.74, -179.861), (147.829, -1.75, -179.861),
    (144.018, -1.759, -179.861), (140.207, -1.709, 178.41), (136.458, -1.6, 178.34), (132.709, -1.517, 179.336),
    (128.897, -1.499, 179.764), (125.147, -1.483, 179.764), (121.334, -1.468, 179.764), (117.584, -1.462, -179.956),
    (113.71, -1.491, -179.178), (109.835, -1.554, -179.035), (106.023, -1.627, -178.825), (102.212, -1.708, -178.755),
    (98.463, -1.79, -178.755), (94.651, -1.867, -178.895), (90.775, -1.931, -179.108), (87.016, -1.98, -179.463),
    (83.204, -2, 179.977), (79.329, -1.993, 179.837), (75.579, -1.955, 179.059), (71.83, -1.882, 178.639),
    (68.082, -1.77, 178.146), (64.208, -1.652, 178.572), (60.334, -1.576, 179.131), (56.585, -1.535, 179.411),
    (52.835, -1.512, 179.761), (49.085, -1.496, 179.619), (45.211, -1.471, 179.758), (41.337, -1.464, 179.901),
    (37.588, -1.479, -179.536), (33.775, -1.51, -179.536), (29.898, -1.541, -179.536), (26.023, -1.554, 179.829),
    (22.274, -1.525, 178.972), (18.462, -1.468, 179.538), (14.586, -1.437, 179.538), (10.836, -1.407, 179.538),
    (6.961, -1.387, 179.818), (3.149, -1.375, 179.818), (-0.726, -1.363, 179.818), (-4.601, -1.355, -179.973),
    (-8.352, -1.357, -179.973), (-12.227, -1.359, -179.973), (-16.104, -1.361, -179.973), (-19.979, -1.358, 179.885),
    (-23.792, -1.35, 179.885), (-27.545, -1.342, 179.955), (-31.295, -1.352, -179.693), (-35.17, -1.385, -178.325),
    (-39.029, -1.706, -171.934), (-42.708, -2.423, -165.061), (-46.25, -3.647, -156.524), (-49.663, -5.471, -146.713),
    (-52.737, -7.826, -138.508), (-55.364, -10.497, -130.947), (-57.594, -13.507, -122.113), (-59.382, -16.94, -112.55),
    (-60.523, -20.572, -101.971), (-61.045, -24.283, -94.674), (-61.132, -25.404, -94.383), (-61.132, -25.404, -94.383)
]

RAW_CHILD_TRAJECTORY = [
    (81.173, 2.029, 0.209), (86.338, 2.047, 0.066), (91.42, 2.022, -0.359), (96.586, 1.99, -0.359),
    (101.751, 1.958, -0.359), (106.833, 1.927, -0.289), (112, 1.94, 0.914), (116.999, 2.02, 0.914),
    (120.749, 2.079, 0.914), (120.749, 2.079, 0.914), (122.582, 2.109, 0.914), (123.748, 2.127, 0.914)
]

RAW_EGO_TRAJECTORY = [
    (111.577, 72.542, -91.506), (111.577, 72.542, -91.506), (111.564, 72.042, -91.506), (111.437, 66.96, -91.223),
    (111.35, 61.795, -90.656), (111.307, 56.797, -90.233), (111.334, 51.638, -89.524), (111.373, 46.57, -89.593),
    (111.427, 41.513, -89.24), (111.487, 36.459, -89.59), (111.479, 31.307, -91.088), (111.373, 26.314, -91.228),
    (111.264, 21.238, -91.228), (111.156, 16.244, -91.508), (110.925, 11.17, -94.295), (109.8, 6.158, -112.407),
    (107.095, 2.009, -138.074), (102.695, -0.652, -157.86), (97.809, -1.531, -177.747), (92.647, -1.682, -179.472),
    (87.481, -1.696, -179.968), (82.398, -1.696, 179.822), (77.398, -1.665, 179.539), (72.231, -1.635, 179.822),
    (67.065, -1.646, -179.536), (62.065, -1.688, -179.323), (56.898, -1.755, -179.253), (51.814, -1.821, -179.253),
    (46.645, -1.883, -179.679), (41.478, -1.888, 179.758), (36.312, -1.865, 179.545), (31.281, -1.799, 179.189),
    (26.195, -1.727, 179.189), (21.112, -1.655, 179.189)
]

SPRINTER_TRAJECTORY = remove_duplicate_points(RAW_SPRINTER_TRAJECTORY)
CHILD_TRAJECTORY = remove_duplicate_points(RAW_CHILD_TRAJECTORY)
EGO_TRAJECTORY = remove_duplicate_points(RAW_EGO_TRAJECTORY)

# ================= 主程序 =================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 天气设置
    weather = carla.WeatherParameters(
        cloudiness=40.0, precipitation=100.0, precipitation_deposits=100.0,
        wind_intensity=100.0, sun_azimuth_angle=90.0, sun_altitude_angle=14.0,
        fog_density=2.0, fog_distance=0.0, fog_falloff=0.0, wetness=100.0,
        scattering_intensity=3.5, mie_scattering_scale=0.2000, rayleigh_scattering_scale=0.1200, dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_sprinter = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= Actor 1: Sprinter =================
        bp_sprinter = bp_lib.find('vehicle.mercedes.sprinter')
        sp_x, sp_y, sp_yaw = SPRINTER_TRAJECTORY[0]
        sp_wp = carla_map.get_waypoint(carla.Location(x=sp_x, y=sp_y, z=0))
        sp_loc = carla.Location(x=sp_x, y=sp_y, z=sp_wp.transform.location.z + 1.0)
        sprinter = world.try_spawn_actor(bp_sprinter, carla.Transform(sp_loc, carla.Rotation(yaw=sp_yaw)))
        if sprinter: actor_list.append(sprinter)

        # ================= Actor 2: 小孩 (改用标准行人蓝图) =================
        ch_x, ch_y, ch_yaw = CHILD_TRAJECTORY[0]
        # 获取真实地面高度，防止生成在地底下
        ch_wp = carla_map.get_waypoint(carla.Location(x=ch_x, y=ch_y, z=0), project_to_road=True,
                                       lane_type=carla.LaneType.Any)
        ground_z = ch_wp.transform.location.z if ch_wp else 1.0

        # 使用标准的行人蓝图 (0012 或 0015 通常是小孩/青少年模型)
        bp_child = bp_lib.find('walker.pedestrian.0012')
        if bp_child.has_attribute('is_invincible'):
            bp_child.set_attribute('is_invincible', 'false')

        trans_child = carla.Transform(carla.Location(x=ch_x, y=ch_y, z=ground_z + 1.5), carla.Rotation(yaw=ch_yaw))
        child = world.try_spawn_actor(bp_child, trans_child)
        if child:
            actor_list.append(child)
            print("小孩 生成成功！")
        else:
            print("警告: 小孩生成失败，请检查坐标是否有障碍物。")

        # ================= Actor 3: 物理滚动足球 =================
        bp_football = bp_lib.find('static.prop.mesh')
        bp_football.set_attribute('mesh_path', '/Game/Carla/Static/RoadTailModel/soccerball_Solid.soccerball_Solid')
        bp_football.set_attribute('mass', '50')  # 保持 50kg
        bp_football.set_attribute('scale', '0.2')

        # 足球生成在小孩运动方向前方 1 米处，且Z轴稍微抬高让它掉落，防止卡地
        ball_x = ch_x + 1.0 * math.cos(math.radians(ch_yaw))
        ball_y = ch_y + 1.0 * math.sin(math.radians(ch_yaw))
        trans_football = carla.Transform(carla.Location(x=ball_x, y=ball_y, z=ground_z + 1.0))
        football = world.try_spawn_actor(bp_football, trans_football)

        # 如果没有对应资源模型，备用生成一个小方块测试物理
        if not football:
            print("警告: 找不到自定义足球模型，改用普通物理箱子代替。")
            bp_fallback = bp_lib.find('static.prop.box01')
            bp_fallback.set_attribute('mass', '100')
            football = world.try_spawn_actor(bp_fallback, trans_football)

        if football:
            actor_list.append(football)
            football.set_simulate_physics(True)
            try:
                # 增大摩擦阻尼，让球被踢飞后能较快停下，等待小孩追上来
                football.set_linear_damping(1.5)
                football.set_angular_damping(1.5)
            except:
                pass
            print("足球 生成成功，已开启物理模拟！")

        # ================= Actor 4: Ego Vehicle =================
        bp_ego = bp_lib.find('vehicle.audi.tt')
        if bp_ego.has_attribute('role_name'):
            pass
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '0,0,255')
        ego_x, ego_y, ego_yaw = EGO_TRAJECTORY[0]
        ego_wp = carla_map.get_waypoint(carla.Location(x=ego_x, y=ego_y, z=0))
        ego_loc = carla.Location(x=ego_x, y=ego_y, z=ego_wp.transform.location.z + 1.0)
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        # ---------------- 物理预热与稳定 ----------------
        print("\n车辆与模型生成完毕。等待物理系统稳定(让人物和球落地)...")
        for _ in range(20):  # Tick 20帧让球和人稳稳落地
            world.tick()

        if sprinter: set_initial_velocity(sprinter, 60.0, sp_yaw)

        # 如果足球存在，起步给它一个初始冲量
        if football:
            force_vector = calculate_velocity_vector(150.0, carla.Rotation(yaw=ch_yaw))
            football.add_impulse(force_vector)

        sp_idx, ch_idx, ego_idx = 0, 0, 0
        sprinter_state = 'cruising'
        sp_brake_timer = 0.0
        ego_goal_xy = (EGO_TRAJECTORY[-1][0], EGO_TRAJECTORY[-1][1])
        ego_goal_hits = 0

        print("\n仿真正式开始！")
        while True:
            start_time = time.time()
            world.tick()

            # ================= 1. Sprinter 货车逻辑 =================
            if sprinter and sprinter.is_alive:
                # (此处保留你原有的 Sprinter 控制逻辑)
                sp_loc = sprinter.get_location()
                if check_and_handle_out_of_bounds(sprinter, carla_map):
                    sprinter.destroy()
                elif sp_idx < len(SPRINTER_TRAJECTORY):
                    tx, ty, tyaw = SPRINTER_TRAJECTORY[sp_idx]
                    target_loc = carla.Location(tx, ty, sp_loc.z)
                    if sp_loc.distance(target_loc) < 3.5 and sp_idx < len(SPRINTER_TRAJECTORY) - 1: sp_idx += 1
                    sp_vel = sprinter.get_velocity()
                    current_sp_speed = 3.6 * math.sqrt(sp_vel.x ** 2 + sp_vel.y ** 2)
                    target_speed = 60.0
                    if sprinter_state == 'cruising' and sp_loc.x <= 125.0:
                        sprinter_state = 'braking'
                    elif sprinter_state == 'braking':
                        target_speed = 30.0
                        if current_sp_speed <= 32.0:
                            sprinter_state = 'waiting'
                            sp_brake_timer = time.time()
                    elif sprinter_state == 'waiting':
                        target_speed = 30.0
                        if time.time() - sp_brake_timer > 1.0: sprinter_state = 'recovering'
                    elif sprinter_state == 'recovering':
                        target_speed = 60.0
                    apply_pid_control(sprinter, pid_sprinter['lon'], pid_sprinter['lat'], target_speed, target_loc)
                else:
                    sprinter.apply_control(carla.VehicleControl(brake=1.0))

            # ================= 2. 小孩追赶足球逻辑 (核心修改区) =================
            child_is_running = False
            if child and child.is_alive:
                c_loc = child.get_location()
                if ch_idx < len(CHILD_TRAJECTORY):
                    tx, ty, tyaw = CHILD_TRAJECTORY[ch_idx]
                    target_loc = carla.Location(tx, ty, c_loc.z)

                    if c_loc.distance(target_loc) < 0.5 and ch_idx < len(CHILD_TRAJECTORY) - 1:
                        ch_idx += 1

                    # 使用官方的 WalkerControl 来控制行人
                    dir_vec = carla.Vector3D(tx - c_loc.x, ty - c_loc.y, 0)
                    norm = math.sqrt(dir_vec.x ** 2 + dir_vec.y ** 2)
                    if norm > 0:
                        walker_ctrl = carla.WalkerControl()
                        # 设置移动方向
                        walker_ctrl.direction = carla.Vector3D(dir_vec.x / norm, dir_vec.y / norm, 0)
                        # 设置跑步速度 5m/s
                        walker_ctrl.speed = 5
                        child.apply_control(walker_ctrl)
                        child_is_running = True
                else:
                    # 抵达终点，让小孩停下
                    child.apply_control(carla.WalkerControl(speed=0.0))

            # ================= 3. 足球动态被踢逻辑 =================
            if football and football.is_alive and child and child.is_alive:
                if child_is_running:
                    # 计算小孩和足球的距离
                    dist_to_ball = child.get_location().distance(football.get_location())

                    # 设定：当小孩靠近足球 < 1.2米时，模拟“踢一脚”
                    if dist_to_ball < 1.2:
                        # 获取小孩的前进方向作为踢球方向
                        forward_vec = child.get_transform().get_forward_vector()

                        # 足球质量是50kg，为了让它瞬间加速到约6m/s（比小孩4.5m/s快，从而保持在前方）
                        # 冲量 I = m * Δv = 50 * 6 = 300
                        kick_impulse = carla.Vector3D(forward_vec.x * 350.0, forward_vec.y * 350.0, 0.0)
                        football.add_impulse(kick_impulse)
                        # 结果：球被踢出 -> 受阻尼减速 -> 小孩继续跑追上 -> 再次触发踢球，形成持续带球的画面！

            # ================= 4. Ego 车 (Audi TT) 逻辑 =================

                    # 同步帧率计算
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        for actor in actor_list:
            if actor and actor.is_alive:
                actor.destroy()
        print("清理完毕。")

if __name__ == '__main__':
    main()
