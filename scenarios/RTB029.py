import carla
import time
import math
import numpy as np


# ==========================
# 基础控制算法 (PID)
# ==========================
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
        # 【修复点】将 numpy.float64 强转为 python 原生 float，防止 CARLA 底层 C++ 崩溃
        return float(np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 0.8))


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
        # 【修复点】强转为 python 原生 float
        return float(np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.7, 0.7))


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed_kmh, target_loc):
    target_speed_ms = target_speed_kmh / 3.6
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    current_speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed_ms, current_speed_ms)
    steer_output = pid_lat.run_step(target_loc, tf)

    if abs(steer_output) < 0.02: steer_output = 0.0

    control = carla.VehicleControl()
    # 确保传入的都是原生 float 类型
    control.steer = float(steer_output)
    if throttle_output >= 0.0:
        control.throttle = float(throttle_output)
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = float(abs(throttle_output))

    vehicle.apply_control(control)


def check_and_handle_out_of_bounds(actor, carla_map, tolerance=6.0):
    if actor is None or not actor.is_alive:
        return True
    loc = actor.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    if wp_nearest is None:
        actor.destroy()
        return True
    if wp_nearest.transform.location.distance(loc) > tolerance:
        print(f"[{actor.type_id}] out of bounds, destroyed.")
        actor.destroy()
        return True
    return False


def destroy_all_actors(actors):
    for actor in actors:
        if actor and actor.is_alive:
            actor.destroy()


def find_actor_by_role_name(world, role_name):
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == role_name:
            return actor
    return None


# ==========================
# 辅助函数：过滤连续重复及过近的轨迹点
# ==========================
def remove_duplicates(trajectory, min_dist=0.5):
    clean_traj = []
    for pt in trajectory:
        if not clean_traj:
            clean_traj.append(pt)
        else:
            dist = math.hypot(pt[0] - clean_traj[-1][0], pt[1] - clean_traj[-1][1])
            if dist >= min_dist:
                clean_traj.append(pt)
    return clean_traj


# ==========================
# 轨迹数据定义
# ==========================
RAW_V1_TRAJECTORY = [
    (-15.402, 153.973, -87.961), (-15.034, 148.737, -85.07), (-15.034, 148.737, -85.07), (-14.345, 141.52, -84.359),
    (-13.593, 133.805, -84.869), (-12.911, 126.211, -84.869), (-12.232, 118.491, -85.009), (-11.588, 110.768, -85.429),
    (-11.018, 103.541, -85.499), (-11.008, 103.416, -85.569), (-10.709, 99.553, -85.569), (-10.709, 99.553, -85.569),
    (-10.709, 99.553, -85.569), (-10.709, 99.553, -85.569), (-10.709, 99.553, -85.569), (-10.709, 99.553, -85.569),
    (-10.709, 99.553, -85.639), (-10.709, 99.553, -85.709), (-10.709, 99.553, -85.849), (-10.709, 99.553, -86.901),
    (-10.709, 99.553, -86.901), (-10.709, 99.553, -86.901), (-10.709, 99.553, -86.901), (-10.709, 99.553, -86.901),
    (-10.709, 99.553, -86.971), (-10.445, 94.57, -86.971), (-9.574, 87.132, -77.012), (-8.031, 83.451, -59.876),
    (-2.812, 78.235, -26.907), (4.649, 76.331, -1.95), (12.395, 76.502, 3.533), (19.979, 77.279, 7.069),
    (27.423, 78.199, 6.429), (34.885, 78.947, 5.605), (42.6, 79.658, 4.902), (50.068, 80.288, 4.76),
    (57.782, 80.982, 6.198), (65.473, 81.874, 6.836), (72.914, 82.766, 6.693), (80.615, 83.584, 5.403),
    (88.204, 84.273, 5.186), (95.668, 85.008, 6.106), (103.373, 85.853, 6.598), (111.069, 86.772, 6.813),
    (118.775, 87.595, 5.608), (126.372, 88.231, 4.476), (133.964, 88.911, 5.771), (141.425, 89.665, 5.771),
    (142.544, 89.778, 5.771), (142.544, 89.778, 5.771)
]

RAW_PED_TRAJECTORY = [
    (-6.295, 88.401, -174.839), (-6.295, 88.401, -174.839), (-6.295, 88.401, -174.839), (-6.295, 88.401, -175.766),
    (-6.295, 88.401, -175.766), (-7.499, 88.312, -175.696), (-8.785, 88.192, -174.063), (-10.069, 88.055, -173.713),
    (-11.349, 87.888, -170.856), (-12.601, 87.674, -170.142), (-13.873, 87.453, -170.142), (-15.104, 87.244, -170.779),
    (-16.338, 87.049, -171.203), (-17.615, 86.855, -171.627), (-18.894, 86.679, -172.475), (-20.134, 86.523, -172.967),
    (-21.395, 86.367, -172.967), (-22.678, 86.225, -174.318), (-23.922, 86.102, -174.318), (-25.187, 85.978, -174.53),
    (-26.452, 85.858, -174.384), (-27.682, 85.657, -165.799), (-28.849, 85.226, -149.044), (-29.903, 84.521, -154.57),
    (-31.12, 84.287, -177.646), (-32.37, 84.293, 179.179), (-33.619, 84.303, 179.815), (-34.911, 84.308, 179.815),
    (-36.202, 84.312, 179.955), (-37.452, 84.308, -179.692), (-38.722, 84.29, -179.125), (-39.347, 84.28, -179.125),
    (-39.347, 84.28, -179.125), (-39.347, 84.28, -179.125)
]

RAW_EGO_TRAJECTORY = [
    (-16.596, 176.14, -86.172), (-16.596, 176.14, -86.102), (-16.596, 176.14, -85.252),
    (-16.24, 172.376, -84.304), (-15.25, 162.326, -84.745), (-14.324, 152.205, -84.815),
    (-13.638, 142.271, -86.591), (-13.226, 135.355, -86.591), (-12.751, 127.597, -85.267),
    (-12.062, 119.497, -85.477), (-11.582, 113.363, -85.477), (-11.313, 103.227, -94.055),
    (-12.224, 93.115, -90.164), (-11.694, 86.647, -80.169), (-7.702, 77.159, -68.148),
    (-4.603, 67.307, -73.467), (-2.728, 57.516, -86.579), (-2.252, 47.195, -86.529),
    (-1.437, 37.236, -84.934), (-0.455, 27.118, -84.364), (0.446, 17.158, -84.934),
    (1.309, 7.196, -85.289), (2.133, -2.936, -85.359), (2.98, -13.234, -85.004),
    (3.85, -23.195, -85.004), (4.721, -33.155, -85.004)
]
# 自动清洗数据
V1_TRAJ = remove_duplicates(RAW_V1_TRAJECTORY)
PED_TRAJ = remove_duplicates(RAW_PED_TRAJECTORY)
EGO_TRAJ = remove_duplicates(RAW_EGO_TRAJECTORY)


# ==========================
# 主程序
# ==========================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # Traffic Manager 设定
    tm = client.get_trafficmanager(8000)
    tm_port = tm.get_port()

    weather = carla.WeatherParameters(
        cloudiness=30.0, precipitation=0.0, precipitation_deposits=0.0,
        wind_intensity=35.0, sun_azimuth_angle=124.0, sun_altitude_angle=15.0,
        fog_density=2.0, fog_distance=0.75, fog_falloff=0.1000, wetness=0.0,
        scattering_intensity=1.0, mie_scattering_scale=0.1300, rayleigh_scattering_scale=0.0831
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    # 状态标志与控制器
    v1_active = False
    ego_active = False
    ped_active = False
    v3_active = False

    pid_v1 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
    pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

        # ================= Actor 1：V1车 (Seat Leon) =================
        bp_v1 = bp_lib.find('vehicle.seat.leon')
        v1_sx, v1_sy, v1_syaw = V1_TRAJ[0]
        v1_loc = carla.Location(x=v1_sx, y=v1_sy, z=0.5)
        v1_loc.z = carla_map.get_waypoint(v1_loc).transform.location.z + 0.5
        v1 = world.try_spawn_actor(bp_v1, carla.Transform(v1_loc, carla.Rotation(yaw=v1_syaw)))
        if v1:
            actor_list.append(v1)
            v1_active = True
            print("生成 V1 (Seat Leon) 成功。")

        # ================= Actor 2：Ego车 (BMW Grandtourer, 蓝色) =================
        bp_ego = bp_lib.find('vehicle.bmw.grandtourer')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '111,111,255')
        if bp_ego.has_attribute('role_name'):
            bp_ego.set_attribute('role_name', 'ego')
        ego_sx, ego_sy, ego_syaw = EGO_TRAJ[0]
        ego_loc = carla.Location(x=ego_sx, y=ego_sy, z=0.5)
        ego_loc.z = carla_map.get_waypoint(ego_loc).transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego, carla.Transform(ego_loc, carla.Rotation(yaw=ego_syaw)))
        if ego:
            actor_list.append(ego)
            ego_active = True
            print("生成 Ego (BMW) 成功。")

            # ================= Actor 3：V3 (Nissan Micra, TM 控制) =================
            bp_v3 = bp_lib.find('vehicle.nissan.micra')
            v3_loc = carla.Location(x=62.142, y=70.357, z=0.5)
            v3_wp = carla_map.get_waypoint(v3_loc)
            v3_loc.z = v3_wp.transform.location.z + 0.5
            v3 = world.try_spawn_actor(bp_v3, carla.Transform(v3_loc, v3_wp.transform.rotation))
            if v3:
                actor_list.append(v3)
                v3.set_autopilot(True, tm_port)
                # 设为0%代表开启避障（不要无视行人车辆）。设为100%是撞上去！
                tm.ignore_vehicles_percentage(v3, 0)
                tm.ignore_walkers_percentage(v3, 0)
                tm.distance_to_leading_vehicle(v3, 5.0)
                tm.vehicle_percentage_speed_difference(v3, -60.0)
                v3_active = True

                # 【新增】记录生成时的偏航角(yaw)，为后续赋予初速度做准备
                v3_syaw = v3_wp.transform.rotation.yaw
                print("生成 V3 (Nissan Micra, TM接管) 成功。")

        # ================= Actor 4：行人 =================
        bp_ped = bp_lib.filter('walker.pedestrian.*')[0]
        if bp_ped.has_attribute('is_invincible'):
            bp_ped.set_attribute('is_invincible', 'true')
        ped_sx, ped_sy, ped_syaw = PED_TRAJ[0]
        ped_loc = carla.Location(x=ped_sx, y=ped_sy, z=1.0)
        ped = world.try_spawn_actor(bp_ped, carla.Transform(ped_loc, carla.Rotation(yaw=ped_syaw)))
        if ped:
            actor_list.append(ped)
            ped_active = True
            print("生成 行人 成功。")

        # 预热20帧
        for _ in range(20):
            world.tick()

        # 赋予初始物理速度
        if v1_active:
            v1_speed_ms = 70.0 / 3.6
            v1.set_target_velocity(carla.Vector3D(
                v1_speed_ms * math.cos(math.radians(v1_syaw)),
                v1_speed_ms * math.sin(math.radians(v1_syaw)), 0.0))

        if ego_active:
            ego_speed_ms = 70.0 / 3.6
            ego.set_target_velocity(carla.Vector3D(
                ego_speed_ms * math.cos(math.radians(ego_syaw)),
                ego_speed_ms * math.sin(math.radians(ego_syaw)), 0.0))

        # ================== 【新增对V3的初速度设定】 ==================
        if v3_active:
            v3_speed_ms = 100.0 / 3.6  # 初始速度 100 km/h
            v3.set_target_velocity(carla.Vector3D(
                v3_speed_ms * math.cos(math.radians(v3_syaw)),
                v3_speed_ms * math.sin(math.radians(v3_syaw)), 0.0))
        # ==========================================================

        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds

        v1_traj_idx = 0
        v1_state = "NORMAL"
        v1_brake_start_time = 0.0

        ego_traj_idx = 0
        ped_traj_idx = 0

        print("\n仿真正式开始！")
        # 将视角绑定到 Ego 车后方以便观察
        spectator = world.get_spectator()
        while True:
            start_time = time.time()
            world.tick()
            current_snap_time = world.get_snapshot().timestamp.elapsed_seconds
            sim_time = current_snap_time - start_sim_time
            # ---------------- 视角跟随 ----------------
            spectator_ego = find_actor_by_role_name(world, 'ego')
            if spectator_ego and spectator_ego.is_alive:
                tf = spectator_ego.get_transform()
                spectator.set_transform(carla.Transform(
                    tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
                    carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
                ))
            # ==========================
            # 行人控制
            # ==========================
            if ped_active and ped.is_alive:
                if sim_time < 2.0:
                    ped.apply_control(carla.WalkerControl(speed=0.0))
                elif ped_traj_idx < len(PED_TRAJ):
                    tx, ty, tyaw = PED_TRAJ[ped_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=ped.get_location().z)

                    dist_to_target = ped.get_location().distance(target_loc)
                    if dist_to_target < 0.5 and ped_traj_idx < len(PED_TRAJ) - 1:
                        ped_traj_idx += 1

                    direction_vector = np.array([tx - ped.get_location().x, ty - ped.get_location().y])
                    norm = np.linalg.norm(direction_vector)

                    if norm > 0.0:
                        # 【修复点】强转为 python native float
                        dir_x = float(direction_vector[0] / norm)
                        dir_y = float(direction_vector[1] / norm)
                        direction = carla.Vector3D(dir_x, dir_y, 0.0)

                        ped_vel = ped.get_velocity()
                        # 【修复点】强转为 python native bool
                        is_stuck = bool(math.hypot(ped_vel.x, ped_vel.y) < 0.2 and norm > 1.0)

                        ped.apply_control(carla.WalkerControl(direction=direction, speed=3.5, jump=is_stuck))
                else:
                    ped.apply_control(carla.WalkerControl(speed=0.0))
                    ped_active = False

            # ==========================
            # V1 (Seat Leon) 逻辑控制
            # ==========================
            if v1_active and v1.is_alive:
                if check_and_handle_out_of_bounds(v1, carla_map, tolerance=8.0):
                    v1_active = False
                elif v1_traj_idx < len(V1_TRAJ):
                    tx, ty, tyaw = V1_TRAJ[v1_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v1.get_location().z)

                    if v1.get_location().distance(target_loc) < 3.5 and v1_traj_idx < len(V1_TRAJ) - 1:
                        v1_traj_idx += 1

                    current_y = v1.get_location().y

                    if v1_state == "NORMAL":
                        if current_y <= 110.0:
                            v1_state = "BRAKING"
                            v1_brake_start_time = sim_time
                            print(f"[事件] V1 在 y={current_y:.1f} 处执行紧急制动！")
                            v1.apply_control(carla.VehicleControl(brake=1.0, steer=0.0))
                        else:
                            apply_pid_control(v1, pid_v1['lon'], pid_v1['lat'], 70.0, target_loc)

                    elif v1_state == "BRAKING":
                        v1.apply_control(carla.VehicleControl(brake=1.0, steer=0.0))
                        if sim_time - v1_brake_start_time >= 3.0:
                            v1_state = "RECOVERING"
                            print("[事件] V1 制动结束，重新起步恢复至 60 km/h。")

                    elif v1_state == "RECOVERING":
                        apply_pid_control(v1, pid_v1['lon'], pid_v1['lat'], 60.0, target_loc)
                else:
                    v1.apply_control(carla.VehicleControl(brake=1.0))
                    v1_active = False

            # ==========================
            # Ego 车控制
            # ==========================
            ego_terminal = False
            if ego_active and ego and ego.is_alive:
                if check_and_handle_out_of_bounds(ego, carla_map):
                    ego_active = False
                    ego_terminal = True
                elif ego_traj_idx < len(EGO_TRAJ):
                    tx, ty, tyaw = EGO_TRAJ[ego_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=ego.get_location().z)

                    if ego.get_location().distance(target_loc) < 3.5 and ego_traj_idx < len(EGO_TRAJ) - 1:
                        ego_traj_idx += 1
                        tx, ty, tyaw = EGO_TRAJ[ego_traj_idx]
                        target_loc = carla.Location(x=tx, y=ty, z=ego.get_location().z)

                    if ego_traj_idx >= len(EGO_TRAJ) - 1 and ego.get_location().distance(target_loc) < 3.5:
                        ego.apply_control(carla.VehicleControl(brake=1.0))
                        ego_active = False
                        ego_terminal = True
                    else:
                        apply_pid_control(ego, pid_ego['lon'], pid_ego['lat'], 70.0, target_loc)
                else:
                    ego.apply_control(carla.VehicleControl(brake=1.0))
                    ego_active = False
                    ego_terminal = True

            # 结束判断
            elif ego_active:
                ego_active = False
                ego_terminal = True

            if ego_terminal:
                print("Ego reached the end or was destroyed. Destroying all actors and ending simulation.")
                destroy_all_actors(actor_list)
                break

            if not v1_active and not ego_active and not ped_active:
                print("主体车辆及行人已完成剧本，仿真结束。")
                break

            # 帧率同步控制
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    except Exception as e:
        print(f"\n捕获到异常: {e}")
    finally:
        print("\n清理环境并恢复异步设置...")
        destroy_all_actors(actor_list)

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        tm.set_synchronous_mode(False)
        print("清理完毕。")


if __name__ == '__main__':
    main()
