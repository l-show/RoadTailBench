import carla
import time
import math
import numpy as np


# ==========================================
# 1. 基础控制算法 (PID)
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
        # 💡 优化：将油门上限从 0.6 提高到 0.85，确保加速到 70km/h 时有足够动力
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.8, 0.85)


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


# ==========================================
# 2. 辅助函数：车辆出界检测与初速度赋予
# ==========================================
def check_and_handle_out_of_bounds(actor, carla_map):
    loc = actor.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    wp_exact = carla_map.get_waypoint(loc, project_to_road=False)

    is_out = False
    if wp_exact is None:
        is_out = True
    elif wp_nearest and wp_nearest.transform.location.distance(loc) > 4.0:
        is_out = True

    if is_out:
        actor.destroy()
        return True
    return False


def set_initial_velocity(vehicle, speed_kmh):
    """根据车辆当前朝向赋予一个初始物理速度，避免从0起步"""
    if not vehicle or not vehicle.is_alive:
        return
    speed_ms = speed_kmh / 3.6
    yaw = math.radians(vehicle.get_transform().rotation.yaw)
    vx = speed_ms * math.cos(yaw)
    vy = speed_ms * math.sin(yaw)
    # 强制设置目标速度向量
    vehicle.set_target_velocity(carla.Vector3D(x=vx, y=vy, z=0.0))


# ==========================================
# 3. 轨迹数据录入
# ==========================================
TRAJ_EGO = [(59.791, -48.223, 147.768), (56.406, -46.089, 147.767), (56.406, -46.089, 147.767),
            (56.406, -46.089, 147.767), (56.371, -46.067, 147.767), (54.221, -44.711, 147.767),
            (52.072, -43.355, 147.626), (49.89, -41.972, 147.556), (47.747, -40.605, 147.346),
            (45.572, -39.21, 147.206), (43.438, -37.83, 146.996), (41.341, -36.468, 146.996), (39.175, -35.06, 146.996),
            (37.078, -33.699, 146.996), (34.98, -32.339, 147.276), (32.806, -30.943, 147.345),
            (30.702, -29.594, 147.345), (28.596, -28.247, 147.695), (26.483, -26.911, 147.695),
            (24.299, -25.53, 147.695), (22.151, -24.172, 147.695), (19.968, -22.792, 147.695),
            (17.86, -21.448, 147.135), (15.695, -20.038, 146.925), (13.53, -18.629, 146.925),
            (11.401, -17.242, 146.925), (9.236, -15.832, 146.995), (7.037, -14.478, 149.676), (4.846, -13.275, 153.783),
            (2.536, -12.323, 159.797), (0.085, -11.512, 165.226), (-2.419, -11.017, 173.935),
            (-4.913, -10.847, 177.534), (-7.453, -10.778, 179.236), (-9.953, -10.764, -178.806),
            (-12.449, -10.887, -175.643), (-15.017, -11.168, -171.779), (-17.487, -11.549, -170.426),
            (-19.944, -12.01, -168.71), (-22.473, -12.539, -167.929), (-24.918, -13.062, -167.929),
            (-27.446, -13.591, -170.505), (-30.022, -13.725, 179.936), (-32.564, -13.736, -178.143),
            (-35.138, -13.948, -173.256), (-37.609, -14.326, -170.007), (-40.068, -14.783, -168.595),
            (-42.51, -15.32, -166.16), (-45.003, -15.994, -164.009), (-47.482, -16.735, -162.736),
            (-49.869, -17.477, -162.736), (-52.296, -18.231, -162.736), (-54.763, -18.998, -162.736),
            (-57.15, -19.742, -162.666), (-59.537, -20.487, -162.736), (-61.966, -21.236, -162.876),
            (-64.355, -21.972, -162.875), (-66.784, -22.72, -162.875), (-69.173, -23.455, -163.085),
            (-71.647, -24.187, -163.717), (-74.043, -24.887, -163.717), (-76.518, -25.61, -163.717),
            (-78.993, -26.333, -163.717), (-81.391, -27.028, -164.136), (-83.839, -27.711, -164.557),
            (-86.329, -28.396, -165.266), (-88.792, -28.773, -178.181), (-91.371, -28.722, 178.05),
            (-93.909, -28.646, 179.193), (-96.407, -28.662, -178.016), (-98.977, -28.897, -170.982),
            (-101.505, -29.422, -166.093), (-104.001, -30.087, -164.25), (-106.399, -30.794, -163.054),
            (-108.861, -31.574, -162.207), (-111.28, -32.35, -162.207), (-113.66, -33.114, -162.277),
            (-116.041, -33.875, -162.277), (-118.504, -34.662, -162.277), (-120.885, -35.423, -162.277),
            (-123.349, -36.201, -162.906), (-125.821, -36.95, -163.188), (-126.578, -37.179, -163.188),
            (-126.578, -37.179, -163.188)]
TRAJ_jeep = [(13.465, -14.074, -54.434), (14.291, -15.038, -44.591),
             (15.247, -15.842, -36.017), (16.332, -16.541, -31.485), (17.431, -17.178, -29.198),
             (18.529, -17.777, -28.138), (19.669, -18.384, -28.068), (20.78, -19.001, -29.849),
             (21.871, -19.652, -32.056), (22.924, -20.325, -33.19), (23.963, -21.021, -34.183),
             (24.997, -21.723, -34.183), (26.031, -22.425, -34.183), (27.1, -23.151, -34.113),
             (28.173, -23.87, -33.618), (29.233, -24.57, -33.408), (30.295, -25.268, -33.196),
             (31.341, -25.952, -33.196), (32.422, -26.66, -33.196), (33.485, -27.355, -33.196),
             (34.391, -27.949, -33.196), (34.391, -27.949, -33.196), (34.827, -28.234, -33.266),
             (35.593, -28.738, -33.336), (37.333, -29.882, -33.336), (41.611, -32.627, -31.76),
             (43.172, -33.589, -31.618), (43.172, -33.589, -31.618)]
TRAJ_PED = [
    (-11.916, -20.109, -6.396), (-11.916, -20.109, -6.396), (-11.916, -20.109, -5.898),
    (-11.916, -20.109, -6.913), (-11.767, -20.127, -6.913), (-11.254, -20.189, -6.983),
    (-10.749, -20.247, -6.344), (-10.244, -20.301, -5.994), (-9.738, -20.352, -5.359),
    (-9.223, -20.392, -4.141), (-8.724, -20.428, -4.071), (-8.217, -20.462, -3.503),
    (-7.718, -20.492, -2.503), (-7.21, -20.509, -1.792), (-6.71, -20.524, -1.157),
    (-6.194, -20.522, 0.575), (-5.694, -20.522, -0.068), (-5.177, -20.523, -0.068),
    (-4.677, -20.525, -0.776), (-4.177, -20.533, -0.919), (-3.669, -20.536, 1.385),
    (-3.174, -20.476, 14.906), (-2.712, -20.27, 34.825), (-2.345, -19.921, 52.566),
    (-2.063, -19.499, 60.221), (-1.832, -19.037, 68.177), (-1.704, -18.546, 78.633),
    (-1.61, -18.039, 79.924), (-1.526, -17.546, 80.556), (-1.441, -17.037, 80.486),
    (-1.355, -16.527, 80.486), (-1.253, -16.029, 75.851), (-1.112, -15.549, 71.463),
    (-0.882, -15.088, 58.819), (-0.622, -14.661, 58.321), (-0.322, -14.241, 50.394),
    (0.035, -13.894, 34.975), (0.477, -13.643, 26.251), (0.933, -13.438, 22.69),
    (1.41, -13.241, 22.194), (1.875, -13.055, 21.491), (2.358, -12.872, 20.559),
    (2.827, -12.7, 19.051), (3.196, -13.011, 34.993), (3.63, -12.731, 31.568), (4.075, -12.47, 28.488),
    (4.53, -12.243, 25.052),
    (5.005, -12.04, 21.748), (5.479, -11.856, 21.103), (5.945, -11.676, 21.103), (6.427, -11.49, 21.103),
    (6.909, -11.304, 21.103), (7.392, -11.121, 19.853), (7.871, -10.949, 19.853), (8.356, -10.771, 20.206),
    (8.841, -10.593, 20.206), (9.326, -10.414, 20.206), (9.795, -10.242, 20.206), (10.272, -10.066, 20.206),
    (10.749, -9.89, 20.206), (11.218, -9.718, 20.206), (11.703, -9.539, 20.206), (12.173, -9.366, 20.206),
    (12.65, -9.191, 20.206), (13.135, -9.012, 20.206), (13.604, -8.839, 20.631), (14.075, -8.628, 28.671),
    (14.51, -8.351, 33.535), (14.934, -8.07, 33.535), (15.365, -7.785, 33.535), (15.784, -7.513, 32.117),
    (16.234, -7.278, 22.066)]
TRAJ_HARLEY = [(-3.685, 53.572, -93.29), (-4.084, 47.235, -93.715), (-4.503, 40.79, -93.715), (-4.908, 34.553, -93.715),
               (-5.335, 28.317, -94.207), (-5.92, 21.886, -96.182), (-6.935, 15.617, -103.07),
               (-9.079, 9.539, -114.497), (-12.036, 4.038, -121.897), (-15.956, -1.073, -133.907),
               (-20.885, -5.218, -145.843), (-26.352, -8.447, -152.948), (-32.229, -11.116, -156.23),
               (-38.171, -13.636, -158.841), (-44.257, -15.78, -161.469), (-50.271, -17.832, -161.909),
               (-56.23, -19.713, -162.616), (-62.392, -21.642, -162.616), (-68.368, -23.466, -163.329),
               (-74.367, -25.217, -164.175), (-80.58, -26.978, -164.035), (-86.774, -28.8, -163.402),
               (-92.826, -30.737, -161.696), (-98.914, -32.748, -161.981), (-105.069, -34.701, -162.753),
               (-111.248, -36.57, -163.315), (-117.245, -38.336, -163.74), (-123.247, -40.087, -163.74),
               (-126.246, -40.966, -163.457), (-126.246, -40.966, -163.457), (-126.246, -40.966, -163.457),
               (-126.246, -40.966, -163.457), (-126.246, -40.966, -163.457)]


# ==========================================
# 4. 主程序 (Main Loop)
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    tm = client.get_trafficmanager(8000)

    weather = carla.WeatherParameters(
        cloudiness=40.0, precipitation=40.0, precipitation_deposits=55.0,
        wind_intensity=0.0, sun_azimuth_angle=0.0, sun_altitude_angle=18.0,
        fog_density=2.0, fog_distance=0.0, fog_falloff=0.0, wetness=55.0,
        scattering_intensity=0.0, mie_scattering_scale=0.02, rayleigh_scattering_scale=0.1000
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
        tm.set_synchronous_mode(True)

        # 辅助生成函数，防止车辆陷入地下
        def spawn_actor(bp_id, traj, color=None, is_pedestrian=False):
            bp = bp_lib.find(bp_id) if not is_pedestrian else bp_lib.filter(bp_id)[0]
            if color and bp.has_attribute('color'):
                bp.set_attribute('color', color)

            # 强制关闭行人的无敌状态，使得碰撞物理生效(可以被撞飞)
            if is_pedestrian and bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'false')

            x, y, yaw = traj[0]
            loc = carla.Location(x=x, y=y, z=1.5)  # 给点初始高度，让重力系统接管
            rot = carla.Rotation(yaw=yaw)
            actor = world.try_spawn_actor(bp, carla.Transform(loc, rot))
            if actor: actor_list.append(actor)
            return actor

        print("正在生成所有 Actor...")

        ego_veh = spawn_actor('vehicle.audi.tt', TRAJ_EGO, '0,255,0')
        ego_pid = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        jeep_veh = spawn_actor('vehicle.jeep.wrangler_rubicon', TRAJ_jeep)
        jeep_pid = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        pedestrian = spawn_actor('walker.pedestrian.*', TRAJ_PED, is_pedestrian=True)

        harley_veh = spawn_actor('vehicle.harley-davidson.low_rider', TRAJ_HARLEY)
        harley_pid = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        print("等待物理引擎环境预热 (让车辆和平稳落地)...")
        for _ in range(40):
            world.tick()

        print("预热完毕，赋予初始运动速度...")
        # 💡 车初始速度km/h
        set_initial_velocity(ego_veh, 20.0)
        set_initial_velocity(jeep_veh, 10.0)
        set_initial_velocity(harley_veh, 20.0)

        print("仿真正式开始！")

        idx_ego, idx_jeep, idx_ped, idx_harley = 0, 0, 0, 0

        # 💡 增加 Ego 车辆的目标速度动态变量，以及加速度参数
        ego_current_target_speed = 20.0  # 初始目标 20km/h
        ego_max_speed = 70.0  # 最高目标 70km/h
        ego_accel_rate_kmh_per_s = 15.0  # 加速度：km/h
        # 💡 增加 harley 车辆的目标速度动态变量，以及加速度参数

        harley_current_target_speed = 20.0  # 初始目标 km/h
        harley_max_speed = 90.0  # 最高目标 km/h
        harley_accel_rate_kmh_per_s = 10.0  # 加速度：km/h
        while True:
            start_time = time.time()
            world.tick()

            # --- 控制 Ego 车辆 ---
            if ego_veh and ego_veh.is_alive:
                if not check_and_handle_out_of_bounds(ego_veh, carla_map):

                    # 💡 获取当前车辆位置，判定是否过线进行加速
                    current_ego_loc = ego_veh.get_location()

                    # 因为轨迹的 Y 从 -48 逐渐向 0 和正数增加，所以使用 >= -20
                    if current_ego_loc.y >= -30.0:
                        # 每一个 dt (0.05秒)，速度目标增加 (加速度 * dt)
                        ego_current_target_speed += ego_accel_rate_kmh_per_s * dt
                        # 上限裁剪到 70 km/h
                        ego_current_target_speed = min(ego_current_target_speed, ego_max_speed)

                    if idx_ego < len(TRAJ_EGO):
                        tx, ty, _ = TRAJ_EGO[idx_ego]
                        target_loc = carla.Location(x=tx, y=ty, z=current_ego_loc.z)
                        if current_ego_loc.distance(target_loc) < 2.5 and idx_ego < len(TRAJ_EGO) - 1:
                            idx_ego += 1

                        # 💡 将动态计算的目标速度喂给 PID
                        apply_pid_control(ego_veh, ego_pid['lon'], ego_pid['lat'], ego_current_target_speed, target_loc)
                    else:
                        ego_veh.apply_control(carla.VehicleControl(brake=1.0))

            # --- 控制 Jeep 车辆 (10 km/h) ---
            if jeep_veh and jeep_veh.is_alive:
                if not check_and_handle_out_of_bounds(jeep_veh, carla_map):
                    if idx_jeep < len(TRAJ_jeep):
                        tx, ty, _ = TRAJ_jeep[idx_jeep]
                        target_loc = carla.Location(x=tx, y=ty, z=jeep_veh.get_location().z)
                        if jeep_veh.get_location().distance(target_loc) < 1.0 and idx_jeep < len(TRAJ_jeep) - 1:
                            idx_jeep += 1
                        apply_pid_control(jeep_veh, jeep_pid['lon'], jeep_pid['lat'], 10.0, target_loc)
                    else:
                        jeep_veh.apply_control(carla.VehicleControl(brake=1.0))

            # --- 控制 Harley 摩托车 ---
            if harley_veh and harley_veh.is_alive:
                if not check_and_handle_out_of_bounds(harley_veh, carla_map):

                    # 💡 获取当前车辆位置，判定是否过线进行加速
                    current_harley_loc = harley_veh.get_location()
                    if current_harley_loc.y <= 15.0:
                        # 每一个 dt (0.05秒)，速度目标增加 (加速度 * dt)
                        harley_current_target_speed += harley_accel_rate_kmh_per_s * dt
                        # 上限裁剪到 90 km/h
                        harley_current_target_speed = min(harley_current_target_speed, harley_max_speed)

                    if idx_harley < len(TRAJ_HARLEY):
                        tx, ty, _ = TRAJ_HARLEY[idx_harley]
                        target_loc = carla.Location(x=tx, y=ty, z=harley_veh.get_location().z)
                        if harley_veh.get_location().distance(target_loc) < 3.0 and idx_harley < len(TRAJ_HARLEY) - 1:
                            idx_harley += 1
                        # 💡 将动态计算的目标速度喂给 PID
                        apply_pid_control(harley_veh, harley_pid['lon'], harley_pid['lat'], harley_current_target_speed, target_loc)
                    else:
                        harley_veh.apply_control(carla.VehicleControl(brake=1.0))

            # --- 重点：CARLA 0.9.15 行人方向与分段运动控制 ---
            if pedestrian and pedestrian.is_alive:
                current_loc = pedestrian.get_location()

                # 动态判断速度。如果 x 还没到 -1.7，速度是 1.5，到了就突变为 4.0
                ped_speed = 4.0 if current_loc.x >= -1.7 else 1.5

                if idx_ped < len(TRAJ_PED):
                    tx, ty, _ = TRAJ_PED[idx_ped]
                    target_loc = carla.Location(x=tx, y=ty)

                    # 距离判定
                    if current_loc.distance(target_loc) < 1.0 and idx_ped < len(TRAJ_PED) - 1:
                        idx_ped += 1
                        tx, ty, _ = TRAJ_PED[idx_ped]
                        target_loc = carla.Location(x=tx, y=ty)

                    direction = carla.Vector3D(target_loc.x - current_loc.x, target_loc.y - current_loc.y, 0)
                    norm = math.sqrt(direction.x ** 2 + direction.y ** 2)
                    if norm > 0.001:
                        direction.x /= norm
                        direction.y /= norm

                    walker_control = carla.WalkerControl(direction=direction, speed=ped_speed, jump=False)
                    pedestrian.apply_control(walker_control)
                else:
                    # 到达终点，停止
                    pedestrian.apply_control(carla.WalkerControl(direction=carla.Vector3D(0, 0, 0), speed=0.0))

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
        if tm: tm.set_synchronous_mode(False)
        print("清理完毕。")


if __name__ == '__main__':
    main()