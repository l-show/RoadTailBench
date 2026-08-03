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

# ==========================================
# 辅助函数：去除轨迹中距离过近（重复）的点
# ==========================================
def clean_trajectory(traj_list, min_distance=0.5):
    if not traj_list: return []
    cleaned = [traj_list[0]]
    for pt in traj_list[1:]:
        last_pt = cleaned[-1]
        dist = math.sqrt((pt[0] - last_pt[0]) ** 2 + (pt[1] - last_pt[1]) ** 2)
        if dist > min_distance:
            cleaned.append(pt)
    return cleaned

# ==========================================
# 物理冲量计算函数：根据质量、速度和方向计算 Impulse
# ==========================================
def calculate_impulse(mass_kg, speed_ms, pitch_deg, yaw_deg):
    """
    通过动量公式 P = m * v 计算冲量 (Impulse)
    """
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)

    # 将速度分解到 X, Y, Z 三个轴向 (注意 CARLA 中 Pitch 负数是低头/下坡)
    v_x = speed_ms * math.cos(pitch_rad) * math.cos(yaw_rad)
    v_y = speed_ms * math.cos(pitch_rad) * math.sin(yaw_rad)
    v_z = speed_ms * math.sin(pitch_rad)

    # 冲量 = 质量 * 速度增量
    impulse_x = mass_kg * v_x
    impulse_y = mass_kg * v_y
    impulse_z = mass_kg * v_z

    return carla.Vector3D(x=impulse_x, y=impulse_y, z=impulse_z)

def is_vehicle_or_walker(actor):
    if not actor or not actor.is_alive:
        return False
    try:
        actor_type = actor.type_id
    except Exception:
        return False
    return actor_type.startswith('vehicle.') or actor_type.startswith('walker.')

def cleanup_vehicles_and_walkers(client, world, actor_list):
    actors = []
    seen = set()

    def add(actor):
        if not is_vehicle_or_walker(actor):
            return
        actor_id = actor.id
        if actor_id in seen:
            return
        seen.add(actor_id)
        actors.append(actor)

    for actor in actor_list:
        add(actor)
    try:
        world_actors = world.get_actors()
        for pattern in ('vehicle.*', 'walker.*'):
            for actor in world_actors.filter(pattern):
                add(actor)
    except Exception:
        pass

    commands = [carla.command.DestroyActor(actor.id) for actor in actors if is_vehicle_or_walker(actor)]
    if commands:
        client.apply_batch(commands)

# 原始轨迹数据
RAW_TRUCK_TRAJECTORY = [
    (12.394, 356.41, -86.547), (12.867, 348.808, -86.405), (13.395, 341.087, -85.697),
    (14.026, 333.499, -84.426), (14.968, 325.825, -80.964), (16.319, 318.21, -79.385),
    (17.8, 310.745, -77.081), (19.817, 303.273, -73.48), (22.03, 296.119, -72.134),
    (24.477, 288.778, -70.344), (27.289, 281.84, -66.939), (30.534, 274.816, -64.235),
    (33.98, 268.169, -62.001), (37.617, 261.339, -61.719), (41.256, 254.797, -59.791),
    (45.131, 248.244, -59.223), (49.087, 241.597, -59.506), (52.469, 234.904, -68.93),
    (54.632, 227.748, -79.028), (54.974, 220.051, -95.668), (53.34, 212.76, -109.048),
    (49.898, 205.857, -124.133), (44.93, 200.109, -133.403), (38.773, 195.855, -159.026),
    (31.647, 193.185, -161.194), (24.226, 190.978, -166.516), (16.75, 189.49, -169.765),
    (9.106, 188.239, -173.144), (1.405, 187.428, -174.08), (-6.046, 186.656, -174.08),
    (-13.524, 186.433, 178.793), (-21.241, 186.904, 174.609), (-28.648, 187.951, 169.122),
    (-36.202, 189.605, 167.449), (-43.454, 191.436, 163.351), (-50.512, 193.909, 158.42),
    (-57.574, 196.735, 156.834), (-64.385, 199.843, 154.759), (-71.239, 203.165, 153.631),
    (-78.138, 206.678, 152.775), (-84.801, 210.105, 152.845), (-91.475, 213.509, 153.128),
    (-98.163, 216.883, 153.341), (-105.036, 220.334, 153.341), (-111.842, 223.751, 153.341),
    (-118.76, 227.224, 153.341), (-125.589, 230.653, 153.341), (-132.508, 234.121, 153.481),
    (-139.433, 237.576, 153.481), (-146.245, 240.975, 153.481), (-153.169, 244.43, 153.481)
]

RAW_EGO_TRAJECTORY = [
    (8.045, 410.244, -85.645), (8.44, 405.185, -85.647), (8.787, 400.035, -86.216),
    (9.116, 395.049, -86.216), (9.452, 389.981, -86.216), (9.792, 384.83, -86.216),
    (10.133, 379.678, -86.216), (10.496, 374.616, -85.513), (10.89, 369.607, -85.443),
    (11.308, 364.557, -85.23), (11.737, 359.425, -85.23), (12.16, 354.374, -85.23),
    (12.581, 349.32, -85.23), (12.988, 344.347, -85.44), (13.395, 339.242, -85.44),
    (13.803, 334.183, -85.086), (14.376, 329.055, -82.425), (15.216, 323.968, -78.355),
    (16.241, 318.913, -78.638), (17.224, 314.019, -78.638), (18.253, 308.965, -77.927),
    (19.376, 303.931, -76.206), (20.726, 298.951, -74.208), (22.208, 294.181, -71.47),
    (23.83, 289.458, -70.343), (25.611, 284.792, -68.269), (27.571, 280.018, -66.033),
    (29.712, 275.509, -63.952), (31.918, 271.031, -63.74), (34.2, 266.405, -63.74),
    (36.103, 261.629, -76.181), (38.463, 257.281, -56.366), (40.865, 252.833, -55.402),
    (42.529, 250.651, -50.848), (42.529, 250.651, -50.848), (42.529, 250.651, -50.848),
    (43.043, 249.997, -51.853), (46.017, 245.883, -56.75), (48.577, 241.498, -62.186),
    (50.863, 236.966, -64.397), (52.672, 232.326, -73.946), (53.876, 227.401, -79.395),
    (54.368, 222.359, -90.04), (53.573, 217.46, -106.084), (52.145, 212.505, -106.084),
    (50.291, 207.714, -118.138), (47.615, 203.414, -129.142), (43.872, 199.878, -140.222),
    (39.949, 196.793, -142.702), (35.623, 194.001, -155.267), (30.812, 192.145, -159.326),
    (25.944, 190.724, -165.623), (20.924, 189.546, -168.209), (15.869, 188.525, -171.527),
    (10.761, 187.821, -172.316), (5.729, 187.151, -173.688), (0.586, 186.746, -175.714),
    (-4.56, 186.366, -176.067), (-9.717, 186.211, 179.862), (-14.793, 186.309, 177.508),
    (-19.939, 186.675, 174.504), (-24.976, 187.313, 171.343), (-29.895, 188.172, 169.348),
    (-34.873, 189.173, 166.826), (-39.868, 190.462, 165.246), (-44.823, 191.891, 161.89),
    (-49.725, 193.494, 161.747), (-54.462, 195.313, 158.212), (-59.237, 197.264, 156.713)
]

TRUCK_TRAJECTORY = clean_trajectory(RAW_TRUCK_TRAJECTORY)
EGO_TRAJECTORY = clean_trajectory(RAW_EGO_TRAJECTORY)

# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 【1. 设置极强灾害天气】
    weather = carla.WeatherParameters(
        cloudiness=25.0, precipitation=90.0, precipitation_deposits=60.0,
        wind_intensity=100.0, sun_azimuth_angle=99.0, sun_altitude_angle=45.0,
        fog_density=0.0, fog_distance=0.0, fog_falloff=0.0, wetness=80.0,
        scattering_intensity=0.0, mie_scattering_scale=0.0300, rayleigh_scattering_scale=0.2331, dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    rocks_list = []

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_truck = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= Actor 1：前车 (Sprinter 卡车) =================
        bp_truck = bp_lib.find('vehicle.mercedes.sprinter')
        truck_start_x, truck_start_y, truck_start_yaw = TRUCK_TRAJECTORY[0]
        truck_loc = carla.Location(x=truck_start_x, y=truck_start_y, z=0.5)
        truck_loc.z = carla_map.get_waypoint(truck_loc).transform.location.z + 0.5
        truck = world.try_spawn_actor(bp_truck, carla.Transform(truck_loc, carla.Rotation(yaw=truck_start_yaw)))
        if truck:
            actor_list.append(truck)
            print("生成 前车(Sprinter) 成功。")

        # ================= Actor 2：自车 (Audi TT) =================
        bp_ego = bp_lib.find('vehicle.audi.tt')
        if bp_ego.has_attribute('role_name'):
            pass
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '0,255,0')
        ego_start_x, ego_start_y, ego_start_yaw = EGO_TRAJECTORY[0]
        ego_loc = carla.Location(x=ego_start_x, y=ego_start_y, z=0.5)
        ego_loc.z = carla_map.get_waypoint(ego_loc).transform.location.z + 0.5
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        if ego:
            print("生成 自车(Audi TT) 成功。")

        # 【2. 物理预热】先让物理引擎走几帧
        for _ in range(15):
            world.tick()

        # 【3. 赋予初始速度向量】拒绝0基础加速瞬移
        if truck:
            init_speed_ms = 55.0 / 3.6
            yaw_rad = math.radians(truck_start_yaw)
            truck.set_target_velocity(
                carla.Vector3D(init_speed_ms * math.cos(yaw_rad), init_speed_ms * math.sin(yaw_rad), 0.0))

        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds

        # 状态机变量
        truck_traj_idx, ego_traj_idx = 0, 0
        truck_state, ego_state = "NORMAL", "NORMAL"
        truck_wait_timer, ego_wait_timer = 0.0, 0.0
        truck_target_speed, ego_target_speed = 55.0, 65.0
        rock_spawned = False

        print("\n仿真正式开始！")

        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds - start_sim_time

            # ==========================
            # 前车 (Truck) 复杂逻辑循迹控制
            # ==========================
            if truck and truck.is_alive and truck_traj_idx < len(TRUCK_TRAJECTORY):
                t_loc = truck.get_location()
                tx, ty, tyaw = TRUCK_TRAJECTORY[truck_traj_idx]
                target_loc = carla.Location(x=tx, y=ty, z=t_loc.z)

                if t_loc.distance(target_loc) < 4.5 and truck_traj_idx < len(TRUCK_TRAJECTORY) - 1:
                    truck_traj_idx += 1

                vel = truck.get_velocity()
                truck_spd_kmh = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2) * 3.6

                if truck_state == "NORMAL":
                    truck_target_speed = 55.0
                    if t_loc.y <= 280.0:
                        truck_state = "BRAKING"
                        print(f"[{sim_time:.1f}s] Truck 开始紧急制动躲避落石！")
                elif truck_state == "BRAKING":
                    truck_target_speed = 30.0
                    if truck_spd_kmh <= 32.0:
                        truck_state = "WAITING"
                        truck_wait_timer = sim_time
                        print(f"[{sim_time:.1f}s] Truck 减速完成，等待中...")
                elif truck_state == "WAITING":
                    truck_target_speed = 30.0
                    if (sim_time - truck_wait_timer) >= 3.0:
                        truck_state = "RECOVER"
                        print(f"[{sim_time:.1f}s] Truck 恢复加速！")
                elif truck_state == "RECOVER":
                    truck_target_speed = 60.0

                apply_pid_control(truck, pid_truck['lon'], pid_truck['lat'], truck_target_speed, target_loc)
            elif truck and truck_traj_idx >= len(TRUCK_TRAJECTORY) - 1:
                truck.apply_control(carla.VehicleControl(brake=1.0))

            # ==========================
            # 自车 (Ego TT) 复杂逻辑循迹控制
            # ==========================
            # 帧率同步控制
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)
    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        cleanup_vehicles_and_walkers(client, world, actor_list)

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("清理完毕。")

if __name__ == '__main__':
    main()
