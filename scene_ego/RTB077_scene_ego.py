import carla
import time
import math
import random
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.8, 0.6)

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
# 2. 物理落叶管理器 (Leaf Wind Manager)
# ==========================================
class LeafWindManager:
    def __init__(self, world, blueprint_library):
        self.world = world
        self.bp_lib = blueprint_library
        self.spawn_point = carla.Location(x=7.743, y=-99.173, z=10.423)
        self.target_point = carla.Location(x=-1.175, y=-105.217, z=0.258)
        self.mesh_path = '/Game/Carla/Static/RoadTailModel/Maple__leave_SM_Leaf_21.Maple__leave_SM_Leaf_21'
        self.num_leaves = 50
        self.leaf_mass = 0.02
        self.leaf_scale = 3
        self.base_wind_strength = 0.09
        self.upward_lift_force = 0.06
        self.flutter_amplitude = 0.05
        self.flutter_frequency = 6.0
        self.linear_drag_coeff = 0.03
        self.angular_drag_coeff = 0.002
        self.leaves_data = []
        self.has_spawned = False
        dx = self.target_point.x - self.spawn_point.x
        dy = self.target_point.y - self.spawn_point.y
        distance_xy = math.sqrt(dx ** 2 + dy ** 2)
        self.dir_x = dx / distance_xy if distance_xy > 0 else 0
        self.dir_y = dy / distance_xy if distance_xy > 0 else 0

    def spawn_leaves(self):
        try:
            bp_prop = self.bp_lib.find('static.prop.mesh')
            bp_prop.set_attribute('mesh_path', self.mesh_path)
            bp_prop.set_attribute('mass', str(self.leaf_mass))
            bp_prop.set_attribute('scale', str(self.leaf_scale))
            spawned_count = 0
            for i in range(self.num_leaves):
                offset_x = random.uniform(-1.5, 1.5)
                offset_y = random.uniform(-1.5, 1.5)
                offset_z = random.uniform(-1.0, 1.0)
                loc = carla.Location(self.spawn_point.x + offset_x, self.spawn_point.y + offset_y,
                                     self.spawn_point.z + offset_z)
                rot = carla.Rotation(pitch=random.uniform(0, 360), yaw=random.uniform(0, 360),
                                     roll=random.uniform(0, 360))
                leaf_actor = self.world.try_spawn_actor(bp_prop, carla.Transform(loc, rot))
                if leaf_actor:
                    leaf_actor.set_simulate_physics(True)
                    self.leaves_data.append({'actor': leaf_actor, 'phase_x': random.uniform(0, math.pi * 2),
                                             'phase_y': random.uniform(0, math.pi * 2), 'settled': False})
                    spawned_count += 1
            self.has_spawned = True
            print(f"\n[落叶系统] 成功生成 {spawned_count} 片树叶，狂风开始吹袭！")
        except:
            print("\n[落叶系统] 找不到对应Mesh，跳过落叶生成。")
            self.has_spawned = True

    def tick(self, sim_time):
        if not self.has_spawned: return
        for leaf in self.leaves_data:
            if leaf['settled']: continue
            actor = leaf['actor']
            if not actor.is_alive: continue
            loc = actor.get_location()
            if loc.z <= self.target_point.z + 0.3:
                leaf['settled'] = True
                continue
            vel = actor.get_velocity()
            ang_vel = actor.get_angular_velocity()
            f_x = self.dir_x * self.base_wind_strength
            f_y = self.dir_y * self.base_wind_strength
            f_z = self.upward_lift_force
            flutter_x = math.sin(sim_time * self.flutter_frequency + leaf['phase_x']) * self.flutter_amplitude
            flutter_y = math.cos(sim_time * self.flutter_frequency + leaf['phase_y']) * self.flutter_amplitude
            drag_x = -self.linear_drag_coeff * vel.x
            drag_y = -self.linear_drag_coeff * vel.y
            drag_z = -self.linear_drag_coeff * vel.z
            total_force = carla.Vector3D(x=f_x + flutter_x + drag_x, y=f_y + flutter_y + drag_y, z=f_z + drag_z)
            actor.add_force(total_force)
            torque_x = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.x
            torque_y = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.y
            torque_z = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.z
            actor.add_torque(carla.Vector3D(x=torque_x, y=torque_y, z=torque_z))

    def cleanup(self):
        for leaf in self.leaves_data:
            if leaf['actor'].is_alive: leaf['actor'].destroy()

# ==========================================
# 3. 轨迹数据
# ==========================================
# 提取行人的XY坐标进行轨迹追踪
WALKER_TRAJECTORY = [
    (9.078, -102.297), (9.078, -102.297), (9.078, -102.297),
    (8.054, -102.728), (8.054, -102.728), (7.365, -103.143),
    (6.376, -103.622), (5.729, -103.831), (5.276, -103.508),
    (4.674, -103.072), (4.042, -103.507), (3.075, -104.087),
    (1.275, -104.939), (-0.335, -105.469), (-2.662, -105.832),
    (-5.787, -105.689), (-8.181, -105.580,), (-10.694, -105.545),
    (-11.571, -105.582), (-12.870, -105.636), (-13.165, -105.648),
    (-13.165, -105.648), (-13.165, -105.648)
]

# PID控制的第三辆车轨迹 (x, y, yaw)
V3_TRAJECTORY_DATA = [
    (-53.167, -118.612, 1.115), (-53.167, -118.612, 1.045), (-51.946, -118.589, 1.21), (-47.63, -118.409, 4.352),
    (-47.63, -118.409, 4.352),
    (-47.597, -118.406, 4.352), (-47.112, -118.325, 16.137), (-46.614, -118.175, 18.673), (-46.13, -117.996, 11.012),
    (-45.63, -118.047, -21.32),
    (-45.161, -118.245, -22.26), (-44.676, -118.429, -18.93), (-44.207, -118.59, -18.93), (-43.722, -118.742, -15.273),
    (-43.228, -118.848, -9.367),
    (-42.725, -118.907, -4.833), (-42.225, -118.936, -1.289), (-41.725, -118.919, 4.937), (-41.212, -118.851, 9.743),
    (-40.717, -118.767, 8.744),
    (-40.717, -118.767, 8.744), (-40.304, -118.7, 9.616), (-39.867, -118.648, -2.892), (-39.594, -118.661, -2.892),
    (-38.315, -118.838, -12.149),
    (-37.07, -119.171, -15.019), (-35.803, -119.353, -4.191), (-34.519, -119.445, -4.026), (-33.269, -119.431, 6.951),
    (-32.046, -119.254, 7.693),
    (-30.8, -119.091, 7.373), (-29.557, -118.959, 9.683), (-28.295, -118.746, 6.845), (-26.993, -118.64, 2.727),
    (-25.723, -118.638, -3.911),
    (-24.424, -118.761, -5.565), (-23.15, -118.94, -9.903), (-21.897, -119.148, -6.467), (-20.647, -119.125, 6.259),
    (-19.384, -118.897, 13.929),
    (-18.14, -118.563, 15.908), (-16.906, -118.211, 15.908), (-15.649, -117.873, 11.379), (-14.419, -117.853, -8.127),
    (-13.203, -118.204, -21.725),
    (-12.033, -118.714, -25.527), (-10.91, -119.319, -31.111), (-9.883, -120.057, -39.227), (-8.924, -120.884, -42.761),
    (-7.996, -121.792, -46.881),
    (-7.172, -122.747, -49.665), (-6.374, -123.715, -51.199), (-5.586, -124.724, -52.681), (-4.851, -125.722, -54.602),
    (-4.127, -126.764, -56.059),
    (-3.954, -127.021, -56.059), (-3.797, -127.256, -56.856), (-3.105, -128.333, -57.537), (-2.44, -129.378, -57.537),
    (-1.754, -130.485, -60.006),
    (-1.141, -131.602, -62.795), (-0.602, -132.738, -64.97), (-0.086, -133.892, -69.033), (0.329, -135.103, -71.556),
    (0.71, -136.293, -73.425),
    (1.008, -137.512, -77.764), (1.251, -138.764, -80.85), (1.43, -140.031, -83.932), (1.528, -141.322, -87.869),
    (1.551, -142.602, -89.723),
    (1.557, -143.88, -89.933), (1.536, -145.125, -91.73), (1.494, -146.387, -92.1), (1.447, -147.689, -91.8),
    (1.418, -148.975, -91.22),
    (1.389, -150.24, -91.312), (1.359, -151.519, -91.312), (1.339, -152.408, -91.312), (1.339, -152.408, -91.312),
    (1.318, -153.351, -91.312),
    (1.251, -158.36, -90.061), (1.345, -163.505, -88.301), (1.494, -168.511, -88.301), (1.599, -173.493, -89.24),
    (1.666, -178.527, -89.24),
    (1.735, -183.737, -89.24), (1.796, -188.902, -89.59), (1.8, -193.979, -90.033), (1.797, -199.134, -90.033),
    (1.763, -204.263, -90.734),
    (1.697, -209.394, -90.734), (1.631, -214.599, -90.665), (1.59, -219.733, -90.167), (1.575, -224.909, -90.167),
    (1.575, -229.908, -89.951),
    (1.6, -234.938, -89.379), (1.66, -239.973, -89.309), (1.72, -244.972, -89.309), (1.75, -247.472, -89.309),
    (1.75, -247.472, -89.309),
    (1.75, -247.472, -89.309)
]

def check_and_handle_out_of_bounds(vehicle, carla_map):
    loc = vehicle.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    wp_exact = carla_map.get_waypoint(loc, project_to_road=False)

    is_out = False
    if wp_exact is None:
        is_out = True
    elif wp_nearest and wp_nearest.transform.location.distance(loc) > 4.0:
        is_out = True

    if is_out:
        vehicle.destroy()
        return True
    return False

def is_near_xy(actor, goal_xy, threshold=5.0):
    if not actor or not actor.is_alive:
        return False
    loc = actor.get_location()
    return math.hypot(loc.x - goal_xy[0], loc.y - goal_xy[1]) <= threshold

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

    # ---------------- 依据您的截图设置精确天气 ----------------
    weather = carla.WeatherParameters(
        cloudiness=20.0, precipitation=0.0, precipitation_deposits=20.0,
        wind_intensity=5.0, sun_azimuth_angle=95.0, sun_altitude_angle=18.0,
        fog_density=2.0, fog_distance=0.0, fog_falloff=0.0, wetness=10.0,
        scattering_intensity=1.0, mie_scattering_scale=0.0, rayleigh_scattering_scale=0.1,
        dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    active_actors = {'ego': False, 'v3': False, 'walker': False}
    ego_end_xy = (-73.882, -123.361)

    leaf_manager = LeafWindManager(world, bp_lib)

    try:
        # 同步模式设置
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

        pid_v3 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= Actor 1：静止停靠的卡车 =================
        bp_truck = bp_lib.find('vehicle.mercedes.sprinter')
        bp_truck.set_attribute('color', '128,0,0')  # 随便设置个颜色
        truck_loc = carla.Location(x=4.127, y=-99.172, z=0.5)
        truck_wp = carla_map.get_waypoint(truck_loc)
        truck_loc.z = truck_wp.transform.location.z + 0.5
        truck = world.try_spawn_actor(bp_truck, carla.Transform(truck_loc, truck_wp.transform.rotation))
        if truck:
            actor_list.append(truck)
            truck.apply_control(carla.VehicleControl(hand_brake=True))  # 强制静止不动

        # ================= Actor 2：按轨迹跑步的行人 =================
        bp_walker = bp_lib.find('walker.pedestrian.0001')
        walker_loc = carla.Location(x=WALKER_TRAJECTORY[0][0], y=WALKER_TRAJECTORY[0][1], z=1.0)  # Z稍微拉高防止穿模
        walker = world.try_spawn_actor(bp_walker, carla.Transform(walker_loc, carla.Rotation()))
        if walker:
            actor_list.append(walker)
            active_actors['walker'] = True

        # ================= Actor 3：TM 控制的 Ego (Cybertruck) =================
        bp_ego = bp_lib.find('vehicle.tesla.cybertruck')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '255,255,255')  # 白色
        if bp_ego.has_attribute('role_name'):
            bp_ego.set_attribute('role_name', 'ego')
        ego_loc = carla.Location(1.923, -51.492, z=0.5)
        ego_wp = carla_map.get_waypoint(ego_loc)
        ego_loc.z = ego_wp.transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego, carla.Transform(ego_loc, ego_wp.transform.rotation))
        if ego:
            actor_list.append(ego)
            active_actors['ego'] = True
            ego.set_autopilot(True, tm.get_port())
            # 设置TM：忽略红绿灯，超速10%达到60km/h，强制在前方路口左转
            tm.ignore_lights_percentage(ego, 100.0)
            tm.vehicle_percentage_speed_difference(ego, -10.0)

            # ❗修改点 1: CARLA 0.9.15 中没有 force_turn，改成 set_route ❗
            tm.set_route(ego, ['Left'])

        # ================= Actor 4：PID 控制的 V3 (VW T2) =================
        bp_v3 = bp_lib.find('vehicle.volkswagen.t2')
        v3_loc = carla.Location(x=V3_TRAJECTORY_DATA[0][0], y=V3_TRAJECTORY_DATA[0][1], z=0.5)
        v3_loc.z = carla_map.get_waypoint(v3_loc).transform.location.z + 0.5
        v3_yaw = V3_TRAJECTORY_DATA[0][2]
        v3 = world.try_spawn_actor(bp_v3, carla.Transform(v3_loc, carla.Rotation(yaw=v3_yaw)))
        if v3:
            actor_list.append(v3)
            active_actors['v3'] = True

        # 解决瞬移BUG：重力加载完后再赋予物理初速度
        print("\n正在等待悬挂系统贴合地面...")
        for _ in range(30): world.tick()

        print("为 Ego 和 V3 瞬间赋予 60km/h 物理初速度...")
        initial_speed_ms = 60.0 / 3.6
        for vehicle in [ego, v3]:
            if vehicle and vehicle.is_alive:
                yaw = math.radians(vehicle.get_transform().rotation.yaw)
                vehicle.set_target_velocity(
                    carla.Vector3D(x=initial_speed_ms * math.cos(yaw), y=initial_speed_ms * math.sin(yaw), z=0.0))

        print("仿真正式开始！")
        walker_traj_idx = 0
        v3_traj_idx = 0

        # ================= 进入主循环 =================
        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            # 触发落叶
            if sim_time >= 0.0 and not leaf_manager.has_spawned:
                leaf_manager.spawn_leaves()
            leaf_manager.tick(sim_time)

            # ==========================
            # 行人逻辑：追踪点跑步过街
            # ==========================
            if active_actors['walker'] and walker.is_alive:
                if walker_traj_idx < len(WALKER_TRAJECTORY):

                    # ❗修改点 2: 修复变量名拼写错误，将 walker_idx 改为 walker_traj_idx ❗
                    tgt_x, tgt_y = WALKER_TRAJECTORY[walker_traj_idx]

                    w_loc = walker.get_location()
                    tgt_loc = carla.Location(x=tgt_x, y=tgt_y, z=w_loc.z)

                    if w_loc.distance(tgt_loc) < 0.5:  # 跑到点附近就切换下一个点
                        walker_traj_idx += 1
                    else:
                        dir_vec = tgt_loc - w_loc
                        norm = math.sqrt(dir_vec.x ** 2 + dir_vec.y ** 2)
                        if norm > 0:
                            ctrl = carla.WalkerControl()
                            ctrl.direction = carla.Vector3D(dir_vec.x / norm, dir_vec.y / norm, 0)
                            ctrl.speed = 3.5  # 3.5m/s 模拟跑步速度
                            walker.apply_control(ctrl)
                else:
                    # 到达终点，停下
                    walker.apply_control(carla.WalkerControl(speed=0.0))

            # ==========================
            # Ego 车：TM 控制出界销毁
            # ==========================
            if active_actors['ego'] and ego.is_alive:
                if is_near_xy(ego, ego_end_xy, threshold=5.0):
                    print("TM Ego reached scenario endpoint; ending simulation.")
                    return
                elif check_and_handle_out_of_bounds(ego, carla_map):
                    print("TM Ego 驶出地图边界，已自动销毁。")
                    active_actors['ego'] = False

            # ==========================
            # V3 车：PID 循迹，完成/出界后解除控制
            # ==========================
            if active_actors['v3'] and v3.is_alive:
                # 若出界，直接解除控制让物理引擎接管
                if check_and_handle_out_of_bounds(v3, carla_map):
                    print("V3 驶出物理边界，停止PID控制。")
                    active_actors['v3'] = False
                elif v3_traj_idx < len(V3_TRAJECTORY_DATA):
                    tx, ty, tyaw = V3_TRAJECTORY_DATA[v3_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v3_loc.z)

                    if v3.get_location().distance(target_loc) < 3.0 and v3_traj_idx < len(V3_TRAJECTORY_DATA) - 1:
                        while v3_traj_idx < len(V3_TRAJECTORY_DATA) - 1:
                            v3_traj_idx += 1
                            if v3.get_location().distance(carla.Location(x=V3_TRAJECTORY_DATA[v3_traj_idx][0],
                                                                         y=V3_TRAJECTORY_DATA[v3_traj_idx][1],
                                                                         z=v3_loc.z)) > 2.0: break

                    # 使用PID以 80km/h 行驶
                    apply_pid_control(v3, pid_v3['lon'], pid_v3['lat'], 80.0, target_loc)
                else:
                    # 到达终点，按要求解除控制/刹车
                    print("V3 到达轨迹终点，解除循迹。")
                    v3.apply_control(carla.VehicleControl(brake=1.0))
                    active_actors['v3'] = False  # 不再触发本段代码

            # 帧率同步控制
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        leaf_manager.cleanup()
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
