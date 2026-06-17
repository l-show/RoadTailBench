import carla
import time
import math
import random
import numpy as np


# ==========================================
# 1. 基础控制算法 (PID) - 针对雨天防滑优化
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

        # --- 核心参数配置 ---
        self.spawn_point = carla.Location(x=34.740, y=-203.749, z=17.054)  # A点
        self.target_point = carla.Location(x=25.217, y=-210.177, z=1.097)  # B点
        self.mesh_path = '/Game/Carla/Static/RoadTailModel/Maple__leave_SM_Leaf_21.Maple__leave_SM_Leaf_21'
        self.num_leaves = 150
        self.leaf_mass = 0.02  # 保持 20克 极轻，被车撞到毫无影响
        self.leaf_scale = 5

        # 【优化修改点】：修改风力参数，模拟“下压狂风”
        self.base_wind_strength = 0.09  # 加大吹向B点的基础风力
        self.upward_lift_force = 0.06  # 大幅削弱升力 (原本是0.16)，让重力主导，加速下落
        self.flutter_amplitude = 0.05  # 加大摇摆幅度，掩盖下落变快的事实
        self.flutter_frequency = 6.0  # 摇摆频率加快，显得风很大

        self.linear_drag_coeff = 0.03  # 降低空气阻尼 (原本是0.08)，不拦着它往下掉
        self.angular_drag_coeff = 0.002

        self.leaves_data = []
        self.has_spawned = False

        dx = self.target_point.x - self.spawn_point.x
        dy = self.target_point.y - self.spawn_point.y
        distance_xy = math.sqrt(dx ** 2 + dy ** 2)
        self.dir_x = dx / distance_xy if distance_xy > 0 else 0
        self.dir_y = dy / distance_xy if distance_xy > 0 else 0

    def spawn_leaves(self):
        """在生成点附近生成一堆树叶"""
        bp_prop = self.bp_lib.find('static.prop.mesh')
        bp_prop.set_attribute('mesh_path', self.mesh_path)
        bp_prop.set_attribute('mass', str(self.leaf_mass))
        bp_prop.set_attribute('scale', str(self.leaf_scale))

        spawned_count = 0
        for i in range(self.num_leaves):
            # 在 A 点附近散开生成，防止重叠爆炸
            offset_x = random.uniform(-1.5, 1.5)
            offset_y = random.uniform(-1.5, 1.5)
            offset_z = random.uniform(-1.0, 1.0)

            loc = carla.Location(
                x=self.spawn_point.x + offset_x,
                y=self.spawn_point.y + offset_y,
                z=self.spawn_point.z + offset_z
            )
            rot = carla.Rotation(
                pitch=random.uniform(0, 360), yaw=random.uniform(0, 360), roll=random.uniform(0, 360)
            )

            leaf_actor = self.world.try_spawn_actor(bp_prop, carla.Transform(loc, rot))
            if leaf_actor:
                leaf_actor.set_simulate_physics(True)
                self.leaves_data.append({
                    'actor': leaf_actor,
                    'phase_x': random.uniform(0, math.pi * 2),
                    'phase_y': random.uniform(0, math.pi * 2),
                    'settled': False
                })
                spawned_count += 1

        self.has_spawned = True
        print(f"\n[落叶系统] 成功生成 {spawned_count} 片树叶，狂风开始吹袭！")

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
        print("清理树叶...")
        for leaf in self.leaves_data:
            if leaf['actor'].is_alive:
                leaf['actor'].destroy()


# ==========================================
# 3. 轨迹数据与辅助功能
# ==========================================
V1_TRAJECTORY_DATA = [
    (22.891, -165.265, -95.392), (22.891, -165.265, -96.701), (22.891, -165.265, -97.561),
    (22.776, -166.194, -96.351), (22.568, -170.061, -89.776), (22.654, -173.935, -88.336),
    (22.744, -177.809, -89.115), (22.803, -181.620, -89.257), (22.828, -185.495, -89.684),
    (22.893, -189.369, -88.557), (22.962, -193.118, -89.618), (22.968, -196.993, -90.540),
    (22.928, -200.868, -90.610), (22.888, -204.618, -90.610), (22.886, -204.806, -90.610),
    (22.886, -204.806, -94.587), (22.699, -207.111, -95.669), (21.725, -210.722, -112.772),
    (20.055, -214.211, -109.155), (19.367, -218.008, -92.868), (19.293, -221.881, -89.629),
    (19.223, -225.631, -90.614), (19.182, -229.443, -90.614), (19.142, -233.193, -90.544),
    (19.161, -237.068, -89.402), (19.192, -240.944, -89.822), (19.199, -244.694, -89.892),
    (19.206, -248.444, -89.892), (19.214, -252.194, -89.822), (19.235, -256.067, -89.682),
    (19.245, -259.800, -90.032), (19.184, -263.543, -91.303), (19.102, -267.417, -90.743),
    (19.118, -271.290, -89.391), (19.157, -275.037, -89.391), (19.187, -278.909, -89.811),
    (19.200, -282.782, -89.811), (19.196, -286.657, -90.306), (19.172, -290.469, -90.376),
    (19.132, -294.343, -90.731), (19.090, -297.906, -90.661)
]

EGO_TRAJECTORY_DATA = [
    (23.645, -135.552, -94.448), (23.645, -135.552, -94.448), (23.645, -135.552, -94.448),
    (23.645, -135.552, -92.966), (23.347, -141.367, -92.261), (23.123, -147.716, -92.258),
    (23.119, -147.821, -90.994), (23.119, -147.821, -89.942), (23.119, -147.821, -89.942),
    (23.121, -149.193, -89.942), (23.095, -151.774, -91.706), (23.025, -154.273, -91.214),
    (22.976, -156.855, -90.794), (22.952, -159.397, -91.396), (22.889, -161.979, -91.396),
    (22.825, -164.562, -91.466), (22.747, -167.060, -92.167), (22.653, -169.557, -92.167),
    (22.579, -172.139, -91.324), (22.531, -174.680, -91.044), (22.487, -177.263, -90.834),
    (22.440, -179.846, -91.184), (22.389, -182.346, -91.184), (22.374, -184.845, -89.426),
    (22.423, -187.345, -87.876), (22.519, -189.926, -87.876), (22.628, -192.467, -86.325),
    (22.828, -195.043, -84.000), (23.202, -197.514, -79.016), (23.773, -199.947, -73.398),
    (24.525, -202.331, -71.917), (25.405, -204.715, -68.246), (26.340, -207.034, -66.372),
    (27.407, -209.387, -65.315), (28.462, -211.651, -64.894), (29.556, -213.945, -63.772),
    (30.748, -216.187, -59.541), (32.100, -218.339, -55.093), (33.615, -220.410, -53.749),
    (35.144, -222.492, -53.327), (36.724, -224.534, -51.845), (38.288, -226.485, -50.015),
    (39.971, -228.331, -45.527), (41.815, -230.139, -42.994), (43.691, -231.789, -40.246),
    (45.687, -233.427, -38.133), (47.713, -234.958, -35.813), (49.835, -236.427, -34.401),
    (51.984, -237.858, -32.922), (54.180, -239.214, -29.963), (56.377, -240.400, -26.181),
    (58.669, -241.394, -21.522), (61.074, -242.333, -20.675), (63.500, -243.220, -19.903),
    (65.851, -244.071, -19.903), (68.201, -244.922, -19.903), (70.633, -245.802, -19.903),
    (73.062, -246.681, -19.903), (75.413, -247.532, -19.903), (77.803, -248.397, -19.973),
    (80.202, -249.352, -25.056), (82.272, -250.732, -45.228), (83.435, -252.776, -69.403),
    (83.435, -252.776, -69.403), (83.435, -252.776, -69.403), (83.435, -252.776, -69.403)
]


def custom_ad_algorithm_step(vehicle, target_wp_loc, sensor_data=None):
    control = carla.VehicleControl()
    control.brake = 1.0
    return control


def check_and_handle_out_of_bounds(vehicle, carla_map, name="Vehicle"):
    loc = vehicle.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    wp_exact = carla_map.get_waypoint(loc, project_to_road=False)

    is_out = False
    if wp_exact is None:
        is_out = True
    elif wp_nearest and wp_nearest.transform.location.distance(loc) > 4.0:
        is_out = True

    if is_out:
        print(f"[{name}] 警告: 车辆偏离道路边界，已自动销毁以免干扰交通！")
        vehicle.destroy()
        return True
    return False


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

    # 天气设置
    weather = carla.WeatherParameters(
        cloudiness=55.0, precipitation=70.0, precipitation_deposits=80.0,
        wind_intensity=100.0, sun_azimuth_angle=90.0, sun_altitude_angle=50.0,
        fog_density=6.0, fog_distance=0.0, fog_falloff=0.0, wetness=70.0,
        scattering_intensity=5.0, mie_scattering_scale=0.3, rayleigh_scattering_scale=0.05,
        dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    active_vehicles = {'v1': False, 'v2': False, 'ego': False}

    # 实例化落叶管理器
    leaf_manager = LeafWindManager(world, bp_lib)

    try:
        # 同步模式设置
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

        pids = {
            'v1': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'ego': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        }
        STORM_LIGHTS = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam

        # =================车辆生成区域=================
        bp_v1 = bp_lib.find('vehicle.chevrolet.impala')
        if bp_v1.has_attribute('color'): bp_v1.set_attribute('color', '0,255,0')
        v1_init_loc = carla.Location(x=V1_TRAJECTORY_DATA[0][0], y=V1_TRAJECTORY_DATA[0][1], z=0.5)
        v1_init_loc.z = carla_map.get_waypoint(v1_init_loc).transform.location.z + 0.5
        v1 = world.try_spawn_actor(bp_v1, carla.Transform(v1_init_loc, carla.Rotation(yaw=V1_TRAJECTORY_DATA[0][2])))
        if v1:
            actor_list.append(v1)
            active_vehicles['v1'] = True
            v1.set_light_state(carla.VehicleLightState(STORM_LIGHTS))

        bp_ego = bp_lib.find('vehicle.mercedes.coupe_2020')
        if bp_ego.has_attribute('color'): bp_ego.set_attribute('color', '0,50,0')
        ego_init_loc = carla.Location(x=EGO_TRAJECTORY_DATA[0][0], y=EGO_TRAJECTORY_DATA[0][1], z=0.5)
        ego_init_loc.z = carla_map.get_waypoint(ego_init_loc).transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego,
                                    carla.Transform(ego_init_loc, carla.Rotation(yaw=EGO_TRAJECTORY_DATA[0][2])))
        if ego:
            actor_list.append(ego)
            active_vehicles['ego'] = True
            ego.set_light_state(carla.VehicleLightState(STORM_LIGHTS))

        bp_v2 = bp_lib.find('vehicle.citroen.c3')
        if bp_v2.has_attribute('color'): bp_v2.set_attribute('color', '255,255,0')
        loc_v2 = carla.Location(x=23.393, y=-120, z=0.5)
        v2_wp = carla_map.get_waypoint(loc_v2)
        loc_v2.z = v2_wp.transform.location.z + 0.5
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(loc_v2, v2_wp.transform.rotation))
        if v2:
            actor_list.append(v2)
            active_vehicles['v2'] = True
            v2.set_light_state(carla.VehicleLightState(STORM_LIGHTS))
            v2.set_autopilot(True, tm.get_port())
            tm.ignore_vehicles_percentage(v2, 100)
            tm.ignore_lights_percentage(v2, 100.0)
            tm.distance_to_leading_vehicle(v2, 2.0)
            tm.global_percentage_speed_difference(-80.0)
            tm.force_lane_change(v2, True)

        # 解决瞬移BUG：重力加载完后再赋予物理初速度
        print("\n正在等待悬挂系统贴合地面...")
        for _ in range(30):
            world.tick()

        print("为所有车辆瞬间赋予 80km/h 物理初速度...")
        initial_speed_ms = 80.0 / 3.6
        for vehicle in [v1, v2, ego]:
            if vehicle and vehicle.is_alive:
                yaw = math.radians(vehicle.get_transform().rotation.yaw)
                vehicle.set_target_velocity(carla.Vector3D(
                    x=initial_speed_ms * math.cos(yaw),
                    y=initial_speed_ms * math.sin(yaw),
                    z=0.0))

        print("仿真正式开始！等待仿真运行到 2.0 秒触发落叶...")

        v1_traj_idx, ego_traj_idx = 0, 0
        enable_external_takeover = False
        _is_currently_taken_over = False

        # =================进入主循环=================
        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            # # ----------------------------------------
            # # 【完美触发落叶系统：基于动态距离】
            # # ----------------------------------------
            # if active_vehicles['ego'] and ego.is_alive and not leaf_manager.has_spawned:
            #     # 计算 Ego 车距离树叶目标落点的平面距离
            #     dist_to_leaf_target = ego.get_location().distance(leaf_manager.target_point)
            #
            #     # 80km/h = 22.2m/s。设置在距离落点 45 米时触发（大约提前 2 秒）
            #     # 这个距离参数(45.0)您可以微调：设大一点(比如55)，树叶就更早掉在地上等车；
            #     # 设小一点(比如35)，树叶刚好砸在车的挡风玻璃上。
            #     if dist_to_leaf_target < 45.0:
            #         print(f"\n[触发器] Ego 距落点 {dist_to_leaf_target:.1f}m，触发狂风落叶！")
            #         leaf_manager.spawn_leaves()
            #
            # # 每帧刷新树叶受力
            # leaf_manager.tick(sim_time)
            # # ----------------------------------------

            # ----------------------------------------
            # 【落叶系统模块】
            # 因为悬挂贴地占用了前 1.5s，故树叶设置在 0.0s 触发
            # ----------------------------------------
            if sim_time >= 0.0 and not leaf_manager.has_spawned:
                leaf_manager.spawn_leaves()

            # 每帧刷新树叶受力
            leaf_manager.tick(sim_time)
            # ----------------------------------------

            # ==========================
            # 全局出界检测 (Off-Road Check)
            # ==========================
            if active_vehicles['v1'] and v1.is_alive:
                if check_and_handle_out_of_bounds(v1, carla_map, "V1"): active_vehicles['v1'] = False
            if active_vehicles['v2'] and v2.is_alive:
                if check_and_handle_out_of_bounds(v2, carla_map, "V2"): active_vehicles['v2'] = False
            if active_vehicles['ego'] and ego.is_alive:
                if check_and_handle_out_of_bounds(ego, carla_map, "Ego"): active_vehicles['ego'] = False

            # ==========================
            # V1 控制逻辑
            # ==========================
            if active_vehicles['v1'] and v1.is_alive:
                if v1_traj_idx < len(V1_TRAJECTORY_DATA):
                    tx, ty, tyaw = V1_TRAJECTORY_DATA[v1_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v1_init_loc.z)
                    if v1.get_location().distance(target_loc) < 3.0 and v1_traj_idx < len(V1_TRAJECTORY_DATA) - 1:
                        while v1_traj_idx < len(V1_TRAJECTORY_DATA) - 1:
                            v1_traj_idx += 1
                            if v1.get_location().distance(carla.Location(x=V1_TRAJECTORY_DATA[v1_traj_idx][0],
                                                                         y=V1_TRAJECTORY_DATA[v1_traj_idx][1],
                                                                         z=v1_init_loc.z)) > 2.0: break
                    apply_pid_control(v1, pids['v1']['lon'], pids['v1']['lat'],
                                      50.0 if v1.get_location().y <= -200.0 else 80.0, target_loc)
                else:
                    v1.apply_control(carla.VehicleControl(brake=1.0))

            # ==========================
            # Ego 控制逻辑
            # ==========================
            if active_vehicles['ego'] and ego.is_alive:
                if ego_traj_idx < len(EGO_TRAJECTORY_DATA):
                    tx, ty, tyaw = EGO_TRAJECTORY_DATA[ego_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=ego_init_loc.z)
                    if ego.get_location().distance(target_loc) < 3.0 and ego_traj_idx < len(EGO_TRAJECTORY_DATA) - 1:
                        while ego_traj_idx < len(EGO_TRAJECTORY_DATA) - 1:
                            ego_traj_idx += 1
                            if ego.get_location().distance(carla.Location(x=EGO_TRAJECTORY_DATA[ego_traj_idx][0],
                                                                          y=EGO_TRAJECTORY_DATA[ego_traj_idx][1],
                                                                          z=ego_init_loc.z)) > 2.0: break

                    desired_speed_ego = 40.0 if ego.get_location().y <= -175.0 else 80.0

                    if enable_external_takeover:
                        if not _is_currently_taken_over:
                            print(f"\n[{sim_time:.1f}s] Ego: 接管 Flag=True，已切入外部算法接管模式！")
                            _is_currently_taken_over = True
                        ego.apply_control(custom_ad_algorithm_step(ego, target_loc, None))
                    else:
                        if _is_currently_taken_over:
                            print(f"\n[{sim_time:.1f}s] Ego: 接管 Flag=False，已恢复 PID 默认循迹模式！")
                            _is_currently_taken_over = False
                        apply_pid_control(ego, pids['ego']['lon'], pids['ego']['lat'], desired_speed_ego, target_loc)
                else:
                    ego.apply_control(carla.VehicleControl(brake=1.0))

            # 保持固定帧率计算/dt=0.05
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        # ==========================
        # 释放所有内存资源
        # ==========================
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