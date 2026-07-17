import carla
import time
import math
import numpy as np


# ==========================================
# 1. 基础控制算法 (PID) - 核心保留
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


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

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed, target_wp_loc):
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed, speed)
    steer_output = pid_lat.run_step(target_wp_loc, tf)

    if abs(steer_output) < 0.02:
        steer_output = 0.0

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
# 2. 轨迹数据解析工具
# ==========================================
def parse_trajectory(raw_text):
    """将文本形式的轨迹数据解析为 (x, y, yaw) 列表"""
    lines = raw_text.strip().split('\n')
    trajectory = []
    for line in lines:
        if "Location" in line or not line.strip(): continue
        parts = line.split()
        if len(parts) >= 3:
            trajectory.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return trajectory


# 原始轨迹数据 (直接嵌入以保持代码独立性)
MOTO_DATA = """
26.000  30.555  3.444
28.000  30.855  3.666
30.443  31.451  4.000
33.34	32.271	4.288
35.706	32.468	5.008
39.558	32.871	7.59
43.388	33.454	9.162
47.09	34.051	9.162
50.86	34.614	8.172
54.694	35.121	7.106
58.541	35.59	6.893
62.264	36.04	6.893
66.115	36.466	5.968
69.909	36.845	5.4
73.645	37.184	5.117
77.881	37.751	10.189
82.296	38.896	27.806
86.477	42.46	57.953
88.305	47.138	73.79
90.216	52.923	70.867
93.829	55.86	13.992
102.993	56.879	2.472
118.403	57.258	0.702
138.977	56.733	-4.621
159.152	55.14	-4.481
179.362	52.931	-8.051
195.399	50.31	-8.787
"""

BIKE_DATA = """
85.966	53.03	4.229
93.655	53.523	2.233
101.402	53.744	0.382
109.027	53.697	1.543
116.345	55.715	25.064
123.903	56.676	-0.12
131.585	56.568	-0.967
139.27	56.436	-1.252
146.88	56.091	-3.981
154.548	55.557	-3.981
162.088	54.978	-5.035
169.172	54.221	-6.321
"""

TRUCK_DATA = """
11.35	39.908	12.643
21.319	42.101	10.811
31.406	43.656	9.449
41.546	45.415	9.527
51.619	46.754	5.062
61.695	47.745	5.556
71.868	48.722	4.147
79.566	49.106	1.8
79.566	49.106	1.8
79.566	49.106	1.8
84.471	52.252	13.209
94.655	53.156	1.572
104.474	51.208	-17.507
114.557	50.081	0.849
124.766	50.163	-0.69
135.048	49.806	-1.956
145.29	49.384	-2.45
155.393	48.754	-6.413
165.553	47.408	-6.779
175.742	46.303	-5.509
185.867	45.006	-8.152
195.895	43.536	-9.209
205.821	41.749	-10.556
215.769	39.865	-10.56
225.676	38.012	-10.842
236.395	35.807	-12.686
"""


# ==========================================
# 3. 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # ------------------------------------------
    # 按照截图精准设置天气参数
    # ------------------------------------------
    weather = carla.WeatherParameters(
        cloudiness=10.0,  # Clouds
        precipitation=0.0,  # Rain
        precipitation_deposits=60.0,  # Puddles
        wind_intensity=100.0,  # Wind
        sun_azimuth_angle=350.0,  # Sun Azim
        sun_altitude_angle=18.0,  # Sun Alt
        fog_density=0.0,  # Fog Dens
        fog_distance=0.0,  # Fog Dist
        fog_falloff=0.0,  # Fog Fall
        wetness=50.0,  # Wetness
        scattering_intensity=0.0,  # Scatter
        mie_scattering_scale=0.0,  # Mie
        rayleigh_scattering_scale=0.05  # Rayleigh
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    # Traffic Manager 配置 (用于丰田普锐斯)
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm.global_percentage_speed_difference(0.0)

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        # PID 控制器字典
        pids = {
            'moto': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'bike': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'truck': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        }

        # 解析轨迹数据
        moto_traj = parse_trajectory(MOTO_DATA)
        bike_traj = parse_trajectory(BIKE_DATA)
        truck_traj = parse_trajectory(TRUCK_DATA)

        # 记录当前寻迹目标点索引
        idx = {'moto': 0, 'bike': 0, 'truck': 0}

        # ==========================================
        # 车辆 1: 摩托车 Harley Davidson
        # ==========================================
        bp_moto = bp_lib.find('vehicle.harley-davidson.low_rider')
        loc_moto = carla.Location(x=moto_traj[0][0], y=moto_traj[0][1], z=0.5)
        rot_moto = carla.Rotation(yaw=moto_traj[0][2])
        moto = world.try_spawn_actor(bp_moto, carla.Transform(loc_moto, rot_moto))
        if moto:
            actor_list.append(moto)
            print("摩托车 (Harley) 生成成功，目标速度 40km/h")

        # ==========================================
        # 车辆 2: 自行车 Diamondback
        # ==========================================
        bp_bike = bp_lib.find('vehicle.diamondback.century')
        loc_bike = carla.Location(x=bike_traj[0][0], y=bike_traj[0][1], z=0.5)
        rot_bike = carla.Rotation(yaw=bike_traj[0][2])
        bike = world.try_spawn_actor(bp_bike, carla.Transform(loc_bike, rot_bike))
        if bike:
            actor_list.append(bike)
            print("自行车 (Diamondback) 生成成功，目标速度 20km/h")

        # ==========================================
        # 车辆 3: 卡车 (使用标准卡车蓝图替代自定义静态资产)
        # ==========================================
        bp_truck = bp_lib.find('vehicle.carlamotors.carlacola')
        loc_truck = carla.Location(x=truck_traj[0][0], y=truck_traj[0][1], z=1.0)
        rot_truck = carla.Rotation(yaw=truck_traj[0][2])
        truck = world.try_spawn_actor(bp_truck, carla.Transform(loc_truck, rot_truck))
        if truck:
            actor_list.append(truck)
            print("卡车 (Carlacola) 生成成功，目标速度 60km/h，包含停止等待逻辑")

        truck_state = "MOVING"
        truck_wait_start = 0.0

        # ==========================================
        # 车辆 4: 丰田 普锐斯 (TM控制)
        # ==========================================
        bp_prius = bp_lib.find('vehicle.toyota.prius')
        if bp_prius.has_attribute('role_name'):
            bp_prius.set_attribute('role_name', 'ego')
        if bp_prius.has_attribute('color'):
            bp_prius.set_attribute('color', '154,205,50')
        loc_prius = carla.Location(x=-7.668, y=39.253, z=0.5)
        # 从地图获取该位置的合适偏航角
        prius_wp = world.get_map().get_waypoint(loc_prius)
        prius = world.try_spawn_actor(bp_prius, carla.Transform(loc_prius, prius_wp.transform.rotation))
        if prius:
            actor_list.append(prius)
            prius.set_autopilot(True, tm.get_port())
            # 设置超速
            tm.vehicle_percentage_speed_difference(prius, -20.0)
            # 避障关闭：无视前车、无视行人
            tm.ignore_vehicles_percentage(prius, 100)
            tm.ignore_walkers_percentage(prius, 100)
            # 跟车距离：0米
            tm.distance_to_leading_vehicle(prius, 0.0)
            print("丰田 (Prius) 生成成功，已托管至 TM")

        print("\n车辆加载完毕，等待物理稳定...")
        ego_goal_location = carla.Location(x=143.397, y=52.818, z=0.5)
        ego_goal_radius_m = 3.0

        def destroy_all_scene_actors(reason):
            print(reason)
            live_actors = [actor for actor in actor_list if actor and actor.is_alive]
            if live_actors:
                client.apply_batch([carla.command.DestroyActor(actor) for actor in live_actors])
            actor_list.clear()

        for _ in range(20): world.tick()
        print("仿真正式开始...")

        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            if prius and prius.is_alive:
                prius_loc = prius.get_location()
                if math.hypot(prius_loc.x - ego_goal_location.x, prius_loc.y - ego_goal_location.y) <= ego_goal_radius_m:
                    destroy_all_scene_actors("[RTB084] Ego reached goal; cleaning all scene actors and exiting.")
                    prius = moto = bike = truck = None
                    break

            # ==========================
            # 摩托车寻迹控制 (40 km/h)
            # ==========================
            if moto and moto.is_alive and idx['moto'] < len(moto_traj):
                curr_loc = moto.get_location()
                target_pt = moto_traj[idx['moto']]
                target_loc = carla.Location(x=target_pt[0], y=target_pt[1], z=0.5)

                # 到达目标点附近，切换下一个点
                if curr_loc.distance(target_loc) < 2.5:
                    idx['moto'] += 1

                if idx['moto'] < len(moto_traj):
                    target_pt = moto_traj[idx['moto']]
                    target_loc = carla.Location(x=target_pt[0], y=target_pt[1], z=0.5)
                    apply_pid_control(moto, pids['moto']['lon'], pids['moto']['lat'], 30.0, target_loc)
            elif moto and moto.is_alive:
                moto.apply_control(carla.VehicleControl(brake=1.0))  # 终点刹车

            # ==========================
            # 自行车寻迹控制 (20 km/h)
            # ==========================
            if bike and bike.is_alive and idx['bike'] < len(bike_traj):
                curr_loc = bike.get_location()
                target_pt = bike_traj[idx['bike']]
                target_loc = carla.Location(x=target_pt[0], y=target_pt[1], z=0.5)

                if curr_loc.distance(target_loc) < 2.0:
                    idx['bike'] += 1

                if idx['bike'] < len(bike_traj):
                    target_pt = bike_traj[idx['bike']]
                    target_loc = carla.Location(x=target_pt[0], y=target_pt[1], z=0.5)
                    apply_pid_control(bike, pids['bike']['lon'], pids['bike']['lat'], 20.0, target_loc)
            elif bike and bike.is_alive:
                bike.apply_control(carla.VehicleControl(brake=1.0))

            # ==========================
            # 卡车寻迹与启停控制 (60 km/h)
            # ==========================
            if truck and truck.is_alive and idx['truck'] < len(truck_traj):
                curr_loc = truck.get_location()
                target_pt = truck_traj[idx['truck']]
                target_loc = carla.Location(x=target_pt[0], y=target_pt[1], z=0.5)

                # 计算与停车点(x=79.566, y=49.106)的距离
                stop_loc = carla.Location(x=79.566, y=49.106, z=0.5)
                dist_to_stop = curr_loc.distance(stop_loc)

                target_speed = 60.0

                if truck_state == "MOVING":
                    # 距离停止点不足25米时，开始合理减速
                    if dist_to_stop < 25.0 and curr_loc.x < 79.5:
                        target_speed = max(0.0, 60.0 * (dist_to_stop / 25.0))

                        # 完全停下，切换到等待状态
                        if dist_to_stop < 1.5 and truck.get_velocity().length() < 0.5:
                            truck_state = "WAITING"
                            truck_wait_start = sim_time
                            target_speed = 0.0
                            print("==> 卡车已到达目标点，刚刚停稳，开始等待 10 秒...")

                elif truck_state == "WAITING":
                    target_speed = 0.0
                    if sim_time - truck_wait_start >= 10.0:
                        truck_state = "RESUMING"
                        print("==> 卡车等待结束，继续循迹行驶...")
                        # 略过轨迹中停留在原地的冗余点
                        while idx['truck'] < len(truck_traj) and abs(truck_traj[idx['truck']][0] - 79.566) < 1.0:
                            idx['truck'] += 1

                elif truck_state == "RESUMING":
                    target_speed = 60.0  # 恢复目标速度，PID会自动计算合理加速度

                # 正常推进循迹索引
                if curr_loc.distance(target_loc) < 3.0:
                    idx['truck'] += 1

                if idx['truck'] < len(truck_traj):
                    target_pt = truck_traj[idx['truck']]
                    target_loc = carla.Location(x=target_pt[0], y=target_pt[1], z=0.5)
                    apply_pid_control(truck, pids['truck']['lon'], pids['truck']['lat'], target_speed, target_loc)
            elif truck and truck.is_alive:
                truck.apply_control(carla.VehicleControl(brake=1.0))

            # 保持实时帧率同步
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        # 恢复TM同步模式
        tm.set_synchronous_mode(False)

        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()
        print("清理完毕。")


if __name__ == '__main__':
    main()
