import carla
import time
import math
import numpy as np

# ==========================================
# 轨迹数据定义 (将您提供的数据转为长字符串便于解析)
# ==========================================
TRAJECTORY_1_DATA = """
-51.374 -93.785
-50.872 -91.592
-49.881 -87.846
-48.732 -84.211
-47.477 -80.545
-46.253 -77.001
-44.877 -73.378
-43.529 -69.879
-42.095 -66.414
-40.46 -63.039
-38.73 -59.571
-37.003 -56.102
-35.307 -52.688
-33.611 -49.273
-31.717 -45.965
-29.805 -42.96
-28.331 -40.712
-26.151 -37.509
-24.038 -34.41
-23.653 -33.841
-21.58 -30.716
-19.197 -27.661
-16.747 -24.824
-14.004 -22.087
-11.237 -19.556
-10.865 -19.223
-8.308 -17.115
-5.925 -14.641
-3.723 -11.458
-1.777 -8.252
-1.339 -7.495
-1.187 -4.628
-1.546 -0.896
-1.903 2.835
-2.066 6.707
-2.161 10.517
-2.125 14.393
-2.089 18.143
-2.089 21.893
-2.118 25.768
-2.176 29.58
-2.241 33.392
-2.312 37.141
-2.378 40.953
-2.427 44.765
-2.474 48.515
-2.504 52.265
-2.494 56.14
-2.465 59.952
-2.401 63.826
-2.366 67.704
-2.353 71.579
-2.379 75.392
-2.435 79.266
-2.5 83.141
-2.564 87.015
-2.627 90.828
-2.673 94.64
-2.692 98.515
-2.675 102.329
-2.641 106.205
-2.578 110.079
-2.508 113.894
-2.43 117.705
-2.351 121.517
-2.274 125.267
-2.209 128.454
-2.165 130.454
-2.029 134.322
-1.866 138.152
-1.832 138.954
"""

TRAJECTORY_2_DATA = """
-56.726 -129.732
-56.694 -127.799
-56.512 -123.979
-56.236 -120.239
-55.948 -116.376
-55.417 -112.665
-54.78 -108.843
-54.039 -105.104
-53.211 -101.319
-52.376 -97.663
-51.521 -93.947
-50.628 -90.305
-49.586 -86.703
-48.486 -83.118
-47.291 -79.564
-45.929 -75.936
-44.477 -72.344
-42.943 -68.785
-41.379 -65.24
-39.836 -61.755
-38.206 -58.239
-36.423 -54.797
-34.562 -51.542
-32.702 -48.285
-30.801 -44.98
-28.928 -41.732
-27.008 -38.438
-24.995 -35.127
-24.098 -33.772
-23.919 -33.517
-21.76 -30.45
-19.416 -27.524
-16.825 -24.73
-14.016 -22.153
-10.997 -19.937
-7.564 -18.295
-7.086 -18.148
-4.893 -17.416
-1.158 -16.411
2.646 -16.449
6.432 -17.221
9.917 -18.596
13.19 -20.426
16.265 -22.773
18.902 -25.522
21.23 -28.62
23.425 -31.661
25.456 -34.813
27.515 -38.022
29.668 -41.169
31.849 -44.296
34.03 -47.424
36.249 -50.601
38.477 -53.771
40.697 -56.871
42.962 -60.015
45.136 -63.07
47.245 -66.173
49.345 -69.357
51.418 -72.482
53.552 -75.565
55.823 -78.705
58.137 -81.734
60.559 -84.759
63.023 -87.75
65.501 -90.73
67.952 -93.649
70.468 -96.598
72.922 -99.435
75.506 -102.322
78.059 -105.07
80.758 -107.85
83.384 -110.527
86.114 -113.277
88.79 -115.904
91.493 -118.504
94.269 -121.118
97.095 -123.769
99.835 -126.329
102.69 -128.949
105.468 -131.468
108.398 -134.014
111.295 -136.492
114.24 -139.01
117.185 -141.528
120.083 -144.006
122.933 -146.443
125.878 -148.961
128.75 -151.372
131.841 -153.708
134.957 -155.907
138.185 -158.051
141.468 -160.11
144.727 -161.964
148.146 -163.787
151.483 -165.498
154.929 -167.13
158.463 -168.719
161.931 -170.144
165.556 -171.512
169.244 -172.701
172.907 -173.758
176.536 -174.704
180.295 -175.643
184.072 -176.511
187.888 -177.179
191.668 -177.673
195.399 -178.052
199.141 -178.305
202.946 -178.541
206.759 -178.757
210.632 -178.864
214.445 -178.856
218.257 -178.794
222.068 -178.718
225.942 -178.64
229.816 -178.563
233.69 -178.485
237.501 -178.425
241.25 -178.387
245.124 -178.349
248.867 -178.311
252.742 -178.272
256.617 -178.221
260.366 -178.184
264.1 -178.214
266.645 -178.25
268.371 -178.299
268.678 -178.309
269.539 -178.334
269.748 -178.342
"""


# ==========================================
# PID 控制器类 (保留原逻辑)
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController:
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
def parse_trajectory(data_str):
    """解析多行字符串坐标，去重后返回[(x, y), ...]格式的轨迹点列表"""
    traj = []
    lines = data_str.strip().split('\n')
    for line in lines:
        parts = line.strip().split()
        if len(parts) >= 2:
            pt = (float(parts[0]), float(parts[1]))
            # 简单去重
            if not traj or (abs(traj[-1][0] - pt[0]) > 0.1 or abs(traj[-1][1] - pt[1]) > 0.1):
                traj.append(pt)
    return traj

def get_lane_keeping_waypoint(carla_map, vehicle_loc, lookahead_dist=6.0):
    """动态利用CARLA地图提取车道中心点进行车道保持"""
    current_wp = carla_map.get_waypoint(vehicle_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    next_wps = current_wp.next(lookahead_dist)
    if next_wps:
        loc = next_wps[0].transform.location
        return (loc.x, loc.y, loc.z)
    return (current_wp.transform.location.x, current_wp.transform.location.y, current_wp.transform.location.z)


def get_target_from_trajectory(vehicle_loc, trajectory, lookahead_dist=12.0):
    """从给定的(x,y)硬编码轨迹列表中获取前方目标点，并判断是否偏离路线或到达终点"""
    if not trajectory:
        return None, True, True

    # 1. 寻找最近的轨迹点
    min_dist = float('inf')
    closest_idx = 0
    for i, pt in enumerate(trajectory):
        d = math.hypot(vehicle_loc.x - pt[0], vehicle_loc.y - pt[1])
        if d < min_dist:
            min_dist = d
            closest_idx = i

    # 偏差大于 8 米，判定为跑出道路
    is_off_road = min_dist > 8.0

    # 2. 沿着轨迹往前方找 lookahead_dist 米的目标点
    target_idx = closest_idx
    dist_accum = 0.0
    while target_idx < len(trajectory) - 1 and dist_accum < lookahead_dist:
        pt1 = trajectory[target_idx]
        pt2 = trajectory[target_idx + 1]
        dist_accum += math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])
        target_idx += 1

    # 是否到达终点
    is_finished = target_idx >= len(trajectory) - 1

    # 返回目标坐标(x, y, 当前Z)
    target_pt = trajectory[target_idx]
    return (target_pt[0], target_pt[1], vehicle_loc.z), is_finished, is_off_road


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed, target_wp):
    """通用的PID控制器执行流程"""
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed, speed)
    steer_output = pid_lat.run_step(target_wp, tf)

    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)

    vehicle.apply_control(control)


def set_vehicle_lights(vehicle):
    """强制开启近光灯和示宽灯"""
    light_state = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
    vehicle.set_light_state(carla.VehicleLightState(light_state))

def get_proper_spawn_transform(world, x, y):
    loc = carla.Location(x=x, y=y, z=0.0)
    waypoint = world.get_map().get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    trans = waypoint.transform
    trans.location.z += 0.5
    return trans
# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # 获取 Traffic Manager
    tm = client.get_trafficmanager(8000)
    tm_port = tm.get_port()

    # --------------------------
    # 1. 设置严格对应截图的天气参数
    # --------------------------
    weather = carla.WeatherParameters(
        cloudiness=25.0,
        precipitation=60.0,
        precipitation_deposits=60.0,  # Puddles
        wind_intensity=100.0,
        sun_azimuth_angle=110.0,
        sun_altitude_angle=8.0,
        fog_density=44.0,
        fog_distance=0.0,
        fog_falloff=0.0,
        wetness=60.0,
        scattering_intensity=20.0,  # Scatter
        mie_scattering_scale=0.0,  # Mie
        rayleigh_scattering_scale=0.08  # Rayleigh
    )
    world.set_weather(weather)

    bp_lib = world.get_blueprint_library()
    actor_list = []

    # 车辆当前目标速度字典，用于平滑减速
    current_speeds = {'v1': 60.0, 'v2': 60.0}

    try:
        # 同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        tm.set_synchronous_mode(True)

        # 定义使用的PID控制器 (大卡车转向K_P设为1.2防侧滑)
        pids = {
            'v1': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController(K_P=1.2)},
            'v2': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController(K_P=1.2)},
            'jeep': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()}
        }

        # 解析轨迹数据
        traj_1 = parse_trajectory(TRAJECTORY_1_DATA)
        traj_2 = parse_trajectory(TRAJECTORY_2_DATA)

        print("正在生成车辆...")

        # ====================
        # 1. 生成第一辆车 (红头HGV)
        # ====================
        bp_v1 = bp_lib.find('vehicle.carlamotors.european_hgv')
        if bp_v1.has_attribute('color'):
            bp_v1.set_attribute('color', '255,0,0')  # 红色
        # 根据轨迹第一点生成
        spawn_yaw_1 = 76.088  # 从数据中取得初始偏航角
        trans_v1 = carla.Transform(carla.Location(x=-51.374, y=-93.785, z=20), carla.Rotation(yaw=spawn_yaw_1))
        v1 = world.try_spawn_actor(bp_v1, trans_v1)
        if v1: actor_list.append(v1); print("1. 红色 HGV 生成成功")

        # ====================
        # 2. 生成第二辆车 (黄头HGV)
        # ====================
        bp_v2 = bp_lib.find('vehicle.carlamotors.european_hgv')
        if bp_v2.has_attribute('color'):
            bp_v2.set_attribute('color', '255,255,0')  # 黄色
        spawn_yaw_2 = 89.005  # 从数据中取得初始偏航角
        trans_v2 = carla.Transform(carla.Location(x=-56.726, y=-129.732, z=20), carla.Rotation(yaw=spawn_yaw_2))
        v2 = world.try_spawn_actor(bp_v2, trans_v2)
        if v2: actor_list.append(v2); print("2. 黄色 HGV 生成成功")

        # ====================
        # 3. 生成第三辆车 (TM控制)
        # ====================
        bp_v3 = bp_lib.filter('vehicle.audi.*')[0]  # 选一辆常见车
        trans_v3 = carla.Transform(carla.Location(x=1.438, y=113.258, z=10), carla.Rotation(yaw=0.0))
        # 因为强制生成坐标可能未对准路面，用 get_waypoint 对齐一下
        wp_v3 = world.get_map().get_waypoint(trans_v3.location, project_to_road=True)
        v3_trans = wp_v3.transform
        v3_trans.location.z += 0.5
        v3 = world.try_spawn_actor(bp_v3, v3_trans)

        if v3:
            actor_list.append(v3)
            print("3. TM控制车辆 生成成功")
            # --- 严格遵循您的 TM 规则配置 ---
            v3.set_autopilot(True, tm_port)
            tm.vehicle_percentage_speed_difference(v3, -55.0)  # 超速
            tm.ignore_lights_percentage(v3, 0)  # 不忽略红绿灯
            tm.ignore_signs_percentage(v3, 0)  # 不忽略标志
            tm.ignore_vehicles_percentage(v3, 0)  # 不忽略车辆(100%避让)
            tm.ignore_walkers_percentage(v3, 0)  # 不忽略行人(100%避让)
            tm.distance_to_leading_vehicle(v3, 5.0)  # 跟车距离5米
            tm.auto_lane_change(v3, True)  # 允许自动变道

        # 4. 车辆4: vehicle.jeep.wrangler_rubicon (新增：橙色)
        bp_jeep = bp_lib.find('vehicle.jeep.wrangler_rubicon')
        if bp_jeep.has_attribute('color'):
            bp_jeep.set_attribute('color', '255,100,0')  # CARLA 标准橙色
        trans_jeep = get_proper_spawn_transform(world, x=34.843, y=72.746)
        jeep = world.try_spawn_actor(bp_jeep, trans_jeep)
        if jeep:
            actor_list.append(jeep)
            print("4. jeep 生成成功 (橙色，PID自动搜寻前方锚点循迹，初始30km/h)")

        print("初始化物理系统并开始主循环...")
        for _ in range(20): world.tick()

        # --- 为 vehicle.jeep.wrangler_rubicon 赋予物理初速度 100km/h  ---
        if jeep and jeep.is_alive:
            forward_vec = jeep.get_transform().get_forward_vector()
            initial_speed_mps = 30.0 / 3.6
            jeep.set_target_velocity(carla.Vector3D(
                forward_vec.x * initial_speed_mps,
                forward_vec.y * initial_speed_mps,
                forward_vec.z * initial_speed_mps
            ))

        # 车辆销毁标记
        v1_destroyed = False
        v2_destroyed = False

        while True:
            start_time = time.time()
            world.tick()

            # ==========================
            # 控制 1: 红色 HGV
            # ==========================
            if v1 and v1.is_alive and not v1_destroyed:
                set_vehicle_lights(v1)
                loc = v1.get_location()

                # 获取目标点并检查状态
                # HGV 较重，速度 60 时的前瞻距离设为 12 米左右比较平顺
                target_wp, finished, off_road = get_target_from_trajectory(loc, traj_1, lookahead_dist=12.0)

                if finished or off_road:
                    reason = "到达终点" if finished else "偏离道路"
                    print(f"红色 HGV 因 [{reason}] 被销毁。")
                    v1.destroy()
                    v1_destroyed = True
                else:
                    # 速度逻辑：基础60，过 y=-30 后减速到30
                    target_speed = 60.0
                    if loc.y > -30.0:
                        target_speed = 30.0

                    # 平滑减速逻辑 (最大减速度约 -3m/s^2 模拟卡车刹车)
                    if current_speeds['v1'] > target_speed:
                        current_speeds['v1'] = max(target_speed, current_speeds['v1'] - 0.5)
                    else:
                        current_speeds['v1'] = target_speed

                    apply_pid_control(v1, pids['v1']['lon'], pids['v1']['lat'], current_speeds['v1'], target_wp)

            # ==========================
            # 控制 2: 黄色 HGV
            # ==========================
            if v2 and v2.is_alive and not v2_destroyed:
                set_vehicle_lights(v2)
                loc = v2.get_location()

                target_wp, finished, off_road = get_target_from_trajectory(loc, traj_2, lookahead_dist=12.0)

                if finished or off_road:
                    reason = "到达终点" if finished else "偏离道路"
                    print(f"黄色 HGV 因 [{reason}] 被销毁。")
                    v2.destroy()
                    v2_destroyed = True
                else:
                    # 速度逻辑：基础60，过 y=-20 后减速到30
                    target_speed = 60.0
                    # 对于V2的轨迹，Y是从大负数走向0再走向正/负，确保触发逻辑
                    if loc.y > -20.0:
                        target_speed = 30.0

                    # 平滑减速逻辑
                    if current_speeds['v2'] > target_speed:
                        current_speeds['v2'] = max(target_speed, current_speeds['v2'] - 0.5)
                    else:
                        current_speeds['v2'] = target_speed

                    apply_pid_control(v2, pids['v2']['lon'], pids['v2']['lat'], current_speeds['v2'], target_wp)

            # ==========================
            # 控制 3: TM 控制的车辆
            # ==========================
            # 交给 Traffic Manager 控制，此处仅作状态监测
            if v3 and v3.is_alive:
                set_vehicle_lights(v3)

            # 若三辆车均已销毁，可选择退出循环
            # if v1_destroyed and v2_destroyed and (not v3 or not v3.is_alive):
            #    break

            # ==========================
            # 控制 5: jeep TT (恒定自动搜索前方锚点循迹)
            # ==========================
            if jeep and jeep.is_alive:
                # 动态提取前方 6 米的道路中心锚点进行车道保持
                target_wp = get_lane_keeping_waypoint(carla_map, jeep.get_location(), lookahead_dist=6.0)
                apply_pid_control(jeep, pids['jeep']['lon'], pids['jeep']['lat'], 20.0, target_wp)

            # --- 帧率同步 ---
            compute_time = time.time() - start_time
            if compute_time < 0.05:
                time.sleep(0.05 - compute_time)

    except KeyboardInterrupt:
        print("\n用户按 Ctrl+C 停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        tm.set_synchronous_mode(False)

        # 销毁仍然存活的车辆
        for act in actor_list:
            if act and act.is_alive:
                act.destroy()
        print("清理完成。")


if __name__ == '__main__':
    main()