# -*- coding: utf-8 -*-
import sys
import carla
import time
import math

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

import RoadTailBenchInitV9 as RTB

def _rtb_actor_alive(actor):
    return bool(actor is not None and getattr(actor, "is_alive", False))

# ==========================================
# 轨迹数据硬编码区域
# ==========================================
RAW_PATH_A2 = [
    (-27.097, 54.855, -93.617), (-27.097, 54.855, -93.617), (-27.097, 54.855, -93.617),
    (-27.097, 54.855, -93.617), (-27.097, 54.855, -93.401), (-27.097, 54.855, -92.474),
    (-27.107, 54.606, -92.261), (-27.188, 52.024, -91.269), (-27.228, 49.525, -90.051),
    (-27.23, 47.025, -90.194), (-27.246, 44.523, -90.406), (-27.264, 41.984, -90.406),
    (-27.281, 39.484, -90.406), (-27.297, 36.984, -90.336), (-27.312, 34.483, -90.479),
    (-27.334, 31.899, -90.479), (-27.353, 29.316, -90.409), (-27.368, 26.816, -90.339),
    (-27.383, 24.274, -90.339), (-27.418, 21.691, -91.467), (-27.532, 19.111, -93.416),
    (-27.661, 16.531, -92.285), (-27.736, 13.949, -91.226), (-27.777, 11.366, -90.871),
    (-27.823, 8.866, -91.293), (-27.879, 6.367, -91.293), (-27.936, 3.825, -91.293),
    (-27.982, 1.325, -90.873), (-28.013, -1.258, -90.661), (-28.042, -3.8, -90.661),
    (-28.071, -6.3, -90.661), (-28.1, -8.8, -90.661), (-28.129, -11.341, -90.661),
    (-28.158, -13.842, -90.661), (-28.154, -16.425, -88.832), (-28.008, -18.95, -84.975),
    (-27.775, -21.481, -84.048), (-27.47, -23.963, -82.682), (-27.438, -26.537, -97.528),
    (-27.876, -29.039, -100.148), (-28.353, -31.577, -101.296), (-28.858, -34.024, -101.934),
    (-29.383, -36.509, -101.864), (-29.796, -38.973, -98.812), (-30.238, -41.517, -100.256),
    (-30.683, -44.06, -98.983), (-31.007, -46.539, -96.04), (-31.206, -49.03, -92.636),
    (-31.321, -51.527, -92.636), (-31.427, -54.108, -91.426), (-31.461, -56.608, -90.705),
    (-31.493, -59.191, -90.705), (-31.514, -61.774, -90.492), (-31.536, -64.357, -90.492),
    (-31.558, -66.857, -90.492), (-31.58, -69.44, -90.492), (-31.598, -71.94, -90.422),
    (-31.616, -74.442, -90.282), (-31.617, -76.985, -89.856), (-31.61, -79.568, -89.856),
    (-31.609, -82.151, -90.139), (-31.629, -84.651, -90.704), (-31.67, -87.192, -91.057),
    (-31.707, -89.692, -90.707), (-31.739, -92.285, -90.707), (-31.769, -94.785, -90.707),
    (-31.809, -97.328, -91.277), (-31.865, -99.91, -91.207), (-31.914, -102.491, -90.927),
    (-31.953, -104.991, -90.857), (-31.983, -107.574, -90.507), (-32.006, -110.157, -90.507),
    (-32.028, -112.699, -90.367), (-32.044, -115.24, -90.367), (-32.06, -117.74, -90.367),
    (-32.076, -120.24, -90.367), (-32.092, -122.782, -90.367), (-32.109, -125.365, -90.367),
    (-32.125, -127.906, -90.367), (-32.141, -130.406, -90.367), (-32.157, -132.905, -90.367),
    (-32.171, -135.405, -90.297), (-32.185, -137.989, -90.297), (-32.196, -140.53, -90.154),
    (-32.194, -143.071, -89.729), (-32.182, -145.613, -89.729), (-32.17, -148.197, -89.729),
    (-32.167, -150.696, -91.533), (-32.406, -153.223, -103.667), (-33.388, -155.6, -125.558),
    (-35.049, -157.523, -131.509), (-35.38, -157.897, -131.509)
]

RAW_PATH_TRUCK = [
    (-23.241, 52.669, -94.819), (-23.241, 52.669, -94.819), (-23.283, 52.172, -94.819),
    (-23.484, 49.644, -93.54), (-23.568, 47.148, -90.599), (-23.586, 44.603, -90.882),
    (-23.637, 42.103, -91.302), (-23.692, 39.602, -90.882), (-23.732, 37.061, -90.882),
    (-23.764, 34.519, -90.602), (-23.801, 32.02, -91.024), (-23.852, 29.52, -91.526),
    (-23.919, 27.021, -91.455), (-23.967, 24.438, -90.896), (-24.006, 21.938, -90.896),
    (-24.046, 19.396, -90.896), (-24.086, 16.812, -90.896), (-24.119, 14.226, -90.686),
    (-24.143, 11.724, -90.333), (-24.157, 9.223, -90.333), (-24.192, 6.64, -91.038),
    (-24.214, 4.098, -89.68), (-24.2, 1.597, -89.392), (-24.132, -0.985, -86.907),
    (-23.974, -3.509, -84.149), (-23.933, -3.882, -83.602), (-23.933, -3.882, -84.91),
    (-23.933, -3.882, -92.294), (-23.965, -5.381, -90.72), (-23.933, -7.922, -86.659),
    (-23.654, -10.45, -81.251), (-23.654, -10.45, -80.531), (-23.654, -10.45, -78.664),
    (-23.654, -10.45, -76.638), (-23.272, -12.029, -76.425), (-22.665, -14.542, -76.143),
    (-22.033, -16.961, -75.134), (-21.37, -19.372, -73.119), (-20.549, -21.778, -70.795),
    (-19.694, -24.172, -70.159), (-18.804, -26.598, -69.655), (-17.903, -28.93, -66.132),
    (-16.816, -31.275, -64.777), (-15.707, -33.608, -64.209), (-14.606, -35.853, -62.85),
    (-13.41, -38.097, -61.207), (-12.159, -40.357, -59.983), (-10.794, -42.55, -56.957),
    (9.397, -61.332, -39.018), (9.467, -61.324, -38.802), (9.662, -61.478, -34.632),
    (9.992, -61.703, -34.142), (10.375, -61.848, -36.561), (10.696, -62.085, -36.561),
    (11.018, -62.321, -35.931), (11.338, -62.561, -39.318), (11.683, -62.848, -38.438),
    (11.998, -63.096, -38.088), (12.313, -63.342, -37.808), (12.631, -63.584, -37.102),
    (12.952, -63.824, -35.982), (13.279, -64.054, -34.873), (14.263, -64.740, -34.873),
    (14.592, -64.968, -35.154), (14.917, -65.200, -35.504), (15.284, -65.461, -35.154),
    (15.616, -65.684, -32.862), (15.953, -65.900, -31.221), (16.297, -66.104, -30.539),
    (17.029, -66.536, -30.539), (17.377, -66.733, -28.649), (17.730, -66.921, -27.739),
    (18.086, -67.104, -26.689), (18.434, -67.300, -33.124), (18.769, -67.519, -33.054),
    (19.775, -68.172, -32.775), (20.112, -68.387, -32.355), (20.494, -68.625, -31.655),
    (21.175, -69.043, -30.953), (21.517, -69.251, -31.864), (22.543, -69.869, -29.070),
    (23.285, -70.281, -28.511), (23.989, -70.660, -26.544), (24.351, -70.830, -23.416),
    (25.505, -71.305, -21.806), (25.878, -71.449, -20.843), (26.996, -71.878, -21.193),
    (27.740, -72.167, -21.473), (28.530, -72.478, -21.473), (29.275, -72.769, -21.193),
    (30.025, -73.045, -19.673), (30.831, -73.316, -17.454), (31.596, -73.549, -16.404),
    (32.364, -73.772, -15.564), (33.187, -73.986, -14.276), (33.477, -74.059, -14.066)
]

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05
        sim_time = 0.0

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        weather = carla.WeatherParameters(
            cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0, wind_intensity=10.0,
            sun_azimuth_angle=-1.0, sun_altitude_angle=15.0, fog_density=2.0, fog_distance=0.75,
            fog_falloff=0.1, wetness=0.0, scattering_intensity=1.0, mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已设置")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化
        # ==========================================
        # 剥离原数组中的 Yaw 角度（极其重要：防止插值函数将其误识别为 -90 米的地下海拔）
        path_a2_xy = [(p[0], p[1]) for p in RAW_PATH_A2]
        path_truck_xy = [(p[0], p[1]) for p in RAW_PATH_TRUCK]

        # 第一步：去除原始数组中极近的重合点和原点噪音
        traj_a2 = RTB.clean_trajectory(path_a2_xy, min_dist=0.1)
        traj_truck = RTB.clean_trajectory(path_truck_xy, min_dist=0.1)

        # 第二步：将稀疏的锚点稠密化为 0.5米 间距，彻底消除 PID 寻点死角
        traj_a2 = RTB.interpolate_trajectory(traj_a2, interval=0.5)
        traj_truck = RTB.interpolate_trajectory(traj_truck, interval=0.5)

        # ==========================================
        # 3. 车辆实体安全生成
        # ==========================================
        # 使用未剥离的原数组坐标和角度进行精确生成
        a2 = RTB.spawn_vehicle(world, 'vehicle.audi.a2', x=RAW_PATH_A2[0][0], y=RAW_PATH_A2[0][1],
                               yaw=RAW_PATH_A2[0][2], color='0,0,255')
        actor_list.append(a2)

        truck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck', x=RAW_PATH_TRUCK[0][0],
                                  y=RAW_PATH_TRUCK[0][1], yaw=RAW_PATH_TRUCK[0][2], color='255,0,0', z_offset=1.5)
        actor_list.append(truck)

        ego = RTB.spawn_vehicle(world, 'vehicle.citroen.c3', x=-23.472, y=25.188, yaw=-90.0, color='255,255,0',
                                role_name='ego')
        actor_list.append(ego)

        # ==========================================
        # 4. 车辆PID与独立控制器挂载
        # ==========================================
        pid_lon_a2 = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_a2 = RTB.PIDLateralController(preset='default_car')

        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_truck = RTB.PIDLateralController(preset='truck')

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')

        idx_a2, idx_truck = 0, 0

        # ==========================================
        # 5. 车辆灯光管理器
        # ==========================================
        lights_a2 = RTB.VehicleLightManager(a2)
        lights_truck = RTB.VehicleLightManager(truck)
        lights_ego = RTB.VehicleLightManager(ego)

        # ==========================================
        # 6. 剧本状态机编排
        # ==========================================
        sm_a2 = RTB.MultiStageBehaviorMachine(initial_speed=80.0)
        sm_a2.add_stage('y_less', target_speed=120.0, trigger_val=-15.0, accel=10.0)

        sm_truck = RTB.MultiStageBehaviorMachine(initial_speed=70.0)
        sm_truck.add_stage('y_less', target_speed=40.0, trigger_val=-10.0, accel=10.0)

        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=50.0)
        sm_ego.add_stage('y_less', target_speed=25.0, trigger_val=-23.0, accel=20.0)
        sm_ego.add_stage('y_less', target_speed=80.0, trigger_val=-48.3, accel=15.0)

        # ==========================================
        # 7. 预热与初始状态瞬间注入（核心修复区）
        # ==========================================
        # 【关键修复1】必须给重型车辆自由落体时间，防止在空中被赋予初速导致落地摩擦掉速
        print("等待 1 秒，让所有车辆物理模型接触地面并稳定...")
        for _ in range(20):
            world.tick()

        # 【关键修复2】恢复原代码的物理超额注入（100km/h），抵消阻尼，制造呼啸压迫感
        RTB.set_vehicle_initial_speed(a2, target_speed_kmh=100.0)
        RTB.set_vehicle_initial_speed(truck, target_speed_kmh=100.0)
        RTB.set_vehicle_initial_speed(ego, target_speed_kmh=70.0)

        print("\n[场景运行] 仿真开始，车辆PID控制已接管...\n")

        # ==========================================
        # 8. 仿真主循环（帧率同步与环境清理守护）
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- Audi A2 控制逻辑 ----------------
            if not RTB.check_vehicle_out_of_bounds(a2, carla_map, auto_destroy=True):
                spd = sm_a2.tick(a2.get_location(), sim_time, dt)
                wp, idx_a2 = RTB.get_target_waypoint(a2.get_location(), traj_a2, idx_a2, speed_kmh=spd)
                RTB.apply_pid_control(a2, pid_lon_a2, pid_lat_a2, spd, wp)
                lights_a2.auto_update_from_control()

            # ---------------- Firetruck 控制逻辑 ----------------
            if not RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True):
                spd = sm_truck.tick(truck.get_location(), sim_time, dt)
                wp, idx_truck = RTB.get_target_waypoint(truck.get_location(), traj_truck, idx_truck, speed_kmh=spd)
                RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, spd, wp)
                lights_truck.auto_update_from_control()

            # ---------------- Ego 控制逻辑 ----------------
            if not RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                spd = sm_ego.tick(ego.get_location(), sim_time, dt)
                target_wp_loc = RTB.get_random_lane_keeping_waypoint(carla_map, ego.get_location(), lookahead_dist=6.0)
                RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, spd, target_wp_loc)
                lights_ego.auto_update_from_control()

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            if not any(_rtb_actor_alive(actor) for actor in actor_list):
                print("[RoadTailBench] RTB001check actors all destroyed; ending simulation.")
                break

            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[发生异常] {e}")
    finally:
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()
