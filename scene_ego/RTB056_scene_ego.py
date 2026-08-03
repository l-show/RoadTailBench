import sys
import os
import carla
import time

# 1. 动态引入您的标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as rtb

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05

        # 开启同步模式，固定时间步长为 0.05s (20 FPS)
        rtb.enable_synchronous_mode(world, dt=dt)

        # ==========================================
        # 一、天气系统配置 (利用标准化库或直接赋值)
        # ==========================================
        weather = carla.WeatherParameters(
            cloudiness=5.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=10.0,
            sun_azimuth_angle=-1.0,
            sun_altitude_angle=15.0,
            fog_density=2.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已设置为定制的 Sunset 状态。")

        # ==========================================
        # 二、轨迹数据硬编码与清洗
        # ==========================================
        raw_traj_v1 = [
            (26.636, 80.849), (26.636, 80.849), (26.636, 80.849), (26.468, 80.425), (25.98, 79.234), (25.49, 78.044),
            (25.047, 76.878), (24.374, 74.474), (23.882, 72.003), (23.639, 69.455), (23.607, 66.663), (23.559, 61.58),
            (23.558, 56.497), (23.557, 51.413), (23.549, 43.783), (23.513, 38.617), (23.469, 33.617), (23.431, 28.492),
            (23.427, 27.992), (23.427, 27.992), (23.421, 26.118), (23.427, 23.619), (23.522, 18.495), (23.614, 13.37),
            (23.655, 8.287), (23.635, 3.162), (23.609, -1.922), (23.58, -6.963), (23.53, -12.053), (23.432, -17.094),
            (23.308, -22.217), (23.31, -27.259), (23.335, -32.343), (23.361, -37.426), (23.386, -42.51),
            (23.411, -47.593),
            (23.437, -52.678), (23.595, -57.716), (24.757, -62.641), (26.969, -67.151), (29.997, -71.266),
            (33.763, -74.538),
            (38.234, -76.974), (42.957, -78.59), (47.958, -79.479), (53.069, -79.835), (57.066, -79.992)
        ]
        raw_traj_v2 = [
            (123.376, -5.116), (123.376, -5.116), (119.614, -3.442), (108.749, -0.073), (97.37, 1.039), (88.747, 1.215),
            (77.872, 1.254), (66.247, 1.253), (51.435, 1.252), (48.81, 1.25), (37.818, 3.216), (26.652, 4.182),
            (3.612, 1.283),
            (-7.811, 0.716), (-19.059, 0.534), (-30.498, 0.672), (-41.748, 0.804), (-52.997, 0.969), (-64.224, 1.613),
            (-75.737, 3.166), (-86.662, 5.819), (-97.075, 10.048), (-107.01, 16.057), (-116.051, 23.354),
            (-124.403, 31.231),
            (-131.465, 39.969), (-136.833, 50.261), (-141.478, 60.917), (-150.469, 81.543), (-154.932, 91.87),
            (-158.86, 101.406)
        ]
        raw_traj_v3 = [
            (3.734, -8.075), (3.734, -8.075), (6.102, -8.256), (8.585, -8.535), (11.045, -8.979), (13.562, -9.327),
            (16.099, -9.392), (18.655, -9.033), (21.067, -8.384), (23.507, -7.538), (25.92, -6.618), (28.184, -5.469),
            (30.29, -3.975), (32.246, -2.396), (36.113, 0.773), (40.159, 3.972), (42.544, 4.697), (47.599, 5.177),
            (55.327, 5.276), (63.078, 5.346), (70.828, 5.389), (78.328, 5.465), (85.828, 5.429), (93.455, 5.383),
            (101.071, 5.088), (108.609, 3.973), (116.122, 2.08), (123.368, -0.289), (130.463, -3.397),
            (137.001, -7.066),
            (143.344, -11.511), (149.315, -16.449), (154.867, -21.672), (160.303, -27.014), (165.656, -32.44),
            (171.016, -37.862), (176.376, -43.285), (181.642, -48.625), (192.262, -59.397), (198.405, -65.628)
        ]
        raw_traj_ego = [
            (135.144, -14.556, 150.211), (135.144, -14.556, 150.211), (135.144, -14.556, 150.211), (135.144, -14.556, 150.211),
            (135.093, -14.527, 150.211), (134.660, -14.279, 150.211), (134.227, -14.031, 150.211), (133.786, -13.779, 150.211),
            (133.345, -13.526, 150.211), (131.546, -12.496, 150.211), (129.363, -11.280, 151.026), (127.121, -10.084, 152.422),
            (124.886, -8.963, 153.934), (122.616, -7.917, 156.960), (120.252, -6.970, 158.482), 
            (119.676, -6.990, 160.349), (119.676, -6.990, 160.349), (119.676, -6.990, 160.349), (119.510, -6.933, 161.144),
            (119.029, -6.769, 161.144), (118.548, -6.604, 161.144), (117.669, -6.304, 161.144), (115.224, -5.472, 161.571),
            (112.804, -4.698, 163.185), (110.353, -4.029, 166.584), (107.905, -3.521, 170.069), (105.390, -3.154, 172.301),
            (102.913, -2.819, 172.301), (100.424, -2.502, 174.191), (97.849, -2.297, 176.288), (95.313, -2.132, 176.288),
            (92.816, -2.025, 178.374), (90.275, -1.975, 178.910), (87.775, -1.942, 179.673), (85.233, -1.929, 179.891),
            (82.691, -1.932, -179.721), (80.149, -1.945, -179.721), (77.608, -1.957, -179.721), (75.108, -1.939, 179.473),
            (72.566, -1.916, 179.473), (70.025, -1.916, -179.689), (67.484, -1.930, -179.689), (64.192, -1.948, -179.689),
            (60.442, -1.968, -179.689), (56.692, -1.997, -179.362), (52.880, -2.063, -178.488), (49.132, -2.144, -179.235),
            (45.320, -2.182, -179.703), (41.570, -2.172, 179.751), (37.820, -2.131, 179.095), (34.009, -2.052, 178.768),
            (30.200, -1.925, 177.980), (26.390, -1.791, 177.980), (22.643, -1.654, 177.652), (18.897, -1.490, 177.324),
            (15.154, -1.315, 177.543), (11.281, -1.180, 178.759), (7.532, -1.175, -179.690), (3.721, -1.268, -177.486),
            (-0.026, -1.432, -177.486), (-3.897, -1.603, -177.156), (-7.641, -1.815, -177.137), (-11.451, -1.954, -178.292),
            (-15.203, -2.013, -179.517), (-18.955, -2.038, -179.847), (-22.767, -2.057, -179.328), (-26.579, -2.102, -179.328),
            (-30.328, -2.193, -178.075), (-34.135, -2.387, -176.544), (-37.878, -2.606, -176.875), (-41.685, -2.788, -178.603),
            (-45.434, -2.792, 179.126), (-49.182, -2.685, 178.112), (-52.930, -2.562, 178.112), (-56.741, -2.463, 178.730),
            (-60.614, -2.343, 177.217), (-61.238, -2.312, 177.152), (-61.238, -2.312, 177.152), (-61.238, -2.312, 177.152)
        ]

        # 去重清洗，防止密集点原地抽搐
        traj_v1 = rtb.clean_trajectory(raw_traj_v1, min_dist=0.5)
        traj_v2 = rtb.clean_trajectory(raw_traj_v2, min_dist=0.5)
        traj_v3 = rtb.clean_trajectory(raw_traj_v3, min_dist=0.5)
        traj_ego = rtb.clean_trajectory(raw_traj_ego, min_dist=0.5)

        # ==========================================
        # 三、车辆生成 (安全生成)
        # ==========================================
        v1 = rtb.spawn_vehicle(world, 'vehicle.audi.tt', traj_v1[0][0], traj_v1[0][1], yaw=-111.457)
        v2 = rtb.spawn_vehicle(world, 'vehicle.chevrolet.impala', traj_v2[0][0], traj_v2[0][1], yaw=155.871)
        v3 = rtb.spawn_vehicle(world, 'vehicle.yamaha.yzf', traj_v3[0][0], traj_v3[0][1], yaw=-4.794)
        ego = rtb.spawn_vehicle(world, 'vehicle.lincoln.mkz_2020', x=traj_ego[0][0], y=traj_ego[0][1],
                                yaw=traj_ego[0][2], color='192,192,192', role_name="ego")

        for v in [v1, v2, v3, ego]:
            if v: actor_list.append(v)

        # ==========================================
        # 四、控制器与状态机实例化
        # ==========================================
        pid_lon1 = rtb.PIDLongitudinalController(preset='default_car')
        pid_lat1 = rtb.PIDLateralController(preset='default_car')
        pid_lon2 = rtb.PIDLongitudinalController(preset='default_car')
        pid_lat2 = rtb.PIDLateralController(preset='default_car')
        pid_lon3 = rtb.PIDLongitudinalController(preset='motorcycle')
        pid_lat3 = rtb.PIDLateralController(preset='motorcycle')
        pid_lon_ego = rtb.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = rtb.PIDLateralController(preset='default_car')

        # V1: 多阶段状态机 (到达 Y < 27 减速到20，2秒后加速到60)
        v1_sm = rtb.MultiStageBehaviorMachine(initial_speed=60.0)
        v1_sm.add_stage(trigger_type='y_less', trigger_val=27.0, target_speed=20.0, accel=25.0)
        v1_sm.add_stage(trigger_type='time', trigger_val=1.0, target_speed=60.0, accel=20.0)

        # V3: 摩托车大油门瞬间爆发
        v3_sm = rtb.MultiStageBehaviorMachine(initial_speed=20.0)
        v3_sm.add_stage(trigger_type='immediate', target_speed=80.0, accel=40.0)

        ego_sm = rtb.MultiStageBehaviorMachine(initial_speed=65.0)
        ego_sm.add_stage(trigger_type='x_less', trigger_val=47.0, target_speed=30.0, accel=25.0)
        ego_sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=65.0, accel=20.0)

        current_idx_v1, current_idx_v2, current_idx_v3, current_idx_ego = 0, 0, 0, 0
        sim_time = 0.0

        # ==========================================
        # 物理预热与稳定
        # ==========================================
        print("等待物理系统预热并稳定车辆底盘...")
        for _ in range(20):
            world.tick()

        # 预热完毕后，赋予绝对物理初速度，防止启动卡顿
        if v1: rtb.set_vehicle_initial_speed(v1, 60.0)
        if v2: rtb.set_vehicle_initial_speed(v2, 60.0)
        if v3: rtb.set_vehicle_initial_speed(v3, 20.0)
        if ego: rtb.set_vehicle_initial_speed(ego, 65.0, yaw_deg=traj_ego[0][2])

        print("\n仿真正式开始！(无尽模式，按 Ctrl+C 退出)")

        # ==========================================
        # 五、无限循环仿真与硬件同步 (Real-Time Factor)
        # ==========================================
        while True:
            # 记录真实时间，用于帧率锁
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- V1 控制 ----------------
            if v1 and v1.is_alive:
                # 检查出界：偏离道路过远则自动销毁
                if not rtb.check_vehicle_out_of_bounds(v1, carla_map, threshold_dist=6.0, auto_destroy=True):
                    # 检查是否到达轨迹终点
                    if current_idx_v1 >= len(traj_v1) - 2:
                        v1.apply_control(carla.VehicleControl(brake=1.0))
                    else:
                        target_wp1, current_idx_v1 = rtb.get_target_waypoint(v1.get_location(), traj_v1, current_idx_v1,
                                                                             speed_kmh=60.0)
                        target_speed_v1 = v1_sm.tick(v1.get_location(), sim_time, dt)
                        rtb.apply_pid_control(v1, pid_lon1, pid_lat1, target_speed_v1, target_wp1)

            # ---------------- V2 控制 ----------------
            if v2 and v2.is_alive:
                if not rtb.check_vehicle_out_of_bounds(v2, carla_map, auto_destroy=True):
                    if current_idx_v2 >= len(traj_v2) - 2:
                        v2.apply_control(carla.VehicleControl(brake=1.0))
                    else:
                        target_wp2, current_idx_v2 = rtb.get_target_waypoint(v2.get_location(), traj_v2, current_idx_v2,
                                                                             speed_kmh=60.0)
                        rtb.apply_pid_control(v2, pid_lon2, pid_lat2, 60.0, target_wp2)

            # ---------------- V3 控制 ----------------
            if v3 and v3.is_alive:
                if not rtb.check_vehicle_out_of_bounds(v3, carla_map, auto_destroy=False):
                    if current_idx_v3 >= len(traj_v3) - 2:
                        v3.apply_control(carla.VehicleControl(brake=1.0))
                    else:
                        target_wp3, current_idx_v3 = rtb.get_target_waypoint(v3.get_location(), traj_v3, current_idx_v3,
                                                                             speed_kmh=80.0)
                        target_speed_v3 = v3_sm.tick(v3.get_location(), sim_time, dt)
                        rtb.apply_pid_control(v3, pid_lon3, pid_lat3, target_speed_v3, target_wp3)

            # ---------------- Ego车 控制 ----------------
            if ego and ego.is_alive:
                if not rtb.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    if current_idx_ego >= len(traj_ego) - 2:
                        print("[RoadTailBench] Ego reached trajectory endpoint; cleaning actors and stopping scenario.")
                        rtb.cleanup_actors(client, actor_list)
                        break
                    else:
                        target_speed_ego = ego_sm.tick(ego.get_location(), sim_time, dt)
                        ego_target_wp, current_idx_ego = rtb.get_target_waypoint(ego.get_location(), traj_ego,
                                                                                 current_idx_ego,
                                                                                 speed_kmh=target_speed_ego)
                        rtb.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_speed_ego, ego_target_wp)

            # ---------------- 硬件时钟补齐 (强制20帧真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体
        rtb.disable_synchronous_mode(world)
        rtb.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()
