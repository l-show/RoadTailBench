import sys
import carla
import time
import random

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        bp_lib = world.get_blueprint_library()
        dt = 0.05

        # ==========================================
        # 引擎配置：开启同步模式，固定时间步长 0.05s (20 FPS)
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # ==========================================
        # 一、天气系统 (完全复刻截图 ClearNight 面板参数)
        # ==========================================
        weather = carla.WeatherParameters(
            cloudiness=5.0,
            precipitation=45.0,
            precipitation_deposits=35.0,
            wind_intensity=40.0,
            sun_azimuth_angle=-1.0,
            sun_altitude_angle=-90.0,
            fog_density=0.0,
            fog_distance=0.0,
            fog_falloff=0.85,
            wetness=40.0,
            scattering_intensity=0.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已按照截图复刻")

        # ==========================================
        # 二、轨迹数据硬编码与清洗
        # ==========================================
        # 1. 警车轨迹
        raw_traj_v1 = [
            (-6.914, -168.832), (-6.914, -168.832), (-6.914, -168.832), (-6.914, -168.832), (-7.043, -165.617),
            (-7.244, -160.562), (-7.294, -155.487), (-7.238, -150.504), (-7.12, -145.688), (-6.954, -140.106),
            (-6.814, -135.387), (-6.668, -129.834), (-6.555, -124.862), (-6.472, -120.058), (-6.42, -114.532),
            (-6.396, -109.267), (-6.378, -104.845), (-6.361, -99.815), (-6.347, -94.837), (-6.326, -89.269),
            (-6.26, -84.528), (-6.133, -79.076), (-6.024, -74.274), (-6.018, -68.805), (-6.045, -64.009),
            (-6.086, -59.366), (-6.162, -53.989), (-6.255, -49.322), (-6.361, -44.074), (-6.344, -38.859),
            (-6.152, -33.79), (-5.969, -28.583), (-5.84, -23.924), (-5.701, -18.881), (-5.565, -13.836),
            (-5.425, -8.167), (-5.305, -3.132), (-5.165, 2.764), (-5.055, 8.954), (-5.024, 15.635),
            (-5.051, 21.753), (-5.08, 28.412), (-5.115, 34.921), (-5.157, 41.148), (-5.2, 47.264),
            (-5.202, 53.265), (-5.158, 59.835), (-5.1, 68.596), (-5.007, 77.91), (-4.857, 86.585),
            (-4.686, 95.763), (-4.56, 104.453), (-4.481, 113.465), (-4.412, 122.463), (-4.336, 131.261),
            (-3.784, 140.084), (-2.87, 148.795), (-1.974, 157.646), (-1.429, 166.377), (-1.182, 175.419),
            (-1.053, 182.271), (-1.053, 182.271), (-1.053, 182.271)
        ]

        # 2. v3货车轨迹
        raw_traj_v3 = [
            (5.697, 85.498, -87.232), (5.697, 85.498, -87.232), (5.697, 85.498, -87.232),
            (5.697, 85.498, -87.232), (5.756, 83.915, -87.932), (6.081, 73.738, -89.74),
            (6.007, 63.248, -90.579), (5.859, 52.796, -91.069), (5.672, 42.317, -90.999),
            (5.527, 33.528, -90.649), (5.431, 22.834, -90.159), (5.436, 12.826, -91.215),
            (4.405, 2.239, -99.856), (2.637, -7.215, -98.802), (1.833, -17.288, -90.942),
            (1.677, -27.687, -90.662), (1.606, -38.862, -90.172), (1.589, -48.595, -90.032),
            (1.583, -58.528, -90.032), (1.578, -68.593, -90.032), (1.572, -78.712, -90.032),
            (1.554, -89.104, -90.242), (1.494, -97.922, -90.802), (1.279, -109.261, -91.292),
            (1.087, -118.686, -90.872), (0.95, -128.251, -90.732), (0.825, -139.571, -90.522),
            (0.746, -148.609, -90.452), (0.673, -160.039, -90.242), (0.626, -171.006, -90.242),
            (0.616, -179.562, -89.822), (0.733, -191.02, -88.352), (1.083, -199.867, -86.742),
            (1.931, -210.799, -84.152), (3.092, -221.371, -82.163), (5.086, -230.935, -72.991),
            (9.548, -240.097, -54.166), (16.234, -248.115, -45.794), (23.971, -254.636, -35.511),
            (24.706, -255.16, -35.511), (24.706, -255.16, -35.511),
        ]

        # 3. ego轨迹
        raw_traj_ego = [
            (12.852, 103.868, -87.625),
            (12.852, 103.868, -88.205),
            (13.060, 97.169, -88.783),
            (13.177, 88.938, -89.263),
            (13.356, 78.070, -89.033),
            (13.499, 68.240, -89.767),
            (13.495, 58.122, -90.117),
            (13.214, 47.688, -95.239),
            (11.867, 38.418, -101.995),
            (9.802, 28.675, -101.645),
            (8.481, 19.314, -91.891),
            (8.429, 7.969, -93.398),
            (4.884, -1.926, -108.811),
            (2.554, -11.250, -94.488),
            (3.464, -20.784, -75.214),
            (6.112, -30.942, -77.878),
            (8.289, -41.914, -81.973),
            (9.219, -51.918, -88.370),
            (9.376, -62.549, -89.769),
            (9.247, -71.665, -92.292),
            (8.854, -81.151, -92.222),
            (8.630, -92.976, -89.657),
            (8.697, -101.100, -89.237),
            (8.836, -111.541, -89.587),
            (8.804, -122.579, -91.056),
            (8.573, -133.438, -91.266),
            (8.392, -141.712, -91.196),
            (8.223, -152.008, -90.287),
            (8.238, -162.311, -89.517),
            (8.337, -174.013, -89.517),
            (8.414, -183.121, -89.517),
            (8.551, -193.366, -88.280),
            (9.132, -202.990, -83.898),
            (10.549, -213.695, -81.342),
            (12.092, -223.798, -80.792),
            (14.812, -234.131, -64.682),
            (20.282, -242.180, -47.984),
            (28.328, -249.279, -36.997),
            (36.446, -255.372, -36.787),
            (44.715, -261.357, -35.276),
            (47.713, -263.474, -35.206),
        ]
        # 4. 行人轨迹
        raw_traj_walker = [
            (33.789, -7.248), (33.789, -7.248), (33.789, -7.248), (33.789, -7.248), (23.892, -6.546),
            (23.892, -6.546), (19.992, -5.893), (16.032, -5.138), (14.84, -4.655), (10.419, -2.325),
            (4.686, 2.146), (0.917, 6.791), (-5.987, 15.737), (-7.38, 17.46), (-11.07, 21.423),
            (-11.07, 21.423), (-13.374, 21.898), (-20.665, 21.973), (-29.829, 21.773), (-40.816, 21.175),
            (-50.116, 20.426), (-53.151, 20.226), (-53.151, 20.226), (-53.151, 20.226)
        ]

        # 【修复】：调用库函数清洗所有冗余噪点，防止卡死
        traj_v1 = RTB.clean_trajectory(raw_traj_v1, min_dist=0.5)
        traj_v3 = RTB.clean_trajectory(raw_traj_v3, min_dist=0.5)
        traj_ego = RTB.clean_trajectory(raw_traj_ego, min_dist=0.5)
        traj_walker = RTB.clean_trajectory(raw_traj_walker, min_dist=0.5)

        # ==========================================
        # 三、实体生成 (车辆与行人)
        # ==========================================
        # 1. 生成警车 (V1)
        v1 = RTB.spawn_vehicle(world, 'vehicle.dodge.charger_police', traj_v1[0][0], traj_v1[0][1], yaw=91.791, z_offset=1)
        if v1: actor_list.append(v1)

        # 3. 生成货车 V3 【修复：改为使用其自己的轨迹点和偏航角生成】
        v3 = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', traj_v3[0][0], traj_v3[0][1], yaw=-87.232, z_offset=1)
        if v3: actor_list.append(v3)

        # 4. 生成ego 【修复：改为使用其自己的轨迹点和偏航角生成】
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', traj_ego[0][0], traj_ego[0][1], yaw=-87.232, z_offset=2)
        if ego: actor_list.append(ego)

        # 5. 原生生成行人
        bp_walker = random.choice(bp_lib.filter('walker.pedestrian.*'))
        walker_loc = carla.Location(x=traj_walker[0][0], y=traj_walker[0][1], z=1.0)
        walker = world.try_spawn_actor(bp_walker, carla.Transform(walker_loc))
        if walker:
            actor_list.append(walker)
            print("[实体生成] 成功生成行人模型。")

        # ==========================================
        # 四、控制器与剧本状态机初始化
        # ==========================================
        # 1. 车辆 PID
        pid_lon1 = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat1 = RTB.PIDLateralController(preset='default_car')
        pid_lon3 = RTB.PIDLongitudinalController(preset='truck') # 货车使用 truck 预设更好
        pid_lat3 = RTB.PIDLateralController(preset='truck')
        pid_lon4 = RTB.PIDLongitudinalController(preset='default_car') # ego 使用 pid_lon4
        pid_lat4 = RTB.PIDLateralController(preset='default_car')


        # 3. 车辆灯光管理器
        v1_lights = RTB.VehicleLightManager(v1)
        v1_lights.turn_on(carla.VehicleLightState.HighBeam)
        v1_lights.start_flashing(mode='police')

        v3_lights = RTB.VehicleLightManager(v3)
        v3_lights.turn_on(carla.VehicleLightState.HighBeam)

        ego_lights = RTB.VehicleLightManager(ego)
        ego_lights.turn_on(carla.VehicleLightState.HighBeam)

        # 4. 速度剧本编排 (MultiStageBehaviorMachine)
        # V1: 初始 40，立刻加速到 80
        v1_sm = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        v1_sm.add_stage(trigger_type='immediate', target_speed=80.0, accel=20.0)

        # 【新增剧本】v3: 初始 30，车辆向南(Y减小)，当 y < 16 时加速到 80
        v3_sm = RTB.MultiStageBehaviorMachine(initial_speed=30.0)
        v3_sm.add_stage(trigger_type='y_less', trigger_val=16.0, target_speed=80.0, accel=20.0)

        # 【新增剧本】ego: 初始 30，车辆向南(Y减小)，当 y < 20 时减速到20，等待2秒恢复60
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=30.0)
        ego_sm.add_stage(trigger_type='y_less', trigger_val=20.0, target_speed=20.0, accel=25.0)
        ego_sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=60.0, accel=15.0)

        # Walker: 初始速度 1.5m/s (走路)，2秒后加速到 3.5m/s (奔跑)
        walker_sm = RTB.MultiStageBehaviorMachine(initial_speed=1.5)
        walker_sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=3.5, accel=100.0)

        # 挂载行人中枢控制器
        ped_ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=traj_walker)

        # ==========================================
        # 物理预热与稳定
        # ==========================================
        print("等待物理系统预热并稳定实体底盘...")
        for _ in range(20):
            world.tick()

        # 预热完毕后注入绝对物理初速度
        if v1: RTB.set_vehicle_initial_speed(v1, 40.0)
        if v3: RTB.set_vehicle_initial_speed(v3, 30.0)
        if ego: RTB.set_vehicle_initial_speed(ego, 30.0)

        current_idx_v1, current_idx_v2, current_idx_v3, current_idx_ego = 0, 0, 0, 0
        sim_time = 0.0

        print("\n仿真正式开始！(无尽模式，按 Ctrl+C 退出)")

        # ==========================================
        # 五、无限循环仿真与硬件同步 (Real-Time Factor)
        # ==========================================
        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- V1 控制 (警车) ----------------
            if v1 and v1.is_alive:
                v1_lights.tick(sim_time)
                if not RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True):
                    if current_idx_v1 >= len(traj_v1) - 2:
                        v1.apply_control(carla.VehicleControl(brake=1.0))
                    else:
                        target_wp1, current_idx_v1 = RTB.get_target_waypoint(v1.get_location(), traj_v1, current_idx_v1, speed_kmh=80.0)
                        target_speed_v1 = v1_sm.tick(v1.get_location(), sim_time, dt)
                        RTB.apply_pid_control(v1, pid_lon1, pid_lat1, target_speed_v1, target_wp1)

            # ---------------- V3 控制 (货车) 【修复大小写BUG，并应用剧本】 ----------------
            if v3 and v3.is_alive:
                if not RTB.check_vehicle_out_of_bounds(v3, carla_map, threshold_dist=6.0, auto_destroy=True):
                    if current_idx_v3 >= len(traj_v3) - 2:
                        v3.apply_control(carla.VehicleControl(brake=1.0))
                    else:
                        target_wp3, current_idx_v3 = RTB.get_target_waypoint(v3.get_location(), traj_v3, current_idx_v3, speed_kmh=80.0)
                        target_speed_v3 = v3_sm.tick(v3.get_location(), sim_time, dt)
                        RTB.apply_pid_control(v3, pid_lon3, pid_lat3, target_speed_v3, target_wp3)

            # ---------------- ego 控制 (主车) 【修复PID下标BUG，并应用剧本】 ----------------
            if ego and ego.is_alive:
                if not RTB.check_vehicle_out_of_bounds(ego, carla_map, threshold_dist=6.0, auto_destroy=True):
                    if current_idx_ego >= len(traj_ego) - 2:
                        ego.apply_control(carla.VehicleControl(brake=1.0))
                    else:
                        target_wp_ego, current_idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, current_idx_ego, speed_kmh=60.0)
                        target_speed_ego = ego_sm.tick(ego.get_location(), sim_time, dt)
                        # 【修复】：使用为 ego 专属初始化的 pid_lon4 和 pid_lat4
                        RTB.apply_pid_control(ego, pid_lon4, pid_lat4, target_speed_ego, target_wp_ego)

            # ---------------- Walker 控制 (行人) ----------------
            if walker and walker.is_alive:
                current_ped_speed = walker_sm.tick(walker.get_location(), sim_time, dt)
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=current_ped_speed)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()