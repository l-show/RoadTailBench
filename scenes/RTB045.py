import sys
import carla
import time
import random
import math

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 原始长尾场景轨迹数据 (直接提取自题目)
# ==========================================
RAW_TRAJ_V1 = """
-0.605	136.193	-73.173
-0.605	136.193	-73.173
-0.605	136.193	-73.173
-0.605	136.193	-73.902
1.297	128.929	-78.726
2.043	118.863	-90.365
1.971	108.512	-89.366
2.124	98.418	-88.855
2.229	88.126	-90.267
2.181	77.946	-90.267
2.142	69.469	-90.267
2.118	63.037	-89.663
2.154	56.762	-89.663
2.174	50.499	-89.872
2.194	44.246	-89.802
2.216	37.853	-89.802
2.242	31.563	-89.732
2.272	25.177	-89.732
2.26	18.823	-91.399
1.838	12.509	-99.959
-0.174	6.634	-116.299
-3.447	1.14	-129.556
-8.154	-2.847	-151.84
-14.167	-4.602	-172.532
-20.471	-5.215	-176.158
-26.812	-5.404	-179.532
-33.208	-5.42	179.717
-39.391	-5.382	179.647
-45.794	-5.337	179.577
-52.072	-5.291	179.577
-58.243	-5.245	179.577
-64.701	-5.198	179.787
-70.965	-5.229	-179.63
-77.202	-5.269	-179.63
-83.577	-5.306	-179.84
-89.893	-5.319	179.95
-96.222	-5.314	179.95
-102.537	-5.308	179.95
-108.825	-5.303	179.95
-116.666	-5.296	179.95
-125.568	-5.288	179.95
-134.451	-5.265	178.552
-143.233	-4.435	172.43
-151.982	-2.613	165.121
-160.154	0.258	156.534
-167.918	4.573	145.244
-174.844	10.145	137.833
-181.436	16.115	137.833
-187.292	22.761	123.475
-191.658	30.667	115.899
-195.298	38.781	111.317
-197.806	47.299	101.2
-198.771	55.979	92.372
-198.965	64.872	90.164
-199.008	73.768	91.146
-199.154	81.059	91.146
"""

RAW_TRAJ_EGO = """
-156.617	10.048	-24.111
-156.328	9.919	-24.111
-155.201	9.414	-24.111
-154.018	8.885	-24.111
-152.877	8.377	-22.825
-151.688	7.904	-19.652
-150.485	7.524	-15.645
-149.26	7.22	-13.543
-146.928	6.658	-13.543
-144.46	6.178	-8.124
-141.967	5.822	-8.124
-139.455	5.523	-3.726
-136.889	5.405	-1.691
-134.35	5.355	-1.051
-131.799	5.308	-1.051
-129.31	5.264	-0.533
-126.732	5.295	0.948
-122.711	5.362	0.948
-117.623	5.414	-0.008
-112.593	5.413	-0.008
-107.563	5.413	-0.008
-102.467	5.411	-0.078
-97.408	5.393	-0.1
-92.398	5.404	0.35
-87.254	5.426	0.21
-82.237	5.44	0.14
-77.172	5.446	-0.14
-72.073	5.416	-0.304
-66.985	5.404	0.074
-61.831	5.411	0.074
-56.838	5.418	0.144
-51.836	5.441	0.284
-46.696	5.467	0.284
-41.592	5.492	0.284
-36.548	5.517	0.284
-31.538	5.542	0.284
-26.42	5.568	0.284
-21.351	5.593	0.284
-16.33	5.618	0.284
-11.22	5.643	0.284
-6.144	5.668	0.284
-1.106	5.693	0.284
4.041	5.718	0.144
8.954	5.73	0.144
14.214	5.744	0.144
19.19	5.756	0.144
24.309	5.769	0.144
29.39	5.81	0.583
34.415	5.856	0.396
39.556	5.891	0.396
44.555	5.92	0.326
50.476	5.954	0.326
58.246	5.974	-1.815
65.667	5.296	-6.637
73.41	4.395	-6.637
80.919	3.522	-6.637
88.483	2.706	-3.914
95.963	2.485	-0.448
103.581	2.575	1.234
111.099	2.737	1.234
118.622	2.899	1.234
126.257	3.013	0.282
133.893	3.012	-0.049
141.414	2.815	-3.292
148.988	2.081	-9.212
156.328	0.456	-16.079
163.561	-2.11	-24.76
170.189	-5.937	-35.603
176.021	-10.691	-40.106
181.467	-16.018	-48.737
186.149	-22.023	-56.135
189.871	-28.661	-65.506
191.393	-32.224	-67.16
"""

RAW_TRAJ_BUS = """
-81.586	8.991	-0.605
-71.381	9.014	0.81
-61.116	9.159	0.81
-52.222	9.285	0.81
-46.231	9.37	0.81
-41.23	9.424	-0.575
-36.129	9.353	-1.065
-31.022	9.226	-1.259
-26.052	9.265	0.761
-23.233	9.302	0.761
-19.363	9.243	-1.09
-14.235	9.136	-1.682
-9.092	9.007	-1.192
-3.91	8.899	-1.192
3.409	8.746	-1.192
10.891	8.705	0.276
18.677	8.826	1.992
26.272	9.2	3.287
33.82	9.573	1.483
41.268	9.589	-1.032
48.889	9.429	-1.063
56.387	9.366	-0.333
64.191	9.368	0.764
71.732	9.466	0.484
79.029	9.463	0.173
86.79	9.533	1.011
94.429	9.724	1.182
101.902	9.764	0.176
109.547	9.788	0.176
117.128	9.811	0.176
124.81	9.835	0.176
132.315	9.858	0.176
139.836	9.875	-0.854
147.45	9.494	-5.448
154.985	8.254	-13.766
159.003	7.127	-16.617
"""

RAW_TRAJ_PED = """
-33.834	11.1	179.598
-34.397	11.096	-179.074
-34.893	11.086	-178.156
-35.419	11.066	-177.736
-35.919	11.047	-177.876
-36.437	11.027	-177.876
-36.945	11.009	-177.876
-39.037	10.931	-177.876
-41.642	10.941	177.053
-43.48	11.036	177.053
-43.997	11.062	177.053
-44.503	11.089	176.983
-45.005	11.121	174.991
-45.503	11.186	169.25
-45.982	11.323	158.397
-46.431	11.565	142.985
-46.822	11.863	141.597
-47.213	12.181	140.068
-47.595	12.523	135.298
-47.952	12.886	134.502
-48.309	13.249	134.502
-49.039	13.992	134.502
-49.945	14.831	139.918
-50.981	15.611	145.442
-52.01	16.296	147.579
-53.108	16.919	153.816
-54.317	17.359	167.182
-54.944	17.48	169.523
"""


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
        # 1. 环境初始化：帧率同步与高阶天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 根据截图参数完美复现天气
        weather = carla.WeatherParameters(
            cloudiness=15.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_azimuth_angle=210.0, sun_altitude_angle=15.0,
            fog_density=2.0, fog_distance=0.0, fog_falloff=0.0, wetness=0.0,
            scattering_intensity=0.5, mie_scattering_scale=0.22, rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 长尾截屏天气系统已装载。")

        # ==========================================
        # 2. 轨迹数据解析清洗与全景可视化
        # ==========================================
        traj_v1 = RTB.parse_string_trajectory(RAW_TRAJ_V1)
        traj_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO)
        traj_bus = RTB.parse_string_trajectory(RAW_TRAJ_BUS)
        traj_ped = RTB.parse_string_trajectory(RAW_TRAJ_PED)

        RTB.draw_preset_trajectory(world, traj_v1, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_bus, color=carla.Color(150, 150, 150))

        # ==========================================
        # 3. 实体生成与控制器绑定
        # ==========================================

        # --- 车辆 1：对向轿车 (Audi TT) ---
        v1 = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=traj_v1[0][0], y=traj_v1[0][1], yaw=traj_v1[0][2],
                               role_name="npc")
        actor_list.append(v1)
        RTB.set_vehicle_initial_speed(v1, 30.0)

        v1_lights = RTB.VehicleLightManager(v1)
        v1_lights.set_static_lights(low_beam=True)

        pid_lon_v1 = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_v1 = RTB.PIDLateralController(preset='default_car')
        idx_v1 = 0

        sm_v1 = RTB.MultiStageBehaviorMachine(initial_speed=30.0)
        sm_v1.add_stage('y_less', trigger_val=30.0, target_speed=65.0, accel=25.0)
        sm_v1.add_stage('y_less', trigger_val=5.0, target_speed=40.0, accel=15.0)

        # --- 车辆 2：主车 Ego (Chevrolet Impala) ---
        ego = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=traj_ego[0][0], y=traj_ego[0][1],
                                yaw=traj_ego[0][2], role_name="ego")
        actor_list.append(ego)
        RTB.set_vehicle_initial_speed(ego, 60.0)

        ego_lights = RTB.VehicleLightManager(ego)
        ego_lights.set_static_lights(low_beam=True)

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')
        idx_ego = 0

        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        sm_ego.add_stage('x_greater', trigger_val=-30.0, target_speed=20.0, accel=25.0)
        sm_ego.add_stage('x_greater', trigger_val=-2.0, target_speed=60.0, accel=20.0)

        # --- 车辆 3：公交车 (Mercedes Sprinter / 任意大巴) ---
        bus = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck', x=traj_bus[0][0], y=traj_bus[0][1],
                                yaw=traj_bus[0][2], role_name="bus")
        actor_list.append(bus)
        RTB.set_vehicle_initial_speed(bus, 60.0)

        bus_lights = RTB.VehicleLightManager(bus)
        bus_lights.set_static_lights(low_beam=True)

        pid_lon_bus = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_bus = RTB.PIDLateralController(preset='truck')
        idx_bus = 0

        # 【物理与逻辑重构】公交车精准刹车计算：
        # 目标：平稳停在 x = -31.212
        # 计算：60km/h (16.67m/s)，若加速度设为 20(约5.55m/s2)，所需刹车距离约为 25米。
        # 结论：在 X 轴行驶中，必须在 -31.212 减去 25米 也就是 -56.2 时提前踩下刹车！
        sm_bus = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        # 1. 触发刹车点（提前 25 米）
        sm_bus.add_stage('x_greater', trigger_val=-60, target_speed=0.0, accel=20.0)
        # 2. 等待阶段（刹车过程约需 3 秒，总共需要停留 10 秒，故填入 13.0 秒后恢复 30km/h）
        sm_bus.add_stage('time', trigger_val=13.0, target_speed=30.0, accel=10.0)

        # --- 行人系统初始化 ---
        ped_bp = bp_lib.filter('walker.pedestrian.*')[0]

        # 1. 上车队伍
        boarding_peds = []
        boarding_count = 0
        last_board_spawn_time = 0.0

        # 2. 下车队伍 (流式排队生成控制)
        alighting_peds = []
        alighting_triggered = False
        alighting_spawning = False
        alighting_spawn_count = 0
        last_alight_spawn_time = 0.0

        # 将视角绑定到 Ego 车后方以便观察
        spectator = world.get_spectator()
        # ==========================================
        # 4. 仿真主循环 (时钟同步与环境守护)
        # ==========================================
        sim_time = 0.0
        print("[主循环] 仿真开始...")

        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 视角跟随 ----------------
            if ego and ego.is_alive:
                tf = ego.get_transform()
                spectator.set_transform(carla.Transform(
                    tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
                    carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
                ))

            # --- 出界安全守护器 ---
            if RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True): actor_list.remove(
                v1) if v1 in actor_list else None
            if RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True): actor_list.remove(
                ego) if ego in actor_list else None
            if RTB.check_vehicle_out_of_bounds(bus, carla_map, auto_destroy=True): actor_list.remove(
                bus) if bus in actor_list else None

            # --- 车辆 1 (对向车) 控制 ---
            if v1 and v1.is_alive:
                loc = v1.get_location()
                speed = sm_v1.tick(loc, sim_time, dt)
                wp, idx_v1 = RTB.get_target_waypoint(loc, traj_v1, idx_v1, speed)
                if wp:
                    RTB.apply_pid_control(v1, pid_lon_v1, pid_lat_v1, speed, wp)
                    RTB.draw_lookahead_point(world, loc, wp, color=carla.Color(255, 0, 0))  # 红色线段
                v1_lights.auto_update_from_control()

            # --- 车辆 2 (主车 Ego) 控制 ---
            if ego and ego.is_alive:
                loc = ego.get_location()
                speed = sm_ego.tick(loc, sim_time, dt)
                wp, idx_ego = RTB.get_target_waypoint(loc, traj_ego, idx_ego, speed)
                if wp:
                    RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, speed, wp)
                    RTB.draw_lookahead_point(world, loc, wp, color=carla.Color(0, 255, 0))  # 绿色线段
                ego_lights.auto_update_from_control()

            # --- 车辆 3 (公交车) 控制 ---
            if bus and bus.is_alive:
                loc = bus.get_location()
                speed = sm_bus.tick(loc, sim_time, dt)
                wp, idx_bus = RTB.get_target_waypoint(loc, traj_bus, idx_bus, speed)
                if wp:
                    RTB.apply_pid_control(bus, pid_lon_bus, pid_lat_bus, speed, wp)
                    RTB.draw_lookahead_point(world, loc, wp, color=carla.Color(0, 0, 255))  # 蓝色线段
                bus_lights.auto_update_from_control()

            # ==========================================
            # 行人剧本: 上下车逻辑处理
            # ==========================================

            # 【剧本 1: 模拟排队上车 (10人)】
            # 修复：计算生成瞬间的偏航角(Yaw)，强迫行人一出生就面朝目标方向，抬高Z轴防卡地
            if boarding_count < 10 and (sim_time - last_board_spawn_time > 1.5):
                # 几何计算朝向
                dx = -23.818 - (-22.015)
                dy = 10.420 - 16.911
                spawn_yaw = math.degrees(math.atan2(dy, dx))

                spawn_loc = carla.Location(x=-22.015, y=16.911, z=1.0)
                spawn_rot = carla.Rotation(yaw=spawn_yaw)

                walker = world.try_spawn_actor(ped_bp, carla.Transform(spawn_loc, spawn_rot))
                if walker:
                    actor_list.append(walker)
                    ctrl = RTB.PedestrianController(walker, mode='roam', target_list=[(-23.818, 10.420)],
                                                    default_speed=2.0)
                    boarding_peds.append({'actor': walker, 'ctrl': ctrl})
                    boarding_count += 1
                    last_board_spawn_time = sim_time

            # 执行上车行走，强制传入 dynamic_speed=2.0 激活脚步
            for p_dict in boarding_peds[:]:
                w = p_dict['actor']
                if not w.is_alive:
                    boarding_peds.remove(p_dict)
                    continue
                p_dict['ctrl'].run_step(dt, sim_time, dynamic_speed=2.0)

                # 检测碰撞/抵达终点 -> 上车消失
                if w.get_location().distance(carla.Location(-23.818, 10.420, 0.5)) < 0.6:
                    w.destroy()
                    boarding_peds.remove(p_dict)

            # 【剧本 2: 模拟到站列队下车 (5人)】
            # 修复：不再一瞬间生成导致互挤卡死，而是每 0.4 秒流式“吐出”一个下车乘客，营造列队效果
            if not alighting_triggered and sm_bus.current_idx == 1 and bus.get_velocity().length() < 0.1:
                alighting_triggered = True
                alighting_spawning = True
                print("[长尾事件] 公交车已停稳靠站，开始列队生成下车人群。")

            # 流式生成队列
            if alighting_spawning and alighting_spawn_count < 5 and (sim_time - last_alight_spawn_time > 0.4):
                # 几何计算下车朝向(从轨迹点0看向轨迹点1)
                dx = traj_ped[1][0] - traj_ped[0][0]
                dy = traj_ped[1][1] - traj_ped[0][1]
                spawn_yaw = math.degrees(math.atan2(dy, dx))

                spawn_loc = carla.Location(x=traj_ped[0][0], y=traj_ped[0][1], z=1.0)
                spawn_rot = carla.Rotation(yaw=spawn_yaw)

                walker = world.try_spawn_actor(ped_bp, carla.Transform(spawn_loc, spawn_rot))
                if walker:
                    actor_list.append(walker)
                    # 调快下车人员步行速度 (3.5m/s 小跑)
                    ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=traj_ped, default_speed=3.5)
                    alighting_peds.append({'actor': walker, 'ctrl': ctrl})

                    alighting_spawn_count += 1
                    last_alight_spawn_time = sim_time

                if alighting_spawn_count >= 5:
                    alighting_spawning = False

            # 下车队列逻辑控制
            ped_0_finished = False
            if alighting_peds and alighting_peds[0]['ctrl'].traj_index >= len(traj_ped) - 1:
                ped_0_finished = True  # 头车(领头人)已抵达终点

            for i, p_dict in enumerate(alighting_peds):
                w = p_dict['actor']
                if not w.is_alive: continue

                # 头车抵达终点后，后面所有人急停在原地 (模拟驻足等候)
                if ped_0_finished and i > 0:
                    w.apply_control(carla.WalkerControl(speed=0.0))
                else:
                    # 强制下发 3.5m/s 跑步指令
                    p_dict['ctrl'].run_step(dt, sim_time, dynamic_speed=3.5)

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