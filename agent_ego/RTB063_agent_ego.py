import sys
import carla
import time
import math

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 原始轨迹数据文本硬编码 (包含 X, Y, Yaw)
# ==========================================
RAW_TRAJ_BIKE = """
-0.299	-29.466	75.851
-0.299	-29.466	75.851
-0.299	-29.466	75.851
-0.27	-29.353	75.64
-0.141	-28.862	73.575
0.016	-28.378	71.141
0.214	-27.799	71.141
0.961	-25.611	71.141
1.718	-23.142	75.768
2.152	-20.682	83.794
2.318	-18.147	87.382
2.351	-15.607	91.345
2.251	-13.068	92.686
2.132	-10.53	92.686
2.016	-8.033	92.616
1.904	-5.495	92.406
1.796	-2.957	92.974
1.599	-0.425	94.779
1.397	2.065	94.359
1.207	4.598	94.289
1.029	7.132	92.932
0.942	9.668	91.097
0.968	12.207	87.946
1.066	14.703	87.526
1.187	17.24	86.896
1.335	19.775	86.546
1.465	22.31	87.893
1.527	24.848	88.75
1.53	27.343	93.928
1.24	29.862	97.181
0.923	32.378	97.111
0.64	34.899	95.984
0.417	37.383	93.479
0.31	39.931	91.278
0.3	42.428	89.498
0.365	44.966	88.279
0.46	48.128	88.279
0.612	53.204	88.279
0.762	58.198	88.279
0.928	63.275	87.929
1.111	68.353	87.929
1.295	73.43	87.929
1.401	78.426	89.618
1.385	83.588	90.328
1.374	85.42	90.328
1.365	86.94	90.328
1.347	90.125	90.328
1.344	90.746	90.328
1.344	90.746	90.328
1.344	90.746	90.328
"""

RAW_TRAJ_EGO = """
-69.674	-49.643	3.699
-69.674	-49.643	3.699
-69.491	-49.63	3.839
-68.984	-49.59	5.109
-68.403	-49.538	5.109
-67.033	-49.416	5.109
-63.09	-49.063	5.109
-58.048	-48.427	9.129
-53.047	-47.52	10.762
-48.066	-46.509	12.187
-43.188	-45.409	12.894
-38.229	-44.3	11.62
-33.241	-43.326	10.913
-29.254	-42.558	10.913
-26.33	-41.994	10.913
-23.877	-41.521	10.913
-21.381	-41.054	10.424
-18.923	-40.611	10.144
-16.421	-40.185	8.807
-13.899	-39.914	3.38
-11.407	-39.765	3.52
-8.883	-39.507	8.94
-6.42	-38.91	18.823
-4.208	-37.699	39.143
-2.251	-36.09	39.713
-0.615	-34.169	54.96
0.709	-32.055	59.362
1.979	-29.86	61.237
2.984	-27.533	71.519
3.646	-25.126	75.611
4.077	-22.631	86.582
4.136	-20.094	90.124
4.13	-17.555	90.124
4.121	-13.267	90.124
3.975	-8.191	92.55
3.741	-3.201	92.9
3.433	1.952	94.829
2.843	6.912	97.795
2.154	11.946	97.795
1.482	16.981	96.509
1.141	21.964	92.622
1.063	27.125	89.474
1.109	32.121	89.474
1.165	37.2	89.264
1.229	42.196	89.264
1.295	47.275	89.264
1.36	52.353	89.264
1.425	57.433	89.264
1.49	62.512	89.264
1.554	67.507	89.264
1.615	72.587	89.404
1.649	77.583	89.824
1.628	82.663	90.594
"""

RAW_TRAJ_C3 = """
5.561	91.881	-93.406
5.561	91.881	-93.406
5.509	91.008	-93.406
5.421	89.531	-93.406
5.223	85.871	-91.899
5.187	81.997	-89.914
5.218	78.254	-88.785
5.332	74.451	-88.287
5.445	70.645	-88.287
5.547	66.837	-88.847
5.579	63.091	-90.911
5.514	59.284	-90.981
5.461	55.474	-90.771
5.41	51.666	-90.771
5.361	47.858	-90.701
5.32	44.549	-90.701
5.32	44.549	-90.701
5.32	44.549	-90.701
5.32	44.549	-90.701
5.32	44.549	-90.701
5.294	42.427	-90.701
5.281	38.618	-89.281
5.345	34.81	-88.861
5.421	31.002	-88.861
5.496	27.192	-88.861
5.566	23.384	-89.001
5.626	19.637	-89.211
5.664	15.828	-89.771
5.661	13.706	-90.19
5.661	13.706	-90.19
5.657	12.644	-90.19
5.644	8.836	-90.19
5.632	5.09	-90.19
5.619	1.281	-90.19
5.586	-2.527	-90.54
5.55	-6.336	-90.54
5.515	-10.083	-90.54
5.479	-13.891	-90.54
5.443	-17.699	-90.54
5.407	-21.508	-90.54
5.26	-25.314	-94.198
4.759	-29.024	-101.872
3.719	-32.686	-110.405
1.987	-36.043	-124.296
-0.554	-38.859	-141.972
-3.857	-40.743	-154.149
-7.323	-42.321	-156.019
-10.936	-43.48	-170.04
-14.643	-44.019	-171.991
-18.409	-44.586	-171.348
-22.172	-45.181	-170.858
-25.934	-45.791	-170.573
-29.751	-46.45	-170.148
-33.506	-47.102	-170.148
-37.238	-47.875	-168.345
-40.913	-48.615	-168.694
-44.713	-49.358	-169.329
-48.457	-50.064	-169.329
-52.201	-50.769	-169.329
-55.944	-51.474	-169.329
-59.649	-52.172	-169.329
-63.392	-52.878	-169.329
-67.136	-53.583	-169.329
-70.879	-54.288	-169.329
-74.623	-54.994	-169.329
-75.482	-55.156	-169.329
-75.482	-55.156	-169.329
-75.482	-55.156	-169.329
"""

RAW_TRAJ_PED = """
0.027	-18.146	85.064
0.269	-15.344	85.064
0.485	-12.812	85.274
0.673	-10.277	85.906
0.841	-7.741	86.536
0.916	-5.243	91.099
0.753	-2.709	95.696
0.381	-0.198	101.085
-0.131	2.29	102.423
-0.685	4.768	102.633
-1.155	7.22	97.982
-1.387	9.746	91.836
-1.245	12.27	75.495
-0.305	14.624	63.795
1.009	16.786	49.375
1.036	16.817	47.707
1.036	16.817	47.707
1.036	16.817	47.707
1.036	16.817	47.707
1.036	16.817	73.184
0.855	18.507	103.854
0.123	20.938	108.276
-0.671	23.351	107.196
-1.214	25.83	99.076
-1.566	28.344	96.211
-1.834	30.912	95.649
-1.987	33.447	92.086
-2.017	35.987	89.816
-2.003	38.528	89.393
-1.987	41.069	90.026
-2.007	43.611	90.59
-2.033	46.11	90.59
-2.055	48.651	90.238
-2.066	51.192	90.238
-2.076	53.733	90.238
-2.092	57.482	90.238
-2.113	62.565	90.238
-2.122	67.647	89.748
-2.119	72.725	90.168
-2.133	77.714	90.168
-2.139	82.79	89.958
-2.135	87.869	89.958
-2.143	92.948	90.168
-2.158	97.882	90.168
-2.16	98.715	90.168
-2.16	98.715	90.168
-2.16	98.715	90.168
-2.16	98.715	90.168
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
        sim_time = 0.0

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按照截图要求映射构建静态天气参数
        RTB.set_static_weather(world,
                               cloudiness=5.0000,
                               precipitation=10.0000,
                               precipitation_deposits=55.0000,
                               wind_intensity=25.0000,
                               sun_azimuth_angle=224.0000,
                               sun_altitude_angle=13.0000,
                               fog_density=2.0000,
                               fog_distance=0.7500,
                               fog_falloff=0.1000,
                               wetness=70.0000,
                               scattering_intensity=1.0000,
                               mie_scattering_scale=0.0300,
                               rayleigh_scattering_scale=0.0331,
                               dust_storm=0.0000)
        print("[场景配置] 天气系统已按照截图要求设置完毕。")

        # ==========================================
        # 2. 轨迹数据解析、清洗与稠密化插值 (到0.5m)
        # ==========================================
        # 提取数据时保留了原始元组结构(X, Y, Yaw)，交给稠密化工具补充细节
        traj_bike = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_BIKE), interval=0.5)
        traj_ego = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_EGO), interval=0.5)
        traj_c3 = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_C3), interval=0.5)
        traj_ped = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_PED), interval=0.5)

        # 轨迹寻点索引初始化
        bike_idx, ego_idx, c3_idx = 0, 0, 0

        # ==========================================
        # 3. 车辆、行人模型实体安全生成
        # ==========================================
        # 获取第一组轨迹的原始起点偏航角(Yaw)，防画龙
        yaw_bike = float(RAW_TRAJ_BIKE.strip().split('\n')[0].split()[2])
        yaw_ego = float(RAW_TRAJ_EGO.strip().split('\n')[0].split()[2])
        yaw_c3 = float(RAW_TRAJ_C3.strip().split('\n')[0].split()[2])
        yaw_ped = float(RAW_TRAJ_PED.strip().split('\n')[0].split()[2])

        # 生成车辆
        vehicle_bike = RTB.spawn_vehicle(world, 'vehicle.diamondback.century', x=traj_bike[0][0], y=traj_bike[0][1],
                                         yaw=yaw_bike)
        vehicle_ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        vehicle_c3 = RTB.spawn_vehicle(world, 'vehicle.citroen.c3', x=traj_c3[0][0], y=traj_c3[0][1], yaw=yaw_c3)

        # 生成行人 (行人稍微抬高 1.0m 抛下防穿模)
        ped_bp = bp_lib.find('walker.pedestrian.0001')
        ped_tf = RTB.get_transform(x=traj_ped[0][0], y=traj_ped[0][1], z=world.get_map().get_waypoint(
            carla.Location(traj_ped[0][0], traj_ped[0][1], 0)).transform.location.z + 1.0, yaw=yaw_ped)
        walker = world.try_spawn_actor(ped_bp, ped_tf)

        # 将生成的实体统一加入资源回收名单
        for a in [vehicle_bike, vehicle_ego, vehicle_c3, walker]:
            if a: actor_list.append(a)

        # ==========================================
        # 4. 车辆灯光管理器
        # ==========================================

        # ==========================================
        # 5. 车辆PID与行人控制器挂载 (每个实体需独立PID)
        # ==========================================
        # 摩托车/自行车 (使用 'motorcycle' 预设)
        pid_lon_bike = RTB.PIDLongitudinalController(preset='motorcycle', dt=dt)
        pid_lat_bike = RTB.PIDLateralController(preset='motorcycle', dt=dt)

        # Ego / C3 (使用 'default_car' 预设)
        pid_lon_c3 = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_c3 = RTB.PIDLateralController(preset='default_car', dt=dt)

        # ==========================================
        # 6. 剧本状态机编排
        # ==========================================
        # (1) C3 轿车: y=30减速到20km/h，过10s恢复50km/h
        sm_c3 = RTB.MultiStageBehaviorMachine(initial_speed=45.0)
        # 阶段1: C3 从Y轴91往下开，当 Y < 30 时触发，平缓制动到 25 km/h
        sm_c3.add_stage(trigger_type='y_less', trigger_val=30.0, target_speed=10.0, accel=35.0)
        # 阶段2: 进入阶段1后等待 5秒 触发，加速回 50 km/h
        sm_c3.add_stage(trigger_type='time', trigger_val=5.0, target_speed=50.0, accel=8.0)

        # (2) 行人: 初始4.5m/s，在y=16.817静止2s不动，恢复3.5m/s (注意：行人状态机输入输出单位为 m/s)
        ped_ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=traj_ped)
        sm_ped = RTB.MultiStageBehaviorMachine(initial_speed=4.5)
        # 阶段1: 行人从Y轴-22往上走，当 Y > 16.817 时触发静止 (加速度极大模拟瞬间立正)
        sm_ped.add_stage(trigger_type='y_greater', trigger_val=16.817, target_speed=0.0, accel=100.0)
        # 阶段2: 等待 2秒 触发，以 3.5m/s 继续行走
        sm_ped.add_stage(trigger_type='time', trigger_val=2.0, target_speed=3.5, accel=100.0)

        # ==========================================
        # 7. 预热与初始状态瞬间注入 (零阻塞)
        # ==========================================
        RTB.set_vehicle_initial_speed(vehicle_bike, target_speed_kmh=15.0, yaw_deg=yaw_bike)
        RTB.set_vehicle_initial_speed(vehicle_c3, target_speed_kmh=50.0, yaw_deg=yaw_c3)

        # 给物理引擎10帧落地预热时间
        for _ in range(10): world.tick()

        # ==========================================
        # 8. 仿真主循环
        # ==========================================
        print("[主循环] 仿真正式开始...")
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 状态机刷新 ----------------
            if vehicle_c3 and vehicle_c3.is_alive:
                current_c3_speed = sm_c3.tick(vehicle_c3.get_location(), sim_time, dt)

            if walker and walker.is_alive:
                current_ped_speed = sm_ped.tick(walker.get_location(), sim_time, dt)
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=current_ped_speed)

            # ---------------- 车辆 PID 控制 ----------------
            # 自行车 (恒定 15km/h)
            if vehicle_bike and vehicle_bike.is_alive:
                target_wp, bike_idx = RTB.get_target_waypoint(vehicle_bike.get_location(), traj_bike, bike_idx, 15.0)
                if target_wp:
                    RTB.apply_pid_control(vehicle_bike, pid_lon_bike, pid_lat_bike, 15.0, target_wp)

            # Ego 轿车 (恒定 50km/h)

            # Citroen C3 (状态机控制车速)
            if vehicle_c3 and vehicle_c3.is_alive:
                target_wp, c3_idx = RTB.get_target_waypoint(vehicle_c3.get_location(), traj_c3, c3_idx,
                                                            current_c3_speed)
                if target_wp:
                    RTB.apply_pid_control(vehicle_c3, pid_lon_c3, pid_lat_c3, current_c3_speed, target_wp)

            # ---------------- 安全守护：出界检测销毁 ----------------
            vehicles = [vehicle_bike, vehicle_ego, vehicle_c3]
            for v in vehicles:
                # 自动判别是否偏离路网，若是则利用内置 auto_destroy 直接销毁
                RTB.check_vehicle_out_of_bounds(v, carla_map, threshold_dist=6.0, auto_destroy=True)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体
        try:
            RTB.disable_synchronous_mode(world)
            RTB.cleanup_actors(client, actor_list)
        except Exception as e:
            print(f"[清理异常] 忽略: {e}")
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()