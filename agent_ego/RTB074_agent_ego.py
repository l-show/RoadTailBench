import sys
import carla
import time

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 长尾场景：轨迹硬编码区 (去除表头，纯数值)
# ==========================================
VAN_TRAJ_STR = """
19.214	50.124	-98.398
19.214	50.124	-98.398
19.214	50.124	-98.398
19.214	50.124	-98.823
18.491	46.04	-101.216
16.991	39.889	-105.956
15.096	33.735	-109.037
12.895	27.797	-110.968
10.629	21.883	-110.968
8.34	15.979	-112.029
7.097	13.124	-118.651
7.097	13.124	-118.651
7.097	13.124	-118.651
7.097	13.124	-118.651
7.097	13.124	-118.651
7.097	13.124	-118.651
7.097	13.124	-118.651
7.097	13.124	-118.651
7.097	13.124	-118.651
7.097	13.124	-118.72
4.109	8.386	-124.434
0.361	3.153	-127.425
-3.749	-1.661	-133.728
-8.343	-5.858	-142.098
-13.685	-9.243	-152.533
-19.322	-12.138	-152.821
-24.981	-14.988	-154.696
-30.78	-17.532	-158.292
-36.75	-19.62	-163.919
-42.859	-20.8	-172.32
-49.144	-21.639	-173.107
-55.464	-22.057	-177.455
-61.797	-22.202	178.223
-68.006	-21.581	171.788
-74.233	-20.382	162.452
-80.174	-18.131	157.243
-85.754	-15.099	149.214
-91.065	-11.632	139.808
-95.743	-7.336	135.45
-100.191	-2.798	134.587
-104.67	1.709	134.867
-109.153	6.211	134.797
-113.631	10.722	134.797
-118.107	15.23	134.797
-122.593	19.73	135.007
-127.075	24.235	134.797
-131.538	28.755	134.446
-135.986	33.291	134.446
-140.449	37.842	134.375
-144.96	42.463	134.444
-149.452	46.952	135.366
-150.711	48.195	135.366
-150.711	48.195	135.366
-150.711	48.195	135.366
"""

EGO_TRAJ_STR = """
8.718	103.576	-70.467
8.718	103.576	-70.467
8.718	103.576	-70.467
11.655	94.727	-73.608
14.417	84.816	-76.076
16.646	74.972	-77.827
18.424	65.036	-84.645
18.706	55.122	-89.846
18.413	46.276	-97.922
16.268	37.69	-106.673
13.578	29.248	-111.359
10.241	21.038	-112.65
8.94	17.961	-112.937
8.94	17.961	-112.937
8.94	17.961	-112.937
8.94	17.961	-112.937
8.94	17.961	-112.937
8.94	17.961	-112.937
7.237	13.95	-114.085
2.88	6.248	-124.643
-2.771	-0.572	-137.034
-9.824	-5.946	-145.607
-17.226	-10.843	-148.959
-25.047	-15.013	-154.998
-33.177	-18.522	-160.205
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-36.632	-19.631	-164.192
-37.59	-19.902	-164.192
-42.501	-21.099	-168.516
-47.489	-21.922	-173.184
-52.524	-22.431	-175.442
-57.668	-22.644	-178.815
-62.652	-22.747	-178.815
-67.802	-22.864	179.424
-72.809	-22.061	164.755
-77.473	-20.279	156.618
-82.071	-18.122	150.28
-86.442	-15.532	146.237
-90.482	-12.45	140.047
-94.287	-9.08	136.679
-97.91	-5.513	134.967
-101.502	-1.917	134.967
-105.094	1.68	134.827
-108.673	5.29	134.757
"""

TT_TRAJ_STR = """
-82.2	-60.915	88.898
-82.2	-60.915	88.898
-82.112	-56.083	89.324
-82.03	-45.916	89.604
-80.375	-35.954	69.71
-74.864	-27.516	41.054
-66.458	-21.861	25.377
-56.69	-19.523	5.634
-46.679	-18.041	12.433
-38.445	-16.225	12.433
-32.333	-14.572	19.332
-26.519	-12.062	26.788
-20.971	-9.236	27.074
-17.865	-7.648	27.36
-15.684	-6.361	31.977
-13.603	-4.997	34.469
-11.545	-3.529	36.041
-9.54	-1.984	38.37
-7.477	-0.516	30.554
-5.256	0.701	27.671
-3.021	1.895	30.505
-0.93	3.251	37.091
0.948	5.009	48.049
2.461	7.038	58.132
3.664	9.268	64.982
4.565	11.592	72.747
5.17	14.053	78.266
5.687	16.535	77.916
6.355	18.985	72.438
7.117	21.354	72.149
7.97	23.733	68.704
9.226	26.954	68.704
11.296	32.915	75.81
12.792	39.06	76.35
14.023	45.26	81.214
14.791	51.534	87.863
14.866	57.966	91.647
14.609	64.294	94.412
13.882	70.585	98.116
12.81	76.724	102.261
11.389	82.903	103.326
9.728	89.015	106.579
7.758	95.021	110.116
5.366	100.874	114.263
2.718	106.506	116.88
-0.186	112.129	117.376
-3.152	117.722	118.428
-6.238	123.251	120.809
-9.55	128.648	121.664
-12.875	134.038	121.664
-16.199	139.428	121.664
-16.199	139.428	121.664
-16.199	139.428	121.664
-16.199	139.428	121.664
"""

def prepare_trajectory(raw_str):
    """辅助函数：解析文本，防止yaw变为z，返回密集的 Location 列表和初始 Yaw"""
    # 1. 字符串清洗并提取 (x, y, yaw)
    raw_tuples = RTB.parse_string_trajectory(raw_str, min_dist=0.1)
    initial_yaw = raw_tuples[0][2] if len(raw_tuples[0]) > 2 else 0.0

    # 2. 为了防止插值器把 yaw 当成高度 Z 去插值，我们只传递 (x, y, 0.0)
    xy_tuples = [(p[0], p[1], 0.0) for p in raw_tuples]

    # 3. 0.5米稠密化
    dense_tuples = RTB.interpolate_trajectory(xy_tuples, interval=0.5)

    # 4. 转化为符合 RTB 工具链的标准化 carla.Location 对象，预留 0.5米 高度防止埋入地下
    path_locations = [carla.Location(x=p[0], y=p[1], z=0.5) for p in dense_tuples]

    return path_locations, initial_yaw

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 严格按照需求设定的长尾天气参数
        weather = RTB.build_weather(
            cloudiness=45.0,
            precipitation=45.0,
            precipitation_deposits=75.0,
            wind_intensity=35.0,
            sun_azimuth_angle=49.0,
            sun_altitude_angle=15.0,
            fog_density=20.0,
            fog_distance=5.0,
            fog_falloff=0.5,
            wetness=60.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.3300,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0000
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已设置完毕。")

        # ==========================================
        # 2. 轨迹数据解析与绘制
        # ==========================================
        path_van, yaw_van = prepare_trajectory(VAN_TRAJ_STR)
        path_ego, yaw_ego = prepare_trajectory(EGO_TRAJ_STR)
        path_tt, yaw_tt = prepare_trajectory(TT_TRAJ_STR)

        # ==========================================
        # 3. 车辆实体安全生成
        # ==========================================
        # 第一辆：小货车 (van)
        van = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', path_van[0].x, path_van[0].y, yaw=yaw_van,
                                role_name="van")
        actor_list.append(van)

        # 第二辆：Ego 小轿车
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        # 第三辆：Audi TT
        tt = RTB.spawn_vehicle(world, 'vehicle.audi.tt', path_tt[0].x, path_tt[0].y, yaw=yaw_tt, role_name="audi_tt")
        actor_list.append(tt)

        # 赋予无视物理阻塞的瞬间初速度
        RTB.set_vehicle_initial_speed(van, 60.0)
        RTB.set_vehicle_initial_speed(tt, 70.0)

        # ==========================================
        # 4. 车辆PID、索引与灯光系统挂载
        # ==========================================
        pid_lon_van = RTB.PIDLongitudinalController(preset='truck')  # 货车使用卡车预设防止刹不住
        pid_lat_van = RTB.PIDLateralController(preset='truck')
        idx_van = 0

        idx_ego = 0

        pid_lon_tt = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_tt = RTB.PIDLateralController(preset='default_car')
        idx_tt = 0

        # 灯光设置：Ego 和 TT 开启行车灯、雾灯、近光灯

        light_tt = RTB.VehicleLightManager(tt)
        light_tt.turn_on(
            carla.VehicleLightState.Position | carla.VehicleLightState.Fog | carla.VehicleLightState.LowBeam)

        # ==========================================
        # 5. 剧本状态机编排
        # ==========================================
        # Van 剧本：初始60。y < 13 时减速到20，等待2s后恢复60。
        sm_van = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_van.add_stage('y_less', trigger_val=13.0, target_speed=20.0, accel=25.0)  # 急刹
        sm_van.add_stage('time', trigger_val=2.0, target_speed=60.0, accel=15.0)  # 恢复

        # Ego 剧本：初始60。y < 17 时减到30；y < -19 时减到10；等3s恢复60。

        # TT 剧本：初始70。第一次 x > -6 的时候减速到30，等5s恢复60。
        # 【逻辑注意】：TT 的 X 坐标是从 -82 逐渐往正数增加的，如果用 x_less 会一出生就触发。因此必须用 x_greater 跨越触发。
        sm_tt = RTB.MultiStageBehaviorMachine(initial_speed=70.0)
        sm_tt.add_stage('x_greater', trigger_val=-6.0, target_speed=30.0, accel=25.0)
        sm_tt.add_stage('time', trigger_val=5.0, target_speed=60.0, accel=15.0)

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        sim_time = 0.0
        print("[RoadTailBench] 🚀 长尾仿真正式开始！")

        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ------------- 货车 Van 控制 -------------
            if van and van.is_alive:
                if RTB.check_vehicle_out_of_bounds(van, carla_map, auto_destroy=True):
                    van = None  # 实体已被销毁，释放指针防止后续报错
                else:
                    target_spd = sm_van.tick(van.get_location(), sim_time, dt)
                    target_wp, idx_van = RTB.get_target_waypoint(van.get_location(), path_van, idx_van,
                                                                 speed_kmh=target_spd)
                    if target_wp:
                        RTB.apply_pid_control(van, pid_lon_van, pid_lat_van, target_spd, target_wp)

            # ------------- 轿车 Ego 控制 -------------

            # ------------- 轿车 TT 控制 -------------
            if tt and tt.is_alive:
                if RTB.check_vehicle_out_of_bounds(tt, carla_map, auto_destroy=True):
                    tt = None
                else:
                    target_spd = sm_tt.tick(tt.get_location(), sim_time, dt)
                    target_wp, idx_tt = RTB.get_target_waypoint(tt.get_location(), path_tt, idx_tt,
                                                                speed_kmh=target_spd)
                    if target_wp:
                        RTB.apply_pid_control(tt, pid_lon_tt, pid_lat_tt, target_spd, target_wp)

                    light_tt.auto_update_from_control()

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