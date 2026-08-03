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
# 原始轨迹数据 (直接存放多行文本)
# ==========================================
RAW_TRAJ_V1 = """
-76.627	1.213	-0.095
-71.549	1.213	0.185
-66.467	1.229	0.185
-61.384	1.224	-0.235
-56.303	1.213	0.255
-51.229	1.255	0.745
-46.329	1.342	1.235
-46.329	1.342	1.235
-46.329	1.342	1.235
-46.329	1.342	1.235
-46.329	1.342	1.235
-46.329	1.342	1.235
-42.761	1.419	1.235
-37.699	1.497	0.391
-32.715	1.513	-0.029
-27.564	1.51	-0.099
-22.494	1.539	0.989
-21.355	1.559	0.989
-20.847	1.568	0.989
-20.34	1.577	0.989
-19.833	1.585	1.059
-19.326	1.607	4.488
-18.828	1.65	4.989
-17.612	1.756	4.989
-16.333	1.922	9.585
-15.11	2.176	14.406
-13.908	2.585	21.742
-12.794	3.199	35.285
-11.789	3.975	40.452
-10.904	4.857	47.982
-10.117	5.853	55.632
-9.493	6.958	64.01
-8.961	8.111	72.844
-8.756	9.34	87.174
-8.735	10.611	91.734
-8.79	11.86	92.672
-8.848	13.129	92.033
-8.885	14.4	91.61
-8.969	17.378	91.61
-9.072	21.19	91.33
-9.155	25.002	91.12
-9.21	28.752	90.77
-9.25	32.564	90.56
-9.28	36.377	90.42
-9.308	40.189	90.42
-9.339	44.002	90.49
-9.372	47.814	90.49
-9.404	51.564	90.49
-9.447	55.377	90.84
-9.503	59.189	90.84
-9.558	62.939	90.84
-9.614	66.751	90.84
-9.662	70.563	90.7
-9.675	71.626	90.7
-9.675	71.626	90.7
-9.675	71.626	90.7
-9.675	71.626	90.7
-9.675	71.626	90.7
"""

RAW_TRAJ_EGO = """
-117.171	0.829	0.741
-117.171	0.829	0.741
-117.171	0.829	0.741
-114.359	0.858	0.248
-110.546	0.872	0.178
-106.733	0.873	-0.172
-102.921	0.843	-0.592
-99.108	0.828	0.038
-95.358	0.855	0.948
-91.484	0.93	1.158
-87.672	1.007	1.158
-83.86	1.091	1.438
-80.048	1.187	1.438
-76.237	1.281	1.368
-72.425	1.362	1.085
-68.613	1.434	1.085
-64.801	1.506	1.085
-60.989	1.576	1.015
-57.177	1.643	1.015
-53.304	1.712	1.015
-49.557	1.778	1.015
-45.747	1.843	0.735
-41.941	1.886	0.595
-38.14	1.925	0.595
-34.343	1.961	0.315
-30.55	1.972	0.105
-26.758	1.979	0.245
-22.955	2.005	0.525
-19.143	2.052	0.875
-15.332	2.12	1.085
-11.521	2.191	0.875
-7.772	2.248	0.875
-3.897	2.305	0.735
-0.064	2.346	0.665
3.683	2.413	1.295
7.493	2.504	1.505
11.367	2.606	1.505
15.324	2.71	1.505
20.407	2.843	1.435
25.49	2.95	0.735
30.572	3.022	1.088
35.656	3.117	0.878
40.655	3.185	0.878
45.735	3.263	0.878
50.815	3.336	0.738
55.815	3.395	0.668
60.897	3.467	0.878
65.979	3.545	0.878
71.062	3.623	0.878
76.145	3.701	0.878
"""

RAW_TRAJ_V3 = """
73.868	-0.803	179.848
73.868	-0.803	179.848
65.368	-0.78	179.848
50.366	-0.864	-178.945
35.121	-1.225	-178.449
19.876	-1.618	-178.449
4.631	-2.012	-178.659
-10.615	-2.384	-178.659
-25.862	-2.583	-179.149
-40.848	-2.821	-179.079
-56.08	-3.144	-179.009
-71.064	-3.364	-179.221
-86.31	-3.587	-179.151
-101.549	-3.8	-179.361
-113.542	-3.934	-179.361
-113.542	-3.934	-179.361
-113.542	-3.934	-179.361
"""

RAW_TRAJ_V4 = """
-4.743	67.524	-89.066
-4.743	67.524	-89.066
-4.743	67.524	-89.066
-4.7	64.172	-89.559
-4.757	55.276	-90.474
-4.764	46.382	-89.623
-4.689	37.488	-89.413
-4.593	28.595	-89.343
-4.493	19.699	-89.413
-4.454	10.803	-90.182
-4.479	1.908	-89.972
-4.398	-6.981	-89.272
-4.264	-15.874	-88.849
-4.112	-24.622	-89.199
-4.056	-33.517	-89.759
-4.023	-42.414	-89.829
-3.991	-51.31	-89.689
-3.943	-60.202	-89.689
-3.895	-69.094	-89.689
-3.855	-78.136	-89.759
-3.817	-87.033	-89.759
-3.781	-95.783	-89.759
-3.743	-104.68	-89.759
-3.71	-113.576	-89.829
-3.707	-114.597	-89.829
-3.707	-114.597	-89.829
-3.707	-114.597	-89.829
"""

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

        # 根据截图参数完美设置天气
        weather_kwargs = {
            'cloudiness': 5.0000,
            'precipitation': 0.0000,
            'precipitation_deposits': 0.0000,
            'wind_intensity': 10.0000,
            'sun_azimuth_angle': 139.0000,
            'sun_altitude_angle': 35.0000,
            'fog_density': 2.0000,
            'fog_distance': 0.7500,
            'fog_falloff': 0.1000,
            'wetness': 0.0000,
            'scattering_intensity': 1.0000,
            'mie_scattering_scale': 0.0300,
            'rayleigh_scattering_scale': 0.0331,
            'dust_storm': 0.0000
        }
        RTB.set_static_weather(world, **weather_kwargs)
        print("[场景配置] 天气系统已设置。")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化 (0.5m)
        # ==========================================
        print("[场景配置] 正在解析并稠密化轨迹...")
        # V1 轨迹处理
        raw_pts_v1 = RTB.parse_string_trajectory(RAW_TRAJ_V1, min_dist=0.1)
        traj_v1 = RTB.interpolate_trajectory(raw_pts_v1, interval=0.5)

        # V2 (Ego) 轨迹处理
        raw_pts_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO, min_dist=0.1)
        traj_ego = RTB.interpolate_trajectory(raw_pts_ego, interval=0.5)

        # V3 轨迹处理
        raw_pts_v3 = RTB.parse_string_trajectory(RAW_TRAJ_V3, min_dist=0.1)
        traj_v3 = RTB.interpolate_trajectory(raw_pts_v3, interval=0.5)

        # V4 轨迹处理
        raw_pts_v4 = RTB.parse_string_trajectory(RAW_TRAJ_V4, min_dist=0.1)
        traj_v4 = RTB.interpolate_trajectory(raw_pts_v4, interval=0.5)

        print("[场景配置] 预设轨迹绘制完成。")

        # ==========================================
        # 3. 车辆生成与预热 (利用轨迹第一个点进行安全生成)
        # ==========================================
        # 第1辆：Jeep Wrangler (V1)
        v1 = RTB.spawn_vehicle(world, 'vehicle.jeep.wrangler_rubicon', x=traj_v1[0][0], y=traj_v1[0][1],
                               yaw=traj_v1[0][2])
        if v1: actor_list.append(v1)

        # 第2辆：Audi TT (Ego)
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        # 第3辆：Citroen C3 (V3)
        v3 = RTB.spawn_vehicle(world, 'vehicle.citroen.c3', x=traj_v3[0][0], y=traj_v3[0][1], yaw=traj_v3[0][2])
        if v3: actor_list.append(v3)

        # 第4辆：Citroen C3 (V4)
        v4 = RTB.spawn_vehicle(world, 'vehicle.citroen.c3', x=traj_v4[0][0], y=traj_v4[0][1], yaw=traj_v4[0][2])
        if v4: actor_list.append(v4)

        # ==========================================
        # 4. 速度赋予与车灯控制
        # ==========================================
        # V1: 初速度 60 km/h
        RTB.set_vehicle_initial_speed(v1, 60.0, yaw_deg=traj_v1[0][2])

        # Ego: 初速度 65 km/h，开启行车灯

        # V3: 初速度 50 km/h
        RTB.set_vehicle_initial_speed(v3, 50.0, yaw_deg=traj_v3[0][2])

        # V4: 初速度 50 km/h
        RTB.set_vehicle_initial_speed(v4, 50.0, yaw_deg=traj_v4[0][2])

        # ==========================================
        # 5. PID控制器挂载 (每辆车独立分配)
        # ==========================================
        pid_lon_v1 = RTB.PIDLongitudinalController()
        pid_lat_v1 = RTB.PIDLateralController()

        pid_lon_v3 = RTB.PIDLongitudinalController()
        pid_lat_v3 = RTB.PIDLateralController()

        pid_lon_v4 = RTB.PIDLongitudinalController()
        pid_lat_v4 = RTB.PIDLateralController()

        # 维护每辆车的寻路索引
        idx_v1, idx_ego, idx_v3, idx_v4 = 0, 0, 0, 0

        # ==========================================
        # 6. 剧本状态机编排
        # ==========================================
        # V1剧本：在 x=-46.329 减速到 20km/h，过2s恢复 55km/h (向东开，X逐渐变大，所以用 'x_greater')
        sm_v1 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_v1.add_stage(trigger_type='x_greater', trigger_val=-46.33, target_speed=20.0, accel=25.0)
        sm_v1.add_stage(trigger_type='time', trigger_val=2.0, target_speed=55.0, accel=15.0)

        # Ego剧本：在 x=-45 减速到 25km/h，过3s恢复 70km/h

        print("[场景配置] 剧本状态机加载完毕，仿真开始！")

        # ==========================================
        # 7. 仿真主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- A. 守护：出界检测与销毁 ----------------
            RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(v3, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(v4, carla_map, auto_destroy=True)

            # ---------------- B. V1 车辆控制 ----------------
            if v1 and v1.is_alive:
                target_spd_v1 = sm_v1.tick(v1.get_location(), sim_time, dt)
                target_wp_v1, idx_v1 = RTB.get_target_waypoint(v1.get_location(), traj_v1, idx_v1,
                                                               speed_kmh=target_spd_v1)
                RTB.apply_pid_control(v1, pid_lon_v1, pid_lat_v1, target_spd_v1, target_wp_v1)

            # ---------------- C. Ego 车辆控制与预瞄点绘制 ----------------

            # ---------------- D. V3 车辆控制 (维持50) ----------------
            if v3 and v3.is_alive:
                target_wp_v3, idx_v3 = RTB.get_target_waypoint(v3.get_location(), traj_v3, idx_v3, speed_kmh=50.0)
                RTB.apply_pid_control(v3, pid_lon_v3, pid_lat_v3, 50.0, target_wp_v3)

            # ---------------- E. V4 车辆控制 (维持50) ----------------
            if v4 and v4.is_alive:
                target_wp_v4, idx_v4 = RTB.get_target_waypoint(v4.get_location(), traj_v4, idx_v4, speed_kmh=50.0)
                RTB.apply_pid_control(v4, pid_lon_v4, pid_lat_v4, 50.0, target_wp_v4)

            # ---------------- F. 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
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