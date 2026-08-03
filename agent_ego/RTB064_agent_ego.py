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
# 轨迹数据硬编码
# ==========================================
RAW_TRAJ_1 = """
Location_x	Location_y	Rotation_yaw
-33.3	-43.749	9.745
-32.049	-43.53	9.955
-30.798	-43.304	10.305
-29.548	-43.072	10.936
-28.304	-42.818	11.787
-27.06	-42.559	11.787
-25.818	-42.296	11.998
-24.567	-42.078	6.286
-23.299	-42.008	0.901
-22.029	-42.009	-0.74
-20.76	-42.049	-2.644
-19.47	-42.11	-2.714
-18.201	-42.169	-2.151
-16.953	-42.197	-0.39
-15.683	-42.184	1.825
-14.416	-42.098	5.954
-13.138	-41.917	11.114
-11.905	-41.614	16.953
-10.721	-41.159	27.069
-9.599	-40.565	27.942
-8.477	-39.97	27.942
-7.371	-39.346	31.881
-6.311	-38.647	34.331
-5.271	-37.919	35.81
-4.266	-37.142	40.409
-3.348	-36.296	44.692
-2.458	-35.362	48.431
-1.686	-34.381	55.984
-1.038	-33.265	62.359
-0.769	-32.725	66.272
-0.736	-32.648	66.763
-0.28	-31.464	70.596
0.138	-30.264	71.894
0.496	-29.046	75.352
0.735	-27.8	81.813
1.079	-25.409	81.813
1.611	-20.446	88.499
1.605	-15.369	90.951
1.521	-10.291	90.951
1.449	-5.214	90.248
1.463	-0.22	89.618
1.497	4.858	89.548
1.542	9.934	89.478
1.595	15.74	89.478
1.676	24.624	89.478
1.754	33.509	89.688
1.768	42.394	90.038
1.813	51.141	89.61
1.877	60.029	89.75
1.849	68.922	90.31
1.779	77.815	90.73
1.666	86.706	90.73
1.611	95.597	90.097
1.596	104.342	90.097
1.588	109.047	90.097
"""

RAW_TRAJ_EGO = """
Location_x	Location_y	Rotation_yaw
-76.325	-51.548	10.168
-75.463	-51.393	10.168
-72.962	-50.948	9.745
-70.417	-50.501	10.308
-67.917	-50.039	10.728
-65.422	-49.558	11.149
-62.935	-49.039	12.064
-60.329	-48.482	12.064
-56.662	-47.705	11.502
-52.928	-46.946	11.502
-49.255	-46.198	11.502
-45.516	-45.468	10.38
-41.762	-44.812	9.742
-38.007	-44.167	9.742
-34.251	-43.523	9.742
-30.496	-42.878	9.742
-26.802	-42.243	9.742
-23.044	-41.616	7.817
-19.269	-41.826	-12.59
-15.514	-42.42	-2.274
-11.773	-42.291	9.076
-8.179	-41.071	27.192
-4.987	-39.002	39.416
-2.367	-36.244	52.986
-0.287	-33.053	59.926
0.969	-29.539	79.176
1.18	-25.741	89.144
1.256	-21.932	88.654
1.372	-18.125	88.161
1.477	-14.38	88.581
1.568	-10.572	88.651
1.656	-6.826	88.651
1.746	-3.018	88.651
1.878	2.58	88.651
1.876	8.927	91.296
1.731	15.171	91.366
1.587	21.518	91.156
1.494	27.868	90.31
1.494	34.219	89.89
1.524	40.57	89.68
1.559	46.817	89.68
1.625	53.168	89.33
1.698	59.415	89.33
1.772	65.765	89.33
"""

RAW_TRAJ_3 = """
Location_x	Location_y	Rotation_yaw
5.165	14.824	-88.368
5.19	13.951	-88.368
5.245	11.415	-89.278
5.276	8.88	-89.138
5.316	6.341	-89.208
5.351	3.802	-89.208
5.386	1.263	-89.208
5.418	-1.318	-89.348
5.447	-3.858	-89.348
5.476	-6.397	-89.348
5.505	-8.936	-89.348
5.51	-11.476	-93.68
5.13	-13.985	-103.86
4.54	-16.451	-94.953
4.483	-18.988	-88.551
4.617	-21.522	-86.452
4.764	-24.057	-86.942
4.778	-26.595	-92.714
4.524	-29.12	-99.768
3.964	-31.596	-106.276
2.998	-33.941	-117.461
1.604	-36.063	-124.845
-0.008	-38.023	-134.297
-1.928	-39.684	-140.662
-4.137	-40.917	-160.296
-6.529	-41.774	-160.296
-8.937	-42.58	-165.131
-11.462	-43.119	-169.193
-13.957	-43.596	-169.193
-16.453	-44.072	-169.193
-18.95	-44.542	-169.683
-21.449	-44.999	-169.543
-23.907	-45.453	-169.613
-26.407	-45.899	-169.892
-28.906	-46.354	-169.677
-31.897	-46.899	-169.677
-36.813	-47.794	-169.677
-41.893	-48.721	-169.397
-46.798	-49.669	-168.836
-51.782	-50.653	-168.836
-56.766	-51.636	-168.836
-61.751	-52.612	-169.186
-66.66	-53.539	-169.326
-71.652	-54.48	-169.326
-76.649	-55.394	-169.816
-80.767	-56.134	-169.816
-82.447	-56.435	-169.816
"""

def process_trajectory(raw_str):
    """提取原始字符串并使用标准库解析清洗与稠密化"""
    lines = [line for line in raw_str.strip().split('\n') if "Location" not in line]
    clean_str = '\n'.join(lines)

    # 解析并去重
    raw_pts = RTB.parse_string_trajectory(clean_str, min_dist=0.5)

    # 为防止 Z轴 被误认为偏航角插值，构建只有 x, y, z=0 的元组交给插值器
    xyz_pts = [(p[0], p[1], 0.0) for p in raw_pts]

    # 稠密化
    dense_path = RTB.interpolate_trajectory(xyz_pts, interval=0.5)

    # 返回起点(x,y,yaw) 和 稠密化后的PID轨迹
    return raw_pts[0], dense_path

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

        # 调用标准库接口配置天气（安全自动过滤非法参数）
        RTB.set_static_weather(
            world,
            cloudiness=45.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=10.0,
            sun_azimuth_angle=89.0,
            sun_altitude_angle=25.0,
            fog_density=3.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 天气系统已设置")

        # ==========================================
        # 2. 轨迹数据清洗与解析
        # ==========================================
        start_1, path_1 = process_trajectory(RAW_TRAJ_1)
        start_ego, path_ego = process_trajectory(RAW_TRAJ_EGO)
        start_3, path_3 = process_trajectory(RAW_TRAJ_3)

        # ==========================================
        # 3. 车辆实体安全生成
        # ==========================================
        v1 = RTB.spawn_vehicle(world, 'vehicle.lincoln.mkz_2020', x=start_1[0], y=start_1[1], yaw=start_1[2])
        v2_ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        v3 = RTB.spawn_vehicle(world, 'vehicle.citroen.c3', x=start_3[0], y=start_3[1], yaw=start_3[2])

        for v in [v1, v2_ego, v3]:
            if v: actor_list.append(v)

        # ==========================================
        # 4. 车辆PID控制器挂载与状态机编排
        # ==========================================
        pid_lon_1, pid_lat_1 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_3, pid_lat_3 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()

        idx_1 = idx_ego = idx_3 = 0

        # --- 车辆1状态机 (在x=-10减速到30km/h，过2s恢复60km/h) ---
        sm_1 = RTB.MultiStageBehaviorMachine(initial_speed=45.0)
        sm_1.add_stage(trigger_type='x_greater', trigger_val=-10.0, target_speed=30.0, accel=25.0)
        sm_1.add_stage(trigger_type='time', trigger_val=4.0, target_speed=60.0, accel=15.0)

        # --- Ego车辆状态机

        # ==========================================
        # 5. 车灯管理与初速度注入预热
        # ==========================================
        if v2_ego:
            pass

        RTB.set_vehicle_initial_speed(v1, target_speed_kmh=45.0)
        RTB.set_vehicle_initial_speed(v3, target_speed_kmh=50.0)

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        print("[主循环] 仿真开始运行...")
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 车辆 1 (Lincoln) 控制逻辑 ----------------
            if v1 and v1.is_alive:
                if RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True):
                    v1 = None
                else:
                    tgt_spd_1 = sm_1.tick(v1.get_location(), sim_time, dt)
                    tgt_wp_1, idx_1 = RTB.get_target_waypoint(v1.get_location(), path_1, idx_1, max(tgt_spd_1, 5.0))
                    if tgt_wp_1: RTB.apply_pid_control(v1, pid_lon_1, pid_lat_1, tgt_spd_1, tgt_wp_1)

            # ---------------- 车辆 2 (Ego TT) 控制逻辑 ----------------

            # ---------------- 车辆 3 (Citroen) 控制逻辑 ----------------
            if v3 and v3.is_alive:
                if RTB.check_vehicle_out_of_bounds(v3, carla_map, auto_destroy=True):
                    v3 = None
                else:
                    tgt_spd_3 = 50.0
                    tgt_wp_3, idx_3 = RTB.get_target_waypoint(v3.get_location(), path_3, idx_3, tgt_spd_3)
                    if tgt_wp_3: RTB.apply_pid_control(v3, pid_lon_3, pid_lat_3, tgt_spd_3, tgt_wp_3)

            # ---------------- 硬件时钟补齐 ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()