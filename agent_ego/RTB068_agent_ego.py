# -*- coding: utf-8 -*-
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
# 轨迹原始数据定义 (使用多行字符串)
# ==========================================
TRAJ_BUS_RAW = """
Location_x	Location_y	Rotation_yaw
164.873	-107.506	7.158
168.843	-107.008	7.158
173.887	-106.386	7.019
183.943	-105.291	2.253
196.689	-105.203	3.386
206.458	-102.632	27.87
214.553	-96.629	43.593
220.865	-88.738	57.308
225.458	-79.786	70.574
226.784	-72.496	90.867
225.311	-65.176	106.191
219.676	-56.884	136.698
209.29	-50.009	152.463
198.23	-44.139	149.132
187.656	-37.062	145.166
175.142	-28.554	146.422
162.532	-20.413	147.824
149.868	-12.153	146.562
138.737	-6.168	157.621
129.119	-3.199	167.224
116.756	-1.14	178.983
104.248	-2.478	-171.621
89.279	-4.388	-177.539
76.682	-3.432	166.776
68.105	-0.647	159.851
59.683	3.227	150.773
49.53	10.631	138.346
40.771	19.867	130.663
33.15	29.961	123.486
27.371	41.097	112.273
24.753	53.368	89.819
25.372	66.09	82.877
28.131	78.612	75.217
31.475	91.285	75.217
35.143	105.938	77.323
39.433	124.284	75.072
44.459	142.35	75.94
48.778	160.973	76.791
54.513	179.001	59.471
61.59	187.868	43.941
70.898	194.555	26.748
81.699	198.329	12.392
93.003	198.879	-3.201
104.126	196.641	-18.93
"""

TRAJ_EGO_RAW = """
Location_x	Location_y	Rotation_yaw
92.028	-121.676	8.185
103.861	-119.929	8.406
118.945	-117.69	8.477
133.942	-115.47	8.407
149.022	-113.241	8.407
161.714	-111.367	8.407
172.66	-109.654	8.421
185.293	-107.925	7.861
197.865	-105.838	13.528
209.569	-101.085	37.177
218.569	-92.268	53.489
225.209	-81.514	62.967
227.884	-71.777	85.465
226.26	-61.885	114.288
219.217	-54.679	141.979
208.971	-47.217	145.503
200.213	-41.266	145.923
189.645	-33.711	145.187
177.037	-25.455	147.068
161.062	-15.083	147.136
146.642	-5.954	151.623
132.435	-0.519	165.345
117.344	1.936	177.439
102.213	1.071	-173.953
86.128	-0.507	-178.521
71.221	1.743	162.854
57.613	8.472	142.988
43.912	21.775	133.121
36.173	32.086	118.954
29.574	47.342	102.347
29.027	62.563	85.314
32.534	81.33	74.403
36.625	96.146	74.683
40.494	110.944	76.234
44.898	129.467	76.091
49.641	147.797	74.474
51.595	166.709	86.992
55.223	181.487	65.456
61.537	190.962	47.003
70.69	197.821	27.011
81.483	201.641	8.283
92.963	202.113	-1.665
104.259	200.205	-18.324
114.346	194.847	-43.396
121.71	186.156	-59.779
126.479	171.669	-80.642
127.747	160.244	-84.066
129.969	145.171	-77.899
132.72	133.958	-75.701
136.704	119.112	-72.364
143.21	105.363	-57.831
150.146	96.29	-47.515
161.909	86.686	-23.739
173.118	85.435	7.74
182.773	91.049	45.541
191.265	103.733	61.621
198.621	117.037	60.065
"""

TRAJ_MTRUCK_RAW = """
Location_x	Location_y	Rotation_yaw
90.581	-4.085	-178.049
81.938	-3.84	174.675
70.765	-1.703	162.36
60.469	2.847	150.199
50.871	9.026	142.429
40.678	20.457	128.452
31.856	32.823	113.456
29.194	47.654	91.598
28.672	62.884	91.318
29.221	78.153	80.972
31.36	89.445	76.734
34.104	100.606	74.958
38.546	117.29	74.605
41.671	129.337	75.308
44.741	141.582	77.13
47.455	153.975	75.535
49.939	163.874	76.883
53.088	176.139	71.969
57.256	185.439	61.515
63.53	193.331	42.79
72.139	198.671	26.112
81.83	201.688	11.65
91.898	202.469	-2.653
102.02	201.3	-15.305
111.358	197.365	-30.433
118.947	190.703	-52.734
124.022	182.001	-66.287
127.134	172.333	-74.506
129.263	164.68	-75.66
129.263	164.68	-75.66
129.263	164.68	-75.66
129.263	164.68	-75.66
129.263	164.68	-76.564
129.76	162.603	-76.288
130.377	160.181	-75.556
131.009	157.686	-75.905
131.617	155.202	-76.383
132.223	152.719	-76.032
132.838	150.251	-75.962
133.471	147.788	-75.287
134.132	145.279	-74.937
134.763	142.934	-74.936
135.436	140.423	-75.006
136.089	137.997	-74.935
136.737	135.59	-74.935
137.408	133.097	-74.935
138.077	130.621	-74.795
138.746	128.16	-74.795
139.415	125.71	-74.725
140.086	123.252	-74.725
140.735	120.877	-74.655
141.461	118.389	-72.531
142.308	116.018	-68.607
143.263	113.736	-66.477
144.349	111.339	-63.719
145.543	109.109	-60.138
146.888	106.876	-57.408
148.295	104.823	-54.092
149.841	102.763	-52.122
151.485	100.778	-49.104
153.209	98.879	-45.274
155.068	97.127	-42.409
156.955	95.422	-41.826
158.854	93.756	-40.968
160.829	92.041	-40.968
162.797	90.461	-35.296
163.607	89.888	-35.296
164.023	89.593	-35.296
164.432	89.304	-35.296
164.85	89.01	-34.11
165.28	88.735	-30.886
165.721	88.471	-30.886
166.693	87.89	-30.886
167.848	87.368	-16.795
169.116	87.113	-5.492
170.351	86.995	-5.492
171.637	87.008	7.199
172.889	87.252	14.413
174.078	87.647	21.971
175.239	88.171	27.313
176.343	88.818	34.545
177.346	89.605	40.387
178.269	90.494	47.778
179.058	91.452	53.615
179.782	92.51	57.274
180.466	93.592	57.76
181.15	94.677	57.76
181.969	95.975	57.76
185.153	101.023	57.76
188.499	106.52	60.196
191.583	112.036	60.937
194.683	117.612	60.797
197.828	123.181	60.237
200.978	128.57	59.272
204.311	134.091	58.712
207.621	139.538	58.712
210.905	144.942	58.712
211.93	146.629	58.712
212.137	146.969	58.712
212.137	146.969	58.712
"""

TRAJ_HTRUCK_RAW = """
Location_x	Location_y	Rotation_yaw
182.065	79.811	-161.877
170.821	77.06	179.765
160.125	80.884	147.061
151.09	87.784	140.041
140.812	99.09	126.291
134.049	112.749	109.65
129.331	127.241	105.571
125.312	141.955	104.502
121.538	156.793	104.364
117.692	171.473	104.714
110.437	188.983	131.023
101.026	195.382	155.514
89.871	197.637	179.252
78.607	195.869	-165.591
68.406	190.951	-138.27
62.469	181.27	-107.529
59.54	170.152	-104.097
55.602	155.361	-105.878
51.608	140.667	-104.593
47.79	125.949	-104.455
44.281	111.253	-101.861
41.238	96.403	-101.513
38.133	81.475	-102.146
34.737	66.545	-102.217
33.827	51.374	-80.976
39.177	37.258	-56.635
48.348	25.069	-47.921
59.363	14.586	-37.796
72.97	7.981	-14.407
87.94	5.97	2.726
103.091	6.735	2.586
118.399	7.094	-0.375
133.48	5.177	-13.549
143.236	2.502	-19.218
"""

def parse_custom_trajectory(raw_str):
    """自定义解析器：安全读取 X, Y, Z=0.0，并在第4位保留 Yaw (供 spawn 生成使用)"""
    pts = []
    lines = raw_str.strip().split('\n')[1:]  # 跳过表头
    for line in lines:
        parts = line.split()
        if len(parts) >= 3:
            pts.append((float(parts[0]), float(parts[1]), 0.0, float(parts[2])))
    # 清洗重复点，再稠密化到 0.5m 精度
    cleaned = RTB.clean_trajectory(pts, min_dist=0.5)
    dense = RTB.interpolate_trajectory(cleaned, interval=0.5)
    return dense, pts[0][3] if pts else 0.0  # 返回稠密轨迹和初始朝向角

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05

        # ==========================================
        # 1. 环境初始化：帧率同步与定制化天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按您提供的截图严格设置各项天气与散射参数
        RTB.set_static_weather(
            world,
            cloudiness=5.0, precipitation=70.0, precipitation_deposits=100.0,
            wind_intensity=85.0, sun_azimuth_angle=195.0, sun_altitude_angle=3.0,
            fog_density=2.0, fog_distance=0.75, fog_falloff=0.1,
            wetness=75.0, scattering_intensity=1.5, mie_scattering_scale=0.04,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )
        print("[场景配置] 天气系统已设置完毕。")

        # ==========================================
        # 2. 轨迹数据清洗与解析
        # ==========================================
        traj_bus, yaw_bus = parse_custom_trajectory(TRAJ_BUS_RAW)
        traj_ego, yaw_ego = parse_custom_trajectory(TRAJ_EGO_RAW)
        traj_mtruck, yaw_mtruck = parse_custom_trajectory(TRAJ_MTRUCK_RAW)
        traj_htruck, yaw_htruck = parse_custom_trajectory(TRAJ_HTRUCK_RAW)

        print("[场景配置] 轨迹解析与渲染完成。")

        # ==========================================
        # 3. 实体安全生成
        # ==========================================
        # 公交车 (由于体积庞大，提高 z_offset 防穿模)
        bus = RTB.spawn_vehicle(world, 'vehicle.mitsubishi.fusorosa', traj_bus[0][0], traj_bus[0][1], yaw=yaw_bus,
                                z_offset=1.5)
        # Ego 奥迪TT
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        # 中型货车
        mtruck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.carlacola', traj_mtruck[0][0], traj_mtruck[0][1],
                                   yaw=yaw_mtruck, z_offset=1.0)
        # 欧式重卡
        htruck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.european_hgv', traj_htruck[0][0], traj_htruck[0][1],
                                   yaw=yaw_htruck, z_offset=1.5)

        for act in filter(None, [bus, ego, mtruck, htruck]):
            actor_list.append(act)

        # ==========================================
        # 4. Ego 车灯系统设置
        # ==========================================
        if ego:
            pass

        # ==========================================
        # 5. 重构PID与状态机剧本编排
        # ==========================================
        # 为了突破重车动力瓶颈，对货车强制配置加强版纵向 PID
        heavy_pid = {'K_P': 2.5, 'K_I': 0.1, 'K_D': 0.05}

        # [公交车] 剧本：初始40 -> 1s后变30 -> 再过5s变35
        pid_lon_bus = RTB.PIDLongitudinalController(**heavy_pid, dt=dt)
        pid_lat_bus = RTB.PIDLateralController(preset='truck', dt=dt)
        sm_bus = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        sm_bus.add_stage('time', target_speed=30.0, trigger_val=1.0)
        sm_bus.add_stage('time', target_speed=40.0, trigger_val=2.0)
        sm_bus.add_stage('time', target_speed=20.0, trigger_val=5.0)

        # [Ego小车] 剧本：初始60 -> X>163减速30 -> Y>-41.266加速60 -> Y>0逐渐减速45

        # [中卡车] 剧本：初始1 -> X<35减速20 -> 3s后恢复45
        pid_lon_mtruck = RTB.PIDLongitudinalController(**heavy_pid, dt=dt)
        pid_lat_mtruck = RTB.PIDLateralController(preset='truck', dt=dt)
        sm_mtruck = RTB.MultiStageBehaviorMachine(initial_speed=1.0)
        sm_mtruck.add_stage('time', target_speed=10.0, trigger_val=20.0)
        sm_mtruck.add_stage('time', target_speed=40.0, trigger_val=5.0)
        sm_mtruck.add_stage('x_less', target_speed=20.0, trigger_val=35.0)
        sm_mtruck.add_stage('time', target_speed=45.0, trigger_val=3.0)

        # [重卡车] 剧本：初始10 -> 5s后加速20 -> 5s后加速40 -> 5s后恢复20
        pid_lon_htruck = RTB.PIDLongitudinalController(**heavy_pid, dt=dt)
        pid_lat_htruck = RTB.PIDLateralController(preset='truck', dt=dt)
        sm_htruck = RTB.MultiStageBehaviorMachine(initial_speed=1.0)
        sm_htruck.add_stage('time', target_speed=15.0, trigger_val=25.0)
        sm_htruck.add_stage('time', target_speed=30.0, trigger_val=5.0)
        sm_htruck.add_stage('time', target_speed=40.0, trigger_val=5.0)
        sm_htruck.add_stage('time', target_speed=20.0, trigger_val=5.0)

        # ==========================================
        # 6. 初始速度物理注入预热 (防止车辆起步抽搐打滑)
        # ==========================================
        RTB.set_vehicle_initial_speed(bus, target_speed_kmh=40.0, yaw_deg=yaw_bus)
        RTB.set_vehicle_initial_speed(mtruck, target_speed_kmh=1.0, yaw_deg=yaw_mtruck)
        RTB.set_vehicle_initial_speed(htruck, target_speed_kmh=10.0, yaw_deg=yaw_htruck)

        # 轨迹游标索引初始化
        idx_bus, idx_ego, idx_mtruck, idx_htruck = 0, 0, 0, 0
        sim_time = 0.0

        print("[仿真引擎] 初始化完成，准备进入主循环。")

        # ==========================================
        # 7. 仿真主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 公交车控制 ----------------
            if bus and bus.is_alive:
                if RTB.check_vehicle_out_of_bounds(bus, carla_map, auto_destroy=True):
                    pass
                else:
                    spd_bus = sm_bus.tick(bus.get_location(), sim_time, dt)
                    wp_bus, idx_bus = RTB.get_target_waypoint(bus.get_location(), traj_bus, idx_bus, spd_bus)
                    if wp_bus: RTB.apply_pid_control(bus, pid_lon_bus, pid_lat_bus, spd_bus, wp_bus)

            # ---------------- 中卡车控制 ----------------
            if mtruck and mtruck.is_alive:
                if RTB.check_vehicle_out_of_bounds(mtruck, carla_map, auto_destroy=True):
                    pass
                else:
                    spd_mtruck = sm_mtruck.tick(mtruck.get_location(), sim_time, dt)
                    wp_mtruck, idx_mtruck = RTB.get_target_waypoint(mtruck.get_location(), traj_mtruck, idx_mtruck,
                                                                    spd_mtruck)
                    if wp_mtruck: RTB.apply_pid_control(mtruck, pid_lon_mtruck, pid_lat_mtruck, spd_mtruck, wp_mtruck)

            # ---------------- 重型货车控制 ----------------
            if htruck and htruck.is_alive:
                if RTB.check_vehicle_out_of_bounds(htruck, carla_map, auto_destroy=True):
                    pass
                else:
                    spd_htruck = sm_htruck.tick(htruck.get_location(), sim_time, dt)
                    wp_htruck, idx_htruck = RTB.get_target_waypoint(htruck.get_location(), traj_htruck, idx_htruck,
                                                                    spd_htruck)
                    if wp_htruck: RTB.apply_pid_control(htruck, pid_lon_htruck, pid_lat_htruck, spd_htruck, wp_htruck)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体，确保引擎不会崩溃
        if 'world' in locals():
            RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()