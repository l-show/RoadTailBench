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
# 原始轨迹数据 (已剔除第一行的文本表头，保留纯数据)
# 数据列分别为: Location_x, Location_y, Rotation_yaw
# ==========================================

RAW_PICKUP = """
126.85	1.63	172.219
126.85	1.63	172.219
126.85	1.63	172.219
122.562	2.15	174.374
117.51	2.511	178.053
112.366	2.482	-177.062
107.322	2.075	-173.832
102.386	1.431	-171.179
97.31	0.601	-170.679
92.308	-0.18	-172.244
87.265	-0.645	-177.747
82.198	-0.54	175.246
77.269	0.197	166.706
72.297	1.56	162.324
67.57	3.389	154.663
63.078	5.753	151.762
58.696	8.322	147.383
56.711	9.697	141.359
56.711	9.697	141.359
56.711	9.697	141.359
56.711	9.697	141.359
56.711	9.697	141.359
56.711	9.697	141.359
56.711	9.697	141.359
52.94	12.714	140.934
49.021	15.941	139.879
45.301	19.376	133.528
42.045	23.328	127.805
39.065	27.289	126.723
36.507	31.722	117.389
34.186	36.202	117.389
32.041	40.677	112.244
30.409	45.46	105.175
29.339	50.441	99.852
28.722	55.459	94.367
28.481	60.511	91.069
28.71	65.563	82.477
29.719	70.531	75.192
31.057	75.426	74.693
32.391	80.301	74.693
33.724	85.176	74.902
35.041	90.069	74.831
36.338	94.976	76.02
37.502	99.806	76.582
38.69	104.714	76.091
39.919	109.625	75.67
41.175	114.54	75.67
42.451	119.535	75.67
43.706	124.45	75.67
44.962	129.364	75.67
46.196	134.198	75.67
47.452	139.113	75.67
48.704	144.029	76.022
49.878	148.974	76.871
51.032	153.924	76.871
52.228	158.951	75.807
53.66	163.827	71.626
55.313	168.633	70.145
57.234	173.337	64.493
59.573	177.849	62.107
62.107	182.254	58.212
64.933	186.477	55.482
67.859	190.635	54.292
70.853	194.744	53.732
73.889	198.821	52.395
77.028	202.819	51.687
80.179	206.809	51.757
81.571	208.576	51.757
81.571	208.576	51.757
81.571	208.576	51.757
81.571	208.576	51.757
"""

RAW_EGO = """
231.374	-64.075	138.873
231.374	-64.075	138.873
231.374	-64.075	138.873
229.931	-62.814	138.873
226.076	-59.501	139.718
222.062	-56.248	142.787
217.986	-53.209	143.487
213.824	-50.148	144.055
209.628	-47.28	147.135
205.34	-44.549	147.905
201.037	-41.841	147.554
196.762	-39.089	146.924
192.518	-36.318	146.782
188.339	-33.582	146.782
185.755	-31.89	146.782
183.647	-30.5	146.255
181.601	-29.121	145.974
179.047	-27.397	145.974
175.94	-25.279	145.476
172.784	-23.107	145.335
169.673	-20.946	145.195
166.561	-18.78	145.125
163.439	-16.617	145.763
160.289	-14.486	145.972
157.136	-12.358	145.972
153.984	-10.229	145.972
150.79	-8.163	148.03
147.464	-6.32	153.069
144.06	-4.624	155.002
140.575	-3.103	157.78
137.016	-1.773	160.877
133.391	-0.632	163.957
129.708	0.307	168.061
125.967	0.989	171.288
122.2	1.49	173.498
118.415	1.829	176.72
114.616	1.945	-179.964
110.817	1.835	-176.766
107.029	1.546	-174.049
105.543	1.381	-173.061
105.543	1.381	-173.061
105.543	1.381	-173.061
105.543	1.381	-173.061
105.543	1.381	-173.061
105.543	1.381	-173.061
105.543	1.381	-173.061
105.543	1.381	-173.061
102.267	0.982	-173.061
98.435	0.514	-172.921
94.665	0.044	-172.851
90.894	-0.382	-174.476
87.048	-0.676	-178.056
83.249	-0.64	177.669
79.46	-0.317	172.473
75.719	0.373	167.458
72.821	1.134	162.881
72.821	1.134	162.881
72.821	1.134	162.881
72.821	1.134	162.881
72.821	1.134	162.881
72.821	1.134	162.881
72.821	1.134	162.881
69.332	2.322	158.237
65.836	3.836	154.858
62.463	5.608	149.703
59.172	7.652	146.489
56.08	9.879	143.407
53.003	12.228	141.857
50.054	14.628	139.229
47.293	17.231	134.337
44.728	20.015	130.068
42.291	22.981	129.285
39.958	25.949	125.883
37.827	29.066	123.234
35.794	32.325	121.172
33.861	35.572	119.528
32.131	38.932	114.474
30.78	42.469	108.58
30.289	43.943	108.37
30.289	43.943	108.37
30.289	43.943	108.37
30.289	43.943	108.37
30.289	43.943	108.37
29.956	44.946	108.23
29.022	48.619	98.901
28.579	52.395	93.37
28.474	56.258	89.617
28.523	60.061	88.113
28.839	63.846	84.138
29.298	67.616	81.259
29.992	71.351	78.372
30.761	75.071	77.877
31.581	78.784	77.524
32.415	82.556	77.524
33.24	86.201	76.326
34.216	89.876	74.495
35.25	93.602	74.495
36.634	98.59	74.495
38.668	105.922	74.495
40.628	113.264	75.482
42.489	120.49	75.621
44.37	127.839	75.831
46.197	135.2	76.111
47.724	141.377	76.111
49.205	147.54	76.739
50.692	153.715	76.107
"""

RAW_TRUCK = """
55.249	156.578	-104.604
55.249	156.578	-104.604
55.249	156.578	-104.604
54.845	155.027	-104.604
52.909	147.716	-105.235
50.915	140.365	-104.746
49.009	132.985	-104.397
47.108	125.603	-104.468
45.227	118.228	-104.259
43.355	110.858	-104.259
41.451	103.369	-104.259
39.578	95.998	-104.259
37.706	88.632	-104.259
35.895	81.507	-104.259
34.642	76.575	-104.259
33.441	71.654	-102.545
32.476	66.683	-99.828
31.876	61.665	-94.307
31.566	56.626	-92.458
31.527	51.578	-88.241
31.929	46.545	-82.509
32.854	41.495	-76.729
34.411	36.705	-65.358
36.851	32.315	-55.583
40.085	28.455	-45.778
43.732	24.956	-42.178
47.52	21.6	-41.263
51.325	18.261	-41.263
55.13	14.923	-41.263
59.076	11.749	-36.404
63.34	8.997	-29.641
67.877	6.712	-23.515
72.625	4.937	-17.199
77.519	3.669	-10.985
82.531	3.033	-4.868
87.583	2.912	2.146
92.631	3.276	5.774
97.655	3.898	7.425
102.675	4.557	7.495
107.696	5.205	6.429
112.663	5.569	0.984
117.727	5.557	-0.742
122.778	5.234	-7.716
127.756	4.312	-13.558
132.675	3.089	-14.201
137.533	1.661	-19.673
142.249	-0.189	-22.945
146.839	-2.341	-26.961
151.306	-4.746	-30.255
155.743	-7.374	-30.885
160.035	-10.072	-33.15
164.257	-12.864	-33.85
168.443	-15.687	-34.06
172.693	-18.561	-34.06
176.858	-21.412	-34.553
180.928	-24.249	-35.118
185.107	-27.197	-35.258
189.228	-30.097	-35.119
193.386	-32.972	-33.854
197.691	-35.814	-33.085
201.948	-38.587	-33.085
206.203	-41.368	-33.436
210.446	-44.169	-33.436
214.669	-46.999	-34.276
218.835	-49.912	-35.819
222.936	-52.914	-36.521
227.004	-55.961	-37.223
230.981	-58.99	-37.293
234.931	-62.054	-38.138
234.931	-62.054	-38.138
234.931	-62.054	-38.138
"""

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05

        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 严格按照用户截图数值配置静态天气
        RTB.set_static_weather(
            world,
            cloudiness=40.0,
            precipitation=20.0,
            precipitation_deposits=60.0,
            wind_intensity=10.0,
            sun_azimuth_angle=-1.0,
            sun_altitude_angle=1.0,
            fog_density=24.0,
            fog_distance=3.0,
            fog_falloff=0.0,
            wetness=60.0,
            scattering_intensity=0.5,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 天气系统已按照截图完美设置！")

        # 2. 轨迹数据硬编码与清洗稠密化
        # ==========================================
        # 步骤: 解析多列数据 -> 过滤冗余点 -> 插值稠密化到 0.5m 间距
        traj_pickup_raw = RTB.parse_string_trajectory(RAW_PICKUP, min_dist=0.5)
        traj_pickup = RTB.interpolate_trajectory(traj_pickup_raw, interval=0.5)

        traj_ego_raw = RTB.parse_string_trajectory(RAW_EGO, min_dist=0.5)
        traj_ego = RTB.interpolate_trajectory(traj_ego_raw, interval=0.5)

        traj_truck_raw = RTB.parse_string_trajectory(RAW_TRUCK, min_dist=0.5)
        traj_truck = RTB.interpolate_trajectory(traj_truck_raw, interval=0.5)

        print("[场景配置] 全局预设轨迹已绘制完毕。")

        # 3. 车辆安全生成
        # ==========================================
        # 皮卡生成 (取出第一个点作为出生点，注意提取yaw)
        pickup = RTB.spawn_vehicle(world, 'vehicle.tesla.cybertruck',
                                   x=traj_pickup[0][0], y=traj_pickup[0][1], yaw=traj_pickup[0][2], role_name="pickup")

        # Ego 奥迪 TT 生成
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        # 货车生成 (z_offset 需提高防止底盘卡地穿模)
        truck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.carlacola',
                                  x=traj_truck[0][0], y=traj_truck[0][1], yaw=traj_truck[0][2], role_name="truck",
                                  z_offset=1.5)

        actor_list.extend([actor for actor in [pickup, ego, truck] if actor is not None])

        # 4. 车辆初始速度注入与灯光管理
        # ==========================================
        if pickup:
            RTB.set_vehicle_initial_speed(pickup, 40.0, traj_pickup[0][2])
            lm_pickup = RTB.VehicleLightManager(pickup)
            # 开启行车灯、近光灯
            lm_pickup.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam)

        if ego:
            pass
            # 开启行车灯、远光灯

        if truck:
            RTB.set_vehicle_initial_speed(truck, 55.0, traj_truck[0][2])
            lm_truck = RTB.VehicleLightManager(truck)
            lm_truck.turn_on(carla.VehicleLightState.Position)  # 货车仅开行车灯

        # 5. 挂载独立专属的 PID 控制器
        # ==========================================
        # 皮卡 (偏重)
        pid_lon_pickup = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_pickup = RTB.PIDLateralController(preset='truck')

        # Ego小轿车

        # 中型货车
        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_truck = RTB.PIDLateralController(preset='truck')

        # 6. 长尾场景剧本状态机编排
        # ==========================================

        # 【车1】小皮卡：x=56减速到5km/h，等5s，恢复60km/h
        # 轨迹方向：X 从 126 降到 56 然后转弯。到达56意味着 X <= 57 时触发
        sm_pickup = RTB.MultiStageBehaviorMachine(initial_speed=10.0)
        sm_pickup.add_stage(trigger_type='time', trigger_val=3.0, target_speed=25.0, accel=15.0)
        sm_pickup.add_stage(trigger_type='x_less', trigger_val=63.0, target_speed=5.0, accel=15.0)
        sm_pickup.add_stage(trigger_type='time', trigger_val=5.0, target_speed=5.0, accel=15.0)
        sm_pickup.add_stage(trigger_type='immediate', target_speed=60.0, accel=20.0)

        # 【车2】EGO：第一次x=163减速到30，第一次x=105减速到30，x=72减速到5等5s恢复60
        # 轨迹方向：X 从 231 一路下降到 28

        # 【车3】中型货车：x减小到=30减速到20，等3s恢复50
        # 注意：真实数据中，该货车的X最小只跑到 31.527 就掉头了。为了确保触发，这里设置 x_less 32.5
        sm_truck = RTB.MultiStageBehaviorMachine(initial_speed=55.0)
        sm_truck.add_stage(trigger_type='x_less', trigger_val=32.5, target_speed=20.0, accel=15.0)
        sm_truck.add_stage(trigger_type='time', trigger_val=3.0, target_speed=20.0, accel=15.0)
        sm_truck.add_stage(trigger_type='immediate', target_speed=50.0, accel=15.0)

        # 寻路索引初始化
        idx_pickup = 0
        idx_ego = 0
        idx_truck = 0
        sim_time = 0.0

        print("\n[RoadTailBench] 🚀 仿真主循环启动...")

        # 7. 仿真主循环
        # ==========================================
        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 车辆1：Cybertruck 逻辑 ----------------
            if pickup and pickup.is_alive:
                if not RTB.check_vehicle_out_of_bounds(pickup, carla_map, auto_destroy=True):
                    loc = pickup.get_location()

                    # 1. 状态机推演平滑目标速度
                    target_speed = sm_pickup.tick(loc, sim_time, dt)

                    # 2. 动态预瞄找点
                    target_wp, idx_pickup = RTB.get_target_waypoint(loc, traj_pickup, idx_pickup,
                                                                    speed_kmh=target_speed)

                    # 3. 执行 PID
                    if target_wp:
                        RTB.apply_pid_control(pickup, pid_lon_pickup, pid_lat_pickup, target_speed, target_wp)

                    # 4. 车灯自动响应 (刹车/转弯灯等)
                    lm_pickup.auto_update_from_control()

            # ---------------- 车辆2：EGO (Audi TT) 逻辑 ----------------

            # ---------------- 车辆3：中型货车 (Carlacola) 逻辑 ----------------
            if truck and truck.is_alive:
                if not RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True):
                    loc = truck.get_location()

                    target_speed = sm_truck.tick(loc, sim_time, dt)
                    target_wp, idx_truck = RTB.get_target_waypoint(loc, traj_truck, idx_truck, speed_kmh=target_speed)

                    if target_wp:
                        RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, target_speed, target_wp)

                    lm_truck.auto_update_from_control()

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