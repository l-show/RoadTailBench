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
# 车辆轨迹原始数据 (已去除表头)
# ==========================================
RAW_TRUCK = """
53.812	107.087	-113.156
53.812	107.087	-113.156
53.812	107.087	-113.156
53.812	107.087	-113.086
53.195	105.615	-112.844
50.799	100.08	-113.86
47.648	92.402	-112.028
43.891	83.245	-112.937
39.831	73.706	-113.233
35.873	64.536	-113.391
31.812	55.147	-113.391
27.683	45.601	-113.391
27.357	44.849	-113.391
26.6	43.098	-113.391
24.592	38.472	-113.963
22.441	33.739	-114.552
20.336	29.084	-114.049
18.278	24.423	-113.562
16.195	19.648	-113.562
14.175	15.075	-115.063
11.759	10.512	-120.753
9.019	6.132	-123.237
6.205	1.999	-124.62
3.365	-2.116	-124.62
0.38	-6.229	-128.346
-2.899	-10.222	-130.909
-6.364	-13.823	-137.892
-10.667	-16.642	-153.713
-15.231	-18.878	-154.413
-19.756	-21.004	-155.393
-24.461	-23.138	-156.024
-29.131	-25.147	-156.935
-33.726	-27.176	-154.946
-38.362	-29.392	-154.436
-42.948	-31.585	-154.436
-47.652	-33.835	-154.436
-55.41	-37.547	-154.436
-63.222	-41.283	-154.436
-71.452	-45.026	-155.486
-79.665	-48.807	-155.277
-87.755	-52.507	-155.837
-95.884	-56.121	-156.117
-104.12	-59.852	-155.138
-112.059	-63.53	-155.138
-120.264	-67.333	-155.138
-128.205	-71.012	-155.138
-128.734	-71.258	-155.138
-128.734	-71.258	-155.138
"""

RAW_PICKUP = """
62.088	116.201	-113.01
62.088	116.201	-113.01
62.088	116.201	-113.01
62.088	116.201	-113.01
62.088	116.201	-113.644
62.088	116.201	-114.024
62.088	116.201	-113.604
61.134	114.014	-113.441
60.018	111.432	-113.371
56.967	104.232	-112.679
54.001	97.138	-112.819
50.955	90.036	-113.346
47.935	83.04	-113.345
44.833	75.891	-113.501
41.862	69.06	-113.501
38.785	62.199	-114.784
35.387	55.46	-119.943
31.175	49.187	-125.9
26.793	43.046	-123.669
23.001	36.536	-116.234
19.908	29.636	-113.425
16.842	22.584	-113.709
13.672	15.599	-116.207
9.797	9.042	-124.251
5.561	2.853	-124.392
1.325	-3.336	-124.392
-2.928	-9.549	-124.392
-7.014	-15.832	-116.165
-8.989	-23.262	-88.377
-6.807	-30.648	-66.796
-4.005	-37.868	-72.531
-2.617	-45.355	-84.113
-2.373	-50.096	-102.064
-2.811	-52.237	-101.747
-4.186	-58.542	-102.591
-5.646	-64.778	-103.59
-7.16	-70.942	-103.87
-8.658	-77.01	-103.87
-10.19	-83.179	-104.01
-11.746	-89.339	-104.431
-13.375	-95.481	-104.993
-15.039	-101.722	-104.923
-16.64	-107.765	-104.501
-18.17	-113.824	-104.01
-19.708	-119.989	-104.01
-21.275	-126.254	-104.29
-22.919	-132.5	-105.204
-24.613	-138.731	-105.204
-26.251	-144.76	-105.204
-27.849	-150.91	-104.291
-29.433	-157.172	-104.08
-31.004	-163.436	-104.08
-31.739	-166.368	-104.08
-31.739	-166.368	-104.08
"""

RAW_CAR = """
-78.404	-43.012	25.359
-78.404	-43.012	25.359
-78.404	-43.012	25.359
-78.404	-43.012	24.869
-67.972	-38.354	22.91
-57.42	-33.811	24.709
-46.861	-28.952	24.709
-39.833	-25.734	24.498
-31.139	-21.755	25.057
-26.229	-19.419	26.01
-21.39	-16.942	32.608
-17.616	-14.492	41.021
-14.164	-11.49	41.559
-11.583	-9.015	49.815
-7.9	-4.382	56.42
-5.156	-0.192	57.246
-2.2	4.635	60.473
-0.225	8.237	65.253
3.342	16.135	66.619
3.342	16.135	66.027
3.342	16.135	66.057
6.675	22.865	60.469
12.391	32.729	59.882
18.001	42.933	63.388
22.742	52.932	65.635
27.547	63.661	66.145
32.086	74.163	66.775
36.625	84.773	66.845
41.052	95.124	66.845
45.453	105.439	66.915
49.953	115.997	66.915
50.607	117.53	66.915
50.607	117.53	66.915
50.607	117.53	66.915
"""

RAW_EGO = """
61.399	105.018	-114.802
61.399	105.018	-114.802
61.399	105.018	-114.802
61.399	105.018	-115.317
60.126	102.344	-115.33
57.967	97.783	-114.912
55.837	93.054	-113.8
53.812	88.423	-113.053
51.817	83.737	-113.053
49.821	79.09	-113.41
47.842	74.48	-113.06
45.83	69.754	-113.06
43.792	65.042	-113.566
41.778	60.426	-113.566
40.59	57.701	-113.566
40.59	57.701	-113.566
40.59	57.701	-113.566
40.59	57.701	-113.566
40.59	57.701	-113.566
40.59	57.701	-115.697
40.59	57.701	-119.575
39.079	55.284	-124.668
37.77	53.525	-133.669
34.658	50.343	-134.468
33.401	49.063	-134.468
33.401	49.063	-134.468
30.552	46.16	-134.468
27.13	42.456	-129.197
24.247	38.357	-119.743
21.901	33.852	-115.837
19.872	29.269	-112.304
18.022	24.582	-111.224
16.166	19.788	-111.294
14.189	15.107	-115.882
11.582	10.651	-122.139
8.819	6.485	-124.819
5.842	2.263	-125.662
2.829	-1.935	-125.662
-0.086	-5.998	-125.662
-3.098	-10.195	-125.662
-5.96	-14.294	-122.589
-8.291	-18.712	-111.224
-9.427	-23.648	-93.063
-7.979	-28.511	-57.319
-4.263	-31.718	-23.337
0.827	-32.291	1.792
5.99	-32.117	1.511
11.146	-32.376	-7.428
16.209	-33.391	-14.666
21.037	-34.971	-21.972
25.337	-37.628	-41.917
28.66	-41.456	-53.306
31.588	-45.611	-55.531
34.526	-49.706	-53.834
37.544	-53.83	-53.948
40.435	-57.852	-54.685
43.406	-62.132	-55.598
46.424	-66.287	-53.054
49.48	-70.363	-53.475
52.421	-74.407	-54.317
55.422	-78.613	-54.809
58.42	-82.821	-53.538
61.449	-86.903	-53.677
64.475	-91.091	-54.311
67.454	-95.312	-54.737
70.346	-99.393	-54.667
73.25	-103.463	-54.104
76.28	-107.649	-54.104
79.301	-111.84	-54.385
82.292	-116.053	-54.666
"""

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05

        # ==========================================
        # 1. 环境初始化
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)
        RTB.set_static_weather(
            world, cloudiness=0.0, precipitation=30.0, precipitation_deposits=30.0,
            wind_intensity=5.0, sun_azimuth_angle=50.0, sun_altitude_angle=8.0,
            fog_density=0.0, fog_distance=0.0, fog_falloff=0.0, wetness=30.0,
            scattering_intensity=4.0, mie_scattering_scale=0.21, rayleigh_scattering_scale=0.05, dust_storm=0.0
        )

        # ==========================================
        # 2. 轨迹数据解析与稠密化
        # ==========================================
        traj_truck = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRUCK, min_dist=0.5), interval=0.5)
        traj_pickup = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_PICKUP, min_dist=0.5), interval=0.5)
        traj_car = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_CAR, min_dist=0.5), interval=0.5)
        traj_ego = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_EGO, min_dist=0.5), interval=0.5)

        # ==========================================
        # 3. 车辆安全生成
        # ==========================================
        truck = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', x=traj_truck[0][0], y=traj_truck[0][1],
                                  yaw=traj_truck[0][2], z_offset=0.2)
        pickup = RTB.spawn_vehicle(world, 'vehicle.tesla.cybertruck', x=traj_pickup[0][0], y=traj_pickup[0][1],
                                   yaw=traj_pickup[0][2], z_offset=0.2)
        car = RTB.spawn_vehicle(world, 'vehicle.dodge.charger_2020', x=traj_car[0][0], y=traj_car[0][1],
                                yaw=traj_car[0][2], z_offset=0.2)
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        vehicles = [truck, pickup, car, ego]
        for v in vehicles:
            if v: actor_list.append(v)

        if not actor_list:
            print("🚨 所有车辆均生成失败！")
            return

        # ==========================================
        # 4. 车辆控制模块
        # ==========================================
        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck')
        pid_lon_default = RTB.PIDLongitudinalController(preset='default_car')

        pid_lat_truck = RTB.PIDLateralController(preset='truck')
        pid_lat_pickup = RTB.PIDLateralController(K_P=1.0, K_I=0.02, K_D=0.3)
        pid_lat_car = RTB.PIDLateralController(K_P=0.9, K_I=0.02, K_D=0.3)

        if truck: RTB.set_vehicle_initial_speed(truck, 40.0, yaw_deg=traj_truck[0][2])
        if pickup: RTB.set_vehicle_initial_speed(pickup, 60.0, yaw_deg=traj_pickup[0][2])
        if car: RTB.set_vehicle_initial_speed(car, 60.0, yaw_deg=traj_car[0][2])

        idx_truck, idx_pickup, idx_car, idx_ego = 0, 0, 0, 0

        # ==========================================
        # 5. 剧本状态机编排
        # ==========================================
        sm_truck = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        sm_truck.add_stage(trigger_type='time', trigger_val=2.0, target_speed=70.0, accel=15.0)

        sm_pickup = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_pickup.add_stage(trigger_type='time', trigger_val=2.0, target_speed=30.0, accel=15.0)
        sm_pickup.add_stage(trigger_type='time', trigger_val=5.0, target_speed=70.0, accel=15.0)

        sm_car = RTB.MultiStageBehaviorMachine(initial_speed=60.0)

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        sim_time = 0.0
        LOOKAHEAD_RATIO = 0.8
        MIN_LOOKAHEAD = 8.0

        print("[主循环] 仿真开始，剧本已下发...")

        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # 【新增修复：检测是否所有车辆都已经挂了】
            # 如果列表里的所有车都不存在(被销毁或出界)，直接跳出循环，自然结束
            if not any(v and v.is_alive for v in vehicles):
                print("\n[运行状态] 🏁 所有车辆均已出界或被销毁，仿真正常结束！")
                break

            # ----------------- 车辆控制下发 -----------------
            # 【新增修复：每辆车独立判断 v and v.is_alive，绝不报错连累其他车】

            if truck and truck.is_alive:
                if not RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True):
                    target_spd = sm_truck.tick(truck.get_location(), sim_time, dt)
                    wp, idx_truck = RTB.get_target_waypoint(truck.get_location(), traj_truck, idx_truck, target_spd,
                                                            MIN_LOOKAHEAD, LOOKAHEAD_RATIO)
                    if wp: RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, target_spd, wp)

            if pickup and pickup.is_alive:
                if not RTB.check_vehicle_out_of_bounds(pickup, carla_map, auto_destroy=True):
                    target_spd = sm_pickup.tick(pickup.get_location(), sim_time, dt)
                    wp, idx_pickup = RTB.get_target_waypoint(pickup.get_location(), traj_pickup, idx_pickup, target_spd,
                                                             MIN_LOOKAHEAD, LOOKAHEAD_RATIO)
                    if wp: RTB.apply_pid_control(pickup, pid_lon_default, pid_lat_pickup, target_spd, wp)

            if car and car.is_alive:
                if not RTB.check_vehicle_out_of_bounds(car, carla_map, auto_destroy=True):
                    target_spd = sm_car.tick(car.get_location(), sim_time, dt)
                    wp, idx_car = RTB.get_target_waypoint(car.get_location(), traj_car, idx_car, target_spd,
                                                          MIN_LOOKAHEAD, LOOKAHEAD_RATIO)
                    if wp: RTB.apply_pid_control(car, pid_lon_default, pid_lat_car, target_spd, wp)

            # ==============================================================
            # 落地悬空期姿态锁死机制 (已加 is_alive 保护)
            # ==============================================================
            if sim_time < 0.5:
                for v in [pickup, car, ego]:
                    if v and v.is_alive:
                        ctrl = v.get_control()
                        ctrl.steer = 0.0
                        v.apply_control(ctrl)

            # 硬件时钟补齐
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[运行异常] {e}")
        # 这里加上了追踪报错行号的代码，以后再遇到崩溃你能瞬间看到是哪一行崩的
        import traceback
        traceback.print_exc()
    finally:
        if 'world' in locals():
            RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()