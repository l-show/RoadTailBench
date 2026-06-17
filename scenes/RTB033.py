import sys
import carla
import time
import random
import math

# ==========================================
# 1. 动态引入标准化函数库路径
# ==========================================
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 2. 预设轨迹数据硬编码 (已去除原始数据的表头)
# ==========================================
TRAJ_STR_VAN = """
9.123	-91.146	-91.165
9.123	-91.146	-91.165
9.123	-91.146	-91.165
9.123	-91.146	-91.165
9.123	-91.146	-91.165
9.123	-91.146	-91.305
9.076	-92.728	-91.867
8.991	-95.268	-91.867
8.965	-96.051	-91.867
8.949	-96.559	-91.867
8.932	-97.068	-91.867
8.891	-98.325	-91.587
8.955	-99.594	-84.544
9.153	-100.849	-78.644
9.492	-102.072	-67.84
10.056	-103.209	-59.423
10.846	-104.199	-42.764
11.771	-105.039	-37.318
12.884	-105.645	-21.95
14.105	-105.99	-10.683
15.365	-106.139	-3.014
16.635	-106.18	-1.705
17.904	-106.218	-1.705
21.672	-106.33	-1.705
26.753	-106.385	1.174
31.833	-106.236	1.672
36.915	-106.167	-0.38
41.915	-106.242	-1.291
46.995	-106.355	-1.081
52.078	-106.439	-0.938
57.16	-106.519	-0.868
62.242	-106.596	-0.868
67.242	-106.646	-0.028
72.324	-106.632	0.182
77.406	-106.631	-0.526
82.489	-106.725	-1.442
87.571	-106.856	-1.512
92.652	-106.996	-1.652
97.648	-107.142	-1.722
102.811	-107.297	-1.722
107.889	-107.45	-1.722
112.969	-107.622	-2.072
118.049	-107.809	-2.142
123.128	-107.999	-2.142
128.204	-108.271	-4.189
133.268	-108.712	-6.028
138.319	-109.279	-6.588
143.367	-109.877	-7.218
148.403	-110.55	-7.708
153.357	-111.219	-7.569
158.4	-111.862	-6.939
163.281	-112.455	-6.939
168.546	-113.106	-7.289
173.588	-113.754	-7.639
178.536	-114.466	-8.633
185.519	-115.526	-8.633
195.568	-117.052	-8.633
204.632	-118.428	-8.633
209.368	-119.147	-8.633
209.368	-119.147	-8.633
209.368	-119.147	-8.633
"""

TRAJ_STR_CAR = """
1.328	-124.762	86.941
1.328	-124.762	86.941
1.328	-124.762	86.941
1.328	-124.762	86.941
1.328	-124.762	86.941
1.334	-124.654	86.941
1.36	-124.155	86.941
1.407	-123.29	86.941
1.473	-122.042	86.941
1.542	-120.752	86.941
1.609	-119.504	86.941
1.627	-119.171	86.941
1.627	-119.171	86.941
1.627	-119.171	82.518
2.035	-118.062	62.585
2.657	-116.979	58.169
3.33	-115.901	57.679
4.01	-114.827	57.679
4.7	-113.736	57.679
5.411	-112.683	53.768
6.205	-111.718	47.814
7.084	-110.772	46.032
8.008	-109.9	41.712
8.984	-109.086	37.306
10.034	-108.371	31.741
11.133	-107.731	28.516
12.277	-107.181	21.408
13.475	-106.76	17.01
14.71	-106.469	10.143
15.951	-106.329	2.771
17.22	-106.276	1.791
18.49	-106.243	1.231
19.76	-106.216	1.231
21.031	-106.199	-0.465
22.302	-106.216	-0.821
23.593	-106.235	-0.821
29.342	-106.317	-0.821
38.008	-106.442	-0.821
48.174	-106.517	0.17
58.341	-106.493	-0.18
68.507	-106.561	-0.46
78.506	-106.641	-0.46
88.672	-106.755	-1.02
98.838	-106.966	-1.37
108.997	-107.283	-2.213
118.984	-107.793	-3.336
129.127	-108.478	-4.609
139.25	-109.411	-6.601
149.339	-110.674	-7.237
159.429	-111.915	-6.957
169.519	-113.156	-7.307
179.593	-114.524	-7.867
189.645	-116.035	-9.353
199.68	-117.645	-8.794
209.745	-119.073	-7.814
219.817	-120.455	-7.814
229.894	-121.803	-7.464
239.973	-123.13	-7.534
250.051	-124.463	-7.534
250.714	-124.551	-7.534
250.714	-124.551	-7.534
250.714	-124.551	-7.534
"""

TRAJ_STR_EGO = """
6.712	10.279	-91.584
6.712	10.279	-91.584
6.712	10.279	-91.584
6.705	8.862	-89.927
6.717	3.78	-89.857
6.706	-1.303	-90.277
6.682	-6.386	-90.347
6.637	-11.469	-90.767
6.558	-16.635	-91.187
6.45	-21.718	-91.257
6.338	-26.8	-91.257
6.227	-31.882	-91.257
6.116	-36.963	-91.187
6.012	-42.045	-91.117
5.931	-47.128	-90.554
5.91	-52.211	-89.924
5.919	-57.21	-90.064
5.903	-62.294	-90.204
5.884	-67.377	-90.204
5.866	-72.461	-90.204
5.853	-76.232	-90.204
5.851	-76.853	-90.204
5.849	-77.361	-90.204
5.847	-77.861	-90.204
5.845	-78.37	-90.204
5.845	-78.503	-90.204
5.845	-78.503	-90.204
5.845	-78.503	-90.204
5.845	-78.503	-90.204
5.844	-78.653	-90.484
5.835	-79.169	-91.049
5.826	-79.678	-91.049
5.817	-80.186	-91.049
5.808	-80.694	-91.049
5.798	-81.202	-91.119
5.788	-81.71	-91.119
5.778	-82.218	-91.119
5.766	-82.718	-91.679
5.751	-83.251	-91.679
5.693	-85.229	-91.679
5.618	-87.77	-91.679
5.548	-90.166	-91.679
5.511	-91.436	-91.679
5.474	-92.706	-91.679
5.442	-93.78	-91.679
5.427	-94.289	-91.679
5.412	-94.797	-91.679
5.398	-95.305	-91.679
5.383	-95.813	-91.679
5.357	-96.321	-93.267
5.327	-96.82	-94.571
5.254	-97.322	-102.27
5.123	-97.822	-107.405
4.954	-98.301	-112.463
4.72	-98.752	-118.992
4.471	-99.195	-120.042
4.154	-99.742	-120.042
3.501	-100.832	-123.149
2.784	-101.881	-125.448
2.04	-102.885	-128.048
1.211	-103.847	-131.602
0.367	-104.797	-131.742
-0.506	-105.691	-136.591
-1.446	-106.547	-139.176
-2.449	-107.326	-144.62
-3.517	-108.013	-150.478
-4.663	-108.56	-157.561
-5.838	-108.984	-164.081
-7.096	-109.275	-169.157
-8.328	-109.484	-171.796
-9.591	-109.614	-177.122
-10.861	-109.669	-178.032
-12.131	-109.697	-179.722
-13.402	-109.698	-179.935
-17.319	-109.703	-179.935
-22.402	-109.666	179.005
-27.485	-109.63	179.712
-32.568	-109.602	179.502
-37.651	-109.541	179.222
-42.65	-109.464	179.082
-47.731	-109.382	179.082
-52.814	-109.302	179.152
-57.896	-109.227	179.152
-62.978	-109.171	-179.634
-68.061	-109.226	-178.861
-73.054	-109.463	-175.339
-78.099	-110.08	-170.781
-83.078	-111.096	-165.283
-87.834	-112.631	-159.167
-92.582	-114.669	-154.257
-96.957	-117.084	-147.654
-101.141	-119.97	-142.842
-104.967	-123.31	-134.795
-108.275	-127.054	-128.206
-111.297	-131.141	-124.087
-113.923	-135.49	-117.632
-116.007	-140.214	-110.705
-117.486	-145.074	-103.298
-118.459	-149.977	-98.713
-119.074	-155.022	-95.985
-119.588	-160.075	-95.565
-120.08	-165.045	-95.775
-120.331	-167.529	-95.775
-120.331	-167.529	-95.775
-120.331	-167.529	-95.775
"""

TRAJ_STR_PED = """
20.842	-99.118	-179.937
18.301	-99.122	-179.866
15.761	-99.176	-176.491
13.24	-99.477	-169.92
11.646	-99.788	-168.688
11.646	-99.788	-168.688
11.646	-99.788	-168.688
11.646	-99.788	-168.688
11.646	-99.788	-168.688
9.56	-100.189	-170.258
7.049	-100.571	-172.741
4.558	-100.761	-178.61
2.017	-100.766	178.875
-0.523	-100.668	177.167
-3.019	-100.531	176.747
-5.556	-100.386	176.747
-8.094	-100.245	177.029
-10.632	-100.114	177.029
-13.211	-99.97	175.741
-15.688	-99.652	168.203
-18.173	-99.118	165.602
-20.568	-98.276	155.62
-22.839	-97.138	152.012
-25.084	-95.945	152.082
-27.325	-94.748	151.872
-29.567	-93.55	151.872
-31.808	-92.352	151.872
-34.013	-91.173	151.872
-35.446	-90.407	151.872
-35.446	-90.407	151.872
-35.446	-90.407	151.872
"""


def get_real_speed_kmh(vehicle):
    """计算车辆当前真实速度 km/h"""
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


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
        # 1. 环境初始化：开启帧率同步与静态天气设定
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按截图中具体数值精准还原长尾天气（地表积水并伴随大风与薄雾）
        weather = carla.WeatherParameters(
            cloudiness=20.0, precipitation=0.0, precipitation_deposits=75.0, wind_intensity=60.0,
            sun_azimuth_angle=-1.0, sun_altitude_angle=15.0, fog_density=2.0, fog_distance=0.75,
            fog_falloff=0.1, wetness=75.0, scattering_intensity=1.0, mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 同步模式与天气系统已设置完成")

        # ==========================================
        # 2. 轨迹数据解析与清洗稠密化 (间距0.5m)
        # ==========================================
        # 车1: 小货车
        raw_traj_van = RTB.parse_string_trajectory(TRAJ_STR_VAN, min_dist=0.1)
        dense_traj_van = RTB.interpolate_trajectory(raw_traj_van, interval=0.5)

        # 车2: 小轿车
        raw_traj_car = RTB.parse_string_trajectory(TRAJ_STR_CAR, min_dist=0.1)
        dense_traj_car = RTB.interpolate_trajectory(raw_traj_car, interval=0.5)

        # 车3: EGO 车辆
        raw_traj_ego = RTB.parse_string_trajectory(TRAJ_STR_EGO, min_dist=0.1)
        dense_traj_ego = RTB.interpolate_trajectory(raw_traj_ego, interval=0.5)

        # 行人轨迹
        raw_traj_ped = RTB.parse_string_trajectory(TRAJ_STR_PED, min_dist=0.1)
        dense_traj_ped = RTB.interpolate_trajectory(raw_traj_ped, interval=0.5)

        # 🚀 绘制出所有车辆的预设路线轨迹线（灰色永久显示）
        RTB.draw_preset_trajectory(world, dense_traj_van, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, dense_traj_car, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, dense_traj_ego, color=carla.Color(150, 150, 150))

        # ==========================================
        # 3. 实体生成与初始状态注入
        # ==========================================
        # 【车辆1】 小货车 Sprinter
        yaw_van = raw_traj_van[0][2]
        van = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', raw_traj_van[0][0], raw_traj_van[0][1], yaw=yaw_van,
                                role_name="van")
        if van:
            actor_list.append(van)
            RTB.set_vehicle_initial_speed(van, 0.0)

        # 【车辆2】 小轿车 Impala
        yaw_car = raw_traj_car[0][2]
        car = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', raw_traj_car[0][0], raw_traj_car[0][1], yaw=yaw_car,
                                role_name="car")
        if car:
            actor_list.append(car)
            RTB.set_vehicle_initial_speed(car, 0.0)
            # 配置灯光：开启行车灯
            car_lights = RTB.VehicleLightManager(car)
            car_lights.set_static_lights(low_beam=False, high_beam=False)

            # 【车辆3】 EGO主车 Audi TT
        yaw_ego = raw_traj_ego[0][2]
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', raw_traj_ego[0][0], raw_traj_ego[0][1], yaw=yaw_ego,
                                role_name="ego")
        if ego:
            actor_list.append(ego)
            RTB.set_vehicle_initial_speed(ego, 70.0)
            # 配置灯光：开启行车灯、近光灯
            ego_lights = RTB.VehicleLightManager(ego)
            ego_lights.set_static_lights(low_beam=True, high_beam=False)

        # 【行人】
        ped_bp = bp_lib.filter('walker.pedestrian.*')[0]
        if ped_bp.has_attribute('is_invincible'):
            ped_bp.set_attribute('is_invincible', 'false')
        # Z轴稍微抬高1.0m防止卡死在地底下
        ped_trans = carla.Transform(carla.Location(raw_traj_ped[0][0], raw_traj_ped[0][1], 1.0),
                                    carla.Rotation(yaw=raw_traj_ped[0][2]))
        walker = world.try_spawn_actor(ped_bp, ped_trans)
        if walker:
            actor_list.append(walker)
            # 挂载次世代行人控制器，传入预设好的稠密轨迹点
            ped_ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=dense_traj_ped)

        # ==========================================
        # 4. 建立独立PID控制器 & 多阶段状态机剧本编排
        # ==========================================
        # 1. 货车剧本：静止9s -> 加速到30 -> 3s后加速到70
        pid_lon_van = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_van = RTB.PIDLateralController(preset='truck')
        van_sm = RTB.MultiStageBehaviorMachine(initial_speed=0.0)
        van_sm.add_stage('time', target_speed=30.0, trigger_val=9.0, accel=10.0)
        van_sm.add_stage('time', target_speed=70.0, trigger_val=3.0, accel=15.0)
        idx_van = 0

        # 2. 轿车剧本：静止2s -> 加速到35 -> 3s后加速到70
        pid_lon_car = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_car = RTB.PIDLateralController(preset='default_car')
        car_sm = RTB.MultiStageBehaviorMachine(initial_speed=0.0)
        car_sm.add_stage('time', target_speed=35.0, trigger_val=2.0, accel=12.0)
        car_sm.add_stage('time', target_speed=70.0, trigger_val=3.0, accel=15.0)
        idx_car = 0

        # 3. EGO剧本：初始70。随着车辆沿着单调减小的Y轴前进，依次触发减速和恢复 (由于Y轴不断减小，采用 y_less)
        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=70.0)
        ego_sm.add_stage('y_less', target_speed=20.0, trigger_val=-48.0, accel=20.0)
        ego_sm.add_stage('y_less', target_speed=40.0, trigger_val=-70.0, accel=20.0)
        ego_sm.add_stage('y_less', target_speed=20.0, trigger_val=-90.0, accel=20.0)
        ego_sm.add_stage('time', target_speed=70.0, trigger_val=3.0, accel=15.0)
        idx_ego = 0

        # 4. 行人剧本：按 3m/s 移动，当到达 x < 11 坐标时，开始以 5m/s 跑步 (由于X轴从36递减到负数，采用 x_less)
        ped_sm = RTB.MultiStageBehaviorMachine(initial_speed=3)
        ped_sm.add_stage('x_less', target_speed=5, trigger_val=13.0, accel=50.0)

        # ==========================================
        # 5. 仿真主循环
        # ==========================================
        sim_time = 0.0
        print("\n[仿真开始] 长尾剧本已启动！进入主循环...")

        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 货车控制流 ----------------
            # 加入出界虚空拦截判定
            if van and van.is_alive and not RTB.check_vehicle_out_of_bounds(van, carla_map, auto_destroy=True):
                # 状态机输出当前帧的目标平滑速度
                target_spd_van = van_sm.tick(van.get_location(), sim_time, dt)
                # 计算出前方轨迹合适的预瞄点
                wp_van, idx_van = RTB.get_target_waypoint(van.get_location(), dense_traj_van, idx_van,
                                                          get_real_speed_kmh(van))
                # 下发 PID 控制油门和转向
                RTB.apply_pid_control(van, pid_lon_van, pid_lat_van, target_spd_van, wp_van)

            # ---------------- 轿车控制流 ----------------
            if car and car.is_alive and not RTB.check_vehicle_out_of_bounds(car, carla_map, auto_destroy=True):
                target_spd_car = car_sm.tick(car.get_location(), sim_time, dt)
                wp_car, idx_car = RTB.get_target_waypoint(car.get_location(), dense_traj_car, idx_car,
                                                          get_real_speed_kmh(car))
                RTB.apply_pid_control(car, pid_lon_car, pid_lat_car, target_spd_car, wp_car)

            # ---------------- EGO控制流 ----------------
            if ego and ego.is_alive and not RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                target_spd_ego = ego_sm.tick(ego.get_location(), sim_time, dt)
                wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), dense_traj_ego, idx_ego,
                                                          get_real_speed_kmh(ego))
                RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_spd_ego, wp_ego)

                # 🚀 【新增功能】绘制当前 EGO 的动态预瞄点与绿色牵引线
                RTB.draw_lookahead_point(world, ego.get_location(), wp_ego, color=carla.Color(0, 255, 0))

            # ---------------- 行人控制流 ----------------
            if walker and walker.is_alive:
                # 行人状态机输出其当前应该保持的速度
                current_ped_spd = ped_sm.tick(walker.get_location(), sim_time, dt)
                # 下放控制权限给行人控制器
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=current_ped_spd)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体
        if 'world' in locals():
            RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()