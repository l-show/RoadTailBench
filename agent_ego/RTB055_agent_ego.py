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

# ==========================================
# 原始轨迹数据硬编码 (已去除表头，直接保留纯数据)
# ==========================================
TRUCK_TRAJ_STR = """
31.425	-24.25	-144.908
31.425	-24.25	-144.908
31.425	-24.25	-144.908
31.425	-24.25	-144.908
28.425	-26.358	-144.908
24.336	-29.381	-140.658
20.657	-33.001	-130.974
17.528	-37.008	-125.222
15.029	-41.522	-115.134
12.999	-46.272	-110.545
11.404	-51.098	-106.145
10.303	-56.144	-98.169
9.87	-61.287	-91.797
9.838	-63.786	-90.425
9.838	-63.786	-90.425
9.838	-63.786	-90.425
9.838	-63.786	-90.425
"""

EGO_TRAJ_STR = """
73.084	-14.242	-168.179
73.084	-14.242	-168.179
73.084	-14.242	-168.179
69.088	-15.079	-168.109
64.033	-16.144	-168.109
59.059	-17.195	-168.039
54.005	-18.265	-168.039
49.114	-19.302	-168.039
44.131	-20.361	-167.607
39.12	-21.613	-163.559
34.219	-23.246	-158.708
29.641	-25.45	-149.381
25.408	-28.409	-140.863
21.663	-31.967	-133.028
18.615	-35.921	-123.194
15.979	-40.362	-117.691
13.858	-44.98	-111.628
12.052	-49.819	-114.161
9.653	-54.205	-120.357
7.181	-58.65	-115.028
5.931	-63.376	-94.396
5.931	-63.376	-93.089
5.931	-63.376	-93.089
5.931	-63.376	-93.089
5.931	-63.376	-93.089
5.931	-63.376	-93.089
5.881	-64.375	-92.879
5.711	-69.373	-91.697
5.559	-74.539	-91.484
5.501	-79.624	-90.478
5.462	-84.623	-90.268
5.444	-89.79	-90.198
5.426	-94.958	-90.198
5.409	-100.041	-90.198
5.395	-105.209	-89.915
5.403	-110.377	-89.915
5.409	-115.461	-89.985
5.388	-120.627	-90.41
5.336	-125.794	-90.978
5.221	-130.876	-91.398
5.091	-136.043	-91.468
4.959	-141.207	-91.538
4.722	-146.285	-94.562
4.19	-151.424	-96.987
3.31	-156.513	-101.343
2.304	-161.498	-101.986
2.079	-162.558	-101.986
2.079	-162.558	-101.986
"""

CAR3_TRAJ_STR = """
59.872	-13.861	-171.499
59.872	-13.861	-171.499
59.502	-13.916	-171.499
56.961	-14.333	-170.487
54.458	-14.755	-170.134
51.916	-15.211	-169.781
48.471	-15.832	-169.781
44.736	-16.589	-166.681
40.964	-17.482	-166.681
37.334	-18.418	-163.685
33.682	-19.709	-157.654
30.223	-21.311	-152.908
26.905	-23.307	-146.259
23.714	-25.503	-143.803
20.764	-27.916	-136.866
18.188	-30.638	-130.258
15.874	-33.746	-124.011
13.814	-37.027	-120.293
12.005	-40.382	-115.989
10.524	-43.893	-111.409
9.131	-47.443	-111.339
7.982	-51.144	-105.17
7.024	-54.898	-103.61
6.26	-58.697	-98.475
5.815	-62.483	-94.873
5.579	-66.227	-92.648
5.486	-70.101	-90.646
5.486	-73.915	-89.725
5.507	-77.79	-89.582
5.532	-81.667	-90.005
5.52	-85.482	-90.215
5.506	-89.296	-90.215
5.491	-93.182	-90.215
5.41	-96.994	-96.779
4.314	-100.688	-119.531
2.051	-103.75	-130.9
-0.778	-106.391	-141.654
-4.1	-108.369	-155.893
-7.762	-109.364	-173.772
-11.633	-109.503	178.972
-15.508	-109.402	178.472
-19.319	-109.296	177.829
-23.357	-109.133	177.683
-29.727	-108.877	177.968
-36.08	-108.733	179.244
-42.537	-108.668	179.457
-48.998	-108.607	179.457
-55.354	-108.547	179.457
-57.855	-108.523	179.457
-57.855	-108.523	179.457
"""

PED_TRAJ_STR = """
13.13	-77.927	178.687
13.13	-77.927	178.687
13.13	-77.927	178.687
12.923	-77.922	178.687
12.408	-77.909	178.896
11.9	-77.9	178.896
11.384	-77.89	178.896
10.885	-77.88	178.896
10.369	-77.87	178.896
8.115	-77.826	178.896
5.619	-77.778	178.896
3.082	-77.731	179.106
0.583	-77.714	-179.623
-1.999	-77.757	-178.71
-4.579	-77.839	-174.029
-6.938	-78.761	-135.671
-7.95	-81.015	-107.829
-8.722	-83.393	-111.817
-10.69	-84.663	-177.158
-13.14	-84.802	-176.002
-13.14	-84.802	-176.002
-13.14	-84.802	-176.002
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

        # 按照截图要求精准配置天气 (Cloud, Rain, Puddles, Wind, Fog, Light Scattering 等)
        weather = carla.WeatherParameters(
            cloudiness=40.0,
            precipitation=90.0,
            precipitation_deposits=50.0,  # 对应 Puddles 积水
            wind_intensity=65.0,
            sun_azimuth_angle=90.0,
            sun_altitude_angle=4.0,
            fog_density=8.0,
            fog_distance=0.75,
            fog_falloff=0.1000,
            wetness=35.0,
            scattering_intensity=9.0,
            mie_scattering_scale=0.21,
            rayleigh_scattering_scale=0.07,
            dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 长尾天气系统已按截图设定完毕。")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化 (间距插值 0.5m)
        # ==========================================
        # 货车轨迹
        raw_truck = RTB.parse_string_trajectory(TRUCK_TRAJ_STR, min_dist=0.1)
        traj_truck = RTB.interpolate_trajectory(raw_truck, interval=0.5)
        # EGO 轨迹
        raw_ego = RTB.parse_string_trajectory(EGO_TRAJ_STR, min_dist=0.1)
        traj_ego = RTB.interpolate_trajectory(raw_ego, interval=0.5)
        # 轿车3 轨迹
        raw_car3 = RTB.parse_string_trajectory(CAR3_TRAJ_STR, min_dist=0.1)
        traj_car3 = RTB.interpolate_trajectory(raw_car3, interval=0.5)
        # 行人 轨迹
        raw_ped = RTB.parse_string_trajectory(PED_TRAJ_STR, min_dist=0.1)
        traj_ped = RTB.interpolate_trajectory(raw_ped, interval=0.5)

        # ==========================================
        # 3. 车辆、行人实体生成
        # ==========================================
        # 【货车】: vehicle.mercedes.sprinter
        # 注：Carla引擎原生不支持通过蓝图独立缩放Actor体积(Mesh scale)，此处正常生成。
        truck = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter',
                                  x=traj_truck[0][0], y=traj_truck[0][1], yaw=traj_truck[0][2])
        actor_list.append(truck)

        # 【EGO轿车】: vehicle.audi.tt
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        # 【普通轿车】: vehicle.citroen.c3
        car3 = RTB.spawn_vehicle(world, 'vehicle.citroen.c3',
                                 x=traj_car3[0][0], y=traj_car3[0][1], yaw=traj_car3[0][2])
        actor_list.append(car3)

        # 【行人生成】
        ped_bp = random.choice(bp_lib.filter('walker.pedestrian.*'))
        ped_spawn_loc = carla.Location(x=traj_ped[0][0], y=traj_ped[0][1], z=1.5)
        ped_spawn_rot = carla.Rotation(yaw=traj_ped[0][2])
        walker = world.try_spawn_actor(ped_bp, carla.Transform(ped_spawn_loc, ped_spawn_rot))
        actor_list.append(walker)

        # ==========================================
        # 4. 初始化控制器、状态机、灯光与预瞄索引
        # ==========================================
        # 为每辆车建立独立的PID
        pid_lon_truck, pid_lat_truck = RTB.PIDLongitudinalController(preset='truck'), RTB.PIDLateralController(
            preset='truck')
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(preset='default_car'), RTB.PIDLateralController(
            preset='default_car')
        pid_lon_car3, pid_lat_car3 = RTB.PIDLongitudinalController(preset='default_car'), RTB.PIDLateralController(
            preset='default_car')

        idx_truck, idx_ego, idx_car3 = 0, 0, 0  # 各自的滑窗索引

        # EGO 开启行车灯
        if ego:
            pass

        # 【剧本编排】
        # 货车: 初始 40km/h，x<9.9 处停车
        sm_truck = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        sm_truck.add_stage('x_less', trigger_val=9.9, target_speed=0.0, accel=25.0)

        # EGO: 初始 55km/h，y<-63 减到0，等3秒恢复 50km/h

        # Car3: 初始 70km/h 跑到尾
        sm_car3 = RTB.MultiStageBehaviorMachine(initial_speed=70.0)

        # 行人: 初始 0，原地等5秒后以 4.5m/s (跑步) 前进
        ped_ctrl = None
        sm_ped = RTB.MultiStageBehaviorMachine(initial_speed=0.0)
        if walker:
            ped_ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=traj_ped)
            sm_ped.add_stage('time', trigger_val=5.0, target_speed=4.5, accel=100.0)

        # ==========================================
        # 5. 初始速度注入 (消除起步顿挫)
        # ==========================================
        if truck: RTB.set_vehicle_initial_speed(truck, 40.0, yaw_deg=traj_truck[0][2])
        if car3: RTB.set_vehicle_initial_speed(car3, 70.0, yaw_deg=traj_car3[0][2])

        print("[场景系统] 所有实体生成完毕，开始进入仿真控制循环...")

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- A. Ego 车辆控制与守护 ----------------

            # ---------------- B. 货车车辆控制与守护 ----------------
            if truck and truck.is_alive:
                if RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True):
                    truck = None
                else:
                    target_spd = sm_truck.tick(truck.get_location(), sim_time, dt)
                    wp_truck, idx_truck = RTB.get_target_waypoint(truck.get_location(), traj_truck, idx_truck,
                                                                  target_spd)
                    if wp_truck:
                        RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, target_spd, wp_truck)

            # ---------------- C. 普通轿车控制与守护 ----------------
            if car3 and car3.is_alive:
                if RTB.check_vehicle_out_of_bounds(car3, carla_map, auto_destroy=True):
                    car3 = None
                else:
                    target_spd = sm_car3.tick(car3.get_location(), sim_time, dt)
                    wp_car3, idx_car3 = RTB.get_target_waypoint(car3.get_location(), traj_car3, idx_car3, target_spd)
                    if wp_car3:
                        RTB.apply_pid_control(car3, pid_lon_car3, pid_lat_car3, target_spd, wp_car3)

            # ---------------- D. 行人状态控制 ----------------
            if walker and walker.is_alive and ped_ctrl:
                ped_spd = sm_ped.tick(walker.get_location(), sim_time, dt)
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=ped_spd)

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