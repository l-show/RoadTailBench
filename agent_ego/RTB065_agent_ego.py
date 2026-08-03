# -*- coding: utf-8 -*-
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
# 原始轨迹数据硬编码
# ==========================================
RAW_BIKE_TRAJ = """Location_x	Location_y	Rotation_yaw
26.419	-3.656	-178.115
26.299	-3.66	-178.115
25.796	-3.68	-177.637
24.894	-3.717	-177.637
23.602	-3.77	-178.096
22.367	-3.807	-178.413
19.409	-3.889	-178.413
15.525	-3.953	-179.795
11.67	-3.946	179.715
7.841	-3.927	179.715
3.971	-3.924	-179.865
0.12	-3.933	-179.865
-3.697	-3.939	-179.935
-7.423	-3.919	179.282
-11.29	-3.892	-179.218
-15.151	-3.963	-178.924
-18.983	-4.02	-179.344
-22.777	-4.049	-179.834
-26.598	-4.058	179.956
-30.442	-4.041	179.655
-34.334	-4.027	179.795
-38.166	-4.013	179.795
-42.056	-4.003	179.865
-45.746	-4.002	-179.855
-49.6	-4.011	-179.855
-53.421	-4.024	-179.715
-57.295	-4.05	-179.575
-58.435	-4.058	-179.575"""

RAW_EGO_TRAJ = """Location_x	Location_y	Rotation_yaw
80.101	-1.265	-179.361
74.436	-1.247	179.691
68.023	-1.25	-179.189
61.592	-1.33	-179.679
55.406	-1.354	-179.819
49.891	-1.372	-179.819
45.226	-1.387	-179.819
41.48	-1.398	-179.819
37.597	-1.406	-179.959
33.763	-1.408	-179.959
29.981	-1.411	-179.959
26.161	-1.414	-179.959
22.253	-1.417	-179.959
19.386	-1.419	-179.959
16.804	-1.42	-179.959
14.252	-1.422	-179.959
12.857	-1.423	-179.959
11.561	-1.424	-179.959
10.277	-1.425	-179.959
9.028	-1.442	-177.375
7.755	-1.58	-169.189
6.532	-1.918	-159.39
5.413	-2.475	-146.59
4.419	-3.288	-132.447
3.589	-4.267	-130.077
2.848	-5.28	-116.045
2.467	-6.487	-100.544
2.348	-7.785	-91.201
2.322	-9.038	-91.201
2.277	-11.181	-91.201
2.196	-15.042	-91.201
2.125	-18.94	-90.366
2.157	-22.613	-89.027
2.249	-26.482	-88.614
2.442	-34.481	-88.614
2.683	-44.88	-89.306
2.571	-54.79	-91.407
2.319	-65.058	-91.407
2.072	-75.135	-91.407
1.865	-83.549	-91.407
1.686	-90.859	-91.407
1.529	-97.227	-91.407
1.374	-103.566	-91.407
0.636	-109.882	-104.445
-2.006	-115.741	-122.853
-6.47	-120.206	-143.547
-10.457	-122.658	-151.678"""

RAW_CAR3_TRAJ = """Location_x	Location_y	Rotation_yaw
1.751	74.405	-89.904
1.749	70.456	-90.114
1.738	66.593	-90.254
1.721	62.939	-90.254
1.704	59.082	-90.254
1.693	55.206	-90.114
1.684	51.448	-90.184
1.672	47.616	-90.184
1.659	43.716	-90.184
1.647	39.782	-90.184
1.636	36.061	-90.044
1.646	32.188	-89.834
1.658	28.262	-89.834
1.669	24.495	-89.834
1.678	21.334	-89.834
1.685	18.748	-89.834
1.693	16.153	-89.834
1.7	13.581	-89.834
1.755	11.027	-85.942
2.144	8.54	-74.017
3.246	6.184	-53.878
5.056	4.589	-31.913
7.396	3.533	-14.505
9.847	2.921	-13.993
12.414	2.51	-3.629
14.967	2.486	0.573
17.698	2.513	0.573
22.46	2.561	0.573
27.585	2.612	0.573
32.984	2.666	0.573
39.337	2.73	0.573
45.678	2.793	0.573
52.006	2.85	-0.084
58.466	2.84	-0.084
64.744	2.806	-0.364
71.056	2.766	-0.364
77.434	2.725	-0.364
83.963	2.684	-0.364
90.42	2.642	-0.364
96.867	2.609	0.279
103.006	2.849	5.77
109.15	4.622	28.9"""

RAW_PED_TRAJ = """Location_x	Location_y	Rotation_yaw
12.337	-10.271	90.67
12.331	-9.973	91.666
12.312	-9.453	92.092
12.294	-8.938	92.092
12.275	-8.426	92.092
12.257	-7.924	92.092
12.238	-7.404	92.092
12.219	-6.9	92.092
12.201	-6.377	91.952
12.184	-5.877	91.952
12.167	-5.372	91.952
12.149	-4.85	91.952
12.133	-4.341	91.602
12.128	-3.821	90.3
12.125	-3.328	90.3
12.122	-2.817	90.3
12.12	-2.301	90.3
12.117	-1.785	90.3
12.114	-1.265	90.3
12.112	-0.764	90.3
12.109	-0.246	90.3
12.106	0.276	90.3
12.104	0.781	90.3
12.111	1.304	87.626
12.133	1.793	87.217
12.159	2.3	86.867
12.19	2.812	86.032
12.228	3.324	85.402
12.276	3.821	83.93
12.34	4.318	81.941
12.414	4.831	81.369
12.497	5.344	79.311
12.658	5.824	57.509
12.94	6.244	56.007
13.32	6.584	25.496
13.774	6.776	14.824
14.274	6.875	9.203
14.786	6.951	5.589
15.292	6.988	2.901
15.808	7.014	2.901
16.325	7.041	2.901
17.037	7.077	2.901
18.281	7.14	2.901
19.563	7.205	2.901
20.839	7.26	1.69
22.131	7.295	1.48
23.413	7.328	1.48
24.681	7.361	1.48
25.775	7.389	1.48"""

def parse_and_setup_traj(raw_text, interpolate_interval=0.5):
    pts = []
    lines = raw_text.strip().split('\n')[1:]
    for line in lines:
        parts = line.split()
        if len(parts) >= 3:
            pts.append((float(parts[0]), float(parts[1]), 0.0, float(parts[2])))
    cleaned = RTB.clean_trajectory(pts, min_dist=0.2)
    start_pt = cleaned[0]
    traj = RTB.interpolate_trajectory(cleaned, interval=interpolate_interval)
    return start_pt, traj

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        bp_lib = world.get_blueprint_library()
        dt = 0.05

        # 1. 环境初始化
        RTB.enable_synchronous_mode(world, dt=dt)
        weather_params = {
            'cloudiness': 40.0, 'precipitation': 100.0, 'precipitation_deposits': 100.0,
            'wind_intensity': 100.0, 'sun_azimuth_angle': 0.0, 'sun_altitude_angle': 10.0,
            'fog_density': 10.0, 'fog_distance': 0.75, 'fog_falloff': 0.1,
            'wetness': 100.0, 'scattering_intensity': 0.0, 'mie_scattering_scale': 0.21,
            'rayleigh_scattering_scale': 0.07, 'dust_storm': 0.0
        }
        RTB.set_static_weather(world, **weather_params)

        # 2. 轨迹数据解析
        bike_start, bike_traj = parse_and_setup_traj(RAW_BIKE_TRAJ)
        ego_start, ego_traj = parse_and_setup_traj(RAW_EGO_TRAJ)
        car3_start, car3_traj = parse_and_setup_traj(RAW_CAR3_TRAJ)
        ped_start, ped_traj = parse_and_setup_traj(RAW_PED_TRAJ, interpolate_interval=0.2)

        # 3. 实体生成
        # --- 自行车 ---
        bike_bp = bp_lib.find('vehicle.diamondback.century')
        bike_tf = carla.Transform(carla.Location(x=bike_start[0], y=bike_start[1], z=0.5),
                                  carla.Rotation(yaw=bike_start[3]))
        bike = world.try_spawn_actor(bike_bp, bike_tf)
        if bike:
            actor_list.append(bike)
            RTB.set_vehicle_initial_speed(bike, 35.0)
            bike_pid_lon = RTB.PIDLongitudinalController(preset='motorcycle')
            bike_pid_lat = RTB.PIDLateralController(preset='motorcycle')
            bike_idx = 0

        # --- Ego 小轿车 ---
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        if ego:
            ego_idx = 0

        # --- 背景小轿车 ---
        car3 = RTB.spawn_vehicle(world, 'vehicle.citroen.c3', x=car3_start[0], y=car3_start[1], yaw=car3_start[3])
        if car3:
            actor_list.append(car3)
            RTB.set_vehicle_initial_speed(car3, 50.0)
            car3_pid_lon = RTB.PIDLongitudinalController(preset='default_car')
            car3_pid_lat = RTB.PIDLateralController(preset='default_car')
            car3_idx = 0

        # --- 行人 ---
        ped_bps = bp_lib.filter('walker.pedestrian.*')
        child_bps = [bp for bp in ped_bps if
                     bp.has_attribute('age') and bp.get_attribute('age').as_str().lower() == 'child']
        child_bp = random.choice(child_bps) if child_bps else ped_bps[0]
        child_bp.set_attribute('is_invincible', 'False')

        spawn_tf = carla.Transform(carla.Location(x=ped_start[0], y=ped_start[1], z=0.5),
                                   carla.Rotation(yaw=ped_start[3]))
        walker = world.try_spawn_actor(child_bp, spawn_tf)

        if walker:
            actor_list.append(walker)
            phone_bp = bp_lib.find('static.prop.mesh')
            phone_bp.set_attribute('mesh_path',
                                   "StaticMesh'/Game/Carla/Static/Dynamic/PedestrianProps/SM_Mobile.SM_Mobile'")
            phone_tf = carla.Transform(carla.Location(x=0.2, y=0.1, z=1.0))
            phone = world.try_spawn_actor(phone_bp, phone_tf, attach_to=walker)
            if phone: actor_list.append(phone)

            ped_ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=ped_traj)
            ped_sm = RTB.MultiStageBehaviorMachine(initial_speed=0.0)
            ped_sm.add_stage('time', target_speed=5.0, trigger_val=0.1, accel=100.0)
            ped_sm.add_stage('time', target_speed=2.5, trigger_val=3.8, accel=100.0)

        # ==========================================
        # ⚠️核心特效重构: 风力物理碎屑 (纸片)
        # ==========================================
        # 1. 起点：纸片生成的初始位置 (z=0.846)
        spawn_pt = carla.Location(x=12.774, y=6.572, z=0.846)

        # 2. 终点目标：这里的 Z=-10.0 是故意填的极深地下坐标！
        # 为什么？因为标准函数库 `PhysicalDebrisManager` 内有一行坑人的代码：
        # `if loc.z <= target_point.z + 0.3: item['settled'] = True`
        # 意思是如果纸片落到终点Z轴附近，就会被打上 settled 标记，风力计算将永远跳过。
        # 所以我们传入 -10.0，骗过底层函数，让它永远不觉得纸片“已经掉到终点了”，从而确保持续施加风力。
        target_pt = carla.Location(x=6.316, y=-8.927, z=-10.0)

        mesh_path = "StaticMesh'/Game/Carla/Static/Dynamic/Trash/SM_CreasedBox03.SM_CreasedBox03'"

        debris_mgr = RTB.PhysicalDebrisManager(
            world, bp_lib,
            spawn_point=spawn_pt,
            target_point=target_pt,
            mesh_path=mesh_path,
            num_debris=5,
            mass=0.2,  # 质量0.05kg，其承受的重力为 m*g = 0.05*9.8 = 0.49N
            scale=0.25,
            wind_strength=0.7,  # 【水平风力】大幅加大水平风推力，确保能吹向终点
            lift_force=0.35  # 【垂直升力】必须大于0.49N的重力，纸片才能被托举在空中，而不是坠落！
        )

        # 生成模型（此时系统会默认开启物理引擎，它在前两秒会自由落体掉到地上）
        debris_mgr.spawn_debris()
        debris_started = False  # 狂风状态锁

        # 4. 仿真主循环
        sim_time = 0.0
        print("\n🚀 场景已加载完毕，仿真开始！(按 Ctrl+C 终止)")

        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            for v in [ego, car3]:
                if v and v.is_alive:
                    RTB.check_vehicle_out_of_bounds(v, carla_map, auto_destroy=True)

            if bike and bike.is_alive:
                bike_wp, bike_idx = RTB.get_target_waypoint(bike.get_location(), bike_traj, bike_idx, speed_kmh=35.0)
                RTB.apply_pid_control(bike, bike_pid_lon, bike_pid_lat, 35.0, bike_wp)

            if car3 and car3.is_alive:
                car3_wp, car3_idx = RTB.get_target_waypoint(car3.get_location(), car3_traj, car3_idx, speed_kmh=50.0)
                RTB.apply_pid_control(car3, car3_pid_lon, car3_pid_lat, 50.0, car3_wp)

            if walker and walker.is_alive:
                ped_speed = ped_sm.tick(walker.get_location(), sim_time, dt)
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=ped_speed)

            # ---------------- F. 物理碎屑(纸片)风力更新 ----------------
            # 需求：前4秒自然下落，3秒后被狂风吹飞
            if sim_time > 4.0:
                if not debris_started:
                    # 【核心机制】由于纸片在地上躺了2秒，引擎静摩擦力极大。
                    # 如果只用普通的推力很难推飞，必须施加一次随机的爆炸冲量 (Impulse) 把它“踹”离地面。
                    for item in debris_mgr.debris_data:
                        actor = item['actor']
                        if actor.is_alive:
                            # 根据目标方向(dir_x, dir_y)，添加随机倍数，以及随机的起飞高度(iz)
                            ix = debris_mgr.dir_x * random.uniform(0.5, 1.2)
                            iy = debris_mgr.dir_y * random.uniform(0.5, 1.2)
                            iz = random.uniform(0.8, 1.5)  # 强烈的向上弹射感

                            actor.add_impulse(carla.Vector3D(ix, iy, iz))

                            # 施加一个极大的随机旋转扭矩，让纸片在空中疯狂打转
                            actor.add_torque(carla.Vector3D(
                                random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)
                            ))

                    debris_started = True
                    print("[场景特效] 🌬️ 2秒等待结束，狂风瞬间袭来，地上的纸片被卷起飞舞！")

                # 【持续风力】离开地面后，交给底层的 tick 持续施加水平风推力(1.5)和托举升力(1.2)
                debris_mgr.tick(sim_time)

            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        try:
            debris_mgr.cleanup()
        except:
            pass
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()