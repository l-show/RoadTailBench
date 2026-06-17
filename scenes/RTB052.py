import sys
import carla
import time

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

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
        # 引擎配置：开启同步模式，固定时间步长 0.05s (20 FPS)
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按参数截屏要求配置极端恶劣天气
        weather = RTB.build_weather(
            cloudiness=35.0,
            precipitation=100.0,
            precipitation_deposits=60.0,  # 积水 Puddles
            wind_intensity=85.0,
            sun_azimuth_angle=215.0,
            sun_altitude_angle=10.0,
            wetness=50.0,
            rayleigh_scattering_scale=0.06
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已设置 (积水暴雨环境)")

        # ==========================================
        # 2. 轨迹数据硬编码与清洗
        # ==========================================
        raw_traj_npc = """
-65.804	101.082	-31.803
-65.804	101.082	-31.803
-65.804	101.082	-31.803
-64.314	100.164	-31.163
-62.133	98.86	-29.14
-59.839	97.771	-21.354
-57.372	97.008	-13.978
-54.835	96.528	-8.791
-52.276	96.183	-6.765
-49.749	95.906	-6.126
-47.178	95.658	-4.411
-44.641	95.499	-3.202
-42.062	95.355	-3.202
-39.525	95.202	-4.126
-36.956	94.938	-7.327
-34.399	94.565	-8.416
-31.844	94.186	-8.486
-29.333	93.793	-9.55
-26.801	93.285	-13.303
-24.378	92.669	-15.683
-21.968	91.863	-20.756
-19.608	90.816	-26.495
-17.333	89.682	-26.495
-15.108	88.372	-31.252
-13.021	86.857	-38.732
-11.006	85.241	-38.732
-9.136	83.523	-48.445
-7.45	81.621	-48.445
-5.851	79.595	-53.097
-4.324	77.562	-53.097
-2.882	75.47	-56.276
-1.729	73.257	-75.112
-1.494	70.654	-94.443
-1.713	68.081	-91.316
-1.248	65.553	-70.797
-0.266	63.21	-66.547
0.629	60.833	-72.796
1.056	58.294	-87.857
1.151	55.754	-87.857
1.276	52.049	-89.777
1.18	48.237	-92.025
0.908	44.436	-96.056
0.431	40.717	-98.867
-0.234	36.963	-100.178
-0.93	33.151	-101.402
-1.874	29.459	-106.236
-3.088	25.781	-109.938
-4.689	22.258	-118.041
-6.481	18.893	-118.041
-8.205	15.494	-114.916
-9.827	11.975	-114.921
-11.579	8.59	-118.546
-13.676	5.334	-124.289
-15.909	2.246	-128.569
-18.449	-0.672	-136.311
-21.29	-3.21	-141.548
-24.407	-5.507	-147.149
-27.648	-7.388	-152.158
-31.021	-9.169	-152.158
-34.447	-10.979	-152.158
-37.881	-12.775	-153.606
-41.381	-14.277	-161.094
-45.052	-15.336	-167.959
-48.872	-15.987	-171.83
-52.652	-16.481	-174.024
-56.508	-16.864	-174.805
-60.369	-17.182	-175.44
-64.173	-17.443	-176.504
-68.041	-17.679	-176.504
-71.036	-17.842	-176.932
-71.036	-17.842	-176.932
-71.036	-17.842	-176.932
"""

        raw_traj_ego = """
-80.065	-14.081	-0.879
-80.065	-14.081	-0.879
-80.065	-14.081	-0.879
-80.065	-14.081	-0.879
-80.065	-14.081	-0.879
-80.065	-14.081	-0.879
-80.065	-14.081	-0.879
-80.065	-14.081	-0.879
-70.423	-14.184	0.272
-60.281	-13.824	3.569
-50.335	-12.84	7.712
-40.246	-10.648	16.344
-30.907	-6.707	30.672
-22.518	-0.647	41.293
-15.341	6.544	46.515
-8.119	13.922	50.976
-4.623	23.306	82.471
-3.604	33.576	79.821
-2.007	43.781	83.798
-1.449	54.092	90.643
-2.319	64.212	97.922
-3.015	68.827	99.14
-3.155	69.648	100.658
-8.156	78.52	131.48
-15.834	85.392	145.374
-24.916	89.869	162.789
-35.183	92.046	169.889
-45.093	93.425	173.35
-55.353	94.653	170.452
-64.61	98.277	147.878
-71.471	105.855	119.419
-75.503	115.156	104.313
-76.282	125.076	85.526
-73.615	134.971	65.447
-67.131	142.716	42.226
-59.19	149.32	39.534
-51.135	155.792	37.328
-43.017	161.909	36.973
-41.02	163.413	36.973
-41.02	163.413	36.973
-41.02	163.413	36.973
"""
        # 一键解析并清洗去重
        clean_traj_npc = RTB.parse_string_trajectory(raw_traj_npc, min_dist=0.1)
        clean_traj_ego = RTB.parse_string_trajectory(raw_traj_ego, min_dist=0.1)

        # 插值稠密化到 0.5m，保证循迹顺滑
        traj_npc = RTB.interpolate_trajectory(clean_traj_npc, interval=0.5)
        traj_ego = RTB.interpolate_trajectory(clean_traj_ego, interval=0.5)

        # 绘制所有车辆的完整寻路轨迹 (要求项)
        RTB.draw_preset_trajectory(world, traj_npc, color=carla.Color(0, 0, 255), size=0.05)  # 蓝线：NPC
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(255, 0, 0), size=0.05)  # 红线：EGO

        # ==========================================
        # 3. 车辆与特殊物理模型生成
        # ==========================================
        # 第一辆车 (NPC)
        npc_vehicle = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala',
                                        x=traj_npc[0][0], y=traj_npc[0][1], yaw=traj_npc[0][2], role_name="npc")
        if npc_vehicle: actor_list.append(npc_vehicle)

        # 第二辆车 (EGO)
        ego_vehicle = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                        x=traj_ego[0][0], y=traj_ego[0][1], yaw=traj_ego[0][2], role_name="ego")
        if ego_vehicle: actor_list.append(ego_vehicle)

        # 物理特效生成：生成积水低摩擦力打滑区
        friction_zone = RTB.spawn_friction_region(
            world, bp_lib,
            center_loc=carla.Location(x=-12.463, y=83.618, z=1.474),
            friction=0.1,  # 极低的抓地力
            extent=(20.0, 20.0, 2.0),
            draw_debug=False, debug_life=0.0
        )
        if friction_zone: actor_list.append(friction_zone)

        # ==========================================
        # 4. PID 控制器挂载、灯光与初速注入
        # ==========================================
        pid_lon_npc = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_npc = RTB.PIDLateralController(preset='default_car')

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')

        ego_lights = None
        if ego_vehicle:
            # 挂载灯光管理器并开启行车灯
            ego_lights = RTB.VehicleLightManager(ego_vehicle)
            ego_lights.set_static_lights(low_beam=True, high_beam=False)

        # 赋予瞬间物理初速度防打滑
        if npc_vehicle: RTB.set_vehicle_initial_speed(npc_vehicle, target_speed_kmh=60.0)
        if ego_vehicle: RTB.set_vehicle_initial_speed(ego_vehicle, target_speed_kmh=60.0)

        # ==========================================
        # 5. 剧本状态机编排
        # ==========================================
        # EGO车辆剧本：初始60 -> 过Y=1时减速到20 -> 等待8秒恢复60
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        # 阶段1：判断坐标 y > 1 时触发，以 15 的减速度急刹到 20km/h
        ego_sm.add_stage(trigger_type='y_greater', trigger_val=1.0, target_speed=20.0, accel=15.0)
        # 阶段2：进入阶段1后，持续 8 秒时间，然后以 10 的加速度提速回 60km/h
        ego_sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=60.0, accel=20.0)

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        current_idx_npc = 0
        current_idx_ego = 0

        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 车辆出界守护销毁逻辑 ----------------
            if npc_vehicle and npc_vehicle.is_alive:
                if RTB.check_vehicle_out_of_bounds(npc_vehicle, carla_map, auto_destroy=True):
                    npc_vehicle = None  # 失去引用，不再被控制

            if ego_vehicle and ego_vehicle.is_alive:
                if RTB.check_vehicle_out_of_bounds(ego_vehicle, carla_map, auto_destroy=True):
                    ego_vehicle = None

            # ---------------- NPC 循迹控制逻辑 ----------------
            if npc_vehicle and npc_vehicle.is_alive:
                target_wp_npc, current_idx_npc = RTB.get_target_waypoint(
                    npc_vehicle.get_location(), traj_npc, current_idx_npc, speed_kmh=60.0
                )
                if target_wp_npc:
                    RTB.apply_pid_control(npc_vehicle, pid_lon_npc, pid_lat_npc, 60.0, target_wp_npc)

            # ---------------- EGO 剧本循迹控制与可视化逻辑 ----------------
            if ego_vehicle and ego_vehicle.is_alive:
                # 联动刹车/转向时自动亮起尾灯 (真实物理沉浸感)
                ego_lights.auto_update_from_control()

                # 更新状态机获取当前帧的目标速度
                current_ego_speed = ego_sm.tick(ego_vehicle.get_location(), sim_time, dt)

                target_wp_ego, current_idx_ego = RTB.get_target_waypoint(
                    ego_vehicle.get_location(), traj_ego, current_idx_ego, speed_kmh=current_ego_speed
                )
                if target_wp_ego:
                    # 绘制出 ego 的当前预瞄点及牵引线 (要求项)
                    RTB.draw_lookahead_point(world, ego_vehicle.get_location(), target_wp_ego)

                    RTB.apply_pid_control(ego_vehicle, pid_lon_ego, pid_lat_ego, current_ego_speed, target_wp_ego)

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