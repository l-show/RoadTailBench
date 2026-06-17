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
# 轨迹数据硬编码 (原始字符串格式)
# ==========================================
RAW_TRAJ_TRUCK = """
0.995	-176.825	92.61
0.995	-176.825	92.61
0.995	-176.825	92.61
0.873	-174.996	93.837
0.387	-164.842	91.12
0.292	-154.843	89.912
0.374	-144.511	89.282
0.543	-134.347	88.787
0.758	-124.184	88.787
0.967	-114.187	88.857
1.059	-104.021	90.067
1.047	-93.854	90.067
1.062	-83.681	89.715
1.114	-73.513	89.645
1.187	-63.513	89.502
1.285	-53.347	89.432
1.405	-43.014	89.219
1.545	-32.681	89.219
1.682	-22.682	89.219
1.822	-12.35	89.219
1.964	-2.018	89.219
2.076	8.147	89.429
2.1	18.48	90.064
2.093	28.814	89.924
2.107	39.147	89.854
2.186	49.147	89.434
2.327	59.15	89.079
2.389	62.982	89.079
2.389	62.982	89.079
2.389	62.982	89.079
"""

RAW_TRAJ_SEDAN = """
6.263	-6.034	-89.997
6.263	-6.034	-89.997
6.263	-6.034	-89.306
6.423	-12.364	-88.663
6.697	-22.691	-89.513
6.675	-32.69	-90.788
6.402	-43.02	-91.851
6.068	-53.348	-91.851
5.734	-63.676	-91.851
5.471	-74.006	-90.125
5.685	-84.003	-88.811
5.797	-94.002	-89.803
5.802	-104.335	-90.585
5.678	-114.668	-90.797
5.566	-124.667	-90.375
5.49	-135	-90.445
5.412	-144.999	-90.445
5.332	-155.332	-90.445
5.254	-165.332	-90.445
5.174	-175.665	-90.445
4.789	-185.824	-94.857
3.133	-195.675	-105.221
-0.566	-205.12	-119.389
-6.369	-213.452	-129.248
-13.413	-220.517	-142.228
-22.071	-225.441	-159.673
-32.015	-227.425	-172.335
-42.259	-228.765	-173.845
-52.402	-229.435	-177.978
-62.399	-229.788	-177.978
-72.728	-230.095	-178.543
-83.06	-230.262	-179.613
-93.061	-230.296	179.959
-103.06	-230.353	-178.869
-113.058	-230.563	-178.799
-121.722	-230.736	-178.869
-121.722	-230.736	-178.869
-121.722	-230.736	-178.869
"""

RAW_TRAJ_EGO = """
10.245	-12.393	-90.887
10.245	-12.393	-90.887
10.245	-12.393	-91.528
10.245	-12.393	-90.885
10.218	-14.892	-90.602
10.169	-19.557	-90.957
10.03	-29.057	-91.027
9.829	-39.056	-91.17
9.617	-49.22	-91.31
9.602	-49.886	-91.31
9.522	-53.385	-91.31
9.522	-53.385	-91.31
9.522	-53.385	-91.31
9.522	-53.385	-91.31
9.522	-53.385	-91.31
9.522	-53.385	-91.31
9.522	-53.385	-91.522
9.417	-54.547	-95.882
8.397	-58.739	-109.165
7.459	-61.054	-112.537
6.099	-64.458	-106.047
4.99	-68.816	-99.925
4.646	-74.448	-88.173
5.082	-80.263	-84.737
5.489	-87.082	-91.073
4.944	-95.543	-105.364
-0.216	-104.389	-132.579
-5.571	-108.161	-163.932
-10.902	-109.509	-168.729
-16.207	-110.007	-176.648
-26.37	-110.11	179.019
-36.729	-109.921	178.949
-47.061	-109.732	178.949
-57.216	-109.312	176.198
-67.526	-108.637	174.932
-77.416	-107.215	166.893
-86.979	-103.446	151.063
-95.148	-97.216	132.794
-101.3	-88.942	119.499
-104.938	-79.492	101.552
-106.435	-69.275	95.028
-106.797	-58.948	90.867
-106.928	-48.612	90.724
-107.031	-38.446	90.441
-107.11	-28.114	90.441
-107.175	-18.114	90.301
-107.149	-7.783	89.739
-107.051	2.552	89.101
-106.894	12.552	89.101
-106.737	22.551	89.101
-106.665	32.55	90.603
-106.798	42.881	90.743
-106.952	53.213	90.953
-107.007	56.546	90.953
-107.007	56.546	90.953
-107.007	56.546	90.953
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

        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        RTB.set_static_weather(
            world, cloudiness=15.0, precipitation=5.0, precipitation_deposits=15.0,
            wind_intensity=30.0, sun_azimuth_angle=85.0, sun_altitude_angle=2.0,
            fog_density=2.0, fog_distance=0.0, fog_falloff=0.0, wetness=15.0,
            scattering_intensity=0.5, mie_scattering_scale=0.05, rayleigh_scattering_scale=0.13, dust_storm=0.0
        )
        print("[场景配置] 天气系统已设置")

        # 2. 轨迹数据硬编码与清洗
        # ==========================================
        # 调用标准库一键解析字符串并清洗冗余点
        traj_truck = RTB.parse_string_trajectory(RAW_TRAJ_TRUCK, min_dist=0.5)
        traj_sedan = RTB.parse_string_trajectory(RAW_TRAJ_SEDAN, min_dist=0.5)
        traj_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO, min_dist=0.5)

        # 🚀 绘制出所有车辆的预设灰色轨迹线
        RTB.draw_preset_trajectory(world, traj_truck, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_sedan, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(0, 0, 255))  # Ego标为蓝色作区分

        # 3. 车辆、行人、模型实体安全生成
        # ==========================================
        # 生成车辆 1：大卡车 (使用 CARLA 官方自带消防车代替自定义动画蓝图以确保兼容性)
        truck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck',
                                  x=traj_truck[0][0], y=traj_truck[0][1], yaw=traj_truck[0][2],
                                  role_name="truck", z_offset=1.5)

        # 生成车辆 2：小轿车
        sedan = RTB.spawn_vehicle(world, 'vehicle.dodge.charger_2020',
                                  x=traj_sedan[0][0], y=traj_sedan[0][1], yaw=traj_sedan[0][2],
                                  role_name="sedan")

        # 生成车辆 3：Ego
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                x=traj_ego[0][0], y=traj_ego[0][1], yaw=traj_ego[0][2],
                                role_name="ego")

        actor_list.extend([truck, sedan, ego])

        # 生成行人
        walker_bp = random.choice(bp_lib.filter("walker.pedestrian.*"))
        # 初始点选为徘徊点之一
        spawn_loc = carla.Location(x=19.721, y=-91.630, z=1.0)
        walker = world.try_spawn_actor(walker_bp, carla.Transform(spawn_loc))
        if walker: actor_list.append(walker)

        # 4. 车辆PID控制器挂载
        # ==========================================
        # 卡车 (重量大，使用 truck 预设)
        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck', dt=dt)
        pid_lat_truck = RTB.PIDLateralController(preset='truck', dt=dt)

        # 轿车与Ego (使用 default_car 预设)
        pid_lon_sedan = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_sedan = RTB.PIDLateralController(preset='default_car', dt=dt)
        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_ego = RTB.PIDLateralController(preset='default_car', dt=dt)

        idx_truck, idx_sedan, idx_ego = 0, 0, 0

        # 行人控制器挂载 (徘徊模式)
        ROAM_POINTS = [(19.721, -91.630), (12.899, -97.265), (19.006, -98.451)]
        ped_ctrl = RTB.PedestrianController(walker, mode='roam', target_list=ROAM_POINTS, default_speed=1.5)

        # 5. 剧本状态机编排
        # ==========================================
        # 卡车剧本：初始速度 40km/h，过1s加速到70km/h
        sm_truck = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        sm_truck.add_stage(trigger_type='time', trigger_val=1.0, target_speed=70.0, accel=15.0)

        # 轿车剧本：初始速度 60km/h，过2s减速到30km/h，再过2s加速到70km/h
        sm_sedan = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_sedan.add_stage(trigger_type='time', trigger_val=2.0, target_speed=30.0, accel=20.0)  # 急减速
        sm_sedan.add_stage(trigger_type='time', trigger_val=2.0, target_speed=70.0, accel=10.0)  # 缓加速

        # Ego剧本：初始速度 40km/h，在 y=-53 时刹停，静止3s后恢复30km/h，过3s后加速到60km/h
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        # 注意: 轨迹中 y 是逐渐变小的，因此判断条件为 y_less
        sm_ego.add_stage(trigger_type='y_less', trigger_val=-53.0, target_speed=0.0, accel=30.0)  # 抵达坐标急刹停
        sm_ego.add_stage(trigger_type='time', trigger_val=3.0, target_speed=30.0, accel=10.0)  # 等待3s后起步
        sm_ego.add_stage(trigger_type='time', trigger_val=3.0, target_speed=60.0, accel=15.0)  # 再等3s后加速

        # 6. 预热与初始状态注入
        # ==========================================
        RTB.set_vehicle_initial_speed(truck, 40.0, yaw_deg=traj_truck[0][2])
        RTB.set_vehicle_initial_speed(sedan, 60.0, yaw_deg=traj_sedan[0][2])
        RTB.set_vehicle_initial_speed(ego, 40.0, yaw_deg=traj_ego[0][2])

        # 7. 物理特效管理器预备
        # ==========================================
        start_pt = carla.Location(x=13.16, y=-92.77, z=0.56)

        # 【核心修复】：伪造一个 Z=-10.0 的终点。
        # 这样底层库计算风向时，XY依然是对准B点的，但Z轴判定永远不会在起步时触发。
        target_pt_fake_z = carla.Location(x=-1.76, y=-104.28, z=-10.0)

        # 物理参数校准：
        # 质量 0.01kg，重力为 0.098N。
        # 升力(lift_force)设为 0.5 已经相当于抵抗了5倍重力，纸片会如同被龙卷风卷起一样剧烈升空。
        debris_mgr = RTB.PhysicalDebrisManager(
            world, bp_lib, spawn_point=start_pt, target_point=target_pt_fake_z,
            mesh_path="StaticMesh'/Game/Carla/Static/Dynamic/Trash/SM_CreasedBox01.SM_CreasedBox01'",
            num_debris=30, mass=0.01, scale=0.3,
            wind_strength=0.8,  # 水平吹向B点的强风
            lift_force=0.5,  # 远大于重力的上升狂风
            flutter_freq=15.0  # 加快摇摆频率，显得风极具撕裂感
        )
        debris_spawned = False

        # 8. 仿真主循环（帧率同步与环境清理守护）
        # ==========================================
        sim_time = 0.0
        print("[场景运行] 进入主循环...")

        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 物理特效：第 3 秒触发狂风，第 6 秒风停飘落 ----------------
            if sim_time >= 8.0 and not debris_spawned:
                debris_mgr.spawn_debris()
                debris_spawned = True

            if debris_spawned:
                # 【剧本补充】：被狂风卷上天 3 秒后，风力突然减弱，纸片落回地面
                if sim_time > 10.0:
                    # 强行将升力改为负数（比重力稍大的下压力）或 0，纸箱失去升力就会受重力影响落下
                    debris_mgr.upward_lift_force = -0.05
                    debris_mgr.base_wind_strength = 0.1  # 水平风变为微风

                debris_mgr.tick(sim_time)

            # ---------------- 车辆与行人控制逻辑 ----------------

            # 1. 推进状态机，获取当前帧应该达到的平滑目标速度
            truck_spd = sm_truck.tick(truck.get_location(), sim_time, dt) if truck.is_alive else 0
            sedan_spd = sm_sedan.tick(sedan.get_location(), sim_time, dt) if sedan.is_alive else 0
            ego_spd = sm_ego.tick(ego.get_location(), sim_time, dt) if ego.is_alive else 0

            # 2. 车辆寻迹与 PID 控制
            if truck and truck.is_alive:
                truck_loc = truck.get_location()
                target_wp, idx_truck = RTB.get_target_waypoint(truck_loc, traj_truck, idx_truck, truck_spd)
                if target_wp:
                    RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, truck_spd, target_wp)

            if sedan and sedan.is_alive:
                sedan_loc = sedan.get_location()
                target_wp, idx_sedan = RTB.get_target_waypoint(sedan_loc, traj_sedan, idx_sedan, sedan_spd)
                if target_wp:
                    RTB.apply_pid_control(sedan, pid_lon_sedan, pid_lat_sedan, sedan_spd, target_wp)

            if ego and ego.is_alive:
                ego_loc = ego.get_location()
                target_wp, idx_ego = RTB.get_target_waypoint(ego_loc, traj_ego, idx_ego, ego_spd)
                if target_wp:
                    RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, ego_spd, target_wp)
                    # 绘制出 Ego 当前的预瞄点和牵引线
                    RTB.draw_lookahead_point(world, ego_loc, target_wp)

            # 3. 行人控制
            ped_ctrl.run_step(dt, sim_time)

            # ---------------- 越界守护机制 ----------------
            RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(sedan, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体
        RTB.disable_synchronous_mode(world)
        if 'debris_mgr' in locals():
            debris_mgr.cleanup()  # 清理纸箱
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()