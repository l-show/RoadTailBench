# -*- coding: utf-8 -*-
"""
RoadTailBench 长尾场景构建：复杂多车交互与行人过街
Carla 版本: 0.9.15
依赖库: RoadTailBenchInitV9
"""

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
# 原始轨迹数据硬编码 (已去除表头)
# ==========================================
RAW_TRAJ_BUS = """
-9.033	-76.838	87.212
-8.697	-66.695	89.319
-8.692	-56.702	90.264
-8.678	-46.584	89.229
-8.518	-36.416	89.089
-8.37	-26.26	89.369
-8.29	-15.971	89.579
-8.24	-5.744	89.789
-8.161	4.466	89.136
-8.035	14.644	89.346
-8.03	15.101	89.346
-8.028	15.307	89.136
"""

RAW_TRAJ_EGO = """
-10.453	-137.365	84.618
-10.453	-127.365	84.618
-10.453	-117.365	84.618
-10.453	-107.365	84.618
-10.453	-97.365	84.618
-10.244	-94.939	85.064
-9.495	-86.001	86.546
-9.165	-77.164	88.618
-8.991	-68.101	89.279
-8.904	-59.184	89.698
-8.899	-50.356	89.66
-8.809	-41.479	89.161
-8.648	-32.755	88.881
-8.464	-23.845	88.811
-8.353	-18.48	88.811
-8.344	-18.073	88.811
-8.292	-15.523	88.811
-8.285	-15.205	88.811
-8.281	-15	88.811
-8.27	-14.481	88.811
-8.26	-13.98	88.811
-8.232	-13.482	84.116
-8.145	-12.982	75.701
-7.988	-12.495	69.799
-7.791	-12.011	65.684
-7.582	-11.562	64.323
-7.351	-11.107	62.613
-7.115	-10.653	62.473
-6.885	-10.211	62.473
-6.656	-9.748	64.369
-6.445	-9.302	64.789
-6.227	-8.833	65.279
-6.013	-8.365	65.839
-5.804	-7.897	66.049
-5.598	-7.424	67.621
-5.424	-6.958	71.945
-5.286	-6.475	76.991
-5.187	-5.959	81.05
-5.121	-5.462	84.077
-5.085	-4.958	87.486
-5.062	-4.438	87.486
-5.035	-3.807	87.486
-4.98	-2.564	87.486
-4.83	0.843	87.486
-4.727	4.536	89.161
-4.673	8.464	89.231
-4.626	12.25	89.581
-4.602	15.931	89.651
-4.573	19.875	89.511
-4.534	23.632	89.371
-4.492	27.427	89.371
-4.507	31.228	90.793
-4.563	35.079	90.933
-4.661	38.833	92.726
-4.974	42.613	97.122
-5.594	46.324	102.013
-6.444	50.199	102.399
-7.174	53.909	97.943
-7.567	57.606	95.64
-7.938	61.364	95.64
-7.977	61.761	95.64
-7.99	61.97	93.252
-7.908	65.888	83.889
-7.487	69.574	84.169
-7.199	73.442	86.804
-7.058	77.282	88.758
-6.98	80.954	88.758
-6.896	84.822	88.758
-6.815	88.712	88.758
-6.759	92.414	89.825
-6.762	96.273	90.035
-6.764	100.099	90.105
-6.789	103.79	90.455
-6.829	107.641	90.984
-6.881	111.562	90.006
-6.839	115.455	89.188
-6.782	119.501	89.188
-6.675	127.058	89.188
-6.565	134.83	89.188
-6.457	142.385	89.188
-6.348	150.093	89.188
-6.241	157.619	89.188
-6.131	165.4	89.188
-6.047	172.684	89.188
-5.912	180.464	88.908
"""

RAW_TRAJ_FIRE = """
1.993	135.44	-90.482
1.992	134.656	-89.922
2.035	125.917	-90.369
1.932	116.768	-90.788
1.724	108.21	-91.985
1.414	99.265	-91.985
1.073	90.167	-92.195
0.829	81.175	-90.328
0.781	72.495	-90.258
0.696	63.26	-90.818
0.506	54.363	-91.416
0.228	45.444	-92.045
-0.062	36.956	-91.905
-0.245	28.007	-90.37
-0.248	18.865	-89.825
-0.247	10.096	-90.058
-0.258	1.439	-90.268
-0.317	-7.786	-90.548
-0.416	-16.478	-90.897
-0.596	-25.512	-91.317
-0.817	-34.048	-91.667
-1.037	-43.328	-91.134
-1.193	-51.884	-90.924
-1.34	-61.003	-90.924
-1.472	-69.82	-90.644
-1.564	-78.713	-90.574
-1.627	-87.642	-90.364
-1.684	-96.534	-90.364
-1.755	-105.597	-90.574
-1.914	-114.39	-91.591
-2.196	-123.47	-91.871
-2.452	-132.226	-91.441
-2.651	-141.085	-91.097
-2.805	-150.127	-90.893
-2.897	-159.022	-90.835
-3.067	-167.767	-91.09
-3.145	-176.666	-90.299
-3.152	-185.709	-89.788
-3.129	-194.459	-90.381
-3.241	-203.358	-90.742
-3.47	-212.251	-92.125
-3.8	-221.14	-92.125
-4.174	-230.026	-92.833
-4.758	-238.902	-94.162
-5.471	-247.622	-95.14
-6.495	-256.457	-97.641
-7.678	-265.274	-97.641
-8.901	-274.085	-98.536
-10.378	-282.858	-99.971
-11.981	-291.756	-101.011
-13.671	-300.344	-101.139
-15.39	-309.073	-101.139
-17.128	-317.946	-100.755
-18.725	-326.696	-99.86
-20.232	-335.507	-99.604
-21.753	-344.273	-100.116
-23.337	-353.027	-100.5
-24.979	-361.77	-100.756
-26.69	-370.5	-101.268
-28.4	-379.229	-100.849
-30.074	-387.964	-100.849
-31.73	-396.704	-100.593
-33.357	-405.45	-100.465
-34.973	-414.198	-100.465
-36.589	-422.945	-100.465
-38.205	-431.693	-100.465
-38.443	-432.983	-100.465
"""

RAW_TRAJ_sprinter = """
-6.579	-123.489	89.285
-6.425	-114.524	89.168
-6.303	-105.59	89.308
-6.237	-96.875	89.97
-6.251	-87.738	90.32
-6.309	-79.049	90.39
-6.333	-70.173	89.709
-6.186	-61.282	88.617
-6.045	-52.139	89.302
-5.886	-43.352	88.756
-5.725	-34.608	89.175
-5.554	-25.607	88.755
-5.455	-16.432	90.04
-5.461	-7.54	90.04
-5.378	1.178	88.92
-5.245	7.824	88.85
-5.092	15.428	88.85
-4.911	24.226	88.78
-4.725	33.23	88.85
-4.548	42.391	88.92
-4.478	50.89	89.965
-4.471	60.203	89.825
-4.392	68.969	89.372
-4.312	78.014	89.512
-4.223	86.554	89.092
-3.963	95.717	88.179
-3.699	104.272	88.249
-3.486	113.374	89.301
-3.41	122.411	89.861
-3.361	130.993	88.964
-3.193	139.985	89.03
-3.054	148.958	89.17
-2.913	157.722	88.657
-2.663	166.759	88.812
-2.493	175.436	89.021
-2.38	184.571	89.362
-2.307	193.541	89.572
-2.242	202.231	89.572
-2.149	211.123	89.328
-2.044	220.101	89.328
-2.027	221.527	89.328
"""

RAW_TRAJ_PED = """
-17.513	13.04	45.257
-17.319	13.231	44.287
-16.944	13.582	42.427
-16.572	13.922	42.427
-16.202	14.281	46.649
-15.877	14.657	51.383
-15.568	15.061	53.599
-15.285	15.486	58.638
-15.038	15.929	62.666
-14.799	16.38	59.482
-14.475	16.77	34.97
-14.013	16.972	15.45
-13.503	16.998	-4.419
-12.99	16.958	-4.419
-12.501	16.855	-31.317
-12.121	16.521	-49.345
-11.884	16.09	-70.415
-11.721	15.617	-71.082
-11.551	15.121	-71.082
-11.392	14.645	-72.641
-11.248	14.144	-74.392
-11.115	13.666	-74.392
-10.98	13.184	-74.392
-10.846	12.703	-74.392
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

        # 严格按照截图参数配置极端天气
        weather_params = RTB.build_weather(
            cloudiness=25.0, precipitation=100.0, precipitation_deposits=90.0,
            wind_intensity=100.0, sun_azimuth_angle=-1.0, sun_altitude_angle=45.0,
            fog_density=9.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
            scattering_intensity=1.0, mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )
        world.set_weather(weather_params)
        print("[场景配置] 天气系统已按照截图参数精确设置。")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化
        # ==========================================
        # 先去重，然后再以 0.5m 间隔稠密化，确保 PID 能够平滑转弯
        dense_bus = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_BUS), 0.5)
        dense_ego = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_EGO), 0.5)
        dense_fire = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_FIRE), 0.5)
        dense_sprinter = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_sprinter), 0.5)
        dense_ped = RTB.interpolate_trajectory(RTB.parse_string_trajectory(RAW_TRAJ_PED), 0.2)  # 行人需要更密集的锚点

        # ==========================================
        # 3. 车辆实体安全生成与初始状态注入
        # ==========================================
        print("\n[实体生成] 正在生成车辆...")
        # 车辆1：公交车 (Z_offset 调高防止卡地盘)
        bus = RTB.spawn_vehicle(world, 'vehicle.mitsubishi.fusorosa', dense_bus[0][0], dense_bus[0][1], z_offset=1.5)
        actor_list.append(bus)
        RTB.set_vehicle_initial_speed(bus, target_speed_kmh=60.0)

        # 车辆2：Ego小轿车 Audi TT
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        # 灯光：开启行车灯、近光灯

        # 车辆3：重型消防车
        fire = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck', dense_fire[0][0], dense_fire[0][1],
                                 color='255,255,255', z_offset=1.5)
        actor_list.append(fire)
        RTB.set_vehicle_initial_speed(fire, target_speed_kmh=80.0)
        # 灯光：开启行车灯
        light_fire = RTB.VehicleLightManager(fire)
        light_fire.set_static_lights(low_beam=False, high_beam=False)

        # 车辆4：重型欧洲货车
        sprinter = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', dense_sprinter[0][0], dense_sprinter[0][1],
                                z_offset=1.5)
        actor_list.append(sprinter)

        # 🚀【核心优化：突破重卡物理限制】🚀
        if sprinter:
            physics_control = sprinter.get_physics_control()
            # 1. 适当减重 (防止原版一两万千克的重量拖慢起步)
            physics_control.mass = 8000.0
            # 2. 极大降低空气阻力
            physics_control.drag_coefficient = 0.1
            # 3. 魔改发动机扭矩曲线 (RPM, Torque)，提供源源不断的超级推力
            physics_control.torque_curve = [carla.Vector2D(x=0, y=4000), carla.Vector2D(x=8000, y=8000)]
            physics_control.max_rpm = 8000.0 # 拉高引擎最高转速极限，防止速度刚提起来就断油
            # 将魔改后的物理属性应用回车辆
            sprinter.apply_physics_control(physics_control)

        # 初始速度提升到 80 km/h
        RTB.set_vehicle_initial_speed(sprinter, target_speed_kmh=80.0)

        # ==========================================
        # 4. 行人生成与防碰撞跟随配置
        # ==========================================
        print("\n[实体生成] 正在生成行人...")
        ped_bp = bp_lib.filter("walker.pedestrian.*")[0]

        # 提取轨迹的初始 yaw 角，并将其强制转换为 float 浮点数
        ped_init_yaw = float(RAW_TRAJ_PED.split()[2])

        # 行人1 (前面的行人)：从轨迹第 5 个点出生，走完整段轨迹
        loc_ped1 = carla.Location(x=dense_ped[5][0], y=dense_ped[5][1], z=0.5)
        walker1 = world.try_spawn_actor(ped_bp, carla.Transform(loc_ped1, carla.Rotation(yaw=ped_init_yaw)))
        actor_list.append(walker1)
        ped1_ctrl = RTB.PedestrianController(walker1, mode='trajectory', target_list=dense_ped[5:], default_speed=1.5)

        # 行人2 (后面的行人)：从轨迹第 0 个点出生，但在终点前 15 个锚点（约3米）处截断，防止两人挤撞
        loc_ped2 = carla.Location(x=dense_ped[0][0], y=dense_ped[0][1], z=0.5)
        walker2 = world.try_spawn_actor(ped_bp, carla.Transform(loc_ped2, carla.Rotation(yaw=ped_init_yaw)))
        actor_list.append(walker2)
        ped2_ctrl = RTB.PedestrianController(walker2, mode='trajectory', target_list=dense_ped[:-15], default_speed=1.5)

        # ==========================================
        # 5. 剧本状态机编排与 PID 控制器挂载
        # ==========================================
        # 提醒：每辆车必须分配独占的 PID 实例，防止历史积分混淆干扰！

        # --- Bus 逻辑 ---
        # 剧本：bus 行驶到轨迹末端后停车，并在后续仿真中保持静止。
        pid_bus_lon = RTB.PIDLongitudinalController(preset='truck')
        pid_bus_lat = RTB.PIDLateralController(preset='truck')
        idx_bus = 0
        bus_stop_point = dense_bus[-1]
        bus_stop_radius = 1.5
        bus_stopped = False

        # --- Ego 逻辑 ---
        idx_ego = 0
        ego_target_speed = 60.0
        ego_slowdown_started_at = None
        ego_recovering = False

        # --- Firetruck 逻辑 ---
        # 剧本：一路 80km/h 狂飙。由于消防车极重，给予最大纵向加速度极限
        sm_fire = RTB.MultiStageBehaviorMachine(initial_speed=80.0)
        pid_fire_lon = RTB.PIDLongitudinalController(preset='truck')
        pid_fire_lat = RTB.PIDLateralController(preset='truck')
        idx_fire = 0

        # --- sprinter 逻辑 ---
        sm_sprinter = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_sprinter.add_stage(trigger_type='y_greater', trigger_val=-20, target_speed=85.0, accel=45.0)
        sm_sprinter.add_stage(trigger_type='time', trigger_val=2.0, target_speed=60.0, accel=15.0)
        pid_sprinter_lon = RTB.PIDLongitudinalController(preset='truck')
        pid_sprinter_lat = RTB.PIDLateralController(preset='truck')
        idx_sprinter = 0

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        print("\n[仿真开启] 长尾剧本已激活，正在同步推演...")
        sim_time = 0.0
        endpoint_radius = 2.0
        ego_endpoint_radius = 5.0

        def actor_alive(actor):
            return actor is not None and actor.is_alive

        def distance_to_point(actor, point):
            loc = actor.get_location()
            return math.hypot(loc.x - point[0], loc.y - point[1])

        def destroy_actor(actor, reason):
            if not actor_alive(actor):
                return False
            try:
                actor.destroy()
                print("[生命周期] 销毁 actor {}: {}".format(actor.id, reason))
                return True
            except Exception as exc:
                print("[生命周期] 销毁 actor 失败 {}: {}".format(getattr(actor, "id", "unknown"), exc))
                return False

        def destroy_all_actors(reason):
            print("[生命周期] {}，销毁场景内所有 actor。".format(reason))
            for actor in list(actor_list):
                destroy_actor(actor, reason)

        def destroy_if_out_of_bounds(actor, label):
            if actor_alive(actor) and RTB.check_vehicle_out_of_bounds(actor, carla_map, auto_destroy=True):
                print("[生命周期] {} 出界，已销毁。".format(label))
                return True
            return False

        def destroy_if_reached_endpoint(actor, endpoint, label, radius=endpoint_radius):
            if actor_alive(actor) and distance_to_point(actor, endpoint) <= radius:
                return destroy_actor(actor, "{} 到达终点".format(label))
            return False

        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- A. 出界安全拦截器 ----------------
            # 如果 actor 偏离道路超过 6m、彻底掉出地图或到达自己的终点，自动销毁。
            destroy_if_out_of_bounds(ego, "Ego")
            destroy_if_out_of_bounds(fire, "Firetruck")
            destroy_if_out_of_bounds(sprinter, "Sprinter")

            if actor_alive(ego) and distance_to_point(ego, dense_ego[-1]) <= ego_endpoint_radius:
                destroy_all_actors("Ego 到达终点")
                break
            if not actor_alive(ego):
                print("[生命周期] Ego 已销毁，结束场景主循环。")
                break

            destroy_if_reached_endpoint(fire, dense_fire[-1], "Firetruck")
            destroy_if_reached_endpoint(sprinter, dense_sprinter[-1], "Sprinter")
            destroy_if_reached_endpoint(walker1, dense_ped[-1], "Walker1")
            destroy_if_reached_endpoint(walker2, dense_ped[-15], "Walker2")

            # ---------------- B. 动态车灯更新 ----------------
            if actor_alive(fire): light_fire.auto_update_from_control()

            # ---------------- C. 行人控制器更新 ----------------
            if actor_alive(walker1): ped1_ctrl.run_step(dt, sim_time)
            if actor_alive(walker2): ped2_ctrl.run_step(dt, sim_time)

            # ---------------- D. 车辆状态机与循迹控制 ----------------

            # [1] Bus 控制
            if actor_alive(bus):
                bus_loc = bus.get_location()
                dist_to_bus_stop = math.hypot(bus_loc.x - bus_stop_point[0], bus_loc.y - bus_stop_point[1])
                if bus_stopped or dist_to_bus_stop <= bus_stop_radius:
                    bus_stopped = True
                    bus.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                    bus.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0, hand_brake=True))
                else:
                    stopping_distance = max(dist_to_bus_stop - bus_stop_radius, 0.0)
                    tgt_spd = min(60.0, math.sqrt(2.0 * 5.0 * stopping_distance) * 3.6)
                    tgt_wp, idx_bus = RTB.get_target_waypoint(bus_loc, dense_bus, idx_bus, tgt_spd)
                    if dist_to_bus_stop < 15.0:
                        tgt_wp = bus_stop_point
                    RTB.apply_pid_control(bus, pid_bus_lon, pid_bus_lat, tgt_spd, tgt_wp)

            # [2] Ego 控制 (含预瞄点绘制)

            # [3] Firetruck 控制
            if actor_alive(fire):
                tgt_spd = sm_fire.tick(fire.get_location(), sim_time, dt)
                tgt_wp, idx_fire = RTB.get_target_waypoint(fire.get_location(), dense_fire, idx_fire, tgt_spd)
                RTB.apply_pid_control(fire, pid_fire_lon, pid_fire_lat, tgt_spd, tgt_wp)

            # [4] sprinter 控制
            if actor_alive(sprinter):
                tgt_spd = sm_sprinter.tick(sprinter.get_location(), sim_time, dt)
                tgt_wp, idx_sprinter = RTB.get_target_waypoint(sprinter.get_location(), dense_sprinter, idx_sprinter, tgt_spd)
                RTB.apply_pid_control(sprinter, pid_sprinter_lon, pid_sprinter_lat, tgt_spd, tgt_wp)

            # ---------------- E. 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
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
