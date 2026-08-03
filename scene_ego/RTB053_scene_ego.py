import sys
import time
import carla

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 剧本轨迹数据 (去除表头，直接保留纯数据)
# ==========================================
TRAJ_TRUCK_STR = """
64.007	-9.848	176.341
60.474	-9.61	175.99
54.129	-9.296	178.195
47.779	-9.144	178.983
41.535	-9.045	179.762
35.191	-9.018	179.762
28.844	-8.992	179.762
22.5	-9.022	-177.944
16.056	-9.275	-177.732
9.82	-9.522	-177.732
5.352	-9.71	-176.955
-0.958	-10.371	-168.754
-7.224	-11.871	-165.792
-13.347	-13.514	-163.387
-19.328	-15.612	-159.024
-25.079	-18.02	-155.788
-30.839	-20.909	-151.007
-36.337	-24.264	-146.072
-41.494	-27.95	-143.21
-46.589	-31.897	-141.007
-51.333	-36.105	-135.79
-55.585	-40.809	-128.121
-59.477	-45.82	-127.831
-63.432	-50.913	-127.831
-67.253	-55.847	-127.116
-70.947	-60.878	-125.479
-74.569	-66.14	-122.883
-77.908	-71.657	-118.629
-80.973	-77.332	-117.777
-83.684	-83.068	-112.212
-86.017	-88.86	-111.922
-88.419	-94.848	-111.354
-90.588	-100.809	-109.606
-92.751	-106.883	-109.606
-94.914	-112.956	-109.606
-97.043	-118.932	-109.606
-99.206	-125.005	-109.606
-99.66	-126.279	-109.606
"""

TRAJ_IMPALA_STR = """
126.689	-6.517	177.515
110.357	-6.367	-179.989
103.357	-6.369	-179.919
97.857	-6.377	-179.919
83.889	-6.33	179.796
69.938	-6.28	179.796
55.771	-6.215	179.656
43.665	-6.142	179.656
35.899	-6.096	179.656
21.967	-5.876	177.758
7.814	-5.147	176.906
-2.456	-4.592	176.906
-11.461	-4.523	-173.995
-20.113	-6.943	-159.473
-23.536	-8.227	-159.261
-25.94	-9.156	-156.292
-28.209	-10.376	-147.987
-30.3	-11.849	-140.628
-32.025	-13.699	-127.551
-33.564	-15.715	-127.338
-35.103	-17.731	-127.553
-36.784	-19.638	-137.21
-38.804	-21.244	-144.827
-40.931	-22.708	-145.517
-46.285	-26.385	-145.517
-55.568	-33.208	-136.482
-63.671	-41.265	-132.541
-71.147	-50.137	-129.074
-78.214	-59.336	-125.698
-84.344	-69.167	-117.884
-89.417	-79.381	-114.921
-93.884	-90.075	-111.392
-97.901	-100.755	-109.769
-101.693	-111.723	-108.564
-104.494	-120.065	-108.494
"""

TRAJ_EGO_STR = """
135.099	-10.349	177.451
133.932	-10.298	177.591
124.898	-10.065	178.948
116.005	-9.923	179.16
106.986	-9.887	-179.85
98.121	-9.91	-179.85
89.108	-9.912	-179.99
80.381	-9.914	-179.99
71.506	-9.915	-179.99
62.482	-9.917	-179.99
53.457	-9.919	-179.99
49.09	-9.919	-179.99
46.907	-9.92	-179.99
37.882	-9.921	-179.99
30.438	-9.923	-179.99
25.281	-9.923	-179.99
20.207	-9.924	-179.99
17.419	-9.925	-179.99
14.923	-9.925	-179.99
12.343	-9.926	-179.99
9.764	-9.926	-179.99
7.227	-9.933	-178.733
4.694	-10.053	-175.647
2.873	-10.233	-172.776
2.708	-10.254	-172.776
0.157	-10.589	-171.493
-2.348	-10.994	-170.14
-4.888	-11.44	-169.997
-7.418	-11.938	-167.558
-9.876	-12.573	-164.383
-12.35	-13.3	-162.693
-14.807	-14.078	-162.338
-17.183	-14.839	-162.055
-19.595	-15.624	-161.49
-22.022	-16.495	-159.624
-24.432	-17.413	-158.129
-26.766	-18.402	-155.714
-29.097	-19.503	-154.424
-31.419	-20.623	-152.625
-33.63	-21.867	-149.524
-35.844	-23.195	-146.695
-37.966	-24.589	-146.695
-38.974	-25.252	-146.625
-41.03	-26.802	-136.337
-42.866	-28.555	-136.337
-46.629	-32.145	-136.337
-53.755	-39.079	-132.718
-60.277	-46.642	-130.091
-66.839	-54.389	-131.823
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-66.839, -54.389)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.audi.tt'
_RTB_OPT_EGO_ROLE_NAMES = ['ego', 'hero']
_RTB_OPT_GOAL_RADIUS_M = 5.0
_RTB_OPT_GOAL_HITS = 0

def _rtb_opt_is_alive(actor):
    return bool(actor is not None and hasattr(actor, 'is_alive') and actor.is_alive)

def _rtb_opt_iter_actor_values(value, seen=None):
    if seen is None:
        seen = set()
    obj_id = id(value)
    if obj_id in seen:
        return
    seen.add(obj_id)
    if _rtb_opt_is_alive(value) and hasattr(value, 'get_location'):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from _rtb_opt_iter_actor_values(item, seen)
    elif isinstance(value, (list, tuple, set)):
        for item in value:
            yield from _rtb_opt_iter_actor_values(item, seen)

def _rtb_opt_actor_matches_ego(actor):
    if not _rtb_opt_is_alive(actor):
        return False
    try:
        role_name = actor.attributes.get('role_name', '')
        if role_name in _RTB_OPT_EGO_ROLE_NAMES:
            return True
    except Exception:
        pass
    try:
        if _RTB_OPT_EGO_TYPE_ID and actor.type_id == _RTB_OPT_EGO_TYPE_ID:
            return True
    except Exception:
        pass
    return False

def _rtb_opt_find_ego(local_vars):
    preferred_names = ('ego', 'ego_vehicle', 'vehicle_ego', 'v3_ego', 'v2_ego', 'agent_ego', 'audi', 'tesla', 'moto', 'truck', 'firetruck')
    for name in preferred_names:
        if name in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[name]):
                if _rtb_opt_actor_matches_ego(actor) or 'ego' in name.lower():
                    return actor
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            if _rtb_opt_actor_matches_ego(actor):
                return actor
    return None

def _rtb_opt_collect_scene_actors(local_vars, world):
    actors = []
    seen = set()

    def add(actor):
        if not _rtb_opt_is_alive(actor):
            return
        try:
            actor_id = actor.id
        except Exception:
            actor_id = id(actor)
        if actor_id in seen:
            return
        seen.add(actor_id)
        actors.append(actor)

    for key in ('actor_list', 'actors', 'vehicles', 'spawned_actors'):
        if key in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[key]):
                add(actor)
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            add(actor)
    try:
        world_actors = world.get_actors()
        for pattern in ('vehicle.*', 'walker.*', 'sensor.*', 'controller.*', 'static.prop.*', 'static.trigger.*'):
            for actor in world_actors.filter(pattern):
                add(actor)
    except Exception:
        pass
    return actors

def _rtb_opt_cleanup_scene(local_vars, client, world):
    actors = _rtb_opt_collect_scene_actors(local_vars, world)
    try:
        commands = [carla.command.DestroyActor(actor.id) for actor in actors if _rtb_opt_is_alive(actor)]
        if commands:
            client.apply_batch(commands)
        return
    except Exception:
        pass
    for actor in actors:
        try:
            if _rtb_opt_is_alive(actor):
                actor.destroy()
        except Exception:
            pass

def _rtb_opt_goal_guard(local_vars, client, world):
    global _RTB_OPT_GOAL_HITS
    if _RTB_OPT_EGO_GOAL_XY is None:
        _RTB_OPT_GOAL_HITS = 0
        return False
    ego_actor = _rtb_opt_find_ego(local_vars)
    if not _rtb_opt_is_alive(ego_actor):
        _RTB_OPT_GOAL_HITS = 0
        return False
    try:
        loc = ego_actor.get_location()
        dist = ((loc.x - _RTB_OPT_EGO_GOAL_XY[0]) ** 2 + (loc.y - _RTB_OPT_EGO_GOAL_XY[1]) ** 2) ** 0.5
    except Exception:
        _RTB_OPT_GOAL_HITS = 0
        return False
    if dist <= _RTB_OPT_GOAL_RADIUS_M:
        _RTB_OPT_GOAL_HITS += 1
    else:
        _RTB_OPT_GOAL_HITS = 0
    if _RTB_OPT_GOAL_HITS >= 2:
        print('[RoadTailBench Opt] Ego reached trajectory endpoint; cleaning all scene actors and ending simulation.')
        _rtb_opt_cleanup_scene(local_vars, client, world)
        return True
    return False
# === End RoadTailBench Opt guard ===

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

        # 严格按照给定的截图配置天气
        RTB.set_static_weather(
            world,
            cloudiness=10.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=10.0,
            sun_azimuth_angle=-1.0,
            sun_altitude_angle=3.0,
            fog_density=2.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 天气系统已按照要求配置完毕。")

        # ==========================================
        # 2. 轨迹数据硬编码与清洗
        # ==========================================
        # 先解析字符串，得到去重后的 (x, y, yaw) 列表
        raw_truck = RTB.parse_string_trajectory(TRAJ_TRUCK_STR)
        raw_impala = RTB.parse_string_trajectory(TRAJ_IMPALA_STR)
        raw_ego = RTB.parse_string_trajectory(TRAJ_EGO_STR)

        # 核心防飞天机制：手动将第三个元素(yaw)剔除，锁死 Z=0.0，再进行稠密化插值 (0.5米精度)
        traj_truck = RTB.interpolate_trajectory([(p[0], p[1], 0.0) for p in raw_truck], interval=0.5)
        traj_impala = RTB.interpolate_trajectory([(p[0], p[1], 0.0) for p in raw_impala], interval=0.5)
        traj_ego = RTB.interpolate_trajectory([(p[0], p[1], 0.0) for p in raw_ego], interval=0.5)

        # ==========================================
        # 3. 车辆生成与物理突破机制
        # ==========================================
        # 生成大卡车（使用偏航角 yaw 初始化朝向，卡车需略微抬高防止卡地爆炸）
        truck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck',
                                  raw_truck[0][0], raw_truck[0][1], yaw=raw_truck[0][2], z_offset=1.5,
                                  role_name='truck')
        # 生成小轿车
        impala = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala',
                                   raw_impala[0][0], raw_impala[0][1], yaw=raw_impala[0][2], role_name='impala')
        # 生成EGO轿车
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                raw_ego[0][0], raw_ego[0][1], yaw=raw_ego[0][2], role_name='ego')

        actor_list.extend([truck, impala, ego])

        # 【核心要求】突破卡车动力学物理限制 (让笨重的消防车能起步跑到80km/h)
        if truck:
            physics_control = truck.get_physics_control()
            physics_control.mass = 3500.0  # 大幅减轻质量
            physics_control.drag_coefficient = 0.1  # 减少空气阻力
            # 暴力拉升引擎转矩曲线
            for curve in physics_control.torque_curve:
                curve.y *= 3.0
            truck.apply_physics_control(physics_control)
            print("[车辆配置] 大卡车物理限制已被突破。")

        # ==========================================
        # 4. 初始化状态机、灯光与初速度
        # ==========================================
        # --- 卡车 ---
        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck')  # 使用卡车 PID 预设
        pid_lat_truck = RTB.PIDLateralController(preset='truck')
        idx_truck = 0
        sm_truck = RTB.MultiStageBehaviorMachine(initial_speed=80.0)
        # x坐标在递减，当越过 x=9.8 时减速
        sm_truck.add_stage(trigger_type='x_less', trigger_val=9.8, target_speed=40.0, accel=30.0)
        sm_truck.add_stage(trigger_type='time', trigger_val=4.0, target_speed=70.0, accel=15.0)

        # --- Impala 小轿车 ---
        pid_lon_impala = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_impala = RTB.PIDLateralController(preset='default_car')
        idx_impala = 0
        lights_impala = RTB.VehicleLightManager(impala)
        lights_impala.set_static_lights()  # 开启行车灯
        lights_impala.start_flashing('hazard')  # 开启双闪

        sm_impala = RTB.MultiStageBehaviorMachine(initial_speed=110.0)
        # 注意：轨迹的 Y 是负数( -6 -> -120)，因此修正为 y < -43.0
        sm_impala.add_stage(trigger_type='y_less', trigger_val=-53.0, target_speed=40.0, accel=50.0)
        sm_impala.add_stage(trigger_type='time', trigger_val=2.0, target_speed=20.0, accel=15.0)
        sm_impala.add_stage(trigger_type='time', trigger_val=10.0, target_speed=80.0, accel=15.0)

        # --- Ego 主车 ---
        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')
        idx_ego = 0
        lights_ego = RTB.VehicleLightManager(ego)
        lights_ego.set_static_lights()  # 开启行车灯

        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=110.0)
        # X轴在不断变小，按要求触发
        sm_ego.add_stage(trigger_type='x_less', trigger_val=60.0, target_speed=50.0, accel=30.0)
        sm_ego.add_stage(trigger_type='x_less', trigger_val=2.8, target_speed=30.0, accel=15.0)
        sm_ego.add_stage(trigger_type='x_less', trigger_val=-37.0, target_speed=70.0, accel=15.0)

        # 瞬间注入物理初速度防打滑
        RTB.set_vehicle_initial_speed(truck, 80.0, raw_truck[0][2])
        RTB.set_vehicle_initial_speed(impala, 110.0, raw_impala[0][2])
        RTB.set_vehicle_initial_speed(ego, 110.0, raw_ego[0][2])

        print("[仿真启动] 所有实体初始化完成，开始主循环。")

        # ==========================================
        # 5. 仿真主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ---------------- 卡车逻辑 ----------------
            if truck and truck.is_alive:
                t_loc = truck.get_location()
                t_spd = sm_truck.tick(t_loc, sim_time, dt)
                wp_truck, idx_truck = RTB.get_target_waypoint(t_loc, traj_truck, idx_truck, speed_kmh=t_spd)
                if wp_truck:
                    RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, t_spd, wp_truck)
                RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True)

            # ---------------- Impala逻辑 ----------------
            if impala and impala.is_alive:
                i_loc = impala.get_location()
                i_spd = sm_impala.tick(i_loc, sim_time, dt)
                wp_impala, idx_impala = RTB.get_target_waypoint(i_loc, traj_impala, idx_impala, speed_kmh=i_spd)
                if wp_impala:
                    RTB.apply_pid_control(impala, pid_lon_impala, pid_lat_impala, i_spd, wp_impala)

                # 更新灯光和双闪特效
                lights_impala.tick(sim_time)
                lights_impala.auto_update_from_control()
                RTB.check_vehicle_out_of_bounds(impala, carla_map, auto_destroy=True)

            # ---------------- Ego逻辑 ----------------
            if ego and ego.is_alive:
                e_loc = ego.get_location()
                e_spd = sm_ego.tick(e_loc, sim_time, dt)
                wp_ego, idx_ego = RTB.get_target_waypoint(e_loc, traj_ego, idx_ego, speed_kmh=e_spd)
                if wp_ego:
                    RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, e_spd, wp_ego)

                lights_ego.auto_update_from_control()
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
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()