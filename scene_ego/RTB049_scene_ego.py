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
# 轨迹数据硬编码
# ==========================================
TRAJ_STR_V1 = """
86.47	-6.395	178.011
75.672	-6.395	175.917
64.47	-6.395	178.011
56.672	-5.908	175.917
45.152	-5.471	179.073
33.743	-5.421	-179.509
22.126	-5.603	-179.014
10.505	-5.792	-179.084
-0.881	-6.161	-176.227
-12.218	-8.165	-166.142
-22.827	-11.65	-158.124
-31.857	-15.276	-158.124
-40.171	-18.682	-155.659
-47.618	-23.115	-141.406
-54.114	-28.894	-135.472
-60.138	-35.583	-129.917
-65.783	-42.607	-127.841
-71.098	-49.513	-127.973
-76.678	-56.574	-129.033
-82.633	-63.883	-129.175
"""

TRAJ_STR_EGO = """
138.882	-10.585	178.913
135.776	-10.475	177.7
125.812	-10.066	178.557
114.927	-9.958	-179.523
102.215	-10.028	-179.735
86.877	-10.047	-179.875
71.814	-9.988	179.633
56.683	-9.796	178.86
41.3	-9.509	179.352
25.989	-9.349	179.21
10.668	-9.691	-177.518
-4.52	-10.25	-178.153
-19.685	-10.827	-174.416
-33.663	-16.586	-152.291
-46	-25.361	-139.113
-56.646	-36.145	-131.083
-66.322	-47.822	-128.241
-78.683	-63.725	-127.678
"""

TRAJ_STR_TRUCK = """
-66.287	-31.272	50.535
-54.041	-18.057	42.758
-40.109	-7.618	31.322
-23.986	-0.319	18.743
-6.552	4.218	9.501
11.299	5.235	0.115
29.077	5.233	0.118
47.138	5.138	-0.517
64.948	4.954	0.063
82.614	4.889	-0.714
100.472	4.798	0.128
118.183	4.676	-0.613
135.704	4.654	0.159
147.6	4.687	0.159
"""

TRAJ_STR_POLICE = """
65.349	-9.396	177.45
60.77	-9.302	179.996
55.692	-9.301	179.996
50.346	-9.301	179.996
45.418	-9.368	-178.521
38.985	-9.549	-178.381
28.833	-9.667	179.586
18.586	-9.616	179.801
8.396	-9.58	179.731
-1.445	-9.867	-175.217
-8.876	-10.979	-168.776
-14.336	-12.149	-166.775
-21.363	-15.611	-148.142
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-78.683, -63.725)
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
        bp_lib = world.get_blueprint_library()
        dt = 0.05

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 完全按照截图硬编码天气参数
        RTB.set_static_weather(
            world,
            cloudiness=50.0,
            precipitation=0.0,
            precipitation_deposits=15.0,
            wind_intensity=20.0,
            sun_azimuth_angle=10.0,
            sun_altitude_angle=20.0,
            fog_density=36.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 天气系统已按照截图设置完成")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化 (间距 0.5m)
        # ==========================================
        # v1 (Impala)
        raw_v1 = RTB.parse_string_trajectory(TRAJ_STR_V1)
        path_v1 = RTB.interpolate_trajectory(raw_v1, interval=0.5)

        # Ego (Audi TT)
        raw_ego = RTB.parse_string_trajectory(TRAJ_STR_EGO)
        path_ego = RTB.interpolate_trajectory(raw_ego, interval=0.5)

        # Truck (Firetruck)
        raw_truck = RTB.parse_string_trajectory(TRAJ_STR_TRUCK)
        path_truck = RTB.interpolate_trajectory(raw_truck, interval=0.5)

        # Police (Charger)
        raw_police = RTB.parse_string_trajectory(TRAJ_STR_POLICE)
        path_police = RTB.interpolate_trajectory(raw_police, interval=0.5)
        #

        # ==========================================
        # 3. 实体生成与控制器挂载
        # ==========================================

        # --- 车辆1: 小轿车 Impala ---
        v1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', raw_v1[0][0], raw_v1[0][1], yaw=raw_v1[0][2])
        actor_list.append(v1)
        pid_lon_v1 = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_v1 = RTB.PIDLateralController(preset='default_car')
        lm_v1 = RTB.VehicleLightManager(v1)
        lm_v1.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.Fog)  # 行车灯、雾灯

        # --- 车辆2: Ego (Audi TT) ---
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', raw_ego[0][0], raw_ego[0][1], yaw=raw_ego[0][2],
                                role_name='ego')
        actor_list.append(ego)
        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')
        lm_ego = RTB.VehicleLightManager(ego)
        lm_ego.turn_on(
            carla.VehicleLightState.Position | carla.VehicleLightState.Fog | carla.VehicleLightState.LowBeam)  # 行车灯、雾灯、近光灯

        # --- 车辆3: 大卡车 (Firetruck) ---
        # 注意: 卡车由于模型体积大，必须抬高 z_offset 防卡地穿模
        truck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck', raw_truck[0][0], raw_truck[0][1],
                                  yaw=raw_truck[0][2], z_offset=1.5)
        actor_list.append(truck)
        # 使用专用的 'truck' PID 预设，提供更大的扭矩和更稳的方向盘控制
        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_truck = RTB.PIDLateralController(preset='truck')

        # --- 车辆4: 警车 ---
        police = RTB.spawn_vehicle(world, 'vehicle.dodge.charger_police', raw_police[0][0], raw_police[0][1],
                                   yaw=raw_police[0][2])
        actor_list.append(police)
        pid_lon_police = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_police = RTB.PIDLateralController(preset='default_car')
        lm_police = RTB.VehicleLightManager(police)
        lm_police.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.Fog)
        lm_police.start_flashing(mode='police')  # 警灯闪烁

        # ==========================================
        # 4. 剧本状态机编排 (核心控制逻辑)
        # ==========================================

        # Ego 剧本: 初始60，在 x=55 时降速到30，维持3秒后恢复60
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        # ego 是往 -X 方向开的，所以当 X 小于 55 时触发减速
        sm_ego.add_stage('x_less', target_speed=30.0, trigger_val=55.0, accel=20.0)
        # 降到30后，等待3秒，再慢慢加速回60
        sm_ego.add_stage('time', target_speed=60.0, trigger_val=3.0, accel=10.0)

        # Police 剧本: 初始60，最终缓慢停车到最后一个点
        sm_police = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        end_point_police = (raw_police[-1][0], raw_police[-1][1])
        # 距离最后一个点 15米 的时候触发缓慢刹车 (accel 较小，实现缓慢停稳)
        sm_police.add_stage('point', target_speed=0.0, trigger_val=end_point_police, tolerance=15.0, accel=12.0)

        # ==========================================
        # 5. 初始状态注入 (消除启动时的卡顿与打滑)
        # ==========================================
        RTB.set_vehicle_initial_speed(v1, target_speed_kmh=40.0)
        RTB.set_vehicle_initial_speed(ego, target_speed_kmh=60.0)
        # 突破卡车重量限制的核心：强行赋予巨大的初始物理动量
        RTB.set_vehicle_initial_speed(truck, target_speed_kmh=70.0)
        RTB.set_vehicle_initial_speed(police, target_speed_kmh=60.0)

        # 寻迹索引追踪器
        idx_v1, idx_ego, idx_truck, idx_police = 0, 0, 0, 0
        sim_time = 0.0

        print("[RoadTailBench] 🚀 仿真场景初始化完毕，开始主循环推演...")

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        # 将视角绑定到 Ego 车后方以便观察
        # spectator = world.get_spectator()
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt
            # # ---------------- 视角跟随 ----------------
            # if ego and ego.is_alive:
            #     tf = ego.get_transform()
            #     spectator.set_transform(carla.Transform(
            #         tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
            #         carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
            #     ))
            # ----- 车辆状态刷新与控制器下发 -----

            # 1. 车辆1 (常速 40)
            if v1 and v1.is_alive:
                if RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True):
                    pass
                else:
                    spd_v1 = 3.6 * math.hypot(v1.get_velocity().x, v1.get_velocity().y)
                    wp_v1, idx_v1 = RTB.get_target_waypoint(v1.get_location(), path_v1, idx_v1, spd_v1)
                    if wp_v1: RTB.apply_pid_control(v1, pid_lon_v1, pid_lat_v1, 40.0, wp_v1)
                    lm_v1.auto_update_from_control()  # 自动刹车灯/转向灯

            # 2. Ego 车辆 (状态机动态控速 + 预瞄点绘制)
            if ego and ego.is_alive:
                if RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    pass
                else:
                    ego_loc = ego.get_location()
                    spd_ego = 3.6 * math.hypot(ego.get_velocity().x, ego.get_velocity().y)
                    # 状态机滴答：获取当前帧应该达到的目标速度
                    target_spd_ego = sm_ego.tick(ego_loc, sim_time, dt)

                    wp_ego, idx_ego = RTB.get_target_waypoint(ego_loc, path_ego, idx_ego, spd_ego)
                    if wp_ego:
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_spd_ego, wp_ego)
                    lm_ego.auto_update_from_control()

            # 3. 大卡车 (常速 70)
            if truck and truck.is_alive:
                if RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True):
                    pass
                else:
                    spd_truck = 3.6 * math.hypot(truck.get_velocity().x, truck.get_velocity().y)
                    wp_truck, idx_truck = RTB.get_target_waypoint(truck.get_location(), path_truck, idx_truck,
                                                                  spd_truck)
                    if wp_truck: RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, 70.0, wp_truck)

            # 4. 警车 (状态机缓慢停车 + 爆闪)
            if police and police.is_alive:
                if RTB.check_vehicle_out_of_bounds(police, carla_map, auto_destroy=True):
                    pass
                else:
                    pol_loc = police.get_location()
                    spd_pol = 3.6 * math.hypot(police.get_velocity().x, police.get_velocity().y)
                    target_spd_pol = sm_police.tick(pol_loc, sim_time, dt)

                    wp_police, idx_police = RTB.get_target_waypoint(pol_loc, path_police, idx_police, spd_pol)
                    if wp_police: RTB.apply_pid_control(police, pid_lon_police, pid_lat_police, target_spd_pol,
                                                        wp_police)

                    lm_police.auto_update_from_control()
                    lm_police.tick(sim_time)  # 维持警灯爆闪

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[运行异常] {e}")
    finally:
        # 恢复异步模式并一键清理场景实体
        if 'world' in locals():
            RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()