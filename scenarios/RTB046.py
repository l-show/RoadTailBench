# -*- coding: utf-8 -*-
import sys
import carla
import time
import math

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

import RoadTailBenchInitV9 as RTB

# ==========================================
# 轨迹数据硬编码
# ==========================================
RAW_TRAJ_1 = """
4.344	213.753	-91.294
4.318	212.566	-91.224
4.252	208.753	-91.011
4.191	204.942	-90.195
4.168	201.069	-90.405
4.128	197.197	-90.615
4.102	193.324	-90.049
4.099	189.515	-90.049
4.121	185.645	-89.557
4.163	181.904	-88.709
4.27	178.171	-88.497
4.335	174.314	-89.776
4.301	170.45	-90.414
4.319	166.58	-89.414
4.353	162.771	-89.623
4.378	158.897	-89.623
4.404	155.149	-89.341
4.434	151.281	-89.833
4.389	147.414	-90.988
4.343	143.608	-89.71
4.367	139.737	-89.85
4.351	135.99	-90.838
4.296	132.242	-90.838
4.219	126.974	-90.838
4.092	119.351	-91.26
3.954	111.734	-90.835
3.96	104.121	-89.692
4.001	96.635	-89.622
4.045	89.018	-89.762
4.092	81.523	-89.622
4.106	74.024	-90.184
4.058	66.396	-90.609
4.002	58.896	-90.257
3.979	51.271	-90.114
3.964	43.521	-90.114
3.949	36.021	-90.114
3.884	28.272	-90.821
3.773	20.526	-90.821
3.665	13.029	-90.821
3.656	12.404	-90.821
"""

RAW_TRAJ_EGO = """
-1.146	63.383	90.893
-1.154	63.857	90.963
-1.163	64.365	90.893
-1.185	65.792	90.893
-1.255	70.87	90.468
-1.246	75.869	89.41
-1.181	80.869	89.06
-1.096	86.032	89.06
-1.028	91.189	89.412
-0.984	96.183	89.622
-0.951	101.178	89.622
-0.918	106.174	89.622
-0.885	111.169	89.622
-0.847	116.332	89.412
-0.772	121.413	89.057
-0.683	126.495	88.847
-0.598	131.496	89.269
-0.533	136.576	89.339
-0.524	141.569	90.332
-0.553	146.647	90.332
-0.58	151.722	90.332
-0.635	156.791	90.684
-0.68	161.864	90.332
-0.693	165.942	90.122
-0.698	168.356	90.122
-0.709	173.52	90.122
-0.667	178.679	89.067
-0.531	183.648	88.214
-0.313	188.617	87.288
-0.077	193.764	87.993
0.017	198.759	89.411
0.066	203.757	89.621
0.104	208.925	89.481
0.296	214.086	85.744
0.964	219.208	81.156
1.907	224.116	76.017
3.327	229.083	71.567
5.228	233.794	64.794
7.616	238.375	60.501
10.382	242.735	56.017
13.275	246.911	52.142
16.581	250.768	46.571
20.241	254.169	40.242
24.185	257.375	38.101
28.324	260.467	35.465
32.509	263.353	33.608
36.802	266.073	31.196
41.119	268.596	30.196
41.695	268.931	30.196
"""

RAW_TRAJ_3 = """
74.884	165.632	177.971
69.773	165.758	179.176
62.033	165.645	-178.676
54.419	165.402	-178.109
46.682	165.13	-177.966
38.945	164.855	-177.966
33.516	164.663	-177.966
29.772	164.53	-177.966
25.901	164.445	-179.246
22.092	164.396	-179.459
18.222	164.389	178.894
14.354	164.563	174.7
10.537	165.185	165.246
10.054	165.314	164.956
7.054	166.183	160.991
3.738	168.095	129.64
2.001	171.384	110.501
0.92	175.031	105.407
-0.005	178.718	98.164
-0.113	182.523	89.989
-0.121	186.394	90.131
-0.13	190.141	90.131
-0.143	196.093	90.131
-0.063	204.834	87.678
0.502	213.563	84.024
1.771	222.507	78.094
4.198	230.903	68.969
7.91	239.141	61.034
12.471	246.6	52.856
18.501	253.121	41.728
25.398	258.498	36.19
32.909	263.526	31.791
40.716	268.083	29.165
48.611	272.488	29.165
56.235	276.77	29.52
63.833	281.073	29.52
65.607	282.077	29.52
"""


# ✅ 修复：自定义数据清洗器，剥离第三列的 Yaw，防止变成海拔(Z轴)
def clean_and_strip_yaw(data_str):
    lines = data_str.strip().split('\n')
    raw_pts = []
    yaw_start = 0.0
    got_yaw = False

    for line in lines:
        parts = line.split()
        if len(parts) >= 3:
            # 仅保留 X, Y。这保证了所有预瞄点都在地平面
            raw_pts.append((float(parts[0]), float(parts[1])))
            if not got_yaw:
                yaw_start = float(parts[2])
                got_yaw = True

    # 去重
    cleaned = RTB.clean_trajectory(raw_pts, min_dist=0.5)
    return cleaned, yaw_start




# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (41.695, 268.931)
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

        RTB.set_static_weather(
            world, cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_azimuth_angle=275.0, sun_altitude_angle=5.0,
            fog_density=2.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
            scattering_intensity=1.0, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.0331
        )

        # ==========================================
        # 2. 轨迹解析与稠密化 (剥离脏数据)
        # ==========================================
        clean_1, yaw_1 = clean_and_strip_yaw(RAW_TRAJ_1)
        clean_ego, yaw_ego = clean_and_strip_yaw(RAW_TRAJ_EGO)
        clean_3, yaw_3 = clean_and_strip_yaw(RAW_TRAJ_3)

        traj_1 = RTB.interpolate_trajectory(clean_1, interval=0.5)
        traj_ego = RTB.interpolate_trajectory(clean_ego, interval=0.5)
        traj_3 = RTB.interpolate_trajectory(clean_3, interval=0.5)

        # 绘制调试点
        RTB.draw_preset_trajectory(world, traj_1, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(255, 165, 0))
        RTB.draw_preset_trajectory(world, traj_3, color=carla.Color(150, 150, 150))

        # ==========================================
        # 3. 车辆生成与初速度注入
        # ==========================================
        v1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=clean_1[0][0], y=clean_1[0][1], yaw=yaw_1,
                               role_name="v1")
        RTB.set_vehicle_initial_speed(v1, 60.0, yaw_deg=yaw_1)
        actor_list.append(v1)

        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=clean_ego[0][0], y=clean_ego[0][1], yaw=yaw_ego,
                                role_name="ego")
        RTB.set_vehicle_initial_speed(ego, 60.0, yaw_deg=yaw_ego)
        actor_list.append(ego)

        v3 = RTB.spawn_vehicle(world, 'vehicle.lincoln.mkz_2020', x=clean_3[0][0], y=clean_3[0][1], yaw=yaw_3,
                               role_name="v3")
        RTB.set_vehicle_initial_speed(v3, 40.0, yaw_deg=yaw_3)
        actor_list.append(v3)

        # ==========================================
        # 4. 车辆灯光管理配置
        # ==========================================
        light_1 = RTB.VehicleLightManager(v1)
        light_1.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.HighBeam)

        light_ego = RTB.VehicleLightManager(ego)
        light_ego.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam)

        light_3 = RTB.VehicleLightManager(v3)
        light_3.turn_on(carla.VehicleLightState.Position)

        # ==========================================
        # 5. PID与剧本状态机编排
        # ==========================================
        # ✅ 修复核心：每辆车必须拥有自己独立且专属的 PID 控制器！
        pid_lon_1, pid_lat_1 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_3, pid_lat_3 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()

        idx1, idx_ego, idx3 = 0, 0, 0

        # --- 剧本编排 ---
        sm_1 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_1.add_stage('time', target_speed=30.0, trigger_val=3.0, accel=15.0)
        sm_1.add_stage('time', target_speed=60.0, trigger_val=3.0, accel=15.0)

        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_ego.add_stage('y_greater', target_speed=20.0, trigger_val=140.0, accel=35.0)
        sm_ego.add_stage('time', target_speed=41.0, trigger_val=3.0, accel=15.0)

        sm_3 = RTB.MultiStageBehaviorMachine(initial_speed=40.0)

        # ==========================================
        # 6. 仿真主循环
        # ==========================================

        print("🚀 长尾场景：逆光跟车与动态减速 仿真开始...")
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ----------------【 车辆 1 】----------------
            if v1 and v1.is_alive:
                if RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True):
                    print("[系统] 车辆1 出界已被销毁")
                else:
                    target_spd_1 = sm_1.tick(v1.get_location(), sim_time, dt)
                    wp_1, idx1 = RTB.get_target_waypoint(v1.get_location(), traj_1, idx1, speed_kmh=target_spd_1)
                    if wp_1:
                        # 传入它专属的 PID
                        RTB.apply_pid_control(v1, pid_lon_1, pid_lat_1, target_spd_1, wp_1)
                    light_1.auto_update_from_control()

            # ----------------【 Ego 】----------------
            if ego and ego.is_alive:
                if RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    print("[系统] EGO 出界已被销毁")
                else:
                    target_spd_ego = sm_ego.tick(ego.get_location(), sim_time, dt)
                    wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego,
                                                              speed_kmh=target_spd_ego)
                    if wp_ego:
                        # 传入它专属的 PID
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_spd_ego, wp_ego)
                        RTB.draw_lookahead_point(world, ego.get_location(), wp_ego)
                    light_ego.auto_update_from_control()

            # ----------------【 车辆 3 】----------------
            if v3 and v3.is_alive:
                if RTB.check_vehicle_out_of_bounds(v3, carla_map, auto_destroy=True):
                    print("[系统] 车辆3 出界已被销毁")
                else:
                    target_spd_3 = sm_3.tick(v3.get_location(), sim_time, dt)
                    wp_3, idx3 = RTB.get_target_waypoint(v3.get_location(), traj_3, idx3, speed_kmh=target_spd_3)
                    if wp_3:
                        # 传入它专属的 PID
                        RTB.apply_pid_control(v3, pid_lon_3, pid_lat_3, target_spd_3, wp_3)
                    light_3.auto_update_from_control()

            # ---------------- 硬件时钟补齐 ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        if 'world' in locals():
            RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()