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

def ramp_speed_kmh(current_speed, target_speed, rate_kmh_per_s, dt):
    max_step = rate_kmh_per_s * dt
    delta = target_speed - current_speed
    if abs(delta) <= max_step:
        return target_speed
    return current_speed + math.copysign(max_step, delta)

# ==========================================
# 轨迹数据硬编码 (X, Y, Yaw)
# ==========================================
RAW_TRAJ_AUDI_EGO = [
    (30.2, -60.285, 134.595), (30.2, -60.285, 134.595), (30.2, -60.285, 134.595),
    (30.2, -60.285, 134.595), (30.2, -60.285, 133.729), (30.2, -60.285, 132.6),
    (26.715, -56.499, 132.812), (19.688, -49.186, 135.161), (12.38, -42.123, 136.304),
    (4.985, -35.148, 137.219), (-2.518, -28.296, 137.783), (-10.046, -21.466, 137.783),
    (-17.289, -14.885, 137.713), (-20.585, -11.887, 137.713), (-23.368, -9.345, 136.63),
    (-26.13, -6.719, 136.132), (-28.877, -4.078, 136.132), (-31.58, -1.481, 136.202),
    (-34.331, 1.156, 136.202), (-36.918, 3.945, 126.205), (-38.892, 7.199, 115.504),
    (-40.206, 10.775, 105.165), (-40.832, 14.525, 94.582), (-40.826, 18.258, 82.012),
    (-39.85, 21.915, 65.578), (-37.925, 25.197, 54.915), (-35.388, 28.029, 41.489),
    (-32.355, 30.331, 33.583), (-29.006, 32.141, 23.606), (-25.544, 33.578, 21.614),
    (-21.988, 34.942, 20.054), (-18.41, 36.245, 19.984), (-14.893, 37.533, 21.132),
    (-11.446, 39.136, 30.927), (-8.393, 41.402, 39.771), (-5.631, 43.973, 44.94),
    (-2.963, 46.686, 46.814), (-0.525, 49.522, 52.782), (1.471, 52.753, 62.645),
    (3.201, 56.143, 64.592), (4.625, 59.674, 71.574), (5.558, 63.366, 79.299),
    (6.131, 67.132, 83.34), (6.407, 70.931, 87.894), (6.44, 74.678, 91.346),
    (6.294, 78.487, 93.193), (5.921, 82.28, 96.966), (5.405, 86.057, 97.904),
    (4.777, 90.576, 97.904), (4.072, 95.611, 98.117), (3.364, 100.643, 97.904),
    (2.678, 105.677, 97.412), (2.032, 110.716, 97.272), (1.246, 116.874, 97.272),
    (0.281, 124.435, 97.272), (-0.673, 131.995, 96.847), (-1.581, 139.558, 96.847),
    (-2.497, 147.127, 96.917), (-3.416, 154.698, 96.917), (-4.419, 162.259, 99.075),
    (-5.969, 169.721, 104.793), (-8.884, 176.739, 119.372), (-13.351, 182.89, 134.637),
    (-19.156, 187.814, 143.005), (-25.643, 191.801, 150.406), (-31.706, 196.14, 128.392),
    (-33.78, 203.328, 88.007), (-32.325, 210.771, 68.302), (-28.756, 217.355, 58.545),
    (-24.739, 223.833, 57.697), (-23.868, 225.205, 57.557), (-23.868, 225.205, 57.557),
    (-23.868, 225.205, 57.557), (-23.868, 225.205, 57.557)
]

RAW_TRAJ_IMPALA_NPC = [
    (0.981, 152.671, -87.345), (0.981, 152.671, -87.345), (0.981, 152.671, -87.345),
    (0.981, 152.671, -86.632), (1.126, 150.208, -86.632), (1.478, 145.191, -85.058),
    (1.952, 140.147, -84.493), (2.611, 135.195, -79.95), (3.476, 130.186, -81.24),
    (4.142, 125.147, -83.03), (4.697, 120.094, -84.165), (5.213, 115.042, -83.952),
    (5.788, 110.08, -83.242), (6.407, 105.039, -82.537), (7.1, 100.006, -82.046),
    (7.806, 94.974, -81.836), (8.533, 89.945, -81.766), (9.258, 84.913, -82.191),
    (9.805, 79.862, -86.194), (9.981, 74.867, -88.751), (10.065, 69.786, -89.884),
    (9.928, 64.707, -94.071), (9.004, 59.722, -106.298), (7.349, 54.92, -113.261),
    (5.017, 50.407, -119.964), (2.183, 46.195, -129.723), (-1.327, 42.641, -136.248),
    (-5.12, 39.26, -140.625), (-9.275, 36.343, -147.617), (-13.695, 33.847, -155.295),
    (-18.375, 31.867, -158.326), (-23.157, 30.149, -160.889), (-27.956, 28.479, -160.023),
    (-32.65, 26.538, -156.649), (-37.254, 24.405, -144.605), (-40.104, 20.325, -102.548),
    (-40.286, 15.29, -78.01), (-38.608, 10.506, -64.327), (-35.56, 6.474, -45.6),
    (-31.802, 3.059, -41.606), (-28.076, -0.396, -42.573), (-24.321, -3.822, -42.361),
    (-20.601, -7.285, -43.702), (-16.918, -10.788, -43.419), (-13.227, -14.282, -43.489),
    (-9.563, -17.806, -44.481), (-5.941, -21.372, -44.551), (-2.303, -24.923, -43.195),
    (1.415, -28.265, -41.547), (5.195, -31.664, -42.403), (8.916, -35.127, -43.533),
    (12.594, -38.636, -43.746), (16.201, -42.129, -44.598), (19.82, -45.698, -44.598),
    (23.462, -49.244, -43.895), (27.164, -52.725, -42.478), (30.922, -56.148, -42.193),
    (34.695, -59.553, -42.053), (38.468, -62.957, -42.053), (42.242, -66.361, -42.053),
    (45.986, -69.797, -45.208), (49.452, -73.513, -50.408), (52.282, -77.628, -60.28),
    (54.408, -82.329, -70.191), (55.561, -87.262, -85.654), (55.064, -92.301, -102.506),
    (53.455, -97.111, -115.503), (51.002, -101.46, -123.254), (48.17, -105.675, -124.592),
    (45.238, -109.827, -126.078), (42.218, -113.912, -126.638), (39.186, -117.989, -126.638),
    (36.154, -122.067, -126.495), (33.139, -126.159, -126.355), (30.13, -130.256, -126.355),
    (27.108, -134.344, -126.495), (26.959, -134.545, -126.495), (26.959, -134.545, -126.495)
]

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-23.868, 225.205)
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

        # 严格按照截图参数配置静态天气
        RTB.set_static_weather(
            world,
            cloudiness=60.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=10.0,
            sun_azimuth_angle=-1.0,
            sun_altitude_angle=45.0,
            fog_density=3.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 天气系统已按照要求设置。")

        # ==========================================
        # ==========================================
        traj_ego = RTB.clean_trajectory(RAW_TRAJ_AUDI_EGO, min_dist=0.5)
        traj_npc = RTB.clean_trajectory(RAW_TRAJ_IMPALA_NPC, min_dist=0.5)

        # ==========================================
        # 3. 车辆生成
        # ==========================================
        # 提取轨迹起点作为生成点
        ego_start_x, ego_start_y, ego_start_yaw = traj_ego[0]
        npc_start_x, npc_start_y, npc_start_yaw = traj_npc[0]

        ego_vehicle = RTB.spawn_vehicle(
            world, 'vehicle.audi.tt',
            ego_start_x, ego_start_y, yaw=ego_start_yaw,
            role_name='ego', color='0,255,255'
        )
        npc_vehicle = RTB.spawn_vehicle(
            world, 'vehicle.chevrolet.impala',
            npc_start_x, npc_start_y, yaw=npc_start_yaw,
            role_name='npc', color='128,0,128'
        )

        if npc_vehicle: actor_list.append(npc_vehicle)
        if ego_vehicle: actor_list.append(ego_vehicle)

        # ==========================================
        # 4. 车辆控制器与状态管理器配置
        # ==========================================
        # PID 挂载
        pid_lon_npc = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_npc = RTB.PIDLateralController(preset='default_car', dt=dt)

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_ego = RTB.PIDLateralController(preset='default_car', dt=dt)

        # 寻迹指针初始化
        idx_npc, idx_ego = 0, 0

        # 灯光管理系统：要求开启行车灯
        light_npc = RTB.VehicleLightManager(npc_vehicle)
        light_ego = RTB.VehicleLightManager(ego_vehicle)
        light_npc.set_static_lights(low_beam=False, high_beam=False)
        light_ego.set_static_lights(low_beam=False, high_beam=False)

        # ==========================================
        # 5. EGO 复杂剧本状态机编排
        # ==========================================
        # 初始速度 60km/h -> y<30时减速至20 -> y<-2时恢复60
        # Ego 是从 Y=150 往负方向开的，所以选用 'y_less' 触发器最准确
        npc_target_speed = 60.0
        npc_accel_rate_kmh_per_s = 20.0
        npc_decel_rate_kmh_per_s = 25.0

        # NPC 恒定速度状态机 (只需一层)
        ego_cruise_speed = 30.0
        ego_target_speed = ego_cruise_speed
        ego_accel_rate_kmh_per_s = 20.0
        ego_decel_rate_kmh_per_s = 25.0
        ego_stop_trigger_x = -28.076
        ego_stop_started = False
        ego_wait_start_time = None
        ego_stop_completed = False

        # ==========================================
        # 6. 预热与初始速度注入
        # ==========================================
        if ego_vehicle: RTB.set_vehicle_initial_speed(ego_vehicle, target_speed_kmh=30.0)
        if npc_vehicle: RTB.set_vehicle_initial_speed(npc_vehicle, target_speed_kmh=60.0)

        print("🚀 仿真开始运行...")

        # ==========================================
        # 7. 仿真主循环
        # ==========================================
        while True:
            # 记录本帧开始的时间，用于补齐硬件时钟
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ---------------- 环境守护：出界检测与清理 ----------------
            if npc_vehicle and RTB.check_vehicle_out_of_bounds(npc_vehicle, carla_map, auto_destroy=True):
                npc_vehicle = None
            if ego_vehicle and RTB.check_vehicle_out_of_bounds(ego_vehicle, carla_map, auto_destroy=True):
                ego_vehicle = None

            # ---------------- 车辆 1 (NPC) 控制逻辑 ----------------
            if npc_vehicle and npc_vehicle.is_alive:
                v_loc = npc_vehicle.get_location()
                vel = npc_vehicle.get_velocity()
                speed_kmh = 3.6 * math.hypot(vel.x, vel.y)

                # 获取预瞄点
                target_wp_npc, idx_npc = RTB.get_target_waypoint(v_loc, traj_npc, idx_npc, speed_kmh)

                # 获取状态机期望速度，并执行控制
                if v_loc.y < -2.0:
                    npc_desired_speed = 60.0
                elif v_loc.y < 30.0:
                    npc_desired_speed = 20.0
                else:
                    npc_desired_speed = 60.0
                npc_rate = npc_accel_rate_kmh_per_s if npc_desired_speed > npc_target_speed else npc_decel_rate_kmh_per_s
                npc_target_speed = ramp_speed_kmh(npc_target_speed, npc_desired_speed, npc_rate, dt)
                target_speed_npc = npc_target_speed
                if target_wp_npc:
                    RTB.apply_pid_control(npc_vehicle, pid_lon_npc, pid_lat_npc, target_speed_npc, target_wp_npc)

                # 根据刹车与转向自动亮起尾灯
                light_npc.auto_update_from_control()

            # ---------------- 车辆 2 (EGO) 控制逻辑 ----------------
            if ego_vehicle and ego_vehicle.is_alive:
                v_loc = ego_vehicle.get_location()
                vel = ego_vehicle.get_velocity()
                speed_kmh = 3.6 * math.hypot(vel.x, vel.y)

                # 获取预瞄点
                target_wp_ego, idx_ego = RTB.get_target_waypoint(v_loc, traj_ego, idx_ego, speed_kmh)

                # 状态机推进：根据 Y 坐标触发加减速剧本
                if not ego_stop_completed and not ego_stop_started and v_loc.x <= ego_stop_trigger_x:
                    ego_stop_started = True

                if ego_stop_started and not ego_stop_completed:
                    ego_target_speed = ramp_speed_kmh(ego_target_speed, 0.0, ego_decel_rate_kmh_per_s, dt)
                    target_speed_ego = ego_target_speed
                    if ego_wait_start_time is None and speed_kmh <= 0.5 and ego_target_speed <= 0.5:
                        ego_wait_start_time = sim_time
                    if ego_wait_start_time is not None:
                        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
                        if sim_time - ego_wait_start_time >= 3.0:
                            ego_stop_completed = True
                        else:
                            light_ego.auto_update_from_control()
                            continue
                else:
                    ego_target_speed = ramp_speed_kmh(ego_target_speed, ego_cruise_speed, ego_accel_rate_kmh_per_s, dt)
                    target_speed_ego = ego_target_speed
                if target_wp_ego:
                    RTB.apply_pid_control(ego_vehicle, pid_lon_ego, pid_lat_ego, target_speed_ego, target_wp_ego)

                # 根据刹车与转向自动亮起尾灯
                light_ego.auto_update_from_control()

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
