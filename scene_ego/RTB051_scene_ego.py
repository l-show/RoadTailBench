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
# 轨迹数据硬编码区
# ==========================================
RAW_TRAJ_1 = """
-1.475 -58.462 88.77
-1.455 -57.714 88.417
-1.339 -52.556 89.276
-1.353 -47.394 90.411
-1.435 -42.313 91.041
-1.539 -37.149 91.393
-1.658 -32.151 91.183
-1.703 -26.985 89.985
-1.696 -21.901 89.775
-1.666 -16.735 89.635
-1.636 -11.57 89.565
-1.603 -6.487 89.635
-1.571 -1.405 89.635
-1.538 3.766 89.705
-1.572 8.934 90.98
-1.657 13.933 90.98
-1.716 19.099 90.131
-1.72 24.099 89.781
-1.678 29.182 89.498
-1.633 34.349 89.498
-1.579 39.515 89.358
-1.522 44.598 89.358
-1.465 49.681 89.498
-1.44 54.681 89.781
-1.425 59.847 89.99
-1.432 64.931 90.27
-1.464 70.098 90.553
-1.513 75.181 90.553
-1.568 80.347 90.693
-1.629 85.346 90.693
-1.69 90.513 90.623
-1.745 95.52 90.623
-1.747 100.603 89.834
-1.725 105.768 89.269
-1.578 110.847 86.857
-1.132 115.828 82.294
-0.151 120.897 74.668
1.777 125.491 58.123
5.218 129.194 38.089
9.597 131.909 24.563
14.466 133.614 14.261
19.489 134.327 2.941
24.479 134.107 -5.772
29.62 133.588 -5.772
34.763 133.105 -4.136
39.833 132.736 -4.276
44.978 132.276 -6.117
50.031 131.72 -6.399
53.344 131.348 -6.259
"""

RAW_TRAJ_EGO = """
43.671 128.383 168.989
43.1 128.494 168.989
40.565 128.957 170.91
38.052 129.33 171.98
35.493 129.683 172.403
32.927 129.981 173.745
30.4 130.256 173.885
27.913 130.511 174.308
25.342 130.765 174.66
22.766 130.946 177.369
20.226 131.007 -179.848
17.647 130.881 -174.118
15.179 130.497 -167.153
12.707 129.76 -158.909
10.4 128.609 -148.307
8.371 127.087 -137.773
6.554 125.313 -133.328
5 123.257 -121.557
3.796 121.021 -115.354
2.889 118.694 -107.334
2.264 116.191 -100.303
1.912 113.633 -94.905
1.76 111.096 -92.919
1.672 108.514 -90.838
1.69 105.973 -88.913
1.738 103.431 -88.913
1.787 100.89 -88.913
1.836 98.307 -88.913
1.885 95.724 -88.913
1.929 93.182 -89.196
1.965 90.641 -89.196
2 88.13 -89.196
2.01 87.38 -89.196
2.005 85.838 -90.477
1.962 83.297 -91.537
1.869 80.717 -92.178
1.785 78.177 -91.114
1.752 75.595 -90.689
1.721 73.054 -90.689
1.69 70.471 -90.689
1.659 67.888 -90.689
1.607 63.369 -90.334
1.708 58.203 -89.378
1.695 53.037 -90.594
1.646 48.037 -90.312
1.619 43.037 -90.312
1.604 38.037 -90.102
1.598 33.037 -89.819
1.633 27.954 -89.321
1.702 22.788 -89.038
1.787 17.622 -89.108
1.852 12.539 -89.533
1.868 7.372 -89.886
1.871 2.281 -90.166
1.856 -2.886 -90.166
1.842 -7.969 -90.166
1.821 -13.136 -90.378
1.787 -18.303 -90.378
1.753 -23.388 -90.378
1.732 -28.555 -89.958
1.758 -33.722 -89.609
1.802 -38.805 -89.468
1.85 -43.971 -89.468
1.895 -49.138 -89.678
1.906 -54.221 -89.961
1.91 -59.388 -89.961
1.913 -64.555 -89.961
1.927 -69.64 -89.608
1.961 -74.63 -89.608
1.996 -79.796 -89.678
"""

# ==========================================
# 🚨 终极修复：彻底分离 V3 的倒车轨迹与正向轨迹
# ==========================================
RAW_TRAJ_3_REV = """
10.849 75.429 -2.683
8.564 75.578 -7.7
6.173 76.38 -31.379
4.315 78.087 -52.551
3.029 80.313 -69.938
2.401 82.772 -77.967
2.071 85.329 -88.696
2.158 87.906 -93.055
2.171 88.156 -93.055
"""

RAW_TRAJ_3_FWD = """
2.171 88.156 -93.055
1.858 83.298 -95.154
1.238 75.706 -93.878
0.909 68.089 -91.088
0.851 60.339 -89.531
1.014 52.591 -87.978
1.263 44.97 -88.613
1.298 37.313 -90.488
1.235 29.563 -90.418
1.25 21.812 -89.351
1.363 14.188 -89.139
1.47 6.438 -89.279
1.604 -1.061 -88.783
1.765 -8.809 -89.066
1.846 -16.559 -89.558
1.901 -24.183 -89.698
1.909 -31.934 -90.264
1.865 -39.683 -90.334
1.821 -47.308 -90.334
1.776 -55.058 -90.334
1.731 -62.682 -90.334
1.686 -70.434 -90.334
1.641 -78.183 -90.334
1.597 -85.808 -90.334
1.552 -93.559 -90.334
1.508 -101.059 -90.334
1.463 -108.684 -90.334
1.426 -115.111 -90.334
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (1.996 , -79.796)
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

        weather = carla.WeatherParameters(
            cloudiness=0.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_azimuth_angle=-1.0, sun_altitude_angle=45.0,
            fog_density=2.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
            scattering_intensity=1.0, mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已设置。")

        # ==========================================
        # 2. 轨迹数据解析、清洗与稠密化插值 (精度 0.5m)
        # ==========================================
        print("[场景配置] 正在解析并稠密化轨迹数据...")
        parsed_v1 = RTB.parse_string_trajectory(RAW_TRAJ_1)
        traj_v1 = RTB.interpolate_trajectory(parsed_v1, interval=0.5)

        parsed_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO)
        traj_ego = RTB.interpolate_trajectory(parsed_ego, interval=0.5)

        # 分别解析 V3 的两段轨迹
        parsed_v3_rev = RTB.parse_string_trajectory(RAW_TRAJ_3_REV)
        traj_v3_rev = RTB.interpolate_trajectory(parsed_v3_rev, interval=0.5)

        parsed_v3_fwd = RTB.parse_string_trajectory(RAW_TRAJ_3_FWD)
        traj_v3_fwd = RTB.interpolate_trajectory(parsed_v3_fwd, interval=0.5)

        # ==========================================
        # 3. 车辆生成与初始速度注入
        # ==========================================
        v1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=traj_v1[0][0], y=traj_v1[0][1], yaw=traj_v1[0][2])
        actor_list.append(v1)
        RTB.set_vehicle_initial_speed(v1, target_speed_kmh=60.0, yaw_deg=traj_v1[0][2])

        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=traj_ego[0][0], y=traj_ego[0][1], yaw=traj_ego[0][2],
                                role_name='ego')
        actor_list.append(ego)
        RTB.set_vehicle_initial_speed(ego, target_speed_kmh=60.0, yaw_deg=traj_ego[0][2])

        # V3 绝对坐标无吸附生成 (因为起点在非标准车道)
        print("[场景配置] 正在对 V3 进行无吸附原生坐标生成...")
        bp_v3 = bp_lib.find('vehicle.chevrolet.impala')
        spawn_tf_v3 = carla.Transform(carla.Location(x=traj_v3_rev[0][0], y=traj_v3_rev[0][1], z=0.5),
                                      carla.Rotation(yaw=traj_v3_rev[0][2]))
        v3 = world.try_spawn_actor(bp_v3, spawn_tf_v3)
        if v3:
            actor_list.append(v3)
            RTB.set_vehicle_initial_speed(v3, target_speed_kmh=0.0)

        # ==========================================
        # 4. 控制器挂载与灯光管理器
        # ==========================================
        pid_lon_v1 = RTB.PIDLongitudinalController()
        pid_lat_v1 = RTB.PIDLateralController()

        pid_lon_ego = RTB.PIDLongitudinalController()
        pid_lat_ego = RTB.PIDLateralController()

        pid_lon_v3 = RTB.PIDLongitudinalController()
        pid_lat_v3 = RTB.PIDLateralController()

        ego_lights = RTB.VehicleLightManager(ego)
        ego_lights.set_static_lights(low_beam=True, high_beam=False)

        # ==========================================
        # 5. 剧本状态机编排
        # ==========================================
        # Ego 剧本
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        ego_sm.add_stage(trigger_type='y_less', trigger_val=118.0, target_speed=0.0, accel=35.0)
        ego_sm.add_stage(trigger_type='time', trigger_val=8.0, target_speed=60.0, accel=15.0)

        # 🚀 [修改处] V3 剧本：原地等待4s -> 25km/h 倒车 -> 终点刹停(容差3.5m) -> 等2s -> 正向60km/h
        v3_sm = RTB.MultiStageBehaviorMachine(initial_speed=0.0)  # 初始速度改为 0

        # 新增阶段 0：原地等待 4 秒，然后提速到 25km/h 开始倒车
        v3_sm.add_stage(trigger_type='time', trigger_val=4.0, target_speed=25.0, accel=15.0)

        # 阶段 1：倒车至目标点，彻底刹停
        v3_sm.add_stage(trigger_type='point', trigger_val=(2.171, 88.156), target_speed=0.0, accel=30.0, tolerance=3.5)

        # 阶段 2：驻车等待 2 秒换挡，然后提速到 60km/h 正向驶出
        v3_sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=60.0, accel=15.0)

        idx_v1, idx_ego, idx_v3 = 0, 0, 0
        v3_is_reversing = True
        sim_time = 0.0

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        print("[RoadTailBench] 🚀 仿真正式开始...")
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            ego_lights.auto_update_from_control()

            # 🚨 守护：完全不对 V3 进行出界判定
            if v1 and v1.is_alive and RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True): v1 = None
            if ego and ego.is_alive and RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True): ego = None

            # ----------------------------------------
            # 车辆 1 & Ego (正常循迹)
            # ----------------------------------------
            if v1 and v1.is_alive:
                vel_v1 = v1.get_velocity()
                spd_v1 = 3.6 * math.hypot(vel_v1.x, vel_v1.y)
                wp_v1, idx_v1 = RTB.get_target_waypoint(v1.get_location(), traj_v1, idx_v1, speed_kmh=spd_v1)
                RTB.apply_pid_control(v1, pid_lon_v1, pid_lat_v1, target_speed_kmh=60.0, target_wp=wp_v1)

            if ego and ego.is_alive:
                vel_ego = ego.get_velocity()
                spd_ego = 3.6 * math.hypot(vel_ego.x, vel_ego.y)
                ego_target_spd = ego_sm.tick(ego.get_location(), sim_time, dt)
                wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego, speed_kmh=spd_ego)

                # 画出 Ego 预瞄点与牵引线
                RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_speed_kmh=ego_target_spd, target_wp=wp_ego)

            # ----------------------------------------
            # 车辆 3 (V3) - 独立的双轨迹无缝切换控制
            # ----------------------------------------
            if v3 and v3.is_alive:
                vel_v3 = v3.get_velocity()
                spd_v3 = 3.6 * math.hypot(vel_v3.x, vel_v3.y)
                v3_target_spd = v3_sm.tick(v3.get_location(), sim_time, dt)

                # ========================================================
                # 核心大招：模式切换与轨迹池切换
                # ========================================================
                if v3_sm.current_idx >= 3 and v3_is_reversing:
                    v3_is_reversing = False
                    print("[RoadTailBench] ✅ V3 等待结束，切换为正向专属轨迹！")

                    # 1. 重置 PID：丢掉倒车时积累的反向历史积分包袱，防止暴走
                    pid_lon_v3 = RTB.PIDLongitudinalController()
                    pid_lat_v3 = RTB.PIDLateralController()

                    # 2. 轨迹强制替换：清零索引，彻底脱离倒车轨迹，开始寻址前方的正向轨迹
                    idx_v3 = 0

                # 动态选择当前正在使用的轨迹列表
                current_traj_v3 = traj_v3_rev if v3_is_reversing else traj_v3_fwd

                # 获取预瞄点 (此时两段轨迹物理上完全独立，绝不会出现前后重合卡死)
                wp_v3, idx_v3 = RTB.get_target_waypoint(v3.get_location(), current_traj_v3, idx_v3, speed_kmh=spd_v3)

                if v3_is_reversing:
                    # [倒车模式：物理欺骗控制]
                    tf_v3 = v3.get_transform()
                    tf_flipped = carla.Transform(tf_v3.location, carla.Rotation(pitch=tf_v3.rotation.pitch,
                                                                                yaw=tf_v3.rotation.yaw + 180.0,
                                                                                roll=tf_v3.rotation.roll))

                    throttle = pid_lon_v3.run_step(v3_target_spd / 3.6, spd_v3 / 3.6)

                    # 停车防抽搐：速度为 0 时锁死方向盘不乱打
                    if v3_target_spd <= 0.1:
                        steer = 0.0
                    else:
                        steer = -pid_lat_v3.run_step(wp_v3, tf_flipped)

                    control = carla.VehicleControl(steer=steer, reverse=True)

                    if throttle >= 0.0:
                        control.throttle = throttle;
                        control.brake = 0.0
                    else:
                        control.throttle = 0.0;
                        control.brake = abs(throttle)

                    # 彻底驻车刹停
                    if v3_target_spd <= 0.1:
                        control.throttle = 0.0;
                        control.brake = 1.0

                    v3.apply_control(control)

                else:
                    # [正向模式：标准函数库接管]
                    RTB.apply_pid_control(v3, pid_lon_v3, pid_lat_v3, target_speed_kmh=v3_target_spd, target_wp=wp_v3)

            # ---------------- 硬件时钟补齐 ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()