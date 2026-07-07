import carla
import time
import math
import numpy as np
import sys
import os

# 引入全局路由规划器
try:
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'carla'))
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except ImportError:
    print("警告: 无法导入 agents.navigation。请确保环境变量配置正确。")


# ==========================================
# 1. 基础控制算法 (PID) - 已优化
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        # 增加积分限幅，防止积分饱和
        _ie = np.clip(_ie, -2.0, 2.0)
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController:
    def __init__(self, K_P=1.0, K_I=0.01, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint_loc, vehicle_transform):
        # 【优化】使用航向角(Yaw)差值代替原先的点积acos运算，极大地减小了数值抖动
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)

        # 计算目标点相对于车辆的方位角
        target_vector = np.array([waypoint_loc.x - v_loc.x, waypoint_loc.y - v_loc.y])
        norm = np.linalg.norm(target_vector)
        if norm < 0.1: return 0.0

        target_yaw = math.atan2(target_vector[1], target_vector[0])

        # 计算角度差，并归一化到 [-pi, pi]
        error = target_yaw - v_yaw
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi

        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed, target_wp_loc):
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed, speed)
    steer_output = pid_lat.run_step(target_wp_loc, tf)

    # 【优化保留】抑制微小抖动，因为算法优化，死区可以从0.1大幅降低到0.02
    if abs(steer_output) < 0.02:
        steer_output = 0.0

    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)
    vehicle.apply_control(control)


# ==========================================
# 2. 辅助函数：避障与寻迹
# ==========================================
def check_obstacle_in_front(check_loc, check_fwd, actor_list, ego_id, safe_distance=12.0):
    """检测指定位置前方是否有障碍物，返回是否碰撞及障碍物对象"""
    for actor in actor_list:
        if actor.id == ego_id or not isinstance(actor, carla.Vehicle) or not actor.is_alive:
            continue
        act_loc = actor.get_location()
        dist = check_loc.distance(act_loc)

        if dist < safe_distance:
            dir_vec = act_loc - check_loc
            dir_vec = dir_vec / dist
            dot = check_fwd.x * dir_vec.x + check_fwd.y * dir_vec.y
            if dot > 0.866:  # 前方60度视角内
                return True, actor
    return False, None


def get_straightest_waypoint(current_wp, distance=5.0):
    """【增强】绝对直行逻辑，通过比对Yaw角度差确保直行"""
    next_wps = current_wp.next(distance)
    if not next_wps: return None
    if len(next_wps) == 1: return next_wps[0]

    best_wp = next_wps[0]
    min_yaw_diff = float('inf')
    curr_yaw = current_wp.transform.rotation.yaw

    for wp in next_wps:
        wp_yaw = wp.transform.rotation.yaw
        # 计算角度差
        diff = abs(curr_yaw - wp_yaw)
        while diff > 180.0: diff = abs(diff - 360.0)

        if diff < min_yaw_diff:
            min_yaw_diff = diff
            best_wp = wp
    return best_wp


def get_dynamic_right_turn_waypoint(current_wp, distance=3.0):
    next_wps = current_wp.next(distance)
    if not next_wps: return None
    if len(next_wps) == 1: return next_wps[0]

    best_wp = next_wps[0]
    max_right_dot = -1.0
    right_vec = current_wp.transform.get_right_vector()

    for wp in next_wps:
        wp_fwd = wp.transform.get_forward_vector()
        dot = right_vec.x * wp_fwd.x + right_vec.y * wp_fwd.y
        if dot > max_right_dot:
            max_right_dot = dot
            best_wp = wp
    return best_wp


# ==========================================
# 3. 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 设置精准天气
    weather = carla.WeatherParameters(
        cloudiness=20.0, precipitation=0.0, precipitation_deposits=5.0, wind_intensity=5.0,
        sun_azimuth_angle=240.0, sun_altitude_angle=22.0, fog_density=4.0, fog_distance=0.0,
        fog_falloff=0.0, wetness=5.0, scattering_intensity=0.5, mie_scattering_scale=0.1,
        rayleigh_scattering_scale=0.3, dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    # 车辆存活标志字典，方便随时释放车辆
    active_vehicles = {'v1': False, 'v2': False, 'v3': False, 'ego': False}

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pids = {
            'v2': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'v3': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'ego': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        }

        LIGHT_LOW_BEAM = carla.VehicleLightState.LowBeam
        LIGHT_HIGH_BEAM = carla.VehicleLightState.HighBeam
        LIGHT_BLINKER_RIGHT = carla.VehicleLightState.RightBlinker
        LIGHT_HAZARD = carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.Position
        LIGHT_POSITION = carla.VehicleLightState.Position

        # ==========================================
        # 车辆 1: 特斯拉 Model 3 (静止)
        # ==========================================
        bp_v1 = bp_lib.find('vehicle.tesla.model3')
        loc_v1 = carla.Location(x=1.0, y=10.975, z=0.5)
        v1 = world.try_spawn_actor(bp_v1, carla.Transform(loc_v1, carla_map.get_waypoint(loc_v1).transform.rotation))
        if v1:
            actor_list.append(v1)
            active_vehicles['v1'] = True
            v1.set_light_state(carla.VehicleLightState(LIGHT_HAZARD))
            v1.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            print("V1 (Tesla) 生成成功: 静止打双闪")

        # ==========================================
        # 车辆 2: 奥迪 TT (右转)
        # ==========================================
        bp_v2 = bp_lib.find('vehicle.audi.tt')
        if bp_v2.has_attribute('color'): bp_v2.set_attribute('color', '255,165,0')
        loc_v2 = carla.Location(x=5.089, y=66.621, z=0.5)
        v2_wp = carla_map.get_waypoint(loc_v2)
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(loc_v2, v2_wp.transform.rotation))
        if v2:
            actor_list.append(v2)
            active_vehicles['v2'] = True
            v2.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_LOW_BEAM))
            print("V2 (Audi TT) 生成成功: 橙色，PID控制准备右转")

        # ==========================================
        # 车辆 3: 警车 (严格直行)
        # ==========================================
        bp_v3 = bp_lib.find('vehicle.dodge.charger_police')
        loc_v3 = carla.Location(x=61.362, y=-1.89, z=0.5)
        v3_wp = carla_map.get_waypoint(loc_v3)
        v3 = world.try_spawn_actor(bp_v3, carla.Transform(loc_v3, v3_wp.transform.rotation))
        if v3:
            actor_list.append(v3)
            active_vehicles['v3'] = True
            v3.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_LOW_BEAM))
            print("V3 (Police) 生成成功: 警车，严格直行，远光灯闪烁")

        # ==========================================
        # 车辆 Ego (原v4): 林肯 MKZ 2020 (寻迹避障变道)
        # ==========================================
        bp_ego = bp_lib.find('vehicle.lincoln.mkz_2020')
        bp_ego.set_attribute('role_name', 'ego')
        loc_ego_start = carla.Location(x=-53.962, y=2.019, z=0.5)
        loc_ego_mid = carla.Location(x=1.312, y=1.801, z=0.5)
        loc_ego_end = carla.Location(x=0.429, y=70.616, z=0.5)

        ego_wp = carla_map.get_waypoint(loc_ego_start)
        ego = world.try_spawn_actor(bp_ego, carla.Transform(loc_ego_start, ego_wp.transform.rotation))
        ego_route = []
        if ego:
            actor_list.append(ego)
            active_vehicles['ego'] = True
            ego.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_HIGH_BEAM))
            print("Ego (Lincoln) 生成成功: 执行三点路径规划，具备变道避障能力")

            grp = GlobalRoutePlanner(carla_map, 2.0)
            route1 = grp.trace_route(loc_ego_start, loc_ego_mid)
            route2 = grp.trace_route(loc_ego_mid, loc_ego_end)
            ego_route = [wp[0] for wp in route1] + [wp[0] for wp in route2]
            ego_route_idx = 0

        print("\n车辆加载完毕，等待物理稳定...")
        for _ in range(20): world.tick()
        print("仿真正式开始...")

        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            # ==========================
            # V2 控制: 动态寻迹 + 自动右转打灯 + 【出界销毁】
            # ==========================
            if active_vehicles['v2'] and v2.is_alive:
                lookahead_wps = v2_wp.next(8.0)
                if not lookahead_wps:
                    print("V2 驶出路网，自动销毁释放资源")
                    v2.destroy()
                    active_vehicles['v2'] = False
                else:
                    lookahead_wp = lookahead_wps[0]
                    next_options = lookahead_wp.next(8.0)

                    if next_options and len(next_options) > 1:
                        target_wp = get_dynamic_right_turn_waypoint(lookahead_wp)
                        v2.set_light_state(
                            carla.VehicleLightState(LIGHT_POSITION | LIGHT_LOW_BEAM | LIGHT_BLINKER_RIGHT))
                    else:
                        target_wp = next_options[0] if next_options else lookahead_wp

                    v2_wp = carla_map.get_waypoint(v2.get_location())
                    apply_pid_control(v2, pids['v2']['lon'], pids['v2']['lat'], 25.0, target_wp.transform.location)

            # ==========================
            # V3 控制: 绝对直行 + 闪灯 + 【出界销毁】
            # ==========================
            if active_vehicles['v3'] and v3.is_alive:
                target_wp = get_straightest_waypoint(v3_wp, distance=8.0)
                if not target_wp:
                    print("V3 驶出路网，自动销毁释放资源")
                    v3.destroy()
                    active_vehicles['v3'] = False
                else:
                    v3_wp = carla_map.get_waypoint(v3.get_location())
                    apply_pid_control(v3, pids['v3']['lon'], pids['v3']['lat'], 30.0, target_wp.transform.location)

                    # 闪灯逻辑
                    cycle = sim_time % 2.0
                    is_flashing = True if (cycle < 0.2 or 0.4 <= cycle < 0.6 or 0.8 <= cycle < 1.0) else False
                    if is_flashing:
                        v3.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_HIGH_BEAM))
                    else:
                        v3.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_LOW_BEAM))

            # ==========================
            # Ego 控制: 高阶避障 (上等方案:变道 / 下等方案:ABS+双闪)
            # ==========================
            if active_vehicles['ego'] and ego.is_alive:
                if ego_route_idx < len(ego_route):
                    ego_tf = ego.get_transform()
                    ego_loc = ego_tf.location

                    # 更新当前目标点
                    while ego_route_idx < len(ego_route) - 1:
                        if ego_loc.distance(ego_route[ego_route_idx].transform.location) < 3.0:
                            ego_route_idx += 1
                        else:
                            break
                    target_wp_loc = ego_route[ego_route_idx].transform.location

                    # 前方障碍物检测 (距离缩减到 15 米探测)
                    has_obs, _ = check_obstacle_in_front(ego_loc, ego_tf.get_forward_vector(), actor_list, ego.id,
                                                         safe_distance=15.0)

                    if has_obs:
                        # 尝试上等方案：变道避障
                        curr_wp = carla_map.get_waypoint(ego_loc)
                        left_wp = curr_wp.get_left_lane()  # 获取左侧车道 (即使是对向车道)

                        can_change_lane = False
                        if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                            # 检查左侧车道前方是否安全
                            left_safe, _ = check_obstacle_in_front(left_wp.transform.location,
                                                                   left_wp.transform.get_forward_vector(), actor_list,
                                                                   ego.id, safe_distance=20.0)
                            if not left_safe:
                                can_change_lane = True

                        if can_change_lane:
                            print("Ego: 前方受阻，执行上等方案 -> 自动变道至左侧车道避障")
                            # 重新规划路线：从当前左侧车道点出发，前往终点
                            route_new = grp.trace_route(left_wp.transform.location, loc_ego_end)
                            ego_route = [wp[0] for wp in route_new]
                            ego_route_idx = 0
                            # 继续行驶
                            apply_pid_control(ego, pids['ego']['lon'], pids['ego']['lat'], 35.0,
                                              ego_route[0].transform.location)
                        else:
                            # 下等方案：无路可变，ABS 紧急刹车启动 + 开启双闪与刹车灯
                            print("Ego: 变道空间不足，执行下等方案 -> 紧急刹车并开启双闪")
                            ego.set_light_state(carla.VehicleLightState(LIGHT_HAZARD))  # Carla 刹车>0 会自动亮刹车灯
                            ego.apply_control(carla.VehicleControl(brake=1.0, steer=0.0, hand_brake=False))
                    else:
                        # 正常行驶，恢复远光灯
                        ego.set_light_state(carla.VehicleLightState(LIGHT_POSITION | LIGHT_HIGH_BEAM))
                        apply_pid_control(ego, pids['ego']['lon'], pids['ego']['lat'], 35.0, target_wp_loc)
                else:
                    # 路线跑完
                    print("Ego 到达终点，自动销毁释放资源")
                    ego.destroy()
                    active_vehicles['ego'] = False

            # 保持实时帧率同步
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()
        print("清理完毕。")


if __name__ == '__main__':
    main()
