import carla
import time
import math
import numpy as np


# ================= 基础控制算法 (PID) =================
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
        _ie = np.clip(_ie, -2.0, 2.0)
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.8, 0.6)


class PIDLateralController:
    def __init__(self, K_P=1.0, K_I=0.01, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint_loc, vehicle_transform):
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)
        target_vector = np.array([waypoint_loc.x - v_loc.x, waypoint_loc.y - v_loc.y])
        norm = np.linalg.norm(target_vector)
        if norm < 0.1: return 0.0
        target_yaw = math.atan2(target_vector[1], target_vector[0])
        error = target_yaw - v_yaw
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.7, 0.7)


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed_kmh, target_loc):
    target_speed_ms = target_speed_kmh / 3.6
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    current_speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed_ms, current_speed_ms)
    steer_output = pid_lat.run_step(target_loc, tf)

    if abs(steer_output) < 0.02: steer_output = 0.0
    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)
    vehicle.apply_control(control)



def check_and_handle_out_of_bounds(vehicle, carla_map):
    loc = vehicle.get_location()

    # 强制将坐标投影到最近的合法路面上（忽略高度/细微边界误差）
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)

    # 如果整个地图都找不到投影点（通常不可能，除非飞出世界边缘）
    if wp_nearest is None:
        print(f"[{vehicle.type_id}] 彻底脱离地图，被销毁！")
        vehicle.destroy()
        return True

    # 计算车辆当前物理位置与路网中心点的绝对距离
    distance = wp_nearest.transform.location.distance(loc)

    # 距离大于 6 米才算真正出界（相当于偏离道路中心线两条车道以上）
    if distance > 6.0:
        print(f"[{vehicle.type_id}] 偏离道路中心 {distance:.2f} 米，判定出界被销毁！")
        vehicle.destroy()
        return True

    return False

def apply_initial_velocity(vehicle, speed_kmh, yaw_degrees):
    speed_ms = speed_kmh / 3.6
    yaw_rad = math.radians(yaw_degrees)
    vx = speed_ms * math.cos(yaw_rad)
    vy = speed_ms * math.sin(yaw_rad)
    vehicle.set_target_velocity(carla.Vector3D(x=vx, y=vy, z=0.0))


# ================= 轨迹数据 (原卡车轨迹提供给警车) =================
POLICE_TRAJECTORY = [
    (86.177, 33.616, -109.583), (83.869, 27.148, -109.652), (81.354, 20.087, -109.582),
    (78.933, 12.99, -107.505), (76.669, 5.591, -106.431), (74.575, -1.832, -105.362),
    (72.596, -9.079, -105.649), (70.573, -16.282, -105.649), (68.576, -23.258, -106.075),
    (66.282, -30.598, -110.242), (63.332, -37.582, -117.579), (59.454, -44.129, -124.437),
    (54.909, -50.238, -129.113), (50.078, -56.113, -130.4), (44.862, -61.841, -134.099),
    (39.496, -67.072, -137.835), (33.741, -72.057, -140.039), (27.634, -76.821, -142.939),
    (21.426, -81.461, -143.362), (15.315, -86.022, -142.797), (9.341, -90.558, -142.797),
    (3.166, -95.245, -142.797), (-3.017, -99.919, -143.079), (-9.113, -104.5, -143.079),
    (-15.116, -108.998, -143.362), (-21.244, -113.536, -143.502), (-27.348, -118.106, -143.009),
    (-33.54, -122.771, -142.939), (-33.839, -122.997, -142.87), (-33.839, -122.997, -142.87)
]


# ================= 主程序 =================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 极端天气设置（积水、大雾等）
    weather = carla.WeatherParameters(
        cloudiness=40.0, precipitation=100.0, precipitation_deposits=100.0,
        wind_intensity=100.0, sun_azimuth_angle=90.0, sun_altitude_angle=10.0,
        fog_density=10.0, fog_distance=0.75, fog_falloff=0.1, wetness=100.0,
        scattering_intensity=11.5, mie_scattering_scale=0.21, rayleigh_scattering_scale=0.07
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        # 准备 PID 控制器
        pid_police = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # # ================= 场景构建：生成超低摩擦力积水区 =================
        # bp_friction = bp_lib.find('static.trigger.friction')
        # # 0.1代表极度湿滑，几乎像冰面
        # bp_friction.set_attribute('friction', '0.1')
        # # extend 是半长/半宽，设为10代表生成一个 20x20 米的打滑区域
        # bp_friction.set_attribute('extent_x', '10.0')
        # bp_friction.set_attribute('extent_y', '10.0')
        # bp_friction.set_attribute('extent_z', '2.0')
        #
        # # 在要求的坐标生成打滑区 (z 稍微抬高避免没检测到)
        # friction_loc = carla.Location(x=65.293, y=-29.102, z=0.5)
        # friction_trigger = world.try_spawn_actor(bp_friction, carla.Transform(friction_loc))
        # if friction_trigger:
        #     actor_list.append(friction_trigger)
        #     print("生成摩擦力触发器（积水打滑区）成功。")

        # ================= 场景构建：生成超低摩擦力积水区 =================
        bp_friction = bp_lib.find('static.trigger.friction')
        bp_friction.set_attribute('friction', '0.0')

        # 生成 3x3 米的水坑
        bp_friction.set_attribute('extent_x', '10.0')
        bp_friction.set_attribute('extent_y', '10.0')
        bp_friction.set_attribute('extent_z', '10.0')

        # 将坐标偏移到侧边车轮能压到的地方
        friction_loc = carla.Location(x=65.293, y=-28.500, z=-5)
        friction_trigger = world.try_spawn_actor(bp_friction, carla.Transform(friction_loc))

        if friction_trigger:
            actor_list.append(friction_trigger)
            print("生成摩擦力触发器（单侧积水打滑区）成功。")

            # === 修复的绘图代码 ===
            box = carla.BoundingBox(friction_loc, carla.Vector3D(10.0, 10.0, 10.0))
            world.debug.draw_box(
                box=box,
                rotation=friction_trigger.get_transform().rotation,
                thickness=0.1,
                color=carla.Color(r=255, g=0, b=0),
                life_time=100.0
            )


        # ================= Actor 1：警车 (代替原卡车) =================
        bp_police_car = bp_lib.find('vehicle.dodge.charger_police_2020')

        police_start_x, police_start_y, police_start_yaw = POLICE_TRAJECTORY[0]
        police_loc = carla.Location(x=police_start_x, y=police_start_y, z=0.5)
        police_loc.z = carla_map.get_waypoint(police_loc).transform.location.z + 0.5
        police_car = world.try_spawn_actor(bp_police_car,
                                           carla.Transform(police_loc, carla.Rotation(yaw=police_start_yaw)))

        police_active = False
        if police_car:
            actor_list.append(police_car)
            police_active = True

            # 开启警车的大灯阵列，制造炫目效果
            light_state = carla.VehicleLightState.HighBeam | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position | carla.VehicleLightState.Special1
            police_car.set_light_state(carla.VehicleLightState(light_state))
            print("生成 Dodge Charger 警车成功，大灯和警灯已开启。")

        # ================= Actor 2：Audi TT (Ego) =================
        bp_audi = bp_lib.find('vehicle.audi.tt')
        if bp_audi.has_attribute('color'):
            bp_audi.set_attribute('color', '255,165,0')  # 橙色

        audi_start_loc = carla.Location(x=17.659, y=-78.859, z=0.5)
        audi_start_wp = carla_map.get_waypoint(audi_start_loc, project_to_road=True)
        audi_start_loc.z = audi_start_wp.transform.location.z + 0.5
        audi_start_yaw = audi_start_wp.transform.rotation.yaw
        audi = world.try_spawn_actor(bp_audi, carla.Transform(audi_start_loc, audi_start_wp.transform.rotation))

        if audi:
            actor_list.append(audi)
            print("生成 Audi TT (Ego) 成功。")

        # ================= 稳定系统与初始速度 =================
        print("等待物理系统预热并稳定车辆底盘...")
        for _ in range(20):
            world.tick()

        print("赋予两车 50km/h 的初始速度...")
        if police_car: apply_initial_velocity(police_car, 50.0, police_start_yaw)
        if audi: apply_initial_velocity(audi, 50.0, audi_start_yaw)

        world.tick()

        print("\n仿真正式开始！")
        police_traj_idx = 0

        while True:
            start_time = time.time()
            world.tick()

            # ================= 警车：PID寻路 =================
            if police_active and police_car.is_alive:
                if check_and_handle_out_of_bounds(police_car, carla_map):
                    police_active = False
                elif police_traj_idx < len(POLICE_TRAJECTORY):
                    tx, ty, tyaw = POLICE_TRAJECTORY[police_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=police_car.get_location().z)

                    if police_car.get_location().distance(target_loc) < 3.0 and police_traj_idx < len(
                            POLICE_TRAJECTORY) - 1:
                        police_traj_idx += 1

                    # 警车也是50km/h，驶入摩擦力为0.1的触发器区域时，会产生剧烈打滑甩尾
                    apply_pid_control(police_car, pid_police['lon'], pid_police['lat'], 50.0, target_loc)
                else:
                    police_car.apply_control(carla.VehicleControl(brake=1.0))
                    police_active = False

            # ================= Audi (Ego): PID动态车道保持 =================
            if audi and audi.is_alive:
                if not check_and_handle_out_of_bounds(audi, carla_map):
                    ego_loc = audi.get_location()
                    current_wp = carla_map.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                    next_wps = current_wp.next(10.0)

                    if next_wps:
                        ego_target_loc = next_wps[0].transform.location
                        # Ego 也将面临打滑考验与警车的强光炫目干扰
                        apply_pid_control(audi, pid_ego['lon'], pid_ego['lat'], 50.0, ego_target_loc)
                    else:
                        audi.apply_control(carla.VehicleControl(brake=1.0))

            # 帧率同步
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("清理完毕。")


if __name__ == '__main__':
    main()