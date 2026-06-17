# -*- coding: utf-8 -*-
"""
本场景发生在一个黄昏时段的城市郊区无路灯的T形路口，太阳接近落山，低角度红色炫光与路口人工信号灯处于近似同一方向，共同进入主车前向视野，对主车的信号灯识别和道路边界感知造成强烈干扰；路口处铺满大量枫树叶，交叉口内部的道路边界、车道线以及可通行区域轮廓被大面积遮挡，同时主车进口方向的停止线缺失，使车辆难以通过地面标线判断准确的停车位置和路口边界。场景开始时，主车以约 35 km/h 从主路一侧直行驶向路口，前方人工信号灯处于异常状态，红灯、黄灯和绿灯同时点亮；由于黄昏太阳的红色炫光与信号灯方向重叠，红黄绿三色灯光在雾气、湿润路面和枫叶反射作用下形成混杂光晕，使主车更难可靠判断信号含义。当主车进一步接近停止决策区并进入触发区域后，异常信号灯突然从三灯全亮切换为仅绿灯亮，主车在停止线缺失、枫叶遮挡车道线和太阳炫光干扰的共同作用下，将该绿灯误判为直行放行信号，继续沿主路进入交叉口；与此同时，背景车从支路方向以约 35 km/h 释放并驶入路口，随后转向主路方向，其行驶轨迹与主车直行轨迹在枫叶覆盖最密集、低附着风险最高的交叉口中心区域发生时空重叠。由于枫叶覆盖区域被设定为低摩擦路面，主车即使在最后阶段识别到冲突，也可能因制动距离增加和转向响应变差而难以及时避让，最终形成由“黄昏逆光炫光、信号灯异常显示、停止线缺失、车道边界遮挡、低附着枫叶路面以及支路车辆转入冲突”共同诱发的高风险动态长尾场景。
"""

import sys
import time
import math
import carla


# ============================================================
# 0. 引入标准化函数库
# ============================================================

LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

import RoadTailBenchInitV9 as RTB


# ============================================================
# 1. 全局参数
# ============================================================

DT = 0.05

EGO_INITIAL_SPEED_KMH = 35.0
EGO_TARGET_SPEED_KMH = 35.0

BG_INITIAL_SPEED_KMH = 35.0
BG_TARGET_SPEED_KMH = 35.0

# 背景车释放距离：数值越大，背景车越早启动
BG_RELEASE_DISTANCE = 48.0

# 枫叶覆盖区域低摩擦系数
FRICTION_VALUE = 0.50

# 如果希望真实碰撞，保持 False
ENABLE_EMERGENCY_BRAKE_PROTECTION = False


# ============================================================
# 2. 车辆起点与终点
# ============================================================

# 主车起点
EGO_SPAWN_LOC = carla.Location(x=206.059, y=-1.401, z=4.228)
EGO_SPAWN_ROT = carla.Rotation(pitch=-2.486, yaw=171.544, roll=0.000)

# 主车终点
EGO_END_LOC = carla.Location(x=54.588, y=-2.018, z=3.962)
EGO_END_ROT = carla.Rotation(pitch=0.108, yaw=176.573, roll=0.000)

# 背景车起点
BG_SPAWN_LOC = carla.Location(x=108.455, y=-67.623, z=5.155)
BG_SPAWN_ROT = carla.Rotation(pitch=-9.556, yaw=92.577, roll=0.000)

# 背景车终点：已按你的新要求修改
BG_END_LOC = carla.Location(x=175.399, y=1.976, z=4.426)
BG_END_ROT = carla.Rotation(pitch=-2.542, yaw=177.762, roll=0.000)


# ============================================================
# 3. 场景关键位置
# ============================================================

# 两车冲突区
ISSUE_CENTER_LOC = carla.Location(x=108.500, y=-1.800, z=4.700)

# 主车停止决策区：本应有停止线，但本场景中缺失
STOP_DECISION_LOC = carla.Location(x=130.000, y=-1.700, z=4.700)

# 枫叶低摩擦区域中心
LEAF_ZONE_LOC = carla.Location(x=108.500, y=-1.800, z=4.700)

# 仅用于控制台检测车辆是否进入枫叶区域
LEAF_ZONE_MONITOR_RADIUS = 24.0


# ============================================================
# 4. 基础工具函数
# ============================================================

def tuple_to_loc(p, z_offset=0.0):
    return carla.Location(x=p[0], y=p[1], z=p[2] + z_offset)


def get_speed_kmh(vehicle):
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


def enable_sync(world, dt=0.05):
    if hasattr(RTB, "enable_synchronous_mode"):
        RTB.enable_synchronous_mode(world, dt=dt)
    else:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)


def disable_sync(world):
    if hasattr(RTB, "disable_synchronous_mode"):
        RTB.disable_synchronous_mode(world)
    else:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)


def cleanup_actors(client, actor_list):
    if hasattr(RTB, "cleanup_actors"):
        RTB.cleanup_actors(client, actor_list)
    else:
        for actor in actor_list:
            if actor and actor.is_alive:
                try:
                    actor.destroy()
                except Exception:
                    pass


def safe_spawn_vehicle(world, candidates, x, y, z=None, yaw=None, color=None,
                       role_name="background", z_offset=0.6):
    """
    使用候选车辆蓝图安全生成车辆。
    优先调用 RTB.spawn_vehicle。
    """
    bp_lib = world.get_blueprint_library()
    available = set(bp.id for bp in bp_lib.filter("vehicle.*"))

    for bp_name in candidates:
        if bp_name in available:
            actor = RTB.spawn_vehicle(
                world=world,
                bp_name=bp_name,
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                color=color,
                role_name=role_name,
                z_offset=z_offset
            )
            if actor:
                return actor

    print("[生成失败] 候选车辆蓝图均不可用：", candidates)
    return None


# ============================================================
# 5. 黄昏天气设置
# ============================================================

def apply_dusk_weather(world):
    """
    黄昏天气设置。

    关键太阳角度参数：
        sun_altitude_angle = 8.0

    这个参数控制太阳高度角。
    数值越高，太阳越高；数值越低，越接近日落或夜晚。
    """
    RTB.set_static_weather(
        world,
        preset=None,

        # 云、雨、积水、风
        cloudiness=45.0,
        precipitation=5.0,
        precipitation_deposits=25.0,
        wind_intensity=20.0,

        # 太阳角度
        sun_azimuth_angle=0.0,
        sun_altitude_angle=0.0,

        # 雾气和湿度
        fog_density=1,
        fog_distance=1.0,
        fog_falloff=0.1,
        wetness=75.0,

        # 散射参数
        scattering_intensity=1,
        mie_scattering_scale=0.0300,
        rayleigh_scattering_scale=0.0331,

        # 沙尘
        dust_storm=0.0
    )


# ============================================================
# 6. 人工锚点循迹路径
# ============================================================

def build_ego_path():
    """
    主车路径：
    从 x=206.059, y=-1.401 沿主路向 x=54.588, y=-2.018 直行。
    """

    raw_path = [
        (206.059, -1.401, 4.228),

        (190.000, -1.450, 4.300),
        (175.000, -1.500, 4.400),
        (160.000, -1.550, 4.500),
        (145.000, -1.620, 4.600),

        # 停止决策区附近，停止线缺失
        (130.000, -1.700, 4.650),

        # 枫叶覆盖与冲突区域
        (118.000, -1.750, 4.700),
        (108.500, -1.800, 4.700),
        (98.000, -1.850, 4.650),

        # 驶离路口
        (85.000, -1.900, 4.550),
        (70.000, -1.960, 4.250),

        (54.588, -2.018, 3.962),
    ]

    dense = RTB.interpolate_trajectory(raw_path, interval=1.0)
    dense = RTB.clean_trajectory(dense, min_dist=0.5)
    return dense


def build_background_path():
    """
    背景车路径：
    从 x=108.455, y=-67.623 进入路口，
    到达冲突区附近后转向驶往新的背景车终点：
        x=175.399, y=1.976, z=4.426

    注意：
    这里以 BG_END_LOC 的位置为准进行循迹。
    BG_END_ROT 保存终点期望姿态，但 RTB.get_target_waypoint 控制主要依赖路径点位置。
    """

    raw_path = [
        # 起点
        (108.455, -67.623, 5.155),

        # 背景车沿支路向路口接近
        (108.500, -55.000, 5.100),
        (108.500, -42.000, 5.000),
        (108.500, -30.000, 4.900),
        (108.500, -20.000, 4.800),
        (108.500, -12.000, 4.750),
        (108.500, -6.000, 4.720),

        # 进入冲突区
        (108.500, -1.800, 4.700),

        # 转向驶往新的背景车终点方向
        (113.000, -0.800, 4.720),
        (120.000, 0.000, 4.780),
        (130.000, 0.500, 4.850),
        (142.000, 0.900, 4.930),
        (153.000, 1.200, 4.900),
        (165.000, 1.600, 4.650),

        # 新终点
        (175.399, 1.976, 4.426),
    ]

    dense = RTB.interpolate_trajectory(raw_path, interval=1.0)
    dense = RTB.clean_trajectory(dense, min_dist=0.5)
    return dense


# ============================================================
# 7. 摩擦区监测器：只打印，不画 debug
# ============================================================

class FrictionZoneMonitor:
    """
    只用于控制台打印车辆是否进入枫叶低摩擦区域。
    不做任何画面 debug 展示。
    """

    def __init__(self, center_loc, radius):
        self.center_loc = center_loc
        self.radius = radius
        self.ego_inside_last = False
        self.bg_inside_last = False

    def _inside(self, loc):
        return loc.distance(self.center_loc) <= self.radius

    def tick(self, ego, bg_vehicle):
        ego_loc = ego.get_location()
        bg_loc = bg_vehicle.get_location()

        ego_inside = self._inside(ego_loc)
        bg_inside = self._inside(bg_loc)

        if ego_inside and not self.ego_inside_last:
            print(
                f"[摩擦区检测] EGO 已进入低摩擦枫叶区域 | "
                f"speed={get_speed_kmh(ego):.2f} km/h | friction_value={FRICTION_VALUE}"
            )

        if bg_inside and not self.bg_inside_last:
            print(
                f"[摩擦区检测] 背景车已进入低摩擦枫叶区域 | "
                f"speed={get_speed_kmh(bg_vehicle):.2f} km/h | friction_value={FRICTION_VALUE}"
            )

        self.ego_inside_last = ego_inside
        self.bg_inside_last = bg_inside


# ============================================================
# 8. 独立主车控制模块
# ============================================================

class EgoControlModule:
    """
    主车控制模块。

    重要：
    EGO 有自己独立的纵向 PID 和横向 PID。
    不与背景车共用任何 PID 实例。
    后续替换主车算法时，主要替换 run_step() 即可。
    """

    def __init__(self, vehicle, path, target_speed_kmh=35.0, dt=0.05):
        self.vehicle = vehicle
        self.path = path
        self.target_speed_kmh = target_speed_kmh
        self.dt = dt
        self.current_index = 0

        # EGO 独立 PID
        self.pid_lon = RTB.PIDLongitudinalController(preset="wet_road", dt=dt)
        self.pid_lat = RTB.PIDLateralController(preset="wet_road", dt=dt)

    def run_step(self, world, sim_time):
        if not self.vehicle or not self.vehicle.is_alive:
            return

        ego_loc = self.vehicle.get_location()
        ego_speed_now = get_speed_kmh(self.vehicle)

        target_wp, self.current_index = RTB.get_target_waypoint(
            vehicle_loc=ego_loc,
            path_points=self.path,
            current_index=self.current_index,
            speed_kmh=ego_speed_now,
            min_lookahead=5.5,
            lookahead_ratio=0.45,
            max_search_ahead=45,
            fallback_dist=40.0
        )

        if target_wp is None:
            return

        RTB.apply_pid_control(
            vehicle=self.vehicle,
            pid_lon=self.pid_lon,
            pid_lat=self.pid_lat,
            target_speed_kmh=self.target_speed_kmh,
            target_wp=target_wp
        )

    def set_target_speed(self, speed_kmh):
        self.target_speed_kmh = float(speed_kmh)


# ============================================================
# 9. 独立背景车控制模块
# ============================================================

class BackgroundConstantSpeedController:
    """
    背景车控制模块。

    重要：
    背景车有自己独立的纵向 PID 和横向 PID。
    不与 EGO 共用任何 PID 实例。
    背景车释放后始终以 BG_TARGET_SPEED_KMH 目标速度循迹。
    """

    def __init__(self, vehicle, path, target_speed_kmh=35.0, dt=0.05):
        self.vehicle = vehicle
        self.path = path
        self.target_speed_kmh = target_speed_kmh
        self.dt = dt
        self.current_index = 0

        # 背景车独立 PID
        self.pid_lon = RTB.PIDLongitudinalController(preset="wet_road", dt=dt)
        self.pid_lat = RTB.PIDLateralController(preset="wet_road", dt=dt)

    def run_step(self, world, sim_time):
        if not self.vehicle or not self.vehicle.is_alive:
            return

        bg_loc = self.vehicle.get_location()
        bg_speed_now = get_speed_kmh(self.vehicle)

        target_wp, self.current_index = RTB.get_target_waypoint(
            vehicle_loc=bg_loc,
            path_points=self.path,
            current_index=self.current_index,
            speed_kmh=bg_speed_now,
            min_lookahead=5.0,
            lookahead_ratio=0.40,
            max_search_ahead=45,
            fallback_dist=40.0
        )

        if target_wp is None:
            return

        RTB.apply_pid_control(
            vehicle=self.vehicle,
            pid_lon=self.pid_lon,
            pid_lat=self.pid_lat,
            target_speed_kmh=self.target_speed_kmh,
            target_wp=target_wp
        )


# ============================================================
# 10. 主函数
# ============================================================

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    actor_list = []

    try:
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        sim_time = 0.0

        # 同步模式。
        # 注意：本脚本不设置 spectator，因此观察者视角可自由移动。
        enable_sync(world, dt=DT)

        # 黄昏天气
        apply_dusk_weather(world)

        # 构建路径
        ego_path = build_ego_path()
        bg_path = build_background_path()

        print("[路径检查] EGO path points:", len(ego_path))
        print("[路径检查] BG path points:", len(bg_path))

        # 生成 EGO
        ego = safe_spawn_vehicle(
            world,
            candidates=[
                "vehicle.tesla.model3",
                "vehicle.lincoln.mkz_2020",
                "vehicle.audi.tt",
                "vehicle.dodge.charger_2020"
            ],
            x=EGO_SPAWN_LOC.x,
            y=EGO_SPAWN_LOC.y,
            z=EGO_SPAWN_LOC.z,
            yaw=EGO_SPAWN_ROT.yaw,
            color="0,0,255",
            role_name="ego",
            z_offset=0.6
        )

        # 生成背景车
        bg_vehicle = safe_spawn_vehicle(
            world,
            candidates=[
                "vehicle.audi.tt",
                "vehicle.lincoln.mkz_2020",
                "vehicle.nissan.patrol",
                "vehicle.dodge.charger_2020"
            ],
            x=BG_SPAWN_LOC.x,
            y=BG_SPAWN_LOC.y,
            z=BG_SPAWN_LOC.z,
            yaw=BG_SPAWN_ROT.yaw,
            color="255,255,255",
            role_name="background",
            z_offset=0.6
        )

        if not ego or not bg_vehicle:
            print("[错误] EGO 或背景车生成失败，请检查生成点是否在可行车道路附近。")
            return

        actor_list.extend([ego, bg_vehicle])

        # ========================================================
        # 车辆灯光系统
        # ========================================================

        # 主车灯光：远光灯 + 雾灯
        ego_lights = RTB.VehicleLightManager(ego)
        ego_lights.turn_on(
            carla.VehicleLightState.HighBeam |
            carla.VehicleLightState.Fog
        )

        # 背景车 / 专项策划车辆灯光：左转灯 + 近光灯
        bg_lights = RTB.VehicleLightManager(bg_vehicle)
        bg_lights.turn_on(
            carla.VehicleLightState.LeftBlinker |
            carla.VehicleLightState.LowBeam
        )

        print("[车辆灯光] EGO 已开启远光灯 + 雾灯。")
        print("[车辆灯光] 背景车/专项策划车辆已开启左转灯 + 近光灯。")

        # 低摩擦区域，不画 debug box
        friction_actor = RTB.spawn_friction_region(
            world=world,
            bp_lib=bp_lib,
            center_loc=LEAF_ZONE_LOC,
            friction=FRICTION_VALUE,
            extent=(24.0, 22.0, 2.0),
            draw_debug=False,
            debug_life=0.0
        )
         
        # 低摩擦区域，不画 debug box
        friction_actor = RTB.spawn_friction_region(
            world=world,
            bp_lib=bp_lib,
            center_loc=LEAF_ZONE_LOC,
            friction=FRICTION_VALUE,
            extent=(24.0, 22.0, 2.0),
            draw_debug=False,
            debug_life=0.0
        )

        if friction_actor:
            actor_list.append(friction_actor)

        friction_monitor = FrictionZoneMonitor(
            center_loc=LEAF_ZONE_LOC,
            radius=LEAF_ZONE_MONITOR_RADIUS
        )

        # 独立 EGO 控制器
        ego_controller = EgoControlModule(
            vehicle=ego,
            path=ego_path,
            target_speed_kmh=EGO_TARGET_SPEED_KMH,
            dt=DT
        )

        # 独立背景车控制器
        bg_controller = BackgroundConstantSpeedController(
            vehicle=bg_vehicle,
            path=bg_path,
            target_speed_kmh=BG_TARGET_SPEED_KMH,
            dt=DT
        )

        # 两车初速度
        RTB.set_vehicle_initial_speed(
            ego,
            target_speed_kmh=EGO_INITIAL_SPEED_KMH,
            yaw_deg=EGO_SPAWN_ROT.yaw
        )

        RTB.set_vehicle_initial_speed(
            bg_vehicle,
            target_speed_kmh=BG_INITIAL_SPEED_KMH,
            yaw_deg=BG_SPAWN_ROT.yaw
        )

        # 背景车初始驻停，等待释放
        bg_released = False
        bg_vehicle.apply_control(
            carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
        )

        # 预热几帧
        for _ in range(8):
            world.tick()
            time.sleep(DT)

        print("[场景启动] 黄昏版本启动。")
        print("[PID检查] EGO 与背景车 PID 控制器完全独立。")
        print("[灯光同步] 请使用 UE TriggerBox 蓝图控制灯光：EGO 进入 TriggerBox2 后红黄灯隐藏，仅绿灯显示。")
        print("[天气设置] Dusk | sun_altitude_angle=8.0 | sun_azimuth_angle=35.0")
        print(f"[速度设置] EGO={EGO_TARGET_SPEED_KMH} km/h, BG={BG_TARGET_SPEED_KMH} km/h")
        print(f"[背景车释放距离] BG_RELEASE_DISTANCE={BG_RELEASE_DISTANCE}")
        print(f"[背景车终点] BG_END_LOC=({BG_END_LOC.x:.3f}, {BG_END_LOC.y:.3f}, {BG_END_LOC.z:.3f})")

        # ========================================================
        # 主循环
        # ========================================================

        while True:
            loop_t0 = time.time()
            world.tick()
            sim_time += DT

            ego_loc = ego.get_location()
            bg_loc = bg_vehicle.get_location()

            # 摩擦区进入监测，仅控制台打印
            friction_monitor.tick(ego, bg_vehicle)

            # 背景车释放
            if ego_loc.distance(STOP_DECISION_LOC) <= BG_RELEASE_DISTANCE and not bg_released:
                bg_released = True
                bg_vehicle.apply_control(
                    carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False)
                )

                RTB.set_vehicle_initial_speed(
                    bg_vehicle,
                    target_speed_kmh=BG_INITIAL_SPEED_KMH,
                    yaw_deg=BG_SPAWN_ROT.yaw
                )

                print(
                    f"[事件触发] 背景车释放 | "
                    f"BG_RELEASE_DISTANCE={BG_RELEASE_DISTANCE} | "
                    f"BG_TARGET_SPEED={BG_TARGET_SPEED_KMH} km/h"
                )

            # EGO 独立控制
            ego_controller.run_step(world, sim_time)

            # 背景车独立控制
            if bg_released:
                bg_controller.run_step(world, sim_time)
            else:
                bg_vehicle.apply_control(
                    carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
                )

            # 可选急刹保护
            dist_between = ego_loc.distance(bg_loc)

            if ENABLE_EMERGENCY_BRAKE_PROTECTION and dist_between < 7.5:
                ego.apply_control(
                    carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                )

            # 结束条件
            ego_arrived = ego_loc.distance(EGO_END_LOC) < 8.0
            bg_arrived = bg_loc.distance(BG_END_LOC) < 8.0

            if sim_time > 36.0:
                print("[场景结束] 达到最大仿真时长。")
                break

            if sim_time > 8.0 and ego_arrived and bg_arrived:
                print("[场景结束] 主车和背景车均已到达各自终点附近。")
                break

            elapsed = time.time() - loop_t0
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n[中断] 用户手动终止场景。")

    except Exception as e:
        print("[异常] 运行过程中发生错误：", e)

    finally:
        try:
            disable_sync(world)
        except Exception:
            pass

        try:
            cleanup_actors(client, actor_list)
        except Exception:
            pass

        print("[清理完成] 所有资源已回收。")


if __name__ == "__main__":
    main()