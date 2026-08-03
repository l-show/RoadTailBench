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
# 原始长尾场景轨迹数据 (已去除表头)
# 格式: Location_x, Location_y, Rotation_yaw
# ==========================================
RAW_TRAJ_NPC = """
-43.058 -59.563 75.803
-43.058 -59.563 75.803
-43.058 -59.563 75.803
-41.054 -51.643 75.803
-38.267 -41.879 70.772
-35.043 -32.883 70.265
-33.601 -28.876 69.035
-32.061 -25.332 64.025
-30.308 -21.96 61.831
-28.438 -18.581 59.387
-26.393 -15.309 57.307
-24.219 -12.122 54.031
-21.78 -9.218 44.286
-18.822 -6.741 39.321
-15.691 -4.734 19.033
-11.93 -4.546 -4.207
-8.071 -4.626 4.348
-4.314 -4.062 10.823
-0.609 -2.989 21.563
2.949 -1.483 23.157
6.456 -0.015 19.86
10.135 1.028 11.359
13.964 1.572 5.504
17.829 1.753 1.533
21.636 1.851 1.248
25.505 1.835 -1.711
29.311 1.734 -1.429
35.135 1.589 -1.429
42.872 1.396 -1.429
50.486 1.315 -0.013
58.223 1.349 0.272
65.958 1.377 -0.078
73.689 1.356 -0.218
81.293 1.327 -0.218
90.476 1.292 -0.218
100.451 1.276 0.065
110.424 1.288 0.065
120.397 1.284 -0.285
130.71 1.208 -0.498
137.699 1.147 -0.498
137.699 1.147 -0.498
137.699 1.147 -0.498
137.699 1.147 -0.498
"""

RAW_TRAJ_EGO = """
69.015 -2.936 177.672
69.015 -2.936 177.672
68.099 -2.899 177.672
65.559 -2.814 178.824
62.977 -2.757 178.611
60.395 -2.694 178.611
57.855 -2.633 178.611
55.273 -2.578 179.389
52.694 -2.574 -179.559
50.156 -2.594 -179.559
47.577 -2.606 179.944
45.038 -2.587 179.516
42.458 -2.571 -179.919
39.962 -2.584 -179.567
37.424 -2.606 -179.284
34.845 -2.644 -179.072
32.308 -2.685 -179.072
29.726 -2.706 179.726
27.147 -2.677 178.875
24.61 -2.6 177.165
22.036 -2.435 176.092
19.504 -2.275 177.229
16.927 -2.172 178.729
14.389 -2.15 -179.924
11.811 -2.165 -179.499
9.232 -2.195 -179.144
6.694 -2.238 -178.931
6.652 -2.239 -178.931
6.652 -2.239 -178.931
6.652 -2.239 -178.931
6.652 -2.239 -178.931
6.652 -2.239 -178.931
6.652 -2.239 -178.931
6.652 -2.239 -178.931
4.282 -2.286 -177.993
1.751 -2.455 -173.659
-0.799 -2.836 -169.36
-3.279 -3.369 -166.784
-5.735 -4.007 -163.331
-8.119 -4.974 -151.879
-10.334 -6.293 -147.466
-12.47 -7.735 -143.858
-14.447 -9.32 -139.35
-16.366 -11.039 -137.047
-18.251 -12.794 -137.047
-20.065 -14.564 -133.451
-21.716 -16.431 -129.421
-23.227 -18.465 -125.868
-24.591 -20.647 -119.565
-26.119 -23.34 -119.565
-27.952 -26.67 -117.042
-29.625 -30.153 -115.384
-32.077 -35.405 -111.785
-34.293 -41.448 -109.939
-36.138 -47.401 -106.209
-37.82 -53.62 -104.274
-39.364 -59.769 -103.707
-40.867 -66.034 -103.137
-42.27 -72.325 -101.921
-43.561 -78.536 -101.708
-44.843 -84.854 -100.851
-45.986 -91.094 -100.214
-47.13 -97.44 -100.214
-48.234 -103.687 -99.859
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

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 精确应用截图中定制的天气配置
        RTB.set_static_weather(
            world,
            cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_azimuth_angle=-1.0, sun_altitude_angle=15.0,
            fog_density=20.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
            scattering_intensity=0.5, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 长尾大雾天气系统已精确配置完成。")

        # ==========================================
        # 2. 轨迹数据解析、去重清洗与几何稠密化
        # ==========================================
        # 解析后数据结构为: [(x, y, yaw), (x, y, yaw)...]
        parsed_npc = RTB.parse_string_trajectory(RAW_TRAJ_NPC, min_dist=0.5)
        parsed_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO, min_dist=0.5)

        # 对锚点进行稠密化(间距设为 0.5m，保障循迹丝滑)
        traj_npc = RTB.interpolate_trajectory(parsed_npc, interval=0.5)
        traj_ego = RTB.interpolate_trajectory(parsed_ego, interval=0.5)

        print(f"[场景配置] 轨迹已挂载，NPC轨迹点数: {len(traj_npc)}，Ego轨迹点数: {len(traj_ego)}")

        # ==========================================
        # 3. 实体生成与物理速度注入
        # ==========================================
        # 第一辆车 (NPC)
        npc_vehicle = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala',
                                        x=parsed_npc[0][0], y=parsed_npc[0][1], yaw=parsed_npc[0][2], role_name='npc')
        actor_list.append(npc_vehicle)

        # 第二辆车 (EGO)
        ego_vehicle = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        # 强制瞬发赋予 50km/h 初速度
        RTB.set_vehicle_initial_speed(npc_vehicle, 50.0, yaw_deg=parsed_npc[0][2])

        # ==========================================
        # 4. 车灯系统配置
        # ==========================================
        # 挂载灯光管理器并开启基础行车灯（示宽灯+近光灯）
        light_npc = RTB.VehicleLightManager(npc_vehicle)
        light_npc.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam)

        # ==========================================
        # 5. 独立的 PID 控制器挂载
        # ==========================================
        pid_lon_npc = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_npc = RTB.PIDLateralController(preset='default_car')

        # ==========================================
        # 6. 长尾场景剧本状态机编排
        # ==========================================
        # NPC 一直保持 50km/h 巡航
        npc_sm = RTB.MultiStageBehaviorMachine(initial_speed=50.0)

        # EGO 剧本：在 x=14 减速至0，停顿5秒后恢复50。
        # (因为轨迹 X 逐渐变小，驶入负数区域，故使用 'x_less')

        # ==========================================
        # 7. 预热与初始变量
        # ==========================================
        idx_npc = 0
        idx_ego = 0
        sim_time = 0.0

        print("\n[仿真开启] 🚀 长尾场景已启动，正在执行剧本...")

        # ==========================================
        # 8. 仿真主循环
        # ==========================================
        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # --- NPC 车辆控制 ---
            # 1. 出界守护 (若越界直接在函数内销毁)
            if not RTB.check_vehicle_out_of_bounds(npc_vehicle, carla_map, threshold_dist=6.0, auto_destroy=True):
                # 2. 状态机运算目标速度
                target_speed_npc = npc_sm.tick(npc_vehicle.get_location(), sim_time, dt)
                # 3. 滑窗预瞄获取目标点
                target_wp_npc, idx_npc = RTB.get_target_waypoint(npc_vehicle.get_location(), traj_npc, idx_npc,
                                                                 speed_kmh=target_speed_npc)
                if target_wp_npc:
                    RTB.apply_pid_control(npc_vehicle, pid_lon_npc, pid_lat_npc, target_speed_npc, target_wp_npc)
                    light_npc.auto_update_from_control()  # 联动刹车灯

            # --- Ego 车辆控制 ---

                    # 如果两辆车都走完了/出界销毁了，可选择终止仿真
            if (not npc_vehicle or not npc_vehicle.is_alive) and (not ego_vehicle or not ego_vehicle.is_alive):
                print("[场景提示] 所有参演车辆已离开场地，仿真结束。")
                break

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