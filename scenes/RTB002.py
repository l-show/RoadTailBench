# -- coding: utf-8 --
import sys
import carla
import time
import math

# ==========================================
# 1. 动态引入标准化函数库路径
# ==========================================
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 轨迹数据硬编码区域 (保持原始坐标，格式为 [x, y, yaw])
# ==========================================
RAW_SUV_PATH_POINTS = [
    (-31.867, 54.232, -96.303), (-31.867, 54.232, -96.303), (-31.867, 54.232, -95.948), (-31.867, 54.232, -95.382),
    (-32.042, 52.206, -94.388), (-32.189, 49.629, -91.96), (-32.251, 47.047, -91.181), (-32.292, 44.547, -90.546),
    (-32.316, 41.964, -90.546), (-32.341, 39.422, -90.546), (-32.365, 36.839, -90.546), (-32.394, 34.339, -91.038),
    (-32.457, 31.798, -91.959), (-32.556, 29.3, -92.312), (-32.664, 26.744, -92.735), (-32.803, 24.174, -93.227),
    (-32.903, 21.597, -91.62), (-32.948, 19.098, -90.422), (-32.964, 16.598, -90.142), (-32.968, 14.015, -90.072),
    (-32.969, 11.473, -89.646), (-32.949, 8.89, -89.786), (-32.939, 6.306, -89.786), (-32.93, 3.806, -89.786),
    (-32.935, 1.305, -90.211), (-32.932, -1.195, -89.573), (-32.866, -3.778, -88.296), (-32.904, -6.318, -93.698),
    (-33.033, -7.98, -94.986), (-33.033, -7.98, -94.986), (-33.033, -7.98, -94.986), (-33.033, -7.98, -94.986),
    (-33.033, -7.98, -94.986), (-33.033, -7.98, -94.986), (-33.033, -7.98, -94.986), (-33.033, -7.98, -94.986),
    (-33.033, -7.98, -94.986), (-33.033, -7.98, -94.843), (-33.177, -10.141, -92.395), (-33.283, -12.68, -92.325),
    (-33.385, -15.178, -92.325), (-33.489, -17.759, -92.325), (-33.592, -20.299, -92.325), (-33.694, -22.88, -91.972),
    (-33.739, -25.463, -89.688), (-33.726, -27.963, -89.688), (-33.712, -30.546, -89.688), (-33.704, -33.053, -90.184),
    (-33.721, -35.553, -90.749), (-33.783, -38.052, -92.24), (-33.881, -40.55, -92.24), (-33.97, -43.049, -91.958),
    (-34.053, -45.63, -91.745), (-34.131, -48.171, -91.745), (-34.204, -50.67, -91.532), (-34.267, -53.169, -91.179),
    (-34.32, -55.752, -91.179), (-34.372, -58.294, -91.109), (-34.415, -60.877, -90.967), (-34.459, -63.46, -90.967),
    (-34.502, -66.001, -90.967), (-34.545, -68.585, -90.967), (-34.586, -71.167, -90.897), (-34.629, -73.667, -91.037),
    (-34.677, -76.165, -91.177), (-34.733, -78.747, -91.247), (-34.789, -81.329, -91.247), (-34.845, -83.911, -91.247),
    (-34.9, -86.41, -91.317), (-34.957, -88.992, -91.106), (-34.997, -91.487, -90.894), (-35.033, -93.982, -90.684),
    (-35.063, -96.517, -90.684), (-35.086, -99.098, -90.474), (-35.108, -101.68, -90.614), (-35.146, -104.263, -91.034),
    (-35.192, -106.805, -91.034), (-35.239, -109.388, -91.034), (-35.285, -111.971, -91.034),
    (-35.331, -114.512, -91.034), (-35.376, -117.011, -91.034), (-35.423, -119.594, -91.033),
    (-35.47, -122.179, -91.033), (-35.512, -124.68, -90.891), (-35.551, -127.18, -90.891), (-35.59, -129.681, -90.891),
    (-35.63, -132.264, -90.891), (-35.687, -134.763, -91.523), (-35.763, -137.346, -91.946),
    (-35.855, -139.927, -92.088), (-35.95, -142.508, -92.018), (-35.981, -145.047, -90.279),
    (-35.993, -147.546, -90.279), (-35.992, -150.128, -89.714), (-35.774, -152.613, -77.951),
    (-34.989, -154.98, -65.476), (-34.582, -155.801, -62.578)
]

RAW_TRUCK_PATH_POINTS = [
    (-36.142, 20.437, -95.433), (-36.142, 20.437, -96.356), (-36.142, 20.437, -97.215),
    (-36.3, 19.077, -96.303), (-36.498, 16.591, -92.728), (-36.591, 14.054, -91.729),
    (-36.65, 11.471, -91.023), (-36.691, 8.972, -90.953), (-36.7, 6.43, -89.52),
    (-36.696, 3.889, -90.301), (-36.717, 1.306, -90.654), (-36.751, -1.277, -91.225),
    (-36.852, -3.859, -93.296), (-37.042, -6.351, -95.207), (-37.297, -8.922, -95.775),
    (-37.548, -11.409, -95.775), (-37.799, -13.98, -93.78), (-37.898, -16.478, -90.484),
    (-37.824, -19.018, -86.574), (-37.645, -21.595, -85.933), (-37.462, -24.172, -86.361),
    (-37.389, -26.754, -90.468), (-37.492, -29.335, -94.188), (-37.716, -31.863, -96.67),
    (-38.017, -34.388, -96.812), (-38.286, -36.916, -94.888), (-38.485, -39.491, -93.755),
    (-38.63, -42.07, -92.479), (-38.663, -44.611, -89.698), (-38.583, -47.11, -87.74),
    (-38.484, -49.65, -88.166), (-38.451, -52.149, -90.976), (-38.543, -54.73, -93.553),
    (-38.708, -57.225, -93.976), (-38.866, -59.718, -93.061), (-38.997, -62.296, -92.291),
    (-39.046, -64.839, -90.094), (-39.011, -67.381, -88.259), (-38.933, -69.963, -88.259),
    (-38.866, -72.462, -88.896), (-38.874, -74.961, -91.102), (-38.924, -77.544, -91.102),
    (-38.998, -80.084, -91.945), (-39.096, -82.666, -92.437), (-39.246, -85.162, -94.273),
    (-39.475, -87.651, -95.846), (-39.642, -90.229, -92.805), (-39.661, -92.728, -89.041),
    (-39.618, -95.311, -89.041), (-39.576, -97.811, -89.041), (-39.535, -100.394, -89.542),
    (-39.529, -102.935, -90.609), (-39.588, -105.476, -91.882), (-39.675, -108.017, -91.952),
    (-39.76, -110.516, -91.952), (-39.845, -113.103, -91.882), (-39.843, -115.602, -88.927),
    (-39.795, -118.183, -88.927), (-39.721, -120.723, -87.557), (-39.603, -123.219, -86.267),
    (-39.435, -125.797, -86.267), (-39.312, -128.378, -89.145), (-39.333, -130.963, -91.113),
    (-39.406, -133.462, -91.825), (-39.515, -136.043, -92.605), (-39.628, -138.541, -92.605),
    (-39.746, -141.121, -92.605), (-39.869, -143.66, -95.205), (-40.158, -146.227, -96.508),
    (-40.441, -148.711, -96.508), (-40.734, -151.277, -96.508), (-40.823, -152.064, -96.508)
]


def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        bp_lib = world.get_blueprint_library()
        dt = 0.05
        sim_time = 0.0

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        weather = carla.WeatherParameters(
            cloudiness=25.0, precipitation=35.0, precipitation_deposits=80.0,
            wind_intensity=20.0, sun_azimuth_angle=95.0, sun_altitude_angle=10.0,
            fog_density=4.0, fog_distance=0.0, fog_falloff=0.5, wetness=100.0,
            scattering_intensity=2.5, mie_scattering_scale=0.1,
            rayleigh_scattering_scale=0.07, dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已设置")

        # ==========================================
        # 2. 轨迹数据清洗与绘制
        # ==========================================
        # 直接使用库函数对原始列表数据进行去重和0.5m插值抽稀
        traj_suv = RTB.clean_trajectory(RAW_SUV_PATH_POINTS, min_dist=0.5)
        traj_truck = RTB.clean_trajectory(RAW_TRUCK_PATH_POINTS, min_dist=0.5)

        # 🚀【新增功能】画出所有车辆的预设灰色轨迹线，方便调试观察
        RTB.draw_preset_trajectory(world, traj_suv, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_truck, color=carla.Color(150, 150, 150))

        # ==========================================
        # 3. 车辆、行人、模型实体安全生成
        # ==========================================
        # 生成 SUV (Cybertruck)
        suv = RTB.spawn_vehicle(world, 'vehicle.tesla.cybertruck',
                                x=traj_suv[0][0], y=traj_suv[0][1], yaw=traj_suv[0][2], z_offset=1.0)
        actor_list.append(suv)

        # 生成 消防卡车 (Firetruck)
        # 注意：使用原生 API 生成带有特定颜色的车辆以满足你的红色要求
        bp_truck = bp_lib.find('vehicle.carlamotors.firetruck')
        if bp_truck.has_attribute('color'):
            bp_truck.set_attribute('color', '255,0,0')
        trans_truck = carla.Transform(carla.Location(x=traj_truck[0][0], y=traj_truck[0][1], z=1.0),
                                      carla.Rotation(yaw=traj_truck[0][2]))
        firetruck = world.try_spawn_actor(bp_truck, trans_truck)
        actor_list.append(firetruck)

        # 生成 黄色雪铁龙 EGO (由 TM 接管控制)
        bp_ego = bp_lib.find('vehicle.citroen.c3')
        bp_ego.set_attribute('role_name', 'ego')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '255,255,0')  # 设定黄色
        trans_c3 = carla.Transform(carla.Location(x=-32.672, y=39.250, z=1.0), carla.Rotation(yaw=-90.0))
        c3_ego = world.try_spawn_actor(bp_ego, trans_c3)

        # Traffic Manager 接管 C3
        if c3_ego:
            actor_list.append(c3_ego)
            tm = client.get_trafficmanager(8000)
            tm.set_synchronous_mode(True)
            c3_ego.set_autopilot(True, tm.get_port())
            tm.vehicle_percentage_speed_difference(c3_ego, -20.0)  # TM控制超速
            print("[实体生成] 黄色雪铁龙 C3 生成成功 (由TrafficManager接管)")

        print("[实体生成] 轨迹车辆生成完毕，正在稳定物理环境...")
        for _ in range(20):
            world.tick()

        # ==========================================
        # 4. 车辆PID控制器挂载 (每辆车独立专属)
        # ==========================================
        pid_lon_suv = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_suv = RTB.PIDLateralController(preset='default_car')

        pid_lon_trk = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_trk = RTB.PIDLateralController(preset='truck')

        idx_suv, idx_trk = 0, 0  # 轨迹索引缓存

        # ==========================================
        # 5. 车辆灯光管理器
        # ==========================================
        lights_suv = RTB.VehicleLightManager(suv)
        lights_trk = RTB.VehicleLightManager(firetruck)

        # ==========================================
        # 6. 剧本状态机编排
        # ==========================================
        # SUV 状态机：初始 101km/h，在 Y <= -5.0 时，触发降速到 70km/h
        sm_suv = RTB.MultiStageBehaviorMachine(initial_speed=101.0)
        sm_suv.add_stage('y_less', target_speed=70.0, trigger_val=-5.0, accel=15.0)

        # Firetruck 状态机：全程保持 110km/h
        sm_trk = RTB.MultiStageBehaviorMachine(initial_speed=110.0)

        # ==========================================
        # 7. 预热与初始状态注入
        # ==========================================
        RTB.set_vehicle_initial_speed(suv, target_speed_kmh=101.0)
        RTB.set_vehicle_initial_speed(firetruck, target_speed_kmh=110.0)

        # ==========================================
        # 8. 仿真主循环（帧率同步与环境清理守护）
        # ==========================================
        print("\n[场景运行] 仿真核心逻辑执行中，按 Ctrl+C 退出。")
        while True:
            # 记录本帧开始的时间，用于补齐硬件时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 车辆控制逻辑 ----------------

            # SUV 控制
            if not RTB.check_vehicle_out_of_bounds(suv, carla_map, auto_destroy=True):
                # 状态机更新目标速度
                spd_suv = sm_suv.tick(suv.get_location(), sim_time, dt)
                # 获取路径跟踪点
                wp_suv, idx_suv = RTB.get_target_waypoint(suv.get_location(), traj_suv, idx_suv, speed_kmh=spd_suv)
                # 绘制实时预瞄点引导线
                RTB.draw_lookahead_point(world, suv.get_location(), wp_suv)
                # 执行库函数 PID 控制
                RTB.apply_pid_control(suv, pid_lon_suv, pid_lat_suv, spd_suv, wp_suv)
                # 智能灯光联动 (刹车亮尾灯)
                lights_suv.auto_update_from_control()

            # Firetruck 控制
            if not RTB.check_vehicle_out_of_bounds(firetruck, carla_map, auto_destroy=True):
                spd_trk = sm_trk.tick(firetruck.get_location(), sim_time, dt)
                wp_trk, idx_trk = RTB.get_target_waypoint(firetruck.get_location(), traj_truck, idx_trk,
                                                          speed_kmh=spd_trk)
                RTB.apply_pid_control(firetruck, pid_lon_trk, pid_lat_trk, spd_trk, wp_trk)
                lights_trk.auto_update_from_control()

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[运行异常] 发生错误: {e}")
    finally:
        # 恢复异步模式并一键清理场景实体
        RTB.disable_synchronous_mode(world)
        if 'tm' in locals():
            tm.set_synchronous_mode(False)  # 解除 TM 的同步模式锁定
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()
