# -*- coding: utf-8 -*-
import sys
import carla
import time
import math
import random

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 原始轨迹字符串数据区
# ==========================================
RAW_TRAJ_V1 = """
10.068	-34.081	-92.398
10.068	-34.081	-92.398
10.068	-34.081	-92.398
9.946	-36.577	-92.533
9.72	-42.925	-92.234
9.481	-49.274	-91.954
9.298	-55.626	-91.394
9.158	-61.978	-91.254
9.018	-68.33	-91.254
8.879	-74.683	-91.254
8.741	-81.139	-91.184
8.61	-87.492	-91.184
8.508	-92.428	-91.184
8.433	-96.052	-91.184
8.409	-97.344	-89.445
8.468	-98.592	-84.281
8.64	-99.85	-79.087
8.954	-101.081	-73.121
9.393	-102.273	-65.414
9.992	-103.369	-57.299
10.748	-104.389	-48.78
11.651	-105.251	-38.656
12.723	-105.969	-30.039
13.866	-106.518	-24.309
15.048	-106.966	-8.998
16.296	-106.999	3.385
17.562	-106.898	2.367
18.832	-106.846	2.367
22.079	-106.711	2.367
26.262	-106.539	2.367
33.715	-106.292	0.778
41.339	-106.227	-0.07
48.97	-106.29	-0.775
56.596	-106.409	-1.48
64.218	-106.606	-1.48
71.841	-106.77	-0.702
79.465	-106.863	-0.702
87.093	-106.958	-0.772
94.717	-107.061	-0.772
102.34	-107.202	-1.122
109.963	-107.352	-1.122
117.58	-107.664	-4.069
125.176	-108.325	-5.638
132.764	-109.081	-5.708
140.227	-109.827	-5.708
147.812	-110.614	-6.57
155.389	-111.487	-6.57
162.958	-112.414	-7.34
170.517	-113.423	-7.908
178.056	-114.562	-8.763
185.598	-115.691	-8.413
193.15	-116.752	-7.783
200.706	-117.778	-7.714
203.059	-118.095	-7.644
203.059	-118.095	-7.644
203.059	-118.095	-7.644
"""

RAW_TRAJ_V2 = """
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.92
7.539	40.785	-92.99
7.367	38.354	-94.061
6.687	26.938	-92.226
6.466	15.507	-90.214
6.445	4.071	-90.074
6.376	-7.366	-90.634
6.228	-18.803	-90.774
6.071	-30.432	-90.704
5.956	-41.682	-90.424
5.885	-53.119	-90.284
5.829	-64.556	-90.284
5.782	-74.035	-90.284
5.752	-80.119	-90.284
5.727	-85.202	-90.284
5.665	-90.285	-91.644
5.423	-95.278	-95.295
4.824	-100.323	-102.483
2.511	-104.783	-134.13
-1.664	-107.654	-149.459
-6.209	-109.667	-168.757
-11.249	-110.326	-173.34
-16.323	-110.314	175.522
-21.389	-109.896	175.443
-26.462	-109.575	177.949
-31.545	-109.506	-179.91
-36.6	-109.53	-179.63
-41.683	-109.562	-179.63
-46.766	-109.595	-179.63
-51.767	-109.627	-179.63
-56.933	-109.661	-179.63
-62.016	-109.693	-179.63
-67.1	-109.726	-179.63
-72.182	-109.799	-178.063
-77.334	-110.165	-173.29
-82.345	-110.998	-166.156
-87.136	-112.42	-161.288
-91.847	-114.32	-154.729
-96.274	-116.809	-147.112
-100.397	-119.778	-141.797
-104.183	-123.162	-134.368
-107.611	-126.912	-130.308
-110.726	-131.029	-124.343
-113.353	-135.377	-117.752
-115.453	-140.002	-110.367
-116.921	-144.864	-104.611
-118.18	-149.701	-103.744
-118.969	-154.8	-96.936
-119.586	-159.844	-97.006
-120.196	-164.805	-97.219
-120.809	-169.935	-96.507
-120.828	-170.101	-96.507
-120.828	-170.101	-96.507
-120.828	-170.101	-96.507
-120.828	-170.101	-96.507
"""

RAW_TRAJ_V3_EGO = """
10.274	21.468	-92.638
10.274	21.468	-92.638
10.274	21.468	-92.638
10.202	19.888	-92.428
10.062	14.89	-90.811
10.024	9.807	-90.111
10.013	4.724	-90.531
9.965	-0.442	-90.531
9.919	-5.442	-90.531
9.871	-10.525	-90.741
9.776	-15.607	-91.441
9.648	-20.689	-91.511
9.535	-25.772	-90.951
9.511	-30.854	-90.034
9.498	-35.939	-90.314
9.47	-41.023	-90.314
9.419	-46.106	-90.664
9.36	-51.19	-90.664
9.302	-56.273	-90.664
9.243	-61.356	-90.664
9.174	-66.439	-91.084
9.061	-71.518	-91.294
8.953	-76.513	-91.154
8.889	-81.595	-90.312
8.861	-86.678	-90.312
8.832	-91.761	-90.382
8.798	-96.844	-90.382
8.742	-105.239	-90.382
8.674	-115.404	-90.382
8.61	-125.406	-90.312
8.519	-135.567	-90.942
8.27	-145.731	-92.064
7.848	-155.721	-93.051
7.249	-165.881	-93.472
6.633	-176.029	-93.542
5.851	-186.165	-95.508
4.7	-196.266	-96.847
3.483	-206.359	-96.917
2.298	-216.288	-96.777
1.059	-226.379	-97.547
-0.4	-236.441	-98.463
-2.006	-246.479	-99.168
-3.615	-256.518	-99.168
-5.247	-266.552	-99.238
-6.975	-276.571	-100.011
-8.693	-286.591	-98.671
-10.206	-296.476	-98.881
-11.8	-306.517	-99.231
-13.415	-316.554	-99.021
-15.071	-326.585	-100.215
-17.099	-336.376	-103.035
-19.511	-346.252	-104.227
-22.067	-356.096	-105.351
-24.999	-365.83	-106.635
-27.61	-375.136	-105.29
-27.61	-375.136	-105.29
-27.61	-375.136	-105.29
"""

RAW_TRAJ_V4_TRUCK = """
6.937	-10.55	-94.53
6.937	-10.55	-94.53
6.937	-10.55	-94.53
6.634	-14.371	-94.53
6.074	-24.521	-92.098
5.859	-34.685	-90.601
5.752	-44.851	-90.531
5.696	-54.851	-90.181
5.663	-65.186	-90.181
5.631	-75.353	-90.181
5.598	-85.52	-90.181
5.567	-95.52	-90.181
5.528	-105.688	-90.391
5.423	-115.853	-91.167
5.212	-126.188	-91.167
5.005	-136.353	-91.167
4.682	-146.681	-92.031
4.322	-156.845	-92.031
3.908	-167.003	-93.687
3.093	-177.137	-94.845
2.119	-187.257	-95.837
1.085	-197.371	-95.837
0.051	-207.485	-95.837
-1.1	-217.586	-97.161
-2.542	-227.65	-99.212
-4.181	-237.685	-99.282
-5.83	-247.718	-99.492
-7.534	-257.742	-99.842
-9.287	-267.926	-99.632
-10.939	-277.957	-99.142
-12.542	-287.997	-99.072
-14.145	-298.037	-99.072
-15.748	-308.077	-99.072
-17.49	-318.096	-100.7
-19.436	-328.074	-101.473
-21.616	-338.176	-102.53
-23.919	-348.08	-103.957
-26.411	-357.936	-104.307
-29.016	-367.764	-105.078
-31.68	-377.575	-105.218
-34.395	-387.373	-105.991
-35.359	-390.737	-105.991
-35.359	-390.737	-105.991
-35.359	-390.737	-105.991
"""

# ==========================================
# 独立物理树叶风力系统 (高空特化绕Bug版)
# ==========================================
class LeafWindManager:
    """
    【高空自定义坠落物理系统】
    使用纯 Python 数学逻辑接管物理属性。
    针对 Z=14.2m 的高空抛物进行了气动参数优化，确保树叶优美且跨越马路。
    """

    def __init__(self, world, blueprint_library, spawn_point, target_point, mesh_path, num_leaves=100):
        self.world = world
        self.bp_lib = blueprint_library

        self.spawn_point = spawn_point
        self.target_point = target_point
        self.mesh_path = mesh_path
        self.num_leaves = num_leaves

        self.leaf_mass = 0.02  # 极轻，不影响汽车物理
        self.leaf_scale = 5  # 放大可见度

        # 【针对高空坠落优化】风力与阻力物理参数
        self.base_wind_strength = 0.12  # 稍微加大水平推力，确保能吹向 20m 外的马路对面
        self.upward_lift_force = 0.08  # 稍微增强升力（降落伞效应），延长从 14m 掉下来的滞空时间
        self.flutter_amplitude = 0.08  # 加大空中扰动幅度，视觉上更狂乱
        self.flutter_frequency = 6.0

        # 空气阻力系数
        self.linear_drag_coeff = 0.03
        self.angular_drag_coeff = 0.002

        self.leaves_data = []
        self.has_spawned = False

        # 计算基础风向向量
        dx = self.target_point.x - self.spawn_point.x
        dy = self.target_point.y - self.spawn_point.y
        distance_xy = math.sqrt(dx ** 2 + dy ** 2)
        self.dir_x = dx / distance_xy if distance_xy > 0 else 0
        self.dir_y = dy / distance_xy if distance_xy > 0 else 0

    def spawn_leaves(self):
        """生成实体并交由物理引擎接管"""
        bp_prop = self.bp_lib.find('static.prop.mesh')
        bp_prop.set_attribute('mesh_path', self.mesh_path)
        bp_prop.set_attribute('mass', str(self.leaf_mass))
        bp_prop.set_attribute('scale', str(self.leaf_scale))

        spawned_count = 0
        for i in range(self.num_leaves):
            # X/Y/Z轴散开生成防重叠爆炸
            offset_x = random.uniform(-2.0, 2.0)
            offset_y = random.uniform(-2.0, 2.0)
            # 因为 Z=14.2m 非常高，这里可以放开 Z 轴的随机性，生成一个立体树叶团
            offset_z = random.uniform(-1.5, 1.5)

            loc = carla.Location(
                x=self.spawn_point.x + offset_x,
                y=self.spawn_point.y + offset_y,
                z=self.spawn_point.z + offset_z
            )
            rot = carla.Rotation(pitch=random.uniform(0, 360), yaw=random.uniform(0, 360), roll=random.uniform(0, 360))

            leaf_actor = self.world.try_spawn_actor(bp_prop, carla.Transform(loc, rot))

            if leaf_actor:
                # 开启物理，但不调用 Carla 原生阻尼，避开穿模掉地底的 Bug
                leaf_actor.set_simulate_physics(True)

                self.leaves_data.append({
                    'actor': leaf_actor,
                    'phase_x': random.uniform(0, math.pi * 2),
                    'phase_y': random.uniform(0, math.pi * 2),
                    'settled': False
                })
                spawned_count += 1

        self.has_spawned = True
        print(f"\n🍃 [特效系统] 成功在高空({self.spawn_point.z:.1f}m)生成了 {spawned_count} 片树叶，狂风开始吹袭！")

    def tick(self, sim_time):
        """主循环帧刷新：每帧手动注入空气动力学受力"""
        if not self.has_spawned:
            return

        for leaf in self.leaves_data:
            if leaf['settled']: continue

            actor = leaf['actor']
            if not actor.is_alive: continue

            loc = actor.get_location()

            # 落地判定
            if loc.z <= self.target_point.z + 0.3:
                leaf['settled'] = True
                continue

            vel = actor.get_velocity()
            ang_vel = actor.get_angular_velocity()

            # 1. 基础定向推力
            f_x = self.dir_x * self.base_wind_strength
            f_y = self.dir_y * self.base_wind_strength
            f_z = self.upward_lift_force

            # 2. 湍流正弦扰动
            flutter_x = math.sin(sim_time * self.flutter_frequency + leaf['phase_x']) * self.flutter_amplitude
            flutter_y = math.cos(sim_time * self.flutter_frequency + leaf['phase_y']) * self.flutter_amplitude

            # 3. 纯数学自定义空气阻力(速度越快，阻力越大)
            drag_x = -self.linear_drag_coeff * vel.x
            drag_y = -self.linear_drag_coeff * vel.y
            drag_z = -self.linear_drag_coeff * vel.z

            total_force = carla.Vector3D(x=f_x + flutter_x + drag_x, y=f_y + flutter_y + drag_y, z=f_z + drag_z)
            actor.add_force(total_force)

            # 4. 旋转阻尼
            torque_x = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.x
            torque_y = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.y
            torque_z = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.z
            actor.add_torque(carla.Vector3D(x=torque_x, y=torque_y, z=torque_z))

    def cleanup(self):
        """资源释放"""
        for leaf in self.leaves_data:
            if leaf['actor'].is_alive:
                leaf['actor'].destroy()
        self.leaves_data.clear()

# ==========================================
# 主场景运行逻辑
# ==========================================
def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    leaf_manager = None

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

        target_weather = carla.WeatherParameters(
            cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_azimuth_angle=-1.0, sun_altitude_angle=15.0,
            fog_density=2.0, fog_distance=0.75, fog_falloff=0.1,
            wetness=0.0, scattering_intensity=1.0, mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )
        world.set_weather(target_weather)
        print("[场景配置] 天气系统已按照截图参数精确设置。")

        # ==========================================
        # 2. 轨迹数据硬编码与清洗稠密化
        # ==========================================
        traj1_raw = RTB.parse_string_trajectory(RAW_TRAJ_V1)
        traj1 = RTB.interpolate_trajectory(traj1_raw, interval=0.5)

        traj2_raw = RTB.parse_string_trajectory(RAW_TRAJ_V2)
        traj2 = RTB.interpolate_trajectory(traj2_raw, interval=0.5)

        traj3_raw = RTB.parse_string_trajectory(RAW_TRAJ_V3_EGO)
        traj3 = RTB.interpolate_trajectory(traj3_raw, interval=0.5)

        traj4_raw = RTB.parse_string_trajectory(RAW_TRAJ_V4_TRUCK)
        traj4 = RTB.interpolate_trajectory(traj4_raw, interval=0.5)

        # ==========================================
        # 3. 车辆实体安全生成
        # ==========================================
        v1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala',
                               x=traj1[0][0], y=traj1[0][1], yaw=traj1_raw[0][2], role_name="v1")
        v2 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala',
                               x=traj2[0][0], y=traj2[0][1], yaw=traj2_raw[0][2], role_name="v2")
        v3_ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                   x=traj3[0][0], y=traj3[0][1], yaw=traj3_raw[0][2],
                                   color="255,105,180", role_name="ego")
        v4_truck = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter',
                                     x=traj4[0][0], y=traj4[0][1], yaw=traj4_raw[0][2], color="0,0,0", role_name="v4",
                                     z_offset=1.5)

        for v in [v1, v2, v3_ego, v4_truck]:
            if v: actor_list.append(v)

        # ==========================================
        # 4. 车辆PID与车灯管理器挂载
        # ==========================================
        pid_lon1, pid_lat1 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon2, pid_lat2 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon3, pid_lat3 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon4, pid_lat4 = RTB.PIDLongitudinalController(preset='truck'), RTB.PIDLateralController(preset='truck')

        light_mgrs = {}
        for v in [v1, v2, v3_ego, v4_truck]:
            if v:
                lm = RTB.VehicleLightManager(v)
                lm.turn_on(carla.VehicleLightState.Position)
                if v in [v1, v2, v3_ego]:
                    lm.turn_on(carla.VehicleLightState.LowBeam)
                light_mgrs[v.id] = lm

        idx1 = idx2 = idx3 = idx4 = 0
        v1_goal_x, v1_goal_y = traj1[-1][0], traj1[-1][1]
        v2_goal_x, v2_goal_y = traj2[-1][0], traj2[-1][1]
        ego_goal_x, ego_goal_y = traj3[-1][0], traj3[-1][1]
        v4_goal_x, v4_goal_y = traj4[-1][0], traj4[-1][1]
        vehicle_goal_radius_m = 2.5

        def is_at_goal(loc, goal_x, goal_y):
            return math.hypot(loc.x - goal_x, loc.y - goal_y) <= vehicle_goal_radius_m

        def destroy_scene_vehicle(vehicle):
            if vehicle and vehicle.is_alive:
                vehicle.destroy()
            if vehicle in actor_list:
                actor_list.remove(vehicle)

        # ==========================================
        # 5. 剧本状态机编排
        # ==========================================
        sm1 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm1.add_stage('y_less', 20.0, trigger_val=-80.0, accel=20.0)
        sm1.add_stage('time', 20.0, trigger_val=3.0, accel=0.0)
        sm1.add_stage('immediate', 60.0, accel=15.0)

        sm2 = RTB.MultiStageBehaviorMachine(initial_speed=55.0)
        sm2.add_stage('y_less', 20.0, trigger_val=-80.0, accel=20.0)
        sm2.add_stage('time', 20.0, trigger_val=3.0, accel=0.0)
        sm2.add_stage('immediate', 60.0, accel=15.0)

        sm3 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm3.add_stage('y_less', 20.0, trigger_val=-20.0, accel=15.0)
        sm3.add_stage('y_less', 30.0, trigger_val=-70.0, accel=10.0)
        sm3.add_stage('y_less', 60.0, trigger_val=-130.0, accel=20.0)

        sm4 = RTB.MultiStageBehaviorMachine(initial_speed=50.0)

        # ==========================================
        # 6. 其他物理模型：【使用最新坐标的】树叶漫天特效系统
        # ==========================================
        leaf_manager = LeafWindManager(
            world=world,
            blueprint_library=bp_lib,
            # ✨ 新起点：14米高空
            spawn_point=carla.Location(x=-4.896, y=-70.758, z=14.275),
            # ✨ 新落点
            target_point=carla.Location(x=12.826, y=-79.987, z=1.801),
            mesh_path='/Game/Carla/Static/RoadTailModel/Maple__leave_SM_Leaf_21.Maple__leave_SM_Leaf_21',
            num_leaves=100
        )

        # ==========================================
        # 7. 预热与初始状态注入
        # ==========================================
        RTB.set_vehicle_initial_speed(v1, target_speed_kmh=60.0, yaw_deg=traj1_raw[0][2])
        RTB.set_vehicle_initial_speed(v2, target_speed_kmh=55.0, yaw_deg=traj2_raw[0][2])
        RTB.set_vehicle_initial_speed(v3_ego, target_speed_kmh=60.0, yaw_deg=traj3_raw[0][2])
        RTB.set_vehicle_initial_speed(v4_truck, target_speed_kmh=50.0, yaw_deg=traj4_raw[0][2])

        print("\n🚀 [仿真系统] 准备完毕，进入主循环。按 Ctrl+C 中断。")

        # ==========================================
        # 8. 仿真主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ----------------------------------------
            # 延迟 3 秒触发高空落叶，彻底防备 Carla 第一帧碰撞网格穿模 Bug
            # ----------------------------------------
            if sim_time >= 3 and not leaf_manager.has_spawned:
                leaf_manager.spawn_leaves()

            # ----- V1 控制逻辑 -----
            if v1 and v1.is_alive:
                loc = v1.get_location()
                speed_kmh = v1.get_velocity().length() * 3.6
                target_spd = sm1.tick(loc, sim_time, dt)
                wp, idx1 = RTB.get_target_waypoint(loc, traj1, idx1, speed_kmh)
                if wp:
                    RTB.apply_pid_control(v1, pid_lon1, pid_lat1, target_spd, wp)
                light_mgrs[v1.id].auto_update_from_control()
                if is_at_goal(loc, v1_goal_x, v1_goal_y):
                    print("[RTB032] V1 reached trajectory end; destroying actor.")
                    destroy_scene_vehicle(v1)
                    v1 = None
                elif RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True):
                    if v1 in actor_list:
                        actor_list.remove(v1)
                    v1 = None

            # ----- V2 控制逻辑 -----
            if v2 and v2.is_alive:
                loc = v2.get_location()
                speed_kmh = v2.get_velocity().length() * 3.6
                target_spd = sm2.tick(loc, sim_time, dt)
                wp, idx2 = RTB.get_target_waypoint(loc, traj2, idx2, speed_kmh)
                if wp:
                    RTB.apply_pid_control(v2, pid_lon2, pid_lat2, target_spd, wp)
                light_mgrs[v2.id].auto_update_from_control()
                if is_at_goal(loc, v2_goal_x, v2_goal_y):
                    print("[RTB032] V2 reached trajectory end; destroying actor.")
                    destroy_scene_vehicle(v2)
                    v2 = None
                elif RTB.check_vehicle_out_of_bounds(v2, carla_map, auto_destroy=True):
                    if v2 in actor_list:
                        actor_list.remove(v2)
                    v2 = None

            # ----- V3 (Ego) 控制逻辑 -----
            if v3_ego and v3_ego.is_alive:
                loc = v3_ego.get_location()
                speed_kmh = v3_ego.get_velocity().length() * 3.6
                target_spd = sm3.tick(loc, sim_time, dt)
                wp, idx3 = RTB.get_target_waypoint(loc, traj3, idx3, speed_kmh)
                if wp:
                    RTB.apply_pid_control(v3_ego, pid_lon3, pid_lat3, target_spd, wp)
                light_mgrs[v3_ego.id].auto_update_from_control()
                if is_at_goal(loc, ego_goal_x, ego_goal_y):
                    print("[RTB032] Ego reached trajectory end; destroying scene actors and exiting.")
                    if leaf_manager:
                        leaf_manager.cleanup()
                        leaf_manager = None
                    RTB.cleanup_actors(client, actor_list)
                    v1 = v2 = v3_ego = v4_truck = None
                    break
                if RTB.check_vehicle_out_of_bounds(v3_ego, carla_map, auto_destroy=True):
                    if v3_ego in actor_list:
                        actor_list.remove(v3_ego)
                    v3_ego = None

            # ----- V4 (Truck) 控制逻辑 -----
            if v4_truck and v4_truck.is_alive:
                loc = v4_truck.get_location()
                speed_kmh = v4_truck.get_velocity().length() * 3.6
                target_spd = sm4.tick(loc, sim_time, dt)
                wp, idx4 = RTB.get_target_waypoint(loc, traj4, idx4, speed_kmh)
                if wp:
                    RTB.apply_pid_control(v4_truck, pid_lon4, pid_lat4, target_spd, wp)
                light_mgrs[v4_truck.id].auto_update_from_control()
                if is_at_goal(loc, v4_goal_x, v4_goal_y):
                    print("[RTB032] V4 reached trajectory end; destroying actor.")
                    destroy_scene_vehicle(v4_truck)
                    v4_truck = None
                elif RTB.check_vehicle_out_of_bounds(v4_truck, carla_map, auto_destroy=True):
                    if v4_truck in actor_list:
                        actor_list.remove(v4_truck)
                    v4_truck = None

            # ----- 自定义高空树叶特效系统推演 -----
            if leaf_manager:
                leaf_manager.tick(sim_time)

            # ---------------- 硬件时钟补齐 ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        RTB.disable_synchronous_mode(world)

        # 安全清理树叶
        if leaf_manager:
            leaf_manager.cleanup()
            print("[特效清理] 自定义树叶碎片系统已回收。")

        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()
