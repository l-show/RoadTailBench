import carla
import time
import math
import numpy as np

# ==========================
# 鍩虹鎺у埗绠楁硶 (PID) - 淇濈暀
# ==========================
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 0.8)

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
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    if wp_nearest is None:
        vehicle.destroy()
        return True
    distance = wp_nearest.transform.location.distance(loc)
    if distance > 6.0:
        vehicle.destroy()
        return True
    return False

class EgoSpeedStateMachine:
    def __init__(self):
        self.speed = 80.0
        self.stage = 0
        self.timer_start = None

    def tick(self, loc, sim_time, dt):
        target_speed = 80.0
        if self.stage == 0 and loc.y >= -80.0:
            self.stage = 1
        if self.stage >= 1:
            target_speed = 40.0

        slow_x, slow_y = -263.698, -55.532
        dist_to_slow_point = math.hypot(loc.x - slow_x, loc.y - slow_y)
        if self.stage == 1 and dist_to_slow_point <= 5.0:
            self.stage = 2
            self.timer_start = sim_time
        if self.stage == 2:
            target_speed = 25.0
            if self.timer_start is not None and sim_time - self.timer_start >= 2.0:
                self.stage = 3
        if self.stage >= 3:
            target_speed = 40.0

        accel_kmh_s = 25.0 if target_speed < self.speed else 15.0
        max_delta = accel_kmh_s * dt
        if self.speed < target_speed:
            self.speed = min(target_speed, self.speed + max_delta)
        else:
            self.speed = max(target_speed, self.speed - max_delta)
        return self.speed

def get_trajectory_target(vehicle_loc, trajectory, current_idx, advance_dist=4.0):
    idx = min(current_idx, len(trajectory) - 1)
    while idx < len(trajectory) - 1:
        tx, ty, _ = trajectory[idx]
        if math.hypot(vehicle_loc.x - tx, vehicle_loc.y - ty) >= advance_dist:
            break
        idx += 1
    tx, ty, _ = trajectory[idx]
    return carla.Location(x=tx, y=ty, z=vehicle_loc.z), idx

# ==========================
# 杞ㄨ抗鏁版嵁瀹氫箟
# ==========================
# 绗竴杈嗚溅 (Mercedes Sprinter) 鐨勮建杩?
V1_TRAJECTORY = [
    (-221.172, -0.776, -157.734), (-221.172, -0.776, -157.734), (-222.172, -1.186, -157.734),
    (-224.453, -2.186, -153.614), (-226.746, -3.354, -151.614), (-228.895, -4.605, -148.104),
    (-230.992, -5.939, -147.396), (-233.053, -7.325, -145.408), (-235.093, -8.742, -144.071),
    (-237.141, -10.287, -142.072), (-239.048, -11.878, -139.003), (-240.907, -13.526, -137.942),
    (-242.761, -15.299, -134.37), (-244.518, -17.169, -132.442), (-246.218, -19.036, -132.3),
    (-247.868, -20.893, -130.733), (-249.496, -22.876, -127.727), (-251.014, -24.943, -125.293),
    (-252.462, -27.057, -122.854), (-253.759, -29.171, -120.271), (-254.903, -31.371, -116.133),
    (-255.979, -33.606, -114.411), (-256.995, -35.959, -112.766), (-257.942, -38.251, -112.125),
    (-258.866, -40.553, -110.84), (-259.721, -42.881, -109.128), (-260.519, -45.273, -107.636),
    (-261.235, -47.69, -105.212), (-261.857, -50.135, -104.066), (-262.444, -52.545, -102.264),
    (-262.957, -55.057, -100.905), (-263.391, -57.5, -99.695), (-263.823, -60.027, -99.695),
    (-264.241, -62.472, -99.695), (-264.622, -65.007, -97.541), (-264.948, -67.467, -97.541),
    (-265.229, -69.973, -94.893), (-265.387, -72.49, -92.662), (-265.479, -75.052, -90.945),
    (-265.521, -77.615, -90.945), (-265.53, -80.096, -88.298), (-265.385, -82.611, -85.777),
    (-265.202, -85.083, -85.777), (-265.013, -87.638, -85.777), (-264.828, -90.151, -85.777),
    (-264.612, -92.582, -83.86), (-264.306, -95.127, -82.579), (-263.94, -97.665, -81.438),
    (-263.537, -100.199, -80.517), (-263.11, -102.645, -79.812), (-262.638, -105.125, -78.888),
    (-262.128, -107.556, -77.96), (-261.604, -110.07, -78.734), (-261.126, -112.552, -79.514),
    (-260.672, -115.08, -80.149), (-260.235, -117.611, -80.219), (-259.802, -120.101, -79.724),
    (-259.33, -122.541, -78.313), (-258.753, -125, -74.709), (-257.988, -127.406, -71.737),
    (-257.191, -129.846, -72.162), (-256.424, -132.297, -72.799), (-255.68, -134.668, -72.021),
    (-254.88, -137.059, -70.515), (-253.987, -139.461, -68.294), (-253.015, -141.741, -66),
    (-251.92, -144.011, -62.369), (-250.715, -146.178, -60.07), (-249.439, -148.304, -58.133),
    (-249.242, -148.62, -58.133), (-249.242, -148.62, -58.133), (-245.136, -154.519, -50.821),
    (-240.117, -160.171, -45.712), (-234.634, -165.383, -40.571), (-228.454, -169.956, -32.553),
    (-221.848, -173.913, -30.428), (-215.36, -177.59, -28.578), (-208.788, -181.123, -27.657),
    (-202.156, -184.55, -27.302), (-195.526, -187.981, -28.095), (-188.877, -191.642, -29.375),
    (-182.32, -195.456, -30.508), (-175.91, -199.213, -30.721), (-169.444, -203.112, -31.936),
    (-162.953, -207.22, -32.152), (-156.417, -211.298, -31.803), (-149.866, -215.36, -31.803),
    (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803),
    (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803),
    (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803)
]

EGO_TRAJECTORY = [
    (-6.278, -120.686, 91.503), (-6.484, -112.858, 91.573), (-6.669, -103.966, 91.739),
    (-6.978, -96.765, 92.482), (-7.091, -94.811, 96.541), (-7.281, -93.576, 100.797),
    (-7.525, -92.350, 102.202), (-7.799, -91.110, 102.692), (-8.084, -89.850, 102.902),
    (-8.689, -87.211, 102.902), (-9.540, -83.495, 102.902), (-10.377, -79.841, 102.902),
    (-11.240, -76.128, 104.130), (-12.225, -72.510, 105.719), (-13.409, -68.887, 110.049),
    (-14.825, -65.416, 113.802), (-16.367, -61.929, 114.299), (-17.933, -58.522, 115.566),
    (-19.594, -55.090, 116.196), (-21.350, -51.707, 119.871), (-23.267, -48.484, 121.566),
    (-25.375, -45.228, 124.410), (-27.553, -42.174, 126.388), (-29.818, -39.185, 127.934),
    (-32.190, -36.281, 130.645), (-34.708, -33.419, 133.034), (-37.417, -30.649, 135.324),
    (-40.128, -28.060, 140.378), (-43.102, -25.674, 141.509), (-46.048, -23.354, 143.034),
    (-49.070, -21.134, 144.317), (-52.191, -18.944, 145.440), (-55.303, -16.853, 146.840),
    (-58.538, -14.835, 149.109), (-61.838, -12.927, 151.368), (-65.174, -11.215, 153.794),
    (-66.015, -10.801, 153.794), (-66.015, -10.801, 153.794), (-66.015, -10.801, 153.794),
    (-66.015, -10.801, 153.794), (-66.015, -10.801, 153.794), (-66.015, -10.801, 153.794),
    (-66.015, -10.801, 153.794), (-69.358, -9.245, 156.179), (-72.922, -7.724, 157.805),
    (-76.426, -6.389, 160.341), (-80.046, -5.197, 162.958), (-83.711, -4.148, 164.873),
    (-87.351, -3.246, 166.774), (-91.002, -2.393, 167.200), (-94.728, -1.587, 167.902),
    (-98.395, -0.801, 167.902), (-102.115, 0.031, 167.269), (-105.834, 0.871, 167.269),
    (-109.553, 1.709, 167.618), (-113.222, 2.482, 168.598), (-116.900, 3.215, 168.737),
    (-120.578, 3.947, 168.737), (-124.379, 4.704, 168.737), (-128.057, 5.437, 168.737),
    (-131.796, 6.189, 168.597), (-135.475, 6.915, 169.017), (-139.156, 7.630, 169.017),
    (-142.897, 8.370, 168.527), (-146.630, 9.140, 168.106), (-150.361, 9.930, 167.756),
    (-154.022, 10.740, 167.476), (-157.683, 11.554, 167.476), (-161.413, 12.344, 168.875),
    (-165.112, 12.952, 171.681), (-168.887, 13.490, 171.964), (-172.615, 13.887, 175.879),
    (-176.362, 13.952, -176.535), (-180.159, 13.605, -174.216), (-183.890, 13.227, -174.216),
    (-187.683, 12.839, -173.653), (-191.395, 12.318, -170.123), (-195.102, 11.439, -164.255),
    (-198.689, 10.352, -161.985), (-202.306, 9.157, -160.853), (-205.901, 7.905, -159.553),
    (-209.397, 6.408, -155.026), (-212.760, 4.775, -152.552), (-216.032, 2.968, -150.685),
    (-219.314, 1.108, -150.195), (-222.601, -0.793, -149.635), (-225.863, -2.735, -149.145),
    (-229.053, -4.674, -147.050), (-232.196, -6.797, -145.558), (-235.309, -8.963, -144.578),
    (-238.328, -11.149, -143.458), (-241.276, -13.424, -140.844), (-244.159, -15.876, -137.346),
    (-246.843, -18.541, -132.975), (-249.272, -21.361, -130.017), (-251.661, -24.294, -127.007),
    (-253.820, -27.322, -123.449), (-255.747, -30.570, -118.395), (-257.377, -33.906, -114.208),
    (-258.836, -37.459, -111.569), (-260.111, -40.954, -107.122), (-261.124, -44.535, -105.548),
    (-262.624, -47.207, -100.287), (-262.624, -47.207, -100.287), (-262.624, -47.207, -100.287), (-262.624, -47.207, -100.287),
    (-262.624, -47.207, -101.016), (-263.865, -53.265, -102.357), (-265.419, -60.609, -101.753), (-266.216, -64.438, -101.753),
    (-266.902, -68.094, -99.202), (-267.362, -71.789, -94.867), (-267.666, -75.564, -92.053), (-267.738, -79.284, -92.005),
    (-267.979, -83.067, -94.550), (-268.303, -86.846, -95.173), (-268.411, -88.034, -95.173), (-268.476, -88.761, -95.173),
    (-268.691, -91.134, -94.895), (-268.869, -93.652, -92.936), (-268.931, -96.173, -89.634), (-268.840, -98.650, -85.962),
    (-268.593, -101.158, -82.354), (-268.206, -103.649, -79.991), (-267.758, -106.135, -79.663), (-267.301, -108.577, -79.335),
    (-266.834, -111.058, -79.335), (-266.283, -113.988, -79.335), (-265.594, -117.650, -79.335), (-264.891, -121.373, -78.964),
    (-264.126, -125.082, -78.308), (-263.319, -128.717, -76.274), (-262.352, -132.377, -74.141), (-261.219, -135.989, -71.260),
    (-259.894, -139.470, -67.459), (-258.336, -142.923, -65.032), (-256.654, -146.242, -61.626), (-254.734, -149.576, -58.251),
    (-252.710, -152.708, -56.598), (-250.488, -155.778, -53.239), (-248.193, -158.715, -50.153), (-245.674, -161.542, -46.298),
    (-243.038, -164.176, -42.601), (-240.181, -166.661, -39.134), (-237.229, -169.031, -37.075), (-234.191, -171.284, -35.146),
    (-231.072, -173.326, -31.141), (-227.776, -175.201, -28.071), (-224.450, -176.884, -25.635), (-221.089, -178.497, -25.635),
    (-217.671, -180.137, -25.635), (-214.255, -181.776, -25.635), (-210.896, -183.387, -25.635), (-207.482, -185.025, -25.635),
    (-204.067, -186.664, -25.635), (-200.652, -188.302, -25.635), (-197.299, -189.925, -26.336), (-193.916, -191.627, -27.155)
]

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # ==========================
    # 銆?銆戝ぉ姘旈厤缃?
    # ==========================
    weather = carla.WeatherParameters(
        cloudiness=100.0, precipitation=100.0, precipitation_deposits=100.0,
        wind_intensity=100.0, sun_azimuth_angle=180.0, sun_altitude_angle=20.0,
        fog_density=80.0, fog_distance=15.0, fog_falloff=0.2, wetness=100.0,
        scattering_intensity=10.0, mie_scattering_scale=0.1, rayleigh_scattering_scale=0.04
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    v1_active = False
    ego_active = False

    try:
        # 鍚屾妯″紡
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_v1 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ==========================
        # 銆?銆戣秴澶ц寖鍥存箍婊戣矾闈㈢敓鎴愪笌鍙鍖?
        # ==========================
        friction_bp = bp_lib.find('static.trigger.friction')
        # 璁剧疆鎽╂摝鍔涙瀬浣?(0.1)锛岃寖鍥?extent 鏄崐灏哄 (鍗虫€婚暱瀹?40x40)
        extent_x, extent_y, extent_z = 20.0, 20.0, 5.0
        friction_bp.set_attribute('friction', '0.1')
        friction_bp.set_attribute('extent_x', str(extent_x))
        friction_bp.set_attribute('extent_y', str(extent_y))
        friction_bp.set_attribute('extent_z', str(extent_z))

        friction_loc = carla.Location(x=-255.862, y=-28.314, z=-2.634)
        friction_trigger = world.spawn_actor(friction_bp, carla.Transform(friction_loc))
        actor_list.append(friction_trigger)
        print("Friction trigger spawned.")

        # # 銆愭柊澧烇細鐢诲嚭鎽╂摝鍔涜Е鍙戝尯鍩熺殑绾㈣壊杈规銆?
        # box_extent = carla.Vector3D(x=extent_x, y=extent_y, z=extent_z)
        # bbox = carla.BoundingBox(friction_loc, box_extent)
        # # 绾㈣壊杈规锛屽瓨娲绘椂闂?1000 绉掞紝鍘氬害 0.5
        #                      life_time=1000.0)

        # ==========================
        # 銆?銆慉ctor 1锛歁ercedes Sprinter
        # ==========================
        bp_v1 = bp_lib.find('vehicle.mercedes.sprinter')
        if bp_v1.has_attribute('color'):
            bp_v1.set_attribute('color', '255,255,255')
        v1_x, v1_y, v1_yaw = V1_TRAJECTORY[0]
        v1_loc = carla.Location(x=v1_x, y=v1_y, z=0.5)
        v1_loc.z = carla_map.get_waypoint(v1_loc).transform.location.z + 0.5
        v1 = world.try_spawn_actor(bp_v1, carla.Transform(v1_loc, carla.Rotation(yaw=v1_yaw)))
        if v1:
            actor_list.append(v1)
            v1_active = True
            print("Sprinter spawned.")

        # ==========================
        # 銆?銆慉ctor 2锛欵go杞?(Citroen C3)
        # ==========================
        bp_ego = bp_lib.find('vehicle.citroen.c3')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '255,255,0')  # 榛勮壊

        ego_start_x, ego_start_y, ego_start_yaw = EGO_TRAJECTORY[0]
        ego_start_loc = carla.Location(x=ego_start_x, y=ego_start_y, z=0.5)
        ego_start_wp = carla_map.get_waypoint(ego_start_loc, project_to_road=True)
        ego_start_loc.z = ego_start_wp.transform.location.z + 0.5
        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        if ego:
            ego_active = True
            light_state = carla.VehicleLightState.HighBeam | carla.VehicleLightState.Fog
            print("Ego (Citroen C3) spawned with role_name=ego.")

        # 绛夊緟杞﹁締钀藉湴璐撮潰
        for _ in range(10):
            world.tick()

        # 銆?銆戣祴浜堝垵濮嬬墿鐞嗛€熷害
        if v1_active:
            v1_speed_ms = 10.0 / 3.6
            v1_yaw_rad = math.radians(v1_yaw)
            v1.set_target_velocity(
                carla.Vector3D(v1_speed_ms * math.cos(v1_yaw_rad), v1_speed_ms * math.sin(v1_yaw_rad), 0.0))

        v1_traj_idx = 0
        ego_traj_idx = 0
        ego_speed_sm = EgoSpeedStateMachine()
        sim_time = 0.0
        print("\nSimulation started. Red box marks the low-friction area; green point marks the active target.")

        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ==========================
            # 杞﹁締 1 (Sprinter) 鍥哄畾杞ㄨ抗
            # ==========================
            if v1_active and v1.is_alive:
                if check_and_handle_out_of_bounds(v1, carla_map):
                    v1_active = False
                elif v1_traj_idx < len(V1_TRAJECTORY):
                    tx, ty, tyaw = V1_TRAJECTORY[v1_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v1.get_location().z)
                    if v1.get_location().distance(target_loc) < 2.5 and v1_traj_idx < len(V1_TRAJECTORY) - 1:
                        v1_traj_idx += 1
                    apply_pid_control(v1, pid_v1['lon'], pid_v1['lat'], 15.0, target_loc)
                else:
                    v1.apply_control(carla.VehicleControl(brake=1.0))
                    v1_active = False

            # ==========================
            # Ego杞?(Citroen C3)锛氬熀浜庡悜閲忔姇褰辩殑鏈€鍙充晶杞﹂亾鍒ゅ埆娉?
            # ==========================
            if not v1_active and not ego_active:
                print("All vehicles finished or were cleaned up.")
                break

            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt, stopping simulation.")
    finally:
        print("\nCleaning up and restoring async settings...")
        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("Cleanup complete.")

if __name__ == '__main__':
    main()
