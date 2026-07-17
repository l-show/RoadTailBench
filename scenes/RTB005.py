# -*- coding: utf-8 -*-

import carla
import time
import math
import numpy as np

# ==========================================
# 轨迹数据清洗 (自动去重)
# ==========================================
def clean_path_points(raw_points):
    cleaned_points = []
    if raw_points:
        cleaned_points.append(raw_points[0])
        for i in range(1, len(raw_points)):
            # 简单的去重逻辑，防止重叠点导致PID计算异常
            if raw_points[i] != raw_points[i - 1]:
                cleaned_points.append(raw_points[i])
    return cleaned_points


# ==========================================
# 目标轨迹数据 (Location_x, Location_y, Rotation_yaw)
# ==========================================
RAW_PATH_POINTS = [
    (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.678),
    (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.678),
    (-11.632, -33.337, -67.678), (-11.632, -33.337, -67.224), (-11.632, -33.337, -66.315),
    (-11.505, -33.624, -65.696), (-11.293, -34.094, -65.697), (-11.081, -34.562, -65.627),
    (-10.867, -35.029, -65.277), (-10.649, -35.504, -65.277), (-10.438, -35.956, -64.786),
    (-10.221, -36.416, -64.786), (-10.0, -36.885, -64.786), (-9.788, -37.33, -63.726),
    (-9.545, -37.793, -61.744), (-9.312, -38.226, -61.744), (-9.066, -38.682, -61.741),
    (-8.832, -39.134, -63.374), (-8.599, -39.599, -63.374), (-8.371, -40.054, -63.374),
    (-8.147, -40.501, -63.374), (-7.916, -40.962, -63.374), (-7.679, -41.427, -61.995),
    (-7.438, -41.875, -61.234), (-7.172, -42.315, -55.895), (-6.865, -42.725, -51.569),
    (-6.541, -43.108, -48.166), (-6.203, -43.478, -47.34), (-5.863, -43.847, -46.788),
    (-5.514, -44.206, -44.813), (-5.146, -44.564, -44.123), (-4.775, -44.922, -44.123),
    (-4.415, -45.272, -44.532), (-4.062, -45.626, -45.431), (-3.718, -45.986, -46.957),
    (-3.366, -46.364, -47.224), (-3.015, -46.745, -47.706), (-2.687, -47.119, -49.057),
    (-2.381, -47.513, -53.703), (-2.079, -47.925, -53.843), (-1.772, -48.333, -52.466),
    (-1.462, -48.736, -52.466), (-1.149, -49.136, -51.24), (-0.829, -49.528, -49.54),
    (-0.494, -49.918, -49.264), (-0.166, -50.299, -49.264), (0.171, -50.683, -48.032),
    (0.524, -51.068, -47.351), (0.871, -51.435, -45.648), (1.237, -51.801, -44.951),
    (1.603, -52.166, -44.814), (1.963, -52.524, -44.814), (2.329, -52.888, -44.814),
    (2.694, -53.251, -44.814), (3.064, -53.613, -44.4), (3.432, -53.974, -44.4),
    (3.794, -54.328, -44.4), (4.147, -54.674, -44.4), (4.515, -55.034, -44.4),
    (4.883, -55.395, -44.4), (5.247, -55.751, -44.4), (5.607, -56.102, -44.128),
    (5.974, -56.455, -43.717), (6.338, -56.797, -43.059), (6.713, -57.146, -43.059),
    (7.083, -57.492, -42.447), (7.48, -57.83, -39.352), (7.873, -58.152, -39.352),
    (8.26, -58.469, -39.352), (8.647, -58.785, -37.599), (9.064, -59.078, -32.134),
    (9.491, -59.339, -31.03), (9.927, -59.601, -30.546), (10.37, -59.859, -29.717),
    (10.823, -60.114, -29.302), (11.271, -60.367, -30.148), (11.712, -60.633, -31.952),
    (12.134, -60.896, -31.952), (12.572, -61.169, -31.952), (13.001, -61.438, -32.232),
    (13.438, -61.714, -32.232), (13.875, -61.989, -32.232), (14.31, -62.263, -32.232),
    (14.746, -62.538, -32.232), (15.181, -62.812, -32.232), (15.605, -63.08, -32.232),
    (16.03, -63.348, -32.232), (16.463, -63.621, -32.232), (16.902, -63.898, -32.232),
    (17.334, -64.17, -32.232), (17.768, -64.444, -32.232), (18.2, -64.716, -32.232),
    (18.626, -64.985, -32.232), (19.052, -65.269, -34.781), (19.464, -65.554, -34.781),
    (19.884, -65.846, -34.781), (20.312, -66.139, -34.095), (20.728, -66.416, -32.506),
    (21.165, -66.687, -30.232), (21.618, -66.937, -27.513), (22.077, -67.174, -27.234),
    (22.528, -67.407, -27.722), (22.977, -67.644, -27.862), (23.414, -67.875, -27.862),
    (23.871, -68.116, -27.862), (24.328, -68.358, -27.862), (24.767, -68.589, -26.297),
    (25.238, -68.811, -23.905), (25.701, -69.013, -23.349), (26.173, -69.216, -23.349),
    (26.646, -69.421, -23.349), (27.115, -69.621, -22.86), (27.593, -69.822, -22.72),
    (28.052, -70.015, -23.137), (28.519, -70.214, -23.137), (28.99, -70.416, -23.137),
    (29.463, -70.618, -23.137), (29.923, -70.815, -23.137), (30.395, -71.016, -23.137),
    (30.866, -71.217, -23.137), (31.34, -71.42, -22.792), (31.805, -71.61, -21.598),
    (32.278, -71.791, -20.489), (32.748, -71.966, -20.419), (33.219, -72.145, -21.536),
    (33.697, -72.337, -22.228), (34.176, -72.533, -22.228), (34.635, -72.72, -22.228),
    (35.101, -72.911, -22.228), (35.579, -73.106, -22.228), (36.056, -73.276, -16.659),
    (36.528, -73.415, -15.573), (37.028, -73.554, -15.573), (37.527, -73.693, -15.573),
    (38.02, -73.83, -15.573), (38.509, -73.968, -16.335), (38.986, -74.125, -19.439),
    (39.47, -74.301, -21.446), (39.953, -74.491, -22.067), (40.416, -74.697, -24.31),
    (40.876, -74.91, -25.714), (41.336, -75.138, -26.363), (41.805, -75.349, -22.485),
    (42.269, -75.535, -18.419), (42.752, -75.685, -17.273), (43.249, -75.84, -17.273),
    (43.727, -75.986, -15.713), (44.226, -76.117, -12.549), (44.729, -76.22, -9.507),
    (45.221, -76.3, -8.454), (45.728, -76.373, -7.765), (46.224, -76.441, -7.765),
    (46.727, -76.509, -7.765), (47.24, -76.576, -6.603), (47.743, -76.634, -6.603),
    (48.236, -76.691, -6.477), (48.736, -76.747, -6.477), (49.239, -76.804, -6.477),
    (49.731, -76.86, -6.477), (50.247, -76.919, -6.477), (50.742, -76.976, -7.104),
    (51.255, -77.057, -13.106), (51.736, -77.184, -15.056), (52.236, -77.31, -13.475),
    (52.741, -77.422, -11.649), (53.238, -77.52, -11.161), (53.743, -77.622, -12.876),
    (54.243, -77.753, -15.305), (54.721, -77.884, -15.376), (55.221, -78.021, -15.25),
    (55.703, -78.152, -15.25), (56.181, -78.293, -19.389), (56.664, -78.463, -19.389),
    (57.146, -78.637, -19.897), (57.616, -78.807, -19.897), (58.089, -78.978, -19.897),
    (58.53, -79.235, -39.786), (58.918, -79.573, -41.137), (59.297, -79.904, -41.137),
    (59.681, -80.24, -41.137), (60.071, -80.581, -41.137), (60.456, -80.917, -41.137),
    (60.834, -81.247, -41.137), (61.206, -81.605, -46.419), (61.548, -81.964, -46.291),
    (61.905, -82.338, -46.291), (62.262, -82.711, -46.291), (62.613, -83.079, -46.291),
    (62.958, -83.44, -46.291), (63.309, -83.808, -46.291), (63.66, -84.175, -46.291),
    (64.007, -84.539, -46.421), (64.357, -84.911, -47.005), (64.704, -85.294, -48.311),
    (65.036, -85.668, -48.311), (65.369, -86.041, -48.311), (65.71, -86.428, -48.694),
    (66.045, -86.811, -48.95), (66.378, -87.194, -48.95), (66.703, -87.596, -54.247),
    (67.006, -88.014, -53.862), (67.306, -88.424, -53.862), (67.606, -88.835, -53.862),
    (67.91, -89.252, -53.862), (68.205, -89.656, -54.13), (68.496, -90.072, -55.754),
    (68.757, -90.499, -59.032), (69.019, -90.935, -59.032), (69.276, -91.364, -59.032),
    (69.542, -91.807, -59.032), (69.798, -92.236, -59.288), (70.05, -92.668, -59.93),
    (70.303, -93.118, -61.243), (70.542, -93.557, -61.5), (70.788, -94.01, -61.5),
    (71.031, -94.456, -61.5), (71.268, -94.914, -64.726), (71.484, -95.375, -64.984),
    (71.695, -95.828, -65.113), (71.912, -96.297, -65.499), (72.126, -96.767, -65.499),
    (72.327, -97.225, -66.756), (72.531, -97.699, -66.756), (72.732, -98.166, -66.756),
    (72.936, -98.641, -66.756), (73.14, -99.116, -66.756), (73.344, -99.59, -66.756),
    (73.548, -100.065, -66.756), (73.752, -100.539, -66.756), (73.945, -101.009, -68.173),
    (74.135, -101.49, -68.688), (74.317, -101.955, -68.688), (74.504, -102.437, -69.774),
    (74.664, -102.919, -71.9), (74.819, -103.395, -71.9), (74.98, -103.886, -71.9),
    (75.14, -104.377, -71.9), (75.298, -104.861, -71.9), (75.459, -105.352, -71.9),
    (75.613, -105.828, -72.158), (75.766, -106.304, -72.158), (75.917, -106.789, -73.388),
    (76.062, -107.275, -73.447), (76.201, -107.755, -74.221), (76.338, -108.253, -74.865),
    (76.473, -108.752, -74.865), (76.605, -109.242, -74.865), (76.699, -109.587, -74.865),
    (76.699, -109.587, -74.865), (76.699, -109.587, -74.865), (76.699, -109.587, -74.865),
    (76.699, -109.587, -74.865), (76.699, -109.587, -74.865), (76.699, -109.587, -74.865),
    (76.699, -109.587, -74.865), (77.154, -111.613, -75.398), (77.154, -111.613, -75.398), (77.154, -111.613, -75.398), (77.154, -111.613, -75.398),
    (77.154, -111.613, -75.398), (77.623, -113.492, -76.096), (78.538, -117.195, -76.375), (79.408, -120.909, -76.933),
    (80.256, -124.562, -76.933), (81.113, -128.277, -77.492), (81.938, -131.999, -77.492), (82.764, -135.721, -77.491),
    (83.576, -139.382, -77.491), (84.402, -143.104, -77.490), (84.917, -145.423, -77.489), (84.917, -145.423, -77.489),
    (84.917, -145.423, -77.489), (84.917, -145.423, -77.489), (84.917, -145.423, -77.489), (84.917, -145.423, -77.489),
    (84.917, -145.423, -77.489)
]
VEHICLE_PATH_POINTS = clean_path_points(RAW_PATH_POINTS)


RAW_EGO_PATH_POINTS = [
    (-23.716, 35.122, -93.747), (-23.716, 35.122, -93.747), (-23.716, 35.122, -93.747), (-23.716, 35.122, -93.747),
    (-23.716, 35.122, -93.747), (-23.751, 34.641, -94.307), (-23.789, 34.137, -94.097), (-23.824, 33.621, -93.887),
    (-23.878, 32.828, -93.887), (-24.077, 29.768, -93.130), (-24.255, 25.999, -92.203), (-24.395, 22.194, -91.993),
    (-24.515, 18.359, -91.783), (-24.580, 14.610, -90.289), (-24.553, 10.817, -88.300), (-24.364, 7.032, -86.422),
    (-24.016, 3.230, -83.531), (-23.514, -0.513, -80.964), (-22.823, -4.330, -78.096), (-21.927, -8.039, -74.436),
    (-20.801, -11.695, -72.186), (-20.108, -13.849, -72.186), (-19.726, -15.038, -72.186), (-19.335, -16.252, -71.836),
    (-18.920, -17.453, -69.635), (-18.482, -18.604, -69.056), (-18.021, -19.808, -68.986), (-17.538, -20.997, -67.270),
    (-17.036, -22.164, -66.122), (-16.519, -23.328, -66.052), (-16.007, -24.471, -65.843), (-15.482, -25.641, -65.843),
    (-14.971, -26.776, -65.703), (-14.443, -27.935, -65.355), (-13.914, -29.083, -64.932), (-13.367, -30.220, -63.903),
    (-12.808, -31.361, -63.903), (-12.250, -32.500, -63.903), (-11.698, -33.638, -64.392), (-11.153, -34.785, -64.671),
    (-10.599, -35.953, -64.327), (-10.051, -37.080, -64.044), (-9.476, -38.219, -61.507), (-8.883, -39.280, -59.833),
    (-8.215, -40.397, -58.253), (-7.534, -41.479, -56.873), (-6.812, -42.497, -52.266), (-5.986, -43.487, -48.082),
    (-5.127, -44.400, -45.808), (-4.242, -45.310, -45.808), (-3.365, -46.213, -45.808), (-2.491, -47.117, -46.699),
    (-1.634, -48.058, -48.211), (-0.810, -48.975, -47.414), (0.060, -49.911, -47.071), (0.933, -50.850, -47.071),
    (1.799, -51.781, -47.349), (2.658, -52.713, -46.871), (3.533, -53.625, -45.571), (4.422, -54.519, -43.738),
    (5.349, -55.395, -42.983), (6.288, -56.236, -40.872), (7.254, -57.048, -39.154), (8.244, -57.835, -37.631),
    (9.241, -58.601, -36.941), (10.255, -59.356, -36.594), (11.267, -60.099, -36.114), (12.279, -60.834, -35.576),
    (13.324, -61.557, -33.322), (14.386, -62.240, -32.174), (15.477, -62.926, -32.174), (16.571, -63.595, -31.076),
    (17.653, -64.226, -29.951), (18.769, -64.869, -29.951), (19.864, -65.497, -29.263), (20.981, -66.115, -28.779),
    (22.877, -67.156, -28.779), (26.231, -68.949, -26.136), (29.730, -70.515, -22.737), (33.213, -71.960, -21.531),
    (36.850, -73.271, -17.698), (40.457, -74.355, -16.331), (44.141, -75.359, -14.364), (47.834, -76.222, -12.404),
    (51.604, -77.061, -13.689), (55.197, -78.120, -19.979), (58.592, -79.797, -33.936), (61.577, -82.071, -40.507),
    (64.361, -84.715, -47.442), (66.807, -87.618, -50.774), (69.103, -90.659, -56.659), (71.010, -93.885, -62.406),
    (72.591, -97.350, -68.143), (74.000, -100.890, -69.170), (75.249, -104.491, -72.280), (76.297, -108.092, -74.389),
    (77.323, -111.764, -74.389), (78.330, -115.441, -75.022), (79.231, -119.145, -77.426), (80.021, -122.810, -78.667)
]
EGO_PATH_POINTS = clean_path_points(RAW_EGO_PATH_POINTS)


# ==========================================
# PID 控制器类 (保持原逻辑)
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController2:
    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint, vehicle_transform):
        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint[0] - v_begin.x, waypoint[1] - v_begin.y, 0.0])
        norm_w = np.linalg.norm(w_vec)
        if norm_w < 0.1:
            return 0.0
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0
        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class MultiStageBehaviorMachine:
    def __init__(self, initial_speed=0.0):
        self.current_speed = initial_speed
        self.stages = []
        self.current_idx = 0
        self.stage_enter_time = None

    def add_stage(self, trigger_type, target_speed, trigger_val=None, accel=15.0, tolerance=2.0):
        self.stages.append({
            "type": trigger_type,
            "val": trigger_val,
            "target_speed": target_speed,
            "accel": accel,
            "tolerance": tolerance,
        })

    def _check_trigger(self, stage, vehicle_loc, current_time):
        trigger_type = stage["type"]
        trigger_val = stage["val"]
        if trigger_type == "immediate":
            return True
        if trigger_type == "time":
            if self.stage_enter_time is None:
                self.stage_enter_time = current_time
            if current_time - self.stage_enter_time >= trigger_val:
                self.stage_enter_time = None
                return True
            return False
        if trigger_type == "point":
            return math.hypot(vehicle_loc.x - trigger_val[0], vehicle_loc.y - trigger_val[1]) <= stage["tolerance"]
        if trigger_type == "x_greater":
            return vehicle_loc.x > trigger_val
        if trigger_type == "x_less":
            return vehicle_loc.x < trigger_val
        if trigger_type == "y_greater":
            return vehicle_loc.y > trigger_val
        if trigger_type == "y_less":
            return vehicle_loc.y < trigger_val
        return False

    def tick(self, vehicle_loc, current_time, dt):
        if self.stages and self.current_idx < len(self.stages):
            current_stage = self.stages[self.current_idx]
            if self._check_trigger(current_stage, vehicle_loc, current_time):
                print(f"[Ego状态机] 触发阶段 {self.current_idx}: {current_stage['type']} -> {current_stage['target_speed']} km/h")
                self.current_idx += 1
                self.stage_enter_time = None

        if self.current_idx == 0:
            return self.current_speed

        active_stage = self.stages[max(0, self.current_idx - 1)]
        target_speed = active_stage["target_speed"]
        accel_rate = active_stage["accel"]
        if accel_rate > 0.0:
            if self.current_speed < target_speed:
                self.current_speed = min(target_speed, self.current_speed + accel_rate * dt)
            elif self.current_speed > target_speed:
                self.current_speed = max(target_speed, self.current_speed - accel_rate * dt)
        return self.current_speed


# ==========================================
# 辅助函数
# ==========================================
def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )


def get_target_waypoint(actor_loc, path_points, lookahead_dist=4.0):
    min_dist = float('inf')
    closest_index = 0
    # 找到最近的点
    for i, p in enumerate(path_points):
        dist = math.sqrt((p[0] - actor_loc.x) ** 2 + (p[1] - actor_loc.y) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_index = i

    # 向前寻找 lookahead_dist 距离的点
    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1 = path_points[i]
        p2 = path_points[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist:
            break
    return path_points[target_index]


def destroy_if_out_of_road(vehicle, carla_map, actor_list, threshold_dist=6.0):
    if not vehicle or not vehicle.is_alive:
        return True

    loc = vehicle.get_location()
    wp_exact = carla_map.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving)
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)

    out_of_road = False
    if wp_nearest is None:
        out_of_road = True
        print(f"[道路守护] 车辆 [{vehicle.id}] 无法投影到可行驶道路，销毁 actor。")
    elif wp_exact is None:
        dist_to_road = wp_nearest.transform.location.distance(loc)
        if dist_to_road > threshold_dist:
            out_of_road = True
            print(f"[道路守护] 车辆 [{vehicle.id}] 偏离可行驶道路 {dist_to_road:.1f}m，销毁 actor。")

    if out_of_road:
        try:
            if vehicle in actor_list:
                actor_list.remove(vehicle)
            vehicle.destroy()
        except Exception as exc:
            print(f"[道路守护] 销毁车辆失败: {exc}")
        return True
    return False


# ==========================================
# 主程序
# ==========================================


# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (80.021, -122.810)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.chevrolet.impala'
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
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # ---------------------------------------------------------
    # 设置 Traffic Manager (TM)
    # ---------------------------------------------------------
    tm_port = 8000
    tm = client.get_trafficmanager(tm_port)
    # 重要：为了和世界同步模式配合，TM 也要设为同步
    tm.set_synchronous_mode(True)
    # 设置全局混合物理模式 (可选，减少计算量，半径内物理全开)
    # tm.set_hybrid_physics_mode(True)

    # ---------------------------------------------------------
    # 设置天气参数
    # ---------------------------------------------------------
    weather = carla.WeatherParameters(
        cloudiness=25.0, precipitation=40.0, precipitation_deposits=70.0,
        wind_intensity=10.0, sun_azimuth_angle=115.0, sun_altitude_angle=14.0,
        fog_density=2.0, fog_distance=0.0, fog_falloff=0.0, wetness=40.0,
        scattering_intensity=5.0, mie_scattering_scale=0.0, rayleigh_scattering_scale=0.3,
        dust_storm=0.0
    )
    world.set_weather(weather)
    print("天气参数已更新。")

    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        # ---------------------------------------------------------
        # 1. 生成 原始车辆 (vehicle.volkswagen.t2_2021)
        # ---------------------------------------------------------
        vehicle_bp_name = 'vehicle.volkswagen.t2_2021'
        bp_vehicle = bp_lib.find(vehicle_bp_name)
        initial_point = VEHICLE_PATH_POINTS[0]
        trans_vehicle = get_transform(x=initial_point[0], y=initial_point[1], z=0.5,
                                      yaw=initial_point[2])
        vehicle = world.try_spawn_actor(bp_vehicle, trans_vehicle)
        if vehicle:
            actor_list.append(vehicle)
            vehicle.set_simulate_physics(True)
            print(f"{vehicle_bp_name} 生成成功")

            # 初始化 VW 控制器
            lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0, dt=settings.fixed_delta_seconds)
            lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2, dt=settings.fixed_delta_seconds)

            # 开启车灯
            base_light_state = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
            vehicle.set_light_state(carla.VehicleLightState(base_light_state))
        else:
            print(f"无法生成车辆 {vehicle_bp_name}")

        # ---------------------------------------------------------
        # 2. 生成 Ego 车辆 (vehicle.chevrolet.impala)
        # ---------------------------------------------------------
        ego_bp = bp_lib.find('vehicle.chevrolet.impala')
        ego_bp.set_attribute('role_name', 'ego')

        ego_initial_point = EGO_PATH_POINTS[0]
        ego_spawn_trans = get_transform(
            x=ego_initial_point[0],
            y=ego_initial_point[1],
            z=0.5,
            yaw=ego_initial_point[2],
        )

        ego_vehicle = world.try_spawn_actor(ego_bp, ego_spawn_trans)

        if ego_vehicle:
            actor_list.append(ego_vehicle)
            print("Ego Vehicle 生成成功，正在等待物理落地...")

            # 【关键修改 1】: 刚生成时不要给速度，也不要开自动驾驶
            # 此时车是悬空的 (z=0.5)，让它先掉下来并在原地停稳
        else:
            print("无法生成 Ego Vehicle，请检查位置是否冲突。")

        # ==========================================
        # 等待物理系统稳定 (原地落地)
        # ==========================================
        # 这 10-20 帧用于让车从 z=0.5 掉到地面并停稳悬挂
        for _ in range(20):
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            # 在这里不需要 sleep，因为我们在同步模式下只关心物理计算步数
            # 如果为了看清落地过程，可以加一点 sleep，但实际运行不需要

        print("物理系统已稳定，开始施加初始速度并接管...")

        # ==========================================
        # 【核心修复】：物理稳定后，统一强制开启车灯
        # ==========================================
        # 定义灯光状态：位置灯 + 远光灯 (HighBeam 亮度更高)
        target_light_state = carla.VehicleLightState(
            carla.VehicleLightState.Position | carla.VehicleLightState.HighBeam
        )

        # 1. 设置 VW T2 车灯
        if vehicle and vehicle.is_alive:
            vehicle.set_light_state(target_light_state)
            print("VW T2 车灯已开启 (Position + HighBeam)")

        # 2. 设置 Ego Impala 车灯
        if ego_vehicle and ego_vehicle.is_alive:
            ego_vehicle.set_light_state(target_light_state)
            print("Ego Impala 车灯已开启 (Position + HighBeam)")

            # (可选) 告诉 TM 不要接管这辆车的灯光，防止 TM 觉得白天不需要开灯而自动关掉
            # 注意：如果 TM 自动驾驶接管了灯光，它可能会覆盖你的设置。
            # 下面这行指令在 0.9.15 中可以禁止 TM 更改灯光状态：
            tm.update_vehicle_lights(ego_vehicle, False)

        # ==========================================
        # 【关键修改 2】: 落地后启用 Ego PID 轨迹跟随
        # ==========================================
        ego_lon_controller = None
        ego_lat_controller = None
        if ego_vehicle:
            # -----------------------------------------------------
            # 1. 物理层面的速度初始化
            # -----------------------------------------------------
            # 设定目标初始速度为 65 km/h
            initial_speed_kmh = 65.0
            # Carla 物理引擎使用国际单位制 (m/s)，需要转换：65 / 3.6 ≈ 18.06 m/s
            initial_speed_mps = initial_speed_kmh / 3.6

            yaw_rad = math.radians(ego_initial_point[2])

            # 根据三角函数分解速度向量
            # vx = 速度 * cos(角度)，vy = 速度 * sin(角度)
            vx = initial_speed_mps * math.cos(yaw_rad)
            vy = initial_speed_mps * math.sin(yaw_rad)

            # 【物理注入】：直接修改车辆刚体的线性速度
            # z=0.0 表示不给垂直方向的速度，让车辆紧贴地面平滑滑行
            # 这行代码让车辆在这一帧瞬间获得 65km/h 的动能
            ego_vehicle.set_target_velocity(carla.Vector3D(x=vx, y=vy, z=0.0))
            ego_vehicle.set_autopilot(False, tm_port)
            ego_lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0, dt=settings.fixed_delta_seconds)
            ego_lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2, dt=settings.fixed_delta_seconds)
            print(f"Ego PID 轨迹控制已启用，共 {len(EGO_PATH_POINTS)} 个路径点。")

        print("场景运行中...")
        sim_time = 0.0
        target_speed_vw_kmh = 25.0
        sm_ego = MultiStageBehaviorMachine(initial_speed=65.0)
        sm_ego.add_stage("y_less", target_speed=25.0, trigger_val=-20.0, accel=20.0)

        # 控制开关：是否使用外部模型控制 Ego 车辆
        # 如果为 True，则关闭 Autopilot 并应用下方计算的 Control
        # 如果为 False，则继续使用 Traffic Manager 的车道保持
        ENABLE_EXTERNAL_CONTROL = False

        # 主循环
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += settings.fixed_delta_seconds

            # -------------------------------------------------
            # A. 控制 VW T2 (PID 轨迹跟随)
            # -------------------------------------------------
            if vehicle and vehicle.is_alive and not destroy_if_out_of_road(vehicle, carla_map, actor_list):
                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                current_speed_kmh = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
                target_wp = get_target_waypoint(tf.location, VEHICLE_PATH_POINTS, lookahead_dist=5.0)

                throttle_output = lon_controller.run_step(target_speed_vw_kmh, current_speed_kmh)
                steer_output = lat_controller.run_step(target_wp, tf)

                control = carla.VehicleControl()
                control.steer = steer_output
                if throttle_output >= 0.0:
                    control.throttle = throttle_output
                    control.brake = 0.0
                else:
                    control.throttle = 0.0
                    control.brake = abs(throttle_output)
                vehicle.apply_control(control)

            # -------------------------------------------------
            # B. 控制 Ego Impala (PID 轨迹跟随，预留外部接口)
            # -------------------------------------------------
            if ego_vehicle and ego_vehicle.is_alive and not destroy_if_out_of_road(ego_vehicle, carla_map, actor_list):
                # 可以在这里获取传感器数据、图像等传给 UniAD
                # ego_transform = ego_vehicle.get_transform()
                # ego_velocity = ego_vehicle.get_velocity()

                # ==================================================
                # TODO: UniAD / External Model Interface
                # ==================================================
                if ENABLE_EXTERNAL_CONTROL:
                    # 1. 确保关闭 TM 自动驾驶
                    # (如果在循环外已经关闭，这里可以省略判断，但为了安全建议检查)
                    # ego_vehicle.set_autopilot(False, tm_port)

                    # 2. 接收模型输出的控制量
                    # 假设模型输出为: model_steer, model_throttle, model_brake
                    model_steer = 0.0  # [-1, 1]
                    model_throttle = 0.5  # [0, 1]
                    model_brake = 0.0  # [0, 1]

                    # 3. 应用控制
                    ego_control = carla.VehicleControl()
                    ego_control.steer = model_steer
                    ego_control.throttle = model_throttle
                    ego_control.brake = model_brake
                    ego_control.manual_gear_shift = False
                    ego_vehicle.apply_control(ego_control)

                else:
                    tf = ego_vehicle.get_transform()
                    vel = ego_vehicle.get_velocity()
                    current_speed_kmh = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
                    target_speed_ego_kmh = sm_ego.tick(tf.location, sim_time, settings.fixed_delta_seconds)
                    target_wp = get_target_waypoint(tf.location, EGO_PATH_POINTS, lookahead_dist=6.0)

                    throttle_output = ego_lon_controller.run_step(target_speed_ego_kmh, current_speed_kmh)
                    steer_output = ego_lat_controller.run_step(target_wp, tf)

                    ego_control = carla.VehicleControl()
                    ego_control.steer = steer_output
                    if throttle_output >= 0.0:
                        ego_control.throttle = throttle_output
                        ego_control.brake = 0.0
                    else:
                        ego_control.throttle = 0.0
                        ego_control.brake = abs(throttle_output)
                    ego_vehicle.apply_control(ego_control)

            # ==============================
            # 同步时间控制
            # ==============================
            compute_time = time.time() - start_time
            if compute_time < settings.fixed_delta_seconds:
                time.sleep(settings.fixed_delta_seconds - compute_time)

    except Exception as e:
        print(f"发生异常: {e}")
    except KeyboardInterrupt:
        print("\n用户停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        # 恢复异步模式
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        # 恢复 TM 模式 (虽然 destroy 后通常不需要，但为了保险)
        if 'tm' in locals():
            tm.set_synchronous_mode(False)

        # 清理车辆
        if actor_list:
            client.apply_batch([carla.command.DestroyActor(a) for a in actor_list])
        print("清理完成。")


if __name__ == '__main__':
    main()
