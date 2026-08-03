"""
RoadTailBenchInit: 下一代长尾场景测试数据集 RoadTailBench 数据集核心标准化函数库
作用: 提供自动驾驶长尾场景仿真中通用的数学计算、车辆/行人控制、灯光系统、风力特效系统、环境守护等标准化接口。
优化: [本次更新] 增加字符串轨迹解析、带安全状态的轨迹预瞄、平滑加减速状态机、车辆驻车器、天气一键配置接口。
"""

import carla
import math
import time
import numpy as np
import random

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except ImportError:
    pass

# ==========================================
# 1. 坐标、几何与数学辅助函数(Geometry & Math Utils)
# ==========================================

def clamp(v, min_val, max_val):
    """
    限制数值范围
    【用法】speed = clamp(speed, 0.0, 5.0)
    """
    return max(min_val, min(max_val, v))

def _extract_xy(p):
    """【内部工具】多态兼容:兼容传入元组、Location或Transform对象,一律提取出X与Y"""
    if isinstance(p, carla.Transform):
        return p.location.x, p.location.y
    elif isinstance(p, carla.Location):
        return p.x, p.y
    else:
        return p[0], p[1]
    

def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    """根据给定的坐标和欧拉角生成 carla.Transform 对象
    【用法】trans = get_transform(-27.0, 54.8, 1.0, yaw=-90.0)
    """
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )

def calculate_velocity_vector(speed, rotation):
    """
    根据速度大小和车辆朝向计算三维速度向量
    【用法】vel_vec = calculate_velocity_vector(10.0, ego.get_transform().rotation)
    """
    pitch_rad = math.radians(rotation.pitch)
    yaw_rad = math.radians(rotation.yaw)
    x = speed * math.cos(yaw_rad) * math.cos(pitch_rad)
    y = speed * math.sin(yaw_rad) * math.cos(pitch_rad)
    z = speed * math.sin(pitch_rad)
    return carla.Vector3D(x=x, y=y, z=z)

def get_angle_diff(yaw1, yaw2):
    """
    解决-180到180度跳变问题,用于防止方向盘突变
    CARLA里角度不是线性的,是“环形”的,比如-179和179实际上只差2度,但直接相减会得到-358度
    把“任意角度差”映射到 [-180°, 180°] 最短路径
    """
    diff =(yaw1 - yaw2 + 180) % 360 - 180
    return abs(diff)

# ==========================================
# 2. 轨迹处理
# ==========================================

def clean_path_points(raw_points):
    """
    清洗普通轨迹数据:去除连续重复的点
    【用法】cleaned_path = clean_path_points(RAW_PATH_LIST)
    """
    cleaned_points = []
    if raw_points:
        cleaned_points.append(raw_points[0])
        for i in range(1, len(raw_points)):
            if raw_points[i] != raw_points[i - 1]:
                cleaned_points.append(raw_points[i])
    return cleaned_points


def clean_trajectory(raw_points, min_dist=0.5):
    """
    【一站式轨迹清洗与稀疏化工具(去重/去噪)】
    功能:
        1. 默认去除距离极近的噪点（防止 PID 控制器在连续密集点处原地抽搐）。
        2. 如果仅仅想去除“完全重合”的错误点,只需传入 min_dist=1e-5。
        3. [多态兼容] 无论传入的是(X,Y,Z,YAW) 元组,还是 carla.Location 对象,都能原样保留数据结构并返回。
        
    使用场景:从自动驾驶数据集/日志中提取的轨迹往往存在冗余点或原点停顿点,喂给控制器前必须清洗。
    基于两点间的欧式距离进行清洗,防止车辆/行人循迹时因坐标过于密集发生卡顿。
    【用法】:
        # 1. 常规清洗去噪(两点间距小于0.1米则丢弃)
        CLEANED_TRAJ = clean_trajectory(RAW_TRAJ, min_dist=0.1)
        # 2. 仅去除完全相同的相邻点
        CLEANED_TRAJ = clean_trajectory(RAW_TRAJ, min_dist=1e-5)
    """
    if not raw_points: return []
    cleaned_points = [raw_points[0]]
    for pt in raw_points[1:]:
        last_pt = cleaned_points[-1]
        
        # 多态提取 XY 进行平面距离计算
        x1, y1 = _extract_xy(pt)
        x2, y2 = _extract_xy(last_pt)
        
        if math.hypot(x1 - x2, y1 - y2) >= min_dist:
            # 距离大于阈值,判定为有效行进点,原样追加（保留传入时的原始对象或元组维度）
            cleaned_points.append(pt)
            
    return cleaned_points


def parse_string_trajectory(data_str, min_dist=0.1):
    """
    【文本轨迹解析器(自带清洗)】
    功能:将从论文、数据集或日志中复制的杂乱多行文本,一键解析为标准的 Python 元组列表。
    防呆设计:自动兼容空格分割或逗号分割,自动提取 X, Y, [Z], [YAW],并直接调用 clean_trajectory 进行去重。
    
    【用法】:
        traj_str = '''
            10.5 20.3 -0.1
            10.5, 20.3, -0.1
            11.0 21.0 -0.1
        '''
        MY_TRAJ = parse_string_trajectory(traj_str, min_dist=0.5)
    """
    raw_points = []
    # 替换逗号为空格,防止某些数据集以 csv 格式存储
    data_str = data_str.replace(',', ' ')
    lines = data_str.strip().split('\n')
    
    for line in lines:
        parts = line.split()
        if len(parts) >= 2:
            # 将每一行转化为(X, Y) 或(X, Y, Z...) 元组
            pt = tuple(float(p) for p in parts)
            raw_points.append(pt)
            
    # 解析完毕后,直接交给统一清洗器进行处理
    return clean_trajectory(raw_points, min_dist)

def interpolate_trajectory(raw_points, interval=1.0):
    """
    【稀疏轨迹插值稠密化工具(无Numpy依赖版)】
    功能:将断点较多、间距巨大的稀疏轨迹（例如仅有转角锚点）,按照指定间距（interval）进行线性插值,生成平滑密集的轨迹。
    使用场景:当使用类似 A* 算法生成的路线只有路口拐点时,必须使用此函数将路线稠密化,否则汽车会直接“切内道”越野。
    
    注意:此操作与 clean_trajectory 作用完全相反。
         - clean: 太多太密 -> 稀疏化
         - interpolate: 太少太远 -> 稠密化
         
    【用法】:
        SPARSE_PATH = [(0,0),(0,50),(50,50)]
        DENSE_PATH = interpolate_trajectory(SPARSE_PATH, interval=2.0)
    """
    if not raw_points: return []
    dense_path = []
    
    for i in range(len(raw_points) - 1):
        p1 = raw_points[i]
        p2 = raw_points[i + 1]
        
        # 多态提取坐标
        x1, y1 = _extract_xy(p1)
        x2, y2 = _extract_xy(p2)
        
        # 兼容性尝试提取 Z 轴,若没有 Z 则默认为 0.0
        z1 = getattr(p1, 'z', p1[2] if isinstance(p1,(tuple, list)) and len(p1) > 2 else 0.0)
        z2 = getattr(p2, 'z', p2[2] if isinstance(p2,(tuple, list)) and len(p2) > 2 else 0.0)
        
        dist = math.hypot(x2 - x1, y2 - y1)
        num_points = max(1, int(dist / interval))
        
        for j in range(num_points):
            ratio = j / num_points
            nx = x1 +(x2 - x1) * ratio
            ny = y1 +(y2 - y1) * ratio
            nz = z1 +(z2 - z1) * ratio
            dense_path.append((nx, ny, nz))
            
    # 强力补全:确保将原始数组的最后一个目标点精确加入
    last_p = raw_points[-1]
    lx, ly = _extract_xy(last_p)
    lz = getattr(last_p, 'z', last_p[2] if isinstance(last_p,(tuple, list)) and len(last_p) > 2 else 0.0)
    dense_path.append((lx, ly, lz))
    
    return dense_path

def build_transforms_from_trajectory(raw_path_points, z_offset=0.5):
    """
    【工业级轨迹转换器:坐标到 Transform 转换与航向角几何计算】
    功能:输入只有(X,Y,Z) 的裸坐标列表,自动去重,并利用前后点的几何关系反推计算出精确的航向角(Yaw)。
    【用法】:MY_TRANSFORMS = build_transforms_from_trajectory(DENSE_PATH)
    """
    path_transforms = []
    n = len(raw_path_points)
    for i in range(n):
        p = raw_path_points[i]
        yaw = 0.0
        found_next = False
        # 寻找下一个非重合点(距离>0.1米)以计算正确的航向角
        for j in range(i + 1, n):
            next_p = raw_path_points[j]
            dx, dy = next_p[0] - p[0], next_p[1] - p[1]
            if math.hypot(dx, dy) > 0.1:
                yaw = math.degrees(math.atan2(dy, dx))
                found_next = True
                break
        
        # 如果后面全重合,则沿用上一次的角度
        if not found_next and i > 0:
            yaw = path_transforms[-1].rotation.yaw
            
        z = p[2] + z_offset if len(p) > 2 else z_offset
        path_transforms.append(carla.Transform(
            carla.Location(x=p[0], y=p[1], z=z),
            carla.Rotation(yaw=yaw)
        ))
    return path_transforms

# ==========================================
# 【彻底修复版】：三合一终极预热防滑器
# 包含：1. 底盘下落稳定  2. 物理速度注入  3. 轮胎抓地力建立期
# ==========================================
def warmup_and_inject_speeds(world, vehicle_configs, warmup_ticks=10, stable_ticks=10):
    """
    参数:
        - warmup_ticks: 纯自由落体帧数，稳定避震器（默认20帧）。
        - stable_ticks: 【核心防画龙】注入速度后，强行锁死方向盘直行的帧数（默认10帧，即0.5秒），等待轮胎建立抓地力。
    """
    if not vehicle_configs: return

    # 1. 纯物理自由落体预热（此时没有速度，稳定底盘）
    print(f"[预热模块] 1/3 正在静止下落稳定底盘 ({warmup_ticks} 帧)...")
    for _ in range(warmup_ticks):
        world.tick()

    # 2. 瞬间赋予物理初速度
    print("[预热模块] 2/3 正在注入物理初速度...")
    for config in vehicle_configs:
        veh = config.get('vehicle')
        spd = config.get('speed', 0.0)
        yaw = config.get('yaw')
        if veh and veh.is_alive:
            # 假设你的 RTB 库里有这个方法，或者直接用 veh.set_target_velocity()
            import RoadTailBenchInitV9 as RTB
            RTB.set_vehicle_initial_speed(veh, target_speed_kmh=spd, yaw_deg=yaw)

    # 3. 【防画龙最核心的一步】：抓地力建立期 (Tire Friction Stabilization)
    # 注入速度后，绝对不能立刻交给 PID！必须让世界推演 10 帧左右，同时强行锁死方向盘为 0。
    print(f"[预热模块] 3/3 正在锁死方向盘，等待轮胎建立抓地力 ({stable_ticks} 帧)...")
    for _ in range(stable_ticks):
        for config in vehicle_configs:
            veh = config.get('vehicle')
            spd = config.get('speed', 0.0)
            if veh and veh.is_alive:
                # 强行下发绝对笔直的指令，给点油门维持初速度
                throttle_val = 0.5 if spd > 0 else 0.0
                ctrl = carla.VehicleControl(steer=0.0, throttle=throttle_val, brake=0.0)
                veh.apply_control(ctrl)
        world.tick() # 让引擎在直线行驶状态下推演摩擦力

    print("[预热模块] 预热完毕！轮胎抓地力已建立，即将移交 PID 接管。")

# ==========================================
# 轨迹预瞄跟踪
# ==========================================

def get_target_waypoint(vehicle_loc, path_points, current_index, speed_kmh, min_lookahead=5.0, lookahead_ratio=0.4, max_search_ahead=30, fallback_dist=25.0):
    """
    功能:根据当前车速动态伸缩预瞄距离,结合局部滑窗与全局兜底,在给定轨迹中找出适合车速的平滑预瞄点。
    【参数】:

        --- 状态输入 ---
        - vehicle_loc: 车辆当前坐标(carla.Location)
        - path_points: 外部已经清洗好的轨迹列表,支持 carla.Location 或(X,Y) 元组
        - current_index: 上一帧的最近点索引(作为局部搜索起点,O(1)复杂度)
        - speed_kmh: 车辆当前真实车速(用于计算动态预瞄距离)

        --- 预瞄参数(第二步:决定往哪开) --
        - min_lookahead: 最小预瞄距离,防止低速时原地抽搐(默认 5.0m)
        - lookahead_ratio: 车速(m/s)到预瞄距离的放大系数(默认 0.4)

        --- 搜索参数(第一步:决定现在在哪) ---
        - max_search_ahead: 局部搜索最近点的滑窗大小 O(1)(默认 30)
        - fallback_dist: 局部搜索容错距离,偏离此距离自动触发 O(N)全局搜索防丢(默认 25.0m)
    【返回】:
        - target_point: 找出的前方预瞄点(原样返回 path_points 中的对象)
        - closest_index: 车身实际所在的最近点索引(务必将其作为下一帧的 current_index 传入)
    【用法】:
        target_wp, current_idx = get_target_waypoint(ego.get_location(), path, current_idx, speed_kmh=50.0)
        apply_pid_control(ego, pid_lon, pid_lat, target_speed, target_wp)
    """
    if not path_points: 
        return None, current_index
    
    vx, vy = vehicle_loc.x, vehicle_loc.y
    min_dist_sq = float('inf')
    closest_index = current_index

    # 1. 动态预瞄距离计算(速度越快,看的越远)
    speed_ms = speed_kmh / 3.6
    lookahead_dist = max(min_lookahead, speed_ms * lookahead_ratio)

    # 2. 默认局部滑窗搜索,定位车身投影点(假设车辆单调前进, O(1)性能最佳)
    search_end = min(current_index + max_search_ahead, len(path_points))
    for i in range(current_index, search_end):
        p = path_points[i]
        px, py =(p.x, p.y) if isinstance(p, carla.Location) else(p[0], p[1])
        
        dist_sq =(px - vx)**2 +(py - vy)**2
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = i

    # 3. 异常全局定位(如果局部最近点偏离超阈值,说明发生瞬移/错位,触发全局搜索 O(N))
    # 注意:这里 fallback_dist 需要平方,以便与 dist_sq 比较
    if min_dist_sq > fallback_dist ** 2:
        min_dist_sq = float('inf')
        for i, p in enumerate(path_points):
            px, py =(p.x, p.y) if isinstance(p, carla.Location) else(p[0], p[1])
            dist_sq =(px - vx)**2 +(py - vy)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_index = i

    # 4. 沿轨迹前视预瞄(Lookahead)
    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1, p2 = path_points[i], path_points[i + 1]
        p1x, p1y =(p1.x, p1.y) if isinstance(p1, carla.Location) else(p1[0], p1[1])
        p2x, p2y =(p2.x, p2.y) if isinstance(p2, carla.Location) else(p2[0], p2[1])
        
        # 使用 math.hypot 性能更好且代码更简短
        current_dist += math.hypot(p1x - p2x, p1y - p2y)
        target_index = i + 1
        
        # 当累加距离达到了当前车速所需的预瞄距离时,停止寻点
        if current_dist > lookahead_dist:
            break

    return path_points[target_index], closest_index


# ==========================================
# 无轨迹自动巡航
# ==========================================

def get_random_lane_keeping_waypoint(carla_map, vehicle_loc, lookahead_dist=6.0):
    """
    【随机转弯动态车道保持】
    功能:不需要固定轨迹,直接利用地图 API 获取前方车道中心点。
          遇到岔路口时,真正随机选择一条分支,保证 NPC 行为的不可预见性。
    适用:游荡的背景车流(NPC)、不需要精准路线的随机探索场景。

    【参数】:
        - carla_map: world.get_map()
        - vehicle_loc: 车辆当前坐标 ego.get_location()
        - lookahead_dist: 前视距离。速度越快可以设得越大(通常 5~10米)
        
    【返回】:
        - carla.Location 对象(可直接丢给 PID 控制器)

    【用法】:
        target_loc = get_random_lane_keeping_waypoint(world.get_map(), ego.get_location(), 6.0)
        apply_pid_control(ego, pid_lon, pid_lat, 30.0, target_loc)
    """
    #  找到当前车身正下方的合法车道中心点
    current_wp = carla_map.get_waypoint(vehicle_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    #  往前方探索 lookahead_dist 米
    next_wps = current_wp.next(lookahead_dist)
    
    if next_wps:
        # 遇到交叉路口时,随机选择一条路走,增加 NPC 行为多样性
        chosen_wp = random.choice(next_wps)
        return chosen_wp.transform.location
        
    # 如果前方没路了（例如开到了死胡同、断头路）,返回当前点刹停
    return current_wp.transform.location


def get_directional_waypoint(current_wp, distance=5.0, direction='Straight'):
    """
    【单步拓扑寻路(Tick-by-Tick)】
    功能:将原直行、右转函数合并。在单一岔路口根据传入的参数(Left/Right/Straight)自动选择最优分支点。
    用法:每次只做一次判断,适合没有长远路线规划、只在当前时刻想强行转向的游荡背景车。
    参数:
        - direction: 'Straight'(直行), 'Right'(右转), 'Left'(左转)
    """
    next_wps = current_wp.next(distance)
    if not next_wps: return None
    if len(next_wps) == 1: return next_wps[0]

    best_wp = next_wps[0]
    current_yaw = current_wp.transform.rotation.yaw
    current_loc = current_wp.transform.location

    if direction == 'Straight':
        min_diff = float('inf')
        for wp in next_wps:
            # 工业级:使用目标点位置的几何航向角,而不是目标点自身的航向角(易失真)
            geom_yaw = math.degrees(math.atan2(wp.transform.location.y - current_loc.y, 
                                               wp.transform.location.x - current_loc.x))
            diff = abs((geom_yaw - current_yaw + 180) % 360 - 180)
            if diff < min_diff: min_diff, best_wp = diff, wp

    elif direction == 'Right':
        max_diff = -float('inf')
        for wp in next_wps:
            geom_yaw = math.degrees(math.atan2(wp.transform.location.y - current_loc.y, 
                                               wp.transform.location.x - current_loc.x))
            # diff 为正代表右转,取最大正值
            diff =(geom_yaw - current_yaw + 180) % 360 - 180
            if diff > max_diff: max_diff, best_wp = diff, wp

    elif direction == 'Left':
        min_diff = float('inf')  # 寻找最小负数(即向左偏离最大)
        for wp in next_wps:
            geom_yaw = math.degrees(math.atan2(wp.transform.location.y - current_loc.y, 
                                               wp.transform.location.x - current_loc.x))
            # diff 为负代表左转,取最小负值
            diff =(geom_yaw - current_yaw + 180) % 360 - 180
            if diff < min_diff: min_diff, best_wp = diff, wp

    return best_wp


# ==========================================
# 调试与可视化辅助工具 (Debug & Visualization)
# ==========================================


def draw_preset_trajectory(world, path_points, z_offset=0.5, color=carla.Color(150, 150, 150), size=0.1, life_time=0.0):
    """
    【预设轨迹静态可视化 (地表吸附修复版)】
    功能: 在仿真世界中画出车辆的全局预设轨迹锚点。
    修复: 直接向 Carla 地图查询真实地表 Z 轴海拔，完美解决缺少 Z 轴或误把 Yaw 当 Z 轴导致的“天上/地下”不可见问题。
    """
    if not path_points: return

    carla_map = world.get_map()  # 获取地图实例

    for p in path_points:
        # 无论数据是 (X,Y) 还是 (X,Y,Yaw) 还是 Location，统统只取 X 和 Y
        x, y = (p.x, p.y) if isinstance(p, carla.Location) else (p[0], p[1])

        # 核心逻辑：向地图查询 (x,y) 对应的正下方路面的真实海拔 Z
        wp = carla_map.get_waypoint(carla.Location(x=x, y=y, z=0.0), project_to_road=True)
        real_z = wp.transform.location.z if wp else 0.0

        # 加上偏移量(防止和柏油路面重叠闪烁)
        loc = carla.Location(x=x, y=y, z=real_z + z_offset)
        world.debug.draw_point(loc, size=size, color=color, life_time=life_time)


def draw_lookahead_point(world, vehicle_loc, target_point, z_offset=0.5, color=carla.Color(0, 255, 0), size=0.12,
                         life_time=0.1):
    """
    【动态预瞄点与牵引线可视化 (车身平齐版)】
    功能: 实时画出绿色的预瞄点和牵引线。
    修复: 强制预瞄点的 Z 轴高度与车身当前高度一致，确保拉出来的牵引线是平行的，不会钻地。
    """
    if not target_point or not vehicle_loc: return

    tx, ty = (target_point.x, target_point.y) if isinstance(target_point, carla.Location) else (target_point[0],
                                                                                                target_point[1])

    # 直接盗用汽车当前的 Z 轴高度，保证牵引线水平
    tz = vehicle_loc.z

    t_loc = carla.Location(x=tx, y=ty, z=tz + z_offset)
    v_loc = carla.Location(x=vehicle_loc.x, y=vehicle_loc.y, z=vehicle_loc.z + z_offset)

    # 稍微加粗了一点(size=0.12)，看起来更显眼
    world.debug.draw_point(t_loc, size=size, color=color, life_time=life_time)
    world.debug.draw_line(v_loc, t_loc, thickness=0.05, color=color, life_time=life_time)

# ==========================================
# 实体生成与配置工具(Actor Spawning & Setup)
# ==========================================

def force_vehicle_stop(vehicle):
    """
    【车辆强制驻车防溜车】
    用途:用于充当路障、事故车的静态车辆,一键拉起手刹,防止在斜坡上溜车滑动。
    【用法】 force_vehicle_stop(parked_truck)
    """
    if vehicle and vehicle.is_alive:
        vehicle.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0, steer=0.0, throttle=0.0))

def control_vehicle_door(vehicle, door_enum=carla.VehicleDoor.FL, is_open=True):
    """
    【长尾场景:开门杀(Dooring) 触发器】
    可独立控制每一扇车门,常用于停在路边的汽车突然开门。
    注意:为了能够播放动画,车辆绝对不能关闭物理模拟(simulate_physics),应使用 force_vehicle_stop 驻车。
    
    【枚举】:carla.VehicleDoor.FL(左前), FR(右前), RL(左后), RR(右后), Hood(引擎盖), Trunk(后备箱)
    【用法】:control_vehicle_door(parked_car, carla.VehicleDoor.FL, is_open=True)
    """
    if not vehicle or not vehicle.is_alive: return
    try:
        if is_open:
            vehicle.open_door(door_enum)
        else:
            vehicle.close_door(door_enum)
    except Exception as e:
        print(f"[RoadTailBench 警告] 车门操作失败,模型可能不支持开门动画: {e}")

def spawn_physics_prop(world, bp_lib, mesh_path, location, mass=50.0, scale=1.0):
    """
    【通用物理道具生成器(如足球、落石、纸箱)】
    用于生成具有真实质量和碰撞体积的道具。
    【用法】:
        football = spawn_physics_prop(world, bp_lib, '...soccerball...', spawn_loc, mass=10.0)
    """
    bp_prop = bp_lib.find('static.prop.mesh')
    bp_prop.set_attribute('mesh_path', mesh_path)
    bp_prop.set_attribute('mass', str(mass))
    bp_prop.set_attribute('scale', str(scale))
    
    transform = carla.Transform(location) if isinstance(location, carla.Location) else location
    prop = world.try_spawn_actor(bp_prop, transform)
    
    if prop:
        prop.set_simulate_physics(True)
        # 增加阻尼让物品不要无休止地滚出地图
        try:
            prop.set_linear_damping(1.5)
            prop.set_angular_damping(1.5)
        except: pass
        print(f"[RoadTailBench] ✅ 生成物理道具成功(Mass: {mass}kg)")
    return prop

def apply_directional_impulse(actor, magnitude, direction_vector=None, yaw_deg=None):
    """
    【物理冲量施加器】
    制造踢足球、追尾撞击、甚至车体侧翻的瞬间力。
    【参数】:支持直接传入方向向量,或者传入基于世界坐标系的偏航角(yaw_deg)。
    【用法】:apply_directional_impulse(football, magnitude=350.0, yaw_deg=child.get_transform().rotation.yaw)
    """
    if not actor or not actor.is_alive: return
    
    if direction_vector is None and yaw_deg is not None:
        rad = math.radians(yaw_deg)
        direction_vector = carla.Vector3D(math.cos(rad), math.sin(rad), 0.0)
    
    if direction_vector:
        # 归一化后乘以力度
        norm = math.hypot(direction_vector.x, direction_vector.y)
        if norm > 0:
            nx = direction_vector.x / norm
            ny = direction_vector.y / norm
            actor.add_impulse(carla.Vector3D(nx * magnitude, ny * magnitude, 0.0))

def hide_map_objects(world, target_names):
    """
    【地图原生静态物体隐藏器】
    用途:用于将官方地图自带的“死模型”(如垃圾袋、路障)剔除,方便你在原地生成具有真实物理特效的替代品。
    【用法】:
        hidden_ids = hide_map_objects(world, {"SM_TrasdhBag_Opt", "SM_TrasdhBag_Opt4"})
    要在场景结束时恢复,调用 world.enable_environment_objects(hidden_ids, True)
    """
    env_objects = world.get_environment_objects(carla.CityObjectLabel.Any)
    objects_to_hide = [obj.id for obj in env_objects if obj.name in target_names]
    
    if objects_to_hide:
        world.enable_environment_objects(objects_to_hide, False)
        print(f"[RoadTailBench 环境] ✅ 成功隐藏了 {len(objects_to_hide)} 个指定的静态地图原生模型。")
    return objects_to_hide


def spawn_vehicle(world, bp_name, x, y, z=None, yaw=None, color=None, role_name="background", z_offset=0.5):
    """
    【工业级·全地形车辆安全生成器】
    融合了二维吸附、三维防重叠、以及防初始画龙的最佳实践。
    
    【核心优化】:
    1. 立交桥支持:允许传入近似的 Z 值。如果该(x,y) 存在上下两层路(立交桥),传入 z=50 会精准吸附到桥上,不传则默认吸附到底层。
    2. 绝对海拔自适应:利用 waypoint 的真实 Z 轴加上 z_offset,无论是在 0米平原还是 100米高山,都能完美贴合地表。
    3. 防画龙机制:如果传入了轨迹数据集的 yaw,则强制使用；如果不传,则自动继承车道线的完美朝向。
    
    【用法范例】:
        # 1. 简单生成背景车(只知道 XY)
        v1 = spawn_vehicle(world, 'vehicle.audi.tt', 10.5, 20.0, color='255,0,0')
        
        # 2. 从长尾轨迹数据集严格生成(自带 XYZ 和 Yaw)
        ego = spawn_vehicle(world, 'vehicle.tesla.model3', 
                             x=TRAJ[0][0], y=TRAJ[0][1], z=TRAJ[0][2], yaw=TRAJ[0][3], 
                             role_name='ego')
                                 
        # 3. 生成大卡车(需要抬高 1.5 米防底盘卡地爆炸)
        truck = spawn_vehicle(world, 'vehicle.carlamotors.firetruck', x, y, z_offset=1.5)
    """
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find(bp_name)
    
    # 属性配置
    if role_name and bp.has_attribute('role_name'): 
        bp.set_attribute('role_name', role_name)
    if color and bp.has_attribute('color'): 
        bp.set_attribute('color', color)

    carla_map = world.get_map()
    
    # 1. 解决立交桥/高山投影问题:如果有粗略的Z,用它去搜；没有就默认0.0
    search_z = z if z is not None else 0.0
    search_loc = carla.Location(x=x, y=y, z=search_z)
    
    # 让 CARLA 寻找在垂直方向上距离 search_loc 最近的合法行车道
    waypoint = carla_map.get_waypoint(search_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

    if not waypoint:
        print(f"[RoadTailBench] ❌ 警告: 无法在({x:.1f}, {y:.1f}) 周围找到合法车道,生成失败！")
        return None

    # 2. 获取路面绝对真实海拔,并加上安全偏移量防止穿模
    # 普通轿车 z_offset=0.5 足够；SUV/卡车 建议 z_offset=1.0 ~ 1.5
    spawn_z = waypoint.transform.location.z + z_offset

    # 3. 初始航向角判定(防画龙)
    # 如果数据集给了强制朝向(yaw),就用数据集的；否则顺着车道的默认方向生成
    spawn_yaw = yaw if yaw is not None else waypoint.transform.rotation.yaw

    # 组装最终的安全生成坐标
    spawn_transform = carla.Transform(
        carla.Location(x=waypoint.transform.location.x, y=waypoint.transform.location.y, z=spawn_z),
        carla.Rotation(yaw=spawn_yaw)
    )

    # 4. 执行生成与碰撞拦截
    actor = world.try_spawn_actor(bp, spawn_transform)
    if actor:
        print(f"[RoadTailBench] ✅ 成功生成 [{bp_name.split('.')[-1]}] | 坐标:({x:.1f}, {y:.1f}, 高程:{spawn_z:.1f})")
    else:
        print(f"[RoadTailBench] ❌ 生成失败! 位置({x:.1f}, {y:.1f}) 可能已被其他车辆占据, 或 z_offset={z_offset} 过小导致物理穿模卡死。")
        
    return actor


def spawn_friction_region(world, bp_lib, center_loc, friction=0.0, extent=(10.0, 10.0, 10.0), draw_debug=False, debug_life=100.0):
    """
    【物理级低摩擦力打滑区生成器】
    用途:用于长尾场景制造“水坑”、“冰面”或“漏油区”。当车辆驶入此区域,轮胎物理抓地力会暴降,引发剧烈甩尾。
    多Actor:只要区域生成,任何碾过它的 Actor 都会被打滑物理接管。
    
    【用法】:
        friction_trigger = spawn_friction_region(world, bp_lib, carla.Location(x=65.2, y=-28.5, z=-5.0), 
                                                 friction=0.1, extent=(10.0, 10.0, 10.0), draw_debug=True)
        # 最后别忘清理: actor_list.append(friction_trigger)
    """
    bp_friction = bp_lib.find('static.trigger.friction')
    bp_friction.set_attribute('friction', str(friction))
    bp_friction.set_attribute('extent_x', str(extent[0]))
    bp_friction.set_attribute('extent_y', str(extent[1]))
    bp_friction.set_attribute('extent_z', str(extent[2]))

    spawn_tf = center_loc if isinstance(center_loc, carla.Transform) else carla.Transform(center_loc)
    friction_trigger = world.try_spawn_actor(bp_friction, spawn_tf)
    
    if friction_trigger and draw_debug:
        box_extent = carla.Vector3D(x=extent[0], y=extent[1], z=extent[2])
        bbox = carla.BoundingBox(spawn_tf.location, box_extent)
        world.debug.draw_box(box=bbox, rotation=spawn_tf.rotation, thickness=0.2, color=carla.Color(255, 0, 0), life_time=debug_life)
        print(f"[RoadTailBench] ✅ 生成摩擦力突变区成功(Friction: {friction})")
        
    return friction_trigger


# ==========================================
# 4. 场景感知与避障辅助工具(Perception & Obstacle Utils)
# ==========================================

def check_obstacle_in_front(check_loc, check_fwd, actor_list, ego_id, safe_distance=15.0, fov_degrees=60.0):
    """
    【轻量化前向 FOV 雷达避障检测】
    无需挂载 Carla Sensor 浪费算力,利用向量夹角快速计算前方扇形区域内是否有障碍车辆。
    
    【参数】
    - check_loc: 探测发出的中心坐标(通常为 ego.get_location())
    - check_fwd: 探测的前向向量(通常为 ego.get_transform().get_forward_vector())
    - actor_list: 你自己维护的存活车辆列表或 world.get_actors().filter('vehicle.*')
    - safe_distance: 探测距离（米）
    - fov_degrees: 视场角,默认60度（即前方左30度到右30度）
    
    【用法】
    has_obs, obs_vehicle = check_obstacle_in_front(ego.get_location(), ego.get_transform().get_forward_vector(), actors, ego.id)
    if has_obs: ego.apply_control(brake=1.0)
    """
    fov_cos = math.cos(math.radians(fov_degrees / 2.0))
    for actor in actor_list:
        if actor.id == ego_id or not isinstance(actor, carla.Vehicle) or not actor.is_alive:
            continue
        act_loc = actor.get_location()
        dist = check_loc.distance(act_loc)

        if dist < safe_distance and dist > 0.1:
            dir_vec = act_loc - check_loc
            dir_vec_norm = carla.Vector3D(dir_vec.x/dist, dir_vec.y/dist, dir_vec.z/dist)
            # 计算车辆前向向量与障碍物连线向量的点积
            dot = check_fwd.x * dir_vec_norm.x + check_fwd.y * dir_vec_norm.y
            if dot > fov_cos:  
                return True, actor
    return False, None



# ==========================================
# 5. 车辆物理与控制工具(Vehicle Control & PID)
# ==========================================


PID_PRESETS = {
    'default_car': {'K_P_lon': 1.0, 'K_I_lon': 0.05, 'K_D_lon': 0.0, 'K_P_lat': 1.95, 'K_I_lat': 0.05, 'K_D_lat': 0.2},
    'truck':       {'K_P_lon': 1.5, 'K_I_lon': 0.05, 'K_D_lon': 0.1, 'K_P_lat': 1.20, 'K_I_lat': 0.02, 'K_D_lat': 0.4},
    'motorcycle':  {'K_P_lon': 0.8, 'K_I_lon': 0.02, 'K_D_lon': 0.0, 'K_P_lat': 2.50, 'K_I_lat': 0.05, 'K_D_lat': 0.1},
    'wet_road':    {'K_P_lon': 0.7, 'K_I_lon': 0.01, 'K_D_lon': 0.0, 'K_P_lat': 1.00, 'K_I_lat': 0.01, 'K_D_lat': 0.5}
}

class PIDLongitudinalController:
    """
    【标准纵向 PID 控制器】 - 负责控制油门(Throttle) 与刹车(Brake)
    工业级抗滑与防饱和设计:
    - output_clip: 限制最终输出的油门刹车范围。例如雨天设为(-0.8, 0.6),最大油门被锁死在60%,防止起步打滑。
    - i_clip: 积分抗饱和(Anti-Windup)。防止车辆被卡住时积分器无限累加,导致障碍物消失后车辆发生火箭弹射。
    """
    def __init__(self, K_P=None, K_I=None, K_D=None, dt=0.05, preset='default_car', output_clip=(-1.0, 1.0), i_clip=(-2.0, 2.0)):
        if K_P is None:
            K_P, K_I, K_D = PID_PRESETS[preset]['K_P_lon'], PID_PRESETS[preset]['K_I_lon'], PID_PRESETS[preset]['K_D_lon']
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt

        # 性能优化:不再每次 sum 列表,而是维护一个运行时的积分累加器,时间复杂度 O(1)
        self._error_integral = 0.0
        self._previous_error = 0.0

        self.output_clip = output_clip
        self.i_clip = i_clip

    def run_step(self, target_speed, current_speed):
        # 1. 比例项 P(当前误差):决定加速的粗略力度
        error = target_speed - current_speed

        # 2. 微分项 D(误差变化率):提供阻尼,防止车速到达目标时刹不住导致超调(Overshoot)
        _de =(error - self._previous_error) / self._dt
        self._previous_error = error

        # 3. 积分项 I(历史误差累积):专门消除稳态误差。例如上坡时P控制不够力,车速卡在58达不到60,I会不断累加帮你踩死油门
        self._error_integral += error * self._dt
        # 【关键抗饱和】直接在累加器上裁剪,防止历史包袱过重
        if self.i_clip:
            self._error_integral = np.clip(self._error_integral, self.i_clip[0], self.i_clip[1])

        # 4. 汇总输出并裁剪物理极限(限幅)
        output =(self._k_p * error) +(self._k_d * _de) +(self._k_i * self._error_integral)
        return np.clip(output, self.output_clip[0], self.output_clip[1])


class PIDLateralController:
    """
    【横向 PID 控制器(防画龙终极重构版)】 - 负责控制方向盘转向(Steering)
    核心亮点:
    弃用老旧的 Vector Dot/Cross 计算法,改用纯几何数学 atan2 计算两点绝对航向角。
    完美解决 CARLA 中由于 Waypoint 插值导致的弯道方向盘微小抖动与画龙现象。
    """
    def __init__(self, K_P=None, K_I=None, K_D=None, dt=0.05, preset='default_car', output_clip=(-0.8, 0.8), i_clip=(-2.0, 2.0)):
        if K_P is None:
            K_P, K_I, K_D = PID_PRESETS[preset]['K_P_lat'], PID_PRESETS[preset]['K_I_lat'], PID_PRESETS[preset]['K_D_lat']
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt

        # 性能优化:同样采用 O(1) 的状态保存机制
        self._error_integral = 0.0
        self._previous_error = 0.0

        # 输出剪裁默认设为 -0.8 到 0.8,防止方向盘瞬间打满死区导致侧翻
        self.output_clip = output_clip
        self.i_clip = i_clip

    def run_step(self, waypoint, vehicle_transform):
        # 提取车辆的三维坐标与航向角(弧度制)
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)

        # 多态兼容:无论传入的是 CARLA对象 还是 外部元组,都能提取 XY
        wp_x, wp_y =(waypoint.x, waypoint.y) if isinstance(waypoint, carla.Location) else(waypoint[0], waypoint[1])

        # 【降维打击】:这里数组只有两个元素,彻底抛弃了 Z 轴,拍扁成上帝俯视视角
        target_vector = np.array([wp_x - v_loc.x, wp_y - v_loc.y])

        # 如果距离目标点太近(小于0.1米),直接回正方向盘,防止除以 0 以及原地抽搐
        norm = np.linalg.norm(target_vector)
        if norm < 0.1: return 0.0

        # 计算绝对几何目标航向角(目标点相对于当前车的完美朝向)
        target_yaw = math.atan2(target_vector[1], target_vector[0])

        # 初始误差:目标朝向减去当前车头朝向
        error = target_yaw - v_yaw

        # 【环形空间跳变优化】:
        # 车辆在 -179度 到 179度 之间徘徊时,直接相减会导致 358度的诡异大回转。
        # 利用取模运算将误差瞬间锁定在 [-pi, pi] 的最短路径,取代低效的 while 循环
        error =(error + math.pi) %(2.0 * math.pi) - math.pi

        # 微分计算:当前角度误差 - 上一帧角度误差
        _de =(error - self._previous_error) / self._dt
        self._previous_error = error

        # 积分计算:累加转角误差,抵抗长缓弯道的持续离心力偏移
        self._error_integral += error * self._dt
        if self.i_clip:
            self._error_integral = np.clip(self._error_integral, self.i_clip[0], self.i_clip[1])

        # 汇总并输出方向盘控制量(-1.0 左满舵,1.0 右满舵)
        output =(self._k_p * error) +(self._k_d * _de) +(self._k_i * self._error_integral)
        return np.clip(output, self.output_clip[0], self.output_clip[1])


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed_kmh, target_wp):
    """【统一PID控制执行】自动处理坐标获取速度,并带有转向死区过滤微小抖动指令"""
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    current_speed_kmh = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    # 将 kmh 转为 ms 给纵向 PID(根据你最新代码的习惯,统一在外部传入 kmh,内部转换为 m/s 控制)
    throttle_output = pid_lon.run_step(target_speed_kmh / 3.6, current_speed_kmh / 3.6)
    steer_output = pid_lat.run_step(target_wp, tf)

    control = carla.VehicleControl()
    # 【死区拦截】抑制微小扰动
    control.steer = steer_output if abs(steer_output) >= 0.02 else 0.0

    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)

    vehicle.apply_control(control)


# ==========================================
# 车辆平滑控制工具 (SmoothDrive PID)
# ==========================================

# 【参数大洗牌】：大幅降低了横向 P，提高了横向 D，调整了积分器参数
SMOOTH_DYNAMICS_PRESETS = {
    # 默认车辆：P降至0.75（防猛打方向），D升至0.4（增强方向盘回正阻尼）
    'default_car': {'K_P_lon': 1.0, 'K_I_lon': 0.05, 'K_D_lon': 0.1,
                    'K_P_lat': 0.75, 'K_I_lat': 0.10, 'K_D_lat': 0.4},

    'truck': {'K_P_lon': 1.5, 'K_I_lon': 0.05, 'K_D_lon': 0.1,
              'K_P_lat': 0.50, 'K_I_lat': 0.05, 'K_D_lat': 0.5},

    'motorcycle': {'K_P_lon': 0.8, 'K_I_lon': 0.02, 'K_D_lon': 0.0,
                   'K_P_lat': 1.20, 'K_I_lat': 0.05, 'K_D_lat': 0.3},

    'wet_road': {'K_P_lon': 0.7, 'K_I_lon': 0.01, 'K_D_lon': 0.0,
                 'K_P_lat': 0.40, 'K_I_lat': 0.02, 'K_D_lat': 0.6}
}


class SmoothLongitudinalPID:
    """
    纵向控制器：加入漏积分（Leaky Integrator）防止长时间堵车后的油门爆发
    """

    def __init__(self, K_P=None, K_I=None, K_D=None, dt=0.05, preset='default_car', output_clip=(-1.0, 1.0)):
        if K_P is None:
            K_P, K_I, K_D = SMOOTH_DYNAMICS_PRESETS[preset]['K_P_lon'], SMOOTH_DYNAMICS_PRESETS[preset]['K_I_lon'], \
            SMOOTH_DYNAMICS_PRESETS[preset]['K_D_lon']
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt

        self._error_integral = 0.0
        self._previous_error = 0.0
        self.output_clip = output_clip

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        _de = (error - self._previous_error) / self._dt
        self._previous_error = error

        # 【漏积分机制】：每帧遗忘 2% 的历史误差，防止积分爆表
        self._error_integral = self._error_integral * 0.98 + error * self._dt

        output = (self._k_p * error) + (self._k_d * _de) + (self._k_i * self._error_integral)
        return np.clip(output, self.output_clip[0], self.output_clip[1])


class SmoothLateralPID:
    """
    横向控制器（抗画龙终极版）：
    1. 引入了 D项低通滤波（Low-pass filter），防止 Waypoint 插值导致的剧烈抖动
    2. 使用漏积分（Leaky Integrator）代替强制限幅
    """

    def __init__(self, K_P=None, K_I=None, K_D=None, dt=0.05, preset='default_car', output_clip=(-0.8, 0.8)):
        if K_P is None:
            K_P, K_I, K_D = SMOOTH_DYNAMICS_PRESETS[preset]['K_P_lat'], SMOOTH_DYNAMICS_PRESETS[preset]['K_I_lat'], \
            SMOOTH_DYNAMICS_PRESETS[preset]['K_D_lat']
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt

        self._error_integral = 0.0
        self._previous_error = 0.0
        self._filtered_de = 0.0  # D项低通滤波器状态

        self.output_clip = output_clip

    def run_step(self, lookahead_waypoint, vehicle_transform):
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)

        wp_x, wp_y = (lookahead_waypoint.x, lookahead_waypoint.y) if isinstance(lookahead_waypoint,
                                                                                carla.Location) else (
            lookahead_waypoint[0], lookahead_waypoint[1])
        target_vector = np.array([wp_x - v_loc.x, wp_y - v_loc.y])

        norm = np.linalg.norm(target_vector)
        if norm < 0.1:
            return 0.0

        target_yaw = math.atan2(target_vector[1], target_vector[0])
        error = target_yaw - v_yaw

        # 环形空间跳变优化（保留了你优秀的原始逻辑）
        error = (error + math.pi) % (2.0 * math.pi) - math.pi

        # 【核心优化 1：D项低通滤波】
        # 原始的 (error - prev_error) / dt 在遇到路径点切换时会产生巨大的毛刺
        # 使用指数移动平均(EMA)平滑微分项，让方向盘回正更加柔和
        raw_de = (error - self._previous_error) / self._dt
        self._filtered_de = (0.7 * self._filtered_de) + (0.3 * raw_de)
        self._previous_error = error

        # 【核心优化 2：漏积分(Leaky Integrator)】
        # 抛弃 clip，改用衰减率(0.95)。每次转弯后，积分项会迅速自然归零，不会带着历史包袱冲向反方向
        self._error_integral = self._error_integral * 0.95 + error * self._dt

        # 使用滤波后的 D 项计算输出
        output = (self._k_p * error) + (self._k_d * self._filtered_de) + (self._k_i * self._error_integral)
        return np.clip(output, self.output_clip[0], self.output_clip[1])


def smooth_pid_control(vehicle, pid_lon, pid_lat, target_speed_kmh, lookahead_wp):
    """
    执行车辆控制：去除了致命的死区逻辑
    """
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    current_speed_kmh = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    # 计算输出
    throttle_output = pid_lon.run_step(target_speed_kmh / 3.6, current_speed_kmh / 3.6)
    steer_output = pid_lat.run_step(lookahead_wp, tf)

    control = carla.VehicleControl()

    # 【核心优化 3：废除死区】
    # 去掉了 abs(steer_output) >= 0.02 的判断。让方向盘接受微小调整指令！
    control.steer = steer_output

    # 油门/刹车逻辑保持原样
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)

    vehicle.apply_control(control)

# ==========================================
# 车辆速度初始化
# ==========================================

def set_vehicle_initial_speed(vehicle, target_speed_kmh, yaw_deg=None):
    """
    【一键式车辆初速度赋予器(无类化/零阻塞)】
    功能:在车辆生成时瞬间赋予绝对物理速度,省去PID从零漫长加速的过程。
    
    【工业级防崩/防滑处理】:
        1. 零阻塞时间线:纯函数执行,绝不调用 world.tick(),完美兼容外部自动驾驶算法的同步流。
        2. 彻底剔除Z轴:只赋予XY平面速度,Z轴速度强行锁死为 0.0,把下落交还给重力,杜绝“起步钻地”或“发射升空”。
        3. 动量扭矩清零:重置角速度,防止落地瞬间发生诡异的死亡翻滚。
        4. 引擎瞬间唤醒(Anti-Slip):瞬间打满油门指令,强迫 PhysX 引擎在下一帧立刻提升轮胎 RPM,完美化解“车速100、轮速0”导致的严重硬摩擦打滑。

    【用法】:
        npc = spawn_vehicle(world, 'vehicle.audi.tt', x, y)
        set_vehicle_initial_speed(npc, target_speed_kmh=50.0)
    """
    if not vehicle or not vehicle.is_alive: 
        return
        
    # 1. 速度解算
    speed_ms = target_speed_kmh / 3.6
    yaw = yaw_deg if yaw_deg is not None else vehicle.get_transform().rotation.yaw
    yaw_rad = math.radians(yaw)
    
    vx = speed_ms * math.cos(yaw_rad)
    vy = speed_ms * math.sin(yaw_rad)
    
    # 2. 动量注入与姿态锁定
    # 给予平面速度,严格锁死Z轴
    vehicle.set_target_velocity(carla.Vector3D(x=vx, y=vy, z=0.0))
    # 强制清零角速度,防止因为生成点微小的倾斜导致落地侧翻
    vehicle.set_target_angular_velocity(carla.Vector3D(x=0.0, y=0.0, z=0.0))
    
    # 3. 物理防抱死/防滑预热技巧(Throttle Injection)
    # 给出一脚满油门指令,就算下一帧你的 PID 控制器立刻接管了,
    # 这一脚油门也足以让物理引擎瞬间判定“引擎已启动”,消除死锁打滑。
    control = carla.VehicleControl()
    control.throttle = 1.0
    control.brake = 0.0
    control.steer = 0.0
    control.hand_brake = False
    vehicle.apply_control(control)
    
    print(f"[RoadTailBench] 🚀 已为车辆 [{vehicle.id}] 瞬间注入 {target_speed_kmh}km/h 稳定初速度。")

# ==========================================
# 4. 车辆物理控制与动态状态机(Control & PID)
# ==========================================

class MultiStageBehaviorMachine:
    """
    【车辆多阶段行为状态机(Sequential Behavior Machine)】
    真正意义上的状态机！用于定义车辆在长尾场景中复杂的“剧本(Scenario)”。
    采用队列(Queue)顺序执行机制,完美解决高架桥、环岛、回头弯导致的“重复触发”问题。

    【支持的触发器类型(trigger_type)】:
        - 'point': 当车辆到达指定的(X, Y, Z) 坐标附近时触发（Z轴可选）。
        - 'x_greater' / 'x_less': 当车辆的 X 坐标大于/小于某个值时触发。
        - 'y_greater' / 'y_less': 当车辆的 Y 坐标大于/小于某个值时触发。
        - 'time': 在进入当前阶段后,等待指定秒数后触发。
        - 'immediate': 无条件立刻触发。

    【用法范例】:
        # 1. 实例化状态机,初始速度 0
        ego_sm = MultiStageBehaviorMachine(initial_speed=0.0)

        # 2. 编排长尾场景剧本(依次执行)
        # 阶段1: 立刻以 15的加速度 提速到 50km/h
        ego_sm.add_stage(trigger_type='immediate', target_speed=50.0, accel=15.0)
        
        # 阶段2: 当车辆跑到 Y < 100 的位置时,以 25 的急刹车减速到 10km/h
        ego_sm.add_stage(trigger_type='y_less', trigger_val=100.0, target_speed=10.0, accel=25.0)
        
        # 阶段3: 维持 10km/h 等待 3 秒钟,然后以极慢的加速度(5.0)提速到 30km/h
        ego_sm.add_stage(trigger_type='time', trigger_val=3.0, target_speed=30.0, accel=5.0)
        
        # 阶段4: 当接近特定坐标(25.5, -40.0) 半径 3米 内时,彻底刹停
        ego_sm.add_stage(trigger_type='point', trigger_val=(25.5, -40.0), target_speed=0.0, accel=20.0, tolerance=3.0)

        # 3. 在主循环中每帧调用
        while True:
            # tick会自己判断是否满足条件,并返回当前帧【被平滑计算过】的目标速度
            current_target_speed = ego_sm.tick(ego.get_location(), current_sim_time, dt)
            apply_pid_control(ego, pid_lon, pid_lat, current_target_speed, target_wp)
    """

    def __init__(self, initial_speed=0.0):
        self.current_speed = initial_speed  # 当前输出的平滑速度
        self.stages = []                    # 剧本队列
        self.current_idx = 0                # 当前执行到的阶段索引
        self.stage_enter_time = None        # 用于记录进入当前阶段的时间（处理 'time' 触发器）

    def add_stage(self, trigger_type, target_speed, trigger_val=None, accel=15.0, tolerance=2.0):
        """
        向状态机添加一个阶段。
        参数:
            - trigger_type: 触发条件类型
            - target_speed: 该阶段需要达到的目标速度(km/h)
            - trigger_val: 触发条件的值（坐标元组、浮点数或时间秒数）
            - accel: 达到目标速度的加速度/减速度限制(km/h per second)
            - tolerance: 仅针对 'point' 触发器,表示到达目标的判定半径(米)
        """
        self.stages.append({
            'type': trigger_type,
            'val': trigger_val,
            'target_speed': target_speed,
            'accel': accel,
            'tolerance': tolerance
        })

    def _check_trigger(self, stage, vehicle_loc, current_time):
        """【内部私有函数】检查是否满足进入下一阶段的条件"""
        t_type = stage['type']
        t_val = stage['val']

        if t_type == 'immediate':
            return True
        elif t_type == 'time':
            # 如果是该阶段的第一帧,记录进入时间
            if self.stage_enter_time is None:
                self.stage_enter_time = current_time
            # 检查时间是否流逝完毕
            if current_time - self.stage_enter_time >= t_val:
                self.stage_enter_time = None # 清空计时器给下一个阶段用
                return True
            return False
        elif t_type == 'point':
            # 兼容(X, Y) 或(X, Y, Z)
            dx = vehicle_loc.x - t_val[0]
            dy = vehicle_loc.y - t_val[1]
            dist = math.hypot(dx, dy)
            return dist <= stage['tolerance']
        elif t_type == 'x_greater':
            return vehicle_loc.x > t_val
        elif t_type == 'x_less':
            return vehicle_loc.x < t_val
        elif t_type == 'y_greater':
            return vehicle_loc.y > t_val
        elif t_type == 'y_less':
            return vehicle_loc.y < t_val
        
        return False

    def tick(self, vehicle_loc, current_time, dt):
        """
        【主循环刷新接口】
        返回车辆当前帧应该设定的平滑速度。
        """
        if not self.stages:
            return self.current_speed

        # 1. 取出当前正在“等待触发”的阶段
        # 如果已经执行到最后一个阶段,就不再越界判断
        if self.current_idx < len(self.stages):
            current_stage = self.stages[self.current_idx]
            
            # 检查条件是否满足,如果满足,推进状态机(阶段+1)
            if self._check_trigger(current_stage, vehicle_loc, current_time):
                print(f"[RoadTailBench 状态机] ✅ 触发阶段 {self.current_idx}: {current_stage['type']} -> 目标速度变更为 {current_stage['target_speed']}km/h")
                self.current_idx += 1
                self.stage_enter_time = None  # 重置时间记录器

        # 2. 获取当前应该执行的动力学参数
        # 如果当前在阶段0等待,那说明还没触发阶段0,维持初始速度或上一阶段速度
        # 如果触发了,那就执行已经触发的最新的阶段参数
        active_idx = max(0, self.current_idx - 1)
        if self.current_idx == 0:
            # 剧本还没开始任何一个阶段,保持当前状态
            target_spd = self.current_speed
            accel_rate = 0.0
        else:
            active_stage = self.stages[active_idx]
            target_spd = active_stage['target_speed']
            accel_rate = active_stage['accel']

        # 3. 平滑加减速执行器(融合了老 DynamicSpeedController 的功能)
        if accel_rate > 0.0:
            if self.current_speed < target_spd:
                self.current_speed = min(target_spd, self.current_speed + accel_rate * dt)
            elif self.current_speed > target_spd:
                self.current_speed = max(target_spd, self.current_speed - accel_rate * dt)

        return self.current_speed


# ==========================================
# 天气控制系统(Weather System)
# ==========================================

# 需要插值的 CARLA 天气物理属性列表
WEATHER_ATTRS = [
    "cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
    "sun_azimuth_angle", "sun_altitude_angle", "fog_density", "fog_distance",
    "fog_falloff", "wetness", "scattering_intensity", "mie_scattering_scale",
    "rayleigh_scattering_scale", "dust_storm"
]

def clamp(value, minimum=0.0, maximum=100.0):
    """【内部辅助函数】限幅器"""
    return max(minimum, min(value, maximum))


# ==========================================
# 函数1:天气参数构建器 (生产状态 A 和 状态 B 的工具)
# ==========================================
def build_weather(base_weather=None, preset=None, **kwargs):
    """
    【功能】组装并返回一个标准化的 carla.WeatherParameters 对象。
    用于在主循环外提前定义好你想转换的「状态A」和「状态B」。
    【用法】循环外定义状态 A 和 状态 B
        # 状态 A (例如:微弱晨光,有点薄雾)
        weather_A = build_weather(preset="ClearNoon", sun_altitude_angle=10.0, fog_density=5.0)

        # 状态 B (例如:恐怖的沙尘暴叠加暴雨积水)
        weather_B = build_weather(dust_storm=100.0, precipitation_deposits=100.0, wind_intensity=100.0, fog_density=80.0)
    """
    if preset and hasattr(carla.WeatherParameters, preset):
        w = getattr(carla.WeatherParameters, preset)
    elif base_weather:
        w = carla.WeatherParameters(**{attr: getattr(base_weather, attr) for attr in WEATHER_ATTRS})
    else:
        w = carla.WeatherParameters()

    for key, val in kwargs.items():
        if hasattr(w, key):
            setattr(w, key, float(val))
    return w


# ==========================================
# 函数2:静态/镜头天气设置 (瞬间切换)
# ==========================================
def set_static_weather(world, preset=None, **kwargs):
    """【功能】一键更改当前世界的静态天气,瞬间生效无渐变。"""
    current_w = world.get_weather()
    new_w = build_weather(base_weather=current_w, preset=preset, **kwargs)
    world.set_weather(new_w)


# ==========================================
# 函数3:全能动态天气演化器 (支持显式 A->B 与 长尾极端场景)
# ==========================================
def tick_dynamic_weather(world, tracker, dt, mode="CustomLerp", duration=10.0, 
                         start_weather=None, end_weather=None, target_preset=None, **kwargs):
    """
    【功能描述】
    作为仿真主循环 (while True) 中的天气推进器。
    支持两大核心机制:
    1. Lerp 机制 (线性插值):在指定 duration(秒) 内,从「状态A」平滑过渡到「状态B」。
    2. Math 机制 (无尽循环):接管 CARLA 官方公式,实现无限的日月交替与雷暴循环。

    【核心参数说明】:
        - world         : CARLA 世界对象 (world = client.get_world())
        - tracker       : 字典追踪器,用于保存当前渐变的进度和状态。首次调用必须传入 None。
        - dt            : 仿真时间步长 (Delta time),通常用 world.wait_for_tick().timestamp.delta_seconds 或固定值如 0.05。
        - mode          : 运行模式,见下方详细列表。
        - duration      : 天气渐变的持续时间（秒）。仅在 Lerp 机制下有效。
        - start_weather : 【状态A】起点天气参数 (carla.WeatherParameters 对象)。若不传,默认抓取调用时的当前世界天气。
        - end_weather   : 【状态B】终点天气参数。若传入,优先级最高,直接向该状态过渡。
        - target_preset : 【状态B】目标预设名称 (如 "HardRainNoon")。若传入,向该预设过渡。
        - **kwargs      : 【状态B】零散的属性修改 (如 fog_density=100)。

    【自动驾驶长尾场景预设 (mode 列表)】:
        [1. 基础渐变类]
        - "CustomLerp"     : 自定义渐变模式。配合 kwargs, target_preset, end_weather 使用。
        
        [2. 视觉/感知失效类长尾场景]
        - "BlindingGlare"  : 逆光致盲 (太阳高度极低 + 强烈散射,复现隧道出口/傍晚阳光直射,导致摄像头大面积过曝)
        - "WhiteoutFog"    : 白盲团雾 (能见度瞬间降至 5 米以内,高浓度且带强光散射,激光雷达点云大量噪点)
        - "Sandstorm"      : 极限沙尘暴 (漫天黄沙,高米氏散射(Mie),彻底遮蔽视觉与激光雷达)
        - "Snowstorm"      : 暴风雪白化 (高云量,极大降水与积水,结合高散射模拟暴雪遮挡,路面标线完全丢失)
        
        [3. 控制/动力学失效类长尾场景]
        - "FlashFlood"     : 暴雨积水 (极大降水 + 100% 地表积水 + 狂风,用于测试轮胎滑水效应与车辆失控)
        - "PitchBlackRain" : 暗夜暴雨 (切断所有自然光源 + 狂风暴雨,测试夜间感知极限与车灯反射干扰)
        - "SunGlareRain"   : 太阳雨反光 (有强光照射同时伴随地表积水,路面产生极强镜面反射,干扰车道线识别)

        [4. 无尽循环类]
        - "OfficialCycle"  : 完美复刻官方的雷暴与日月交替无尽循环系统。

    【================ 全场景详尽用法示例 ================】

    [前期准备]: 
    tracker = None  # 发车前,必须在循环外初始化追踪器

    【用法 1:最简单的局部参数渐变 (从当前天气 -> 逐渐起雾)】
    # 场景:进入了一片湿地,当前天气不变,仅用 15 秒缓慢把雾气加满
    while True:
        world.tick()
        tracker = tick_dynamic_weather(
            world, tracker, dt=0.05, mode="CustomLerp", duration=15.0,
            fog_density=100.0, fog_distance=0.5 
        )

    【用法 2:利用内置长尾场景 (触发突发致盲逆光)】
    # 场景:车辆驶出隧道,要求在短短 3 秒内,环境突变为傍晚逆光致盲状态
    while True:
        world.tick()
        tracker = tick_dynamic_weather(
            world, tracker, dt=0.05, mode="BlindingGlare", duration=3.0
        )

    【用法 3:使用字符串预设作为 状态B (当前 -> 官方预设)】
    # 场景:从当前环境,花 20 秒过渡到 CARLA 自带的狂风暴雨预设
    while True:
        world.tick()
        tracker = tick_dynamic_weather(
            world, tracker, dt=0.05, mode="CustomLerp", duration=20.0,
            target_preset="HardRainNoon"
        )

    【用法 4:极其硬核的完全自定义 A -> B (双显式状态传入)】
    # 场景:严格控制实验变量,不在乎当前什么天气,强行要求从指定的 A 过渡到 B。
    weather_A = build_weather(preset="ClearNoon", fog_density=0.0)
    weather_B = build_weather(preset="ClearNight", fog_density=100.0)
    
    while True:
        world.tick()
        tracker = tick_dynamic_weather(
            world, tracker, dt=0.05, mode="CustomLerp", duration=30.0,
            start_weather=weather_A,   # 显式指定状态 A
            end_weather=weather_B      # 显式指定状态 B
        )

    【用法 5:激活官方的无尽动态天气 (挂机推演)】
    # 场景:需要跑长达几个小时的泛化性测试,让天气自己按物理规律循环
    while True:
        world.tick()
        # 注意:此模式下 duration, start, end 等参数全部无效,受公式接管
        tracker = tick_dynamic_weather(world, tracker, dt=0.05, mode="OfficialCycle")
    """
    
    # ========================================================================
    # 阶段 1: 任务初始化 (仅在 tracker 为 None 的第一帧执行,配置完整的生命周期)
    # ========================================================================
    if tracker is None:
        # 1. 确定真实的起点天气 (状态 A)。如果用户没给,就去世界里拿当前的。
        actual_start = start_weather if start_weather is not None else world.get_weather()
        
        # 2. 建立追踪器字典 (Tracker)。 active 标志位用于判断任务是否存活。
        tracker = {'active': True, 'mode': mode, 'duration': float(duration)}

        # 3. 区分任务类型:是 Math(无尽循环) 还是 Lerp(线性插值渐变)
        if mode in ["OfficialCycle"]:
            tracker['type'] = 'math'
            # 初始化官方公式所需的时间参数
            tracker['sun_t'] = 0.0
            tracker['azimuth'] = actual_start.sun_azimuth_angle
            tracker['storm_t'] = actual_start.precipitation if actual_start.precipitation > 0 else -50.0
            tracker['increasing'] = True
        else:
            tracker['type'] = 'lerp'
            tracker['elapsed'] = 0.0       # 记录已流逝的时间
            tracker['start_w'] = actual_start  # 冻结起点状态
            
            # 4. 构建终点天气 (状态 B) 的决策树 (优先级: end_weather > 长尾预设 > kwargs)
            if end_weather is not None:
                tracker['end_w'] = end_weather
                
            # [长尾场景硬编码配置]:这些参数经过精心调试,能有效阻碍视觉和激光雷达算法
            elif mode == "BlindingGlare":
                # 逆光致盲:压低太阳角,消除云层直射,拉高光散射系数,容易让相机白平衡崩溃
                tracker['end_w'] = build_weather(actual_start, cloudiness=0.0, sun_altitude_angle=5.0, scattering_intensity=10.0, fog_density=2.0)
            elif mode == "WhiteoutFog":
                # 白盲团雾:能见度极低(0.5),伴随极速衰减(falloff=5),激光雷达基本报废
                tracker['end_w'] = build_weather(actual_start, fog_density=100.0, fog_distance=0.5, fog_falloff=5.0)
            elif mode == "FlashFlood":
                # 暴雨积水:测试动力学滑水效应,积水拉满,伴随狂风
                tracker['end_w'] = build_weather(actual_start, cloudiness=100.0, precipitation=100.0, precipitation_deposits=100.0, wetness=100.0, wind_intensity=80.0)
            elif mode == "PitchBlackRain":
                # 暗夜暴雨:太阳沉底,彻底剥夺自然光,暴雨产生雨滴噪点
                tracker['end_w'] = build_weather(actual_start, sun_altitude_angle=-90.0, cloudiness=100.0, precipitation=100.0, wetness=100.0)
            elif mode == "Sandstorm":
                # 极限沙尘:漫天黄沙,使用原生 dust_storm 参数彻底干扰所有光学传感器
                tracker['end_w'] = build_weather(actual_start, preset="DustStorm", dust_storm=100.0, wind_intensity=100.0)
            elif mode == "Snowstorm":
                # 暴风雪:利用高米氏散射(mie)模拟白化,伴随积水模拟冰雪路面
                tracker['end_w'] = build_weather(actual_start, cloudiness=100.0, precipitation=100.0, precipitation_deposits=80.0, mie_scattering_scale=0.1, scattering_intensity=3.0)
            elif mode == "SunGlareRain":
                # 太阳雨反光:太阳高度角中等偏低,强散射光 + 地面积水镜面反射,车道线直接消失
                tracker['end_w'] = build_weather(actual_start, sun_altitude_angle=25.0, cloudiness=20.0, precipitation=50.0, wetness=100.0, scattering_intensity=5.0)
            else:
                # 默认 CustomLerp 模式:利用底层的 build_weather 融和 kwargs 生成状态 B
                tracker['end_w'] = build_weather(actual_start, preset=target_preset, **kwargs)

    # ========================================================================
    # 阶段 2: 帧推演计算 (每帧被 while 循环调用,负责修改天气)
    # ========================================================================
    # 拦截:如果渐变任务已经完成,直接 Return,避免占用 CPU 进行无意义的运算
    if not tracker.get('active', False): 
        return tracker 

    new_w = carla.WeatherParameters()

    if tracker['type'] == 'lerp':
        # ----------------------------------------
        # 【执行 Lerp 机制】: A -> B 线性插值
        # ----------------------------------------
        tracker['elapsed'] += dt
        
        # 计算进度百分比 (例如过了5秒,总长10秒,progress 就是 0.5)
        # 用 min/max 钳制在 0.0 到 1.0 之间,防止过冲
        progress = min(1.0, max(0.0, tracker['elapsed'] / tracker['duration']))
        
        # 遍历所有 CARLA 物理属性,执行数学插值公式:当前值 = 起点值 + (终点值 - 起点值) * 进度
        for attr in WEATHER_ATTRS:
            start_v = getattr(tracker['start_w'], attr)
            end_v = getattr(tracker['end_w'], attr)
            setattr(new_w, attr, start_v + (end_v - start_v) * progress)
            
        # 当进度到达 100% 时,将 active 设为 False 终止该任务
        if progress >= 1.0:
            tracker['active'] = False
            print(f"[环境引擎] ⚠️ 长尾场景渐变已完成,当前驻留在目标状态: {mode}")

    elif tracker['type'] == 'math':
        # ----------------------------------------
        # 【执行 Math 机制】: 官方正弦波振荡公式
        # ----------------------------------------
        new_w = build_weather(world.get_weather()) # 继承底色
        
        # [子系统 1] 太阳交替公式:控制太阳高度(altitude) 和 方位角(azimuth) 绕圆周运动
        tracker['sun_t'] = (tracker['sun_t'] + 0.008 * dt) % (2.0 * math.pi)
        tracker['azimuth'] = (tracker['azimuth'] + 0.25 * dt) % 360.0
        new_w.sun_azimuth_angle = tracker['azimuth']
        new_w.sun_altitude_angle = (70 * math.sin(tracker['sun_t'])) - 20
        
        # [子系统 2] 风暴曲线公式:控制晴朗 -> 暴雨 -> 积水 -> 放晴 的无穷震荡
        delta = (1.3 if tracker['increasing'] else -1.3) * dt
        storm_t = clamp(tracker['storm_t'] + delta, -250.0, 100.0)
        tracker['storm_t'] = storm_t
        
        # 触达边界后,反转变化方向
        if storm_t == -250.0: tracker['increasing'] = True
        if storm_t == 100.0: tracker['increasing'] = False
        
        # 根据核心变量 storm_t 映射出云、雨、风、雾的参数
        new_w.cloudiness = clamp(storm_t + 40.0, 0.0, 90.0)
        new_w.precipitation = clamp(storm_t, 0.0, 80.0)
        new_w.precipitation_deposits = clamp(storm_t + (-10.0 if tracker['increasing'] else 90.0), 0.0, 85.0)
        new_w.wetness = clamp(storm_t * 5.0, 0.0, 100.0)
        new_w.fog_density = clamp(storm_t - 10.0, 0.0, 30.0)
        # 根据云量粗略决定风的强度
        new_w.wind_intensity = 5.0 if new_w.cloudiness <= 20 else (90.0 if new_w.cloudiness >= 70 else 40.0)

    # ========================================================================
    # 阶段 3: 环境应用 (将计算结果应用到物理引擎,并返回追踪器状态)
    # ========================================================================
    world.set_weather(new_w)
    return tracker

# ==========================================
# 6. Traffic Manager 守护/接管机制
# ==========================================

def freeze_all_traffic_lights(world, state=carla.TrafficLightState.Red):
    """
    【全局红绿灯冻结接管】
    将地图内所有红绿灯一键设为指定颜色并彻底冻结,常用于制造路口冲突、闯红灯或静止拥堵场景。
    【用法】freeze_all_traffic_lights(world, carla.TrafficLightState.Red)
    """
    tls = world.get_actors().filter('traffic.traffic_light')
    count = 0
    for tl in tls:
        tl.set_state(state)
        tl.freeze(True)
        count += 1
    print(f"[RoadTailBench] 已强制接管并冻结 {count} 个红绿灯为 {state} 状态。")

def takeover_ego_vehicle(ego_vehicle, tm):
    """
    【退出TM托管接口】
    【自研算法一键接管接口】
    调用此接口后,TM 将不再干预该车辆,控制权完全移交给你自己的 AI 算法模型。
    【用法】takeover_ego_vehicle(ego, tm)
    """
    ego_vehicle.set_autopilot(False, tm.get_port())
    control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0, hand_brake=False)
    ego_vehicle.apply_control(control)
    print(f"[RoadTailBench] 车辆 {ego_vehicle.id} 已退出TM托管,准备接入外部接管控制。")


def enable_tm_autopilot(tm, vehicle, behavior="normal", speed_delta_pct=0.0):
    """【一键配置TM自动驾驶行为】"""
    vehicle.set_autopilot(True, tm.get_port())
    tm.vehicle_percentage_speed_difference(vehicle, speed_delta_pct)
    
    # 彻底禁止TM乱关灯
    tm.ignore_lights_percentage(vehicle, 100.0)
    
    if behavior == "aggressive":
        tm.ignore_lights_percentage(vehicle, 100)
        tm.ignore_signs_percentage(vehicle, 100)
        tm.ignore_vehicles_percentage(vehicle, 100)
        tm.ignore_walkers_percentage(vehicle, 100)
        tm.auto_lane_change(vehicle, False)
    elif behavior == "no_lane_change":
        tm.auto_lane_change(vehicle, False)

def force_tm_route(carla_map, tm, vehicle, end_location, resolution=1.0):
    """【强制TM遵循路线】"""
    try:
        grp = GlobalRoutePlanner(carla_map, sampling_resolution=resolution)
        start_loc = vehicle.get_location()
        route = grp.trace_route(start_loc, end_location)
        
        path_locations = [snapshot[0].transform.location for snapshot in route]
        if path_locations:
            tm.set_path(vehicle, path_locations)
            return True
        return False
    except NameError:
        print("[警告] 无法加载 GlobalRoutePlanner,强制路线失败。请检查 agents 模块。")
        return False
    
# ==========================================
# 7. 长尾场景特效:物理风力与碎片系统
# ==========================================

class VisualOcclusionEffectManager:
    """
    【车辆传感器与挡风玻璃遮挡物理/视觉特效管理器】
    颠覆性重构:使用车辆本地局部坐标矩阵转换,支持**任何朝向**下的车辆遮挡特效。
    不仅可以实现垃圾袋/纸箱迎面飞来,还可以“贴”在摄像头或玻璃上遮挡视野并伴随风力抖动,
    最后条件触发时被风吹飞飘落销毁。完美支持长尾感知阻断测试！
    
    【参数说明】:
        - target_vehicle: 特效绑定的目标受害车辆(通常是Ego)
        - offsets_and_sticky: 一个列表,元素为((相对X, 相对Y, 相对Z), 是否粘滞)。
            X>0表示车头前方。例如(30.0, 0.0, 2.2) 表示车前30米,高2.2米的飞行物。
        - sticky_x: 粘滞拦截距离(相对车辆X坐标)。轿车前置摄像头通常为 2.8 米。
        
    【用法】:
        offsets = [((30.0, 0.0, 2.2), True),((35.0, 1.5, 2.0), False) ]
        occlusion_mgr = VisualOcclusionEffectManager(world, bp_lib, ego_vehicle, offsets)
        
        while True:
            # 每帧调用更新飞行与抖动
            occlusion_mgr.tick(current_sim_time)
            
            # 条件满足解除粘滞
            if passed_the_obstacle: occlusion_mgr.release_all()
    """
    def __init__(self, world, blueprint_lib, target_vehicle, offsets_and_sticky, bp_name='static.prop.shoppingbag', sticky_x=2.8):
        self.world = world
        self.target_vehicle = target_vehicle
        self.active_props = []
        self.props_rel_loc = []
        self.is_sticky = []
        self.sticky_x = sticky_x
        
        bp = blueprint_lib.find(bp_name)
        v_tf = self.target_vehicle.get_transform()
        
        for(ox, oy, oz), sticky in offsets_and_sticky:
            # 局部坐标系
            spawn_loc = carla.Location(x=ox, y=oy, z=oz)
            # 瞬间转换为真实世界绝对坐标系生成,避免物理碰撞穿模
            spawn_tf = carla.Transform(v_tf.transform(spawn_loc), v_tf.rotation)
            prop = self.world.try_spawn_actor(bp, spawn_tf)
            if prop:
                prop.set_collisions(False)
                prop.set_simulate_physics(False) # 必须关物理,交由本代码数学控制
                self.active_props.append(prop)
                self.props_rel_loc.append([ox, oy, oz])
                self.is_sticky.append(sticky)

    def tick(self, current_time):
        """【帧刷新】处理迎面飞行、遮挡抖动与随风脱落状态机"""
        if not self.target_vehicle or not self.target_vehicle.is_alive: return
            
        v_tf = self.target_vehicle.get_transform()
        vel = self.target_vehicle.get_velocity()
        curr_spd_mps = math.hypot(vel.x, vel.y)
        
        for i in range(len(self.active_props)-1, -1, -1):
            prop = self.active_props[i]
            if not prop.is_alive: continue
            
            # 状态1:粘滞状态,且已经砸中了前档/摄像头
            if self.is_sticky[i] and self.props_rel_loc[i][0] <= self.sticky_x:
                self.props_rel_loc[i][0] = self.sticky_x # 锁死相对距离
                shake = 0.005 * math.sin(current_time * 20 + i) # 模拟高速风吹微小高频抖动
                
                local_loc = carla.Location(x=self.props_rel_loc[i][0], 
                                           y=self.props_rel_loc[i][1] + shake, 
                                           z=self.props_rel_loc[i][2] + shake)
                # 利用汽车真实姿态矩阵还原三维坐标
                abs_loc = v_tf.transform(local_loc)
                # 旋转:保持宽面正对车辆视野(相对车辆航向偏转90度)
                abs_rot = carla.Rotation(pitch=v_tf.rotation.pitch, yaw=v_tf.rotation.yaw + 90, roll=v_tf.rotation.roll)
                prop.set_transform(carla.Transform(abs_loc, abs_rot))

            # 状态2:迎面飞来,或解除粘滞后向后飞离
            else:
                # 迎面时的相对速度略小(让遮挡大概率发生),解除粘滞后相对速度暴增(瞬间吹飞)
                m_speed =(curr_spd_mps * 0.05) +(0.3 if self.is_sticky[i] else 0.8)
                self.props_rel_loc[i][0] -= m_speed
                
                local_loc = carla.Location(x=self.props_rel_loc[i][0], y=self.props_rel_loc[i][1], z=self.props_rel_loc[i][2])
                abs_loc = v_tf.transform(local_loc)
                
                # 飞行中的狂乱翻滚特效
                abs_rot = carla.Rotation(pitch=current_time*200, yaw=current_time*150, roll=current_time*120)
                prop.set_transform(carla.Transform(abs_loc, abs_rot))
                
            # 状态3:清理垃圾,越过车身一定距离(相对负值)后销毁
            if not self.is_sticky[i] and self.props_rel_loc[i][0] < -15.0:
                prop.destroy()
                self.active_props.pop(i)
                self.props_rel_loc.pop(i)
                self.is_sticky.pop(i)

    def release_all(self):
        """【解除接口】一键让所有遮挡屏幕的物品随风解散脱落"""
        for i in range(len(self.is_sticky)):
            self.is_sticky[i] = False
            
    def cleanup(self):
        for prop in self.active_props:
            if prop.is_alive: prop.destroy()
        self.active_props.clear()


def apply_gust_of_wind(actors, target_loc=None, direction_vec=None, force_mag=300.0, apply_torque=True):
    """
    【突发阵风物理冲击器】
    用途:制造一瞬间的狂风,把路边的垃圾袋、纸箱、甚至锥桶吹飞砸向主车。
    【参数】:
        - actors: 需要被施加风力的 Actor 列表(必须开启了 set_simulate_physics(True))
        - target_loc: 选填。给一个目标点,风将朝这个点吹。
        - direction_vec: 选填。直接给一个 carla.Vector3D 风向向量。
        - force_mag: 风力强度倍数(根据物品重量调整,通常300-1000)
    【用法】:
        if dist < 25.0:  # 主车靠近时触发
            apply_gust_of_wind(trash_bags, target_loc=ego.get_location(), force_mag=500.0)
    """
    if not actors: return

    for actor in actors:
        if not actor.is_alive: continue
        
        # 计算吹拂方向
        dir_x, dir_y = 0.0, 0.0
        if direction_vec:
            dir_x, dir_y = direction_vec.x, direction_vec.y
        elif target_loc:
            act_loc = actor.get_location()
            dx, dy = target_loc.x - act_loc.x, target_loc.y - act_loc.y
            norm = math.hypot(dx, dy)
            if norm > 0.001:
                dir_x, dir_y = dx/norm, dy/norm

        # 加点微小的随机偏差和向上的升力
        rx = dir_x + random.uniform(-0.1, 0.1)
        ry = dir_y + random.uniform(-0.1, 0.1)
        up_force = random.uniform(10.0, 30.0)

        actor.add_force(carla.Vector3D(x=rx * force_mag, y=ry * force_mag, z=up_force))
        
        # 施加扭矩让其不规则翻滚
        if apply_torque:
            actor.add_torque(carla.Vector3D(
                x=random.uniform(-50, 50), y=random.uniform(-50, 50), z=random.uniform(-50, 50)
            ))

class PhysicalDebrisManager:
    """
    【标准化风力与物理碎片生成系统】
    长尾场景必备武器。利用物理引擎,通过持续施加向量力与扭矩,完美还原被狂风卷起的落叶、漫天飞舞的垃圾或纸箱。
    
    【参数说明】:
        - spawn_point: 碎片生成的起始抛出点。
        - target_point: 碎片最终落脚的地面目标点。
        - mesh_path: 你想吹拂的物品蓝图路径(树叶、纸箱等)。
        - wind_strength: 基础水平风力大小。
        - lift_force: 上升升力。若想模拟下压阵风,将此设为极小值。
    
    【用法】:
        debris_sys = PhysicalDebrisManager(world, bp_lib, spawn_point=A, target_point=B)
        # 触发时
        debris_sys.spawn_debris()
        # 循环中调用(每帧刷新受力)
        debris_sys.tick(sim_time)
        # 最后别忘了清理
        debris_sys.cleanup()
    """
    def __init__(self, world, blueprint_library, spawn_point, target_point, 
                 mesh_path='/Game/Carla/Static/RoadTailModel/Maple__leave_SM_Leaf_21.Maple__leave_SM_Leaf_21',
                 num_debris=150, mass=0.02, scale=5.0, 
                 wind_strength=0.09, lift_force=0.06, flutter_freq=6.0):
        self.world = world
        self.bp_lib = blueprint_library
        self.spawn_point = spawn_point
        self.target_point = target_point
        self.mesh_path = mesh_path
        
        self.num_debris = num_debris
        self.mass = mass # 保持 20克 极轻,被车撞到毫无影响
        self.scale = scale

        # 风力物理核心参数
        self.base_wind_strength = wind_strength  # 加大吹向B点的基础风力
        self.upward_lift_force = lift_force  # 大幅削弱升力(原本是0.16),让重力主导,加速下落
        self.flutter_amplitude = 0.05 # 加大摇摆幅度,掩盖下落变快的事实
        self.flutter_frequency = flutter_freq # 摇摆频率加快,显得风很大
        self.linear_drag_coeff = 0.03  # 降低空气阻尼(原本是0.08),不拦着它往下掉
        self.angular_drag_coeff = 0.002

        self.debris_data = []
        self.has_spawned = False

        # 计算风向向量
        dx = self.target_point.x - self.spawn_point.x
        dy = self.target_point.y - self.spawn_point.y
        dist_xy = math.hypot(dx, dy)
        self.dir_x = dx / dist_xy if dist_xy > 0 else 0.0
        self.dir_y = dy / dist_xy if dist_xy > 0 else 0.0

    def spawn_debris(self):
        """【生成】在生成点周围散开产生物体并启动物理引擎"""
        bp_prop = self.bp_lib.find('static.prop.mesh')
        bp_prop.set_attribute('mesh_path', self.mesh_path)
        bp_prop.set_attribute('mass', str(self.mass))
        bp_prop.set_attribute('scale', str(self.scale))

        spawned_count = 0
        for _ in range(self.num_debris):
            # 在 A 点附近散开生成,防止重叠爆炸
            offset_x = random.uniform(-1.5, 1.5)
            offset_y = random.uniform(-1.5, 1.5)
            offset_z = random.uniform(-1.0, 1.0)
            loc = carla.Location(x=self.spawn_point.x + offset_x, y=self.spawn_point.y + offset_y, z=self.spawn_point.z + offset_z)
            rot = carla.Rotation(pitch=random.uniform(0, 360), yaw=random.uniform(0, 360), roll=random.uniform(0, 360))

            actor = self.world.try_spawn_actor(bp_prop, carla.Transform(loc, rot))
            if actor:
                actor.set_simulate_physics(True)
                # 记录每一个独立物体的时间相位差,让它们的飘动更凌乱自然
                self.debris_data.append({
                    'actor': actor,
                    'phase_x': random.uniform(0, math.pi * 2),
                    'phase_y': random.uniform(0, math.pi * 2),
                    'settled': False
                })
                spawned_count += 1
                
        self.has_spawned = True
        print(f"[RoadTailBench 特效] 成功生成 {spawned_count} 个物理碎片,狂风开始吹拂！")

    def tick(self, sim_time):
        """【受力更新】主循环中调用,维持物品的风阻和扰动受力"""
        if not self.has_spawned: return

        for item in self.debris_data:
            if item['settled']: continue
            actor = item['actor']
            if not actor.is_alive: continue

            # 如果落地了就停止持续施加风力
            loc = actor.get_location()
            if loc.z <= self.target_point.z + 0.3:
                item['settled'] = True
                continue

            vel = actor.get_velocity()
            ang_vel = actor.get_angular_velocity()

            # 计算基础风力推向
            f_x = self.dir_x * self.base_wind_strength
            f_y = self.dir_y * self.base_wind_strength
            f_z = self.upward_lift_force

            # 基于正弦波模拟风的扰动与摆动
            flutter_x = math.sin(sim_time * self.flutter_frequency + item['phase_x']) * self.flutter_amplitude
            flutter_y = math.cos(sim_time * self.flutter_frequency + item['phase_y']) * self.flutter_amplitude

            # 空气阻力
            drag_x, drag_y, drag_z = -self.linear_drag_coeff * vel.x, -self.linear_drag_coeff * vel.y, -self.linear_drag_coeff * vel.z

            total_force = carla.Vector3D(x=f_x + flutter_x + drag_x, y=f_y + flutter_y + drag_y, z=f_z + drag_z)
            actor.add_force(total_force)

            # 随机旋转力矩,让物体翻滚
            torque_x = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.x
            torque_y = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.y
            torque_z = random.uniform(-0.002, 0.002) - self.angular_drag_coeff * ang_vel.z
            actor.add_torque(carla.Vector3D(x=torque_x, y=torque_y, z=torque_z))

    def cleanup(self):
        """【资源释放】场景结束时调用,清空生成的物理模型"""
        for item in self.debris_data:
            if item['actor'].is_alive:
                item['actor'].destroy()
        self.debris_data.clear()


# ==========================================
# 8. 车辆灯光管理系统(Vehicle Light Manager)
# ==========================================

class VehicleLightManager:
    """
    【全能车辆灯光管理系统(位运算叠加 + RPC防拥堵优化)】
    1. 支持 Carla 所有的 11 种灯光独立控制与任意叠加。
    2. 支持根据车辆当前控制状态(刹车、倒车、打方向)自动亮起对应车灯。
    3. 支持动态爆闪(警车、双闪)。
    4. 带有状态缓存机制,仅在灯光真正变化时才发送 RPC 指令,绝对不卡帧。

    【支持的灯光类型(carla.VehicleLightState)】:
        Position(位置灯), LowBeam(近光), HighBeam(远光), Brake(刹车灯), 
        RightBlinker(右转向), LeftBlinker(左转向), Reverse(倒车灯), 
        Fog(雾灯), Interior(内饰灯), Special1(警灯1), Special2(警灯2)

    【用法 1 - 静态随意叠加】:
        lights = VehicleLightManager(ego_vehicle)
        lights.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.Fog) # 叠加开位置灯和雾灯
        lights.turn_off(carla.VehicleLightState.Fog) # 单独关掉雾灯,其他灯不变

    【用法 2 - 傻瓜式动态自动车灯】(根据车辆油门刹车转向自动亮灯):
        # 主循环 while True 中:
        lights.auto_update_from_control()
        
    【用法 3 - 特效爆闪】:
        lights.start_flashing(mode='police')
        # 主循环 while True 中:
        lights.tick(current_sim_time)
    """
    def __init__(self, vehicle):
        self.vehicle = vehicle
        # current_state 是一个整型,存放二进制掩码
        self.current_state = 0 
        self._last_applied_state = None 
        
        self.flash_mode = None

    def _apply_state(self):
        """【内部核心】对比前后状态,确保只在发生改变时才真正调用 Carla RPC API"""
        if self.vehicle and self.vehicle.is_alive:
            if self._last_applied_state != self.current_state:
                self.vehicle.set_light_state(carla.VehicleLightState(self.current_state))
                self._last_applied_state = self.current_state

    # ---------------------------------------------------------
    # 基础控制 API:任意灯光的开、关、切换
    # ---------------------------------------------------------
    def turn_on(self, light_flag):
        """开启指定的灯光（支持使用 | 叠加传入多个灯光）"""
        self.current_state |= light_flag
        self._apply_state()

    def turn_off(self, light_flag):
        """关闭指定的灯光,其他灯光保持不变"""
        # 使用 ~(按位取反) 和 &(按位与) 来精准关闭指定位,无需手写 0b111...0...111
        self.current_state &= ~light_flag
        self._apply_state()

    def set_static_lights(self, low_beam=True, high_beam=False):
        """便捷函数:一键设置常用行车灯"""
        self.flash_mode = None
        self.turn_on(carla.VehicleLightState.Position)
        if low_beam:
            self.turn_on(carla.VehicleLightState.LowBeam)
        if high_beam:
            self.turn_on(carla.VehicleLightState.HighBeam)

    # ---------------------------------------------------------
    # 高级控制 API:根据车辆真实物理状态自动亮灯
    # ---------------------------------------------------------
    def auto_update_from_control(self):
        """
        【傻瓜式车灯联动】
        必须在主循环中调用。自动读取车辆的刹车、倒车、方向盘信息,并亮起对应车灯。
        这与人工驾驶汽车的逻辑完全一致。
        """
        if not self.vehicle or not self.vehicle.is_alive: return
        control = self.vehicle.get_control()

        # 1. 刹车灯逻辑
        if control.brake > 0.1:
            self.current_state |= carla.VehicleLightState.Brake
        else:
            self.current_state &= ~carla.VehicleLightState.Brake

        # 2. 倒车灯逻辑
        if control.reverse:
            self.current_state |= carla.VehicleLightState.Reverse
        else:
            self.current_state &= ~carla.VehicleLightState.Reverse

        # 3. 转向灯逻辑
        if control.steer < -0.1:  # 左转
            self.current_state |= carla.VehicleLightState.LeftBlinker
            self.current_state &= ~carla.VehicleLightState.RightBlinker
        elif control.steer > 0.1: # 右转
            self.current_state |= carla.VehicleLightState.RightBlinker
            self.current_state &= ~carla.VehicleLightState.LeftBlinker
        else:                     # 回正
            self.current_state &= ~carla.VehicleLightState.LeftBlinker
            self.current_state &= ~carla.VehicleLightState.RightBlinker

        self._apply_state()

    # ---------------------------------------------------------
    # 特效控制 API:警车爆闪 / 故障双闪
    # ---------------------------------------------------------
    def start_flashing(self, mode='hazard'):
        """开启频闪。mode = 'hazard'(普通双闪) 或 'police'(警务爆闪)"""
        self.flash_mode = mode

    def stop_flashing(self):
        """停止频闪,并清除转向灯/远光灯遗留状态"""
        self.flash_mode = None
        self.turn_off(carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker | carla.VehicleLightState.HighBeam)

    def tick(self, sim_time):
        """
        【爆闪控制器】
        如果开启了 flash_mode,必须在 while True 主循环中传入当前仿真时间。
        自带缓存对比拦截,每帧调用也不会造成任何网络负担！
        """
        if not self.flash_mode: return 

        if self.flash_mode == 'police':
            # 警车爆闪特效(周期 2.0 秒,三连闪)
            # 同时开启 Special1 和 Special2(警车顶灯)
            self.current_state |=(carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2 | carla.VehicleLightState.Position)
            
            cycle = sim_time % 2.0
            if cycle < 0.2 or(0.4 <= cycle < 0.6) or(0.8 <= cycle < 1.0):
                self.current_state |= carla.VehicleLightState.HighBeam
            else:
                self.current_state &= ~carla.VehicleLightState.HighBeam

        elif self.flash_mode == 'hazard':
            # 普通故障双闪特效(周期 1.0 秒,亮0.5秒灭0.5秒)
            cycle = sim_time % 1.0
            if cycle < 0.5:
                self.current_state |=(carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker)
            else:
                self.current_state &= ~(carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker)

        self._apply_state()


# ==========================================
# 9. 行人标准化控制器(Pedestrian Controller)
# ==========================================

class PedestrianController:
    """
    【次世代行人中枢系统(融合版)】
    功能:接管行人的移动、循迹、漫游、以及物理越障防卡死逻辑。
    
    【模式1 - 随机漫游(Roam)】:
        在给定的几个点内随机闲逛。到达一个点后,等待指定时间,再前往下一个点。
        用法:
            ped_ctrl = PedestrianController(walker, mode='roam', target_list=ROAM_POINTS, 
                                            roam_wait_time=3.0, default_speed=1.5)
            # 主循环中:
            ped_ctrl.run_step(dt, current_time)
            
    【模式2 - 严格循迹与剧本控制(Trajectory + State Machine)】:
        完全按照给定的清洗后的轨迹列表行走,并且可以复用车辆的【多阶段状态机】来实现“红绿灯等待”、“突然冲刺”等剧本。
        用法:
            # 1. 实例化控制器
            ped_ctrl = PedestrianController(walker, mode='trajectory', target_list=CLEANED_TRAJ)
            # 2. 复用车辆状态机定义剧本(注意:将 accel 设为极大的值 100.0,模拟行人瞬间启停)
            ped_sm = MultiStageBehaviorMachine(initial_speed=0.0)
            ped_sm.add_stage('time', trigger_val=2.0, target_speed=2.5, accel=100.0) # 等2秒后以2.5m/s奔跑
            
            # 3. 主循环中联动:
            speed = ped_sm.tick(walker.get_location(), current_time, dt)
            ped_ctrl.run_step(dt, current_time, dynamic_speed=speed)
    """
    def __init__(self, walker, mode='roam', target_list=None, default_speed=1.5, roam_wait_time=2.0):
        self.walker = walker
        self.mode = mode # 'roam' 或 'trajectory'
        self.target_list = target_list if target_list else []
        self.default_speed = default_speed
        
        # 漫游模式专用参数
        self.roam_wait_time = roam_wait_time
        self.is_waiting = False
        self.wait_start_time = 0.0
        
        # 内部寻路状态
        self.arrival_dist = 0.6
        if self.mode == 'roam' and self.target_list:
            self.current_target = random.choice(self.target_list)
        elif self.mode == 'trajectory' and self.target_list:
            self.traj_index = 0
            self.current_target = self.target_list[self.traj_index]
            
        # 物理防卡死跨栏状态
        self.stuck_timer = 0.0
        self.last_loc = None

    def run_step(self, dt, current_time, dynamic_speed=None):
        """
        主循环帧刷新。
        - dynamic_speed: 如果传入此参数（例如状态机传来的速度）,则无视 default_speed,强制以此速度行走。
        """
        if not self.walker or not self.walker.is_alive or not self.target_list: 
            return
            
        # 速度决断:如果有外部状态机强制输入速度,则优先使用；否则使用默认速度
        target_speed = dynamic_speed if dynamic_speed is not None else self.default_speed

        # ---------------- 漫游模式的等待逻辑 ----------------
        if self.is_waiting:
            if current_time - self.wait_start_time >= self.roam_wait_time:
                self.is_waiting = False # 等待结束,继续走
                available = [t for t in self.target_list if t != self.current_target]
                self.current_target = random.choice(available) if available else self.current_target
            else:
                self.walker.apply_control(carla.WalkerControl(speed=0.0))
                return

        # 如果外部状态机要求速度为0（比如等红绿灯）,直接静止,跳过后续所有移动逻辑
        if target_speed <= 0.01:
            self.walker.apply_control(carla.WalkerControl(speed=0.0))
            return

        # ---------------- 获取目标与距离计算 ----------------
        loc = self.walker.get_location()
        tx, ty = _extract_xy(self.current_target)
        dx, dy = tx - loc.x, ty - loc.y
        dist = math.hypot(dx, dy)

        # ---------------- 抵达目标点逻辑 ----------------
        if dist <= self.arrival_dist:
            if self.mode == 'roam':
                self.is_waiting = True
                self.wait_start_time = current_time
                self.walker.apply_control(carla.WalkerControl(speed=0.0))
            elif self.mode == 'trajectory':
                self.traj_index += 1
                if self.traj_index < len(self.target_list):
                    self.current_target = self.target_list[self.traj_index]
                else:
                    self.walker.apply_control(carla.WalkerControl(speed=0.0)) # 轨迹彻底走完
            return

        # ---------------- 行走控制下发 ----------------
        ctrl = carla.WalkerControl()
        if dist > 1e-6:
            nx, ny = dx / dist, dy / dist
            ctrl.direction = carla.Vector3D(nx, ny, 0.0)
            
            # 只有严格循迹模式在靠近目标锚点时平滑减速防抖动
            if self.mode == 'trajectory' and dist < 1.5: 
                ctrl.speed = clamp(target_speed *(dist / 1.5), 0.25, target_speed)
            else:
                ctrl.speed = target_speed
                
        self.walker.apply_control(ctrl)

        # ---------------- 物理跨栏防卡死(Anti-stuck Step-Over) ----------------
        if self.last_loc is not None:
            moved_dist = loc.distance(self.last_loc)
            
            # 如果一帧内移动距离小于 1cm,认为卡住了
            if moved_dist < 0.01: 
                self.stuck_timer += dt
            else:
                self.stuck_timer = 0.0

            # 卡住超过 0.5 秒,触发最高 0.3米 的跨栏动作(应对马路牙子、低矮路障)
            if self.stuck_timer > 0.5:
                jump_loc = loc
                jump_loc.z += 0.3  # 抬高 0.3 米
                jump_loc.x += nx * 0.2 # 顺势向前推 0.2 米,完成跨栏
                jump_loc.y += ny * 0.2
                self.walker.set_location(jump_loc)
                self.stuck_timer = 0.0
                print(f"[RoadTailBench] 🚧 行人 {self.walker.id} 遇到低矮障碍,触发 0.3m 跨栏动作！")

        self.last_loc = loc

# ==========================================
# 10. 环境配置与生命周期(Environment Setup) 
# ==========================================

def check_vehicle_out_of_bounds(vehicle, carla_map, threshold_dist=6.0, auto_destroy=False):
    """
    【高鲁棒性出界守护与虚空拦截器】
    """
    # 【修复点】：如果车已经没了，直接视为“已出界/失效”，返回 True 阻止后续调用
    if not vehicle or not vehicle.is_alive:
        return True

    loc = vehicle.get_location()

    # 精确查找正下方是否有路面
    wp_exact = carla_map.get_waypoint(loc, project_to_road=False)
    # 广域搜索最近的合法路面
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)

    is_out = False

    if wp_nearest is None:
        is_out = True
        print(f"[RoadTailBench 守护] 🚨 车辆 [{vehicle.id}] 完全脱离世界网格,判定出界！")

    elif wp_exact is None:
        dist_to_road = wp_nearest.transform.location.distance(loc)
        if dist_to_road > threshold_dist:
            is_out = True
            print(f"[RoadTailBench 守护] 🚨 车辆 [{vehicle.id}] 偏离道路中心 {dist_to_road:.1f}m,判定出界！")

    if is_out and auto_destroy:
        try:
            vehicle.destroy()
        except:
            pass

    return is_out

# def check_vehicle_out_of_bounds(vehicle, carla_map, threshold_dist=6.0, auto_destroy=False):
#     """
#     【高鲁棒性出界守护与虚空拦截器】
#     原理:结合精确匹配与路网投影,完美识别“压草坪”与“飞出地图”的区别。
#
#     【参数】:
#         - threshold_dist: 允许偏离最近合法道路的最大距离（米）。默认 6.0 米（约两条车道宽）。
#         - auto_destroy: 如果判定出界,是否直接在函数内执行 .destroy()
#
#     【返回】:布尔值(True 表示已出界)
#     """
#     if not vehicle or not vehicle.is_alive:
#         return False
#
#     loc = vehicle.get_location()
#
#     # 精确查找正下方是否有路面
#     wp_exact = carla_map.get_waypoint(loc, project_to_road=False)
#     # 广域搜索最近的合法路面
#     wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
#
#     is_out = False
#
#     if wp_nearest is None:
#         # 极其罕见:完全脱离了整个路网世界（比如掉入无限地底）
#         is_out = True
#         print(f"[RoadTailBench 守护] 🚨 车辆 [{vehicle.id}] 完全脱离世界网格,判定出界！")
#
#     elif wp_exact is None:
#         # 正下方没有合法路,检查距离最近道路的投影距离
#         dist_to_road = wp_nearest.transform.location.distance(loc)
#         if dist_to_road > threshold_dist:
#             is_out = True
#             print(f"[RoadTailBench 守护] 🚨 车辆 [{vehicle.id}] 偏离道路中心 {dist_to_road:.1f}m,超出阈值 {threshold_dist}m,判定出界！")
#
#     if is_out and auto_destroy:
#         try:
#             vehicle.destroy()
#         except: pass
#
#     return is_out


def enable_synchronous_mode(world, dt=0.05):
    """
    【开启世界同步模式与高精物理引擎】
    作用:仅控制世界时间线和物理精度,绝不干涉其他模块。
    """
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    # 强化物理子步,防止高速穿模或剧烈碰撞崩溃
    settings.max_substeps = 10
    settings.max_substep_delta_time = dt / 10.0
    world.apply_settings(settings)
    print(f"[RoadTailBench] ⏱️ 已开启世界同步模式(dt={dt}s, {int(1.0/dt)} FPS)")


def disable_synchronous_mode(world):
    """
    【恢复世界异步模式】
    作用:让世界时间恢复自由流动,通常在测试结束或异常捕获时调用。
    """
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    print("[RoadTailBench] ⏱️ 已恢复异步模式,时间自由流动。")


def setup_traffic_manager(client, port=8000, sync_mode=True, hybrid_radius=100.0):
    """
    【独立初始化 Traffic Manager】
    作用:按需挂载 TM 控制器。如果不生成背景车流,就不要调用它。
    """
    tm = client.get_trafficmanager(port)
    tm.set_synchronous_mode(sync_mode)
    if hybrid_radius > 0:
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(hybrid_radius)
    print(f"[RoadTailBench] 🚦 Traffic Manager(Port:{port}) 已接管,混合物理半径: {hybrid_radius}m")
    return tm


def cleanup_actors(client, actor_list):
    """
    【底层批处理安全清理器】
    作用:无论实体是否因为碰撞、出界被提前销毁,都能安全地清扫剩余实体,绝不报错崩溃。
    """
    if not actor_list: return

    commands = []
    count = 0
    for actor in actor_list:
        if actor and actor.is_alive:
            commands.append(carla.command.DestroyActor(actor.id))
            count += 1

    if commands:
        client.apply_batch(commands)

    actor_list.clear() # 清空引用,释放内存
    print(f"[RoadTailBench] 🧹 环境清理完毕,成功销毁 {count} 个残留 Actors。")