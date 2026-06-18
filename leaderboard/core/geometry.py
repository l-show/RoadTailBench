import math


EPS = 1e-6


def clamp(value, lo=0.0, hi=1.0):
    return max(lo, min(hi, value))


def vec2(value, default=(0.0, 0.0)):
    if value is None:
        return default
    return (float(value[0]), float(value[1]))


def vec3(value, default=(0.0, 0.0, 0.0)):
    if value is None:
        return default
    return (float(value[0]), float(value[1]), float(value[2]))


def norm2(value):
    return math.hypot(value[0], value[1])


def sub2(a, b):
    return (a[0] - b[0], a[1] - b[1])


def dot2(a, b):
    return a[0] * b[0] + a[1] * b[1]


def distance2(a, b):
    return norm2(sub2(a, b))


def project_point_to_polyline(point, polyline):
    if not polyline:
        return 0.0, float("inf"), 0
    if len(polyline) == 1:
        return 0.0, distance2(point, polyline[0]), 0

    accum = [0.0]
    for i in range(1, len(polyline)):
        accum.append(accum[-1] + distance2(polyline[i - 1], polyline[i]))

    best_s = 0.0
    best_d = float("inf")
    best_i = 0
    px, py = point
    for i in range(len(polyline) - 1):
        ax, ay = polyline[i]
        bx, by = polyline[i + 1]
        ab = (bx - ax, by - ay)
        ap = (px - ax, py - ay)
        denom = dot2(ab, ab)
        t = 0.0 if denom < EPS else clamp(dot2(ap, ab) / denom)
        proj = (ax + ab[0] * t, ay + ab[1] * t)
        d = distance2(point, proj)
        if d < best_d:
            best_d = d
            best_s = accum[i] + distance2(polyline[i], proj)
            best_i = i
    return best_s, best_d, best_i


def yaw_to_forward(yaw_deg):
    yaw = math.radians(float(yaw_deg))
    return (math.cos(yaw), math.sin(yaw))


def point_xy(point):
    if isinstance(point, dict):
        loc = point.get("location", point)
        if isinstance(loc, dict):
            return (float(loc.get("x", 0.0)), float(loc.get("y", 0.0)))
        return (float(loc[0]), float(loc[1]))
    return (float(point[0]), float(point[1]))


def angle_delta_deg(a, b):
    delta = (float(a) - float(b) + 180.0) % 360.0 - 180.0
    return abs(delta)


def polyline_lengths(polyline):
    lengths = [0.0]
    for index in range(1, len(polyline)):
        lengths.append(lengths[-1] + distance2(polyline[index - 1], polyline[index]))
    return lengths


def sample_polyline_at_s(polyline, distances, s):
    if not polyline:
        return (0.0, 0.0), 0
    if len(polyline) == 1 or s <= 0.0:
        return polyline[0], 0
    if s >= distances[-1]:
        return polyline[-1], max(0, len(polyline) - 2)
    for index in range(len(polyline) - 1):
        if distances[index] <= s <= distances[index + 1]:
            span = max(distances[index + 1] - distances[index], EPS)
            t = clamp((s - distances[index]) / span)
            ax, ay = polyline[index]
            bx, by = polyline[index + 1]
            return (ax + (bx - ax) * t, ay + (by - ay) * t), index
    return polyline[-1], max(0, len(polyline) - 2)
