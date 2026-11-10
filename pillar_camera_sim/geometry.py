"""Pure projection functions (no ROS dependency), in metres and pixels."""
import math


def project_point(x, y, z, cx, cy, cz, yaw, fx, fy, width, height):
    """Map xyz -> camera uv,depth. World X forward, Y left, Z up."""
    dx, dy = x - cx, y - cy
    depth = math.cos(yaw) * dx + math.sin(yaw) * dy
    right = math.sin(yaw) * dx - math.cos(yaw) * dy
    if depth <= 0.05:
        return None
    u = width / 2.0 + fx * right / depth
    v = height / 2.0 - fy * (z - cz) / depth
    return (u, v, depth)


def estimate_from_two_centers(u_left, u_right, baseline_m, fx, image_width):
    """Fronto-parallel, equal-depth assumption. Signed offset: image-right positive."""
    separation = abs(u_right - u_left)
    if separation < 1e-6:
        return None
    distance = fx * baseline_m / separation
    midpoint = (u_left + u_right) / 2.0
    offset_px = midpoint - image_width / 2.0
    return distance, offset_px, offset_px * distance / fx
