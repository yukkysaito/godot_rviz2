# Motorcycle (no rider): blender -b --python tools/dynamic_object_models/motorcycle.py [-- output.glb]
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

WHEEL_R = 0.31
X_FRONT_AXLE, X_REAR_AXLE = 0.70, -0.70
STEERING_HEAD = (0.53, 0.0, 0.92)
SWINGARM_PIVOT = -0.15


def wheel(x, width, m):
    tire = c.cylinder_y(x, WHEEL_R, WHEEL_R, -width / 2, width / 2, m.tire, 12, "tire")
    tire = c.bevel(tire, 0.4 * width, 1)
    rim = c.cylinder_y(x, WHEEL_R, 0.21, -width / 2 - 0.004, width / 2 + 0.004, m.metal, 8, "rim")
    return [tire, rim]


def fender(x, width, a0, a1, m):
    """A mudguard following the tire from angle a0 to a1 (degrees, 0: forward)."""
    def arc(r):
        return [(x + r * math.cos(math.radians(a)), WHEEL_R + r * math.sin(math.radians(a)))
                for a in range(a0, a1 + 1, 20)]
    points = arc(WHEEL_R + 0.06) + arc(WHEEL_R + 0.04)[::-1]
    return c.prism(points, 'Y', -width / 2, width / 2, m.trim, "fender")


def body(m):
    parts = []
    # Tank, seat and tail
    parts.append(c.bevel(c.prism([(0.0, 0.82), (0.46, 0.88), (0.42, 1.0), (0.24, 1.07), (0.02, 1.0)],
                                 'Y', -0.16, 0.16, m.body, "tank"), 0.06, 3))
    parts.append(c.bevel(c.prism([(-0.62, 0.84), (0.03, 0.84), (0.0, 0.94), (-0.3, 0.92), (-0.62, 0.96)],
                                 'Y', -0.13, 0.13, m.trim, "seat"), 0.03, 3))
    parts.append(c.bevel(c.prism([(-0.88, 0.94), (-0.6, 0.84), (-0.55, 0.9), (-0.6, 0.97), (-0.88, 1.0)],
                                 'Y', -0.1, 0.1, m.body, "tail"), 0.025, 2))
    parts.append(c.box(-0.9, -0.06, 0.93, -0.87, 0.06, 0.98, m.taillight, "taillight"))
    parts.append(c.tube((-0.82, 0.0, 0.88), (-0.95, 0.0, 0.66), 0.015, m.trim))  # license plate holder
    parts.append(c.box(-0.97, -0.09, 0.56, -0.95, 0.09, 0.68, m.trim, "plate"))
    # Engine with its cylinders, under the tank
    parts.append(c.box(-0.15, -0.16, 0.28, 0.32, 0.16, 0.6, m.trim, "crankcase"))
    parts.append(c.box(0.04, -0.14, 0.58, 0.34, 0.14, 0.8, m.metal, "cylinders"))
    parts.append(fender(X_REAR_AXLE, 0.18, 60, 130, m))
    # Exhaust on the right
    parts.append(c.tube((0.32, -0.08, 0.62), (0.36, -0.12, 0.3), 0.025, m.metal))
    parts.append(c.tube((0.36, -0.12, 0.3), (-0.3, -0.17, 0.32), 0.025, m.metal))
    parts.append(c.prism(c.circle(-0.17, 0.42, 0.065, 8), 'X', -0.78, -0.28, m.metal, "muffler"))
    # Frame, swingarm and rear shocks
    for y in (0.1, -0.1):
        parts.append(c.tube(STEERING_HEAD, (SWINGARM_PIVOT, y, 0.82), 0.025, m.trim))
        parts.append(c.tube((SWINGARM_PIVOT, y, 0.82), (SWINGARM_PIVOT, y, 0.4), 0.025, m.trim))
        parts.append(c.tube((SWINGARM_PIVOT, y * 1.1, 0.42), (X_REAR_AXLE, y * 1.1, WHEEL_R), 0.028, m.trim))
        parts.append(c.tube((-0.48, y * 1.1, 0.36), (-0.32, y * 1.1, 0.84), 0.022, m.metal))
        parts.append(c.box(-0.05, y * 1.6 - 0.04, 0.33, 0.03, y * 1.6 + 0.04, 0.36, m.trim, "footpeg"))
    parts.append(c.tube(STEERING_HEAD, (0.3, 0.0, 0.4), 0.03, m.trim))  # down tube
    return parts


def front(m):
    parts = []
    for y in (0.09, -0.09):  # fork
        parts.append(c.tube((X_FRONT_AXLE, y, WHEEL_R), (0.6, y, 0.7), 0.03, m.metal))
        parts.append(c.tube((0.6, y, 0.7), (0.5, y, 1.0), 0.025, m.trim))
    parts.append(c.box(0.46, -0.13, 0.94, 0.58, 0.13, 1.0, m.trim, "clamp"))
    parts.append(fender(X_FRONT_AXLE, 0.14, 30, 140, m))
    parts.append(c.prism(c.circle(0.0, 0.88, 0.09, 8), 'X', 0.56, 0.67, m.trim, "headlight_case"))
    parts.append(c.prism(c.circle(0.0, 0.88, 0.075, 8), 'X', 0.67, 0.68, m.headlight, "headlight"))
    parts.append(c.tube((0.45, -0.36, 1.04), (0.45, 0.36, 1.04), 0.012, m.metal, 6, "handlebar"))
    for y in (1, -1):
        parts.append(c.tube((0.45, y * 0.28, 1.04), (0.45, y * 0.38, 1.04), 0.018, m.trim, 6, "grip"))
        parts.append(c.tube((0.46, y * 0.26, 1.04), (0.44, y * 0.31, 1.22), 0.007, m.metal, 6, "mirror_stem"))
        parts.append(c.box(0.42, y * 0.31 - 0.06, 1.2, 0.46, y * 0.31 + 0.06, 1.27, m.trim, "mirror"))
    return parts


def build(path):
    c.reset_scene()
    m = c.Materials("Motorcycle")
    parts = body(m) + front(m) + wheel(X_FRONT_AXLE, 0.12, m) + wheel(X_REAR_AXLE, 0.17, m)
    c.export_model(parts, path)


if __name__ == "__main__":
    c.main(build, "Motorcycle.glb")
