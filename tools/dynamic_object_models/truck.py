# Cab-over box truck: blender -b --python tools/dynamic_object_models/truck.py [-- output.glb]
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

L = 7.6
XF, XR = L / 2, -L / 2
CAB_HW, BOX_HW = 1.1, 1.15  # half widths
X_CAB_REAR, X_BOX_FRONT = 2.05, 1.95
CAB_BOTTOM, CAB_TOP = 0.55, 2.85
BOX_BOTTOM, BOX_TOP = 1.0, 3.2
WHEEL_R = 0.46
X_FRONT_AXLE, X_REAR_AXLE = 2.65, -1.9
# Box sections repeated for the length, between the seams at the middle of ribs
BAY = 1.1
X_FRONT_SEAM = 1.0
X_REAR_SEAM = X_FRONT_SEAM - 2 * BAY
LEFT, RIGHT = 1, -1


def chassis(m):
    parts = []
    for side in (LEFT, RIGHT):
        y0, y1 = c.span(side, 0.38, 0.48)
        parts.append(c.box(XR + 0.1, y0, 0.62, 3.3, y1, 0.98, m.trim, "frame"))
        # Side guard between the axles
        y0, y1 = c.span(side, 1.02, 1.07)
        x0, x1 = X_REAR_AXLE + 0.6, X_FRONT_AXLE - 0.6
        for z in (0.55, 0.78):
            parts.append(c.box(x0, y0, z, x1, y1, z + 0.07, m.metal, "guard"))
        # Fender over the rear wheels and the mud flap behind them
        y0, y1 = c.span(side, 0.6, 1.12)
        parts.append(c.box(X_REAR_AXLE - 0.6, y0, 0.94, X_REAR_AXLE + 0.6, y1, 1.0, m.trim, "fender"))
        parts.append(c.box(X_REAR_AXLE - 0.64, y0, 0.25, X_REAR_AXLE - 0.6, y1, 0.95, m.trim, "flap"))
    parts.append(c.box(1.15, -1.0, 0.5, 1.85, -0.55, 0.95, m.metal, "fuel_tank"))
    # Rear: under-run bar and the lights on a bar under the box
    parts.append(c.box(XR + 0.05, -1.0, 0.4, XR + 0.15, 1.0, 0.55, m.trim, "underrun"))
    parts.append(c.box(XR + 0.06, -1.05, 0.62, XR + 0.16, 1.05, 0.8, m.trim, "light_bar"))
    for side in (LEFT, RIGHT):
        y0, y1 = c.span(side, 0.6, 1.0)
        parts.append(c.box(XR + 0.02, y0, 0.64, XR + 0.07, y1, 0.78, m.taillight, "taillight"))
    return parts


def build(path):
    c.reset_scene()
    m = c.Materials("Truck")
    ribs = [X_FRONT_SEAM - k * BAY for k in range(5)]  # at the seams, and on to the rear
    parts = c.cab_over(XF, X_CAB_REAR, CAB_HW, CAB_BOTTOM, CAB_TOP, X_FRONT_AXLE, WHEEL_R, m)
    parts += c.van_box(X_BOX_FRONT, XR, BOX_HW, BOX_BOTTOM, BOX_TOP, ribs, m) + chassis(m)
    for side in (LEFT, RIGHT):
        parts += c.wheel(X_FRONT_AXLE, side, side * (CAB_HW - 0.04), WHEEL_R, 0.26, m)
        parts += c.wheel(X_REAR_AXLE, side, side * (BOX_HW - 0.08), WHEEL_R, 0.5, m)
    c.export_length_fitted(parts, XF, X_FRONT_SEAM, BAY, X_REAR_SEAM, XR, path)


if __name__ == "__main__":
    c.main(build, "Truck.glb")
