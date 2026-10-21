# Bicycle (no rider): blender -b --python tools/dynamic_object_models/bicycle.py [-- output.glb]
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

WHEEL_R = 0.34
X_FRONT_AXLE, X_REAR_AXLE = 0.53, -0.52
BOTTOM_BRACKET = (-0.02, 0.0, 0.30)
SEAT = (-0.17, 0.0, 0.82)
HEAD_TOP, HEAD_BOTTOM = (0.38, 0.0, 0.84), (0.43, 0.0, 0.68)


def wheel(x, m):
    return [c.torus_y(x, 0, WHEEL_R, WHEEL_R - 0.02, 0.02, m.tire, 16, 4, "tire")]


def frame(m):
    parts = [c.tube(SEAT, HEAD_TOP, 0.017, m.body),  # top tube
             c.tube(BOTTOM_BRACKET, HEAD_BOTTOM, 0.022, m.body),  # down tube
             c.tube(BOTTOM_BRACKET, SEAT, 0.017, m.body),  # seat tube
             c.tube(HEAD_BOTTOM, HEAD_TOP, 0.024, m.body)]  # head tube
    for y in (0.06, -0.06):
        rear = (X_REAR_AXLE, y, WHEEL_R)
        parts.append(c.tube(BOTTOM_BRACKET, rear, 0.011, m.body))  # chain stay
        parts.append(c.tube(SEAT, rear, 0.010, m.body))  # seat stay
        fork_top = (HEAD_BOTTOM[0], y * 0.8, HEAD_BOTTOM[2])
        parts.append(c.tube(fork_top, (X_FRONT_AXLE, y * 0.8, WHEEL_R), 0.012, m.body))
    # Saddle, stem and handlebar
    parts.append(c.tube(SEAT, (-0.2, 0.0, 0.92), 0.013, m.metal))
    parts.append(c.box(-0.32, -0.07, 0.92, -0.08, 0.07, 0.97, m.trim, "saddle"))
    parts.append(c.tube(HEAD_TOP, (0.41, 0.0, 0.94), 0.014, m.metal))
    parts.append(c.tube((0.41, 0.0, 0.94), (0.47, 0.0, 0.95), 0.014, m.metal))
    parts.append(c.tube((0.47, -0.25, 0.95), (0.47, 0.25, 0.95), 0.012, m.metal, 6, "handlebar"))
    for y in (0.2, -0.2):
        parts.append(c.tube((0.47, y, 0.95), (0.47, y * 1.3, 0.95), 0.018, m.trim, 6, "grip"))
    # Drive: chainring and cranks on the right, pedals
    bx, _, bz = BOTTOM_BRACKET
    parts.append(c.cylinder_y(bx, bz, 0.1, -0.075, -0.068, m.metal, 8, "chainring"))
    for y, a in ((-0.085, 0.0), (0.085, math.pi)):
        end = (bx + 0.17 * math.cos(a + 0.6), y, bz + 0.17 * math.sin(a + 0.6))
        parts.append(c.tube((bx, y, bz), end, 0.012, m.metal, 6, "crank"))
        y0, y1 = sorted((y, y + math.copysign(0.09, y)))
        parts.append(c.box(end[0] - 0.05, y0, end[2] - 0.012, end[0] + 0.05, y1, end[2] + 0.012, m.trim,
                           "pedal"))
    return parts


def build(path):
    c.reset_scene()
    m = c.Materials("Bicycle")
    c.export_model(frame(m) + wheel(X_FRONT_AXLE, m) + wheel(X_REAR_AXLE, m), path)


if __name__ == "__main__":
    c.main(build, "Bicycle.glb")
