# Car: blender -b --python tools/dynamic_object_models/car.py [-- output.glb]
# The body keeps the shape of the original model (car_body.obj); this adds the glass, the lights
# and the wheels to it. Seen from afar: no wheel arches, few parts.
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

XF, XR, HW = 2.4, -2.39, 1.03
WHEEL_R, WHEEL_W = 0.36, 0.24
X_AXLES = (1.67, -1.46)
LEFT, RIGHT = 1, -1


def build(path):
    c.reset_scene()
    m = c.Materials("Car")
    s = c.Surface(c.load_obj("car_body.obj", m.body))
    # Glass: windshield, rear window, side windows (the pillars are the body between them)
    s.inlay(c.end_region(-0.72, 1.2, 0.72, 1.56, 0.06, 1.2, 1.0), m.glass)
    s.inlay(c.end_region(-0.68, 1.22, 0.68, 1.55, 0.06, -2.1, 1.2), m.glass)
    for side in (LEFT, RIGHT):
        for window in ([(-0.1, 1.18), (1.02, 1.18), (0.36, 1.55), (-0.1, 1.55)],
                       [(-1.7, 1.19), (-0.22, 1.18), (-0.22, 1.55), (-1.0, 1.55), (-1.36, 1.42)]):
            s.inlay(c.side_polygon(window, side, HW, 0.45), m.glass)
        y0, y1 = c.span(side, 0.45, HW + 0.1)
        s.inlay(c.end_region(y0, 0.72, y1, 0.84, 0.04, XF, 0.45), m.headlight)
        s.inlay(c.end_region(y0, 0.8, y1, 0.93, 0.03, XR, 0.5), m.taillight)
    s.inlay(c.end_region(-0.42, 0.45, 0.42, 0.63, 0.04, XF), m.trim)  # grille
    parts = s.parts()
    for side in (LEFT, RIGHT):
        y0, y1 = c.span(side, HW - 0.1, HW + 0.12)
        parts.append(c.box(0.95, y0, 1.1, 1.1, y1, 1.21, m.body, "mirror"))
        for x in X_AXLES:  # outside the body, as originally
            parts += c.wheel(x, side, side * (HW + 0.005), WHEEL_R, WHEEL_W, m)
    c.export_model(parts, path)


if __name__ == "__main__":
    c.main(build, "Car.glb")
