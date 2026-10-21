# City bus: blender -b --python tools/dynamic_object_models/bus.py [-- output.glb]
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

L, W = 10.5, 2.5
XF, XR, HW = L / 2, -L / 2, W / 2
Z_BOTTOM, Z_ROOF = 0.32, 3.0
WHEEL_R, WHEEL_W = 0.5, 0.28
X_FRONT_AXLE, X_REAR_AXLE = 2.85, -2.45
# Window bays repeated for the length, between the seams at the middle of pillars
BAY = 1.35
BAYS = 2  # as modeled
X_FRONT_SEAM = 2.20
X_REAR_SEAM = X_FRONT_SEAM - BAYS * BAY
LEFT, RIGHT = 1, -1  # the doors are on the left (curb side in left-hand traffic)


def body_edge_weight(e):
    a, b = e.verts
    if abs(b.co.y - a.co.y) < 0.5:  # outline of the sides: plan corners, roof and bottom sides
        return 0.25 if max(a.co.z, b.co.z) <= Z_BOTTOM + 1e-3 else 0.85
    x, z = a.co.x, a.co.z  # across the width: the corners of the side profile
    if z >= Z_ROOF - 1e-3 or (z > 2.7 and x > 0):
        return 1.0
    if z <= Z_BOTTOM + 1e-3:
        return 0.25
    return 0.3 if x > 0 and z < 0.7 else 0.15


def build(path):
    c.reset_scene()
    m = c.Materials("Bus")
    # Side profile with a slightly raked windshield
    profile = [(XR, Z_BOTTOM), (XF - 0.06, Z_BOTTOM), (XF, 0.62), (XF, 1.05),
               (XF - 0.16, 2.78), (XF - 0.40, Z_ROOF), (XR + 0.12, Z_ROOF), (XR, 2.85)]
    body = c.beveled_prism(profile, 'Y', -HW, HW, body_edge_weight, 0.3, 2, m.body, "body")
    for x in (X_FRONT_AXLE, X_REAR_AXLE):
        for side in (LEFT, RIGHT):
            c.boolean(body, c.cylinder_y(x, WHEEL_R, 0.60, *c.span(side, 0.75, 1.6)))
    s = c.Surface(body)

    def side(x0, z0, x1, z1, r, which):
        return c.side_region(x0, z0, x1, z1, r, which, HW)

    def front(y0, z0, y1, z1, r):
        return c.end_region(y0, z0, y1, z1, r, XF)

    def rear(y0, z0, y1, z1, r):
        return c.end_region(y0, z0, y1, z1, r, XR)

    # Front: windshield around the corners, destination sign, bumper, headlights
    s.inlay(c.end_region(-HW - 0.3, 1.08, HW + 0.3, 2.70, 0.0, XF, 0.32), m.glass)
    s.inlay(front(-0.85, 2.77, 0.85, 2.89, 0.02), m.sign)
    s.inlay(c.end_region(-HW - 0.3, Z_BOTTOM - 0.1, HW + 0.3, 0.70, 0.0, XF, 0.25), m.trim)
    for y in (LEFT, RIGHT):
        y0, y1 = c.span(y, 0.72, 1.12)
        s.inlay(front(y0, 0.80, y1, 0.98, 0.0), m.headlight)
    # Rear: window, bumper, tail lights
    s.inlay(rear(-0.85, 2.00, 0.85, 2.68, 0.08), m.glass)
    s.inlay(c.end_region(-HW - 0.3, Z_BOTTOM - 0.1, HW + 0.3, 0.68, 0.0, XR, 0.25), m.trim)
    for y in (LEFT, RIGHT):
        y0, y1 = c.span(y, 0.88, 1.16)
        s.inlay(rear(y0, 0.80, y1, 1.65, 0.0), m.taillight)

    # Side windows: the pillars are the body between the panes
    z0, z1 = 1.38, 2.70
    bays = [(X_FRONT_SEAM - (k + 1) * BAY + 0.06, X_FRONT_SEAM - k * BAY - 0.06) for k in range(BAYS)]
    left = [(X_FRONT_SEAM + 0.06, 3.50)] + bays + [(-3.20, -1.92), (-4.80, -3.32)]
    right = [(X_FRONT_SEAM + 0.06, 3.58)] + bays + [(X_REAR_SEAM - 1.35, X_REAR_SEAM - 0.06), (-3.20, -1.97),
                                                    (-4.80, -3.32)]
    for x0, x1 in left:
        s.inlay(side(x0, z0, x1, z1, 0.07, LEFT), m.glass)
    s.inlay(side(3.70, 1.22, 4.88, z1, 0.07, RIGHT), m.glass)  # driver's window
    for x0, x1 in right:
        s.inlay(side(x0, z0, x1, z1, 0.07, RIGHT), m.glass)
    for x0, x1 in [(3.62, 4.78), (X_REAR_SEAM - 1.30, X_REAR_SEAM - 0.06)]:  # doors
        s.inlay(side(x0, 0.45, x1, 2.70, 0.04, LEFT), m.glass)
    parts = s.parts()

    # Air conditioner, on the rear: what lies between the seams is repeated
    roof_unit = c.prism(c.rounded_rect(-3.3, -0.95, -0.7, 0.95, 0.35, 5), 'Z', Z_ROOF - 0.05, Z_ROOF + 0.2,
                        m.body)
    parts.append(c.bevel(roof_unit, 0.07, 1))
    for y in (LEFT, RIGHT):
        arm = [(XF - 0.03, y * 1.10, 2.62), (XF + 0.38, y * 1.10, 2.62), (XF + 0.38, y * 1.10, 2.36)]
        parts += c.mirror_on_arm(arm, (XF + 0.40, y * 1.16, 2.16), (0.08, 0.26, 0.42), m)
    for x in (X_FRONT_AXLE, X_REAR_AXLE):
        for y in (LEFT, RIGHT):
            parts += c.wheel(x, y, y * (HW - 0.03), WHEEL_R, WHEEL_W, m)
    c.export_length_fitted(parts, XF, X_FRONT_SEAM, BAY, X_REAR_SEAM, XR, path)


if __name__ == "__main__":
    c.main(build, "Bus.glb")
