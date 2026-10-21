# Helpers shared by the dynamic object model scripts (run inside Blender).
#
# Coordinates: +X forward, +Y left, +Z up, in meters; the origin is on the ground at the center.
# A model is built as solid parts with details inlaid into their surface, and then exported (buses
# and trucks split into the nodes LengthFittedModel.gd expects, see README.md).
import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
GODOT_MODEL_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "godot-project/3DViewer/DynamicObject"))


def main(build, file_name):
    """Runs build(path): blender -b --python <script> [-- <path>], by default into the project."""
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    build(args[0] if args else os.path.join(GODOT_MODEL_DIR, file_name))


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def load_obj(name, material):
    """A mesh kept next to these scripts as an OBJ file."""
    bpy.ops.wm.obj_import(filepath=os.path.join(HERE, name), forward_axis='Y', up_axis='Z')
    o = bpy.context.selected_objects[0]
    o.data.materials.clear()
    o.data.materials.append(material)
    return o


def span(side, a, b):
    """The interval from side * a to side * b, in order (side: 1 left, -1 right)."""
    return tuple(sorted((side * a, side * b)))


def _material(name, color):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    m.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (*color, 1.0)
    return m


class Materials:
    """The colors of the parts. They are materials while building, and vertex colors in the end."""

    def __init__(self, prefix):
        def m(name, color):
            return _material(prefix + name, color)
        self.body = m("Body", (0.82, 0.83, 0.84))
        self.glass = m("Glass", (0.03, 0.04, 0.05))
        self.trim = m("Trim", (0.10, 0.10, 0.11))
        self.metal = m("Metal", (0.55, 0.56, 0.58))
        self.tire = m("Tire", (0.03, 0.03, 0.03))
        self.headlight = m("Headlight", (1.0, 0.98, 0.9))
        self.taillight = m("Taillight", (0.8, 0.04, 0.04))
        self.sign = m("Sign", (1.0, 0.55, 0.05))


# ================== Primitives ==================
def _object(bm, name, material):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    if material is not None:
        me.materials.append(material)
    return o


def apply_modifiers(o):
    bpy.context.view_layer.objects.active = o
    for m in list(o.modifiers):
        bpy.ops.object.modifier_apply(modifier=m.name)


def bevel(o, width, segments=3, limit='ANGLE'):
    m = o.modifiers.new("bevel", 'BEVEL')
    m.width = width
    m.segments = segments
    m.limit_method = limit
    apply_modifiers(o)
    return o


def prism(points, axis, a0, a1, material=None, name="prism"):
    """The polygon `points` extruded along `axis` from a0 to a1.

    The points are (u, v) across the axis: (y, z) for X, (x, z) for Y and (x, y) for Z.
    """
    def co(u, v, a):
        return {'X': (a, u, v), 'Y': (u, a, v), 'Z': (u, v, a)}[axis]
    bm = bmesh.new()
    bottom = [bm.verts.new(co(u, v, a0)) for u, v in points]
    top = [bm.verts.new(co(u, v, a1)) for u, v in points]
    bm.faces.new(bottom[::-1])
    bm.faces.new(top)
    for i in range(len(points)):
        j = (i + 1) % len(points)
        bm.faces.new((bottom[i], bottom[j], top[j], top[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _object(bm, name, material)


def beveled_prism(points, axis, a0, a1, weight, width, segments, material=None, name="prism"):
    """A prism with rounded edges: weight(edge) (0 to 1) scales `width` per edge."""
    o = prism(points, axis, a0, a1, material, name)
    bm = bmesh.new()
    bm.from_mesh(o.data)
    layer = bm.edges.layers.float.get("bevel_weight_edge") or bm.edges.layers.float.new("bevel_weight_edge")
    for e in bm.edges:
        e[layer] = weight(e)
    bm.to_mesh(o.data)
    bm.free()
    m = o.modifiers.new("bevel", 'BEVEL')
    m.limit_method = 'WEIGHT'
    m.width = width
    m.segments = segments
    apply_modifiers(o)
    return o


def rounded_rect(u0, v0, u1, v1, r, segments=1):
    if r <= 0.0:
        return [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    points = []
    corners = ((u1 - r, v0 + r, -90), (u1 - r, v1 - r, 0), (u0 + r, v1 - r, 90), (u0 + r, v0 + r, 180))
    for cu, cv, start in corners:
        for k in range(segments + 1):
            a = math.radians(start + 90 * k / segments)
            points.append((cu + r * math.cos(a), cv + r * math.sin(a)))
    return points


def circle(cu, cv, r, n):
    angles = [2 * math.pi * i / n for i in range(n)]
    return [(cu + r * math.cos(a), cv + r * math.sin(a)) for a in angles]


def box(x0, y0, z0, x1, y1, z1, material=None, name="box"):
    return prism(rounded_rect(x0, y0, x1, y1, 0.0), 'Z', z0, z1, material, name)


def cylinder_y(x, z, r, y0, y1, material=None, n=12, name="cylinder"):
    return prism(circle(x, z, r, n), 'Y', min(y0, y1), max(y0, y1), material, name)


def tube(p0, p1, r, material=None, n=6, name="tube"):
    """A cylinder from p0 to p1."""
    p0, p1 = Vector(p0), Vector(p1)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=n, radius1=r, radius2=r, depth=(p1 - p0).length)
    rotation = Vector((0, 0, 1)).rotation_difference(p1 - p0).to_matrix().to_4x4()
    bmesh.ops.transform(bm, matrix=Matrix.Translation((p0 + p1) / 2) @ rotation, verts=bm.verts)
    return _object(bm, name, material)


def torus_y(x, y, z, major, minor, material=None, segments=24, minor_segments=6, name="torus"):
    """A ring around the Y axis, like a thin tire."""
    bpy.ops.mesh.primitive_torus_add(major_radius=major, minor_radius=minor, major_segments=segments,
                                     minor_segments=minor_segments, location=(x, y, z),
                                     rotation=(math.pi / 2, 0, 0))
    o = bpy.context.active_object
    o.name = name
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if material is not None:
        o.data.materials.append(material)
    return o


def boolean(target, cutter, operation='DIFFERENCE', self_intersecting=False, transfer_material=False):
    m = target.modifiers.new("boolean", 'BOOLEAN')
    m.operation = operation
    m.solver = 'EXACT'
    m.use_self = self_intersecting
    m.object = cutter
    if transfer_material:  # the faces from the cutter keep its material
        m.material_mode = 'TRANSFER'
    apply_modifiers(target)
    bpy.data.objects.remove(cutter, do_unlink=True)


def join(objects, name):
    bpy.ops.object.select_all(action='DESELECT')
    for o in objects:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.join()
    o = bpy.context.view_layer.objects.active
    o.name = name
    o.data.name = name
    return o


# ================== Regions ==================
# Prisms that select a part of a surface for the details: through a side wall, or through an end
def side_region(x0, z0, x1, z1, r, side, half_width, depth=0.9):
    """Through the left (side=1) or right (side=-1) wall."""
    return side_polygon(rounded_rect(x0, z0, x1, z1, r), side, half_width, depth)


def side_polygon(points, side, half_width, depth=0.9):
    """Like side_region(), for the polygon `points` (x, z)."""
    return prism(points, 'Y', side * (half_width - depth), side * (half_width + 0.3))


def end_region(y0, z0, y1, z1, r, x_end, depth=0.5):
    """Through the front (x_end > 0) or rear end at x_end."""
    sign = 1.0 if x_end > 0 else -1.0
    a0, a1 = sorted((x_end - sign * depth, x_end + sign * 0.3))
    return prism(rounded_rect(y0, z0, y1, z1, r), 'X', a0, a1)


# ================== Details ==================
class Surface:
    """Inlays other materials (glass, lights, trim) into the surface of a closed solid.

    The faces of the solid are replaced, not covered: drawn see-through, the models are sorted, not
    depth tested, against themselves, so a layer over the body would flicker with the view.
    """

    def __init__(self, solid):
        self.solid = solid
        # The inlays follow the surface as it was first: the cuts made would disturb the later ones
        self._base = solid.copy()
        self._base.data = solid.data.copy()

    def inlay(self, region, material, depth=0.01):
        """Gives `material` to the surface inside `region`, sunk by `depth`."""
        out = 0.05  # past the surface, so that no face of the cutter lies on it
        cutter = self._base.copy()
        cutter.data = self._base.data.copy()
        bpy.context.scene.collection.objects.link(cutter)
        m = cutter.modifiers.new("solidify", 'SOLIDIFY')
        m.thickness = depth + out
        m.offset = 2 * out / (depth + out) - 1
        apply_modifiers(cutter)
        # The offset surface overlaps itself at concave edges (e.g. around the wheel arches)
        boolean(cutter, region, 'INTERSECT', self_intersecting=True)
        cutter.data.materials.clear()
        cutter.data.materials.append(material)
        boolean(self.solid, cutter, transfer_material=True)

    def parts(self):
        bpy.data.objects.remove(self._base, do_unlink=True)
        return [self.solid]


def wheel(x, side, outer_y, radius, width, materials):
    """A tire with its wheel; `outer_y` is the outer face (simple: the models are seen from afar)."""
    inward = -side
    tire = cylinder_y(x, radius, radius, outer_y, outer_y + inward * width, materials.tire, 12, "tire")
    hub = cylinder_y(x, radius, 0.62 * radius, outer_y + inward * 0.05, outer_y + side * 0.005,
                     materials.metal, 8, "hub")
    return [bevel(tire, min(0.06, 0.3 * width), 1), hub]


def van_box(x_front, x_rear, half_width, z0, z1, rib_xs, materials):
    """A cargo box with ribs on its sides and two doors at the rear; returns its parts."""
    m = materials
    s = Surface(box(x_rear, -half_width, z0, x_front, half_width, z1, m.body, "van"))
    for side in (1, -1):
        for x in rib_xs:
            s.inlay(side_region(x - 0.04, z0 + 0.12, x + 0.04, z1 - 0.12, 0.0, side, half_width), m.metal)
        # Rails along the bottom and the top, between the ribs and the edges (not on the roof)
        for a, b in ((z0 + 0.005, z0 + 0.12), (z1 - 0.12, z1 - 0.005)):
            s.inlay(side_region(x_rear, a, x_front, b, 0.0, side, half_width), m.metal)
    # Rear: a frame with the two doors in it (deeper, not to cut the frame's faces again)
    s.inlay(end_region(-half_width - 0.1, z0 - 0.1, half_width + 0.1, z1 + 0.1, 0.0, x_rear), m.metal)
    for side in (1, -1):
        y0, y1 = span(side, 0.015, half_width - 0.07)
        s.inlay(end_region(y0, z0 + 0.1, y1, z1 - 0.1, 0.0, x_rear), m.body, 0.02)
    return s.parts()


def cab_over(x_front, x_rear, half_width, z0, z1, x_axle, wheel_radius, materials):
    """The cab of a cab-over truck, over the front wheels at x_axle; returns its parts."""
    m = materials
    hw = half_width

    def edge_weight(e):
        a, b = e.verts
        if abs(b.co.y - a.co.y) < 0.5:  # outline of the sides
            return 0.15 if max(a.co.z, b.co.z) <= z0 + 1e-3 else 0.7
        x, z = a.co.x, a.co.z  # across the width: the corners of the side profile
        if z > z1 - 0.35 and x > x_front - 0.8:
            return 1.0
        return 0.3 if z >= z1 - 1e-3 else 0.15

    profile = [(x_rear, z0), (x_front - 0.06, z0), (x_front, z0 + 0.35), (x_front, z0 + 0.9),
               (x_front - 0.08, z1 - 0.23), (x_front - 0.28, z1), (x_rear, z1)]
    solid = beveled_prism(profile, 'Y', -hw, hw, edge_weight, 0.25, 2, m.body, "cab")
    for side in (1, -1):  # wheel arches
        y0, y1 = span(side, hw - 0.5, hw + 0.3)
        boolean(solid, cylinder_y(x_axle, wheel_radius, wheel_radius + 0.1, y0, y1))
    s = Surface(solid)
    s.inlay(end_region(-hw - 0.3, z0 + 1.05, hw + 0.3, z1 - 0.33, 0.0, x_front, 0.3), m.glass)  # windshield
    for side in (1, -1):
        s.inlay(side_region(x_rear + 0.25, z0 + 1.07, x_front - 0.4, z1 - 0.33, 0.06, side, hw), m.glass)
        for x in (x_rear + 0.15, x_front - 0.28):  # door edges
            s.inlay(side_region(x - 0.012, z0 + 0.1, x + 0.012, z1 - 0.25, 0.0, side, hw), m.trim)
        y0, y1 = span(side, hw - 0.4, hw - 0.1)
        s.inlay(end_region(y0, z0 + 0.19, y1, z0 + 0.35, 0.05, x_front), m.headlight)
    s.inlay(end_region(-0.6, z0 + 0.45, 0.6, z0 + 0.83, 0.04, x_front), m.trim)  # grille
    parts = s.parts()
    parts.append(box(x_front - 0.1, -hw, z0 - 0.13, x_front + 0.08, hw, z0 + 0.17, m.trim, "bumper"))
    for side in (1, -1):
        x, y = x_front - 0.25, side * (hw + 0.2)
        arm = [(x, side * (hw - 0.02), z0 + 1.8), (x, y, z0 + 1.8), (x, y, z0 + 1.65)]
        parts += mirror_on_arm(arm, (x, side * (hw + 0.26), z0 + 1.4), (0.06, 0.2, 0.48), m)
    return parts


def mirror_on_arm(points, mirror_center, mirror_size, materials):
    """A mirror (box) held by an arm through `points`."""
    parts = []
    for a, b in zip(points, points[1:]):
        a, b = Vector(a), Vector(b)
        lo = Vector(map(min, a, b)) - Vector((0.025,) * 3)
        hi = Vector(map(max, a, b)) + Vector((0.025,) * 3)
        parts.append(box(*lo, *hi, materials.trim, "arm"))
    c, s = Vector(mirror_center), Vector(mirror_size) / 2
    parts.append(box(*(c - s), *(c + s), materials.trim, "mirror"))
    return parts


# ================== Output ==================
def _clean(o):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = o
    o.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=0.0005)
    bpy.ops.mesh.delete_loose()
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))


def _bake_colors(o, name):
    """Turns the materials into vertex colors on one material.

    A model is then one surface: drawn see-through, the surfaces of a mesh are sorted against each
    other in no fixed order, and its parts would flicker with the view. It is also one draw call.
    """
    me = o.data
    # (the booleans can leave an empty slot, unused)
    colors = [tuple(_base_color(m).default_value) if m else None for m in me.materials]
    attribute = me.color_attributes.new("Color", 'FLOAT_COLOR', 'CORNER')
    for p in me.polygons:
        for i in p.loop_indices:
            attribute.data[i].color = colors[p.material_index]
        p.material_index = 0
    me.color_attributes.active_color = attribute
    for uv in list(me.uv_layers):  # no textures
        me.uv_layers.remove(uv)
    material = _material(name, (1.0, 1.0, 1.0))  # times the vertex colors
    # Matte: sharp highlights would change with the view
    material.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.7
    material.use_backface_culling = True  # exported single-sided
    me.materials.clear()
    me.materials.append(material)


def _base_color(material):
    return material.node_tree.nodes["Principled BSDF"].inputs["Base Color"]


def _cut(source, name, x_min, x_max, origin_x):
    """The part of `source` between x_min and x_max (None: no limit), open at the cuts."""
    me = source.data.copy()
    me.name = name
    bm = bmesh.new()
    bm.from_mesh(me)
    for x, keep_ahead in ((x_min, True), (x_max, False)):
        if x is not None:
            bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=(x, 0, 0),
                                   plane_no=(1, 0, 0), clear_inner=keep_ahead, clear_outer=not keep_ahead)
    bmesh.ops.dissolve_degenerate(bm, dist=1e-4, edges=bm.edges[:])
    for v in bm.verts:
        v.co.x -= origin_x
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    o.location.x = origin_x
    return o


def _marker(name, parent, x):
    o = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(o)
    o.parent = parent
    o.location.x = x - parent.location.x
    return o


def export_length_fitted(parts, front_end, front_seam, bay, rear_seam, rear_end, path):
    """Splits the model into Front, MidBay and Rear (see README.md) and exports it as glTF binary.

    Between the seams, the model must be the same in every `bay` long section, cut at the same place
    of its pattern (e.g. at the middle of a pillar).
    """
    bays = (front_seam - rear_seam) / bay
    assert abs(bays - round(bays)) < 1e-6 and bays >= 1, "the seams must be whole bays apart"
    whole = join(parts, "Whole")
    _clean(whole)
    _bake_colors(whole, _model_name(path))
    front = _cut(whole, "Front", front_seam, None, front_seam)
    _cut(whole, "MidBay", front_seam - bay, front_seam, front_seam - bay / 2)
    rear = _cut(whole, "Rear", None, rear_seam, rear_seam)
    bpy.data.objects.remove(whole, do_unlink=True)
    _marker("FrontEnd", front, front_end)
    _marker("RearEnd", rear, rear_end)
    _export(path)


def export_model(parts, path):
    """Exports the model as one mesh, as glTF binary."""
    body = join(parts, "Body")
    _clean(body)
    _bake_colors(body, _model_name(path))
    _export(path)


def _model_name(path):
    return os.path.splitext(os.path.basename(path))[0]


def _export(path):
    for o in bpy.context.scene.objects:
        if o.type == 'MESH':
            print("%-8s %6d triangles" % (o.name, sum(len(p.vertices) - 2 for p in o.data.polygons)))
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', use_selection=True, export_yup=True,
                              export_apply=True, export_vertex_color='ACTIVE')
    print("Exported", path)
