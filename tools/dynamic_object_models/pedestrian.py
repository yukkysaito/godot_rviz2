# Pedestrian: blender -b --python tools/dynamic_object_models/pedestrian.py [-- output.glb]
# Smooths the original model (pedestrian_source.glb: the body, its skeleton and the walk), whose
# faces were flat: rounded once and reduced back to about 2k triangles. The skeleton and the walk
# are kept as they are (Pedestrian.tscn and PedestrianAnimator.gd refer to them). Note: the bones
# come out in name order, and Pedestrian.tscn poses some of them by index in that order.
import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402


def apply(o, kind, **settings):
    m = o.modifiers.new(kind, kind)
    for key, value in settings.items():
        setattr(m, key, value)
    bpy.ops.object.modifier_move_to_index(modifier=m.name, index=0)  # before the armature
    bpy.ops.object.modifier_apply(modifier=m.name)


def build(path):
    c.reset_scene()
    bpy.ops.import_scene.gltf(filepath=os.path.join(c.HERE, "pedestrian_source.glb"))
    for o in [o for o in bpy.data.objects if o.type == 'MESH' and o.parent is None]:
        bpy.data.objects.remove(o, do_unlink=True)  # a bone shape made by the importer
    body = bpy.data.objects["Body.001"]
    bpy.ops.object.select_all(action='DESELECT')
    body.select_set(True)
    bpy.context.view_layer.objects.active = body
    # Its faces had their own vertices (flat): join them, then round and reduce the body
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=1e-5)
    bpy.ops.object.mode_set(mode='OBJECT')
    apply(body, 'SUBSURF', levels=1, render_levels=1)
    apply(body, 'DECIMATE', ratio=0.3)
    bpy.ops.object.shade_smooth()
    print("%-8s %6d triangles" % (body.name, sum(len(p.vertices) - 2 for p in body.data.polygons)))
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', use_selection=True, export_yup=True,
                              export_animations=True, export_skins=True)
    print("Exported", path)


if __name__ == "__main__":
    c.main(build, "Pedestrian.glb")
