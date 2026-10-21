# Dynamic object models

Blender scripts that generate the vehicle models in `godot-project/3DViewer/DynamicObject/`
(trailers are drawn as trucks). They are seen from afar, many at once: low poly (1 to 2k triangles),
no textures, and one material with the colors as vertex colors. One surface per mesh also keeps
the see-through models from flickering, as the surfaces of a mesh are drawn in no fixed order.

```bash
blender -b --python tools/dynamic_object_models/car.py         # -> Car.glb
blender -b --python tools/dynamic_object_models/bus.py         # -> Bus.glb
blender -b --python tools/dynamic_object_models/truck.py       # -> Truck.glb
blender -b --python tools/dynamic_object_models/bicycle.py     # -> Bicycle.glb
blender -b --python tools/dynamic_object_models/motorcycle.py  # -> Motorcycle.glb
```

Pass `-- <path>.glb` to write somewhere else. Tested with Blender 5.2.

## Fitting the measured length

The car, the bicycle and the motorcycle keep their size. The length of buses and trucks varies a
lot, so `LengthFittedModel.gd` fits them to the length of the detected object. The front and the
rear keep their shape, and the section between them is repeated as many times as fits, each copy
stretched a little to fill the gap. For that, a model is split into these nodes:

| Node               | What it is                                                              |
| ------------------ | ----------------------------------------------------------------------- |
| `Front`            | The front end, with its origin at the front seam                        |
| `Front/FrontEnd`   | Empty at the very front of the body                                     |
| `MidBay`           | One section between the seams (e.g. a window bay), origin at its center |
| `Rear`             | The rear end, with its origin at the rear seam                          |
| `Rear/RearEnd`     | Empty at the very rear of the body                                      |

`common.export_length_fitted()` makes them. Between the seams, the model must be the same in
every section, and the seams should cut through something plain (the middle of a pillar or a rib)
so that a stretched section does not show.

Coordinates: +X forward, +Y left, +Z up, in meters, with the origin on the ground at the center
of the body.
