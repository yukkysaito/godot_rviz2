// Depth only: the twin of a see-through mesh (see SeeThroughDepth.gd: a truck or a bus, the ego
// vehicle) draws it before the mesh, whose color then shows only where it is nearest, so that its
// parts behind (the box behind the cab, the far wheels) do not show through it. Culls like the
// models' materials.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_always, cull_back, shadows_disabled;

void fragment() {
	ALBEDO = vec3(0.0);
	ALPHA = 0.0;
}
