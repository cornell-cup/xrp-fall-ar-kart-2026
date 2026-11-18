// Light stripes round the outer edge of a tyre's sides, so you can see the wheel rolling. Used as a
// material overlay on the kart's front wheels: it draws over the wheel's own
// materials and is see-through everywhere except the stripes.
// The wheel's axle runs along its local Z, so the stripes go round in X/Y
shader_type spatial;
render_mode blend_mix, cull_back, depth_draw_never;

// The wheel's centre and radius in its own mesh coordinates (set from code)
uniform vec3 centre = vec3(0.0, 0.46, 0.0);
uniform float radius = 0.48;
uniform vec4 mark_color : source_color = vec4(0.92, 0.94, 0.97, 1.0);
// How many stripes go round the wheel, and how wide each is (0..1 of the gap)
uniform float marks = 8.0;
uniform float mark_width = 0.28;
// Stripes only on the outer part of the wheel (the tyre), as a fraction of the radius
uniform float tyre_start = 0.72;

varying vec3 local_pos;
varying vec3 local_normal;

void vertex() {
	local_pos = VERTEX;
	local_normal = NORMAL;
}

void fragment() {
	vec2 d = local_pos.xy - centre.xy;
	float r = length(d) / radius;
	// 0 in the middle of a stripe, 1 halfway between two
	float around = atan(d.y, d.x) / TAU * marks;
	float from_mid = abs(fract(around) - 0.5) * 2.0;
	float stripe = 1.0 - smoothstep(mark_width - 0.06, mark_width, from_mid);
	float on_tyre = smoothstep(tyre_start - 0.04, tyre_start, r);
	// only on the sides of the tyre (facing along the axle), not across the tread
	float on_side = smoothstep(0.5, 0.7, abs(normalize(local_normal).z));
	ALBEDO = mark_color.rgb;
	ROUGHNESS = 0.6;
	ALPHA = stripe * on_tyre * on_side * mark_color.a;
}
