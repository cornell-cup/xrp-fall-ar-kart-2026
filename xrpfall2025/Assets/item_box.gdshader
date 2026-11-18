// The item box, styled after the original mystery box: a rainbow gradient from
// pink and purple on top through blue to teal and green at the bottom, a soft
// blocky pattern, light edges and a white "?" with a blue outline on every face.
// All one material, so the box is a single draw call.
// BoxMesh lays its six faces out in a 3x2 grid of UVs, so each face's own
// 0..1 coordinates are fract(UV * (3, 2))
shader_type spatial;

uniform vec4 color_top : source_color = vec4(0.93, 0.45, 0.85, 1.0);
uniform vec4 color_upper : source_color = vec4(0.55, 0.30, 0.95, 1.0);
uniform vec4 color_middle : source_color = vec4(0.15, 0.35, 0.98, 1.0);
uniform vec4 color_lower : source_color = vec4(0.10, 0.75, 0.90, 1.0);
uniform vec4 color_bottom : source_color = vec4(0.40, 0.82, 0.30, 1.0);
uniform vec4 edge_color : source_color = vec4(0.98, 0.85, 1.0, 1.0);
uniform vec4 outline_color : source_color = vec4(0.10, 0.22, 0.75, 1.0);
// The "?" (white with a dark outline, on transparent)
uniform sampler2D question : source_color, filter_linear_mipmap, repeat_disable;
// How much of a face the "?" covers, top to bottom
uniform float question_size = 0.62;
// Frame width, as a fraction of a face
uniform float edge_width = 0.07;
// Squares per face side in the blocky pattern, and how strong it is
uniform float blocks = 4.0;
uniform float block_strength = 0.12;
// How much it glows on its own, so it stays bright in shadow
uniform float glow = 0.5;

varying float height;
varying float is_top;

void vertex() {
	// 0 at the bottom of the box, 1 at the top, whichever face this is
	height = clamp(VERTEX.y / 1.85 + 0.5, 0.0, 1.0);
	is_top = step(0.5, NORMAL.y);
}

vec3 rainbow(float h) {
	if (h > 0.75) return mix(color_upper.rgb, color_top.rgb, (h - 0.75) / 0.25);
	if (h > 0.5) return mix(color_middle.rgb, color_upper.rgb, (h - 0.5) / 0.25);
	if (h > 0.25) return mix(color_lower.rgb, color_middle.rgb, (h - 0.25) / 0.25);
	return mix(color_bottom.rgb, color_lower.rgb, h / 0.25);
}

void fragment() {
	vec2 f = fract(UV * vec2(3.0, 2.0));
	// the top face's UVs run the other way round, which would turn its "?" upside down
	if (is_top > 0.5) f = 1.0 - f;
	vec3 color = rainbow(height);

	// soft checker of lighter and darker squares
	vec2 cell = floor(f * blocks);
	float checker = mod(cell.x + cell.y, 2.0) * 2.0 - 1.0;
	color *= 1.0 + checker * block_strength;

	float to_edge = min(min(f.x, 1.0 - f.x), min(f.y, 1.0 - f.y));
	float edge = 1.0 - smoothstep(edge_width - 0.03, edge_width, to_edge);
	color = mix(color, edge_color.rgb, edge);

	// the glyph fills 170 of the texture's 256 pixels top to bottom; its fill is
	// white and its outline dark, which is recoloured to outline_color
	vec2 q_uv = (f - 0.5) * (170.0 / 256.0) / question_size + 0.5;
	vec4 q = texture(question, q_uv);
	float fill = smoothstep(0.35, 0.75, dot(q.rgb, vec3(0.333)));
	vec3 glyph = mix(outline_color.rgb, vec3(1.0), fill);
	color = mix(color, glyph, q.a);

	ALBEDO = color * (1.0 - glow);
	EMISSION = color * glow;
	ROUGHNESS = 0.25;
	SPECULAR = 0.6;
}
