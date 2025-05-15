#version 120

varying vec3 position;
varying vec3 normal;
varying float sqr_radius;
varying float billboard_half_side;

void circleImpl()
{
  vec2 billboard_d = (gl_PointCoord.st - vec2(0.5f)) * vec2(2.0f,-2.0f) * billboard_half_side;
  vec3 billboard_pos = position;
  billboard_pos.xy += billboard_d;
  vec3 l = normalize(billboard_pos);
  if (abs(dot(l,normal)) < 0.005f)
    discard;

  vec3 cpos = (dot(position, normal) / dot(l, normal)) * l;

  vec3 diff = cpos - position;
  if (dot(diff, diff) > sqr_radius)
    discard;
}
