#version 120

uniform mat4 worldviewproj_matrix;
uniform mat4 worldview_matrix;
uniform mat4 inverse_transpose_worldview_matrix;
uniform mat4 projection_matrix;
uniform vec4 size;

uniform float viewport_width;
uniform float viewport_height;

varying vec3 normal;
varying vec3 position;
varying float sqr_radius;
varying float billboard_half_side;

#ifdef WITH_DEPTH
  //include:
  void passDepth( vec4 pos );
#endif

void main()
{
  gl_Position = worldviewproj_matrix * gl_Vertex;
  vec3 proj3 = gl_Position.xyz / gl_Position.w;
  normal = mat3(inverse_transpose_worldview_matrix) * gl_Normal;
  vec4 pos4 = worldview_matrix * gl_Vertex;
  vec3 pos3 = pos4.xyz / pos4.w;
  position = pos3;

  // radius
  float radius = gl_SecondaryColor.x * size.x;
  vec3 pos3_dx = pos3;
  pos3_dx.x += radius;
  vec4 proj4_dx = projection_matrix * vec4(pos3_dx, 1.0f);
  vec3 proj3_dx = proj4_dx.xyz / proj4_dx.w;
  float dxmx = length((proj3_dx.xy - proj3.xy) * vec2(viewport_width, viewport_height));
  gl_PointSize = dxmx * 1.2f; // 1.2 > 1/cos(FOV)
  billboard_half_side = radius * 1.2f;
  sqr_radius = radius * radius;

  gl_FrontColor = gl_Color;

#ifdef WITH_DEPTH
  passDepth( gl_Vertex );
#endif
}
