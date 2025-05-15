#version 120

// Draws a circle in the fragment color, multiplying a with the alpha param

uniform vec4 highlight;
uniform float alpha;

void circleImpl();

void main()
{
  circleImpl();
  vec3 col = gl_Color.xyz + gl_Color.xyz * highlight.xyz;
  gl_FragColor = vec4(col, alpha * gl_Color.a);
}
