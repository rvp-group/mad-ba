#version 120

// Draws a circle in the fragment color

varying vec3 normal;

//includes:
void circleImpl();

void main()
{
  circleImpl();
  gl_FragColor = gl_Color;
}
