#version 120

// Draws a circle in the pick color 

//includes:
void circleImpl();

uniform vec4 pick_color;

void main()
{
  circleImpl();
  gl_FragColor = pick_color;
}
