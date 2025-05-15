#version 120

// Draws a circle with the packed depth value 

// includes
vec4 packDepth( );
void circleImpl( );

void main()
{
  circleImpl();

  gl_FragColor = packDepth();
}
