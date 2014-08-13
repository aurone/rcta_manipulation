#version 330 core // tells the compiler to use OPenGL 3's syntax

layout(location = 0) in vec3 vertexPosition_modelspace; // declares the input data
uniform mat4 MVP;

void main()
{
	vec4 v = vec4(vertexPosition_modelspace, 1);
	gl_Position = MVP * v;

//    gl_Position.xyz = vertexPosition_modelspace;
//    gl_Position.w = 1.0;
}
