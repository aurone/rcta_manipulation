#version 330 core // tells the compiler to use OPenGL 3's syntax

layout(location = 0) in vec3 vertexPosition_modelspace; // declares the input data

void main()
{
    //mat4 myMatrix;
    //vec4 myVector;
    //vec4 transformedVector = myMatrix * myVector;

    gl_Position.xyz = vertexPosition_modelspace;
    gl_Position.w = 1.0;
}
