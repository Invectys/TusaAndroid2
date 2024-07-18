attribute vec4 vertexPosition;
uniform mat4 u_matrix;

uniform float u_pointSize;

void main() {
    gl_PointSize = u_pointSize;
    gl_Position = u_matrix * vertexPosition;
}