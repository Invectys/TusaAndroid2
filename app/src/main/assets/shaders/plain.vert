attribute vec4 vertexPosition;
uniform mat4 u_matrix;

void main() {
    gl_Position = u_matrix * vertexPosition;
}