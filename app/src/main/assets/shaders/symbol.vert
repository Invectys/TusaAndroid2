
attribute vec4 vertexPosition;
attribute vec2 vertexTextureCord;

varying vec2 textureCord;

uniform mat4 u_matrix;

void main() {
    gl_Position = u_matrix * vertexPosition;
    textureCord = vertexTextureCord;
}