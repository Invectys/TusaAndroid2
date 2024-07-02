
uniform mat4 u_matrix;

attribute vec4 a_vertexPosition;
attribute vec2 a_unit_square_cords;

varying vec2 v_tile_cords_unit_square;

void main() {
    gl_Position = u_matrix * a_vertexPosition;

    v_tile_cords_unit_square = a_unit_square_cords;
}