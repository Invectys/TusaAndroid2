

uniform mat4 u_matrix;

attribute vec4 a_vertexPosition;
attribute vec2 a_tile_cords_0;
attribute vec2 a_tile_cords_1;
attribute vec2 a_tile_cords_2;
attribute vec2 a_tile_cords_3;
attribute vec2 a_unit_square_cords;

varying vec2 v_tile_cords_unit_square;
varying vec2 v_tile_cords_0;
varying vec2 v_tile_cords_1;
varying vec2 v_tile_cords_2;
varying vec2 v_tile_cords_3;

void main() {
    gl_Position = u_matrix * a_vertexPosition;

    v_tile_cords_unit_square = a_unit_square_cords;
    v_tile_cords_0 = a_tile_cords_0;
    v_tile_cords_1 = a_tile_cords_1;
    v_tile_cords_2 = a_tile_cords_2;
    v_tile_cords_3 = a_tile_cords_3;
}