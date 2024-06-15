precision highp float;


uniform sampler2D u_tile_0;
uniform sampler2D u_tile_1;
uniform sampler2D u_tile_2;
uniform sampler2D u_tile_3;

varying vec2 v_tile_cords_unit_square;
varying vec2 v_tile_cords_0;
varying vec2 v_tile_cords_1;
varying vec2 v_tile_cords_2;
varying vec2 v_tile_cords_3;



vec4 tile(sampler2D tile_tex, vec2 cords, vec4 other) {
    return mix(texture2D(tile_tex, cords), other, step(cords.x, 0.0));
}

vec4 tile0(vec4 other) {
    return tile(u_tile_0, v_tile_cords_0, other);
}

vec4 tile1(vec4 other) {
    return tile(u_tile_1, v_tile_cords_1, other);
}

vec4 tile2(vec4 other) {
    return tile(u_tile_2, v_tile_cords_2, other);
}

vec4 tile3(vec4 other) {
    return tile(u_tile_3, v_tile_cords_3, other);
}

void main() {
    //gl_FragColor = tile0(tile1(tile2(tile3(vec4(0.0, 0.0, 0.0, 1.0)))));
    //gl_FragColor = tile0(vec4(0.0, 0.0, 0.0, 1.0));
    gl_FragColor = texture2D(u_tile_0, v_tile_cords_unit_square);
}