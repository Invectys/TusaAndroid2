precision highp float;

uniform sampler2D u_tile_0;

uniform float xTopLeft;
uniform float yTopLeft;
uniform float xBottomRight;
uniform float yBottomRight;

uniform float u_zoom;
uniform float u_shiftX;
uniform float u_shiftY;

uniform float u_startX;
uniform float u_endX;

uniform float u_startY;
uniform float u_endY;

varying vec2 v_tile_cords_unit_square;

void main() {
    float shifted_planet_x = v_tile_cords_unit_square.x + u_shiftX;

    float xStartBorder = u_startX;
    float xEndBorder = u_endX;
    float yStartBorder = u_startY;
    float yEndBorder = u_endY;

    float xTile = (shifted_planet_x - xStartBorder) / (xEndBorder - xStartBorder);
    float yTile = (v_tile_cords_unit_square.y - yStartBorder) / (yEndBorder - yStartBorder);

    gl_FragColor = texture2D(u_tile_0, vec2(xTile, yTile));
}