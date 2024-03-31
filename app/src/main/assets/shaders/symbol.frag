
uniform sampler2D u_text;
uniform vec3 u_color;

varying vec2 textureCord;

void main() {
    gl_FragColor = texture2D(u_text, textureCord);
}