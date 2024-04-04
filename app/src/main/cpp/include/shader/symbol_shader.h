//
// Created by Artem on 30.03.2024.
//

#ifndef TUSA_ANDROID_SYMBOL_SHADER_H
#define TUSA_ANDROID_SYMBOL_SHADER_H

#include "shader.h"
#include <GLES2/gl2.h>

class SymbolShader: public Shader {
public:
    SymbolShader(AAssetManager* assetManager, const char* vertexShaderName, const char* fragmentShaderName);

    GLint getPosLocation() const {return a_pos;}
    GLint getColorLocation() const {return u_color;}
    GLint getTextureLocation() const {return u_text;}
    GLint getTextureCord() const {return a_textureCord;}

private:
    GLint a_textureCord;
    GLint a_pos;
    GLint u_color;
    GLint u_text;
};


#endif //TUSA_ANDROID_SYMBOL_SHADER_H
