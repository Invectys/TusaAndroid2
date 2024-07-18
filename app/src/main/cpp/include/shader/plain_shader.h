//
// Created by Artem on 17.12.2023.
//

#ifndef TUSA_PLAIN_SHADER_H
#define TUSA_PLAIN_SHADER_H

#include "shader.h"
#include <GLES2/gl2.h>

class PlainShader : public Shader {
public:
    PlainShader(AAssetManager* assetManager, const char* vertexShaderName, const char* fragmentShaderName);

    GLint getPosLocation() const {return a_pos;}
    GLint getColorLocation() const {return u_color;}
    GLint getPointSizeLocation() const {return u_pointSize;}

private:
    GLint a_pos;
    GLint u_color;
    GLint u_pointSize;
};


#endif //TUSA_PLAIN_SHADER_H
