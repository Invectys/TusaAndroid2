//
// Created by Artem on 17.12.2023.
//

#ifndef TUSA_SHADER_H
#define TUSA_SHADER_H

#include <cstdint>
#include <array>
#include "util/matrices.h"
#include <GLES2/gl2.h>
#include <android/asset_manager.h>

class Shader {
public:
    Shader(AAssetManager* assetManager, const char* vertexShaderName, const char* fragmentShaderName);
    ~Shader();
    bool valid;
    GLuint program;

    GLuint getMatrixLocation() const {return u_matrix;}

private:
    bool compileShader(GLuint *shader, GLenum shaderType, const char *source, GLint length);

protected:
    GLuint u_matrix = -1;
};



#endif //TUSA_SHADER_H
