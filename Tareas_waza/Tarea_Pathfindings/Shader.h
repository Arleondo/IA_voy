//
// Created by Thegamerpro987654321 on 15/03/2026.
//
#ifndef SHADER_H
#define SHADER_H

#pragma once

#include <glad/glad.h>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

class Shader
{
public:

    unsigned int ID;

    Shader(const char* vertexPath,const char* fragmentPath)
    {
        string vertexCode;
        string fragmentCode;

        ifstream vShaderFile(vertexPath);
        ifstream fShaderFile(fragmentPath);

        stringstream vStream,fStream;

        vStream << vShaderFile.rdbuf();
        fStream << fShaderFile.rdbuf();

        vertexCode = vStream.str();
        fragmentCode = fStream.str();

        const char* vShaderCode = vertexCode.c_str();
        const char* fShaderCode = fragmentCode.c_str();

        unsigned int vertex,fragment;

        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex,1,&vShaderCode,nullptr);
        glCompileShader(vertex);

        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment,1,&fShaderCode,nullptr);
        glCompileShader(fragment);

        ID = glCreateProgram();

        glAttachShader(ID,vertex);
        glAttachShader(ID,fragment);

        glLinkProgram(ID);

        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    void use() const
    {
        glUseProgram(ID);
    }

    void setFloat(const std::string& name,float value) const
    {
        glUniform1f(glGetUniformLocation(ID,name.c_str()),value);
    }

    void setVec3(const char* name,float x,float y,float z) const
    {
        GLint loc = glGetUniformLocation(ID,name);
        glUniform3f(loc,x,y,z);
    }

};

#endif //SHADER_H
