
#include "TextureManager.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>
#include <cassert>


TextureManager& TextureManager::instance(void){
    static TextureManager texture_manager;
    return texture_manager;
}


unsigned TextureManager::loadTexture(unsigned char *data, unsigned width, unsigned height){
    unsigned new_texture;

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &new_texture);
    glBindTexture(GL_TEXTURE_2D, new_texture);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);


    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB,  width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

    loaded_textures.push_back(new_texture);
    //texture_map[name] = new_texture;

    return new_texture;
}


void TextureManager::deleteTexture(unsigned tex_id){
    glDeleteTextures(1, &tex_id);
}


void TextureManager::bindTexture(unsigned tex_id){
    glBindTexture(GL_TEXTURE_2D, tex_id);
}


TextureManager::TextureManager(){
}

TextureManager::~TextureManager(){
}

