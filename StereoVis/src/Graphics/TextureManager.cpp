
#include "TextureManager.h"
#include "../Settings.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif


#include <SDL/SDL_image.h>
#include <iostream>
#include <cassert>


TextureManager& TextureManager::instance(void){
    static TextureManager texture_manager;
    return texture_manager;
}


unsigned TextureManager::loadTexture(std::string filename){
    if(!Settings::instance().getIntValue("graphics", "render")){
        return 0;
    }

    // first check to see if we already have this texture on hand.
    std::map<std::string, unsigned>::iterator it = texture_map.find(filename);
    if(it != texture_map.end()){
        return it->second;
    }

    std::cout << "loading texture" << std::endl;
    SDL_Surface *img_def = IMG_Load(filename.c_str());
    if(!img_def){
        std::cerr << "Invalid texture file: " << filename << std::endl;  
        return 0;
    }

    if(img_def->format->BytesPerPixel != 3 && img_def->format->BytesPerPixel != 4){
        std::cerr << "Invalid texture file format: " << filename << std::endl;  
        return 0;
    }

    unsigned char* data = (unsigned char *)(img_def->pixels);
    unsigned new_texture;
    
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &new_texture);
    glBindTexture(GL_TEXTURE_2D, new_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

   
    if(img_def->format->BytesPerPixel == 3){
        //gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB,  img_def->w, img_def->h, GL_RGB, GL_UNSIGNED_BYTE, data);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_def->w, img_def->h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    }
    else if(img_def->format->BytesPerPixel == 4){
        //gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA,  img_def->w, img_def->h, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img_def->w, img_def->h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    }
    else{ 
        assert(false); 
    }
    
    loaded_textures.push_back(new_texture);
    texture_map[filename] = new_texture;
    
    return new_texture;
}

unsigned TextureManager::loadTexture(unsigned char *data, unsigned width, unsigned height){
    unsigned new_texture;
    
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &new_texture);
    glBindTexture(GL_TEXTURE_2D, new_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

   
    //gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB,  width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    
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

