
#include "GraphicsBase.h"
#include "../Settings.h"

#include <SDL/SDL.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <iostream>

#define SCREEN_BPP 24


int cur_window_width = 0, cur_window_height = 0;

void initGL(void);


bool GraphicsBase::initialise(void){
    if(!Settings::instance().getIntValue("graphics", "render")){
        return true;
    }

    int videoFlags;
    const SDL_VideoInfo *videoInfo;
    SDL_Surface *surface;

    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_AUDIO) < 0){
        std::cerr << "Video initialization failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    videoInfo = SDL_GetVideoInfo();

    if(!videoInfo){
	    std::cerr << "Video query failed: " << SDL_GetError() << std::endl;
	    SDL_Quit();
        return false;
    }

    videoFlags  = SDL_OPENGL;          // Enable OpenGL in SDL
    videoFlags |= SDL_GL_DOUBLEBUFFER; // Enable double buffering
    videoFlags |= SDL_HWPALETTE;       // Store the palette in hardware
    //videoFlags |= SDL_RESIZABLE;       // Enable window resizing

    // This checks to see if surfaces can be stored in memory
    if(videoInfo->hw_available){
        videoFlags |= SDL_HWSURFACE;
    }
    else{
	    videoFlags |= SDL_SWSURFACE;
    }    

    // This checks if hardware blits can be done
    if(videoInfo->blit_hw){
	    videoFlags |= SDL_HWACCEL;
    }

    // Sets up OpenGL double buffering 
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_WM_SetCaption("StereoVis", NULL);

    // get a SDL surface
    int screen_width = Settings::instance().getIntValue("graphics", "screen_width");
    int screen_height = Settings::instance().getIntValue("graphics", "screen_height");

    surface = SDL_SetVideoMode(screen_width, screen_height, SCREEN_BPP, videoFlags);

    // Verify there is a surface
    if(!surface){
	    std::cerr << "Video mode set failed: " << SDL_GetError() << std::endl;
	    SDL_Quit();
    }

    // initialize OpenGL
    initGL();

    // resize the initial window
    resizeWindow(screen_width, screen_height);
    
    return true;
}

void GraphicsBase::resizeWindow(int width, int height){
    float ratio;

    if(height == 0){
	    height = 1;
    }

    cur_window_width = width;
    cur_window_height = height;

    ratio = (float)width/(float)height;

    // Setup our viewport.
    glViewport(0, 0, width, height);

    // change to the projection matrix and set our viewing volume.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Set our perspective
    gluPerspective(60.0f, ratio, 0.1f, 500.0f );

    // Make sure we're chaning the model view and not the projection
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GraphicsBase::switchToOrthographic(void){
    // switch to projection mode
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    
    float ratio = (float)cur_window_height/(float)cur_window_width;

    // set a 2D orthographic projection
    gluOrtho2D(0, 1.0f, 0, ratio);
    
    glMatrixMode(GL_MODELVIEW);
}


void GraphicsBase::switchFromOrthographic(void){
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}


float GraphicsBase::getCameraHeight(void){
    return Settings::instance().getFloatValue("graphics", "camera_height");
}

int GraphicsBase::getScreenWidth(void){
    return cur_window_width;
}

int GraphicsBase::getScreenHeight(void){
    return cur_window_height;
}

void initGL(void){
    glShadeModel(GL_SMOOTH);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);
    glDisable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glDepthMask(GL_TRUE);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    glPointSize(1.0f);
    glLineWidth(1.0f);
}

