/*
 * GraphicsBase.cpp
 *
 *  Created on: 22/06/2009
 *      Author: osushkov
 */


#include "GraphicsBase.h"

#include <iostream>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL/SDL.h>

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480


GraphicsBase& GraphicsBase::instance(void){
    static GraphicsBase graphics_base;
    return graphics_base;
}

GraphicsBase::GraphicsBase(){
    is_initialised = false;
}

GraphicsBase::~GraphicsBase(){
}

void GraphicsBase::initialise(void){
    if(!is_initialised){
        initialiseSDL();
        initialiseGL();
        resizeWindow(SCREEN_WIDTH, SCREEN_HEIGHT);
        is_initialised = true;
    }
}

void GraphicsBase::resizeWindow(int width, int height){
    if(height == 0){
        height = 1;
    }

    float ratio = (float)width/(float)height;

    glViewport( 0, 0, width, height);

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();

    gluPerspective( 45.0f, ratio, 0.1f, 1000.0f );

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );
}

void GraphicsBase::test(void){
    bool done = false;
    SDL_Event event;

    while(!done){
        while (SDL_PollEvent(&event)){
            switch( event.type ){
            case SDL_VIDEORESIZE:
                resizeWindow( event.resize.w, event.resize.h );
                break;
            case SDL_QUIT:
                done = true;
            default:
                break;
            }
        }

        testDraw();
    }

    SDL_Quit();
}

bool GraphicsBase::initialiseSDL(void){
    int videoFlags;
    const SDL_VideoInfo *videoInfo;

    if(SDL_Init(SDL_INIT_VIDEO) < 0){
        std::cerr << "Video initialisation failed: " << SDL_GetError() << std::endl;
        return false;
    }

    videoInfo = SDL_GetVideoInfo();
    if(!videoInfo){
        std::cerr << "Video query failed: " << SDL_GetError() << std::endl;
        return false;
    }

    videoFlags  = SDL_OPENGL;
    videoFlags |= SDL_GL_DOUBLEBUFFER;
    videoFlags |= SDL_HWPALETTE;
    videoFlags |= SDL_RESIZABLE;

    if(videoInfo->hw_available){
        videoFlags |= SDL_HWSURFACE;
    }
    else{
        videoFlags |= SDL_SWSURFACE;
    }

    if(videoInfo->blit_hw){
        videoFlags |= SDL_HWACCEL;
    }

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    /* get a SDL surface */
    surface = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, 24, videoFlags );

    if (!surface){
        std::cerr << "Video mode set failed: " << SDL_GetError() << std::endl;
        return false;
    }

    return true;
}

bool GraphicsBase::initialiseGL(void){
    glShadeModel( GL_SMOOTH );
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
    glClearDepth( 1.0f );
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LEQUAL );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );

    glPointSize(2.0f);
    return true;
}

void GraphicsBase::testDraw(void){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    /* Move Left 1.5 Units And Into The Screen 6.0 */
    glLoadIdentity();
    glTranslatef( -1.5f, 0.0f, -6.0f );

    glBegin( GL_TRIANGLES );            /* Drawing Using Triangles */
      glVertex3f(  0.0f,  1.0f, 0.0f ); /* Top */
      glVertex3f( -1.0f, -1.0f, 0.0f ); /* Bottom Left */
      glVertex3f(  1.0f, -1.0f, 0.0f ); /* Bottom Right */
    glEnd( );                           /* Finished Drawing The Triangle */

    /* Move Right 3 Units */
    glTranslatef( 3.0f, 0.0f, 0.0f );

    glBegin( GL_QUADS );                /* Draw A Quad */
      glVertex3f( -1.0f,  1.0f, 0.0f ); /* Top Left */
      glVertex3f(  1.0f,  1.0f, 0.0f ); /* Top Right */
      glVertex3f(  1.0f, -1.0f, 0.0f ); /* Bottom Right */
      glVertex3f( -1.0f, -1.0f, 0.0f ); /* Bottom Left */
    glEnd( );                           /* Done Drawing The Quad */

    /* Draw it to the screen */
    SDL_GL_SwapBuffers( );

}
