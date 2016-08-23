/*
 * SceneRenderer.cpp
 *
 *  Created on: 23/06/2009
 *      Author: osushkov
 */

#include "SceneRenderer.h"
#include "RenderObject.h"
#include "GraphicsBase.h"
#include "TextureManager.h"
#include "../Util/Semaphore.h"
#include "../Util/Vector3D.h"
#include "../Util/Geometry.h"
#include <cv.h>

#include <pthread.h>

#ifdef _WIN32
#include <windows.h>
#include <SDL.h>
#else
#include <SDL/SDL.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>


SceneRenderer& SceneRenderer::instance(void){
    static SceneRenderer scene_renderer;
    return scene_renderer;
}

SceneRenderer::SceneRenderer() :
    is_initialised(false), is_rot_mouse_down(false), is_zoom_mouse_down(false),
    cur_yrot(0.0f), cur_xrot(0.0f), cur_zoom(1.0f), lock(1),
    signal_sem0(1), signal_sem1(0)  {

}

SceneRenderer::~SceneRenderer(){
	shutdown();
}

void SceneRenderer::initialise(void){
    if(!is_initialised){
        GraphicsBase::instance().initialise();
        is_initialised  = true;
    }
}

void SceneRenderer::shutdown(void){
	done = true;
}

std::pair<IplImage*,IplImage*> SceneRenderer::renderObject(RenderObject *obj, Transform transform){
    return std::pair<IplImage*,IplImage*>(renderCamera(Vector3D(-6.0f, 0.0f, 0.0f), obj, transform), 
                                          renderCamera(Vector3D(6.0f, 0.0f, 0.0f), obj, transform));
}

bool SceneRenderer::addObject(RenderObject *new_object){
    lock.wait();
    if(objects.find(new_object->getId()) != objects.end()){
        std::cout << "RenderObject exists" << std::endl;
        lock.signal();
        return false;
    }

    objects.insert(std::pair<unsigned, RenderObject*>(new_object->getId(), new_object));
    lock.signal();
    return true;
}


void SceneRenderer::clearObjects(void){
    lock.wait();
    objects.clear();
    lock.signal();
}

IplImage* SceneRenderer::renderCamera(Vector3D camera_pos, RenderObject *obj, Transform transform){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();

    gluLookAt(camera_pos.x, camera_pos.y, camera_pos.z, camera_pos.x, camera_pos.y, camera_pos.z - 1.0f, 0.0f, 1.0f, 0.0f);
    //glTranslatef(transform.shift.x, transform.shift.y, transform.shift.z);
  
    float adj[16];
    Geometry::transformToOpenGLMatrix(transform, adj);
    glMultMatrixf(adj);

    obj->render();

    IplImage *result = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
    glReadPixels(0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, result->imageData);

    for(int y = 0; y < result->height; y++){
        for(int x = 0; x < result->width; x++){
            double r, g, b;
            Common::getPixel(result, x, y, r, g, b);
            Common::setPixel(result, x, y, b, g, r);
        }
    }

    return result;
}

bool SceneRenderer::renderScene(void){
    SDL_Event event;

	while (SDL_PollEvent(&event)){
		switch( event.type ){
		case SDL_MOUSEBUTTONDOWN:
			if(event.button.button == SDL_BUTTON_LEFT && !is_rot_mouse_down){
					is_rot_mouse_down = true;
					mouse_click_pos.x = event.button.x;
					mouse_click_pos.y = event.button.y;
			}
			else if(event.button.button == SDL_BUTTON_RIGHT && !is_zoom_mouse_down){
					is_zoom_mouse_down = true;
					mouse_click_pos.x = event.button.x;
					mouse_click_pos.y = event.button.y;
			}

			break;
		case SDL_MOUSEBUTTONUP:
			if(event.button.button == SDL_BUTTON_LEFT){
				is_rot_mouse_down = false;
			}
			else if(event.button.button == SDL_BUTTON_RIGHT){
				is_zoom_mouse_down = false;
			}
			break;

		case SDL_MOUSEMOTION:
			if(is_rot_mouse_down){
				cur_yrot +=	(event.motion.x-mouse_click_pos.x)/2.0f;
				cur_xrot += (event.motion.y-mouse_click_pos.y)/2.0f;
				mouse_click_pos.x = event.motion.x;
				mouse_click_pos.y = event.motion.y;
			}
			else if(is_zoom_mouse_down) {
				cur_zoom += (mouse_click_pos.y - event.motion.y) / 100.0f;
				mouse_click_pos.x = event.motion.x;
				mouse_click_pos.y = event.motion.y;
			}
			break;

		case SDL_VIDEORESIZE:
			GraphicsBase::resizeWindow( event.resize.w, event.resize.h );
			break;
		case SDL_QUIT:
			return false;
		default:
			break;
		}
    }

	drawScene();
    return true;
}

void SceneRenderer::drawScene(void){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();

    gluLookAt(0.0f, 0.0f, 100.0f/cur_zoom, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
    //gluLookAt(-6.0f, 0.0f, 0.0f, -6.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f);
    

    glRotatef(cur_xrot, 1.0f, 0.0f, 0.0f);
    glRotatef(cur_yrot, 0.0f, 1.0f, 0.0f);

    //glTranslatef(0.0, -10.0f, 35.0);
    //glRotatef(-90.0f, 1.0f, 0.0f, 0.0);

    glColor3f(1.0f, 1.0f, 1.0f);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    lock.wait();
    std::map<unsigned, RenderObject*>::iterator it;
    for(it = objects.begin(); it != objects.end(); ++it){
        it->second->render();
    }
    lock.signal();

    SDL_GL_SwapBuffers( );
}

/*
unsigned char* SceneRenderer::getTextureData(const std::vector<unsigned char> cam_buffer,
                                             unsigned width, unsigned height){
    unsigned rwidth = nextPowerOf2(width);
    unsigned rheight = nextPowerOf2(height);
    unsigned char *result = new unsigned char[rwidth*rheight*3];

    for(unsigned y = 0; y < rheight; y++){
        for(unsigned x = 0; x < rwidth; x++){
            float fx = (float)x/(float)rwidth;
            float fy = (float)y/(float)rheight;
            unsigned ix = fx*width;
            unsigned iy = fy*height;

            result[(x + y*rwidth)*3] = cam_buffer[(ix + iy*width)*3+2];
            result[(x + y*rwidth)*3+1] = cam_buffer[(ix + iy*width)*3+1];
            result[(x + y*rwidth)*3+2] = cam_buffer[(ix + iy*width)*3];
        }
    }

    return result;
}

unsigned SceneRenderer::nextPowerOf2(unsigned var){
    unsigned result = 1;

    while(result < var){
        result = result << 1;
    }

    return result;
}

Vector3D SceneRenderer::getPixelColor(unsigned x, unsigned y){
    Vector3D result;
    result.x = cam_buffer[(x + y*cam_width)*3+2]/255.0f;
    result.y = cam_buffer[(x + y*cam_width)*3+1]/255.0f;
    result.z = cam_buffer[(x + y*cam_width)*3]/255.0f;
    return result;
}*/
