/*
 * SceneRenderer.cpp
 *
 *  Created on: 23/06/2009
 *      Author: osushkov
 */

#include "SceneRenderer.h"
#include "GraphicsBase.h"
#include "TextureManager.h"
#include "../Util/Semaphore.h"
#include "../Util/Vector3D.h"
#include "../Util/Common.h"

#include <pthread.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <SDL.h>


SceneRenderer& SceneRenderer::instance(void){
    static SceneRenderer scene_renderer;
    return scene_renderer;
}

SceneRenderer::SceneRenderer() :
    is_initialised(false), is_rot_mouse_down(false), is_zoom_mouse_down(false),
    cur_yrot(0.0f), cur_xrot(0.0f), cur_zoom(1.0f), lock(1) {

}

SceneRenderer::~SceneRenderer(){
	shutdown();
}

void SceneRenderer::initialise(void){
    if(!is_initialised){
        done = false;
        pthread_create(&render_thread_id, NULL, &SceneRenderer::renderThread, (void *)this);
        is_initialised  = true;
    }

}

void SceneRenderer::shutdown(void){
	done = true;
	scene_rendering_sem.wait();
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

bool SceneRenderer::removeObject(unsigned id){
    bool result = false;
    lock.wait();

    std::map<unsigned, RenderObject*>::iterator it = objects.find(id);
    if(it == objects.end()){
        result = false;
    }
    else{
        objects.erase(it);
        result = true;
    }

    lock.signal();
    return result;
}

bool SceneRenderer::haveObject(unsigned id){
    bool result;

    lock.wait();
    result = objects.find(id) != objects.end();
    lock.signal();

    return result;
}

RenderObject* SceneRenderer::getObject(unsigned id){
    lock.wait();
    std::map<unsigned, RenderObject*>::iterator it = objects.find(id);
    lock.signal();

    if(it == objects.end()){
        return NULL;
    }
    else{
        return it->second;
    }
}

void* SceneRenderer::renderThread(void *thread_arg){
    SceneRenderer *scene_renderer = (SceneRenderer *)thread_arg;
    GraphicsBase::instance().initialise();

    scene_renderer->done = false;
    SDL_Event event;

    while(!scene_renderer->done){
        while (SDL_PollEvent(&event)){
            switch( event.type ){
            case SDL_MOUSEBUTTONDOWN:
                if(event.button.button == SDL_BUTTON_LEFT &&
                   !scene_renderer->is_rot_mouse_down){
                    scene_renderer->is_rot_mouse_down = true;
                    scene_renderer->mouse_click_pos.x = event.button.x;
                    scene_renderer->mouse_click_pos.y = event.button.y;
                }
                else if(event.button.button == SDL_BUTTON_RIGHT &&
                        !scene_renderer->is_zoom_mouse_down){
                    scene_renderer->is_zoom_mouse_down = true;
                    scene_renderer->mouse_click_pos.x = event.button.x;
                    scene_renderer->mouse_click_pos.y = event.button.y;
                }

                break;
            case SDL_MOUSEBUTTONUP:
                if(event.button.button == SDL_BUTTON_LEFT){
                    scene_renderer->is_rot_mouse_down = false;
                }
                else if(event.button.button == SDL_BUTTON_RIGHT){
                    scene_renderer->is_zoom_mouse_down = false;
                }
                break;

            case SDL_MOUSEMOTION:
                if(scene_renderer->is_rot_mouse_down){
                    scene_renderer->cur_yrot +=
                        (event.motion.x-scene_renderer->mouse_click_pos.x)/2.0f;
                    scene_renderer->cur_xrot +=
                        (event.motion.y-scene_renderer->mouse_click_pos.y)/2.0f;
                    scene_renderer->mouse_click_pos.x = event.motion.x;
                    scene_renderer->mouse_click_pos.y = event.motion.y;
                }
                else if(scene_renderer->is_zoom_mouse_down) {
                    scene_renderer->cur_zoom +=
                        (scene_renderer->mouse_click_pos.y - event.motion.y) / 100.0f;
                    scene_renderer->mouse_click_pos.x = event.motion.x;
                    scene_renderer->mouse_click_pos.y = event.motion.y;
                }
                break;

            case SDL_VIDEORESIZE:
                GraphicsBase::resizeWindow( event.resize.w, event.resize.h );
                break;
            case SDL_QUIT:
                scene_renderer->done = true;
            default:
                break;
            }
        }

        scene_renderer->drawScene();
    }

    SDL_Quit();
    scene_renderer->scene_rendering_sem.signal();
    pthread_exit((void *)NULL);
    return NULL;
}

void SceneRenderer::drawScene(void){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glPushMatrix();
    gluLookAt(0.0f, 175.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f);


    glRotatef(cur_xrot, 1.0f, 0.0f, 0.0f);
    glRotatef(cur_yrot, 0.0f, 1.0f, 0.0f);

    //glTranslatef(0.0, -20.0f, 35.0);
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0);

    glColor3f(1.0f, 1.0f, 1.0f);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    drawWorkspaceBox();
    //drawTabletop();

    lock.wait();
    std::map<unsigned, RenderObject*>::iterator it;
    for(it = objects.begin(); it != objects.end(); ++it){
        it->second->render();
    }
    lock.signal();

    glPopMatrix();
    SDL_GL_SwapBuffers();

    Common::sleep(10);
}


void SceneRenderer::drawWorkspaceBox(void){
    float xrange[] = {-40.0f, 40.0f};
    float yrange[] = {0.0f, 70.0f};
    float zrange[] = {0.0f, 60.0f};


    glPushMatrix();


    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glBegin(GL_LINES);
/*
    glVertex3f(xrange[0], yrange[0], zrange[0]);
    glVertex3f(xrange[0], yrange[1], zrange[0]);

    glVertex3f(xrange[1], yrange[0], zrange[0]);
    glVertex3f(xrange[1], yrange[1], zrange[0]);

    glVertex3f(xrange[0], yrange[0], zrange[0]);
    glVertex3f(xrange[1], yrange[0], zrange[0]);

    glVertex3f(xrange[0], yrange[1], zrange[0]);
    glVertex3f(xrange[1], yrange[1], zrange[0]);


    glVertex3f(xrange[0], yrange[0], zrange[1]);
    glVertex3f(xrange[0], yrange[1], zrange[1]);

    glVertex3f(xrange[1], yrange[0], zrange[1]);
    glVertex3f(xrange[1], yrange[1], zrange[1]);

    glVertex3f(xrange[0], yrange[0], zrange[1]);
    glVertex3f(xrange[1], yrange[0], zrange[1]);

    glVertex3f(xrange[0], yrange[1], zrange[1]);
    glVertex3f(xrange[1], yrange[1], zrange[1]);


    glVertex3f(xrange[0], yrange[0], zrange[0]);
    glVertex3f(xrange[0], yrange[0], zrange[1]);

    glVertex3f(xrange[0], yrange[1], zrange[0]);
    glVertex3f(xrange[0], yrange[1], zrange[1]);

    glVertex3f(xrange[1], yrange[0], zrange[0]);
    glVertex3f(xrange[1], yrange[0], zrange[1]);

    glVertex3f(xrange[1], yrange[1], zrange[0]);
    glVertex3f(xrange[1], yrange[1], zrange[1]);
*/

    const float axis_length = 15.0f;
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(axis_length, 0.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, axis_length, 0.0f); 

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, axis_length); 

    glEnd();

    glPopMatrix();
}

void SceneRenderer::drawTabletop(void){
    float height = 15.5f;
    unsigned cell_dim = 30;
    float xrange[] = {-30.0f, 30.0f};
    float yrange[] = {0.0f, 50.0f};

    glPushMatrix();

    glColor4f(0.2f, 1.0f, 0.2f, 1.0f);
    glBegin(GL_LINES);

    for(unsigned i = 0; i <= cell_dim; i++){
        float a = (float)i/(float)cell_dim;
        float b = 1.0f - a;

        glVertex3f(a*xrange[0] + b*xrange[1], yrange[0], height);
        glVertex3f(a*xrange[0] + b*xrange[1], yrange[1], height);

        glVertex3f(xrange[0], a*yrange[0] + b*yrange[1], height);
        glVertex3f(xrange[1], a*yrange[0] + b*yrange[1], height);
    }

    glEnd();

    glPopMatrix();
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
