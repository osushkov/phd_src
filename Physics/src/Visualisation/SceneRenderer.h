/*
 * SceneRenderer.h
 *
 *  Created on: 23/06/2009
 *      Author: osushkov
 */

#ifndef SCENERENDERER_H_
#define SCENERENDERER_H_

#include <list>
#include <vector>
#include <map>
#include <pthread.h>

#include "../Features/StereoFeature.h"
#include "../Util/Semaphore.h"
#include "../Util/Vector2D.h"
#include "../Util/Vector3D.h"
#include "RenderObject.h"

enum SceneFeatureType {
    BACKGROUND_SF,
    EDGE_SF,
    ARM_SF,
    OBJECT_SF
};

struct SceneFeature {
    Vector2D left_pos, right_pos;
    Vector3D position;
    Vector3D cam_pos;
    SceneFeatureType type;
};



class SceneRenderer {
  public:
    static SceneRenderer& instance(void);

    void initialise(bool independent=true);
    void shutdown(void);

    bool addObject(RenderObject *new_object);

    void clearObjects(void);

    bool removeObject(unsigned id);
    bool haveObject(unsigned id);

    RenderObject* getObject(unsigned id);
    void signal(void); // only use this is renderer is NOT independent

  private:
    bool is_independent;
    bool is_initialised;
    bool done;

    bool is_rot_mouse_down, is_zoom_mouse_down;
    Vector2D mouse_click_pos;
    float cur_yrot, cur_xrot, cur_zoom;

    std::map<unsigned, RenderObject*> objects;

    Util::Semaphore lock, scene_rendering_sem;
    pthread_t render_thread_id;

    Util::Semaphore signal_sem0, signal_sem1;


    SceneRenderer();
    ~SceneRenderer();

    static void* renderThread(void *thread_arg);
    void drawScene(void);
    void drawWorkspaceBox(void);
    void drawFloor(float height, unsigned cell_dim);

    /*
    unsigned char* getTextureData(const std::vector<unsigned char> cam_buffer,
                                  unsigned width, unsigned height);

    unsigned nextPowerOf2(unsigned var);
    Vector3D getPixelColor(unsigned x, unsigned y);
    */

};

#endif /* SCENERENDERER_H_ */
