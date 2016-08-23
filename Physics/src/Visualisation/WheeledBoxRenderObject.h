
#ifndef _WHEELED_BOX_RENDER_OBJECT_H_
#define _WHEELED_BOX_RENDER_OBJECT_H_

#include "RenderObject.h"
#include "../Util/Vector3D.h"
#include "../SuperQuadric.h"

#include <GL/glu.h>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif


class WheeledBoxRenderObject : public RenderObject {
  public:
    WheeledBoxRenderObject(Vector3D box_size, std::vector<Vector3D> stub_sizes, 
                           std::vector<std::pair<float,float> > wheel_sizes);

    ~WheeledBoxRenderObject();

    void setUseTexture(bool var);
    void setColor(Vector3D var);

    void render(void);

    void setBoxTransform(Transform transform);
    void setStubTransforms(std::vector<Transform> transforms);
    void setWheelTransforms(std::vector<Transform> transforms);

  private:
    Vector3D box_size;
    std::vector<Vector3D> stub_sizes;
    std::vector<std::pair<float,float> > wheel_sizes; // width and radius

    Transform box_transform;
    std::vector<Transform> stub_transforms;
    std::vector<Transform> wheel_transforms;

    bool use_texture;
    Vector3D color;

    SuperQuadric box_sq;
    std::vector<SuperQuadric> stub_sqs;
    std::vector<SuperQuadric> wheel_sqs;

    void renderBox(void);
    void renderStubs(void);
    void renderWheels(void);

    void renderSuperQuadric(SuperQuadric sq);
    void applyTransform(Transform transform);
};


#endif
