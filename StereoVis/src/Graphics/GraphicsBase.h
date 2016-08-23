
#ifndef _GraphicsBase_H_
#define _GraphicsBase_H_

namespace GraphicsBase {

    bool initialise(void);
    void resizeWindow(int width, int height);

    void switchToOrthographic(void);
    void switchFromOrthographic(void);

    float getCameraHeight(void);
    int getScreenWidth(void);
    int getScreenHeight(void);
}

#endif

