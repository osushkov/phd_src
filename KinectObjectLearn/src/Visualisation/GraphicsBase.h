/*
 * GraphicsBase.h
 *
 *  Created on: 22/06/2009
 *      Author: osushkov
 */

#ifndef GRAPHICSBASE_H_
#define GRAPHICSBASE_H_

#include <SDL.h>

class GraphicsBase {
  public:
    static GraphicsBase& instance(void);

    void initialise(void);
    static void resizeWindow(int width, int height);

    void test(void);

  private:

    bool is_initialised;

    /* This is our SDL surface */
    SDL_Surface *surface;

    bool initialiseSDL(void);
    bool initialiseGL(void);

    void testDraw(void);

    GraphicsBase();
    ~GraphicsBase();
};


#endif /* GRAPHICSBASE_H_ */
