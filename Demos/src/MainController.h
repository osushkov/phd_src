
#ifndef _MainController_H_
#define _MainController_H_

#include <string>
#include <set>
#include <list>
#include <opencv/cv.h>
#include <opencv/highgui.h>

struct ImageRegion {
	unsigned minx, maxx;
	unsigned miny, maxy;
};

class MainController {
  public:
    static MainController& instance(void);

    void initialise(void);
    void reload(void);

    void demo(void);
    void mouseHandler(int event, int x, int y, int left_or_right);

  private:

    bool running;
    bool paused;

    unsigned cur_frame;

    ImageRegion left_region, right_region;
    IplImage *img_left, *img_right;


    void pickupObject(float leftx, float lefty, float rightx, float righty);

    void fillRegion(IplImage *img, unsigned x, unsigned y, std::set<unsigned> &visited_pixels,
					float max_delta, float max_ori_delta, double ro, double go, double bo,
					std::list< std::pair<unsigned,unsigned> > &check_queue, ImageRegion &region);

    float pixelDist(double r0, double g0, double b0, double r1, double g1, double b1);

    MainController();
};

#endif

