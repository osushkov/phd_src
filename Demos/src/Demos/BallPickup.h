/*
 * BallPickup.h
 *
 *  Created on: 19/03/2009
 *      Author: osushkov
 */

#ifndef BALLPICKUP_H_
#define BALLPICKUP_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "../Util/Vector3D.h"

class BallPickup {
  public:
    BallPickup();
    ~BallPickup();

    void lookForBall(IplImage *left_img, IplImage *right_img);
    bool canSeeBall(void);

    void grabBall(void);
    void moveBall(void);
    void releaseBall(void);

  private:
    bool saw_ball_last_frame;
    Vector3D ball_world_pos;

    unsigned num_frames_saw_ball;


    IplImage* thresholdBallColour(IplImage *src, std::vector<CvConnectedComp> &balls);

};

#endif /* BALLPICKUP_H_ */
