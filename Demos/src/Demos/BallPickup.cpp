
#include "BallPickup.h"
#include "../Control.h"
#include "../Util/Common.h"
#include "../Arm/Arm.h"

BallPickup::BallPickup(){
    saw_ball_last_frame = false;

    cvNamedWindow("LeftBall", 1);
    cvMoveWindow("LeftBall", 600, 400);

    cvNamedWindow("RightBall", 1);
    cvMoveWindow("RightBall", 950, 400);

    num_frames_saw_ball = 0;
}

BallPickup::~BallPickup() {
    for (int i = 0; i < 2; i++) {
        cvDestroyWindow("LeftBall");
        cvWaitKey(5);
        cvDestroyWindow("RightBall");
        cvWaitKey(5);
    }
}

void BallPickup::lookForBall(IplImage *left_img, IplImage *right_img){
    std::vector<CvConnectedComp> left_balls, right_balls;

    IplImage *left_threshold = thresholdBallColour(left_img, left_balls);
    IplImage *right_threshold = thresholdBallColour(right_img, right_balls);

    Vector2D left_ball_middle =
        Vector2D(left_balls[0].rect.x, left_balls[0].rect.y) +
        Vector2D(left_balls[0].rect.width/2.0f, left_balls[0].rect.height/2.0f);

    Vector2D right_ball_middle =
            Vector2D(right_balls[0].rect.x, right_balls[0].rect.y) +
            Vector2D(right_balls[0].rect.width/2.0f, right_balls[0].rect.height/2.0f);

    ball_world_pos = Common::stereoProject(left_ball_middle, right_ball_middle);
    ball_world_pos = Common::cameraPointToWorldSpace(ball_world_pos, 0.0f, -65.0f, 0.0f);
    num_frames_saw_ball++;

    std::cout << ball_world_pos.x << " "
              << ball_world_pos.y << " "
              << ball_world_pos.z << std::endl;

    cvCircle(left_threshold, cvPoint(left_ball_middle.x, left_ball_middle.y),
             1, cvScalar(200), 2.0);

    cvShowImage("LeftBall", left_threshold);
    cvShowImage("RightBall", right_threshold);

    cvWaitKey(5);

    cvReleaseImage(&left_threshold);
    cvReleaseImage(&right_threshold);
}

bool BallPickup::canSeeBall(void){
    return saw_ball_last_frame;
}

void BallPickup::grabBall(void){

}

void BallPickup::moveBall(void){
    if(num_frames_saw_ball < 20){
        std::cout << "Not picking up : " << num_frames_saw_ball << std::endl;
        return;
    }

    Vector3D approach = Vector3D(1.0, 0.0, -0.4);
    Control::moveArmToWorldPoint(ball_world_pos + Vector3D(0.0f, 0.0f, 10.0f), approach);
    Control::moveArmToWorldPoint(ball_world_pos + Vector3D(0.0f, 0.0f, -0.2f), approach);
    Arm::getArm()->closeHand(0.45f);
    sleep(2);
    Control::moveArmToWorldPoint(ball_world_pos + Vector3D(0.0f, 0.0f, 15.0f), approach);
    Arm::getArm()->openHand(0.7f);
    sleep(2);
    exit(1);
}

void BallPickup::releaseBall(void){

}

IplImage* BallPickup::thresholdBallColour(IplImage *src, std::vector<CvConnectedComp> &balls) {
    std::vector<unsigned char> blueness_data;

    for (int y = 0; y < src->height; y++) {
        for (int x = 0; x < src->width; x++) {
            double r, g, b;
            Common::getPixel(src, x, y, b, g, r);
            Vector3D cur_col(r, g, b);
            Vector3D target_col(0.0f, 0.0f, 255.0f);

            float dist = (target_col - cur_col).length()
                    / Vector3D(255.0f, 255.0f, 255.0f).length();
            dist = 1.0f - dist;
            dist = dist * 255.0f;

            blueness_data.push_back(dist);
        }
    }

    IplImage* hi_img = Common::imageFromBuffer(blueness_data, src->width, src->height, 1);
    IplImage* hi_img_thresh = cvCloneImage(hi_img);

    cvThreshold(hi_img, hi_img_thresh, 130, 255, CV_THRESH_BINARY); // threshold the heightmap image
    cvDilate(hi_img_thresh, hi_img_thresh, NULL, 1);

    for (int y = 0; y < hi_img_thresh->height; y++) {
        for (int x = 0; x < hi_img_thresh->width; x++) {

            double val;
            Common::getPixel(hi_img_thresh, x, y, val);
            unsigned char pix_val = (unsigned char) val;

            if (pix_val == 255) {
                CvConnectedComp new_component;
                cvFloodFill(hi_img_thresh, cvPoint(x, y), cvScalar(100),
                            cvScalarAll(1),  cvScalarAll(1),
                            &new_component);
                balls.push_back(new_component);
            }
        }
    }

    cvReleaseImage(&hi_img);
    return hi_img_thresh;
}
