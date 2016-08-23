
#include "MainController.h"


#include "Camera/Camera.h"
#include "Camera/RemoteCamera.h"
#include "Camera/PlaybackCamera.h"
#include "Camera/Recorder.h"

#include "Util/Common.h"
#include "Util/ParallelServer.h"
#include "Util/Timer.h"
#include "Util/PerfStats.h"
#include "Util/Semaphore.h"
#include "Util/Geometry.h"

#include "Arm/Arm.h"
#include "Settings.h"
#include "Control.h"

#include "Console.h"
#include "Demos/BallPickup.h"
#include "Arm/Arm.h"
#include "Arm/ArmForwardKinematics.h"

#include "CameraController.h"


MainController& MainController::instance(void){
    static MainController main_controller;
    return main_controller;
}

MainController::MainController() : cur_frame(0) {

}


void MainController::initialise(void){
    cur_frame = 0;
}

void MainController::reload(void){

    //initialise();
}

void mouseHandleLeft(int event, int x, int y, int flags, void *param){
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        MainController::instance().mouseHandler(event, x, y, 0);
        break;

    }
}

void mouseHandleRight(int event, int x, int y, int flags, void *param){
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        MainController::instance().mouseHandler(event, x, y, 1);
        break;

    }
};

void MainController::demo(void){
    std::cout << "MainController: demo " << std::endl;
    Control::moveArmOutOfTheWay();

    CameraController camera_controller(true, "");
    camera_controller.createViewWindows();

    left_region.maxx = left_region.maxy = left_region.minx = left_region.miny = 0.0f;
    right_region = left_region;

    cvSetMouseCallback("LeftEye", mouseHandleLeft, NULL);
    cvSetMouseCallback("RightEye", mouseHandleRight, NULL);

    while (camera_controller.isActive()) {
        if (!camera_controller.getNewFrame()) {
            break;
        }

        img_left = camera_controller.getLeftImage();
        img_right = camera_controller.getRightImage();

        cvRectangle(img_left, cvPoint(left_region.minx, left_region.miny),
        		    cvPoint(left_region.maxx, left_region.maxy), CV_RGB(255, 0, 0));
        cvRectangle(img_right, cvPoint(right_region.minx, right_region.miny),
					cvPoint(right_region.maxx, right_region.maxy), CV_RGB(255, 0, 0));

        char key = camera_controller.showCameraViewAndContinue();
        switch(key){
        case 'q':
        	break;
        case 'p':
        	std::cout << "picking up" << std::endl;
        	pickupObject((left_region.maxx+left_region.minx)/2.0f,(left_region.maxy+left_region.miny)/2.0f,
						 (right_region.maxx+right_region.minx)/2.0f,(right_region.maxy+right_region.miny)/2.0f);
        	break;
        }
    }

}

void MainController::mouseHandler(int event, int x, int y, int left_or_right){
	std::cout << x << " " << y << " " << left_or_right << std::endl;

	std::list<std::pair<unsigned,unsigned> > check_queue;
	check_queue.push_back(std::pair<unsigned,unsigned>(x,y));

	std::set<unsigned> visited_pixels;

	if(left_or_right == 0){
		left_region.minx = left_region.maxx = x;
		left_region.miny = left_region.maxy = y;

		double r, g, b;
		Common::getPixel(img_left, x, y, r, g, b);

		while(check_queue.size() > 0){
			std::pair<unsigned,unsigned> cur_pos = check_queue.front();
			fillRegion(img_left, cur_pos.first, cur_pos.second, visited_pixels, 10.0f, 80.0f,
					   r, g, b, check_queue, left_region);
			check_queue.pop_front();
		}
		std::cout << "damn" << std::endl;
	}
	else{
		right_region.minx = right_region.maxx = x;
		right_region.miny = right_region.maxy = y;

		double r, g, b;
		Common::getPixel(img_right, x, y, r, g, b);

		while(check_queue.size() > 0){
			std::pair<unsigned,unsigned> cur_pos = check_queue.front();
			fillRegion(img_right, cur_pos.first, cur_pos.second, visited_pixels, 10.0f, 80.0f,
					   r, g, b, check_queue, right_region);
			check_queue.pop_front();
		}
	}
}

void MainController::pickupObject(float leftx, float lefty, float rightx, float righty){
	Vector3D pos = Common::stereoProject(Vector2D(leftx, lefty), Vector2D(rightx, righty));
	std::cout << pos.length() << std::endl;

	Vector3D world_pos = Common::cameraPointToWorldSpace(pos, 0.0f, -65.0f, 0.0f);
	Control::moveArmToWorldPoint(world_pos + Vector3D(0.0f, 0.0f, 1.0f));

    //Control::tiltWritst(-5.0f);

	//Control::moveArmToWorldPoint(world_pos);
	//Arm::getArm()->softCloseHand();
    Arm::getArm()->closeHand(0.85f);


    Control::moveArmToWorldPoint(world_pos + Vector3D(0.0f, 0.0f, 10.0f));

    std::cout << "Drop?" << std::endl;
    getchar();
    Arm::getArm()->openHand(1.0f);
    Control::moveArmOutOfTheWay();

	//Control::moveArmToWorldPoint(world_pos + Vector3D(0.0f, 0.0f, 7.0f));
}

void MainController::fillRegion(IplImage *img, unsigned x, unsigned y,
								std::set<unsigned> &visited_pixels,
								float max_delta, float max_ori_delta,
								double ro, double go, double bo,
								std::list< std::pair<unsigned,unsigned> > &check_queue,
								ImageRegion &region){
	unsigned cur_index = x + y*img->width;
	if(visited_pixels.find(cur_index) != visited_pixels.end()){
		return;
	}

	//std::cout << x << " : " << y << std::endl;
	visited_pixels.insert(cur_index);
	if(x < region.minx){ region.minx = x; }
	if(y < region.miny){ region.miny = y; }
	if(x > region.maxx){ region.maxx = x; }
	if(y > region.maxy){ region.maxy = y; }

	double cur_r, cur_g, cur_b;
	Common::getPixel(img, x, y, cur_r, cur_g, cur_b);

	float odiff = pixelDist(cur_r, cur_g, cur_b, ro, go, bo);
	if(odiff > max_ori_delta){ return; }

	if(x > 0){
		double left_r, left_g, left_b;
		Common::getPixel(img, x-1, y, left_r, left_g, left_b);
		float diff = pixelDist(cur_r, cur_g, cur_b, left_r, left_g, left_b);
		if(diff < max_delta){
			check_queue.push_back(std::pair<unsigned,unsigned>(x-1, y));
		}
	}
	if(x < img->width-1){
		double right_r, right_g, right_b;
		Common::getPixel(img, x+1, y, right_r, right_g, right_b);
		float diff = pixelDist(cur_r, cur_g, cur_b, right_r, right_g, right_b);
		if(diff < max_delta){
			check_queue.push_back(std::pair<unsigned,unsigned>(x+1, y));
		}
	}

	if(y > 0){
		double up_r, up_g, up_b;
		Common::getPixel(img, x, y-1, up_r, up_g, up_b);
		float diff = pixelDist(cur_r, cur_g, cur_b, up_r, up_g, up_b);
		if(diff < max_delta){
			check_queue.push_back(std::pair<unsigned,unsigned>(x, y-1));
		}
	}
	if(y < img->height-1){
		double down_r, down_g, down_b;
		Common::getPixel(img, x, y+1, down_r, down_g, down_b);
		float diff = pixelDist(cur_r, cur_g, cur_b, down_r, down_g, down_b);
		if(diff < max_delta){
			check_queue.push_back(std::pair<unsigned,unsigned>(x, y+1));
		}
	}
}

float MainController::pixelDist(double r0, double g0, double b0, double r1, double g1, double b1){
	float rd = (r0-r1)*(r0-r1);
	float gd = (g0-g1)*(g0-g1);
	float bd = (b0-b1)*(b0-b1);
	return sqrtf(rd + gd + bd);
}
