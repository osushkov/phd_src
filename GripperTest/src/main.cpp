
#include <iostream>
#include "DXL/dxl_api.h"

#define HAND_PORT	"/dev/ttyUSB1"
#define handSpeed	127
//position closed
#define hand_0_0	735
#define hand_0_1	280
#define hand_0_2	600
#define hand_0_3	735
//position opened
#define hand_1_0	850
#define hand_1_1	170
#define hand_1_2	485
#define hand_1_3	850


#define neck_position_0_0	500	
// pan: 490 and 495 creates error on left position
#define neck_position_0_1	150
#define neck_position_0_2	440	//490 ?
//front
#define neck_position_1_0	430
#define neck_position_1_1	140
#define neck_position_1_2	440
//right
#define neck_position_2_0	360
#define neck_position_2_1	150
#define neck_position_2_2	440	//390 ?


struct DXL_hand{
	int port;
	int speed;
	int status;
	int coords[2][4];
};
struct DXL_hand hand;


void move_hand(int dir){
	int id, pos;
	if(dir == 0 || dir == 1) {
		for(id=0; id<4; id++) {
			pos = hand.coords[dir][id];
			DXL_move_to(hand.port, id, pos, hand.speed);
		}
		hand.status = dir;
	}
}

int startup_hand(void){
	hand.port = DXL_open(HAND_PORT);
	hand.speed = handSpeed;
/*
	hand.coords[0][0] = neck_position_0_0;
	hand.coords[0][1] = neck_position_0_1;
	hand.coords[0][2] = neck_position_0_2;
	hand.coords[0][3] = neck_position_0_3;

	//opened position
	hand.coords[1][0] = neck_position_1_0;
	hand.coords[1][1] = neck_position_1_1;
	hand.coords[1][2] = neck_position_1_2;
	hand.coords[1][3] = neck_position_1_3;
*/

	//closed position
	
	hand.coords[0][0] = hand_0_0;
	hand.coords[0][1] = hand_0_1;
	hand.coords[0][2] = hand_0_2;
	hand.coords[0][3] = hand_0_3;

	//opened position
	hand.coords[1][0] = hand_1_0;
	hand.coords[1][1] = hand_1_1;
	hand.coords[1][2] = hand_1_2;
	hand.coords[1][3] = hand_1_3;
	
	std::cout << "Connected to Hand" << std::endl;

	//open hand
	//move_hand(1);

	//close hand
	//move_hand(0);
	std::cout << "Hand started up" << std::endl;

    return 1;
}

void shutdown_hand(void){
	int id;
	for(id=0; id<4; id++)
		DXL_set_torque(hand.port, id, 0);
	DXL_close(hand.port);
}

int main(int argc, char **argv){
    startup_hand();
    sleep(5);
    move_hand(0);
    sleep(5);
    shutdown_hand();

    return 0;
}

