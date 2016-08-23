/**
 * dxl_api_param.h
 * Author: Nawid Jamali, Aung Kyaw Htet
 * Date: November 2006
 * This file implements a preliminary Linux API for the Dynamixel DXL series servos,
 *	it is based on the windows implementation of the API provided by Robotis.
 */

#define MAX_SPEED		0
#define ATTEMPTS		4
#define LENGHT_INDEX	3
#define MAX_BUFF		30
#define POSITION_LIMIT	1023
#define SPEED_LIMIT		1023

#define MAX_PARAM_NUM		128			// Maximum parameter number

#define DXL_360		1228.8	//1024 for 300 degrees means 1228.8 for 360
