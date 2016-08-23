/**
 * dxl_api.h
 * Author: Nawid Jamali, Aung Kyaw Htet
 * Date: November 2006
 * This file implements a preliminary Linux API for the Dynamixel DXL series servos,
 *	it is based on the windows implementation of the API provided by Robotis.
 */

#ifndef DXL_API_H
#define DXL_API_H

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include "dxl_api_param.h"
#include "dxl_constants.h"
#include "serial.h"



/* Sends a command to the DXL over serial port 
 * returns -1 on falure to send, otherwise 0
 */
int send_command(const unsigned char *command,int lenght, int fd);


/* Read n number of data from serial port 
 * on success returns 0, on failure returns
 * the number of bytes read
 */
int nread(unsigned char *buffer, int number, int fd);


/* Sends a packet to the DXL 
 * returns -1 on failure to send, otherwise 0
 */
int send_packet(int dxl_port, struct DXL_inst_packet * instPacket);


/* Read a packet from the serial port */
int read_packet(int dxl_port, struct DXL_status_packet* statusPacket);



/* Initialsies DXL
 * returns -1 on failure, 0 on success
 * NB: Opens the port at a default speed of 1Mbps
 * Need to make a change so the baude rate can be defined by the user
 */
int DXL_open(const char *pathname);


/* Close DXL device */
void DXL_close(int port);


void DXL_set_response(int dxl_port, int responseType, int id);


/* Move to a position 
 * id: The Id of the servo to be moved
 * position: the final intended position
 * speed: speed at which the move action will be carried
 */
void DXL_move_to(int dxl_port, int id, int position, int speed);


/* Read the current position of the servo 
 * returns the position, on error returns -1
 */
int DXL_get_position(int dxl_port, int id);


void DXL_set_torque(int dxl_port, int id, char enable);

#endif
