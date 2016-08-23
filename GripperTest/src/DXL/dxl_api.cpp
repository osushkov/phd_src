/*
 * dxl_api.c
 * Author: Nawid Jamali, Aung Kyaw Htet
 * Date: November 2006
 * This file implements a preliminary Linux API for the Dynamixel DXL series servos,
 *	it is based on the windows implementation of the API provided by Robotis.
 */

#ifndef DXL_API_C
#define DXL_API_C

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include "dxl_constants.h"
#include "serial.h"

#define MAX_SPEED		0
#define ATTEMPTS		4
#define LENGHT_INDEX	3
#define MAX_BUFF		30
#define POSITION_LIMIT	1023
#define SPEED_LIMIT		1023

#define MAX_PARAM_NUM		128			// Maximum parameter number


/* These structs are based on the windows version of the API provided by the Robotis */
struct DXL_status_packet
{
	unsigned char	id;					// DXL ID
	unsigned char	length;				// packet length = parameter's number + 1(ID) + 1(error)
	unsigned char	error;				// DXL error
	unsigned char	parameter[MAX_PARAM_NUM];	// parameters								
};

struct DXL_inst_packet
{
	unsigned char	id;			      	// DXL ID
	unsigned char	length;				// packet length = parameter's number + 1(ID) + 1(Instruction) + 1(address)
	unsigned char	instruction;			// DXL instruction
	unsigned char	address;			// DXL address
	unsigned char	parameter[MAX_PARAM_NUM];	// parameters
};


/* Sends a command to the DXL over serial port 
 * returns -1 on falure to send, otherwise 0
 */
int send_command(const unsigned char *command,int lenght, int fd)
{
	int nbytes;
	//tcflush(fd , TCOFLUSH);
	nbytes = write(fd,command, lenght);
	if(nbytes < lenght){
		fprintf(stderr, "dxl_api.c: send command failed\n");
		return -1;
	}
	//tcflush(fd , TCOFLUSH);
	return 0;
}


/* Read n number of data from serial port 
 * on success returns 0, on failure returns
 * the number of bytes read
 */
int nread(unsigned char *buffer, int number, int fd)
{
	int nbytes = 0;
	int i;
	for(i =0; i<ATTEMPTS; i++){
		/* Wait till the data becomes available */
		while(!check_port_for_data(fd, 1 ,0));
		nbytes += read(fd, &buffer[nbytes], number-nbytes);
		if(nbytes == number) break;
	}
	if(i >= ATTEMPTS){
		fprintf(stderr, "nread: Did not get data in %d Attempts\n", i);
		return nbytes;
	}
	return 0;
}


/* Sends a packet to the DXL 
 * returns -1 on failure to send, otherwise 0
 */
int send_packet(int dxl_port, struct DXL_inst_packet * instPacket){
	unsigned char checksum;
	unsigned char sendBuff[MAX_BUFF];
	int i, j=0;

	checksum = instPacket->id+instPacket->length+instPacket->instruction+instPacket->address;
	
	sendBuff[j++] = 0xFF;
	sendBuff[j++] = 0xFF;
	sendBuff[j++] = instPacket->id;
	sendBuff[j++] = instPacket->length;
	sendBuff[j++] = instPacket->instruction;
	sendBuff[j++] = instPacket->address;
	for(i=0; i < instPacket->length-3; i++){
		/* The length field in packet includes counts for 
		 *elements other than data, hence need to subtract 3*/ 
		sendBuff[j++] = instPacket->parameter[i];
		checksum+= instPacket->parameter[i];
	}
	sendBuff[j++] = ~checksum;

	return send_command(sendBuff, instPacket->length+4, dxl_port); 
	/* Packet length does not account for two 
	 *OxFFs, length, and checksum, hence add 4 */
}


/* Read a packet from the serial port */
int read_packet(int dxl_port, struct DXL_status_packet* statusPacket){
	unsigned char buffer[MAX_BUFF];
	unsigned char checksum = 0;
	int i; 

	if(!nread(buffer, 5, dxl_port)){
		/* Test if this is the beginning of buffer */
		if(buffer[0] != 0xFF ||buffer[1] != 0xFF){
			fprintf(stderr, "read_data: Start of buffer missing!\n");
			/*Flush the input buffer */
			tcflush(dxl_port, TCIFLUSH);
			return -1;
		}
	}

	/* If we are here everthing is fine */
	statusPacket->id = buffer[2];
	statusPacket->length = buffer[3];
	statusPacket->error = buffer[4];

	/* check is we have enough room to read data */
	if (buffer[3] > MAX_PARAM_NUM){
		fprintf(stderr, "read_packet: Incoming Data exceeds maximum number of parameter allwoed\n");
		return -1;
	}
	nread(statusPacket->parameter, buffer[3]-2, dxl_port);
	nread(buffer, 1, dxl_port); /* Read the checksum */

	/* calculate checksum */
	checksum = statusPacket->id+statusPacket->length+statusPacket->error;
	for (i=0; i<buffer[3]-2; i++)
		checksum+= statusPacket->parameter[i];
	
	if(buffer[0] != ((~checksum)&0x0FF)){
		fprintf(stderr, "Checksum failed!\nRecieved %x, Calculated %x", buffer[0], ~checksum);
		return -1;
	}
	return 0;
}



/* Initialsies DXL
 * returns -1 on failure, 0 on success
 * NB: Opens the port at a default speed of 1Mbps
 * Need to make a change so the baude rate can be defined by the user
 */
int DXL_open(const char *pathname)
{
	int dxl_port;	
	if((dxl_port = open_port(pathname)) ==-1){
		fprintf(stderr,"Initiasiation for the DXL port failed\n");
		return -1;
	}
	/* set the port options */
	set_port_options(dxl_port);
	return dxl_port;
}


/* Close DXL device */
void DXL_close(int port){
	 close_port(port);
}


void DXL_set_response(int dxl_port, int responseType, int id)
{
	struct DXL_inst_packet instPacket;
	struct DXL_status_packet statPacket;
	char returnLevel;

	/* Get the packet ready to be sent*/
	instPacket.id = id;
	instPacket.length = 4; /* Read length */
	instPacket.instruction = INST_WRITE;
	instPacket.address = P_RETURN_LEVEL;
	instPacket.parameter[0] = responseType;
	
	/* Send the packet*/
	send_packet(dxl_port, &instPacket);

	/* Read to confirm the status */
	if(id!=0xFE){
		instPacket.id = id;
		instPacket.length = 3; /* Read length */
		instPacket.instruction = INST_READ;
		instPacket.address = P_RETURN_LEVEL;
		instPacket.parameter[0]= 1;
		
		/* Send the packet*/
		send_packet(dxl_port, &instPacket);
		
		/* Read the status returned */
		read_packet(dxl_port, &statPacket);
		
		returnLevel = statPacket.parameter[0];
		switch(returnLevel){
		case 0: 
			printf("Return Level: 0 => Do not respond to any instruction");
			break;
		case 1:
			printf("Return Level: 1 => Respond only to READ instruction");
			break;
		case 3:
			printf("Return Level: 2 => Respond to all instructions");
		}
	}
}


/* Move to a position 
 * id: The Id of the servo to be moved
 * position: the final intended position
 * speed: speed at which the move action will be carried
 */
void DXL_move_to(int dxl_port, int id, int position, int speed){
	struct DXL_inst_packet instPacket;
	struct DXL_status_packet statPacket;
	int j=0;
	
	if(position > POSITION_LIMIT)
		position = POSITION_LIMIT;
	if(speed > SPEED_LIMIT)
		speed = SPEED_LIMIT;

	/* Get the packet ready to be sent*/
	instPacket.id = id;
	instPacket.length = 7; /* Read length */
	instPacket.instruction = INST_WRITE;
	instPacket.address = P_GOAL_POSITION_L;
	
	instPacket.parameter[j++] = (0x0FF&position);
	instPacket.parameter[j++] = ((position >> 8)&0x0FF);
	instPacket.parameter[j++] = (0x0FF&speed);
	instPacket.parameter[j++] = ((speed >> 8) & 0xFF);

	/* Send the packet*/
	send_packet(dxl_port, &instPacket);
}


/* Read the current position of the servo 
 * returns the position, on error returns -1
 */
int DXL_get_position(int dxl_port, int id){
	struct DXL_inst_packet instPacket;
	struct DXL_status_packet statPacket;

	instPacket.id = id;
	instPacket.length = 4; /* Read length */
	instPacket.instruction = INST_READ;
	instPacket.address = P_PRESENT_POSITION_L;
	instPacket.parameter[0]= 2; /* I want to read two bytes */ 
	
	send_packet(dxl_port, &instPacket);
	if(read_packet(dxl_port, &statPacket)!=-1)
		return ((statPacket.parameter[1]<<8)| statPacket.parameter[0]);
	return -1;
}


void DXL_set_torque(int dxl_port, int id, char enable){
	struct DXL_inst_packet instPacket;
	struct DXL_status_packet statPacket;
	instPacket.id = id;
	instPacket.instruction = INST_WRITE;
	instPacket.address = P_TORQUE_ENABLE;
	instPacket.parameter[0] = enable;
	instPacket.length = 4;
	send_packet(dxl_port, &instPacket);
}

#endif
