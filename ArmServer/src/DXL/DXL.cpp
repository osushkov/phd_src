#include "DXL.h"
#include "DXLConstants.h"
#include "serial.h"

#include <cstdio>
#include <iostream>

//#define MAX_SPEED		0
//#define ATTEMPTS		4
//#define LENGTH_INDEX	3
//#define MAX_BUFF		30
#define POSITION_LIMIT	1023
#define SPEED_LIMIT		1023

#define MAX_PARAM_NUM		128			// Maximum parameter number

/* These structs are based on the windows version of the API provided by the Robotis */

struct DXL_status_packet
{
	unsigned char	id;					// DXL ID
	unsigned char	length;				// packet length = parameter's number + 1(ID) + 1(error)
	unsigned char	error;				// DXL error
	unsigned char	parameter[MAX_PARAM_NUM];	// parameter
	unsigned char	checksum;					
};

struct DXL_inst_packet
{
	unsigned char	id;			      	// DXL ID
	unsigned char	length;				// packet length = parameter's number + 1(ID) + 1(Instruction) + 1(address)
	unsigned char	instruction;			// DXL instruction
	unsigned char	address;			// DXL address
	unsigned char	parameter[MAX_PARAM_NUM];	// parameters
	unsigned char	checksum;
};

DXLChain::DXLChain() {
	commPort = -1;
}

bool DXLChain::open(const char *pathname) {
	if((commPort = open_port(pathname)) ==-1){
		fprintf(stderr,"Initiasiation for the DXL port failed\n");
		return false;
	}
	/* set the port options */
	set_port_options(commPort);
	return true;
}

DXL *DXLChain::getDXL(int id) {
	vector<DXL *>::iterator it = dxls.begin();
	while (it != dxls.end()) {
		if ((*it)->id == id)
			return *it;
		
		it++;
	}
	DXL *rval = new DXL(this, id);
	dxls.push_back(rval);
	return rval;
}

bool DXLChain::close() {
	if (commPort == -1)
		return true;
	close_port(commPort);
	commPort = -1;
	return true;;
}

bool DXL_sendPacket(int commPort, struct DXL_inst_packet *instPacket) {
	if (commPort == -1) {
		return false;
	}
	unsigned char buffer[MAX_PARAM_NUM];
	unsigned char checksum = 0;
	buffer[0] = buffer[1] = 0xFF;
	checksum += buffer[2] = instPacket->id;
	checksum += buffer[3] = instPacket->length;
	checksum += buffer[4] = instPacket->instruction;
	checksum += buffer[5] = instPacket->address;
	
	// for each parameter copy and calc checksum
	for (int i = 0; i < instPacket->length - 3; i++) {
		checksum += buffer[i + 6] = instPacket->parameter[i];
	}
	instPacket->checksum = buffer[instPacket->length + 3] = ~checksum;
	int len = instPacket->length + 4;
	
//	printf("sending: [0x%2x", buffer[0]);
//	for (int i = 1; i < len; i++)
//		printf(", 0x%2x", buffer[i]);
//	printf("]\n");
	
	if(write(commPort, buffer, len) != len) {
		fprintf(stderr, "DXL: Could not send instruction.");
		return false;
	}
	return true;
}

int DXL_nread(int commPort, void *buffer, int len) {
	int c = 0;
	int n = c = read(commPort, buffer, len);
	while (n >= 0 && c < len) {
		c += n;
		n = read(commPort, ((char *)buffer) + c, len - c);
	}
	return c;
}

int DXL_receivePacket(int commPort, struct DXL_status_packet *statPacket) {
	unsigned char hdr;
	int n0xFF = 0;
	while (n0xFF < 2) {
		if (DXL_nread(commPort, &hdr, 1) != 1) {
			fprintf(stderr, "DXL: Unable to receive packet header.");
			return 0;
		}
		if (hdr != 0xFF)
			n0xFF = 0;
		else
			n0xFF++;
	}
	if (DXL_nread(commPort, (unsigned char *)statPacket, 3) != 3) {
		fprintf(stderr, "DXL: Unable to receive packet information.");
		return 0;
	}
	int numParams = statPacket->length - 2;
	if (DXL_nread(commPort, statPacket->parameter, numParams + 1) != numParams + 1) {
		fprintf(stderr, "DXL: Unable to receive parameters and checksum.");
		return 0;
	}
	statPacket->checksum = statPacket->parameter[numParams];
	return numParams;
}
	
DXL::DXL(DXLChain *chain, int id) {
	this->chain = chain;
	this->id = id;
}

#define DXL_MAKE_INSTRUCTION(PACKET,INSTRUCTION,ADDRESS, NUM_PARAMS)	\
			PACKET.id = id; \
			PACKET.instruction = INSTRUCTION; \
			PACKET.address = ADDRESS; \
			PACKET.length = NUM_PARAMS + 3;

bool DXL::setResponse(int responseType) {
	struct DXL_inst_packet instPacket;
	DXL_MAKE_INSTRUCTION(instPacket, INST_WRITE, P_RETURN_LEVEL, 1);
	instPacket.parameter[0] = responseType;
	
	struct DXL_status_packet statPacket;
	char returnLevel;
	
	/* Send the packet*/
	if (!DXL_sendPacket(chain->commPort, &instPacket))
		return false;
	
	/* Read to confirm the status */
	if(id != BROADCASTING_ID){
		DXL_MAKE_INSTRUCTION(instPacket, INST_READ, P_RETURN_LEVEL, 1);
		instPacket.parameter[0]= 1;
		
		/* Send the packet*/
		if (!DXL_sendPacket(chain->commPort, &instPacket))
			return false;
		
		/* Read the status returned */
		if(DXL_receivePacket(chain->commPort, &statPacket) != 1)
			return false;
		
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
	return true;
}

bool DXL::getPosn(int *posn) {
	struct DXL_inst_packet instPacket;
	DXL_MAKE_INSTRUCTION(instPacket, INST_READ, P_PRESENT_POSITION_L, 1);
	instPacket.parameter[0]= 2; /* I want to read two bytes */
	
	struct DXL_status_packet statPacket;
	
	if (!DXL_sendPacket(chain->commPort, &instPacket)) {
		fprintf(stderr, "DXL: Unable to send position request packet.");
		return false;
	}
	if(DXL_receivePacket(chain->commPort, &statPacket) != 2) {
		fprintf(stderr, "DXL: Unable to receive position packet.");
		return false;
	}
	
	*posn = ((statPacket.parameter[1] << 8) | statPacket.parameter[0]);
	return true;
}

bool DXL::moveToPosn(int posn, int speed) {
	struct DXL_inst_packet instPacket;
	DXL_MAKE_INSTRUCTION(instPacket, INST_WRITE, P_GOAL_POSITION_L, 4);
	
	if(posn > POSITION_LIMIT)
		posn = POSITION_LIMIT;
	if(speed > SPEED_LIMIT)
		speed = SPEED_LIMIT;
	
	int j=0;
	instPacket.parameter[j++] = (0xFF & posn);
	instPacket.parameter[j++] = ((posn >> 8) & 0xFF);
	instPacket.parameter[j++] = (0xFF & speed);
	instPacket.parameter[j++] = ((speed >> 8) & 0xFF);

	/* Send the packet*/
	bool rval = DXL_sendPacket(chain->commPort, &instPacket);
	if (!rval)
		fprintf(stderr, "DXL: Unable to send move instruction packet.");
	return true;
}

bool DXL::getTorque(int *torqueEnabled) {
	struct DXL_inst_packet instPacket;
	DXL_MAKE_INSTRUCTION(instPacket, INST_READ, P_TORQUE_ENABLE, 1);
	instPacket.parameter[0] = 1;
		
	struct DXL_status_packet statPacket;
	
	if (!DXL_sendPacket(chain->commPort, &instPacket)) {
		fprintf(stderr, "DXL: Unable to send request torque packet.");
		return false;
	}
	if(!DXL_receivePacket(chain->commPort, &statPacket) != 1) {
		fprintf(stderr, "DXL: Unable to receive torque packet.");
		return false;
	}
	
	*torqueEnabled = statPacket.parameter[0];
	return true;
}

bool DXL::setTorque(int torqueEnabled) {
	struct DXL_inst_packet instPacket;
	DXL_MAKE_INSTRUCTION(instPacket, INST_WRITE, P_TORQUE_ENABLE, 1);
	instPacket.parameter[0] = torqueEnabled;
	bool rval = DXL_sendPacket(chain->commPort, &instPacket);
	if (!rval)
		fprintf(stderr, "DXL: Unable to send torque set packet.");
	return rval;
}

bool DXL::getLoad(int *load){
	struct DXL_inst_packet instPacket;
	DXL_MAKE_INSTRUCTION(instPacket, INST_READ, P_PRESENT_LOAD_L, 1);
	instPacket.parameter[0]= 2; /* I want to read two bytes */
	
	struct DXL_status_packet statPacket;
	
	if (!DXL_sendPacket(chain->commPort, &instPacket)) {
		fprintf(stderr, "DXL: Unable to send load request packet.");
		return false;
	}
	if(DXL_receivePacket(chain->commPort, &statPacket) != 2) {
		fprintf(stderr, "DXL: Unable to receive load packet.");
		return false;
	}
	
    *load = ((statPacket.parameter[1] & 3) << 8) | statPacket.parameter[0];
	return true;
}

