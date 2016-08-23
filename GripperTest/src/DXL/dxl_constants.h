


/********************* Define error code *****************************/

////////// For Open method (return value of DXL_open()) ////////////////
#define SUCCESS_COMM_OPEN		0x10		// Success to open USBto485 converter
#define ERROR_COMM_OPEN			0x11		// Fail to open USBto485 converter

////////// For Send method (return value of DXL_send()) ////////////////
#define SUCCESS_SEND_PACKET		0x20		// Success to send instruction packet
#define ERROR_SEND_COMM			0x21		// Fail to send instruction packet because USBto485 is error
#define ERROR_SEND_LENGTH		0x22		// Fail to send instruction packet because Packet length is error

////////// For Receive method (return value of DXL_read()) /////////////
#define SUCCESS_RCV_PACKET		0x30		// Success to receive status packet
#define ERROR_RCV_LOST			0x31		// Fail to receive status packet because packet's data is lost
#define ERROR_RCV_CHSUM			0x32		// Fail to receive status packet because packet's checksum is incorrect
#define ERROR_RCV_TMOUT			0x33		// Fail to receive status packet because packet is not arrived until timeout

////////// For Error status bit ////////////////////////////////////////
#define ERROR_VOLTAGE			0x01		// 0th error bit	'00000001'	- DXL voltage error
#define ERROR_ANGLE				0x02		// 1th error bit	'00000010'	- DXL limited angle error
#define ERROR_OVERHEAT			0x04		// 2th error bit	'00000100'	- DXL overheatting error
#define ERROR_RANGE				0x08		// 3th error bit	'00001000'	- DXL range error
#define ERROR_CHECKSUM			0x10		// 4th error bit	'00010000'	- DXL packet's checksum error
#define ERROR_OVERLOAD			0x20		// 5th error bit	'00100000'	- DXL overload error
#define ERROR_INSTRUCTION		0x40		// 6th error bit	'01000000'	- DXL instruction code error

/**********************************************************************/



/******************* Define DXL instruction ***************************/

#define INST_PING           0x01		// Ping instruction
#define INST_READ           0x02		// Read instruction
#define INST_WRITE          0x03		// Write instruction
#define INST_REG_WRITE      0x04		// Reg_write instruction
#define INST_ACTION         0x05		// Action instruction
#define INST_RESET          0x06		// Reset instruction

/**********************************************************************/


/***************** Define DXL address (don't edit!) *******************/

////////////////////// DXL EEPROM address //////////////////////////////
#define P_MODEL_NUMBER_L      0x00		// Model number lower byte address
#define P_MODOEL_NUMBER_H     0x01		// Model number higher byte address
#define P_VERSION             0x02		// DXL version address
#define P_ID                  0x03		// DXL ID address
#define P_BAUD_RATE           0x04		// DXL baudrate address
#define P_RETURN_DELAY_TIME   0x05		// Return delay time address
#define P_CW_ANGLE_LIMIT_L    0x06		// CW limited angle lower byte address
#define P_CW_ANGLE_LIMIT_H    0x07		// CW limited angle higher byte address
#define P_CCW_ANGLE_LIMIT_L   0x08		// CCW limited angle lower byte address
#define P_CCW_ANGLE_LIMIT_H   0x09		// CCW limited angle higher byte address
#define P_LIMIT_TEMPERATURE   0x0b		// Limited temperature address
#define P_DOWN_LIMIT_VOLTAGE  0x0c		// Down limited voltage address
#define P_UP_LIMIT_VOLTAGE    0x0d		// Up limited voltage address
#define P_MAX_TORQUE_L        0x0e		// Max torque lower byte address
#define P_MAX_TORQUE_H        0x0f		// Max torque higher byte address
#define P_RETURN_LEVEL        0x10		// Return level address
#define P_ALARM_LED           0x11		// Alarm LED address
#define P_ALARM_SHUTDOWN      0x12		// Alarm shutdown address
#define P_DOWN_CALIBRATION_L  0x14		// Down calibration lower byte address
#define P_DOWN_CALIBRATION_H  0x15		// Down calibrarion higher byte address
#define P_UP_CALIBRATION_L    0x16		// Up calibrartion lower byte address
#define P_UP_CALIBRATION_H    0x17		// Up calibration higher byte address

///////////////////// DXL RAM address ////////////////////////////////////
#define P_TORQUE_ENABLE				0x18	// Torque enable flag address
#define P_LED						0x19	// LED on/off flag address
#define P_CW_COMPLIANCE_MARGIN		0x1a	// CW compliance margin address
#define P_CCW_COMPLIANCE_MARGIN		0x1b	// CCW compliance margin address
#define P_CW_COMPLIANCE_SLOPE		0x1c	// CW compliance slope address
#define P_CCW_COMPLIANCE_SLOPE		0x1d	// CCW compliance slope address
#define P_GOAL_POSITION_L			0x1e	// Goal position lower byte address
#define P_GOAL_POSITION_H			0x1f	// Goal position higher byte address
#define P_GOAL_SPEED_L				0x20	// Goal speed lower byte address
#define P_GOAL_SPEED_H				0x21	// Goal speed higher byte address
#define P_TORQUE_LIMIT_L			0x22	// Limited torque lower byte address
#define P_TORQUE_LIMIT_H			0x23	// Limited torque higher byte address
#define P_PRESENT_POSITION_L		0x24	// Present position lower byte address
#define P_PRESENT_POSITION_H		0x25	// Present position higher byte address
#define P_PRESENT_SPEED_L			0x26	// Present speed lower byte address
#define P_PRESENT_SPEED_H			0x27	// Present speed higher byte address
#define P_PRESENT_LOAD_L			0x28	// Present load lower byte address
#define P_PRESENT_LOAD_H			0x29	// Present load higher byte address
#define P_PRESENT_VOLTAGE			0x2a	// Present voltage address
#define P_PRESENT_TEMPERATURE		0x2b	// Present temperature address
#define P_REGISTERED_INSTRUCTION	0x2c	// Registered instruction address
#define P_MOVING					0x2e	// Moving state flag address
#define P_EEPROM_LOCK				0x2f	// EEPROM lock flag address
#define P_PUNCH_L					0x30	// Punch lower byte address
#define P_PUNCH_H					0x31	// Punch higher byte address

/*************************************************************************/


/********************** Define DXL data **********************************/

//////////////////// LED flag /////////////////////////
#define DXL_LED_OFF			0x00			// LED off
#define DXL_LED_ON			0x01			// LED on

//////////////////// Torque flag //////////////////////
#define DXL_TORQUE_OFF		0x00			// Torque disable
#define DXL_TORQUE_ON		0x01			// Torque enable

/////////////////// EEPROM flag ///////////////////////
#define DXL_ROM_UNLOCK		0x00			// EEPROM unlock
#define DXL_ROM_LOCK		0x01			// EEPROM lock

///////////////////// DXL ID //////////////////////////
#define BROADCASTING_ID		0xfe			// Broadcast ID

//////////////////// DXL baudrate /////////////////////
#define DXL_BAUD_1			1000000			// 1000000 bps
#define DXL_BAUD_3			500000			// 500000 bps
#define DXL_BAUD_4			400000			// 400000 bps
#define DXL_BAUD_7			250000			// 250000 bps
#define DXL_BAUD_9			200000			// 200000 bps
#define DXL_BAUD_16			115200			// 115200 bps
#define DXL_BAUD_34			57600			// 57600 bps
#define DXL_BAUD_103		19200			// 19200 bps
#define DXL_BAUD_207		9600			// 9600 bps

/*************************************************************************/


