#include <ros/ros.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <termio.h>


#include "graupner_serial/RCchannelData.h"

#include <math.h>

#define BUFF_SIZE 1024
#define DATA_SIZE 64

int fd = -1;
u_int8_t buff[BUFF_SIZE];
uint data[DATA_SIZE];

uint data_duration[DATA_SIZE];

u_int8_t msgState = 0;
uint byteCount = 0;
uint8_t msgID= 0;
uint8_t timeFlag = 0;

//system variables
bool ConnectionFlag = true;
std::string Port_str = "/dev/tty";
const char* Port = "/dev/tty";
int BaudRate = 57600;

// process variables
double timeStart = 0;

int PWM;
int PWM_old = -1;
int pulseValue = 0;
int pulseValueOld = -1;
int direction = 0;
int directionOld = -1;
int PWM1, PWM2;
int pulseNum = 1, pulseTol = 20;
int pulseNum_r = 0, pulseTol_r = 0;

int duration_CH1 = 0;
int duration_CH2 = 0;
int duration_CH3 = 0;
int duration_CH4 = 0;
int duration_CH5 = 0;
int duration_CH6 = 0;

bool sendRollControlParams = false;
bool sendHeightControlParams = false;
bool sendPulseParams = false;
bool sendTrigAngle = false;
bool sendPWM = false;

float Ki = 1.0;
float Kp = 1.0;
float Kd = 1.0;
int Td = 20;

float Ki_z = 2.0;
float Kp_z = 2.0;
float Kd_z = 2.0;
int Td_z = 50;
int zRef = 50;

float Ki_r = 0.0f, Kp_r = 0.0f, Kd_r = 0.0f;
int Td_r = 0;

float Kp_h = 0.0f, Ki_h = 0.0f, Kd_h = 0.0f;
int Td_h = 0;
int zRef_h = 0;

bool rollControl = false;
bool heightControl = false;
bool sendHeightOnOff = false;
int rControl = 2;
int hControl = 2;
double vz;
int ax, ay, az, mx, my, mz;

//data
double x, y, z, roll, pitch, yaw, w, t;

int trig_angle = 0;
float apm_trig_angle = 0.0f;

void openPort(const char* port, int baudRate) {
	// baudRates = {19200, 38400, 57600, 115200}, default 57600

	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd >= 0) {

		fcntl(fd, F_SETFL, 0);

		struct termios options;

		/*
		 * Get the current options for the port...
		 */
		tcgetattr(fd, &options);

		/*
		 * Set the baud rates...
		 */
		/*
		switch(baudRate) {
			case 19200:	cfsetispeed(&options, B19200);
						break;
			case 38400: cfsetispeed(&options, B38400);
						break;
			case 57600: cfsetispeed(&options, B57600);
									break;
			case 115200: cfsetispeed(&options, B115200)
									break;
			default: cfsetispeed(&options, B57600);
									break;
		}
		*/

		cfsetispeed(&options, B57600);

		/*
		 * Enable the receiver and set local mode...
		 */
		options.c_cflag |= (CLOCAL | CREAD);

		/*
		 * Set the new options for the port...
		 */
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;

		//options.c_cflag &= ~CNEW_RTSCTS;  //CNEW_RTSCTS; //Turn off flow control

		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //Make sure that Canonical input is off (raw input data)

		options.c_iflag &= ~(IXON | IXOFF | IXANY); //Turn off software control flow

		options.c_oflag &= ~OPOST; //Raw output data

		tcsetattr(fd, TCSANOW, &options);

		/*
		if (tcsetattr(fd, TCSANOW, &options) < 0) {
			ROS_INFO("Setting serial communication parameter failed!");
		}
		else {
			ROS_INFO("Setting serial communication parameter successful!");
		}
		*/

	}
}

double Round(double d)
{
  return floor(d + 0.5);
}

int readSerial() {

	int bytes = 0;
	int bytesRec = 0;
	int msgState = 0;
	int msgReady = 0;
	ioctl(fd, FIONREAD, &bytes);

	//ROS_INFO("readSerial started");
	//ROS_INFO("bytes: %d\n", bytes);

	if (bytes > 0) 
	{
		if (bytes < BUFF_SIZE)
			bytesRec = read(fd, buff, bytes);
		else
			bytesRec = read(fd, buff, BUFF_SIZE);


	//for (int i = 0; i < 15; i++)
	for (int i = 0; i < bytesRec; i++) 
	{
		if (buff[i] == 0xEE && msgState == 0)
			msgState = 1;
		else if (buff[i] == 0xEE && msgState == 1)
			msgState = 2;
		else if (msgState == 2)
		{
			//ROS_INFO("data[%d]: %d\n", byteCount, data[byteCount]);
			data[byteCount++] = buff[i];
			if (byteCount == 1)
			{
				if (data[0] >= 1 && data[0] <= 10)
					msgID = data[0];
					//msgID = (short)Convert.ToUInt32(data[0]);
				else
					{
						msgState = 0;
						byteCount = 0;
						msgID = 0;
					}
			}

			switch (msgID)
			{
				case 1: if (byteCount == 33)
                    		{
                        		msgState = 3;
                        		byteCount = 0;
                    		}
                    		break;
                		case 2: if (byteCount == 13)
                    		{
                   			msgState = 3;
                        		byteCount = 0;
                    		}
                    		break;
                		case 3: if (byteCount == 12)
                    		{
                   			msgState = 3;
                        		byteCount = 0;
                    		}
                    		break;
                		case 5: if (byteCount == 3)
                    		{
                   			msgState = 3;
                        		byteCount = 0;
                    		}
                    		break;
                		case 6: if (byteCount == 3)
                    		{
                   			msgState = 3;
                        		byteCount = 0;
                    		}
                    		break;
				default:
					msgState = 0;
					byteCount = 0;
					msgID = 0;
					break;
			}
		}
		else if (buff[i] == 0xFF && msgState == 3)
			msgState = 4;
		else if (buff[i] == 0xFF && msgState == 4)
		{
			msgState = 0;
			switch (msgID)
			{
                		case 2:
					/*Kp_r = data[1] + 256.0 * data[2];
                    			Kp_r = (Kp_r < 10000 ? Kp_r : Kp_r - 20000) / 10.0f;
                    			Ki_r = data[3] + 256.0 * data[4];
                    			Ki_r = (Ki_r < 10000 ? Ki_r : Ki_r - 20000) / 100.0f;
                    			Kd_r = data[5] + 256.0 * data[6];
                    			Kd_r = (Kd_r < 10000 ? Kd_r : Kd_r - 20000) / 10.0f;
                    			Td_r = (int)(data[7] + 256.0 * data[8]);
                    			rControl = (int) data[9];*/
					duration_CH1 = data[1] + 256.0*data[2];
					//ROS_INFO("CH1: %d\n", duration_CH1);
					duration_CH2 = data[3] + 256.0*data[4];
					//ROS_INFO("CH2: %d\n", duration_CH2);
					duration_CH3 = data[5] + 256.0*data[6];
					//ROS_INFO("CH3: %d\n", duration_CH3);
					duration_CH4 = data[7] + 256.0*data[8];
					//ROS_INFO("CH4: %d\n", duration_CH4);
					duration_CH5 = data[9] + 256.0*data[10];
					ROS_INFO("CH5: %d\n", duration_CH5);
					duration_CH6 = data[11] + 256.0*data[12];
					ROS_INFO("CH6: %d\n", duration_CH6);
								
                    		break;
				default:
				break;
			}
			msgReady = msgID;
			byteCount = 0;
			msgState = 0;
			msgID = 0;

		}
		else
		{
			msgState = 0;
			byteCount = 0;
			msgID = 0;
		}
	}
	for (int i = 0; i < bytesRec; i++)
	{
	//ROS_INFO("byte: %d\n", buff[i]);
	//data_duration[i] = buff[i];
	}
	//ROS_INFO("bytes recieved: %d\n", bytesRec);
	//duration_CH1 = data_duration[2] + 256.0*data_duration[3];
	//ROS_INFO("CH1: %d\n", duration_CH1);
	}
	return msgReady;
}



void SendUpdateRequest(){

	uint8_t buff[5];

	buff[0] = 0xEE;
        buff[1] = 0xEE;
        buff[2] = 0x01; //message id
        buff[3] = 0xFF;
        buff[4] = 0xFF;

	int n = 0;
	if (fd > 0){
		n = write(fd, buff, 5);
		ROS_INFO("request sent");
	}
	if (n < 5)
		printf("Warning: Wrote only %d bytes in serial output buffer\n", n);
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_graupner");
	ros::NodeHandle n;

	//ros::Subscriber subSystem = n.subscribe("spincopterSystem", 10, SystemCallback);
	
	while(!ConnectionFlag){
		ros::spinOnce();
	}

	//openPort("/dev/ttyO1", BaudRate);
	openPort("/dev/ttyACM0", BaudRate);
	//openPort("/dev/ttyUSB0", BaudRate);	

	if (fd < 0) 
	{
	    	ROS_ERROR("Cannot open port");
		return -1;
	}

	ROS_INFO("The serial port is opened.");
	
	ros::Publisher pubRCchannel = n.advertise<graupner_serial::RCchannelData>("GraupnerRCchannels", 10);

    	ros::Rate r(50);

//	ROS_INFO("before while");
    	while(ros::ok())
    	{
//		ROS_INFO("before readSerial");
		

		int msgReady = readSerial();

  //              ROS_INFO("after readSerial");
		if (msgReady == 2) 
		{

			graupner_serial::RCchannelData msgRCchdata;
			msgRCchdata.CH1 = duration_CH1;
			msgRCchdata.CH2 = duration_CH2;
			msgRCchdata.CH3 = duration_CH3;
			msgRCchdata.CH4 = duration_CH4;
			msgRCchdata.CH5 = duration_CH5;
			msgRCchdata.CH6 = duration_CH6;
			//PWM step
			//Direction
			pubRCchannel.publish(msgRCchdata);
		}
		SendUpdateRequest();

		
		ros::spinOnce();
        	r.sleep();
    }
}
