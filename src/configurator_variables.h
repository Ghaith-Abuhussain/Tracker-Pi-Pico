#include <stdio.h>
#include <string.h>
#define IO_EVENT_ARRAY_SIZE 25
#define SMS_SEND_TO_SIZE    10
char arrayNum[10] = "";

 // APN Variables
unsigned char gprsContext        = 0;
unsigned char gprsAuthentication = 0;
char APN[32];
char APN_Username[32];
char APN_Password[32];
unsigned char APNLen = 0;
unsigned char APN_UsernameLen = 0;
unsigned char APN_PasswordLen = 0;

// data Aquisition Periods Variables
int dataAqOnStopVars[9]           = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int dataAqMovingVars[18]          = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int dataAqOnDemandTrackingVars[2] = {0, 0};

// main Server Setting
char  mainServerIp[4];
int   mainServerPort           = 0;
unsigned char   mainServerProtocol      = 0;
unsigned char   mainServerTlsEncryption = 0;

// second Server Setting
unsigned char   secondServerMode          = 0;
char   secondServerIp[4];
int    secondServerPort          = 0;
unsigned char   secondServerProtocol      = 0;
unsigned char   secondServerTlsEncryption = 0;

// recordTimeoutSetting
int openLinkTimeout      = 0;
int responseTimeout      = 0;
int networkPingTimeout   = 0;
unsigned char sortBy              = 0;
unsigned char recordTlsEncryption = 0;

// IO Setting
int ioType = 0;
struct IO_System_Event
{
	unsigned char priority;
	int lowLevel ;
	int highLevel;
	unsigned char eventsOnly;
	unsigned char operand;
	int avgConstant;
	unsigned char sendSmsToLength;
	char sendSmsTo[10];
};

struct IO_System_Event ioEventsArray[20];
int sizeof_ioEventsArray = 20;

// send variables to configurator
int  infoType = 0;
unsigned char varIndex = 0;


struct time_struct
{
	int seconds;
	int hours;
	int minutes;
	int days;
	int months;
	int years;
};

// readed message from server variables
int availableBytes = 0;


// parser variables
char c = ' ';
int parserheader = 0;
int parserdataLength = 0;
int parserdirectionOfData = 0;
char parserdataMessage[256];
int parsercommand = 0;
struct time_struct parserdate = {0, 0, 0, 0, 0, 0};
int parsercheckSum = 0;
int parserfooter = 0;

//#include "MessageParser.h"
int incomingByte = 0; // for incoming serial data
char message_to_server[256];

//byte* message_from_server = new byte[1024];
char deviceType[6];

unsigned long time_now = 0;
int r = 0;
int k=0;
int i=0;
uint8_t m = 0;
int indx = 0;


// message variables
int Header = 255;
int dataLength = 0;
int directionOfData = 0;
int command = 0;
int checkSumm = 0;
int Footer = 205;


uint8_t conf_mode = 0;

unsigned long address  = 0x000001;
unsigned long address1 = 0x000101;
unsigned long address2 = 0x000201;
unsigned long address3 = 0x000301;

unsigned long address_conf_state = 0x000601;