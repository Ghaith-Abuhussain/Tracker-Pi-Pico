#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "convertToHexFunctions.h"
#include "configurator_variables.h"
#include "uart1.h"
#include "eeprom_i2c.h"

void serialEvent();


void refresh_data()
{
	parserheader = 0;
	parserdataLength = 0;
	parserdirectionOfData = 0;
	memset(parserdataMessage, 0, sizeof(parserdataMessage));
	parsercommand = 0;
	parserdate.days = 0;
	parserdate.hours = 0;
	parserdate.seconds = 0;
	parserdate.minutes = 0;
	parserdate.months = 0;
	parserdate.years = 0;
	parsercheckSum = 0;
	parserfooter = 0;
}

void parseMessage()
{
	refresh_data();
	parserheader = ((unsigned char) message_to_server[0]);
	parserdataLength = parserdataLength = (unsigned char) message_to_server[2] + 256*((unsigned char) message_to_server[1]);
	parserdirectionOfData = ((unsigned char) message_to_server[3]);
	parserdate.years = (unsigned char) message_to_server[4];
	parserdate.months = (unsigned char) message_to_server[5];
	parserdate.days = (unsigned char) message_to_server[6];
	parserdate.hours = (unsigned char) message_to_server[7];
	parserdate.minutes = (unsigned char) message_to_server[8];
	parserdate.seconds = (unsigned char) message_to_server[9];
	parsercommand = ((unsigned char) message_to_server[11]) + ((unsigned char) message_to_server[10])*256;
	for(i = 0; i < parserdataLength; i++)
	{
		parserdataMessage[i] = message_to_server[12 + i];
	}
	parsercheckSum = (((unsigned char) message_to_server[12 + parserdataLength + 1])  + ((unsigned char) message_to_server[12 + parserdataLength])*256);
	parserfooter   = (unsigned char) message_to_server[12 + parserdataLength + 2];
}

void print_non_io_public_vars()
{
	sprintf(arrayNum, "%d", gprsContext); UART_SendString(arrayNum,3);
	sprintf(arrayNum, "%d", gprsAuthentication); UART_SendString(arrayNum,3);
	sprintf(arrayNum, "%d", APNLen); UART_SendString(arrayNum,3);
	sprintf(arrayNum, "%d", APN_UsernameLen); UART_SendString(arrayNum,3);
	sprintf(arrayNum, "%d", APN_PasswordLen); UART_SendString(arrayNum,3);
	UART_SendString(APN, APNLen);
	UART_SendString(APN_Username, APN_UsernameLen);
	UART_SendString(APN_Password, APN_PasswordLen);
}

void print_non_io_public_vars_coll_1()
{
	indx = 0;
	// ioEventsArray
	for(int i = 0; i < 12; i++)
	{
		sprintf(arrayNum, "%d\n", ioEventsArray[i].priority); UART_SendString(arrayNum,10);
		sprintf(arrayNum, "%d\n", ioEventsArray[i].lowLevel); UART_SendString(arrayNum,10);
		sprintf(arrayNum, "%d\n", ioEventsArray[i].highLevel); UART_SendString(arrayNum,10);
		sprintf(arrayNum, "%d\n", ioEventsArray[i].eventsOnly); UART_SendString(arrayNum,10);
		sprintf(arrayNum, "%d\n", ioEventsArray[i].operand); UART_SendString(arrayNum,10);
		sprintf(arrayNum, "%d\n", ioEventsArray[i].avgConstant); UART_SendString(arrayNum,10);
		sprintf(arrayNum, "%d\n", ioEventsArray[i].sendSmsToLength); UART_SendString(arrayNum,10);
		UART_SendString(ioEventsArray[i].sendSmsTo,10);
	}
}

void dataIntoPublicVars1(unsigned char data[])
{
	indx = 0;
	memset(APN, 0, sizeof(APN));
	memset(APN_Username, 0, sizeof(APN_Username));
	memset(APN_Password, 0, sizeof(APN_Password));
	// APN Variables
	gprsContext =  (unsigned char) data[indx]; indx += 1;
	gprsAuthentication = (unsigned char) data[indx];indx += 1;
	APNLen = (unsigned char) data[indx]; indx += 1;
	APN_UsernameLen = (unsigned char) data[indx]; indx += 1;
	APN_PasswordLen = (unsigned char) data[indx]; indx += 1;
	for(i = 0; i < APNLen; i++)
	{
		APN[i] = data[indx]; indx += 1;
	}
	for(i = 0; i < APN_UsernameLen; i++)
	{
		APN_Username[i] = data[indx]; indx += 1;
	}
	for(i = 0; i < APN_PasswordLen; i++)
	{
		APN_Password[i] = data[indx]; indx += 1;
	}

	// data Aquisition Periods Variables
	dataAqOnStopVars[0] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnStopVars[1] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnStopVars[2] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnStopVars[3] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnStopVars[4] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnStopVars[5] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnStopVars[6] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnStopVars[7] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnStopVars[8] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	
	dataAqMovingVars[0] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[1] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[2] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[3] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[4] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[5] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[6] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[7] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[8] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[9] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[10] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[11] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[12] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[13] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[14] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[15] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[16] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqMovingVars[17] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	
	dataAqOnDemandTrackingVars[0] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	dataAqOnDemandTrackingVars[1] = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;

	// main Server Setting
	mainServerIp[0] = (unsigned char) data[indx]; indx += 1;
	mainServerIp[1] = (unsigned char) data[indx]; indx += 1;
	mainServerIp[2] = (unsigned char) data[indx]; indx += 1;
	mainServerIp[3] = (unsigned char) data[indx]; indx += 1;
	
	mainServerPort = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	mainServerProtocol = (unsigned char) data[indx]; indx += 1;
	mainServerTlsEncryption = (unsigned char) data[indx]; indx += 1;

	// second Server Setting
	secondServerMode = (unsigned char) data[indx]; indx += 1;
	secondServerIp[0] = (unsigned char) data[indx]; indx += 1;
	secondServerIp[1] = (unsigned char) data[indx]; indx += 1;
	secondServerIp[2] = (unsigned char) data[indx]; indx += 1;
	secondServerIp[3] = (unsigned char) data[indx]; indx += 1;
	
	secondServerPort = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	secondServerProtocol = (unsigned char) data[indx]; indx += 1;
	secondServerTlsEncryption = (unsigned char) data[indx]; indx += 1;

	// recordTimeoutSetting
	openLinkTimeout = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	responseTimeout = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	networkPingTimeout = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
	sortBy = (unsigned char) data[indx]; indx += 1;
	recordTlsEncryption = (unsigned char) data[indx]; indx += 1;
}

void dataIntoPublicVars2(unsigned char data[])
{
	indx = 0;
	// ioEventsArray
	for(i = 0; i < 12; i++)
	{
		ioEventsArray[i].priority  = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].lowLevel  = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].highLevel = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].eventsOnly = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].operand = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].avgConstant = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].sendSmsToLength = (unsigned char) data[indx]; indx += 1;
		memset(ioEventsArray[i].sendSmsTo, 0, sizeof(ioEventsArray[i].sendSmsTo));
		for(k = 0; k < ioEventsArray[i].sendSmsToLength; k++)
		{
			ioEventsArray[i].sendSmsTo[k] = data[indx]; indx += 1;
		}
	}
}

void dataIntoPublicVars3(unsigned char data[])
{
	indx = 0;
	// ioEventsArray
	for(i = 12; i < sizeof_ioEventsArray; i++)
	{
		ioEventsArray[i].priority  = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].lowLevel  = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].highLevel = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].eventsOnly = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].operand = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].avgConstant = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].sendSmsToLength = (unsigned char) data[indx]; indx += 1;
		memset(ioEventsArray[i].sendSmsTo, 0, sizeof(ioEventsArray[i].sendSmsTo));
		for(k = 0; k < ioEventsArray[i].sendSmsToLength; k++)
		{
			ioEventsArray[i].sendSmsTo[k] = data[indx]; indx += 1;
		}
	}
}

void dataIntoPublicVars4(unsigned char data[])
{
	indx = 0;
	// ioEventsArray
	for(i = 24; i < sizeof_ioEventsArray; i++)
	{
		ioEventsArray[i].priority  = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].lowLevel  = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].highLevel = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].eventsOnly = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].operand = (unsigned char) data[indx]; indx += 1;
		ioEventsArray[i].avgConstant = ((unsigned char) data[indx])*256 + (unsigned char) data[indx + 1]; indx += 2;
		ioEventsArray[i].sendSmsToLength = (unsigned char) data[indx]; indx += 1;
		//UART_SendString("9", 1);
		memset(ioEventsArray[i].sendSmsTo, 0, sizeof(ioEventsArray[i].sendSmsTo));
		for(k = 0; k < ioEventsArray[i].sendSmsToLength; k++)
		{
			ioEventsArray[i].sendSmsTo[k] = data[indx]; indx += 1;
		}
	}
}

// gen_error_messages_to_server
void gen_error_infoType_onCalling_VarValue()
{
	// infoType
	dataLength = 19;
	directionOfData = 0;
	command = 0;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x01 connection
	to_String_X2(1);
	strcat(message_to_server,value_2);  // data errorType
	to_String_X2((int) infoType);
	strcat(message_to_server,value_2);  // date infoType
	to_String_X2((int) 'N');
	strcat(message_to_server,value_2);  // data message "not valid infoType"
	to_String_X2((int) 'O');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'T');
	strcat(message_to_server,value_2);
	to_String_X2((int) ' ');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'V');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'A');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'I');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'D');
	strcat(message_to_server,value_2);
	to_String_X2((int) ' ');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'I');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'N');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'F');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'O');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'T');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'Y');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'P');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'E');
	strcat(message_to_server,value_2);
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}

void gen_error_commandType_onUpdatingVars()
{
	// infoType
	dataLength = 18;
	directionOfData = 0;
	command = 0;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x01 connection
	to_String_X2(2);
	strcat(message_to_server,value_2);  // data errorType
	to_String_X2((int) parsercommand);
	strcat(message_to_server,value_2);  // date parsercommand
	to_String_X2((int) 'N');
	strcat(message_to_server,value_2);  // data message "not valid command"
	to_String_X2((int) 'O');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'T');
	strcat(message_to_server,value_2);
	to_String_X2((int) ' ');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'V');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'A');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'I');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'D');
	strcat(message_to_server,value_2);
	to_String_X2((int) ' ');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'C');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'O');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'M');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'M');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'A');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'N');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'D');
	strcat(message_to_server,value_2);
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}

// gen_info_message_to_server
void gen_connection_message()
{
	dataLength = 6;
	directionOfData = 0;
	command = 1;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x01 connection
	to_String_X2(0x53);
	strcat(message_to_server,value_2);  // dataType "SCTR01"
	to_String_X2(0x43);
	strcat(message_to_server,value_2);
	to_String_X2((int) 'T');
	strcat(message_to_server,value_2);
	to_String_X2((int) 'R');
	strcat(message_to_server,value_2);
	to_String_X2((int) '0');
	strcat(message_to_server,value_2);
	to_String_X2((int) '1');
	strcat(message_to_server,value_2);
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}

void genClearMemory_message()
{
	dataLength = 0;
	directionOfData = 0;
	command = 11;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR014
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}

void genNonIoVariables_message()
{
	dataLength = 0;
	directionOfData = 0;
	command = 10;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}

void genIoVariablesFirstCollection_message()
{
	dataLength = 0;
	directionOfData = 0;
	command = 12;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}

void genIoVariablesSecondCollection_message()
{
	dataLength = 0;
	directionOfData = 0;
	command = 13;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}

void genIoVariablesThirdCollection_message()
{
	dataLength = 0;
	directionOfData = 0;
	command = 14;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}

void getIoVarsZeroCollection_message()
{
	dataLength = 255;
	directionOfData = 0;
	command = 18;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
}

void getIoVarsFirstCollection_message()
{
	dataLength = 255;
	directionOfData = 0;
	command = 15;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
}

void getIoVarsSecondCollection_message()
{
	dataLength = 255;
	directionOfData = 0;
	command = 16;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
}

void getIoVarsThirdCollection_message()
{
	dataLength = 255;
	directionOfData = 0;
	command = 17;
	checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X2(255);
	strcat(message_to_server, value_2);    // Header
	to_String_X4(dataLength);
	strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
	to_String_X2(directionOfData);
	strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
	to_String_X2(parserdate.years);
	strcat(message_to_server, value_2);    // time years
	to_String_X2(parserdate.months);
	strcat(message_to_server, value_2);    // time months
	to_String_X2(parserdate.days);
	strcat(message_to_server, value_2);    // time days
	to_String_X2(parserdate.hours);
	strcat(message_to_server, value_2);    // time hours
	to_String_X2(parserdate.minutes);
	strcat(message_to_server, value_2);    // time minuts
	to_String_X2(parserdate.seconds);
	strcat(message_to_server, value_2);    // time seconds
	to_String_X4(command);
	strcat(message_to_server, value_4);    // command 0x0b connection
}

void genCheckSumFooter()
{
	memset(message_to_server, 0, sizeof(message_to_server));
	to_String_X4(checkSumm);
	strcat(message_to_server,value_4);
	to_String_X2(Footer);
	strcat(message_to_server,value_2);
}