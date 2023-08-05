#include <string.h>	 /* Include string library */
#include <stdio.h>	 /* Include standard library */
#include <stdlib.h>	 /* Include standard library */
#include <stdbool.h> /* Include standard boolean library */
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "sim7600.h"
#include <time.h>

uint32_t timeOut;
uint16_t ATResponseBufferLength;
int8_t Response_Status;
uint32_t lastReceive;
uint16_t Counter;

bool flag_time_out = false;
char latitudestr[14], longitudestr[14], altitudestr[8], datestr[8], timestr[8], speedstr[8], coursestr[4], satNumstr[2];
char N_S_indicator, E_W_indicator;
char net_type[10] = "", net_state[10] = "", mcc_mnc[10] = "", mcc_str[3] = "", mnc_str[2] = "", lac_str[8] = "", cell_id_str[12] = "";

extern unsigned long int longitude, latitude;
extern int16_t course, altitude;
extern uint8_t satNum;
extern float speed;
extern uint64_t nowGPS;

extern uint16_t mcc;
extern uint8_t mnc;
extern uint16_t lac;
extern char cellid[3];

#pragma region UART_REGION
// RX interrupt handler
void on_uart_rx()
{
	while (uart_is_readable(UART_ID_1))
	{
		RESPONSE_BUFFER[Counter] = uart_getc(UART_ID_1); // Copy data to buffer & increment counter
		printf("%c", RESPONSE_BUFFER[Counter]);
		Counter++;
		if (Counter == DEFAULT_BUFFER_SIZE_1)
			Counter = 0;
	}
}

void setup_uart0()
{
	uart_init(UART_ID_1, BAUD_RATE_1);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN_1, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_1, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID_1, BAUD_RATE_1);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID_1, false, false);

    // Set our data format
    uart_set_format(UART_ID_1, DATA_BITS_1, STOP_BITS_1, PARITY_1);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID_1, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID_1 == uart1 ? UART1_IRQ : UART0_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID_1, true, false);
}
#pragma endregion

#pragma region SIM7600_REGION
void Buffer_Flush()
{
	memset(RESPONSE_BUFFER, 0, DEFAULT_BUFFER_SIZE_1);
	Counter = 0;
	ATResponseBufferLength = 0;
	lastReceive = millis();
}

void readResponse(const uint8_t crlfCount)
{
	char CRLF_BUF[2];
	char CRLF_FOUND;
	static bool bufferFull;
	uint16_t ATResponseBufferLength;
	flag_time_out = false;

	while (1)
	{
		if (millis() - lastReceive >= timeOut)
		{
			Response_Status = SIM7600_RESPONSE_TIMEOUT;
			flag_time_out = true;
			return;
		}

		if (Response_Status == SIM7600_RESPONSE_STARTING)
		{
			bufferFull = false;
			CRLF_FOUND = 0;
			memset(CRLF_BUF, 0, 2);
			Response_Status = SIM7600_RESPONSE_WAITING;
		}
		ATResponseBufferLength = strlen(RESPONSE_BUFFER);
		if (ATResponseBufferLength)
		{
			if (ATResponseBufferLength == strlen(RESPONSE_BUFFER))
			{
				for (uint16_t i = 0; i < ATResponseBufferLength; i++)
				{
					memmove(CRLF_BUF, CRLF_BUF + 1, 1);
					CRLF_BUF[1] = RESPONSE_BUFFER[i];
					if (!strncmp(CRLF_BUF, "\r\n", 2))
					{
						if (++CRLF_FOUND == crlfCount)
						{
							Response_Status = SIM7600_RESPONSE_FINISHED;
							return;
						}
					}
				}
				CRLF_FOUND = 0;
			}
		}
	}
}

void startReadResponse(unsigned int _timeOut)
{
	lastReceive = millis();
	Response_Status = SIM7600_RESPONSE_STARTING;
	timeOut = _timeOut;
	// memset(RESPONSE_BUFFER, 0, DEFAULT_BUFFER_SIZE);
}

void waitForResponse(const uint8_t _crlfCount, const unsigned int _timeOut)
{
	uint8_t crlfCount = _crlfCount;
	startReadResponse(_timeOut);
	do
	{
		readResponse(crlfCount);
	} while (Response_Status == SIM7600_RESPONSE_WAITING);
	switch (Response_Status)
	{
	case SIM7600_RESPONSE_FINISHED:
	{
		break;
	}
	case SIM7600_RESPONSE_TIMEOUT:
	{
		break;
	}
	case SIM7600_RESPONSE_ERROR:
	{
		break;
	}
	default:
	{
		break;
	}
	}
}

bool waitForExpectedResponse(const char *expectedResponse, const uint8_t _crlfCount, const unsigned int _timeOut)
{
	//_delay_ms(75);
	memset(RESPONSE_BUFFER, 0, DEFAULT_BUFFER_SIZE_1);
	Counter = 0;
	ATResponseBufferLength = 0;
	waitForResponse(_crlfCount, _timeOut);

	if ((Response_Status != SIM7600_RESPONSE_TIMEOUT) && (strstr(RESPONSE_BUFFER, expectedResponse) != NULL))
	{
		// USART0_SendString(RESPONSE_BUFFER);
		return true;
		// PORTD=0x40;
	}
	else
	{
		// USART0_SendString(RESPONSE_BUFFER);
		//	PORTD=0x00;
		return false;
	}
}

bool sendATCommandAndExpects(char *ATCommand, const char *expectedResponse, const uint8_t _crlfCount, const unsigned int _timeOut)
{
	//	PORTD = 0x40;
	uart_puts(UART_ID_1, ATCommand); /* Send AT command to SIM7600 */
	uart_putc(UART_ID_1, '\r');
	return waitForExpectedResponse(expectedResponse, _crlfCount, _timeOut);
}

void GetResponseBody(char *Response, uint16_t ResponseLength)
{
	uint16_t i = 12;
	char buffer[5];
	while (Response[i] != '\r')
		++i;

	strncpy(buffer, Response + 12, (i - 12));
	ResponseLength = atoi(buffer);

	i += 2;
	uint16_t tmp = strlen(Response) - i;
	memcpy(Response, Response + i, tmp);

	if (!strncmp(Response + tmp - 6, "\r\nOK\r\n", 6))
		memset(Response + tmp - 6, 0, i + 6);
}

bool SIM7600_Init()
{
	uint16_t timeloop = 0;
	uint32_t time_now = 0;
	uart_puts(UART_ID_1, "AT\r");
	time_now = millis();
	while (strstr(RESPONSE_BUFFER, "OK") == NULL)
	{
		/*timeloop++;
		if(timeloop>65000) return false;*/
		if (millis() - time_now > 3000)
		{
			return false;
		}
	}
	Buffer_Flush();
	timeloop = 0;
	// if(waitForExpectedResponse("OK",2,2000)){
	uart_puts(UART_ID_1, "ATE0\r");
	time_now = millis();
	while (strstr(RESPONSE_BUFFER, "OK") == NULL)
	{
		/*timeloop++;
		if(timeloop>65000) return false;*/
		if (millis() - time_now > 3000)
		{
			return false;
		}
	}
	return true;
}

bool initGPS()
{
	uart_puts(UART_ID_1, "AT+CGPS=1\r");
	if (waitForExpectedResponse("OK", 2, DEFAULT_TIMEOUT_1))
		return true;
	else
		return false;
}

bool signalQuality()
{
	uart_puts(UART_ID_1, "AT+CSQ\r");
	if (waitForExpectedResponse("OK", 4, DEFAULT_TIMEOUT_1))
		return true; // check if the second parameter is 2 or 4
	else
	{
		return false;
	}
}

bool simRegistration()
{
	Buffer_Flush();
	uart_puts(UART_ID_1, "AT+CREG?\r");
	if (!waitForExpectedResponse("+CREG: 0,1", 2, DEFAULT_TIMEOUT_1))
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Init_GPRS(char *apn)
{
	uint32_t timeloop = 0;
	uint32_t time_now = 0;
	memset(at_command, 0, sizeof(at_command));
	strcat(at_command, "AT+CGDCONT=1,\"IP\",\"");
	strcat(at_command, apn);
	strcat(at_command, "\"\r");
	printf("%s", at_command);
	printf("%s", "\r\n");
	memset(RESPONSE_BUFFER, 0, DEFAULT_BUFFER_SIZE_1);
	Counter = 0;
	ATResponseBufferLength = 0;
	uart_puts(UART_ID_1, at_command); // USART0_SendString("AT+CGDCONT=1,\"IP\",\"source.syriatel.com\"\r");
	time_now = millis();
	while (strstr(RESPONSE_BUFFER, "OK") == NULL)
	{
		/*timeloop++;
		if(timeloop > 2000000) return false;*/
		if (millis() - time_now > 3000)
		{
			return false;
		}
	}
	timeloop = 0;
	Buffer_Flush();
	uart_puts(UART_ID_1, "AT+CIPTIMEOUT=120000,120000,120000\r");
	if (!waitForExpectedResponse("OK", 2, DEFAULT_TIMEOUT_1))
	{
		return false;
	}
	else
	{
		uart_puts(UART_ID_1, "AT+CIPSENDMODE=1\r");
		if (waitForExpectedResponse("OK", 2, DEFAULT_TIMEOUT_1))
			return true;
		else
			return false;
	}
}

bool closeSockets()
{
	uart_puts(UART_ID_1, "AT+NETCLOSE\r");
	if (waitForExpectedResponse("+NETCLOSE: 0", 4, 16000))
	{
		return true;
	}
	else
	{
		_delay_ms(1000);
		return false;
	}
}

bool is_openSocket()
{
	uint32_t timeloop = 0;
	uint32_t time_now = 0;
	Buffer_Flush();
	uart_puts(UART_ID_1, "AT+NETOPEN\r");
	time_now = millis();
	if (waitForExpectedResponse("+NETOPEN: 0", 4, 16000))
	{
		uart_puts(UART_ID_1, "AT+CIPRXGET=1\r");
		while (strstr(RESPONSE_BUFFER, "OK") == NULL)
		{
			/*timeloop++;
			if(timeloop > 2000000) return false;*/
			if (millis() - time_now > 3000)
			{
				return false;
			}
		}
		_delay_ms(2000);
		return true;
	}
	else
	{
		if (strstr(RESPONSE_BUFFER, "+NETOPEN: 1"))
		{
			_delay_ms(2000);
			return false;
		}
		else
		{
			if (strstr(RESPONSE_BUFFER, "+IP ERROR: Network is already opened"))
			{
				timeloop = 0;
				uart_puts(UART_ID_1, "AT+CIPRXGET=1\r");
				time_now = millis();
				while (strstr(RESPONSE_BUFFER, "OK") == NULL)
				{
					/*timeloop++;
					if(timeloop > 2000000) return false;*/
					if (millis() - time_now)
					{
						return false;
					}
				}
				return true;
			}
			else
			{
				return false;
			}
		}
	}
}

enum SIM7600_RESPONSE_STATUS establish_connection(unsigned char SERVER[], int portcon)
{
	memset(at_command, 0, sizeof(at_command));
	char int_val[6] = "";
	// USART1_TxChar(SERVER[0]); USART1_TxChar(SERVER[1]); USART1_TxChar(SERVER[2]); USART1_TxChar(SERVER[3]);
	strcat(at_command, "AT+CIPOPEN=1,\"TCP\",\"");
	sprintf(int_val, "%d", SERVER[0]);
	strcat(at_command, int_val);
	strcat(at_command, ".");
	sprintf(int_val, "%d", SERVER[1]);
	strcat(at_command, int_val);
	strcat(at_command, ".");
	sprintf(int_val, "%d", SERVER[2]);
	strcat(at_command, int_val);
	strcat(at_command, ".");
	sprintf(int_val, "%d", SERVER[3]);
	strcat(at_command, int_val);
	strcat(at_command, "\",");
	sprintf(int_val, "%d", portcon);
	strcat(at_command, int_val);
	strcat(at_command, "\r");
	// USART1_SendString(at_command);USART1_SendString("\r\n");
	// USART1_SendString("AT+CIPOPEN=1,\"TCP\",\"10.9.195.5\",5232\r");USART1_SendString("\r\n");
	printf("command: %s\n", at_command);
	uart_puts(UART_ID_1, at_command); // USART0_SendString("AT+CIPOPEN=1,\"TCP\",\"10.9.195.5\",5232\r");
	/*USART1_SendString("AT+CIPOPEN=\"TCP\",\"");
	USART1_SendString(SERVER);
	USART1_SendString("\",");
	USART1_SendString(portcon);
	USART1_SendString("\r");*/
	if (waitForExpectedResponse("+CIPOPEN", 4, 6000))
	{ // if (waitForExpectedResponse("+CIPOPEN: 1,0", 4, 6000)){
		//	USART1_SendString("AT+CIPCLOSE?\r");
		// if(waitForExpectedResponse("+CIPCLOSE: 1,0",4,4000))
		// PORTA = 0x01;
		return SIM7600_RESPONSE_FINISHED;
	}
	else
		return SIM7600_RESPONSE_ERROR;
}

int close_socket()
{
	uart_puts(UART_ID_1, "AT+CIPCLOSE?\r");
	if (waitForExpectedResponse("+CIPCLOSE: 0,1,0,0,0,0,0,0,0,0", 4, 60000))
	{
		uart_puts(UART_ID_1, "AT+CIPCLOSE=1\r");
		if (waitForExpectedResponse("+CIPCLOSE: 1,0", 4, 4000))
			return 0;
		else
			return 1;
	}
	else
	{
		return 2;
	}
}

bool close_NET()
{
	uart_puts(UART_ID_1, "AT+NETCLOSE\r");
	if (waitForExpectedResponse("+NETCLOSE: 0", 4, 10000))
		return true;
	else
		return false;
}

bool reset_device()
{
	Buffer_Flush();
	unsigned long timeloop = 0;
	uart_puts(UART_ID_1, "AT+CRESET\r");

	/*while(strstr(RESPONSE_BUFFER,"OK")==NULL){
		timeloop++;
		if(timeloop > 10000000)
		{
			//USART1_SendString("device_reset true\n");
			return false;
		}
	}*/
	if (waitForExpectedResponse("OK", 4, 20000))
	{
		return true;
	}
	else
	{
		return false;
	}
}

enum SIM7600_RESPONSE_STATUS TCPsend(char *data, uint16_t len)
{
	Buffer_Flush();
	uint32_t timeloop = 0;
	uart_puts(UART_ID_1, "AT+CIPSEND=1,");
	uart_puts(UART_ID_1, itoa(len, "%s", 10));
	uart_putc(UART_ID_1, '\r');
	/*if(waitForExpectedResponse(">",1,DEFAULT_TIMEOUT)){
		PORTA = 0x01;

	}*/
	uint32_t time_now = millis();
	bool respres = false;
	while (millis() - time_now < 20000)
	{
		if (!(strstr(RESPONSE_BUFFER, ">") == NULL))
		{
			respres = true;
			break;
		}
	}
	if (respres == false)
	{
		printf("error with >\n");
		return SIM7600_RESPONSE_ERROR;
	}
	Buffer_Flush();
	timeloop = 0;
	for (int i = 0; i < len; i++)
	{
		printf("%s", "h");
		uart_putc(UART_ID_1, data[i]);
	}
	// uart_puts(UART_ID, "\r\n");
	uart_putc(UART_ID_1, 0x1A);
	// PORTA = 0x02;
	time_now = millis();
	respres = false;
	while (millis() - time_now < 20000)
	{
		// printf("i");
		// printf("%s", RESPONSE_BUFFER);
		if (!(strstr(RESPONSE_BUFFER, "+CIPSEND: 1,22,22") == NULL))
		{
			respres = true;
			break;
		}
	}
	if (respres == false)
	{
		printf("error with +CIPSEND\n");
		return SIM7600_RESPONSE_ERROR;
	}
	timeloop = 0;
	Buffer_Flush();
	time_now = millis();
	respres = false;
	while (millis() - time_now < 20000)
	{
		if (!(strstr(RESPONSE_BUFFER, "+CIPRXGET: 1,1") == NULL))
		{
			respres = true;
			break;
		}
	}
	if (respres == false)
	{
		printf("error with +CIPRXGET\n");
		return SIM7600_RESPONSE_ERROR;
	}
	SIM7600_RESPONSE_FINISHED;
}

enum SIM7600_RESPONSE_STATUS TCPsendLocation(char *data, uint16_t len)
{
	uint32_t time_now = 0;
	Buffer_Flush();
	uint32_t timeloop = 0;
	uart_puts(UART_ID_1, "AT+CIPSEND=1,");
	uart_puts(UART_ID_1, itoa(len, "%s", 10));
	uart_putc(UART_ID_1, '\r');
	/*if(waitForExpectedResponse(">",1,DEFAULT_TIMEOUT)){
		PORTA = 0x01;

	}*/
	time_now = millis();
	while (strstr(RESPONSE_BUFFER, ">") == NULL)
	{
		/*timeloop++;
		if(timeloop>2000000){
		flag_time_out = true; return SIM7600_RESPONSE_ERROR;}*/
		if (millis() - time_now > 3000)
		{
			printf("location_err_1\n");
			flag_time_out = true;
			return SIM7600_RESPONSE_ERROR;
		}
	}
	// PORTA = 0x01;
	// USART0_SendString(RESPONSE_BUFFER);
	Buffer_Flush();
	timeloop = 0;
	for (int i = 0; i < len; i++)
	{
		uart_putc(UART_ID_1, data[i]);
	}
	uart_puts(UART_ID_1, "\r\n");
	uart_putc(UART_ID_1, 0x1A);
	if (waitForExpectedResponse("+CIPSEND: 1,43,43", 4, 60000))
	{
		SIM7600_RESPONSE_FINISHED;
	}

	else
	{
		return SIM7600_RESPONSE_ERROR;
	}
}

enum SIM7600_RESPONSE_STATUS TCPsendHeartbeat(char *data, uint16_t len)
{
	uint32_t time_now = 0;
	Buffer_Flush();
	uint32_t timeloop = 0;
	uart_puts(UART_ID_1, "AT+CIPSEND=1,");
	uart_puts(UART_ID_1, itoa(len, "%s", 10));
	uart_putc(UART_ID_1, '\r');
	/*if(waitForExpectedResponse(">",1,DEFAULT_TIMEOUT)){
		PORTA = 0x01;

	}*/
	time_now = millis();
	while (strstr(RESPONSE_BUFFER, ">") == NULL)
	{
		/*timeloop++;
		if(timeloop>2000000){
		flag_time_out = true; return SIM7600_RESPONSE_ERROR;}*/
		if (millis() - time_now > 3000)
		{
			printf("heartbeat_err_1\n");
			flag_time_out = true;
			return SIM7600_RESPONSE_ERROR;
		}
	}
	Buffer_Flush();
	timeloop = 0;
	for (int i = 0; i < len; i++)
	{
		uart_putc(UART_ID_1, data[i]);
	}
	uart_puts(UART_ID_1, "\r\n");
	uart_putc(UART_ID_1, 0x1A);
	// PORTA = 0x02;
	if (waitForExpectedResponse("+CIPSEND: 1,15,15", 4, 60000))
	{
		timeloop = 0;
		time_now = millis();
		Buffer_Flush();
		while (strstr(RESPONSE_BUFFER, "+CIPRXGET: 1,1") == NULL)
		{
			/*timeloop++;
			if(timeloop>9000000){
			flag_time_out = true; return SIM7600_RESPONSE_ERROR;}*/
			if (millis() - time_now > 20000)
			{
				printf("heartbeat_err_2\n");
				flag_time_out = true;
				return SIM7600_RESPONSE_ERROR;
			}
		}
		SIM7600_RESPONSE_FINISHED;
	}

	else
	{
		return SIM7600_RESPONSE_ERROR;
	}
}

enum SIM7600_RESPONSE_STATUS TCPsendAlarm(char *data, uint16_t len)
{
	uint8_t time_now = 0;
	Buffer_Flush();
	uint32_t timeloop = 0;
	uart_puts(UART_ID_1, "AT+CIPSEND=1,");
	uart_puts(UART_ID_1, itoa(len, "%s", 10));
	uart_putc(UART_ID_1, '\r');
	/*if(waitForExpectedResponse(">",1,DEFAULT_TIMEOUT)){
		PORTA = 0x01;

	}*/
	time_now = millis();
	while (strstr(RESPONSE_BUFFER, ">") == NULL)
	{
		/*timeloop++;
		if(timeloop>2000000){
		flag_time_out = true; return SIM7600_RESPONSE_ERROR;}*/
		if (millis() - time_now > 3000)
		{
			printf("alarm_err_1\n");
			flag_time_out = true;
			return SIM7600_RESPONSE_ERROR;
		}
	}
	Buffer_Flush();
	timeloop = 0;
	for (int i = 0; i < len; i++)
	{
		uart_putc(UART_ID_1, data[i]);
	}
	uart_puts(UART_ID_1, "\r\n");
	uart_putc(UART_ID_1, 0x1A);
	if (waitForExpectedResponse("+CIPSEND: 1,46,46", 4, 60000))
	{
		timeloop = 0;
		time_now = millis();
		Buffer_Flush();
		while (strstr(RESPONSE_BUFFER, "+CIPRXGET: 1,1") == NULL)
		{
			/*timeloop++;
			if(timeloop>2000000){
			flag_time_out = true; return SIM7600_RESPONSE_ERROR;}*/
			if (millis() - time_now > 20000)
			{
				printf("alarm_err_2\n");
				flag_time_out = true;
				return SIM7600_RESPONSE_ERROR;
			}
		}
		SIM7600_RESPONSE_FINISHED;
	}

	else
	{
		return SIM7600_RESPONSE_ERROR;
	}
}

enum SIM7600_RESPONSE_STATUS getDataManually()
{
	uint32_t timeloop = 0;
	uint32_t time_now = 0;
	Buffer_Flush();
	uart_puts(UART_ID_1, "AT+CIPRXGET=3,1,50\r");
	while (strstr(RESPONSE_BUFFER, "OK") == NULL)
	{
		/*timeloop++;
		if(timeloop > 10000000) return SIM7600_RESPONSE_ERROR;*/
		if (millis() - time_now > 3000)
		{
			printf("getDataManually_err_1\n");
			flag_time_out = true;
			return SIM7600_RESPONSE_ERROR;
		}
	}
	GetResponseBody(RESPONSE_BUFFER, 50);
	return SIM7600_RESPONSE_FINISHED;
}

unsigned long convert_to_degrees(char *raw)
{

	double value;
	float decimal_value, temp;
	float val_end = 0.0;
	uint32_t degrees;
	unsigned long position;
	if (raw != NULL)
	{
		value = atof(raw);
		/* convert raw latitude/longitude into degree format */
		decimal_value = (value / 100);
		degrees = (int)(decimal_value);
		val_end = (decimal_value - degrees) / 0.6;
		position = (degrees + val_end) * 1800000; // position = position * 10000000;
		return position;
	}
	else
		return 0;
}

uint64_t get_timeStamp(char date[], char time[])
{
	struct tm t = {0};

	time_t t1, t2;
	uint32_t Time_value, Date_value;
	unsigned int hour, min, sec, day, mon, year;
	if (timestr != NULL)
	{
		Time_value = atol(time);		  // convert string to integer
		hour = (Time_value / 10000);	  // extract hour from integer
		min = (Time_value % 10000) / 100; // extract minute from integer
		sec = (Time_value % 10000) % 100; // extract second from integer
		curr_time[0] = (uint8_t)((hour + 3) % 24);
		curr_time[1] = (uint8_t)min;
		curr_time[2] = (uint8_t)sec;

		Date_value = atol(date); /* convert string to integer */
		//	Date_value = 70122;
		day = (Date_value / 10000);				  /* extract day from integer */
		mon = (Date_value % 10000) / 100;		  /* extract month from integer */
		year = (Date_value % 10000) % 100 + 2000; /* extract year from integer */
		curr_time[3] = (uint8_t)day;
		curr_time[4] = (uint8_t)mon;
		curr_time[5] = (uint8_t)(year - 2000);

		t.tm_year = year - 1900;
		t.tm_mon = mon - 1;
		t.tm_mday = day;
		t.tm_hour = hour;
		t.tm_min = min;
		t.tm_sec = sec;
		t.tm_isdst = 1; // Is DST on? 1 = yes, 0 = no, -1 = unknown
		t1 = mktime(&t);
		t.tm_year = 1970 - 1900;
		t.tm_mon = 0;
		t.tm_mday = 0;
		t.tm_hour = 0;
		t.tm_min = 0;
		t.tm_sec = 0;
		t.tm_isdst = 1; // Is DST on? 1 = yes, 0 = no, -1 = unknown
		t2 = mktime(&t);
		// now = 1641475500;
		// sprintf(now70,"%d",now);
		nowGPS = ((uint64_t)difftime(t1, t2)) * 1000;
		return nowGPS;
	}
	else
		nowGPS = 3600;
}

void getGPSINFO()
{
	uint8_t i = 0, j = 0;
	uart_puts(UART_ID_1, "AT+CGNSSINFO\r");
	waitForExpectedResponse("OK", 4, DEFAULT_TIMEOUT_1);
	for (i = 0; i < 100; i++)
	{
		if (RESPONSE_BUFFER[i] != ':')
			continue;
		else
		{
			i++;
			break;
		}
	}
	while (RESPONSE_BUFFER[i] != ',')
	{
		satNumstr[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	if (satNumstr != NULL)
		satNum = atoi(satNumstr);
	else
		satNum = 0;
	i = i + 10;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		latitudestr[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	i++;
	j = 0;
	if (latitudestr != NULL)
		latitude = convert_to_degrees(latitudestr);
	else
		latitude = 0;
	N_S_indicator = RESPONSE_BUFFER[i];
	i = i + 2;
	while (RESPONSE_BUFFER[i] != ',')
	{
		longitudestr[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	if (longitudestr != NULL)
		longitude = convert_to_degrees(longitudestr);
	else
		longitude = 0;
	i++;
	j = 0;
	E_W_indicator = RESPONSE_BUFFER[i];
	i = i + 2;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		datestr[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	i++;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		timestr[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	get_timeStamp(datestr, timestr);
	i++;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		altitudestr[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	if (altitudestr != NULL)
		altitude = atoi(altitude);
	else
		altitude = 0;
	i++;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		speedstr[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	if (speedstr != NULL)
		speed = atoi(speedstr) * 1.852;
	else
		speed = 0;
	i++;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		coursestr[j] = RESPONSE_BUFFER[i];
		i++;
	}
	if (coursestr != NULL)
		course = atof(coursestr);
	else
		course = 0;
}

void getGPRSINFO()
{
	uint8_t i = 0, j = 0;
	uart_puts(UART_ID_1, "AT+CPSI?\r");
	waitForExpectedResponse("OK", 4, DEFAULT_TIMEOUT_1);
	for (i = 0; i < 100; i++)
	{
		if (RESPONSE_BUFFER[i] != ':')
			continue;
		else
		{
			i++;
			break;
		}
	}
	while (RESPONSE_BUFFER[i] != ',')
	{
		net_type[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	i++;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		net_state[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	i++;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		mcc_mnc[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	int t = 0;
	uint8_t aa = 0;
	uint8_t ind = 0;
	for (t = 0; t < sizeof(mcc_mnc); t++)
	{
		if (aa == 0)
		{
			mcc_str[ind] = mcc_mnc[t];
			ind++;
		}
		else
		{
			mnc_str[ind] = mcc_mnc[t];
			ind++;
		}
		if (mcc_mnc[t] == '-')
		{
			ind = 0;
			aa = 1;
		}
	}
	i++;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		lac_str[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}
	i++;
	j = 0;
	while (RESPONSE_BUFFER[i] != ',')
	{
		cell_id_str[j] = RESPONSE_BUFFER[i];
		i++;
		j++;
	}

	char lac_ss[10] = "";
	mnc = strtol(mnc_str, NULL, 10);
	lac = strtol(lac_str, NULL, 16);
	uint64_t c_id = atol(cell_id_str);
	cellid[0] = (c_id & 0xff);
	cellid[1] = ((c_id >> 8) & 0xff);
	cellid[2] = ((c_id >> 16) & 0xff);
}

#pragma endregion