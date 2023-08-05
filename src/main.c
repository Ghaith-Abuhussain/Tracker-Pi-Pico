#include "pico/stdlib.h"
#include "sim7600.h"
#include <stdint.h>
#include "millis.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <inttypes.h>
#include "pico/binary_info.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "configurator_functions.h"
#include "hardware/resets.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include "pico/stdlib.h"

#pragma region Variables_Declaration

/// \tag::uart_advanced[]
char RESPONSE_BUFFER[DEFAULT_BUFFER_SIZE_1];
char curr_time[6];
char at_command[128];

unsigned long int longitude, latitude;
int16_t course, altitude;
uint8_t satNum;
float speed;
uint64_t nowGPS;
uint16_t mcc;
uint8_t mnc;
uint16_t lac;
char cellid[3];

#define MAX_SEND_TRIES 3

bool login_send = false;
struct tm  timeNow;
int checkSum_index = 0;
char value_1;
bool flag_fix = false;
bool gprsEnabled = false;
bool simreg = false;
bool enable = false;
bool flag_connection = false;
bool openNet = false;
uint8_t start=0;
uint8_t sendimei = 0;
uint8_t response_imei = 0;
uint8_t response_data = 0;
uint8_t counter_sec = 0;
bool flag_reg;

char long_val_str[12] = "";

unsigned int low, high;
unsigned long int lowl, highl;

unsigned char year   = 0x16;
unsigned char month  = 0x04;
unsigned char day    = 0x15;
unsigned char hour   = 0x06;
unsigned char minute = 0x30;
unsigned char second = 0x33;
char curr_time[6] = "";
char at_command[128] = "";
int send_cntr = 0;

unsigned char lon_lat_index = 0;

uint64_t nowGPS = 0x00000000;


int check_sum = 0;

unsigned char IMEI[8];

uint16_t mcc = 417;
uint8_t mnc = 1;
uint16_t lac = 10000;
char cellid[3] = {0x00, 0x1f, 0x71};

char value_data[256];  //1135 for 8 packet
unsigned long int longitude, latitude;
int16_t course, altitude;
uint8_t satNum;
float speed;
uint64_t nowGPS;

uint8_t cong_on   = 0;

bool sos_alarm = false;
int serial_number = 0;
int send_tries = 0;

volatile uint8_t tot_overflow;
unsigned char timer_status = 0;
unsigned char over_flow_flag = 0;
uint16_t offline_timing = 30;
uint16_t offlline_counter = 0;
int num_sent_offline = 3;

#define byte uint8_t
#define word uint16_t
byte readed_buffer[64] = {0x00};
int j = 0;

struct repeating_timer timer;

char server_ip[4] = "";
#pragma endregion

#pragma region Function_Declaration

void genLoginMessage(unsigned char imei_num[], int model_id_code, int time_lang, int serial_number);

void genHeartbeatMessage(unsigned char terminal_info_content
						, int external_voltage
						, unsigned char buildin_battary_level
						, unsigned char gsm_signal_strength
						, int lang_status
						, int serial_number);

void genLocationPackage(int quantity_satalite,
						long latitude,
						long longitude,
						unsigned char speed,
						unsigned int course_status,
						unsigned int mcc,
						unsigned char mnc,
						unsigned int lac,
						char cellid[],
						unsigned char acc,
						unsigned char data_upload_mode,
						unsigned char gps_real_upload,
						long mileage,
						int serial_number);

void genAlarmPackage(int quantity_satalite,
						long latitude,
						long longitude,
						unsigned char speed,
						unsigned int course_status,
						int lbs_length,
						unsigned int mcc,
						unsigned char mnc,
						unsigned int lac,
						char cellid[],
						unsigned char terminal_info,
						unsigned char build_in_battery_level,
						unsigned char gsm_signal_strength,
						int alarm_language,
						long mileage,
						int serial_number);

void i2c_initialization();

void copy_data(char dist[], char source[], int begin,int len);

void strcat_data(char dist[], char source[], int begin, int len);

void read_multiple_data(byte data[], int num_blocks,unsigned long address);

void write_multiple_data(byte data[], int num_blocks,unsigned long address);

void read_eeprom_to_public_vars();

void software_reset();

void serialEvent();

// RX interrupt handler
void on_uart_rx1();

void setup_uart1();

void setIMEI();

void setTime(char current_time[]);

bool repeating_timer_callback(struct repeating_timer *t);

bool sendToserver(int msgType);

#pragma endregion

#pragma region generate_messages

void genLoginMessage(unsigned char imei_num[], int model_id_code, int time_lang, int serial_number)
{
	memset(value_data, 0, sizeof(value_data));
	to_String_X4(0x7878);
	strcat(value_data, value_4); // 2 (start bit)
	to_String_X2(0x11);
	strcat(value_data, value_2); // 1 (packet length)
	to_String_X2(0x01);
	strcat(value_data, value_2); // 1 (protocol number)
	for(k = 0; k < 8; k++)
	{
		to_String_X2(imei_num[k]);
		strcat(value_data, value_2); // 1 (IMEI[k])
	}
	to_String_X4(model_id_code);
	strcat(value_data, value_4); // 4 (model identification code)
	to_String_X4(time_lang);
	strcat(value_data, value_4); // 4 (timezone language)
	to_String_X4(serial_number);
	strcat(value_data, value_4); // 4 (serial number language)
	to_String_X4(0x00);
	strcat(value_data, value_4); // 4 (CheckSum)
	to_String_X4(0x0d0a);
	strcat(value_data, value_4); // 4 (stop bit)
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(17 + 5);i+=2,r++){
		value_data[r] = (char)(convertCharToHex(value_data[i])<<4)|convertCharToHex(value_data[i+1]);
		checkSum_index++;
	}
	check_sum = GetCrc16(value_data, checkSum_index);
	
	memset(value_data, 0, sizeof(value_data));
	to_String_X4(0x7878);
	strcat(value_data, value_4); // 2 (start bit)
	to_String_X2(0x11);
	strcat(value_data, value_2); // 1 (packet length)
	to_String_X2(0x01);
	strcat(value_data, value_2); // 1 (protocol number)
	for(k = 0; k < 8; k++)
	{
		to_String_X2(imei_num[k]);
		strcat(value_data, value_2); // 1 (IMEI[k])
	}
	to_String_X4(model_id_code);
	strcat(value_data, value_4); // 4 (model identification code)
	to_String_X4(time_lang);
	strcat(value_data, value_4); // 4 (timezone language)
	to_String_X4(serial_number);
	strcat(value_data, value_4); // 4 (serial number language)
	to_String_X4(check_sum);
	strcat(value_data, value_4); // 4 (CheckSum)
	to_String_X4(0x0d0a);
	strcat(value_data, value_4); // 4 (stop bit)
	
	// Convert LOGIN pakage to bytes array depending on the length = 17 + 5
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(17 + 5);i+=2,r++){
		value_data[r] = (char)(convertCharToHex(value_data[i])<<4)|convertCharToHex(value_data[i+1]);
		checkSum_index++;
	}
}

void genHeartbeatMessage(unsigned char terminal_info_content
						, int external_voltage
						, unsigned char buildin_battary_level
						, unsigned char gsm_signal_strength
						, int lang_status
						, int serial_number)
{
	memset(value_data, 0, sizeof(value_data));
	to_String_X4(0x7878);
	strcat(value_data, value_4); // 2 (start bit)
	to_String_X2(0x0c);
	strcat(value_data, value_2); // 1 (packet length)
	to_String_X2(0x13);
	strcat(value_data, value_2); // 1 (protocol number)
	to_String_X2(terminal_info_content);
	strcat(value_data, value_2); // 1 (Terminal Information Content)
	//to_String_X4(external_voltage);
	//strcat(value_data, value_4); // 2 (external voltage)
	to_String_X2(buildin_battary_level);
	strcat(value_data, value_2); // 1 (build in battery voltage level)
	to_String_X2(gsm_signal_strength);
	strcat(value_data, value_2); // 1 (gsm signal strength)
	to_String_X4(lang_status);
	strcat(value_data, value_4); // 4 (language/extended port status)
	to_String_X4(serial_number);
	strcat(value_data, value_4); // 4 (serial number language)
	to_String_X4(0x00);
	strcat(value_data, value_4); // 4 (CheckSum)
	to_String_X4(0x0d0a);
	strcat(value_data, value_4); // 4 (stop bit)
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(12 + 5);i+=2,r++){
		value_data[r] = (char)(convertCharToHex(value_data[i])<<4)|convertCharToHex(value_data[i+1]);
		checkSum_index++;
	}
	check_sum = GetCrc16(value_data, checkSum_index);
	
	memset(value_data, 0, sizeof(value_data));
	to_String_X4(0x7878);
	strcat(value_data, value_4); // 2 (start bit)
	to_String_X2(0x0c);
	strcat(value_data, value_2); // 1 (packet length)
	to_String_X2(0x13);
	strcat(value_data, value_2); // 1 (protocol number)
	to_String_X2(terminal_info_content);
	strcat(value_data, value_2); // 1 (Terminal Information Content)
	//to_String_X4(external_voltage);
	//strcat(value_data, value_4); // 2 (external voltage)
	to_String_X2(buildin_battary_level);
	strcat(value_data, value_2); // 1 (build in battery voltage level)
	to_String_X2(gsm_signal_strength);
	strcat(value_data, value_2); // 1 (gsm signal strength)
	to_String_X4(lang_status);
	strcat(value_data, value_4); // 4 (language/extended port status)
	to_String_X4(serial_number);
	strcat(value_data, value_4); // 4 (serial number language)
	to_String_X4(check_sum);
	strcat(value_data, value_4); // 4 (CheckSum)
	to_String_X4(0x0d0a);
	strcat(value_data, value_4); // 4 (stop bit)
	
	// Convert HeartBeat Package To Byte Array Depending On Length = 12 + 5
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(12 + 5);i+=2,r++){
		value_data[r] = (char)(convertCharToHex(value_data[i])<<4)|convertCharToHex(value_data[i+1]);
		checkSum_index++;
	}
}

void genLocationPackage(int quantity_satalite,
						long latitude,
						long longitude,
						unsigned char speed,
						unsigned int course_status,
						unsigned int mcc,
						unsigned char mnc,
						unsigned int lac,
						char cellid[],
						unsigned char acc,
						unsigned char data_upload_mode,
						unsigned char gps_real_upload,
						long mileage,
						int serial_number)
{
	memset(value_data, 0, sizeof(value_data));
	to_String_X4(0x7878);
	strcat(value_data, value_4); // 2 (start bit)
	to_String_X2(0x26);
	strcat(value_data, value_2); // 1 (packet length)
	to_String_X2(0x22);
	strcat(value_data, value_2); // 1 (protocol number)
	to_String_X2(year);
	strcat(value_data, value_2); // 1 (time year)
	to_String_X2(month);
	strcat(value_data, value_2); // 1 (time month)
	to_String_X2(day);
	strcat(value_data, value_2); // 1 (time day)
	to_String_X2(hour);
	strcat(value_data, value_2); // 1 (time hour)
	to_String_X2(minute);
	strcat(value_data, value_2); // 1 (time minute)
	to_String_X2(second);
	strcat(value_data, value_2); // 1 (time second)
	to_String_X2(quantity_satalite);
	strcat(value_data, value_2); // 1 (quantity of satellites)
	to_String_X8(latitude);
	strcat(value_data, value_8); // 4 (latitude)
	to_String_X8(longitude);
	strcat(value_data, value_8); // 4 (longitude)
	to_String_X2(speed);
	strcat(value_data, value_2); // 1 (speed)
	to_String_X4(course_status);
	strcat(value_data, value_4); // 2 (course_status)
	to_String_X4(mcc);
	strcat(value_data, value_4); // 2 (mcc)
	to_String_X2(mnc);
	strcat(value_data, value_2); // 1 (mnc)
	to_String_X4(lac);
	strcat(value_data, value_4); // 2 (lac)
	to_String_X2(cellid[0]);
	strcat(value_data, value_2); // 1 (cellid 1st byte)
	to_String_X2(cellid[1]);
	strcat(value_data, value_2); // 1 (cellid 2nd byte)
	to_String_X2(cellid[2]);
	strcat(value_data, value_2); // 1 (cellid 3rd byte)
	to_String_X2(acc);
	strcat(value_data, value_2); // 1 (acc)
	to_String_X2(data_upload_mode);
	strcat(value_data, value_2); // 1 (data upload mode)
	to_String_X2(gps_real_upload);
	strcat(value_data, value_2); // 1 (gps real time upload mode)
	to_String_X8(mileage);
	strcat(value_data, value_8); // 4 (mileage)
	to_String_X4(serial_number);
	strcat(value_data, value_4); // 2 (serial number language)
	to_String_X4(0x00);
	strcat(value_data, value_4); // 2 (CheckSum)
	to_String_X4(0x0d0a);
	strcat(value_data, value_4); // 2 (stop bit)
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(38 + 5);i+=2,r++){
		value_data[r] = (char)(convertCharToHex(value_data[i])<<4)|convertCharToHex(value_data[i+1]);
		checkSum_index++;
	}
	check_sum = GetCrc16(value_data, checkSum_index);
	
	memset(value_data, 0, sizeof(value_data));
	to_String_X4(0x7878);
	strcat(value_data, value_4); // 2 (start bit)
	to_String_X2(0x26);
	strcat(value_data, value_2); // 1 (packet length)
	to_String_X2(0x22);
	strcat(value_data, value_2); // 1 (protocol number)
	to_String_X2(year);
	strcat(value_data, value_2); // 1 (time year)
	to_String_X2(month);
	strcat(value_data, value_2); // 1 (time month)
	to_String_X2(day);
	strcat(value_data, value_2); // 1 (time day)
	to_String_X2(hour);
	strcat(value_data, value_2); // 1 (time hour)
	to_String_X2(minute);
	strcat(value_data, value_2); // 1 (time minute)
	to_String_X2(second);
	strcat(value_data, value_2); // 1 (time second)
	to_String_X2(quantity_satalite);
	strcat(value_data, value_2); // 1 (quantity of satellites)
	to_String_X8(latitude);
	strcat(value_data, value_8); // 4 (latitude)
	to_String_X8(longitude);
	strcat(value_data, value_8); // 4 (longitude)
	to_String_X2(speed);
	strcat(value_data, value_2); // 1 (speed)
	to_String_X4(course_status);
	strcat(value_data, value_4); // 2 (course_status)
	to_String_X4(mcc);
	strcat(value_data, value_4); // 2 (mcc)
	to_String_X2(mnc);
	strcat(value_data, value_2); // 1 (mnc)
	to_String_X4(lac);
	strcat(value_data, value_4); // 2 (lac)
	to_String_X2(cellid[0]);
	strcat(value_data, value_2); // 1 (cellid 1st byte)
	to_String_X2(cellid[1]);
	strcat(value_data, value_2); // 1 (cellid 2nd byte)
	to_String_X2(cellid[2]);
	strcat(value_data, value_2); // 1 (cellid 3rd byte)
	to_String_X2(acc);
	strcat(value_data, value_2); // 1 (acc)
	to_String_X2(data_upload_mode);
	strcat(value_data, value_2); // 1 (data upload mode)
	to_String_X2(gps_real_upload);
	strcat(value_data, value_2); // 1 (gps real time upload mode)
	to_String_X8(mileage);
	strcat(value_data, value_8); // 4 (mileage)
	to_String_X4(serial_number);
	strcat(value_data, value_4); // 2 (serial number language)
	to_String_X4(check_sum);
	strcat(value_data, value_4); // 2 (CheckSum)
	to_String_X4(0x0d0a);
	strcat(value_data, value_4); // 2 (stop bit)
	
	// Convert Location Package To Bytes Array Depending On Length = 38 + 5
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(38 + 5);i+=2,r++){
		value_data[r] = (char)(convertCharToHex(value_data[i])<<4)|convertCharToHex(value_data[i+1]);
		checkSum_index++;
	}
}

void genAlarmPackage(int quantity_satalite,
						long latitude,
						long longitude,
						unsigned char speed,
						unsigned int course_status,
						int lbs_length,
						unsigned int mcc,
						unsigned char mnc,
						unsigned int lac,
						char cellid[],
						unsigned char terminal_info,
						unsigned char build_in_battery_level,
						unsigned char gsm_signal_strength,
						int alarm_language,
						long mileage,
						int serial_number)
{
	memset(value_data, 0, sizeof(value_data));
	to_String_X4(0x7878);
	strcat(value_data, value_4); // 2 (start bit)
	to_String_X2(0x29);
	strcat(value_data, value_2); // 1 (packet length)
	to_String_X2(0x26);
	strcat(value_data, value_2); // 1 (protocol number)
	to_String_X2(year);
	strcat(value_data, value_2); // 1 (time year)
	to_String_X2(month);
	strcat(value_data, value_2); // 1 (time month)
	to_String_X2(day);
	strcat(value_data, value_2); // 1 (time day)
	to_String_X2(hour);
	strcat(value_data, value_2); // 1 (time hour)
	to_String_X2(minute);
	strcat(value_data, value_2); // 1 (time minute)
	to_String_X2(second);
	strcat(value_data, value_2); // 1 (time second)
	to_String_X2(quantity_satalite);
	strcat(value_data, value_2); // 1 (quantity of satellites)
	to_String_X8(latitude);
	strcat(value_data, value_8); // 4 (latitude)
	to_String_X8(longitude);
	strcat(value_data, value_8); // 4 (longitude)
	to_String_X2(speed);
	strcat(value_data, value_2); // 1 (speed)
	to_String_X4(course_status);
	strcat(value_data, value_4); // 2 (course_status)
	to_String_X2(lbs_length);
	strcat(value_data, value_2); // 1 (lbs length)
	to_String_X4(mcc);
	strcat(value_data, value_4); // 2 (mcc)
	to_String_X2(mnc);
	strcat(value_data, value_2); // 1 (mnc)
	to_String_X4(lac);
	strcat(value_data, value_4); // 2 (lac)
	to_String_X2(cellid[0]);
	strcat(value_data, value_2); // 1 (cellid 1st byte)
	to_String_X2(cellid[1]);
	strcat(value_data, value_2); // 1 (cellid 2nd byte)
	to_String_X2(cellid[2]);
	strcat(value_data, value_2); // 1 (cellid 3rd byte)
	to_String_X2(terminal_info);
	strcat(value_data, value_2); // 1 (terminal information)
	to_String_X2(build_in_battery_level);
	strcat(value_data, value_2); // 1 (build in battery voltage level)
	to_String_X2(gsm_signal_strength);
	strcat(value_data, value_2); // 1 (gsm signal strength)
	to_String_X4(alarm_language);
	strcat(value_data, value_4); // 2 (alarm/languge)
	to_String_X8(mileage);
	strcat(value_data, value_8); // 4 (mileage)
	to_String_X4(serial_number);
	strcat(value_data, value_4); // 2 (serial number language)
	to_String_X4(0x00);
	strcat(value_data, value_4); // 2 (CheckSum)
	to_String_X4(0x0d0a);
	strcat(value_data, value_4); // 2 (stop bit)
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(41 + 5);i+=2,r++){
		value_data[r] = (char)(convertCharToHex(value_data[i])<<4)|convertCharToHex(value_data[i+1]);
		checkSum_index++;
	}
	check_sum = GetCrc16(value_data, checkSum_index);
	
	memset(value_data, 0, sizeof(value_data));
	to_String_X4(0x7878);
	strcat(value_data, value_4); // 2 (start bit)
	to_String_X2(0x29);
	strcat(value_data, value_2); // 1 (packet length)
	to_String_X2(0x26);
	strcat(value_data, value_2); // 1 (protocol number)
	to_String_X2(year);
	strcat(value_data, value_2); // 1 (time year)
	to_String_X2(month);
	strcat(value_data, value_2); // 1 (time month)
	to_String_X2(day);
	strcat(value_data, value_2); // 1 (time day)
	to_String_X2(hour);
	strcat(value_data, value_2); // 1 (time hour)
	to_String_X2(minute);
	strcat(value_data, value_2); // 1 (time minute)
	to_String_X2(second);
	strcat(value_data, value_2); // 1 (time second)
	to_String_X2(quantity_satalite);
	strcat(value_data, value_2); // 1 (quantity of satellites)
	to_String_X8(latitude);
	strcat(value_data, value_8); // 4 (latitude)
	to_String_X8(longitude);
	strcat(value_data, value_8); // 4 (longitude)
	to_String_X2(speed);
	strcat(value_data, value_2); // 1 (speed)
	to_String_X4(course_status);
	strcat(value_data, value_4); // 2 (course_status)
	to_String_X2(lbs_length);
	strcat(value_data, value_2); // 1 (lbs length)
	to_String_X4(mcc);
	strcat(value_data, value_4); // 2 (mcc)
	to_String_X2(mnc);
	strcat(value_data, value_2); // 1 (mnc)
	to_String_X4(lac);
	strcat(value_data, value_4); // 2 (lac)
	to_String_X2(cellid[0]);
	strcat(value_data, value_2); // 1 (cellid 1st byte)
	to_String_X2(cellid[1]);
	strcat(value_data, value_2); // 1 (cellid 2nd byte)
	to_String_X2(cellid[2]);
	strcat(value_data, value_2); // 1 (cellid 3rd byte)
	to_String_X2(terminal_info);
	strcat(value_data, value_2); // 1 (terminal information)
	to_String_X2(build_in_battery_level);
	strcat(value_data, value_2); // 1 (build in battery voltage level)
	to_String_X2(gsm_signal_strength);
	strcat(value_data, value_2); // 1 (gsm signal strength)
	to_String_X4(alarm_language);
	strcat(value_data, value_4); // 1 (alarm/languge)
	to_String_X8(mileage);
	strcat(value_data, value_8); // 4 (mileage)
	to_String_X4(serial_number);
	strcat(value_data, value_4); // 2 (serial number language)
	to_String_X4(check_sum);
	strcat(value_data, value_4); // 2 (CheckSum)
	to_String_X4(0x0d0a);
	strcat(value_data, value_4); // 2 (stop bit)
	
	// Convert Alarm Package To Bytes Array Depending On Length = 41 + 5
	checkSum_index = 0;
	for(int i = 0,r=0; i<2*(41 + 5);i+=2,r++){
		value_data[r] = (char)(convertCharToHex(value_data[i])<<4)|convertCharToHex(value_data[i+1]);
		checkSum_index++;
	}
}

#pragma endregion

#pragma region Read_I2C_Region

void i2c_initialization()
{
	i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void copy_data(char dist[], char source[], int begin,int len)
{
	//memset(dist, 0, sizeof(dist));
	for(k = 0; k < len; k++)
	{
		dist[k] = source[begin + k];
	}
}

void strcat_data(char dist[], char source[], int begin, int len)
{
	//memset(dist, 0, sizeof(dist));
	for(k = 0; k < len; k++)
	{
		dist[k + begin] = source[k];
	}
}

void read_multiple_data(byte data[], int num_blocks,unsigned long address)
{
	//UART_SendString("read: ", 6);
	memset(data, 0, sizeof(data));
	for(j = 0; j < num_blocks; j++)
	{
		memset(readed_buffer, 0, sizeof(readed_buffer));
        int res = read_from_eeprom(i2c_default, at24C256_eeprom_address, address + j*64, 64, readed_buffer);
		if(res == PICO_ERROR_GENERIC)
        {
            break;
        }	
        else
        {
            strcat_data(data, readed_buffer, j*64, 64);
        }
	}
	//UART_SendString(data, 256);
}

void write_multiple_data(byte data[], int num_blocks,unsigned long address)
{
	//UART_SendString("write: ", 7);
	for(j = 0; j < num_blocks; j++)
	{
		memset(readed_buffer,0,sizeof(readed_buffer));
		//strncat(readed_buffer,data + j*16,16);
		copy_data(readed_buffer, data, j*64, 64);
        int res = write_to_eeprom(i2c_default, at24C256_eeprom_address, address + j*64, 64, readed_buffer);
		if(res == PICO_ERROR_GENERIC)
        {
            break;
        }
        //write_to_eprom(readed_buffer, 16, address + j*16);
		//UART_SendString(readed_buffer, 64);
	}
}

void read_eeprom_to_public_vars()
{
	memset(parserdataMessage, 0, sizeof(parserdataMessage));
	read_multiple_data(parserdataMessage, 4, address);
	dataIntoPublicVars1(parserdataMessage);
	
	_delay_ms(50);
	memset(parserdataMessage, 0, sizeof(parserdataMessage));
	read_multiple_data(parserdataMessage, 4, address1);
	dataIntoPublicVars2(parserdataMessage);
	
	_delay_ms(50);
	memset(parserdataMessage, 0, sizeof(parserdataMessage));
	read_multiple_data(parserdataMessage, 4, address2);
	dataIntoPublicVars3(parserdataMessage);
}

#pragma endregion

#pragma region UART_REGION

void software_reset()
{
    watchdog_enable(1, 1);
    while(1);
}

void serialEvent() {
	//Serial.print(parsercommand);
	parseMessage();
	if(parsercommand == 1)
	{
        _delay_ms(100);
		gen_connection_message();
		stringToCharArray(message_to_server, 2*(parserdataLength + 16));
		UART_SendString(message_to_server,parserdataLength + 16);
	}
	else if(parsercommand == 11)
	{
		//sector_erase(address);
		_delay_ms(100);
		genClearMemory_message();
		stringToCharArray(message_to_server, 2*(parserdataLength + 16));
		UART_SendString(message_to_server,parserdataLength + 16);
	}
	else if(parsercommand == 10)
	{
		//UART_SendString(parserdataMessage, 256);
		//UART_SendString(parserdataMessage + 256, 256);
		write_multiple_data(parserdataMessage, 4, address);
		memset(parserdataMessage, 0, sizeof(parserdataMessage));
		read_multiple_data(parserdataMessage, 4, address);
		dataIntoPublicVars1(parserdataMessage);
		//print_non_io_public_vars();
		genNonIoVariables_message();
		stringToCharArray(message_to_server, 2*(dataLength + 15));
		UART_SendString(message_to_server,dataLength + 15);
		//UART_SendString(parserdataMessage, 100);
	}
	else if(parsercommand == 12)
	{
		// address1
		//UART_SendString("write: ", 7);
		//UART_SendString(parserdataMessage, 256);
		write_multiple_data(parserdataMessage, 4, address1);
		memset(parserdataMessage, 0, sizeof(parserdataMessage));
		read_multiple_data(parserdataMessage, 4, address1);
		dataIntoPublicVars2(parserdataMessage);
		//print_non_io_public_vars_coll_1();
		//UART_SendString("read: ", 6);
		//UART_SendString(parserdataMessage, 256);
		genIoVariablesFirstCollection_message();
		stringToCharArray(message_to_server, 2*(dataLength + 15));
		UART_SendString(message_to_server,dataLength + 15);
	}
	else if(parsercommand == 13)
	{
		// address1
		//UART_SendString("write: ", 7);
		//UART_SendString(parserdataMessage, 256);
		write_multiple_data(parserdataMessage, 4, address2);
		memset(parserdataMessage, 0, sizeof(parserdataMessage));
		read_multiple_data(parserdataMessage, 4, address2);
		dataIntoPublicVars3(parserdataMessage);
		//UART_SendString("read: ", 6);
		//UART_SendString(parserdataMessage, 256);
		genIoVariablesSecondCollection_message();
		stringToCharArray(message_to_server, 2*(dataLength + 15));
		UART_SendString(message_to_server,dataLength + 15);
	}
	else if(parsercommand == 14)
	{
		// address1
		//UART_SendString("write: ", 7);
		//UART_SendString(parserdataMessage, 256);
		write_multiple_data(parserdataMessage, 4, address3);
		memset(parserdataMessage, 0, sizeof(parserdataMessage));
		read_multiple_data(parserdataMessage, 4, address3);
		dataIntoPublicVars3(parserdataMessage);
		//UART_SendString("read: ", 6);
		//UART_SendString(parserdataMessage, 256);
		genIoVariablesThirdCollection_message();
		stringToCharArray(message_to_server, 2*(dataLength + 15));
		UART_SendString(message_to_server,dataLength + 15);
        software_reset();
	}
	else if(parsercommand == 18)
	{
		memset(parserdataMessage, 0, sizeof(parserdataMessage));
		read_multiple_data(parserdataMessage, 4, address);
		//UART_SendString("read: ", 6);
		//UART_SendString(parserdataMessage, 256);
		getIoVarsZeroCollection_message();
		stringToCharArray(message_to_server, 2*(12));
		UART_SendString(message_to_server,12);
		UART_SendString(parserdataMessage, 256);
		genCheckSumFooter();
		stringToCharArray(message_to_server, 2*(3));
		UART_SendString(message_to_server,3);
		
	}
	else if(parsercommand == 15)
	{
		memset(parserdataMessage, 0, sizeof(parserdataMessage));
		read_multiple_data(parserdataMessage, 4, address1);
		//UART_SendString("read: ", 6);
		//UART_SendString(parserdataMessage, 256);
		getIoVarsFirstCollection_message();
		stringToCharArray(message_to_server, 2*(12));
		UART_SendString(message_to_server,12);
		UART_SendString(parserdataMessage, 256);
		genCheckSumFooter();
		stringToCharArray(message_to_server, 2*(3));
		UART_SendString(message_to_server,3);
	}
	else if(parsercommand == 16)
	{
		memset(parserdataMessage, 0, sizeof(parserdataMessage));
		read_multiple_data(parserdataMessage, 4, address2);
		//UART_SendString("read: ", 6);
		//UART_SendString(parserdataMessage, 256);
		getIoVarsSecondCollection_message();
		stringToCharArray(message_to_server, 2*(12));
		UART_SendString(message_to_server,12);
		UART_SendString(parserdataMessage, 256);
		genCheckSumFooter();
		stringToCharArray(message_to_server, 2*(3));
		UART_SendString(message_to_server,3);
	}
	else if(parsercommand == 17)
	{
		memset(parserdataMessage, 0, sizeof(parserdataMessage));
		read_multiple_data(parserdataMessage, 4, address3);
		//UART_SendString("read: ", 6);
		//UART_SendString(parserdataMessage, 256);
		getIoVarsThirdCollection_message();
		stringToCharArray(message_to_server, 2*(12));
		UART_SendString(message_to_server,12);
		UART_SendString(parserdataMessage, 256);
		genCheckSumFooter();
		stringToCharArray(message_to_server, 2*(3));
		UART_SendString(message_to_server,3);
	}
	else
	{
		gen_error_commandType_onUpdatingVars();
		stringToCharArray(message_to_server, 2*(dataLength + 15));
		UART_SendString(message_to_server,dataLength + 15);
	}
	//handle_received_message_from_configurator();
}

// RX interrupt handler
void on_uart_rx1() {
    while (uart_is_readable(UART_ID)) {
        c = uart_getc(UART_ID);
		//uart_putc(UART_ID_1, c);
        if(c == 0xff && r == 0)
        {
            memset(message_to_server, 0, sizeof(message_to_server));
            r = 0;
            message_to_server[r] = c;
            r++;
        }
        else if(c == 0xcd && r == parserdataLength + 15 - 1)
        {
            message_to_server[r] = c;
            r++;
            _delay_ms(100);
            serialEvent();
            //if(cong_on == 1)
            //{
                //serialEvent();
            //}
            r = 0;
            
        }
        else
        {
            message_to_server[r] = c;
            if(r == 2)
            {
                parserdataLength = (unsigned char) message_to_server[2] + 256*((unsigned char) message_to_server[1]);
                //UART_TxChar(message_to_server[2]);
                //UART_TxChar(message_to_server[1]);
            }
            r++;
        }
    }
}

void setup_uart1()
{
    
	uart_init(UART_ID, BAUD_RATE);
	// Set the TX and RX pins by using the function select on the GPIO
	// Set datasheet for more information on function select
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

	// Actually, we want a different speed
	// The call will return the actual baud rate selected, which will be as close as
	// possible to that requested
	int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

	// Set UART flow control CTS/RTS, we don't want these, so turn them off
	uart_set_hw_flow(UART_ID, false, false);

	// Set our data format
	uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

	// Turn off FIFO's - we want to do this character by character
	uart_set_fifo_enabled(UART_ID, false);

	// Set up a RX interrupt
	// We need to set up the handler first
	// Select correct interrupt for the UART we are using
	int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

	// And set up and enable the interrupt handlers
	irq_set_exclusive_handler(UART_IRQ, on_uart_rx1);
	irq_set_enabled(UART_IRQ, true);

	// Now enable the UART to send interrupts - RX only
	uart_set_irq_enables(UART_ID, true, false);

	// OK, all set up.
	// Lets send a basic string out, and then run a loop and wait for RX interrupts
	// The handler will count them, but also reflect the incoming data back with a slight change!
	// uart_puts(UART_ID, "\nHello, uart interrupts\n");
}
#pragma endregion

#pragma region helpfull_functions

void setIMEI()
{
	// 172383631313233
	IMEI[0] = 0x01;
	IMEI[1] = 0x72;
	IMEI[2] = 0x38;
	IMEI[3] = 0x36;
	IMEI[4] = 0x31;
	IMEI[5] = 0x31;
	IMEI[6] = 0x32;
	IMEI[7] = 0x33;
}

void setTime(char current_time[])
{
	
	year   = current_time[5];
	month  = current_time[4];
	day    = current_time[3];
	hour   = curr_time[0];
	minute = curr_time[1];
	second = curr_time[2];
}

#pragma endregion

#pragma region parseing_message_region

int package_length = 0;
unsigned char protocol_number = 0;
int information_serial_num = 0;
int error_check = 0;
unsigned char imei[8] = "";

int get_received_message()
{
	unsigned char fill = 0;
	char ch = 0x00;
	memset(value_data, 0, sizeof(value_data));
	int r=0;
	for(int i = 0; i<(sizeof(RESPONSE_BUFFER));i+=2){
		ch = (char)(convertCharToHex(RESPONSE_BUFFER[i])<<4)|convertCharToHex(RESPONSE_BUFFER[i+1]);
		if(ch == 0x78 && fill == 0)
		{
			value_data[r] = ch; r++;
		}
		if(r >= 1)
		{
			if(fill == 1)
			{
				value_data[r] = ch;
				if(value_data[r-1] == 0x0d && value_data[r] == 0x0a)
				{
					break;
				}
				r++;
			}
			if(value_data[r-1] == 0x78)
			{
				fill = 1;
			}
		}
	}
	return r + 1;
}

void parse_received_message(char data_mat[])
{
	protocol_number = (unsigned char) data_mat[3];
	switch(protocol_number)
	{
		case 0x01:								// Login Response
		{
			package_length		   = (unsigned char) data_mat[2];
			information_serial_num = ((unsigned char) data_mat[4]) * 256 + (unsigned char) data_mat[5];
			error_check            = ((unsigned char) data_mat[6]) * 256 + (unsigned char) data_mat[7];
		}break;
		case 0x13:								// HeartBeat Response
		{
			package_length		   = (unsigned char) data_mat[2];
			information_serial_num = ((unsigned char) data_mat[4]) * 256 + (unsigned char) data_mat[5];
			error_check            = ((unsigned char) data_mat[6]) * 256 + (unsigned char) data_mat[7];
		}break;
		case 0x26:								// HeartBeat Response
		{
			package_length		   = (unsigned char) data_mat[2];
			information_serial_num = ((unsigned char) data_mat[4]) * 256 + (unsigned char) data_mat[5];
			error_check            = ((unsigned char) data_mat[6]) * 256 + (unsigned char) data_mat[7];
		}break;
		default:
		{
			
		}break;
	}
}

bool check_message_correctness(int serial_num, int len_of_data)
{
	int ch_sum = GetCrc16(value_data, len_of_data);
	char ch_sum_char[10] = "";
	sprintf(ch_sum_char, "%d", ch_sum);
	if(serial_num == information_serial_num)
	{
		if(ch_sum == error_check)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}


#pragma endregion

#pragma region timer_region

bool repeating_timer_callback(struct repeating_timer *t) {
	if(login_send == true)
	{
		printf("Repeat at %lld\n", time_us_64());
		cancel_repeating_timer(&timer);

		switch(timer_status)
		{
			case 2:
			{

				printf("Sending Heartbeat ...\n");
				genHeartbeatMessage(0x11, 300, 50, 34, 1, serial_number);
				sendToserver(3);
				printf("heartbeat done\n");
			}break;
			case 5:
			{
				memset(curr_time, 0, sizeof(curr_time));
				getGPSINFO();
				getGPRSINFO();
				setTime(curr_time);
				printf("Sending Location/Alarm ...\n");
				
				if(sos_alarm == false)
				{
					if(latitude != 0 && longitude != 0)
					{
						genLocationPackage(satNum,
						latitude,
						longitude,
						speed,
						course | 0x1000,
						mcc,
						mnc,
						lac,
						cellid,
						0x00,
						0x00,
						0x01,
						0x04,
						serial_number);
					}
					sendToserver(2);
					printf("location done\n");
				}
				else
				{
					if(latitude != 0 && longitude != 0)
					{
						_delay_ms(2000);
						genAlarmPackage(satNum,
						latitude,
						longitude,
						speed,
						course | 0x1000,
						0x09,
						mcc,
						mnc,
						lac,
						cellid,
						0x01,
						0x30,
						0x16,
						0x01,
						0x08,
						serial_number);
						sendToserver(4);
						sos_alarm = false;
						printf("alarm done\n");
					}
					
				}
				tot_overflow = 0;
			}break;
			default:
			{
				/*_delay_ms(2000);
				printf("Sending Offline ...\n");
				
				read_from_eeprom_1024(EEPROM_Write_Addess_1024_LOW, EEPROM_Read_Addess_1024_LOW, 0x00, 4, num_offline_msg_char);
				num_of_offline_messages = 0;
				num_of_offline_messages += (uint32_t) num_offline_msg_char[3] << 24;
				num_of_offline_messages += (uint32_t) num_offline_msg_char[2] << 16;
				num_of_offline_messages += (uint32_t) num_offline_msg_char[1] << 8;
				num_of_offline_messages += (uint32_t) num_offline_msg_char[0];
				if(num_of_offline_messages > 0)
				{
					for(k = 0; k < num_sent_offline; k++)
					{
						read_current_offline_msg_from_eeprom();
						sendToserver(2);
						printf("message_ok\n");
					}
				}*/
				printf("offline done\n");
			}break;
		}
	
		add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);
    	return true;
	}
	else
	{
		cancel_repeating_timer(&timer);
		return true;
	}
    
}

#pragma endregion
 
#pragma region send_to_server_region

bool sendToserver(int msgType){
	uint8_t i_loop = 0;
	labelgprs: if(gprsEnabled == false){
		i_loop = 0;
		do
		{
			enable = Init_GPRS(APN);
			if (enable)
			{
				gprsEnabled = true;
			}
			else
			{
				i_loop++;
				if(i_loop < 3)
				{
					gprsEnabled = false;
					continue;
				}
				else{
					flag_reg = false;
					i_loop = 0;
					//ResetTimer1();
					do
					{
						simreg = simRegistration();
					} while (simreg == false);
					flag_reg = true;
					goto labelgprs;
				}
			}
		} while (gprsEnabled == false);
	}// end of if gprsenabled = false;
	i_loop = 0;
	labelNet: do 
	{
		i_loop++;
		if(i_loop < 4){
			openNet = is_openSocket();
			if (openNet == 1)
			{
				break;
			}
			else
			{
				continue;
			}
		}
		else{
			gprsEnabled = false;
			goto labelgprs;
		}
	} while (openNet != 1);
	i_loop = 0;
	do
	{
		i_loop++;
		if (i_loop < 3){
			//USART1_TxChar(server_ip[0]); USART1_TxChar(server_ip[1]); USART1_TxChar(server_ip[2]); USART1_TxChar(server_ip[3]);
			if(login_send == true)
			{
				printf("going to send !!!\n");
				
				goto labelsend;
			}
			start = establish_connection(server_ip, mainServerPort); // 5230 // 5227
			if (start == 1) break;
			else continue;
		}
		else
		{
			i_loop = 0;
			if(msgType == 1)
			{
				printf("msgType = 1\n");
				return false;
			}
			goto labelNet;
			
		}
	} while ((start != 1));
	if(start != 1) return false;

labelsend:
	printf("labelsend\n");
	if(msgType == 1)
	{
		if(TCPsend(value_data,17 + 5) != SIM7600_RESPONSE_ERROR)
		{
			printf("message types 1\n");
			getDataManually();
			send_tries++;
			int len = get_received_message();
			parse_received_message(value_data);
			bool res = check_message_correctness(serial_number, len);
			if(res == true)
			{
				login_send = true;
				printf("res = true\n");
				
				serial_number++;
			}
			else
			{
				printf("error with login !!!\n");
				if(send_tries == MAX_SEND_TRIES)
				{
					close_socket();
					close_NET();
				}
				else
				{
					genLoginMessage(IMEI, 1, 1, serial_number);
					sendToserver(1);
				}
			}
		}
		else
		{
			close_socket();
			close_NET();
		}
	}
	else if(msgType == 2)
	{
		if(TCPsendLocation(value_data,38 + 5) != SIM7600_RESPONSE_ERROR)
		{
			//getDataManually();
		}
		else
		{
			login_send = false;
			close_socket();
			close_NET();
		}
	}
	else if(msgType == 3)
	{
		if(TCPsendHeartbeat(value_data, 15) != SIM7600_RESPONSE_ERROR)
		{
			getDataManually();
			int len = get_received_message();
			parse_received_message(value_data);
			bool res = check_message_correctness(serial_number, len);
			if(res == true)
			{
				serial_number++;
			}
			else
			{
				if(send_tries == MAX_SEND_TRIES)
				{
					printf("Max Tries HeartBeat !!!\n");
					
					login_send = false;
					close_socket();
					close_NET();
				}
				else
				{
					genHeartbeatMessage(0x11, 300, 50, 34, 1, serial_number);
					sendToserver(3);
				}
			}
		}
		else
		{
			printf("Error With HeartBeat Response !!!\n");
			
			login_send = false;
			close_socket();
			close_NET();
		}
	}
	else if(msgType == 4)
	{
		if(TCPsendAlarm(value_data, 46) != SIM7600_RESPONSE_ERROR)
		{
			getDataManually();
			int len = get_received_message();
			parse_received_message(value_data);
			bool res = check_message_correctness(serial_number, len);
			if(res == true)
			{
				serial_number++;
			}
			else
			{
				if(send_tries == MAX_SEND_TRIES)
				{
					close_socket();
					close_NET();
				}
				else
				{
					genHeartbeatMessage(0x11, 300, 50, 34, 1, serial_number);
					sendToserver(3);
					genAlarmPackage(0xc9,
						latitude,
						longitude,
						0x00,
						0x1400,
						0x09,
						0x01cc,
						0x00,
						0x287d,
						cellid,
						0x01,
						0x30,
						0x16,
						0x01,
						0x08,
						serial_number);
					sendToserver(4);
				}
			}
		}
		else
		{
			login_send = false;
			close_socket();
			close_NET();
		}
	}
}

#pragma endregion

int main() {
    // Set up our UART with a basic baud rate.
    stdio_init_all();
    setup_uart0();
	setup_uart1();
	i2c_initialization();
    read_eeprom_to_public_vars();
	server_ip[0] = mainServerIp[0];
	server_ip[1] = mainServerIp[1];
	server_ip[2] = mainServerIp[2];
	server_ip[3] = mainServerIp[3];
    sleep_ms(3000);

    while(!(reset_device()));
    printf("%s", "SIM7600 Reset OK ^-^\n");
    
    while((conf_mode == 0) && (!SIM7600_Init()));
    printf("%s", "SIM7600 Initialization OK ^-^\n");

    while((conf_mode == 0) && (!initGPS()));
    printf("%s", "SIM7600 GPS Initialization OK ^-^\n");

    while((conf_mode == 0) && (!simRegistration()));
    printf("%s", "SIM7600 Registration OK ^-^\n");
	
    while (1)
    {
        while(login_send == false)
		{
			send_tries = 0;
			setIMEI();
			printf("send login message!\n");
			genLoginMessage(IMEI, 1, 1, serial_number);
			printf("login message = %s\n", value_data);
			sendToserver(1);
			if(login_send == true)
			{
				printf("login done!\n");
				add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);
			}
			else
			{
				//write_current_offline_to_eeprom();
				//_delay_ms(3000);
			}
		}
    }
        
}