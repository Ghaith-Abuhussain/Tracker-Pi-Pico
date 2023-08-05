#define UART_ID_1 uart1
#define BAUD_RATE_1 115200
#define DATA_BITS_1 8
#define STOP_BITS_1 1
#define PARITY_1    UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN_1 8
#define UART_RX_PIN_1 9

#define DEFAULT_BUFFER_SIZE_1		200
#define DEFAULT_TIMEOUT_1			2000 // was 1000


extern char RESPONSE_BUFFER[DEFAULT_BUFFER_SIZE_1];
extern char curr_time[6];
extern char at_command[128];

static int chars_rxed = 0;

void on_uart_rx() ;

void setup_uart0();

enum SIM7600_RESPONSE_STATUS {
	SIM7600_RESPONSE_WAITING,
	SIM7600_RESPONSE_FINISHED,
	SIM7600_RESPONSE_TIMEOUT,
	SIM7600_RESPONSE_BUFFER_FULL,
	SIM7600_RESPONSE_STARTING,
	SIM7600_RESPONSE_ERROR
};
void Buffer_Flush();
void readResponse(const uint8_t crlfCount);
void startReadResponse(unsigned int _timeOut);
bool SIM7600_Init();
bool reset_device();
void GetResponseBody(char* Response, uint16_t ResponseLength);
void waitForResponse(const uint8_t _crlfCount, const unsigned int _timeOut);
bool waitForExpectedResponse(const char * expectedResponse,const  uint8_t _crlfCount, const unsigned int _timeOut);
bool sendATCommandAndExpects(char * ATCommand, const char * expectedResponse,const uint8_t _crlfCount,  const unsigned int _timeOut);
bool signalQuality();
bool simRegistration();
bool Init_GPRS(char* apn);
bool is_openSocket();
unsigned long convert_to_degrees(char *raw);
enum SIM7600_RESPONSE_STATUS TCPsend(char* data, uint16_t len);
enum SIM7600_RESPONSE_STATUS TCPsendLocation(char* data, uint16_t len);
enum SIM7600_RESPONSE_STATUS TCPsendHeartbeat(char* data, uint16_t len);
enum SIM7600_RESPONSE_STATUS TCPsendAlarm(char* data, uint16_t len);
enum SIM7600_RESPONSE_STATUS establish_connection(unsigned char SERVER[], int portcon);
enum SIM7600_RESPONSE_STATUS getDataManually();
bool initGPS();
bool closeSockets();
uint64_t get_timeStamp(char date[], char time[]);
void getGPSINFO();
void getGPRSINFO();
int close_socket();
bool close_NET();