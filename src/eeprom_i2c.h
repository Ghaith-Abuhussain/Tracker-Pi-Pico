#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include <stdint.h>

#define at24C256_eeprom_address		    0x50
#define at24C1024_eeprom_address_L		0x52
#define at24C1024_eeprom_address_H		0x53

void _delay_ms(uint32_t ms);

int write_to_eeprom(i2c_inst_t * i2c, uint8_t device_address, uint16_t address, size_t len, uint8_t * data_to_write);

int read_from_eeprom(i2c_inst_t * i2c, uint8_t device_address, uint16_t address, size_t len, uint8_t * data_to_read);