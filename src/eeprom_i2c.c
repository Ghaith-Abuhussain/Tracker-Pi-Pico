#include "eeprom_i2c.h"

void _delay_ms(uint32_t ms)
{
    uint32_t curr_tm = to_ms_since_boot(get_absolute_time());
    while(to_ms_since_boot(get_absolute_time()) - curr_tm <= ms);
}

int write_to_eeprom(i2c_inst_t * i2c, uint8_t device_address, uint16_t address, size_t len, uint8_t * data_to_write)
{
    uint8_t all_data[128] = "";
    uint8_t address_char[2] = "";
    address_char[0] = ((address >> 8) & 0xff);
    address_char[1] = (address & 0xff);
    //printf("address = %d\n", address);
    //printf("address[0] = %d\n", address_char[0]);
    //printf("address[1] = %d\n", address_char[1]);
    all_data[0] = address_char[0];
    all_data[1] = address_char[1];
    for(int i = 0; i < len; i++)
    {
        all_data[2 + i] = data_to_write[i];
    }
    /*for(int i = 0; i < len + 2; i++)
    {
        printf("all_data[%d] = %d, ", i, all_data[i]);
    }*/
    //printf("\n");
    //int res = i2c_write_blocking(i2c, device_address, all_data, len + 2, false);
    int res = i2c_write_blocking(i2c, device_address, all_data, len + 2, false);
    _delay_ms(100);
    return res;
}

int read_from_eeprom(i2c_inst_t * i2c, uint8_t device_address, uint16_t address, size_t len, uint8_t * data_to_read)
{
    uint8_t address_char[2] = "";
    //printf("address = %d\n", address);
    address_char[0] = ((address >> 8) & 0xff);
    address_char[1] = (address & 0xff);
    //printf("address[0] = %d\n", address_char[0]);
    //printf("address[1] = %d\n", address_char[1]);
    int res = i2c_write_blocking(i2c, device_address, address_char, 2, true);
    _delay_ms(100);
    if(res == PICO_ERROR_GENERIC)
    {
        return res;
    }	
    else
    {
        res = i2c_read_blocking	(i2c, device_address, data_to_read, len, false);
        return res;
    }
}