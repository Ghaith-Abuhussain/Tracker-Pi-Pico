#include "millis.h"



uint32_t milliseconds(void)
{
    uint32_t a = to_ms_since_boot(get_absolute_time());
    return a;
}