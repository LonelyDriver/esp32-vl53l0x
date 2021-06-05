#ifndef _LD_VL53L0x_H
#define _LD_VL53L0x_H
/**** Includes ***********************************************/
#include <stdint.h>
/**** Types **************************************************/
typedef enum _debuglevel {
    DEBUG_ERROR=0,
    DEBUG_WARNING=1,
    DEBUG_INFO=2,
    DEBUG_DEBUG=3,
    DEBUG_VERBOSE=4
} DebugLevel;

/**** Public interface ***************************************/
int VL53L0x_Init(int i2c_port, int i2c_sda, int i2c_scl);
int VL53L0x_SetDebugLevel(DebugLevel level);
int VL53L0x_GetData(int16_t* data, int16_t counts);



#endif // _LD_VL53L0x_H