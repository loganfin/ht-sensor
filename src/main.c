/* Set configuration register
 * Read and return configuration register
 * Read and return Manufacturing ID
 * Read and return Unique Serial Number
 * Read and return temperature in C
 * Read and return temperature in F
 * Read and return humidity
 */

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define HDC_ADDRESS         0x40
#define HDC_TEMP_REG        0x00
#define HDC_HUMID_REG       0x01
#define HDC_SN_ONE_REG      0xFB
#define HDC_SN_TWO_REG      0xFC
#define HDC_SN_THREE_REG    0xFD
#define HDC_MAN_ID_REG      0xFE

int main()
{
    stdio_init_all();   // Enable serial port to communicate with serial monitor
}
