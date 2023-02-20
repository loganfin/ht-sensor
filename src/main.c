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
#include "pico/binary_info.h"
#include "hardware/i2c.h"

/* the grove connectors on the shield I'm using are tied to
 * the SCL (GPIO3) and SDA (GPIO2) pins on the Pico */
#define I2C_BLOCK   i2c1
#define I2C_SCL     3
#define I2C_SDA     2

#define HDC_ADDRESS         0x40    // 100 0000 (7 bit address)
#define HDC_TEMP_REG        0x00
#define HDC_HUMID_REG       0x01
#define HDC_CONF_REG        0x02
#define HDC_SN_ONE_REG      0xFB
#define HDC_SN_TWO_REG      0xFC
#define HDC_SN_THREE_REG    0xFD
#define HDC_MAN_ID_REG      0xFE

void SetConfigVal(uint16_t configuration);
uint16_t ReadConfig();
uint16_t ReadManID();
uint64_t ReadSN();      // Serial number is a 5 byte value
uint16_t ReadTemp();
float ReadHum();
float TempToC();
float TempToF();

int main()
{
    stdio_init_all();

    i2c_init(I2C_BLOCK, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);     // i2c is line idle high, active low, so voltage needs to be pulled up high at the start
    gpio_pull_up(I2C_SCL);
    SetConfigVal(0x9000);
    sleep_ms(1000);

    while (true) {
        printf("Config Register: 0x%04X\n", ReadConfig());
        printf("Manufacturer's ID: 0x%04X\n", ReadManID());
        printf("Serial Number: 0x%010llX\n", ReadSN());
        printf("Current Temperature (C): %f\n", TempToC());
        printf("Current Temperature (F): %f\n", TempToF());
        printf("Current Humidity (\%): %f\n", ReadHum());
        printf("\n");
        sleep_ms(1000);
    }
}

void SetConfigVal(uint16_t configuration)
{
    // 0001 0101 0000 0000
    // 0x1500
    uint8_t data[3] = {0x02, 0x15, 0x00} ;
    i2c_write_blocking(I2C_BLOCK, HDC_ADDRESS, data, 3, false);
    sleep_ms(20);
}

uint16_t ReadConfig()
{
    uint8_t byte[2];
    uint8_t confReg = HDC_CONF_REG;
    uint16_t confVal = 0;

    i2c_write_blocking(I2C_BLOCK, HDC_ADDRESS, &confReg, 1, false);
    sleep_ms(10);
    i2c_read_blocking(I2C_BLOCK, HDC_ADDRESS, byte, 2, false);
    confVal = byte[0] << 8 | byte[1];
    return confVal;
}

uint16_t ReadManID()
{
    uint8_t byte[2];
    uint8_t manIdReg = HDC_MAN_ID_REG;
    uint16_t manId = 0;

    // read the data from the manufacturer id register
    i2c_write_blocking(I2C_BLOCK, HDC_ADDRESS, &manIdReg, 1, false);
    sleep_ms(10);
    i2c_read_blocking(I2C_BLOCK, HDC_ADDRESS, byte, 2, false);

    // combine the two distinct bytes
    manId = byte[0] << 8 | byte[1];
    return manId;
}

uint64_t ReadSN()
{
    uint8_t byte[5];
    uint8_t serialNumOneReg     = HDC_SN_ONE_REG;
    uint8_t serialNumTwoReg     = HDC_SN_TWO_REG;
    uint8_t serialNumThreeReg   = HDC_SN_THREE_REG;
    uint64_t serialNumVal       = 0;

    // read the first 2 bytes
    i2c_write_blocking(I2C_BLOCK, HDC_ADDRESS, &serialNumOneReg, 1, false);
    sleep_ms(10);
    i2c_read_blocking(I2C_BLOCK, HDC_ADDRESS, byte, 2, false);

    // read the second 2 bytes
    i2c_write_blocking(I2C_BLOCK, HDC_ADDRESS, &serialNumTwoReg, 1, false);
    sleep_ms(10);
    i2c_read_blocking(I2C_BLOCK, HDC_ADDRESS, &byte[2], 2, false);

    // read the last byte
    i2c_write_blocking(I2C_BLOCK, HDC_ADDRESS, &serialNumThreeReg, 1, false);
    sleep_ms(10);
    i2c_read_blocking(I2C_BLOCK, HDC_ADDRESS, &byte[4], 1, false);

    // combine all bytes into one 5 byte hex number
    serialNumVal = ((((byte[0] << 8 | byte[1]) << 8 | byte[2]) << 8 | byte[3]) << 8 | byte[4]);
    return serialNumVal;
}

uint16_t ReadTemp()
{
    uint8_t byte[2];
    uint8_t tempReg = HDC_TEMP_REG;
    uint16_t rawTemp = 0;

    i2c_write_blocking(I2C_BLOCK, HDC_ADDRESS, &tempReg, 1, false);
    sleep_ms(10);
    i2c_read_blocking(I2C_BLOCK, HDC_ADDRESS, byte, 2, false);
    rawTemp = byte[0] << 8 | byte[1];
    return rawTemp;
}

float TempToC()
{
    uint16_t rawTemp = ReadTemp();
    float tempC = 0;
    tempC = ((float)rawTemp / 65536) * 165 - 40;
    return tempC;
}

float TempToF()
{
    float tempC = TempToC();
    float tempF = (tempC * 9/5) + 32;
    return tempF;
}

float ReadHum()
{
    uint8_t byte[2];
    uint8_t humidReg = HDC_HUMID_REG;
    uint16_t rawHumid = 0;
    float percentHumidity;

    i2c_write_blocking(I2C_BLOCK, HDC_ADDRESS, &humidReg, 1, false);
    sleep_ms(10);
    i2c_read_blocking(I2C_BLOCK, HDC_ADDRESS, byte, 2, false);
    rawHumid = byte[0] << 8 | byte[1];
    percentHumidity = ((float)rawHumid / 65536) * 100;
    return percentHumidity;
}
