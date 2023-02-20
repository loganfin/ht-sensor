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

// had to use const uint8_t to use the ReadRegister function
const uint8_t hdc_address       = 0x40;
const uint8_t hdc_temp_reg      = 0x00;
const uint8_t hdc_humid_reg     = 0x01;
const uint8_t hdc_config_reg    = 0x02;
const uint8_t hdc_man_id_reg    = 0xFE;
const uint8_t hdc_sn_reg_arr[3] = {
    0xFB,
    0xFC,
    0xFD
};

void SetConfig(uint16_t configVal);
uint64_t ReadRegister(const uint8_t* regAddrVec, const uint8_t vecSize, const uint8_t numBytes);
float TempToC();
float TempToF();
float HumidToPercent();

void vPrintTask();

int main()
{
    stdio_init_all();
    i2c_init(I2C_BLOCK, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);     // i2c is line idle high, active low, so voltage needs to be pulled up high at the start
    gpio_pull_up(I2C_SCL);

    xTaskCreate(vPrintTask, "PrintTask", 256, NULL, 2, NULL);
    vTaskStartScheduler();

    while (true);
}

/* BEGIN RTOS TASKS */

void vPrintTask()
{
    SetConfig(0x1500);

    while (true) {
        printf("Configuration Register: 0x%04X\n", (uint16_t)ReadRegister(&hdc_config_reg, 1, 2));
        printf("Serial Number: 0x%012llX\n", ReadRegister(hdc_sn_reg_arr, 3, 6));
        printf("Manufacturer ID: 0x%04X\n", (uint16_t)ReadRegister(&hdc_man_id_reg, 1, 2));
        printf("Current Temperature (C): %f\n", TempToC());
        printf("Current Temperature (F): %f\n", TempToF());
        printf("Current Humidity (\%): %f\n", HumidToPercent());
        printf("\n");
        vTaskDelay(10 * configTICK_RATE_HZ);
    }
}

/* END RTOS TASKS */

/* BEGIN HDC HELPER FUNCTIONS */
void SetConfig(uint16_t configVal)
{
    uint8_t data[3] = {
        hdc_config_reg,                     // the address of the configuration and status register
        (uint8_t)((configVal >> 8) & 0xff), // extract the first byte from config value
        (uint8_t)(configVal & 0xff)         // extract the second byte from config value
    };

    // send the data block to the HDC1080
    i2c_write_blocking(I2C_BLOCK, hdc_address, data, 3, false);
    return;
}

uint64_t ReadRegister(const uint8_t* regAddrVec, const uint8_t vecSize, const uint8_t numBytes)
{
    uint8_t data[numBytes];
    uint64_t returnValue = 0;
    uint64_t temp;

    if (numBytes > 8) {
        return -1;
    }

    for (int i = 0; i < vecSize; i++) {
        i2c_write_blocking(I2C_BLOCK, hdc_address, &regAddrVec[i], 1, false);
        vTaskDelay(1);
        i2c_read_blocking(I2C_BLOCK, hdc_address, &data[i*2], numBytes, false);
    }

    returnValue = data[0];
    for (int i = 0; i < numBytes-1; i++) {
        returnValue <<= 8;
        returnValue |= data[i+1];
    }

    return returnValue;
}

float TempToC()
{
    uint16_t rawTemp = ReadRegister(&hdc_temp_reg, 1, 2);
    float tempC = ((float)rawTemp / 65536) * 165 - 40;
    return tempC;
}

float TempToF()
{
    float tempC = TempToC();
    float tempF = (tempC * 9/5) + 32;
    return tempF;
}

float HumidToPercent()
{
    uint16_t rawHumid = ReadRegister(&hdc_humid_reg, 1, 2);
    float percentHumid = ((float)rawHumid / 65536) * 100;
    return percentHumid;
}

/* END HDC HELPER FUNCTIONS */
