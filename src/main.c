#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

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

// RTOS tasks
void vPrintTask();

// HDC interface functions
void SetConfig(const uint16_t configVal);
uint64_t ReadRegister(const uint8_t* regAddrVec, const uint8_t vecSize);
float TempToC();
float TempToF();
float HumidToPercent();

int main()
{
    stdio_init_all();

    i2c_init(PICO_DEFAULT_I2C_INSTANCE, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    SetConfig(0x1500);

    xTaskCreate(vPrintTask, "PrintTask", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true);
}

/* BEGIN RTOS TASKS */

void vPrintTask()
{
    while (true) {
        printf("Configuration Register:  0x%04X\n", (uint16_t)ReadRegister(&hdc_config_reg, 1));
        printf("Serial Number:           0x%012llX\n", ReadRegister(hdc_sn_reg_arr, 3));
        printf("Manufacturer ID:         0x%04X\n", (uint16_t)ReadRegister(&hdc_man_id_reg, 1));
        printf("Current Temperature (C): %f\n", TempToC());
        printf("Current Temperature (F): %f\n", TempToF());
        printf("Current Humidity (%%):    %f\n", HumidToPercent());
        printf("\n");
        vTaskDelay(10 * configTICK_RATE_HZ);
    }
}

/* END RTOS TASKS */

/* BEGIN HDC HELPER FUNCTIONS */

void SetConfig(const uint16_t configVal)
{
    uint8_t data[3] = {
        hdc_config_reg,                     // the address of the configuration and status register
        (uint8_t)((configVal >> 8) & 0xff), // extract the first byte from configVal
        (uint8_t)(configVal & 0xff)         // extract the second byte from configVal
    };

    i2c_write_blocking(PICO_DEFAULT_I2C_INSTANCE, hdc_address, data, 3, false);
    return;
}

/*
 * A versatile function that can extract and return up to 8 bytes
 * of data from up to 4 different registers.
 * Returns the concatenated value of all read registers or -1 if an error
 * occured.
 * regAddrVec:  a pointer to an ordered array of register addresses to be read
 * vecSize:     the number of register addresses in regAddrVec
 */
uint64_t ReadRegister(const uint8_t* regAddrVec, const uint8_t vecSize)
{
    uint8_t numBytes = vecSize * 2;
    uint8_t data[numBytes];
    uint64_t returnValue = 0;

    if (regAddrVec == NULL || vecSize < 1 || vecSize > 4) {
        return -1;
    }

    for (int i = 0; i < vecSize; i++) {
        i2c_write_blocking(PICO_DEFAULT_I2C_INSTANCE, hdc_address, &regAddrVec[i], 1, false);
        vTaskDelay(1);
        i2c_read_blocking(PICO_DEFAULT_I2C_INSTANCE, hdc_address, &data[i * 2], numBytes, false);
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
    uint16_t rawTemp = ReadRegister(&hdc_temp_reg, 1);
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
    uint16_t rawHumid = ReadRegister(&hdc_humid_reg, 1);
    float percentHumid = ((float)rawHumid / 65536) * 100;
    return percentHumid;
}

/* END HDC HELPER FUNCTIONS */
