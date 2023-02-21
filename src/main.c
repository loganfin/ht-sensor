/*
 * Logan Finley
 * CS 452
 * Assignment #7
 * Feb 20th, 2023
 */

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

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
    // enable serial communication
    stdio_init_all();

    // initalize i2c on the SCL (GPIO3) and SDA (GPIO2) pins
    i2c_init(PICO_DEFAULT_I2C_INSTANCE, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // i2c is line idle high, active low, so voltage needs to be pulled up high at the start
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // configure HDC status and config register
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
        // delay for 10 seconds
        vTaskDelay(10 * configTICK_RATE_HZ);
    }
}

/* END RTOS TASKS */

/* BEGIN HDC HELPER FUNCTIONS */

/*
 * Takes a 16 bit value that will be placed in the status and
 * config register on the HDC.
 */
void SetConfig(const uint16_t configVal)
{
    // compose a three-byte message for the HDC1080
    uint8_t data[3] = {
        hdc_config_reg,                     // the address of the configuration and status register
        (uint8_t)((configVal >> 8) & 0xff), // extract the first byte from configVal
        (uint8_t)(configVal & 0xff)         // extract the second byte from configVal
    };

    // send the data block to the HDC1080
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
    // the HDC1080 has a 2-byte data word, so the total number of bytes read
    // will be 2 * the number of addresses.
    uint8_t numBytes = vecSize * 2;
    uint8_t data[numBytes];
    uint64_t returnValue = 0;

    // return an error if the function is called with invalid paramters
    if (regAddrVec == NULL || vecSize < 1 || vecSize > 4) {
        return -1;
    }

    for (int i = 0; i < vecSize; i++) {
        // request the register at the address represented by index i to output its contents
        i2c_write_blocking(PICO_DEFAULT_I2C_INSTANCE, hdc_address, &regAddrVec[i], 1, false);
        vTaskDelay(1);
        // the contents of the register will be placed into data[] at each even index,
        // accomodating the entire 2-byte message.
        i2c_read_blocking(PICO_DEFAULT_I2C_INSTANCE, hdc_address, &data[i * 2], numBytes, false);
    }

    // iterate through the array, concatenating the data into returnValue
    returnValue = data[0];
    for (int i = 0; i < numBytes-1; i++) {
        returnValue <<= 8;
        returnValue |= data[i+1];
    }

    return returnValue;
}

/*
 * Reads the current temperature from the sensor and converts
 * that data into a floating point celsius measurement.
 */
float TempToC()
{
    // read the raw temperature data from the sensor
    uint16_t rawTemp = ReadRegister(&hdc_temp_reg, 1);
    // convert the raw temperature data into a floating point celcius measurement
    float tempC = ((float)rawTemp / 65536) * 165 - 40;
    return tempC;
}

/*
 * Reads the current celcius temperature and converts
 * that data into a floating point fahrenheit measurement.
 */
float TempToF()
{
    // get the current temperature in celcius
    float tempC = TempToC();
    // convert from celcius to fahrenheit
    float tempF = (tempC * 9/5) + 32;
    return tempF;
}

/*
 * Reads the current humidity from the sensor and converts
 * that data into a floating point percentage value.
 */
float HumidToPercent()
{
    // read the raw humidity data from the sensor
    uint16_t rawHumid = ReadRegister(&hdc_humid_reg, 1);
    // convert the raw humidity into a floating point percentage
    float percentHumid = ((float)rawHumid / 65536) * 100;
    return percentHumid;
}

/* END HDC HELPER FUNCTIONS */
