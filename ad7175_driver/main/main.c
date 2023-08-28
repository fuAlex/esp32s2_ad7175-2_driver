/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "AD7175.h"
#include "Communication.h"

#define ADC_HIGH_SPEED_READ_ENABLE 1

#if ADC_HIGH_SPEED_READ_ENABLE

#define READ_BUFF_LEN 128
static int32_t DRAM_ATTR ADC_BUFF[READ_BUFF_LEN];
/**
 * @note Max data output freq is 50 kHz
*/
static void adc_read_task(void* arg)
{
    AD7175_Setup();
    AD717X_Data_output_freq(0, AD7172_SIN5_SIN1_SING_CYC_50000);
    SemaphoreHandle_t err_sem = adc_error_notify_init();

    for (int i=0; i<10; i++) {
        SemaphoreHandle_t rdy_sem = adc_data_ready_reading_from_isr(ADC_BUFF, READ_BUFF_LEN);

        if (xSemaphoreTake(rdy_sem, portMAX_DELAY)) {
            for(int i = 0; i<READ_BUFF_LEN; i++) {
                printf("[i]%lx ", ADC_BUFF[i]);
            }
            printf("\n");
        }
        if(xSemaphoreTake(err_sem, NULL)) {
            uint8_t state = 0;
            AD7175_GetState(&state);
            esp_rom_printf("state %x\n", state);
        }
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}

#else
/**
 * @note Max data output freq is 500Hz
*/
static void adc_read_task(void* arg)
{
    AD7175_Setup();
    AD717X_Data_output_freq(0, AD7172_SIN5_SIN1_SING_CYC_500);
    SemaphoreHandle_t err_sem = adc_error_notify_init();
    SemaphoreHandle_t rdy_sem = adc_data_ready_notify_init();

    for(;;) {
        if (xSemaphoreTake(rdy_sem, portMAX_DELAY)) {
            // data can be read
            int32_t adc_val = 0;
            AD7175_ReadData(&adc_val);
            esp_rom_printf("adc val %d\n", adc_val);
        }
        if(xSemaphoreTake(err_sem, NULL)) {
            uint8_t state = 0;
            AD7175_GetState(&state);
            esp_rom_printf("state %x\n", state);
        }
    }
    AD717X_Standby();
}

#endif

void app_main(void)
{
    xTaskCreate(&adc_read_task, "adc_read", 4 * 1024, NULL, configMAX_PRIORITIES-1, NULL);

    while(1) {
        vTaskDelay(10);
    }
}
