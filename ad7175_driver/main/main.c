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

#include "AD7175.h"
#include "Communication.h"

static void adc_read_task(void* arg)
{
    AD7175_Setup();
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
}

void app_main(void)
{
    xTaskCreate(&adc_read_task, "adc_read", 4 * 1024, NULL, 5, NULL);

    while(1) {
        vTaskDelay(10);
    }
}
