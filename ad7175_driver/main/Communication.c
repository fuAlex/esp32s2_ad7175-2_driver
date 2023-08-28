/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 0
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "Communication.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "AD7175.h"

static const char *TAG = "ad7175";

#define LOG_DEBUG 	0 // open debug lo for spi_read and spi_write
/**
 * Hardware connect
 * ESP32S2 <-> AD7175-2
*/
#define ADC_HOST     		SPI2_HOST
#define PIN_NUM_MISO 		37
#define PIN_NUM_MOSI 		35
#define PIN_NUM_CLK  		36
#define PIN_NUM_CS   		34
#define PIN_NUM_ADC_ERR   	33			// Monitor the ADC error status. 1: normal; 0: error.
#define PIN_NUM_ADC_RDY   	PIN_NUM_MISO // Monitor the ADC data ready status. 1: busy; 0: ready.

#define AD7175_SPI_CLK_SPEED (20 * 1000 * 1000) // MAX 20 MHz for AD7175-2

/**If compile SPI driver to IRAM can fast exe*/
#ifdef CONFIG_SPI_MASTER_IN_IRAM
#define ADC_COMM_DRIVER_ATTR IRAM_ATTR
#else
#define ADC_COMM_DRIVER_ATTR
#endif

static spi_device_handle_t spi_for_adc_init;

static DMA_ATTR uint32_t dma_access_buf[8] = {0}; // DMA can access buff that Word alignt
static DRAM_ATTR SemaphoreHandle_t ready_evt_sem = NULL;
static DRAM_ATTR SemaphoreHandle_t err_evt_sem = NULL;
static DRAM_ATTR int32_t *DATA_BUFF  = NULL;
static DRAM_ATTR int32_t DATA_BUFF_LEN = 0;
static DRAM_ATTR int32_t DATA_BUFF_CNT = 0;

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	int task_awoken = pdFALSE;
    uint32_t gpio_num = (uint32_t) arg;
	if (gpio_num == PIN_NUM_ADC_RDY) {
#ifdef CONFIG_SPI_MASTER_IN_IRAM
		if (DATA_BUFF_CNT < DATA_BUFF_LEN) {
			AD7175_ReadData(&DATA_BUFF[DATA_BUFF_CNT]); // read adc date quickly
			if (++DATA_BUFF_CNT == DATA_BUFF_LEN) {
				AD717X_Standby(); // Go to standby mdoe to save power.
				spi_device_release_bus(spi_for_adc_init);
				xSemaphoreGiveFromISR(ready_evt_sem, &task_awoken); // reading done and notify user
			}
		}
#else
		if (ready_evt_sem) {
			xSemaphoreGiveFromISR(ready_evt_sem, &task_awoken);
		}
#endif
	}
	if (gpio_num == PIN_NUM_ADC_ERR) {
		xSemaphoreGiveFromISR(err_evt_sem, &task_awoken);
		esp_rom_printf(DRAM_STR("ADC ERROR, please read status\n"));
	}
    if (task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**************************************************************************//**
* @brief Monitor the ready state by MISO interrupt.
*        When CS line is low and MISO line no data to transmit. the MISO will output ready info of adc data (low status is ready).
*        So enable the MISO pin nagitive edge interrupt to monitor the ready state. User can take the sem to get the data state.
*        Of cause, User also can read `AD717X_STATUS_REG` to get ready state.
*
* @note When use the SPI to transmit data, should disable the MISI pin gpio interrupt.
* @note Must make sure the SPI reading time smaller than internal of data output.
* @note Max data output freq is 500Hz.
*
* @param reg - Semaphore handler
*
* @return Returns ESP_OK for success
******************************************************************************/
SemaphoreHandle_t adc_data_ready_notify_init(void)
{
#ifdef CONFIG_SPI_MASTER_IN_IRAM
	ESP_LOGE(TAG, "ERROR! Please undefine `CONFIG_SPI_MASTER_IN_IRAM` or use `adc_data_ready_reading_from_isr()`");
	return NULL;
#endif
	// create a queue to handle gpio event from isr
	if (ready_evt_sem == NULL) {
    	ready_evt_sem = xSemaphoreCreateBinary();
	}

	//change gpio interrupt type for one pin
    gpio_set_intr_type(PIN_NUM_ADC_RDY, GPIO_INTR_NEGEDGE);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PIN_NUM_ADC_RDY, gpio_isr_handler, (void*) PIN_NUM_ADC_RDY);

	/**If output data ready signal via MISO pin, should tie low the CS pin*/
	gpio_set_level(PIN_NUM_CS, 0);

	return ready_evt_sem;
}

/**************************************************************************//**
* @brief Monitor the ready state by MISO interrupt.
*        When CS line is low and MISO line no data to transmit. the MISO will output ready info of adc data (low status is ready).
*        So enable the MISO pin nagitive edge interrupt to monitor the ready state. User can take the sem to get the data state.
*        Of cause, User also can read `AD717X_STATUS_REG` to get ready state.
*
* @note When use the SPI to transmit data, should disable the MISI pin gpio interrupt.
* @note Must make sure the SPI reading time smaller than internal of data output.
* @note Max 50KHz sample rate
* @note The buff data should be in DRAM_ATTR
* @note Please enable CONFIG_SPI_MASTER_IN_IRAM in menuconfig
*
* @param buff - The buff pointer to save adc data. should in DRAM_ATTR
* @param read_len - the data lenth to read
* @param reg - Semaphore handler
*
* @return Returns ESP_OK for success
******************************************************************************/
SemaphoreHandle_t adc_data_ready_reading_from_isr(int32_t *buff, int32_t read_len)
{
#ifndef CONFIG_SPI_MASTER_IN_IRAM
	ESP_LOGE(TAG, "ERROR CONFIG_SPI_MASTER_IN_IRAM no define");
	return NULL;
#endif
	// create a queue to handle gpio event from isr
	if (ready_evt_sem == NULL) {
    	ready_evt_sem = xSemaphoreCreateBinary();
		//change gpio interrupt type for one pin
		gpio_set_intr_type(PIN_NUM_ADC_RDY, GPIO_INTR_NEGEDGE);
		//install gpio isr service
		gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
		//hook isr handler for specific gpio pin
		gpio_isr_handler_add(PIN_NUM_ADC_RDY, gpio_isr_handler, (void*) PIN_NUM_ADC_RDY);

		/**If output data ready signal via MISO pin, should tie low the CS pin*/
		gpio_set_level(PIN_NUM_CS, 0);
	}
	/* Reduce the consumption of SPI reading, which can reduce the time of about 2us */
	spi_device_acquire_bus(spi_for_adc_init, portMAX_DELAY);

	AD717X_Resume();// resume from standby mode.
	DATA_BUFF = buff;
	DATA_BUFF_LEN = read_len;
	DATA_BUFF_CNT = 0;


	return ready_evt_sem;
}

void adc_data_ready_deinit(void)
{
	if (ready_evt_sem) {
    	vSemaphoreDelete(ready_evt_sem);
		ready_evt_sem = NULL;
	}
}

/**************************************************************************//**
* @brief Monitor the adc error state by MISO interrupt.
*        The ERROR pin of AD717X can output the adc error state. So, So enable the AD717X GPIO nagitive edge interrupt to monitor the error state. 
*        User can take the sem to get the data state.
*        Of cause, User also can read `AD717X_STATUS_REG` to get error state.
*
* @note AD7175_regs[IOCon_Register].value |= AD717X_GPIOCON_REG_ERR_EN(2); // error pin ouput adc error status
*
* @param reg - Semaphore handler
*
* @return Returns ESP_OK for success
******************************************************************************/
SemaphoreHandle_t adc_error_notify_init(void)
{
	if (err_evt_sem == NULL) {
    	err_evt_sem = xSemaphoreCreateBinary();
	}
	/*init error number pin*/
	gpio_set_level(PIN_NUM_ADC_ERR, 0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(PIN_NUM_ADC_ERR),
        .mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE, // Note: if err_pin output error status of adc, the pin should pullup outside.
    };
    gpio_config(&cs_cfg);

	//change gpio interrupt type for one pin
    gpio_set_intr_type(PIN_NUM_ADC_ERR, GPIO_INTR_NEGEDGE);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PIN_NUM_ADC_ERR, gpio_isr_handler, (void*) PIN_NUM_ADC_ERR);

	return err_evt_sem;
}

/**CS tie high stop trans the DATA*/
static IRAM_ATTR void cs_high(spi_transaction_t* t)
{
    gpio_ll_set_level(&GPIO, PIN_NUM_CS, 1);
	/**Should enable the MISO/RDY pin interrupt after transmit data by SPI*/
	if(ready_evt_sem) {
		/**If output data ready signal via MISO pin, should tie low the CS pin*/
		gpio_ll_set_level(&GPIO, PIN_NUM_CS, 0);
		gpio_ll_set_intr_type(&GPIO, PIN_NUM_ADC_RDY, GPIO_INTR_NEGEDGE);
		gpio_ll_clear_intr_status(&GPIO, BIT(PIN_NUM_ADC_RDY));
	}
}

/**CS tie low start trans the DATA*/
static IRAM_ATTR void cs_low(spi_transaction_t* t)
{
	/**Should disable the MISO/RDY pin interrupt before transmit data by SPI*/
	if(ready_evt_sem) {
		gpio_ll_clear_intr_status(&GPIO, BIT(PIN_NUM_ADC_RDY));
		gpio_ll_set_intr_type(&GPIO, PIN_NUM_ADC_RDY, GPIO_INTR_DISABLE);
	}
    gpio_ll_set_level(&GPIO, PIN_NUM_CS, 0);
}

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - Idle state for clock is a low level; active
 *                                  state is a high level;
 *	                      0x1 - Idle state for clock is a high level; active
 *                                  state is a low level.
 * @param clockEdg - SPI clock edge (0 or 1).
 *                   Example: 0x0 - Serial output data changes on transition
 *                                  from idle clock state to active clock state;
 *                            0x1 - Serial output data changes on transition
 *                                  from active clock state to idle clock state.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char SPI_Init_AD7175(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg)
{
	esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=32,
		.intr_flags = ESP_INTR_FLAG_IRAM,
    };
    spi_device_interface_config_t devcfg={
		.command_bits = 8,	//8 bit cmd
		.dummy_bits = 0,
        .clock_speed_hz = AD7175_SPI_CLK_SPEED, 	//Clock out at 10 MHz // max 20M
		.input_delay_ns = 0,			//Maximum data valid time of slave
        .mode = 3,                      //SPI mode 3
        .spics_io_num=-1,       //CS pin
        .queue_size=7,                  //We want to be able to queue 7 transactions at a time
		.flags = SPI_DEVICE_HALFDUPLEX,
		.pre_cb = cs_low,
        .post_cb = cs_high,
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(ADC_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

	/*init cs pin*/
	gpio_set_level(PIN_NUM_CS, 0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);

    //Attach the ADC to the SPI bus
    ret=spi_bus_add_device(ADC_HOST, &devcfg, &spi_for_adc_init);
    ESP_ERROR_CHECK(ret);

	return ret;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
inline
unsigned char 
ADC_COMM_DRIVER_ATTR 
SPI_Write(unsigned char slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber)
{
#if LOG_DEBUG
	esp_rom_printf(DRAM_STR("\n[W]cmd %02x, num %d :"), data[0], (int)(bytesNumber-1));
	for(int i=0; i<bytesNumber-1; i++) {
		esp_rom_printf(DRAM_STR"%02x "), data[i+1]);
	}
	esp_rom_printf(DRAM_STR"\n"));
#endif
	// Add your code here.
	esp_err_t ret;
    spi_transaction_t t = {
		.flags = 0,
		.cmd = 0,
		.addr = 0,
		.length = 0,
		.rxlength = 0,
		.user = NULL,
	};
	uint8_t *p = (uint8_t *)dma_access_buf;

	t.cmd = data[0];
	t.length = 8 * (bytesNumber - 1);

	for(int i=0; i<bytesNumber-1; i++) {
		p[i] = data[i+1];
	}
	t.tx_buffer = p;
    ret = spi_device_polling_transmit(spi_for_adc_init, &t);

	return (ret == ESP_OK) ? (bytesNumber) : 0;
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the read buffer.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
inline
unsigned char 
ADC_COMM_DRIVER_ATTR 
SPI_Read(unsigned char slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber)
{
	esp_err_t ret;
    spi_transaction_t t = {
		.flags = 0,
		.cmd = 0,
		.addr = 0,
		.length = 0,
		.rxlength = 0,
		.user = NULL,
	};
	uint8_t *p = dma_access_buf;

	t.cmd = data[0];
	t.rxlength = 8 * (bytesNumber - 1);
	t.rx_buffer = p;
    ret = spi_device_polling_transmit(spi_for_adc_init, &t);
	for(int i=0; i<bytesNumber-1; i++) {
		data[i+1] = p[i];
	}
#if LOG_DEBUG
	esp_rom_printf(DRAM_STR("\n[R]cmd %02x, num %d :"), data[0], (int)(bytesNumber-1));
	for(int i=0; i<bytesNumber-1; i++) {
		esp_rom_printf(DRAM_STR("%02x "), data[i+1]);
	}
	esp_rom_printf(DRAM_STR("\n"));
#endif
	return (ret == ESP_OK) ? (bytesNumber - 1) : 0;
}
