/***************************************************************************//**
 *   @file   Communication.h
 *   @brief  Header file of Communication Driver.
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
#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/


/******************************************************************************/
/**************************** GPIO Definitions ********************************/
/******************************************************************************/
#define PMOD1_CS_PIN        // Add your code here.
#define PMOD1_CS_PIN_OUT    // Add your code here.
#define PMOD1_CS_LOW        // Add your code here.
#define PMOD1_CS_HIGH       // Add your code here.

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/*! Initializes the SPI communication peripheral. */
unsigned char SPI_Init_AD7175(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg);
					   
/*! Writes data to SPI. */
unsigned char SPI_Write(unsigned char slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber);
						
/*! Reads data from SPI. */
unsigned char SPI_Read(unsigned char slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber);

/**************************************************************************//**
* @brief Monitor the ready state by MISO interrupt.
*        When CS line is low and MISO line no data to transmit. the MISO will output ready info of adc data (low status is ready).
*        So enable the MISO pin nagitive edge interrupt to monitor the ready state. User can take the sem to get the data state.
*        Of cause, User also can read `AD717X_STATUS_REG` to get ready state.
*
* @note When use the SPI to transmit data, should disable the MISI pin gpio interrupt.
* @note Must make sure the SPI reading time smaller than internal of data output.
*
* @param reg - Semaphore handler
*
* @return Returns ESP_OK for success
******************************************************************************/
SemaphoreHandle_t adc_data_ready_notify_init(void);
SemaphoreHandle_t adc_data_ready_reading_from_isr(int32_t *buff, int32_t read_len);
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
SemaphoreHandle_t adc_error_notify_init(void);

/** 
 * Example code for adc notify init
 * 
 * static void gpio_task_example(void* arg)
 * {
 *     uint32_t io_num;
 * 	SemaphoreHandle_t *get_err_sem = NULL;
 * 	SemaphoreHandle_t *get_rdy_sem = NULL;
 * 	adc_data_ready_notify_init(&get_rdy_sem);
 * 	adc_error_notify_init(&get_err_sem);
 * 
 *     for(;;) {
 *         if (pdTRUE == xSemaphoreTake(*get_rdy_sem, portMAX_DELAY)) {
 *              // data can be read
 *         }
 * 		vTaskDelay(1);
 * 		if(pdTRUE == xSemaphoreTake(*get_err_sem, portMAX_DELAY)) {
 * 			esp_rom_printf("crc error\n"); //out range or CRC err
 *         }
 *     }
 * }
*/
#endif /*__COMMUNICATION_H__*/
