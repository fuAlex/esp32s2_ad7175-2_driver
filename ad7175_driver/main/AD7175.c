/**************************************************************************//**
*   @file   AD7175.c
*   @brief  AD7175 implementation file.
*   @author acozma (andrei.cozma@analog.com)
*
*******************************************************************************
* Copyright 2011(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
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
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
*   SVN Revision: 0
******************************************************************************/

#define AD7175_INIT

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "Communication.h"
#include "AD7175.h"

static const char *TAG = "adc";

#ifdef CONFIG_SPI_MASTER_IN_IRAM
#define ADC_COMM_DRIVER_ATTR IRAM_ATTR
#else
#define ADC_COMM_DRIVER_ATTR
#endif

/******************************************************************************/
/************************ Local variables and types ***************************/
/******************************************************************************/
struct AD7175_state {
    uint8_t useCRC;
} DRAM_ATTR AD7175_st;

/**************************************************************************//**
* @brief Reads the value of the specified register
*
* @param pReg - Pointer to the register structure holding info about the 
*               register to be read. The read value is stored inside the 
*               register structure.
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
int32_t
ADC_COMM_DRIVER_ATTR
AD7175_ReadRegister(st_reg* pReg)
{
    int32_t ret       = 0;
    uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i         = 0;
    uint8_t crc       = 0;

    /* Build the Command word */
    buffer[0] = COMM_REG_WEN | COMM_REG_RD | pReg->addr;
    
    /* Read data from the device */
    ret = SPI_Read(AD7175_SLAVE_ID, 
                   buffer, 
                   (AD7175_st.useCRC ? pReg->size + 1 : pReg->size) + 1);
    if(ret <= 0) {
        esp_rom_printf(DRAM_STR("READ ERROR\n"));
        return ESP_FAIL;
    }
    /* Check the CRC */
    if(AD717X_IFMODE_REG_CRC_STAT(AD7175_st.useCRC)) {
        crc = AD717X_ComputeCRC8(buffer, pReg->size + 2);
        if(crc) {
            esp_rom_printf(DRAM_STR("CRC ERROR\n"));
            return ESP_ERR_INVALID_CRC;
        }
    }
    if(AD717X_IFMODE_REG_XOR_STAT(AD7175_st.useCRC)) {
		crc = AD717X_ComputeXOR8(buffer, pReg->size + 2);
        if(crc) {
            for(int i=0; i<pReg->size + 2; i++) {
                esp_rom_printf(DRAM_STR("[%x]"), buffer[i]);
            }
            esp_rom_printf(DRAM_STR("CRC ERROR crc %x\n"), crc);
            return ESP_ERR_INVALID_CRC;
        }
	}

    /* Build the result */
    pReg->value = 0;
    for(i = 1; i < pReg->size + 1; i++) {
        pReg->value <<= 8;
        pReg->value |= buffer[i];
    }

    return ESP_OK;
}

/**************************************************************************//**
* @brief Writes the value of the specified register
*
* @param reg - Register structure holding info about the register to be written
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
int32_t
ADC_COMM_DRIVER_ATTR
AD7175_WriteRegister(st_reg reg)
{
    int32_t ret      = 0;
    int32_t regValue = 0;
    uint8_t wrBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i        = 0;
    uint8_t crc      = 0;
    
    /* Build the Command word */
    wrBuf[0] = COMM_REG_WEN | COMM_REG_WR | reg.addr;
    
    /* Fill the write buffer */
    regValue = reg.value;
    for(i = 0; i < reg.size; i++) {
        wrBuf[reg.size - i] = regValue & 0xFF;
        regValue >>= 8;
    }

    /* Compute the CRC */
    if(AD7175_st.useCRC) {
        crc = AD717X_ComputeCRC8(wrBuf, reg.size + 1);
		wrBuf[reg.size + 1] = crc;
        
    }

    /* Write data to the device */
    ret = SPI_Write(AD7175_SLAVE_ID,
                    wrBuf,
                    AD7175_st.useCRC ? reg.size + 2 : reg.size + 1);

    return (ret > 0) ? ESP_OK : ESP_FAIL;
}

/**************************************************************************//**
* @brief Waits until a new conversion result is available
*
* @param timeout - Count representing the number of polls to be done until the
*                  function returns if no new data is available. 
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
int32_t AD7175_WaitForReady(uint32_t timeout)
{
    int32_t ret = 0;
    int8_t ready = 0;

    while(!ready && --timeout)
    {
        /* Read the value of the Status Register */
        ret = AD7175_ReadRegister(&AD7175_regs[Status_Register]);
        /* Check the RDY bit in the Status Register */
        ready = (AD7175_regs[Status_Register].value & STATUS_REG_RDY) != 0;
    }

    return ret;
}

int32_t AD7175_GetState(uint8_t *st)
{
    int32_t ret = 0;

    /* Read the value of the Status Register */
    ret = AD7175_ReadRegister(&AD7175_regs[Status_Register]);
    *st = AD7175_regs[Status_Register].value;

    return ret; 
}

/**************************************************************************//**
* @brief Reads the conversion result from the device.
*
* @param pData - Pointer to store the read data.
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
inline
int32_t
ADC_COMM_DRIVER_ATTR
AD7175_ReadData(int32_t* pData)
{
    int32_t ret;

    /* Read the value of the Status Register */
    ret = AD7175_ReadRegister(&AD7175_regs[Data_Register]);
    /* Get the read result */
    *pData = AD7175_regs[Data_Register].value;

    return ret;
}

/***************************************************************************//**
* @brief Computes the CRC checksum for a data buffer.
*
* @param pBuf    - Data buffer
* @param bufSize - Data buffer size in bytes
*
* @return Returns the computed CRC checksum.
*******************************************************************************/
inline
uint8_t
ADC_COMM_DRIVER_ATTR
AD717X_ComputeCRC8(uint8_t * pBuf,
			   uint8_t bufSize)
{
	uint8_t i   = 0;
	uint8_t crc = 0;

	while(bufSize) {
		for(i = 0x80; i != 0; i >>= 1) {
			if(((crc & 0x80) != 0) != ((*pBuf & i) !=
						   0)) { /* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD7175_CRC_POLYNOMIAL;
			} else {
				crc <<= 1;
			}
		}
		pBuf++;
		bufSize--;
	}
	return crc;
}
/***************************************************************************//**
* @brief Computes the XOR checksum for a data buffer.
*
* @param pBuf    - Data buffer
* @param bufSize - Data buffer size in bytes
*
* @return Returns the computed XOR checksum.
*******************************************************************************/
inline
uint8_t
ADC_COMM_DRIVER_ATTR
AD717X_ComputeXOR8(uint8_t * pBuf, uint8_t bufSize)
{
	uint8_t xor = 0;

	while(bufSize) {
		xor ^= *pBuf;
		pBuf++;
		bufSize--;
	}
	return xor;
}

/***************************************************************************//**
* @brief Resets the device.
*
* @param device - The handler of the instance of the driver.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD717X_Reset(void)
{
	int32_t ret = 0;
	uint8_t wrBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    ret = SPI_Write(AD7175_SLAVE_ID,
                    wrBuf,
                    8);

	return ret;
}

/***************************************************************************//**
* @brief AD717X go to standby mode.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
inline
esp_err_t
ADC_COMM_DRIVER_ATTR
AD717X_Standby(void)
{
    AD7175_regs[ADC_Mode_Register].value &= (~(AD717X_ADCMODE_REG_MODE(0x7)));
    AD7175_regs[ADC_Mode_Register].value |= (AD717X_ADCMODE_REG_MODE(0x2));
    return AD7175_WriteRegister(AD7175_regs[ADC_Mode_Register]);
}

/***************************************************************************//**
* @brief AD717X resume from standby mode.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
inline
esp_err_t
ADC_COMM_DRIVER_ATTR
AD717X_Resume(ad_mode_type_t work_mode)
{
    if (work_mode > ADC_WORK_MODE_SINGLE) {
        return ESP_FAIL;
    }
    AD7175_regs[ADC_Mode_Register].value &= (~(AD717X_ADCMODE_REG_MODE(0x7)));
    AD7175_regs[ADC_Mode_Register].value |= (AD717X_ADCMODE_REG_MODE(work_mode));
    return AD7175_WriteRegister(AD7175_regs[ADC_Mode_Register]);
}

/***************************************************************************//**
* @brief AD717X set data output freq for channel.
*
* @note The output frequency is related to filter selection and register SING_SYS.
*
* @param freq_hz refer to `ad7172_sin5_sin1_sing_sys_freq_hz_t` or `ad7172_sin5_sin1_freq_hz_t`
* @param channel 0, 1, 2, 3
* 
* @return Returns 0 for success or negative error code.
*******************************************************************************/
esp_err_t AD717X_Data_output_freq(int channel, int freq_hz)
{
    if (freq_hz >= AD7172_FREQ_MAX) {
        printf("freq not support\n");
        return ESP_FAIL;
    }
    switch(channel) {
    case 0:
        AD7175_regs[Filter_Config_1].value &= (~AD717X_FILT_CONF_REG_ODR(0x1f));
        AD7175_regs[Filter_Config_1].value |= AD717X_FILT_CONF_REG_ODR(freq_hz);
        return AD7175_WriteRegister(AD7175_regs[Filter_Config_1]);
    case 1:
        AD7175_regs[Filter_Config_2].value &= (~AD717X_FILT_CONF_REG_ODR(0x1f));
        AD7175_regs[Filter_Config_2].value |= AD717X_FILT_CONF_REG_ODR(freq_hz);
        return AD7175_WriteRegister(AD7175_regs[Filter_Config_2]);
    case 2:
        AD7175_regs[Filter_Config_3].value &= (~AD717X_FILT_CONF_REG_ODR(0x1f));
        AD7175_regs[Filter_Config_3].value |= AD717X_FILT_CONF_REG_ODR(freq_hz);
        return AD7175_WriteRegister(AD7175_regs[Filter_Config_3]);
    case 3:
        AD7175_regs[Filter_Config_4].value &= (~AD717X_FILT_CONF_REG_ODR(0x1f));
        AD7175_regs[Filter_Config_4].value |= AD717X_FILT_CONF_REG_ODR(freq_hz);
        return AD7175_WriteRegister(AD7175_regs[Filter_Config_4]);
    }
    return ESP_OK;
}

/***************************************************************************//**
* @brief AD717X calibration.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
inline
esp_err_t
AD717X_Calibration(ad_mode_type_t cal_type)
{
    if (cal_type < ADC_CAL_INTER_OFFSET || cal_type > ADC_CAL_SYS_GAIN) {
        return ESP_FAIL;
    }
    AD7175_regs[ADC_Mode_Register].value &= (~(AD717X_ADCMODE_REG_MODE(0x7)));
    AD7175_regs[ADC_Mode_Register].value |= (AD717X_ADCMODE_REG_MODE(cal_type));
    return AD7175_WriteRegister(AD7175_regs[ADC_Mode_Register]);
}

/**************************************************************************//**
* @brief Initializes the AD7175 
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
int32_t AD7175_Setup(void)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    /* Initialize the SPI communication */
    SPI_Init_AD7175(0, 8000000, 1, 0);

    AD717X_Reset();
    esp_rom_delay_us(500);

    /* Read ID register */
    ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[ID_st_reg]) );
    printf("device id: %04lx\r\n", (uint32_t)AD7175_regs[ID_st_reg].value);

    /* Initialize ADC mode register */
    AD7175_regs[ADC_Mode_Register].value |= (AD717X_ADCMODE_SING_CYC);
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[ADC_Mode_Register]) );

    /* Initialize Interface mode register */
    AD7175_regs[Interface_Mode_Register].value |= (AD717X_IFMODE_REG_XOR_EN | AD717X_IFMODE_REG_DOUT_RESET);
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Interface_Mode_Register]) );
    AD7175_st.useCRC = AD717X_IFMODE_REG_XOR_EN;

    /* Initialize GPIO configuration register */
    AD7175_regs[IOCon_Register].value |= AD717X_GPIOCON_REG_ERR_EN(2); // error pin ouput adc error status
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[IOCon_Register]) );
    
    /* Initialize Setup Configuration registers */
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Setup_Config_1]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Setup_Config_2]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Setup_Config_3]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Setup_Config_4]) );

    /* Initialize Filter Configuration registers */
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Filter_Config_1]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Filter_Config_2]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Filter_Config_3]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Filter_Config_4]) );

    /* Initialize Channel Map registers */
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[CH_Map_1]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[CH_Map_2]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[CH_Map_3]) );
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[CH_Map_4]) );

    ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[ADC_Mode_Register]) );
    printf("ADC MODE val %04lx\n", (uint32_t)AD7175_regs[ADC_Mode_Register].value);
    
    ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[Interface_Mode_Register]) );
    printf("IF mode val %04lx\n", (uint32_t)AD7175_regs[Interface_Mode_Register].value);

    ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[CH_Map_1]) );
    printf("CH1 MAP val %04lx\n", (uint32_t)AD7175_regs[CH_Map_1].value);
    
    ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[Setup_Config_1]) );
    printf("setup 1 val %04lx\n", (uint32_t)AD7175_regs[Setup_Config_1].value);

    ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[IOCon_Register]) );
    printf("IOCON val %04lx\n", (uint32_t)AD7175_regs[IOCon_Register].value);

    // Calibration
    uint64_t offset_sum = 0;
    for (int i=0; i<16; i++) {
        AD717X_Calibration(ADC_CAL_INTER_OFFSET);
        vTaskDelay(10);
        ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[Offset_1]) );
        offset_sum += AD7175_regs[Offset_1].value;
        // printf("Offset_1 val %04lx\n", (uint32_t)AD7175_regs[Offset_1].value);
    }
    AD7175_regs[Offset_1].value = (int32_t)offset_sum / 16;
    ESP_ERROR_CHECK( AD7175_WriteRegister(AD7175_regs[Offset_1]) );
    ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[Offset_1]) );
    printf("Offset_1 val %04lx\n", (uint32_t)AD7175_regs[Offset_1].value);
    ESP_ERROR_CHECK( AD7175_ReadRegister(&AD7175_regs[Gain_1]) );
    printf("Gain_1 val %04lx\n", (uint32_t)AD7175_regs[Gain_1].value);

    AD717X_Resume(ADC_WORK_MODE_CONTINUOUS);

    return ESP_OK;
}
