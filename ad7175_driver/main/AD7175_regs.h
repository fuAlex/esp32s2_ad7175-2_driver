/**************************************************************************//**
*   @file   AD7175_regs.h
*   @brief  AD7175 Registers Definitions.
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

#ifndef __AD7175_REGS_H__
#define __AD7175_REGS_H__

#include "AD717x_common_regs.h"

/*! AD7175 register info */
typedef struct _st_reg 
{
	int32_t addr;
	int32_t value;
    int32_t size;
}st_reg;

/*! AD7175 registers list*/
enum AD7175_registers
{
	Status_Register = 0x00,
	ADC_Mode_Register,
	Interface_Mode_Register,
	Regs_Check,
	Data_Register,
	IOCon_Register,
	ID_st_reg,
	CH_Map_1,
	CH_Map_2,
	CH_Map_3,
	CH_Map_4,
	Setup_Config_1,
	Setup_Config_2,
	Setup_Config_3,
	Setup_Config_4,
	Filter_Config_1,
	Filter_Config_2,
	Filter_Config_3,
	Filter_Config_4,
	Offset_1,
	Offset_2,
	Offset_3,
	Offset_4,
	Gain_1,
	Gain_2,
	Gain_3,
	Gain_4,
	Communications_Register,
	AD7175_REG_NO
};

#ifdef AD7175_INIT
/*! Array holding the info for the AD7175 registers - address, initial value, size */
st_reg AD7175_regs[] = 
{
    /*0x00*/{AD717X_STATUS_REG		, 0x00,   1}, //Status_Register
    /*0x01*/{AD717X_ADCMODE_REG		, 0x800C, 2}, //ADC_Mode_Register
    /*0x02*/{AD717X_IFMODE_REG		, 0x0104, 2}, //Interface_Mode_Register
	/*0x03*/{AD717X_REGCHECK_REG	, 0x0000, 3}, //REGCHECK
    /*0x04*/{AD717X_DATA_REG		, 0x0000, 3}, //Data_Register
    /*0x06*/{AD717X_GPIOCON_REG		, 0x0400, 2}, //IOCon_Register
    /*0x07*/{AD717X_ID_REG			, 0x0000, 2}, //ID_st_reg
    /*0x10*/{AD717X_CHMAP0_REG		, 0x8232, 2}, //CH_Map_1
	/*0x11*/{AD717X_CHMAP1_REG		, 0x0000, 2}, //CH_Map_2
	/*0x12*/{AD717X_CHMAP2_REG		, 0x0000, 2}, //CH_Map_3
	/*0x13*/{AD717X_CHMAP3_REG		, 0x0000, 2}, //CH_Map_4
	/*0x20*/{AD717X_SETUPCON0_REG	, 0x1320, 2}, //Setup_Config_1
	/*0x21*/{AD717X_SETUPCON1_REG	, 0x0000, 2}, //Setup_Config_2
	/*0x22*/{AD717X_SETUPCON2_REG	, 0x0000, 2}, //Setup_Config_3
	/*0x23*/{AD717X_SETUPCON3_REG	, 0x0000, 2}, //Setup_Config_4
	/*0x28*/{AD717X_FILTCON0_REG	, 0x0001, 2}, //Filter_Config_1
	/*0x29*/{AD717X_FILTCON1_REG	, 0x0200, 2}, //Filter_Config_2
	/*0x2a*/{AD717X_FILTCON2_REG	, 0x0200, 2}, //Filter_Config_3
	/*0x2b*/{AD717X_FILTCON3_REG	, 0x0200, 2}, //Filter_Config_4
	/*0x30*/{AD717X_OFFSET0_REG		, 0, 3}, 		//Offset_1
	/*0x31*/{AD717X_OFFSET1_REG		, 0, 3}, 		//Offset_2
	/*0x32*/{AD717X_OFFSET2_REG		, 0, 3}, 		//Offset_3
	/*0x33*/{AD717X_OFFSET3_REG		, 0, 3}, 		//Offset_4
	/*0x38*/{AD717X_GAIN0_REG		, 0, 3}, 		//Gain_1
	/*0x39*/{AD717X_GAIN1_REG		, 0, 3}, 		//Gain_2
	/*0x3a*/{AD717X_GAIN2_REG		, 0, 3}, 		//Gain_3
	/*0x3b*/{AD717X_GAIN3_REG		, 0, 3}, 		//Gain_4
	/*0xFF*/{0xFF, 0, 1}, 		//Communications_Register
};
#else
extern st_reg AD7175_regs[AD7175_REG_NO];
#endif

#define AD7175_SLAVE_ID    1

/* Communication Register bits */
#define COMM_REG_WEN    (0 << 7)
#define COMM_REG_WR     (0 << 6)
#define COMM_REG_RD     (1 << 6)

/* Status Register bits */
#define STATUS_REG_RDY      (1 << 7)
#define STATUS_REG_ADC_ERR  (1 << 6)
#define STATUS_REG_CRC_ERR  (1 << 5)
#define STATUS_REG_REG_ERR  (1 << 4)
#define STATUS_REG_CH(x)    ((x) & 0x03)

/* ADC Mode Register */
#define ADC_MODE_REG_REF_EN         (1 << 15)
#define ADC_MODE_REG_DELAY(x)       (((x) & 0x7) << 8)
#define ADC_MODE_REG_MODE(x)        (((x) & 0x7) << 4)
#define ADC_MODE_REG_CLKSEL(x))     (((x) & 0x3) << 2)

/* Interface Mode Register bits */
#define INTF_MODE_REG_DOUT_RESET    (1 << 8)
#define INTF_MODE_REG_CONT_READ     (1 << 7)
#define INTF_MODE_REG_DATA_STAT     (1 << 6)
#define INTF_MODE_REG_CRC_EN        (0x02 << 2)
#define INTF_MODE_REG_CRC_STAT(x)   (((x) & INTF_MODE_REG_CRC_EN) == INTF_MODE_REG_CRC_EN)

/* GPIO Configuration Register */
#define GPIO_CONF_REG_MUX_IO        (1 << 12)
#define GPIO_CONF_REG_SYNC_EN       (1 << 11)
#define GPIO_CONF_REG_ERR_EN(x)     (((x) & 0x3) << 9)
#define GPIO_CONF_REG_ERR_DAT       (1 << 8)
#define GPIO_CONF_REG_IP_EN1        (1 << 5)
#define GPIO_CONF_REG_IP_EN0        (1 << 4)
#define GPIO_CONF_REG_OP_EN1        (1 << 3)
#define GPIO_CONF_REG_OP_EN0        (1 << 2)
#define GPIO_CONF_REG_DATA1         (1 << 1)
#define GPIO_CONF_REG_DATA0         (1 << 0)

/* ID Register */
#define ID_REG_PRODUCT_ID(x)        (((x) & 0xFF) << 8)

/* Channel Map Register 1-4 */
#define CH_MAP_REG_CHEN         (1 << 15)
#define CH_MAP_REG_SETUP(x)     (((x) & 0x7) << 12)
#define CH_MAP_REG_AINPOS(x)    (((x) & 0x1F) << 5)    
#define CH_MAP_REG_AINNEG(x)    (((x) & 0x1F) << 0)

/* Setup Configuration Register 1-4 */
#define SETUP_CONF_REG_CHOP_MD(x)       (((x) & 0x3) << 14)
#define SETUP_CONF_REG_BI_UNIPOLAR      (1 << 12)
#define SETUP_CONF_REG_REF_BUF_P        (1 << 11)
#define SETUP_CONF_REG_REF_BUF_N        (1 << 10)
#define SETUP_CONF_REG_AIN_BUF_P        (1 << 9)
#define SETUP_CONF_REG_AIN_BUF_N        (1 << 8)
#define SETUP_CONF_REG_BRNOUT_EN        (1 << 7)
#define SETUP_CONF_REG_REF_SEL(x)       (((x) & 0x3) << 4)

/* Filter Configuration Register 1-4 */
#define FILT_CONF_REG_EXTCLKFREQ(x)     (((x) & 0x3) << 13)
#define FILT_CONF_REG_ENHFILTEN         (1 << 11)
#define FILT_CONF_REG_ENHFILTSEL(x)     (((x) & 0x7) << 8)
#define FILT_CONF_REG_ORDER(x)          (((x) & 0x7) << 5)
#define FILT_CONF_REG_ODR(x)            (((x) & 0x1F) << 0)


#endif //__AD7175_REGS_H__
