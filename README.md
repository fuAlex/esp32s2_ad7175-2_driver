# esp32s2_ad7175-2_driver
AD7175-2 work with ESP32-S2

# Feature:

1. Support the fastest data output frequency of 500KHz (`adc_data_ready_reading_from_isr()`).
2. Support ADC ERROR detect
3. ADC select ext xtal for clock source.
4. Accurate data output time. Can save main CPU timing resources. 
5. When reading ADC data at high speed（500KHz）, perform other tasks after waiting.

# HW connect:

```C
AD7175-2        <-> ESP32S2 PIN
PIN_NUM_MISO 		37
PIN_NUM_MOSI 		35
PIN_NUM_CLK  		36
PIN_NUM_CS   		34
PIN_NUM_ADC_ERR   	33			 // Monitor the ADC error status. 1: normal; 0: error.
PIN_NUM_ADC_RDY   	PIN_NUM_MISO // Monitor the ADC data ready status. 1: busy; 0: ready.

AD7175_SPI_CLK_SPEED (20 * 1000 * 1000) // MAX 20 MHz for AD7175-2
```

# 文件结构：

|_ AD7175.c         // AD7175 driver to config Register  
|_ AD7175_regs.h    // Define AD7175 register  
|_ Communication.c  // SPI driver for AD7175  
|_ main.c           // Demo  
