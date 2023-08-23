
/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-TPL0401A";

#define I2C_MASTER_SCL_IO           GPIO_NUM_20      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define TPL0401A_ADDR                 0x2e        /*!< Slave address of the TPL0401A sensor */
#define TPL0401A_CMD                  0x00

/**
 * @brief Read a sequence of bytes from a TPL0401A sensor registers
 */
static esp_err_t TPL0401A_register_read_resistance(uint8_t *res)
{
    uint8_t reg_addr = TPL0401A_CMD;
    return i2c_master_write_read_device(I2C_MASTER_NUM, TPL0401A_ADDR, &reg_addr, 1, res, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}         

/**
 * @brief Write a byte to a TPL0401A sensor register
 */
static esp_err_t TPL0401A_register_change_resistance(uint8_t res)
{
    int ret;
    uint8_t write_buf[2] = {TPL0401A_CMD, res};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, TPL0401A_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void TPL0401A_digital_potentiometer_init(void)
{
    uint8_t data;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    TPL0401A_register_write_byte(0);
    ESP_ERROR_CHECK(TPL0401A_register_read_resistance(&data));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data);

    while(1){
        for(int i=0; i<128; i++) {
            ESP_ERROR_CHECK(TPL0401A_register_change_resistance(i));
            vTaskDelay(20);
        }
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
