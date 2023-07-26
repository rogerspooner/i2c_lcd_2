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
#include <rom/ets_sys.h>

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          40000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7
#define LCD_DISPLAY_ADDR                    0x27        /*!< Slave address of the LCD display */

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t addr)
{
    esp_err_t ret;
    uint8_t write_buf[5] = {0x55, 0x4d, 0x33, 0x0f, 0xff}; // binary bit patterns 01010101 and 01001101 etc

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret == ESP_OK)
        ESP_LOGI(TAG, "Data written OK to address x%x", addr);
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
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t init_lcd_display(void)
{  /* THIS WON'T WORK */
    esp_err_t ret;
    uint8_t write_buf[] = {
         0x01 // clear screen, 
        ,0x0F // display on, cursor on
        ,0x02 // home
        ,0x06 // entry mode set reading left to right
        ,0x80 // set DDRAM address to 0x00 ready for text
    };
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_DISPLAY_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    /*
    The HLF8574T chip is an 8 bit expander from I2C to parallel. But the LCD display requires about 11 pins.
    Fortunately it has a 4 bit mode, so we can probably manage with the 8 bits available.
    Only the most significant nybble is used in 4 bit mode.
    "Function set" command needs the key bits in the most significant nybble too, although  it also takes less critical data in the least significant nybble.
    The other 4 bits from the 8574T chip are used for:
    - P0 = register select. Like a 1 bit address bus for the LCD.
    - P1 = read/write. 0 = write, 1 = read
    - P2 = enable operation
    It looks like the sequence to write data to the LCD will be by many i2c writes:
    - write to address of i2c chip (0x27)
    - assert 4 bits in upper half of byte. In lower half, correct register select and read/write but not enable operation yet.
    - same byte but asserting enable
    - same byte but de-asserting enable
    - assert least signficant 4 bits in upper half of byte. In lower half, same routine as bove.
    - enable
    - de-assert enable
    - repeat for further bytes.
    
    References
    https://components101.com/sites/default/files/component_datasheet/16x2%20LCD%20Datasheet.pdf
    https://www.openhacks.com/uploadsproductos/eone-1602a1.pdf
    https://www.ti.com/lit/ds/symlink/pcf8574.pdf
    https://forum.microchip.com/s/topic/a5C3l000000MYoLEAW/t363873
    https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library/blob/master/LiquidCrystal_I2C.cpp#L233

    */
    return ret;
}

static esp_err_t write_lcd_display(char *str)
{
    return ESP_FAIL;
}

void app_main(void)
{
    // uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 * /
    ESP_ERROR_CHECK(mpu9250_register_read(MPU9250_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    */

    /* Demonstrate writing */
    while (true)
    {
        /*
        ESP_LOGI(TAG, "Testing all I2C addresses");
        for (uint8_t addr = 0x26; addr < 0x7F; addr++)
        { esp_err_t err = mpu9250_register_write_byte(addr);
            if (err == ESP_OK)
            { ESP_LOGI(TAG, "Device x%x written", addr);
            }
            ets_delay_us(100);
            if (addr == 0x28) addr = 0x67; // expecting to find address 0x27 and 0x68
        }
        */
        ESP_LOGI(TAG, "Finished scanning. Now for LCD display");
        init_lcd_display();
        write_lcd_display("Hello World");
        vTaskDelay(5000 / portTICK_PERIOD_MS); // wait 5s
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
