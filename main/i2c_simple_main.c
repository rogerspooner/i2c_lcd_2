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
#include <string.h>
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
#define I2C_MASTER_TIMEOUT_TICKS       1

#define LCD_DISPLAY_ADDR                    0x27        /*!< Slave address of the LCD display */
#define LCD_ENABLECLOCK_4BIT                0x04        /* Flag for enable (clock) bit in mapping in HLF8574T chip to LCD adaptor */
#define LCD_READWRITE_4BIT                  0x02        /* Flag for read/write bit in mapping in HLF8574T chip to LCD adaptor */
#define LCD_REGISTER_SELECT_4BIT            0x01        /* Flag for register select bit in mapping in HLF8574T chip to LCD adaptor */
#define LCD_COMMAND_REGISTER                0x00        /* Flag for command register in mapping in 1602 LCD */
#define LCD_DATA_REGISTER                   0x01        /* Flag for data register in mapping in 1602 LCD */

#define BLACK_BUTTON_GPIO 26
#define GREEN_LED_GPIO 5

// example of use: (TAG, "lcd_send_i2c_4bit", data4bit, data4bit_size);
static void logDumpBytes(const char *tag, const char *msg, uint8_t *data, size_t size)
{
    static char buf[1024];
    char *p = buf;
    sprintf(p, "%s: ", msg);
    p += strlen(p);
    if (size > 32)
        size = 32;
    for (int i = 0; i < size; i++)
    {
        sprintf(p, "%02x ", data[i]);
        p += strlen(p);
    }
    ESP_LOGI(tag, "%s", buf);
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

/**
 * @brief Send data to the LCD display in 4 bit mode through the 8-bit expander on i2c.
 * 
 * The HLF8574T chip is an 8 bit expander from I2C to parallel. But the LCD display requires about 11 pins.
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
    - https://components101.com/sites/default/files/component_datasheet/16x2%20LCD%20Datasheet.pdf
    - https://www.openhacks.com/uploadsproductos/eone-1602a1.pdf
    - https://www.ti.com/lit/ds/symlink/pcf8574.pdf
    - https://forum.microchip.com/s/topic/a5C3l000000MYoLEAW/t363873
    - https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library/blob/master/LiquidCrystal_I2C.cpp#L233
    */
static esp_err_t lcd_send_i2c_4bit(uint8_t *data, size_t size, uint8_t register_select)
{
#define INITIAL_DATA4BIT_SIZE 256
    esp_err_t ret;
    bool buf_malloced = false;
    static uint8_t staticData4bit[INITIAL_DATA4BIT_SIZE];
    static uint8_t* data4bit = staticData4bit;
    int data4bit_size = size*6;
    uint8_t *pWrite, *pRead, *pEnd;
    if ((size*6) < INITIAL_DATA4BIT_SIZE )
        data4bit = staticData4bit;
    else
    {  
        data4bit = (uint8_t*) malloc(size*2+16);
        buf_malloced = true;
    }
    pWrite = data4bit;
    pRead = data;
    pEnd = data + size;
    while (pRead < pEnd)
    {
        *pWrite= (*pRead & 0xF0); // most significant 4 bits, in upper half of byte
        if (register_select)
            *pWrite |= LCD_REGISTER_SELECT_4BIT;
        *(pWrite+1) = *pWrite;
        pWrite++;
        *pWrite |= LCD_ENABLECLOCK_4BIT;
        *(pWrite+1) = *pWrite;
        pWrite++;
        *pWrite++ ^= LCD_ENABLECLOCK_4BIT;
        *pWrite = (*pRead & 0x0F) << 4; // least significant 4 bits, in upper half of byte
        if (register_select)
            *pWrite |= LCD_REGISTER_SELECT_4BIT;
        *(pWrite+1) = *pWrite;
        pWrite++;
        *pWrite |= LCD_ENABLECLOCK_4BIT;
        *(pWrite+1) = *pWrite;
        pWrite++;
        *pWrite++ ^= LCD_ENABLECLOCK_4BIT;
        pRead++;
    }
    data4bit_size = pWrite - data4bit;
    logDumpBytes(TAG, "lcd_send_i2c original 8bit:", data, size);
    logDumpBytes(TAG, "lcd_send_i2c_4bit: ", data4bit, data4bit_size);
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_DISPLAY_ADDR, data4bit, data4bit_size, I2C_MASTER_TIMEOUT_TICKS);
    if (buf_malloced)
        free(data4bit);
    return ret;
}

static esp_err_t lcd_set_4bit_mode(void)
{
    esp_err_t ret;
    uint8_t write_buf[] = {
         0x30 // set 8 bit mode , in case we were previously in 4 bit mode, first half
        ,0x30 | LCD_ENABLECLOCK_4BIT
        ,0x30
        ,0x30 // set 8 bit mode , in case we were previously in 4 bit mode, second half
        ,0x30 | LCD_ENABLECLOCK_4BIT
        ,0x30
        ,0x30 // set 8 bit mode , in case we were previously in 4 bit mode, in case we were 1 byte in to a 4 bit command
        ,0x30 | LCD_ENABLECLOCK_4BIT
        ,0x30
        ,0x20 // set 4 bit mode
        ,0x20 | LCD_ENABLECLOCK_4BIT
        ,0x20
    };
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_DISPLAY_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_TICKS);
    return ret;
}

static esp_err_t lcd_init_display(void)
{   esp_err_t ret;
    uint8_t write_buf[] = {
        ,0x0F // display on, cursor on
        ,0x2c // function set, 4 bit mode, 2 lines, 5x11 font
        //,0x01 // clear screen,  (slow)
        //,0x02 // home (slow)
        ,0x06 // entry mode set reading left to right
        ,0x80 // set DDRAM address to 0x00 ready for text
    };
    ret = lcd_send_i2c_4bit(write_buf, sizeof(write_buf), LCD_COMMAND_REGISTER);
    return ret;
}

static esp_err_t lcd_write(char *str)
{
    esp_err_t ret;
    if (strlen(str) > 40)
        return ESP_ERR_INVALID_SIZE;
    ESP_LOGI(TAG, "lcd_write: %s", str);
    ret = lcd_send_i2c_4bit((uint8_t*) str, strlen(str), LCD_DATA_REGISTER);
    return ret;
}

void app_main(void)
{
    esp_err_t err;
    err = i2c_master_init();
    if (err != ESP_OK)
    { ESP_LOGE(TAG, "Error x%x initializing I2C", err);
    }
    
    while (true)
    {
        gpio_set_direction(GREEN_LED_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_level(GREEN_LED_GPIO, 0);
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
        ESP_LOGI(TAG, "Setting 4 bit mode on LCD by I2C");
        err = lcd_set_4bit_mode();
        if (err != ESP_OK)
        { ESP_LOGE(TAG, "Error x%x setting 4 bit mode", err);
        }
        /*
        err = lcd_init_display();
        if (err != ESP_OK)
        { ESP_LOGE(TAG, "Error x%x initializing display", err);
        }
        */
        err = lcd_write("Hello World");
        if (err != ESP_OK)
        { ESP_LOGE(TAG, "Error x%x writing to display", err);
        }
        else {
           ESP_LOGI(TAG, "Wrote to display");
        }
        gpio_set_level(GREEN_LED_GPIO, 1);
        gpio_set_direction(BLACK_BUTTON_GPIO, GPIO_MODE_INPUT);
        gpio_pullup_en(BLACK_BUTTON_GPIO);
        ESP_LOGI(TAG, "Press the black button again");
        while (gpio_get_level(BLACK_BUTTON_GPIO) == 1) // wait for button press
        { vTaskDelay(10); 
        }
        vTaskDelay(10); 
        while (gpio_get_level(BLACK_BUTTON_GPIO) == 0) // wait for button press
        { vTaskDelay(10); 
        }
        gpio_set_level(GREEN_LED_GPIO, 0);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
