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
#include "esp_err.h"
#include "driver/i2c.h"
#include <rom/ets_sys.h>
#include "i2c_lcd_2.h"

static const char *TAG = "lcd_i2c";
static bool gBackLight_en;

/*
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
*/

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    gBackLight_en = true;

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
esp_err_t lcd_send_i2c_4bit(uint8_t *data, size_t size, uint8_t register_select)
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
        if (gBackLight_en) *pWrite |= LCD_BACKLIGHT;            
        *(pWrite+1) = *pWrite;
        pWrite++;
        if (gBackLight_en) *pWrite |= LCD_BACKLIGHT;            
        *pWrite |= LCD_ENABLECLOCK_4BIT;
        if (gBackLight_en) *pWrite |= LCD_BACKLIGHT;            
        *(pWrite+1) = *pWrite;
        pWrite++;
        if (gBackLight_en) *pWrite |= LCD_BACKLIGHT;            
        *pWrite++ ^= LCD_ENABLECLOCK_4BIT;
        *pWrite = (*pRead & 0x0F) << 4; // least significant 4 bits, in upper half of byte
        if (register_select)
            *pWrite |= LCD_REGISTER_SELECT_4BIT;
        if (gBackLight_en) *pWrite |= LCD_BACKLIGHT;            
        *(pWrite+1) = *pWrite;
        pWrite++;
        if (gBackLight_en) *pWrite |= LCD_BACKLIGHT;            
        *pWrite |= LCD_ENABLECLOCK_4BIT;
        *(pWrite+1) = *pWrite;
        pWrite++;
        if (gBackLight_en) *pWrite |= LCD_BACKLIGHT;            
        *pWrite++ ^= LCD_ENABLECLOCK_4BIT;
        pRead++;
        if (gBackLight_en) *pWrite |= LCD_BACKLIGHT;            
    }
    data4bit_size = pWrite - data4bit;
    // logDumpBytes(TAG, "lcd_send_i2c original 8bit:", data, size);
    // logDumpBytes(TAG, "lcd_send_i2c_4bit: ", data4bit, data4bit_size);
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_DISPLAY_ADDR, data4bit, data4bit_size, I2C_MASTER_TIMEOUT_TICKS);
    if (buf_malloced)
        free(data4bit);
    return ret;
}

esp_err_t lcd_set_4bit_mode(void)
{
    esp_err_t ret;
    uint8_t write_buf[] = {
         0x30 | LCD_ENABLECLOCK_4BIT, 0x30 // set 8 bit mode , in case we were previously in 4 bit mode, first half. Clock on falling edge
        ,0x30 | LCD_ENABLECLOCK_4BIT, 0x30 // set 8 bit mode again. Hopefully ignored
        ,0x00 // clear all lines down
    };
    uint8_t write_buf2[] = {
         0x30 | LCD_ENABLECLOCK_4BIT // set 8 bit mode , in case we were previously in 4 bit mode, second half
        ,0x30 // hold after clock edge
        ,0x30 | LCD_ENABLECLOCK_4BIT, 0x30 // set 8 bit mode , in case we were previously in 4 bit mode, in case we were 1 byte in to a 4 bit command
        ,0x20 | LCD_ENABLECLOCK_4BIT, 0x20  // set 4 bit mode
        ,0x00 // clear all lines down
        // now in 4 bit mode
    };
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_DISPLAY_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_TICKS);
    // logDumpBytes(TAG, "init LCD setting bit mode",write_buf, sizeof(write_buf));
    vTaskDelay(2); // wait more than 4.1ms
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_DISPLAY_ADDR, write_buf2, sizeof(write_buf2), I2C_MASTER_TIMEOUT_TICKS);
    // logDumpBytes(TAG, "init LCD setting bit mode pt2",write_buf2, sizeof(write_buf2));
    return ret;
}

esp_err_t lcd_init_display(void)
{   esp_err_t ret;
    uint8_t init_buf[] = {
         0x28  // Function Set 4 bit mode again to reconfigure line size and font. N=1 > 2 line display. F=0 => 5x8 pixel chars
        ,0x08  // Display off (from HD47780 datasheet)
        ,0x01  // Display off. init from HD47780 data sheet
        ,0x06  // Init I/D=increment, S= no shift 
        ,0x0c  // Cursor or shift Display.  D=display on, C=cursor off, B=blink off
        ,0x02  //  Home LSB and enable backlight
        ,0x80  // set DDRAM address to 0x00 ready for text
    };
    ret = lcd_send_i2c_4bit(init_buf, sizeof(init_buf), LCD_COMMAND_REGISTER);
    return ret;
}

esp_err_t lcd_write(char *str,int linenum)
{
    esp_err_t ret;
    if (strlen(str) > 40) return ESP_ERR_INVALID_SIZE;
    ESP_LOGI(TAG, "lcd_write: %s", str);
    uint8_t ddaddr_buf[2];
    ddaddr_buf[0] = 0x80 | (linenum * 0x40);
    ddaddr_buf[1] = 0x00;
    ret = lcd_send_i2c_4bit(ddaddr_buf, 1, LCD_COMMAND_REGISTER);
    ret = lcd_send_i2c_4bit((uint8_t*) str, strlen(str), LCD_DATA_REGISTER);
    return ret;
}

esp_err_t i2c_lcd_deinit(void)
{
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized");
    return ESP_OK;
}