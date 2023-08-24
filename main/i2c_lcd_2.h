#include "esp_err.h"


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
#define LCD_BACKLIGHT                       0x08        /* P3 of the i2c expander chip goes to the light. */

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */


/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void);

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
esp_err_t lcd_send_i2c_4bit(uint8_t *data, size_t size, uint8_t register_select);

esp_err_t lcd_set_4bit_mode(void);

esp_err_t lcd_init_display(void);

esp_err_t lcd_write(char *str,int linenum);

esp_err_t i2c_lcd_deinit(void);

