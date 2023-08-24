
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <rom/ets_sys.h>

#include "i2c_lcd_2.h"
#include "stepper_motor.h"
#include "keyboard_scan.h"

#define BLACK_BUTTON_GPIO 0
#define GREEN_LED_GPIO 5
// We use many GPIOs in keyboard_scan.c
#define LOG_TAG "main"

static int lineCount;

void app_init(void)
{
    lineCount = 0;}

void app_main(void)
{
    esp_err_t err;
    app_init();
    err = i2c_master_init();
    if (err != ESP_OK)
    { ESP_LOGE(LOG_TAG, "Error x%x initializing I2C", err);
    }
    gpio_set_direction(GREEN_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(GREEN_LED_GPIO, 1);
    ESP_LOGI(LOG_TAG, "Setting 4 bit mode on LCD by I2C");
    err = lcd_set_4bit_mode();
    if (err != ESP_OK) ESP_LOGE(LOG_TAG, "Error x%x setting 4 bit mode", err);
    err = lcd_init_display();
    if (err != ESP_OK) ESP_LOGE(LOG_TAG, "Error x%x initializing display", err);
    // stepper_motor_init(0);
    err = keyboard_init();
    if (err != ESP_OK)
    { ESP_LOGE(LOG_TAG, "Error x%x initializing keyboard", err);
    }
    while (true)
    {
        char line1[40];
        sprintf(line1, "Ahoy Roger %d", lineCount);
        err = lcd_write(line1,0);
        err = lcd_write("Press the button", 1);
        if (err != ESP_OK)
        { ESP_LOGE(LOG_TAG, "Error x%x writing to display", err);
        }
        else {
           ESP_LOGI(LOG_TAG, "Wrote to display");
        }
        keyboard_scan();
        
        gpio_set_level(GREEN_LED_GPIO, 0); // active low
        gpio_set_direction(BLACK_BUTTON_GPIO, GPIO_MODE_INPUT);
        gpio_pullup_en(BLACK_BUTTON_GPIO);
        ESP_LOGI(LOG_TAG, "Press the black button again");
        while (gpio_get_level(BLACK_BUTTON_GPIO) == 1) // wait for button press
        { vTaskDelay(10); 
        }
        gpio_set_level(GREEN_LED_GPIO, 1); // active low
        vTaskDelay(2);  // debounce
        while (gpio_get_level(BLACK_BUTTON_GPIO) == 0) // wait for button release
        {   vTaskDelay(2); 
        }
        lineCount++;
        // stepper_motor_step(lineCount * (lineCount % 2 ? 1 : -1));
    }
    i2c_lcd_deinit();
}
