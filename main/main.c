
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
       lcd_write("Error Kbd init",0);

      vTaskDelay(1000);
    }
    while (true)
    {
        char line1[40];
        sprintf(line1, "%d steps", lineCount);
        err = lcd_write(line1,0);
        keyboard_scan();

        if (err != ESP_OK)
        { ESP_LOGE(LOG_TAG, "Error x%x writing to display", err);
        }
        else {
           ESP_LOGI(LOG_TAG, "Wrote to display");
        }
        keyboard_pressed_chars_str(line1, 40);
        strcat(line1,"  ");
        lcd_write(line1,1);
        ESP_LOGI(LOG_TAG, "ch %s", line1);
        vTaskDelay(10); /* 0.1s */
        /*
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
        */
        lineCount++;
        // stepper_motor_step(lineCount * (lineCount % 2 ? 1 : -1));
    }
    i2c_lcd_deinit();
}
