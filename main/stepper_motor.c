// Drive 28BYJ-48 stepper motor with ULN2003 driver board

/* Wires of 28BYJ-48 stepper motor from a diagram
   Red: +5v
   Orange: Coil 1
   Yellow: Coil 3
   Pink: Coil 2
   Blue: Coil 4
   The wires do not come out of the motor in the same order as various pictures, but the colours match.

   Wires from my ULN2003 driver board
   Yellow: IN1 -> GPIO 34 -> Blue -> coil 4
   White: IN2 -> GPIO 35 -> Pink -> coil 2
   Red: IN3 -> GPIO 32 -> Yellow -> coil 3
   Black: IN4 -> GPIO 33 -> Orange -> coil 1
*/

#include <stdio.h>
#include "esp_log.h"

static uint8_t motor_coil_io[4] = { 33, 35, 34, 32 };

static const char *TAG = "stepper_motor";

static int steps_to_go = 0;
static uint32_t step_delay_us = 1000000 / 120 / 4; /* 120 steps per second, 4 coils */

void stepper_motor_init(uint32_t step_delay_us_param)
{
    ESP_LOGI(TAG, "stepper_motor_init");
    for (int i = 0; i < 4; i++) {
        gpio_pad_select_gpio(motor_coil_io[i]);
        gpio_set_direction(motor_coil_io[i], GPIO_MODE_OUTPUT);
        gpio_set_level(motor_coil_io[i], 1); /* 1 = not energised, cf 5v supply */
    }
    esp_timer_init();
    step_delay_us = step_delay_us_param;
    // esp_timer_create(
}

void stepper_motor_step(int steps)
{
    ESP_LOGI(TAG, "stepper_motor_step %d", steps);
    steps_to_go += steps;
    /* turn the motor in this function, blocking */
      while (steps_to_go != 0) {
         if (steps_to_go > 0) {
               /* turn clockwise */
               for (int i = 0; i < 4; i++) {
                  gpio_set_level(motor_coil_io[i], 0);
                  // esp_timer delay step_delay_us
                  vTaskDelay(1 / portTICK_PERIOD_MS);
                  gpio_set_level(motor_coil_io[i], 1);
               }
               steps_to_go--;
         } else {
               /* turn anticlockwise */
               for (int i = 3; i >= 0; i--) {
                  gpio_set_level(motor_coil_io[i], 0);
                  vTaskDelay(1 / portTICK_PERIOD_MS);
                  gpio_set_level(motor_coil_io[i], 1);
               }
               steps_to_go++;
         }
      }
}