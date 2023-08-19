// Drive 28BYJ-48 stepper motor with ULN2003 driver board

/* Wires of 28BYJ-48 stepper motor from a diagram
   Red: +5v
   Orange: Coil 1
   Yellow: Coil 3
   Pink: Coil 2
   Blue: Coil 4
   The wires do not come out of the motor in the same order as various pictures, but the colours match.

  Maybe the wiring through the ULN2003 board might put the coils in order IN1, IN2, IN3, IN4.
  The lihts on the ULN2003 board are lit when low voltage, and one light doesn't work. the motor gets
  5v supply (no 0v) and the 4 coil lines, active low.


*/

#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static gpio_num_t motor_coil_io[4] = { GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26 };

static const char *TAG = "stepper_motor";

static int steps_to_go = 0;
/* 60 x all-coil steps per second, 4 coils. It does not turn reliably at 120 all-coil steps.
   Data sheet doesn't give a maximum speed, but pwoer consumption was measured at 120Hz */
static uint64_t step_delay_us = ( 1000000 / 60 / 4 ); 


void stepper_motor_init(uint64_t step_delay_us_param)
{
    if (step_delay_us_param != 0) step_delay_us = step_delay_us_param;
    ESP_LOGI(TAG, "stepper_motor_init, delay = %llu", (uint64_t) step_delay_us);
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(motor_coil_io[i]);
        gpio_set_direction(motor_coil_io[i], GPIO_MODE_OUTPUT);
        gpio_set_level(motor_coil_io[i], 0); /* 1 = not energised, cf 5v supply */
    }
    esp_timer_init();
}

uint8_t waitUs(uint64_t us)
{
    uint64_t start = esp_timer_get_time();
    while (esp_timer_get_time() >= start && (esp_timer_get_time() - start) < us) {
        ;
    }
    return 0;
}

void stepper_motor_step(int steps)
{
   ESP_LOGI(TAG, "stepper_motor_step %d, delay %llu", steps, step_delay_us);
   steps_to_go += steps;
   /* turn the motor in this function, blocking, simple logic counts whole steps and reverts
       all IOs to high afterwards,  */
   while (steps_to_go != 0) {
      if (steps_to_go > 0) {
            /* turn clockwise */
            for (int i = 0; i < 4; i++) {
               gpio_set_level(motor_coil_io[i], 1);
               waitUs(step_delay_us);
               gpio_set_level(motor_coil_io[i], 0);
            }
            steps_to_go--;
      } else {
            /* turn anticlockwise */
            for (int i = 3; i >= 0; i--) {
               gpio_set_level(motor_coil_io[i], 1);
               waitUs(step_delay_us);
               gpio_set_level(motor_coil_io[i], 0);
            }
            steps_to_go++;
      }
   }
   for (int i = 0; i < 4; i++) {
        gpio_set_level(motor_coil_io[i], 0); /* 1 = not energised, cf 5v supply */
   }
}
