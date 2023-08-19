#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <stdint.h>

/**
 * Initialize the stepper motor with the given step delay in microseconds.
 */
void stepper_motor_init(uint64_t step_delay_us);

/**
 * Turn the stepper motor by the given number of whole steps (each coil energised once per step).
 */
void stepper_motor_step(int steps);

#endif /* STEPPER_MOTOR_H */
