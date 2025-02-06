//
// Created by guilherme on 03/12/24.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <cstdio>
#include <pico/stdio.h>


#define RASP_CLOCK 125000000

/**
 * @brief Class for controlling a DC motor with an ESC using PWM on the Raspberry Pi Pico.
 *
 * This class abstracts the configuration of PWM settings and provides methods
 * to initialize the motor, set its speed, and manage its behavior.
 */
class Motor {
	/**
		 * @brief Desired PWM frequency in Hertz.
		 *
		 * This parameter determines the speed of the PWM signal, which influences
		 * the behavior of the motor. A typical default value is 300 Hz.
		 */
	uint32_t frequency = 300;

	/**
		 * @brief PWM wrap value (period in clock cycles).
		 *
		 * The `wrap` value defines the total number of clock cycles in one PWM period.
		 * It is calculated based on the desired frequency and the Raspberry Pi Pico clock settings.
		 */

	uint32_t wrap{};

	/**
		 * @brief PWM slice number corresponding to the selected GPIO pin.
		 *
		 * The Raspberry Pi Pico's PWM module is divided into slices, each capable of controlling two channels.
		 * The `slice_num` specifies which slice is associated with the configured GPIO pin.
		 */
	uint slice_num;

	/**
		 * @brief GPIO pin used for motor control.
		 *
		 * The GPIO pin is configured in PWM mode to generate the control signal for the motor.
		 * Default value: 15 (can be changed in the constructor).
		 */
	uint gpio = 15;

	/**
		 * @brief PWM channel number within the associated slice.
		 *
		 * Each PWM slice has two channels (A and B). The `pwm_channel` specifies which channel
		 * (A or B) is used for the motor control signal.
		 */
	uint pwm_channel;
public:
	uint gpio1() const {
		return gpio;
	}

	uint get_slice_num() const;
	/**
	 * @brief Motor constructor that initializes the GPIO and PWM settings.
	 *
	 * @param frequency The desired PWM frequency for the motor control.
	 * @param gpio The GPIO pin number used for controlling the motor.
	 */
	Motor(uint32_t frequency, uint gpio);

	/**
 * @brief Motor destructor (default implementation).
 */
	~Motor();

	/**
 * @brief Configures the PWM frequency and sets up the internal timer wrap value.
 *
 * Calculates the clock divider and wrap value based on the desired frequency
 * and the Raspberry Pi Pico's clock settings.
 */
	void pwm_set_freq();

	/**
 * @brief Initializes the motor GPIO, sets up PWM, and tests speed adjustments.
 *
 * This function configures the GPIO pin for PWM, sets the PWM frequency,
 * and performs an initial speed ramp-up test.
 */
	void vInitMotors();

	/**
 * @brief Sets the motor speed as a percentage of the maximum PWM duty cycle.
 *
 * @param speed The desired speed as a percentage (0-100).
 */
	void setSpeed(uint speed) const;
};



#endif //MOTOR_H
