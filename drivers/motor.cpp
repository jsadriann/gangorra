#include "motor.hpp"

#include <cstdio>
#include <pico/stdio.h>
#include <pico/time.h>

#include "hardware/pwm.h"
#include "hardware/gpio.h"


Motor::Motor(const uint32_t frequency, const uint gpio) {
	this->frequency = frequency;
	this->gpio = gpio;
	this->slice_num = pwm_gpio_to_slice_num(gpio);
	this->pwm_channel = pwm_gpio_to_channel(gpio);
}

Motor::~Motor() = default;

void Motor::pwm_set_freq() {
	//Value that allows to minimize the frequency of pwm bellow 1.9khz (minimal default value)
	uint32_t divider16 = RASP_CLOCK / this->frequency / 4096 +
											 (RASP_CLOCK % (this->frequency * 4096) != 0);

	if (divider16 / 16 == 0)
		divider16 = 16;

	//This is the highest value the counter will count up to before either rolling over to zero
	//or starting to count down to zero
	this->wrap = RASP_CLOCK * 16 / divider16 / frequency - 1;

	printf("Iniciando motor");

	pwm_set_clkdiv_int_frac(this->slice_num, divider16 / 16,
													divider16 & 0xF);
	pwm_set_wrap(this->slice_num, this->wrap);

	printf("Motor iniciado com wrap: %d e divider16: %d\n", this->wrap, divider16);
}

void Motor::vInitMotors() {
	stdio_init_all();
	gpio_set_function(this->gpio, GPIO_FUNC_PWM);

	this->pwm_set_freq();

	this->setSpeed(0);

	pwm_set_enabled(this->slice_num, true);

	printf("Inincializando, colocando 30%% para começar\n");
	this->setSpeed(30);

	sleep_ms(3000);
	printf("Começou, colocando 40%% como vel padrão\n");
	this->setSpeed(40);
	sleep_ms(1000);
	printf("acabei a função de init \n");
}

void Motor::setSpeed(const uint speed) const {
	pwm_set_chan_level(this->slice_num, this->pwm_channel, this->wrap * speed / 100);
}

uint Motor::get_slice_num() const {
	return  this->slice_num;
}
