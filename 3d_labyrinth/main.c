#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
//#include "chprintf.h"

#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/microphone.h"

#include "sensors/imu.h"
#include "sensors/proximity.h"

#include "leds.h"
#include "motors.h"
#include "selector.h"
#include "spi_comm.h"

#include "i2c_bus.h"
#include "usbcfg.h"

#include <main.h>
#include <audio_processing.h>
#include <pi_regulator.h>
#include <fft.h>
#include <arm_math.h>


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)///////////////////////////////////

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root;


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    parameter_namespace_declare(&parameter_root, NULL, NULL);

    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();// led orange que pour imu -> voir assistant
	motors_init();
	proximity_start();
	dac_start(); // kernel panic si on enleve
	spi_comm_start();
	playMelodyStart();

	prox_analyse_start();

	if(get_selector()%2 == MODE_IMU)
	{
		imu_start();
		controle_imu_start();
	}

	if (get_selector()%2 == MODE_SON)
	{
		mic_start(&processAudioData);
		pi_regulator_start();
	}


    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
