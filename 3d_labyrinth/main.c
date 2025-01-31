#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

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
#include <controle.h>


static bool mode_selector = 0;


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



bool get_mode_selector (void)
{
	return mode_selector;
}



int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);


    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
	motors_init();
	proximity_start();
	dac_start();
	spi_comm_start();
	playMelodyStart();

	// Init the proximity sensor analyze
	prox_analyse_start();

	// Read the state of control mode chosen by selector
	mode_selector = get_selector()%2;


	// Init for IMU control
	if(mode_selector == MODE_IMU)
	{
		imu_start();
		controle_imu_start();
	}

	// Init for sound control
	if (mode_selector == MODE_SON)
	{
		mic_start(&processAudioData);
		controle_son_start();
	}

	static bool stop = FALSE;


    /* Infinite loop. */
    while (1)
    {
    	if (get_detection_fin() && !stop)
    	{
    		if (mode_selector == MODE_IMU)
    		{
    			prox_analyse_stop();
    			controle_imu_stop();
    			imu_stop();
    		}

    		if (mode_selector == MODE_SON)
			{
    			prox_analyse_stop();
    			controle_son_stop();
    		}

    		stop = TRUE;
    	}

        chThdSleepMilliseconds(1000);
    }
}



#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
