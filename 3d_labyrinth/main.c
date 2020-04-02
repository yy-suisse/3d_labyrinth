#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "i2c_bus.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h" // fonction angle
#include "usbcfg.h"

#include "leds.h"
#include "motors.h"


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}



static THD_WORKING_AREA(imu_thd_wa, 2048);

static THD_FUNCTION(imu_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    //int16_t leftSpeed = 0, rightSpeed = 0;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    while(1)
    {
    	calibrate_acc();
		calibrate_gyro(); /////////////////// a voir si nécessaire

		//wait for new measures to be published
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		// Read IMU.
		chprintf((BaseSequentialStream *)&SDU1, "IMU\r\n");
		chprintf((BaseSequentialStream *)&SDU1, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n\n", imu_values.acc_raw[0], imu_values.acc_raw[1], imu_values.acc_raw[2], imu_values.gyro_raw[0], imu_values.gyro_raw[1], imu_values.gyro_raw[2]);

		e_display_angle();

		chThdSleepMilliseconds(100);
    }
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Init the peripherals.
    //motors_init();
    imu_start(); 			// lance la thread qui s'occupe de l'update des mesures et de leur publication
    serial_start();
    usb_start();


    chThdCreateStatic(imu_thd_wa, sizeof(imu_thd_wa), NORMALPRIO, imu_thd, NULL);


    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
