#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/microphone.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
//#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include "leds.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"

#include <audio_processing.h>
#include <pi_regulator.h>
#include <fft.h>
#include <arm_math.h>

static bool controle_front = 0;
static bool controle_back = 0;
static bool detection_fin = 0;


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root;

static THD_WORKING_AREA(controle_thd_wa, 2048);
static THD_WORKING_AREA(prox_analyse_thd_wa, 2048);


bool get_controle_front(void)
{
	return controle_front;
}


bool get_detection_fin(void)
{
	return detection_fin;
}


void show_gravity(imu_msg_t *imu_values)
{

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;

    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;

    // Filtre par moyenne pour �viter les accoups lors des changements de vitesse
    static float tab_average[NB_VALEUR_FILTRE] = {0};
    static uint8_t boucle = 0;
    float acceleration_average = 0;

    static bool already_played = FALSE;
    static bool already_played_fin = FALSE;



    tab_average[boucle] = fabs(accel[Y_AXIS]);
    boucle++;

    if (boucle == NB_VALEUR_FILTRE)
    {
    	boucle = 0;
    }

    for(uint8_t i = 0; i < NB_VALEUR_FILTRE ; i++)
    {
    	acceleration_average += tab_average[i];
    }
    acceleration_average = acceleration_average / NB_VALEUR_FILTRE ; //// SHIFT PLUTOT ///////////////////////////////

    /*
    *   example 1 with trigonometry.
    */

    /*
    * Quadrant:
    *
    *       BACK
    *       ####
    *    #    0   #
    *  #            #
    * #-PI/2 TOP PI/2#
    * #      VIEW    #
    *  #            #
    *    # -PI|PI #
    *       ####
    *       FRONT
    */

    if (!detection_fin)
    {

		if(fabs(accel[X_AXIS]) > TRESHOLD_IMU || fabs(accel[Y_AXIS]) > TRESHOLD_IMU)
		{

			chSysLock();

			//clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
			float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);

			chSysUnlock();

			//rotates the angle by 45 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
			angle += M_PI/4;

			//if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
			//so we correct it
			if(angle > M_PI){
				angle = -2 * M_PI + angle;
			}


				if(angle >= 0 && angle < M_PI/2)
				{
					led5 = 1;

					if (!controle_back)
					{
						left_motor_set_speed(-VITESSE_BASE*acceleration_average);
						right_motor_set_speed(-VITESSE_BASE*acceleration_average);
						already_played = FALSE;
					}

					else if (controle_back)
					{
						left_motor_set_speed(NO_SPEED);
						right_motor_set_speed(NO_SPEED);

						if (!already_played)
						{
							playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
							already_played = TRUE;
						}
					}

				}

				else if(angle >= M_PI/2 && angle < M_PI)
				{
					led7 = 1;
					left_motor_set_speed(-VITESSE_BASE);
					right_motor_set_speed(VITESSE_BASE);
					already_played = FALSE;
				}

				else if(angle >= -M_PI && angle < -M_PI/2)
				{
					led1 = 1;

					if(!controle_front)
					{
						left_motor_set_speed(VITESSE_BASE*acceleration_average);
						right_motor_set_speed(VITESSE_BASE*acceleration_average);
						already_played = FALSE;
					}

					else if (controle_front)
					{
						left_motor_set_speed(NO_SPEED);
						right_motor_set_speed(NO_SPEED);

						if (!already_played)
						{
							playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
							already_played = TRUE;
						}
					}

				}

				else if(angle >= -M_PI/2 && angle < 0)
				{
					led3 = 1;
					left_motor_set_speed(VITESSE_BASE);
					right_motor_set_speed(-VITESSE_BASE);
					already_played = FALSE;
				}
			}

			// cas ou on est en dessous du threshold -> pas de mouvements
			else
			{
				left_motor_set_speed(NO_SPEED);
				right_motor_set_speed(NO_SPEED);
			}




			//we invert the values because a led is turned on if the signal is low
			palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
			palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
			palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
			palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);


    }

    else if (detection_fin)
    {
    	left_motor_set_speed(NO_SPEED);
    	right_motor_set_speed(NO_SPEED);

    	if (!already_played_fin)
    	{
    		playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
    		already_played_fin = TRUE;
    	}
    }

}

static THD_FUNCTION(prox_analyse_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;
    static int init_ambient[PROXIMITY_NB_CHANNELS];
    static bool init_in_process = TRUE;

    static int16_t sum = 0;


    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

    calibrate_ir();


    while(1) {
    	time = chVTGetSystemTime();

    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    	if(init_in_process)
    	{
    			for (uint8_t j = 0; j < PROXIMITY_NB_CHANNELS; j++)
    			{
    				init_ambient[j]=prox_values.ambient[j];
    			}

    			init_in_process = FALSE;
    	}


			/* // Read proximity sensors.

				chprintf((BaseSequentialStream *)&SDU1, "LED 1 value is : %4d,\n"  ,prox_values.delta[0]);
	        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
				chprintf((BaseSequentialStream *)&SDU1, "LED 2 value is : %4d,\n"  ,prox_values.delta[1]);
	        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
				chprintf((BaseSequentialStream *)&SDU1, "LED 3 value is : %4d,\n"  ,prox_values.delta[2]);
	        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
				chprintf((BaseSequentialStream *)&SDU1, "LED 4 value is : %4d,\n"  ,prox_values.delta[3]);
	        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
				chprintf((BaseSequentialStream *)&SDU1, "LED 5 value is : %4d,\n"  ,prox_values.delta[4]);
	        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
				chprintf((BaseSequentialStream *)&SDU1, "LED 6 value is : %4d,\n"  ,prox_values.delta[5]);
	        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
				chprintf((BaseSequentialStream *)&SDU1, "LED 7 value is : %4d,\n"  ,prox_values.delta[6]);
	        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
				chprintf((BaseSequentialStream *)&SDU1, "LED 8 value is : %4d,\n"  ,prox_values.delta[7]);
	        	chprintf((BaseSequentialStream *)&SDU1, "\r\n");

*/

    	for(uint8_t i = 0 ; i < NOMBRE_LED_RGB ; i++ )
    	{
    		set_rgb_led(i, 0, 0, 0);
    	}


    	for(uint8_t k = 0 ; k < PROXIMITY_NB_CHANNELS; k++)
    	{
    		sum =+	init_ambient[k]-prox_values.ambient[k];
    	}



    	if (sum >=SEUIL_DETECTION_FIN)
    	{
    		for(uint8_t i = 0 ; i < NOMBRE_LED_RGB ; i++ )
    		{
    			set_rgb_led(i, 10,5 , 2);
    		}

    		detection_fin = TRUE;
    	}

    	sum = 0;


		if (abs(prox_values.delta[0])>SEUIL_PROXI_FB || abs(prox_values.delta[7])>SEUIL_PROXI_FB || abs(prox_values.delta[6])>SEUIL_PROXI_LATERAL || abs(prox_values.delta[1])>SEUIL_PROXI_LATERAL) /// MAG C NUMBER
		{
			set_rgb_led(0, 10,10 , 0);
			set_rgb_led(3, 10,10 , 0);
			controle_front = TRUE ;
		}

		else if (abs(prox_values.delta[0])<SEUIL_PROXI_FB && abs(prox_values.delta[7])<SEUIL_PROXI_FB && abs(prox_values.delta[6])<SEUIL_PROXI_LATERAL && abs(prox_values.delta[1])<SEUIL_PROXI_LATERAL)
		{
			controle_front = FALSE;
		}


		if (abs(prox_values.delta[2])>SEUIL_PROXI_LATERAL)
		{
			set_rgb_led(0, 10,10 , 0);
			set_rgb_led(1, 10,10 , 0);
		}




		if (abs(prox_values.delta[3])>SEUIL_PROXI_FB  || abs(prox_values.delta[4])>SEUIL_PROXI_FB)
		{
			set_rgb_led(1, 10,10 , 0);
			set_rgb_led(2, 10,10 , 0);
			controle_back = TRUE ;
		}


		else if (abs(prox_values.delta[3])<SEUIL_PROXI_FB && abs(prox_values.delta[4])<SEUIL_PROXI_FB)
		{
			controle_back = FALSE;
		}


		if (abs(prox_values.delta[5])>SEUIL_PROXI_LATERAL)
		{
			set_rgb_led(2, 10,10 , 0);
			set_rgb_led(3, 10,10 , 0);
		}

		chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.



    }
}

static THD_FUNCTION(controle_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);


    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;


    calibrate_acc();

    while(1)
    {
    	//wait for new measures to be published
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    	if(!detection_fin)
    	{
    		playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);
    	}

    	show_gravity(&imu_values);
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

    parameter_namespace_declare(&parameter_root, NULL, NULL);

    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
	motors_init();
	proximity_start();
	//battery_level_start();
	dac_start();
	exti_start();
	spi_comm_start();
	sdio_start();
	playMelodyStart();

	if(get_selector()%2 == MODE_IMU)
	{
		imu_start();
		chThdCreateStatic(controle_thd_wa, sizeof(controle_thd_wa), NORMALPRIO, controle_thd, NULL);
	}

	if (get_selector()%2 == MODE_SON)
	{
		mic_start(&processAudioData);
		pi_regulator_start();
	}


    chThdCreateStatic(prox_analyse_thd_wa, sizeof(prox_analyse_thd_wa), NORMALPRIO, prox_analyse_thd, NULL);

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
