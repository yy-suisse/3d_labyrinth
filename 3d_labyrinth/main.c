#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/po8030.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include "ir_remote.h"
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

#define MODE_IMU 0 //// enum
#define MODE_SON 1
#define VITESSE_BASE 				150


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root, aseba_ns;

static THD_WORKING_AREA(controle_thd_wa, 2048);
static THD_WORKING_AREA(prox_analyse_thd_wa, 2048);

static bool load_config(void)
{
    extern uint32_t _config_start;

    return config_load(&parameter_root, &_config_start);
}

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

bool get_controle_front(void)
{
	return controle_front;
}


void show_gravity(imu_msg_t *imu_values)
{

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;

    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 0.4;

    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;

    // Filtre par moyenne pour éviter les accoups lors des changements de vitesse
    static float tab_average[8] = {0};			// 8 en DEFINE
    static uint8_t boucle = 0;
    float acceleration_average = 0;			//////////////////////////// FILTRE MOYENNE PEUT ETRE ESSAYER AVEC DSP

    static bool already_played = 0;
    static bool already_played_fin = 0;



    tab_average[boucle] = fabs(accel[Y_AXIS]);
    boucle++;

    if (boucle == 8)
    {
    	boucle = 0;
    }

    for(int i = 0; i < 8 ; i++)
    {
    	acceleration_average += tab_average[i];
    }
    acceleration_average = acceleration_average / 8 ; //// SHIFT PLUTOT ///////////////////////////////

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

		if(fabs(accel[X_AXIS]) > threshold || fabs(accel[Y_AXIS]) > threshold)
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
						already_played = 0;
					}

					else if (controle_back)
					{
						left_motor_set_speed(0);
						right_motor_set_speed(0);

						if (!already_played)
						{
							playMelody(6, ML_FORCE_CHANGE, NULL);
							already_played = 1;
						}
					}

				}

				else if(angle >= M_PI/2 && angle < M_PI)
				{
					led7 = 1;
					left_motor_set_speed(-VITESSE_BASE);
					right_motor_set_speed(VITESSE_BASE);
					already_played = 0;
				}

				else if(angle >= -M_PI && angle < -M_PI/2)
				{
					led1 = 1;

					if(!controle_front)
					{
						left_motor_set_speed(VITESSE_BASE*acceleration_average);
						right_motor_set_speed(VITESSE_BASE*acceleration_average);
						already_played = 0;
					}

					else if (controle_front)
					{
						left_motor_set_speed(0);
						right_motor_set_speed(0);

						if (!already_played)
						{
							playMelody(6, ML_FORCE_CHANGE, NULL);
							already_played = 1;
						}
					}

				}

				else if(angle >= -M_PI/2 && angle < 0)
				{
					led3 = 1;
					left_motor_set_speed(VITESSE_BASE);
					right_motor_set_speed(-VITESSE_BASE);
					already_played = 0;
				}
			}

			// cas ou on est en dessous du threshold -> pas de mouvements
			else
			{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
			}




			//we invert the values because a led is turned on if the signal is low
			palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
			palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
			palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
			palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);


    }

    else if (detection_fin)
    {
    	left_motor_set_speed(0);
    	right_motor_set_speed(0);

    	if (!already_played_fin)
    	{
    		playMelody(7, ML_FORCE_CHANGE, NULL);
    		already_played_fin = 1;
    	}
    }

}

static THD_FUNCTION(prox_analyse_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;
    static int init_ambient[8];
    static bool init_in_process = 1;

    static bool envoie = 1;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

    calibrate_ir();


    while(1) {
    	time = chVTGetSystemTime();

    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    	if(init_in_process)
    	{
    			for (int j =0; j<8;j++)
    			{
    				init_ambient[j]=prox_values.ambient[j];
    			}

    			init_in_process=0;
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

    			set_rgb_led(0, 0, 0, 0); ////////////////////////// faire un for ////////////////////////////////////
    			set_rgb_led(1, 0, 0, 0);
    			set_rgb_led(2, 0, 0, 0);
    			set_rgb_led(3, 0, 0, 0);

/*
    			if (envoie)
    			{
					for(int i=0;i<8;i++)
					{
						chprintf((BaseSequentialStream *)&SDU1, "LED %d ambient actuel value is : %4d,\r\n"  ,i,prox_values.ambient[i]);
						chprintf((BaseSequentialStream *)&SDU1, "LED %d ambient initial value is : %4d,\r\n"  ,i,init_ambient[i]);
						chprintf((BaseSequentialStream *)&SDU1, "LED %d variation lumiere ambient value is : %4d,\r\n"  ,i,init_ambient[i]-prox_values.ambient[i]);

						//chprintf((BaseSequentialStream *)&SDU1, "LED %d reflected value is : %4d,\r\n"  ,i,prox_values.reflected[i]);
						//chprintf((BaseSequentialStream *)&SDU1, "LED %d distance initial value is : %4d,\r\n"  ,i,prox_values.delta[i]);
					}


    			}

    			envoie = !envoie;
*/





    			uint8_t sum =0;

    			for(uint8_t k = 0 ; k < 8; k++)
    			{
    				sum =+	init_ambient[k]-prox_values.ambient[k];
    			}

    			if (sum >=250)
    			{

    				set_rgb_led(0, 10,5 , 2);
    				set_rgb_led(1, 10,5 , 2);
    				set_rgb_led(2, 10,5 , 2);
    				set_rgb_led(3, 10,5 , 2);
    				detection_fin=1;
    			}


				if (abs(prox_values.delta[0])>200) /// MAGIC NUMBER
				{
					set_rgb_led(0, 10,10 , 0);
					set_rgb_led(3, 10,10 , 0);
					 controle_front = 1;
				}

				else if (abs(prox_values.delta[0])<200)
				{
					controle_front = 0;
				}


				if (abs(prox_values.delta[2])>200)
				{
					set_rgb_led(0, 10,10 , 0);
					set_rgb_led(1, 10,10 , 0);
				}




				if (abs(prox_values.delta[3])>200)
				{
					set_rgb_led(1, 10,10 , 0);
					set_rgb_led(2, 10,10 , 0);
					controle_back = 1 ;
				}


				else if (abs(prox_values.delta[3])<200)
				{
					controle_back = 0;
				}


				if (abs(prox_values.delta[5])>200)
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
    calibrate_gyro();  	// VOIR SI NECESSAIRE //////////////////////////////////////////

    while(1)
    {
    	//wait for new measures to be published
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    	if(!detection_fin)
    	{
    		playMelody(0, ML_SIMPLE_PLAY, NULL); /// MAGIC NUMBER
    	}

		// Reflect the orientation on the LEDs around the robot.
		//e_display_angle();

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
	dcmi_start();
	po8030_start();
	motors_init();
	proximity_start();
	battery_level_start();
	dac_start();
	exti_start();
	ir_remote_start();
	spi_comm_start();
	VL53L0X_start();
	serial_start();
	sdio_start();
	playMelodyStart();
	playSoundFileStart();

	if(get_selector() == MODE_IMU)
	{
		imu_start();
		chThdCreateStatic(controle_thd_wa, sizeof(controle_thd_wa), NORMALPRIO, controle_thd, NULL);
	}

	if (get_selector() == MODE_SON)
	{
		mic_start(&processAudioData);
		pi_regulator_start();
	}

	// Initialise Aseba system, declaring parameters
    parameter_namespace_declare(&aseba_ns, &parameter_root, "aseba");
    aseba_declare_parameters(&aseba_ns);

    /* Load parameter tree from flash. */
    load_config();

    /* Start AsebaCAN. Must be after config was loaded because the CAN id
     * cannot be changed at runtime. */
    aseba_vm_init();
    aseba_can_start(&vmState);
    calibrate_ir();

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
