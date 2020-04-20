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
#include <controle.h>
#include <audio_processing.h>


// 3 control flags which help to decide movement or event
static bool controle_front = 0;
static bool controle_back = 0;
static bool detection_fin = 0;


/***************************INTERNAL FUNCTIONS************************************/

/**
* @brief   Thread which analyzes the distances of 8 proximity sensors to decide the movement
* 		   analyze the ambient light for the detection of End
*/

static THD_WORKING_AREA(prox_analyse_thd_wa, 2048);

static THD_FUNCTION(prox_analyse_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;
    static int init_ambient[PROXIMITY_NB_CHANNELS];
    static bool init_in_process = TRUE;

    static int16_t sum = 0;

    static bool mode = 0;

    mode = get_mode_selector();


    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

    calibrate_ir();


    while(1) {
    	time = chVTGetSystemTime();

    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));


    	// initialize the ambient light once at the beginning, it helps to determine the end of the game
    	if(init_in_process)
    	{
    			for (uint8_t j = 0; j < PROXIMITY_NB_CHANNELS; j++)
    			{
    				init_ambient[j]=prox_values.ambient[j];
    			}

    			init_in_process = FALSE;
    	}



    	// clear all the  RGB LED
    	for(uint8_t i = 0 ; i < NOMBRE_LED_RGB ; i++ )
    	{
    		set_rgb_led(i, 0, 0, 0);
    	}



    	// calculate the total changement of ambient light
    	for(uint8_t k = 0 ; k < PROXIMITY_NB_CHANNELS; k++)
    	{
    		sum =+	init_ambient[k]-prox_values.ambient[k];
    	}


    	//if actual ambient light is intensified, then the robot arrives at the end of mission
    	if (sum >=SEUIL_DETECTION_FIN)
    	{
    		for(uint8_t i = 0 ; i < NOMBRE_LED_RGB ; i++ )
    		{
    			set_rgb_led(i, 10,5 , 2);
    		}

    		detection_fin = TRUE;
    	}

    	sum = 0;

    	// blocage if sensors in the front detect an obstacle but we want to move forward (mode imu)
    	if (mode == MODE_IMU)
    	{
			if (abs(prox_values.delta[0])>SEUIL_PROXI_FB || abs(prox_values.delta[7])>SEUIL_PROXI_FB)
			{
				set_rgb_led(0, 10,10 , 0);
				set_rgb_led(3, 10,10 , 0);
				controle_front = TRUE ;
			}

			else
			{
				controle_front = FALSE;
			}
    	}

		// blocage if sensors in the front 45° detect an obstacle but we want to move forward (mode sound)
    	if (mode == MODE_SON)
    	{
			if (abs(prox_values.delta[0])>SEUIL_PROXI_FB || abs(prox_values.delta[7])>SEUIL_PROXI_FB || abs(prox_values.delta[6])>SEUIL_PROXI_LATERAL || abs(prox_values.delta[1])>SEUIL_PROXI_LATERAL)
			{
				set_rgb_led(0, 10,10 , 0);
				set_rgb_led(3, 10,10 , 0);
				controle_front = TRUE ;
			}

			else
			{
				controle_front = FALSE;
			}
    	}

		// if right sensor detects an obstacle, right LEDs on
		if (abs(prox_values.delta[2])>SEUIL_PROXI_LATERAL)
		{
			set_rgb_led(0, 10,10 , 0);
			set_rgb_led(1, 10,10 , 0);
		}



		// blocage if sensors at the back detect an obstacle but we want to step back ( mode imu)
		if (mode == MODE_IMU)
		{
			if (abs(prox_values.delta[3])>SEUIL_PROXI_FB  || abs(prox_values.delta[4])>SEUIL_PROXI_FB)
			{
				set_rgb_led(1, 10,10 , 0);
				set_rgb_led(2, 10,10 , 0);
				controle_back = TRUE ;
			}


			else
			{
				controle_back = FALSE;
			}
		}



		// if left sensor detects an obstacle, left LEDs on
		if (abs(prox_values.delta[5])>SEUIL_PROXI_LATERAL)
		{
			set_rgb_led(2, 10,10 , 0);
			set_rgb_led(3, 10,10 , 0);
		}

		chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.

    }
}

/**
 * @brief   fonction reads IMU datas then analyzes which direction it should go. In case that no obstable is present,
 * 			robot goes in the direction given by accelerometer.
 * 			In case that obstable is present, robots stop moving and plays melody until player choose another move direction
 * 			where is no obstacle.
 * 			In case of end of mission, robot doesn't move anymore and play melody for once.
 *
 * @param   data from accelerometer
 */


void show_gravity(imu_msg_t *imu_values)
{

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;

    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;

    // Filtre par moyenne pour éviter les accoups lors des changements de vitesse
    static float tab_average[NB_VALEUR_FILTRE] = {0};
    static uint8_t boucle = 0;
    float acceleration_average = 0;

    static bool already_played = FALSE;
    static bool already_played_fin = FALSE;


    //fill the array of aceeleration datas
    tab_average[boucle] = fabs(accel[Y_AXIS]);
    boucle++;

    if (boucle == NB_VALEUR_FILTRE)
    {
    	boucle = 0;
    }


    // caculate average of acceleration
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

    if (!detection_fin) // if mission is not finished yet, calculate movement orientation
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


				if(angle >= 0 && angle < M_PI/2) // move back
				{
					led5 = 1;

					if (!controle_back)
					{
						left_motor_set_speed(-VITESSE_BASE*acceleration_average);
						right_motor_set_speed(-VITESSE_BASE*acceleration_average);
						already_played = FALSE; //stop music is allowed
					}

					else if (controle_back) // if move back is impossible , stop movement
					{
						left_motor_set_speed(NO_SPEED);
						right_motor_set_speed(NO_SPEED);

						if (!already_played) // stop misic can only played once
						{
							playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
							already_played = TRUE;
						}
					}

				}

				else if(angle >= M_PI/2 && angle < M_PI) //rotation
				{
					led7 = 1;
					left_motor_set_speed(-VITESSE_BASE);
					right_motor_set_speed(VITESSE_BASE);
					already_played = FALSE;
				}

				else if(angle >= -M_PI && angle < -M_PI/2) // move forward
				{
					led1 = 1;

					if(!controle_front) //if possible to move forward
					{
						left_motor_set_speed(VITESSE_BASE*acceleration_average);
						right_motor_set_speed(VITESSE_BASE*acceleration_average);
						already_played = FALSE;
					}

					else if (controle_front) //impossible to move forward, stop
					{
						left_motor_set_speed(NO_SPEED);
						right_motor_set_speed(NO_SPEED);

						if (!already_played) // stop music can pnly be played once
						{
							playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
							already_played = TRUE;
						}
					}

				}

				else if(angle >= -M_PI/2 && angle < 0) //rotation
				{
					led3 = 1;
					left_motor_set_speed(VITESSE_BASE);
					right_motor_set_speed(-VITESSE_BASE);
					already_played = FALSE;
				}
			}

			// in case that acceleromater vaues are smaller than noise signal
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

    else if (detection_fin) // if mission is accomplished
    {
    	left_motor_set_speed(NO_SPEED);
    	right_motor_set_speed(NO_SPEED);

    	if (!already_played_fin) // endinf music can only be played once
    	{
    		playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
    		already_played_fin = TRUE;
    	}
    }

}




/**
 * @brief	Thread for getting data from accelerometer, in case of end of mission, robot doesn't move
 * 			anymore and play endinf melody.
 */


static THD_WORKING_AREA(controle_imu_thd_wa, 2048);

static THD_FUNCTION(controle_imu_thd, arg)
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

    	if(!detection_fin) //if mission is not finished yet, play principal melody
    	{
    		playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);
    	}

    	show_gravity(&imu_values);
		chThdSleepMilliseconds(100);
    }
}



/**
 * @brief   simple PI regulator implementation which calculate the speeds of each motor
 * 			depending on the actual angle and final angle
 *
 * @param   error between the actual angle and final angle
 *
 * @return speed calculated
 */


int16_t pi_regulator(float error){

	float speed = 0;

	static float sum_error = 0;


	//disables the PI regulator if the error is to small
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}


/**
 * @brief   Thread which allows robot to follow the sound
 *
 */
static THD_WORKING_AREA(controle_son_thd_wa, 256);

static THD_FUNCTION(controle_son_thd, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;

    static bool already_played_fin = FALSE;

    while(1){
        time = chVTGetSystemTime();

        //computes the speed to give to the motors
        speed = pi_regulator(get_phase_dif());


       if (detection_fin) // if mission is finished, motors stops
            {
            	left_motor_set_speed(NO_SPEED);
            	right_motor_set_speed(NO_SPEED);

            	if (!already_played_fin) // ending music can be only played once
            	{
            		playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
            		already_played_fin = TRUE;
            	}
            }


       else if(!detection_fin) //if mission is in process
       {
			if (controle_front) //if obstable is present in front of robot, no movement
			{
				right_motor_set_speed(speed);
				left_motor_set_speed(-speed);
			}


			else   // if no obstacle is present  i  front of robot, robot can move forward in rotating forward to the sound source
			{
				right_motor_set_speed(MOTOR_SPEED_LIMIT/4 + speed);
				left_motor_set_speed(MOTOR_SPEED_LIMIT/4 -speed);
			}
       }


        chThdSleepUntilWindowed(time, time + MS2ST(10));  //100Hz
    }
}
/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/


void prox_analyse_start(void)
{
	chThdCreateStatic(prox_analyse_thd_wa, sizeof(prox_analyse_thd_wa), NORMALPRIO, prox_analyse_thd, NULL);
}

void controle_imu_start(void)
{
	chThdCreateStatic(controle_imu_thd_wa, sizeof(controle_imu_thd_wa), NORMALPRIO, controle_imu_thd, NULL);
}

void controle_son_start(void)
{
	chThdCreateStatic(controle_son_thd_wa, sizeof(controle_son_thd_wa), NORMALPRIO, controle_son_thd, NULL);
}

/**************************END PUBLIC FUNCTIONS***********************************/
