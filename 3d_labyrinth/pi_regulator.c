#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <audio_processing.h>
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"

//simple PI regulator implementation
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

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;

    static bool already_played_fin = 0;

    while(1){
        time = chVTGetSystemTime();

        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = pi_regulator(get_phase_dif());

        //controle_front_pi = get_controle_front();

       if (get_detection_fin())
            {
            	left_motor_set_speed(0);
            	right_motor_set_speed(0);

            	if (!already_played_fin)
            	{
            		playMelody(7, ML_FORCE_CHANGE, NULL);
            		already_played_fin = 1;
            	}
            }


       else if(!get_detection_fin())
       {
			if (get_controle_front())
			{
				right_motor_set_speed(speed);
				left_motor_set_speed(-speed);
			}


			else
			{
				//applies the speed from the PI regulator and the correction for the rotation
				right_motor_set_speed(MOTOR_SPEED_LIMIT/4 + speed);
				left_motor_set_speed(MOTOR_SPEED_LIMIT/4 -speed);
			}
       }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
