#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

///////////////////////////////////////////////// MESURE ANGLE ///////////////////////////////////////////////////////////////////
/*
//tableau contenant la correlation
static float correlation[2 * FFT_SIZE];
static float correlation_output[FFT_SIZE];

// maximum de la correaltion = dt
static double max_correlation = 0.0;
static float indice =0;

static float tab_max[FFT_SIZE];
static double c_theta_max = 0;

//Angle direction de la source
static float theta = 0.0;
static double c_theta = 0.0;

static int boucle = 0;*/

static float micLeft_real_input[FFT_SIZE];
static float micRight_real_input[FFT_SIZE];
static float correlation_result[2*FFT_SIZE-1];

static double max_correlation = 0;
static double indice = 0;
static double theta = 0;

#define VITESSE_SON		340.29f
#define DISTANCE_MICRO	0.061f

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)


/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//go forward
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
		left_motor_set_speed(600);
		right_motor_set_speed(600);
	}
	//turn left
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	//turn right
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}
	//go backward
	else if(max_norm_index >= FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(-600);
	}
	else{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	static uint16_t nb_samples_correlation = 0;
	max_correlation = 0;
	indice = 0;

	/*
	max_correlation = 0;
	indice = 0;
	c_theta = 0;
	theta = 0;*/

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];


		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}


	for (uint16_t j = 0 ; j < num_samples ; j+=4)
	{
		micRight_real_input[nb_samples_correlation] = (float)data[j + MIC_RIGHT];
		micLeft_real_input[nb_samples_correlation] = (float)data[j + MIC_LEFT];

		nb_samples_correlation++;

		//stop when buffer is full
		if(nb_samples_correlation >= FFT_SIZE)
		{
			break;
		}
	}

	//if(nb_samples_correlation >= FFT_SIZE)
	//{

	//}


	nb_samples_correlation = 0;


	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*
		// conjugué de la transformée du micro left
		for (int k=0; k< FFT_SIZE ; k++)
		{
			micLeft_cmplx_input[2*k+1] = -micLeft_cmplx_input[2*k+1];
		}

		//Calcul de la transformée de la correlation par multiplication du conjugué de la transformée du micro left avec la transformée du micro right
		//(a+bi)*(c+di) = a*c -b*d +i*(a*d + b*c)
		for (int l = 0; l< FFT_SIZE ; l++)
		{
			correlation[2*l] = micLeft_cmplx_input[2*l]*micRight_cmplx_input[2*l] - micLeft_cmplx_input[2*l+1]*micRight_cmplx_input[2*l+1];
			correlation[2*l+1] = micLeft_cmplx_input[2*l]*micRight_cmplx_input[2*l+1] + micLeft_cmplx_input[2*l+1]*micRight_cmplx_input[2*l];
		}

		//Calcul de la correlation par transformée inverse
		doIFFT_optimized(FFT_SIZE, correlation);


		//algorithme de tri pour trouver le maximum de la corrélation
		for (int m = 0; m< 2*FFT_SIZE ; m++)
		{
			if (correlation[m]>max_correlation)
			{
				max_correlation = correlation[m];
			}
		}

		//Calcul de l'angle en radian puis conversion en degrés
		theta = acos((max_correlation/DISTANCE_MICRO)*(VITESSE_SON));
		theta = (theta/M_PI)*(180);


		chprintf((BaseSequentialStream *) &SDU1, "max_corre = %f, angle= %f \r\n",max_correlation, theta);*/

		arm_correlate_f32(micLeft_real_input, FFT_SIZE, micRight_real_input, FFT_SIZE, correlation_result);

				//algorithme de tri pour trouver le maximum de la corrélation
				for (int m = 0; m < (2*FFT_SIZE-1) ; m++)
				{
					if (correlation_result[m] > max_correlation)
					{
						max_correlation = correlation_result[m];
						indice = m;
					}
				}

				//theta = (acos((max_correlation*VITESSE_SON)/(DISTANCE_MICRO)))*(180/3.14);

				chprintf((BaseSequentialStream *) &SDU1, "max corre = %f \r\n",max_correlation);


		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);


		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		sound_remote(micLeft_output);
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
