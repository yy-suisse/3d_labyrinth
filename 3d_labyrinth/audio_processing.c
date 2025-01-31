#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>


#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <arm_const_structs.h>


//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];

// last 2 values of phase difference
static float phase_dif = 0;
static float phase_dif_old = 0;


// coefficients of filters
#define FILTRE_COEF 				 0.3
#define FILTRE_VALEUR_ABERRANTE      0.5

// threshold in order to avoid to detect noise as a maximum of magnitude
#define MIN_VALUE_THRESHOLD	10000 



/***************************INTERNAL FUNCTIONS************************************/


/*
*	function used to detect the highest value in a buffer
*	and calculate a filtered difference of phase
*/
void sound_remote(float* data1, float* data2)
{

	float phase1 = 0;
	float phase2 = 0;

	float max_norm1 = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index1 = -1;

	float max_norm2 = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index2 = -1;


	//search for the highest peak in data1
	for(uint16_t i = 0 ; i <= FFT_SIZE/2 ; i++)
	{
		if(data1[i] > max_norm1)
		{
			max_norm1 = data1[i];
			max_norm_index1 = i;
		}
	}

	//search for the highest peak in data2
	for(uint16_t j = 0 ; j <= FFT_SIZE/2 ; j++)
	{
		if(data2[j] > max_norm2)
		{
			max_norm2 = data2[j];
			max_norm_index2 = j;
		}
	}


	/* since 2 microphones receive the same sound, if the frequencies of peak value in 2 data arrays are the same,
	 * then the source is correct to calculate phase difference
	 */
	if (max_norm_index1 == max_norm_index2)
	{
		// atan2f (x,y) returns arctan (x/y) within -pi pi
		phase1 = atan2f(micLeft_cmplx_input[2*max_norm_index1+1], micLeft_cmplx_input[2*max_norm_index1]);
		phase2 = atan2f(micRight_cmplx_input[2*max_norm_index2+1], micRight_cmplx_input[2*max_norm_index2]);

		// filter to make modifications of phase difference more continuous
		phase_dif = (1-FILTRE_COEF)*(phase1 - phase2) + FILTRE_COEF*phase_dif_old;




		// update phase difference value for the further measurement
		if (phase_dif > -FILTRE_VALEUR_ABERRANTE && phase_dif < FILTRE_VALEUR_ABERRANTE)
		{
			phase_dif_old = phase_dif;
		}

	}
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples)
{

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;



	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
	{
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}



	if(nb_samples >= (2 * FFT_SIZE))
	{
		/*	FFT processing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micRight_cmplx_input, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micLeft_cmplx_input, 0, 1);



		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);


		nb_samples = 0;

		sound_remote(micLeft_output, micRight_output);
	}
}


float get_phase_dif(void)
{

	if (phase_dif > -FILTRE_VALEUR_ABERRANTE && phase_dif < FILTRE_VALEUR_ABERRANTE)
	{
		return phase_dif;
	}

	else return phase_dif_old;
}

/**************************END PUBLIC FUNCTIONS***********************************/
