#ifndef CONTROLE_H
#define CONTROLE_H

#define NOMBRE_LED_RGB				4

// value of ambient light variation to detect the end condition
#define SEUIL_DETECTION_FIN			250

// threshold values of proximity between captors and robot that we don't want to go over
#define SEUIL_PROXI_FB				250
#define SEUIL_PROXI_LATERAL			200

// threshold value to avoid movement of the robot when the robot is too horizontal
#define TRESHOLD_IMU 				0.4

// average filtering made by averaging NB_VALEUR_FILTRE values
#define NB_VALEUR_FILTRE            8

// typical values of speed used in the program
#define VITESSE_BASE 				150
#define VITESSE_MODE_SON			275
#define NO_SPEED					0

// values for the PI regulator
#define ERROR_THRESHOLD			0.1f
#define KP						1000.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)


 /*
 * @brief   start the proximity analysis thread
 */
void prox_analyse_start(void);


/*
* @brief   start the imu control thread
*/
void controle_imu_start(void);



/*
* @brief   start the sound control thread
*/
void controle_son_start(void);


/*
* @brief   stop the proximity analysis thread and his LED activities at the end of mission
*/
void prox_analyse_stop(void);


/*
* @brief  stop the imu control thread at the end of mission
*/
void controle_imu_stop(void);


/*
* @brief   stop the sound control thread at the end of mission
*/
void controle_son_stop(void);


/*
* @brief   send the analysis result about if mission is finished
*
* @return  boolean
* 		   mission is finished: 1
* 		   mission is not finished yet: 0
*/
bool get_detection_fin(void);


#endif /* CONTROLE_H */
