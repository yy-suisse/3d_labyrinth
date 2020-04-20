#ifndef CONTROLE_H
#define CONTROLE_H

#define NOMBRE_LED_RGB				4
#define SEUIL_DETECTION_FIN			250
#define SEUIL_PROXI_FB				250
#define SEUIL_PROXI_LATERAL			200

#define TRESHOLD_IMU 				0.4
#define NB_VALEUR_FILTRE            8
#define VITESSE_BASE 				150
#define NO_SPEED					0

#define ERROR_THRESHOLD			0.1f	//threshold pour le PI
#define KP						1000.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)


 /*
 * @brief   start the proximity analyse thread
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


void prox_analyse_stop(void);


void controle_imu_stop(void);


void controle_son_stop(void);


bool get_detection_fin(void);


#endif /* CONTROLE_H */
