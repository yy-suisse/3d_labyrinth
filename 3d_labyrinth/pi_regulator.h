#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define NOMBRE_LED_RGB				4
#define SEUIL_DETECTION_FIN			250
#define SEUIL_PROXI_FB				250
#define SEUIL_PROXI_LATERAL			200

#define TRESHOLD_IMU 				0.4
#define NB_VALEUR_FILTRE            8
#define VITESSE_BASE 				150
#define NO_SPEED					0

#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera  ///////////
#define KP						1000.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

//start the PI regulator thread
void pi_regulator_start(void);

// start the proximity analyse thread
void prox_analyse_start(void);

// start the imu control thread
void controle_imu_start(void);

#endif /* PI_REGULATOR_H */
