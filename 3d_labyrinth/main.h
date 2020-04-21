#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/** enumeration for 2 states of control:
 * 0: IMU  ;
 * 1: son  */
enum ETAT {MODE_IMU, MODE_SON};


/**
 * @brief  send control mode to control module
 *
 * @return control mode read for selector
 */
bool get_mode_selector (void);



/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
