#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"///////////////
#include "msgbus/messagebus.h"///////////////
#include "parameter/parameter.h"//////////////////

enum ETAT {MODE_IMU, MODE_SON};

bool get_mode_selector (void);

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
