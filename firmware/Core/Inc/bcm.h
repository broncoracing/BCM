#ifndef BCM_H
#define BCM_H

#include "main.h"

enum BCM_State_t {
    PRE_STARTUP,
    STARTUP,
    RUNNING,
};

extern enum BCM_State_t bcm_state;

void bcm_update();

#endif