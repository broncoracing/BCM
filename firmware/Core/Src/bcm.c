#include "bcm.h"

#include "pwm.h"
#include "horn.h"

enum BCM_State_t bcm_state = RUNNING;

uint32_t state_counter = 0;

void bcm_update(){
    ++state_counter;
    // switch(bcm_state) {
    //     case PRE_STARTUP:
    //         bcm_state = STARTUP;
    //         state_counter = 0;
    //         break;
    //     case STARTUP:
    //         if(state_counter == 1) {
    //             start_honk();
    //         }
    //         if(state_counter == 2200) {
    //             start_honk();
    //         }
    //         if(state_counter > 3400) {
    //             state_counter = 0;
    //             bcm_state = RUNNING;
    //         }
    //         break;
    //     case RUNNING:

    //         if(state_counter > 5000) {
    //             state_counter = 0;
    //             bcm_state = PRE_STARTUP;
    //         }
    //         break;
    // }
}
