#include "horn.h"

#include "pwm.h"

uint32_t honk_timer;

enum HornState_t {
    HONKING,
    NOT_HONKING
} horn_state = NOT_HONKING;

void start_honk(void){
    honk_timer = 0;
    horn_state = HONKING;
}

void horn_update(void){
    if(honk_timer < 500) {
        pwm_duties[4] = 0xA000 + (0x3000 / 500) * honk_timer; // aw
    } else if(honk_timer < 1000) {
        pwm_duties[4] = 0xD000; // OOO
    } else if(honk_timer < 1200){
        pwm_duties[4] = 0x8800; // gah
    } else {
        pwm_duties[4] = 0;
    }

    if(honk_timer > 1200) {
        horn_state = NOT_HONKING;
        pwm_duties[4] = 0;
    }

    ++honk_timer;
}
