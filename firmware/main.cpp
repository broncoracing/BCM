#include <mbed.h>
#include "CAN.h"
#include "main.h"
#include "SWO/SWO.h"
#include "can-ids/CAN_IDS.h"

SWO_Channel swo("channel");

CAN can(PIN_CAN_RX, PIN_CAN_TX, CAN_BAUD);

// Initialize outputs
PwmOut fan(PIN_FAN);

DigitalOut upshift(PIN_UPSHIFT);
DigitalOut downshift(PIN_DOWNSHIFT);
DigitalOut pump(PIN_WATERPUMP);
DigitalOut etcEnable(PIN_ETCENABLE);
DigitalOut etcPower(PIN_ETCPOWER);

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;

// init states
BCMState bcmState;
EngineState engineState;
ThrottleState throttleState;

// Function Prototypes
void can_received();
void shift_received();
void upshift();
void downshift();
void halfshift();
void

// Event Handlers
void can_received_handler() {
    queue.call(can_received);
}


int main()
{
#ifdef DEBUG_SWO
    // Print debug info over SWO
    swo.claim();
    printf("Claimed SWO output\n");
#endif //DEBUG_SWO

    // Start event queue thread
    printf("Starting event queue thread...\n");
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    printf("Event queue thread running.\n");

    // Enable CAN callback
    printf("Starting CAN Listener...\n");
    can.attach(can_received_handler);
    printf("Listening on CAN\n");

    while(true){
        sleep();
    }
}

void can_received() {
    CANMessage msg;
    while(can.read(msg)) {
        switch (msg.id) {
            case STEERING_WHEEL_ID:
                shift_received(msg);
                break;
            //TODO
            default:
                break;
        }
    }
}

void shift_received(CANMessage msg) {
    if(msg.data[1])
        upshift();
    if (msg.data[2])
        downshift();
}

void upShift()
{
    printf("\n---------- UPSHIFT ----------\n");

    upShiftPin.write(1);
    ThisThread::sleep_for(UPSHIFT_TIME);
    upShiftPin.write(0);
}

void downShift()
{
    printf("\n---------- DOWNSHIFT ----------\n");

    downShiftPin.write(1);
    ThisThread::sleep_for(DOWNSHIFT_TIME);
    downShiftPin.write(0);
}
