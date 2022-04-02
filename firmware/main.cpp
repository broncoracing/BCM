#include <mbed.h>
#include "CAN.h"
#include "main.h"
#include "SWO/SWO.h"
#include "can-ids/CAN_IDS.h"

using namespace std::chrono;

SWO_Channel swo("channel");

CAN can(PIN_CAN_RX, PIN_CAN_TX, CAN_BAUD);

// Initialize outputs
PwmOut fan(PIN_FAN);

DigitalOut upshift(PIN_UPSHIFT);
DigitalOut downshift(PIN_DOWNSHIFT);
DigitalOut pump(PIN_WATERPUMP);
DigitalOut etcEnable(PIN_ETCENABLE);
DigitalOut ecuPower(PIN_ECUPOWER);

EventQueue queue(32 * EVENTS_EVENT_SIZE);

Thread t;

Ticker ecu_loop;
Ticker dbw_loop;

Timer ecuTimer;
Timer canTimer;

// init states
BCMState bcmState;
EngineState engineState;
ThrottleState throttleState;

// Function Prototypes
void init_outputs();
void init_timers();
void can_received();
void shift_received(CANMessage msg);
void ecu_received(CANMessage msg);
void dbw_received(CANMessage msg);
void check_state();
void check_dbw_status();
void set_state();



// Event Handlers
void can_received_handler() {
    queue.call(can_received);
}

void state_update_handler() {
    queue.call(check_state);
}

void dbw_check_handler() {
    queue.call(check_dbw_status);
}

int main() {
#ifdef DEBUG_SWO
    // Print debug info over SWO
    swo.claim();
    printf("Claimed SWO output\n");
#endif //DEBUG_SWO

    // Start event queue thread
    printf("Starting event queue thread...\n");
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    printf("Event queue thread running.\n");

    init_outputs();
    init_timers();

    // Enable CAN callback
    printf("Starting CAN Listener...\n");
    can.attach(can_received_handler);
    printf("Listening on CAN\n");

#ifndef ETC_I_HARDLY_KNOW_HER
    // Enable ecu state callback
    ecu_loop.attach(&state_update_handler, STATE_UPDATE_INTERVAL);
#else
    bcmState.state = override;
    set_state();
#endif

    // Enable etc safe callback
    dbw_loop.attach(&dbw_check_handler, STATE_UPDATE_INTERVAL);

    while(true)
        sleep();
}

void init_outputs() {
    // Initialize pinouts to rest values
    upshift.write(0);
    downshift.write(0);
    pump.write(0);
    fan.write(0);

    // Power on ECU
    ecuPower.write(1);

    // Wait for ECU to boot, then turn on etc safe rail
    ThisThread::sleep_for(ECU_BOOT_TIME);
    etcEnable.write(1);
    ThisThread::sleep_for(CAN_BOOT_TIME);
}

void init_timers() {
    ecuTimer.reset();
    ecuTimer.start();

    canTimer.reset();
    canTimer.start();
}

void can_received() {
    CANMessage msg;
    while(can.read(msg)) {
        canTimer.reset();

        switch (msg.id) {
            case STEERING_WHEEL_ID:
                shift_received(msg);
                break;
            case ECU1_ID:
            case ECU2_ID:
            case ECU3_ID:
                ecu_received(msg);
                break;
            case DBW_SENSORS_ID:
                dbw_received(msg);
                break;
            default:
                break;
        }
    }
}

void shift_received(CANMessage msg) {
    if (msg.data[1]) {
        printf("\n---------- UPSHIFT ----------\n");
        upshift.write(1);
        ThisThread::sleep_for(UPSHIFT_TIME);
        upshift.write(0);
    } else if (msg.data[2]) {
        printf("\n---------- DOWNSHIFT ----------\n");
        downshift.write(1);
        ThisThread::sleep_for(DOWNSHIFT_TIME);
        downshift.write(0);
    }
}

void ecu_received(CANMessage msg) {
    ecuTimer.reset();

    switch(msg.id) {
        case ECU1_ID:
            engineState.rpm = (msg.data[1] << 8) + msg.data[0];
            break;
        case ECU2_ID:
            // TODO: Verify message format, should probably abstract out in broncoracing/can-ids
            short temp = (msg.data[2] << 8) + msg.data[1];
            engineState.waterTemp = ((temp / 10.0) * 1.8) + 32;
            break;
    }
}

void dbw_received(CANMessage msg) {
    // each value is unsigned int 0-100
    uint8_t apps1 = msg.data[0];
    uint8_t apps2 = msg.data[1];
    uint8_t tps1 =  msg.data[2];
    uint8_t tps2 =  msg.data[3];
    {
        ScopedLock<Mutex> lock(throttleState.mutex);
        throttleState.APPS1 = apps1;
        throttleState.APPS2 = apps2;
        throttleState.TPS1 = tps1;
        throttleState.TPS2 = tps2;
    }
}

// TODO kinda spaghetti tbh
void check_state() {
    {
        ScopedLock<Mutex> lock(bcmState.mutex);
        // Check timers
        if (duration_cast<milliseconds>(ecuTimer.elapsed_time()) > ECU_TIMEOUT)
        {
            bcmState.ECUConnected = false;
            engineState.running = false;

            engineState.rpm = 0;
            engineState.waterTemp = 0;
        }
        else
        {
            bcmState.ECUConnected = true;
            engineState.running = true;
        }

        if (duration_cast<milliseconds>(canTimer.elapsed_time()) > CAN_TIMEOUT)
            bcmState.CANConnected = false;
        else
            bcmState.CANConnected = true;

        if (!(bcmState.CANConnected))
            bcmState.state = safety;
        else if (engineState.rpm == 0 && bcmState.ECUConnected && bcmState.CANConnected)
            bcmState.state = engineOff;
        else if (engineState.rpm > ENGINE_IDLE_RPM && engineState.waterTemp >= (ENGINE_WARM_F + ENGINE_TEMP_DEADBAND))
            bcmState.state = hotRunning;
        else if (engineState.rpm > ENGINE_IDLE_RPM && engineState.waterTemp <= ENGINE_WARM_F)
            bcmState.state = coldRunning;
    }
    set_state();
}

void check_dbw_status() {
    {
        ScopedLock<Mutex> lock(throttleState.mutex);

        // Check APPS1 vs APPS2
        if (abs(throttleState.APPS1 - throttleState.APPS2) >= APPS_VS_APPS_MAX_ERROR)
            throttleState.APPSerrorCount++;
        else
            throttleState.APPSerrorCount = 0;

        // Check TPS1 vs TPS2
        if (abs(throttleState.TPS1 - throttleState.TPS2) >= TPS_VS_TPS_MAX_ERROR)
            throttleState.TPSerrorCount++;
        else
            throttleState.TPSerrorCount = 0;

        // Check APPS vs TPS as long as one is above idle threshold
        if ((abs(throttleState.APPS1 - throttleState.TPS1) >= APPS_VS_TPS_MAX_ERROR) &&
            ((throttleState.TPS1 > APPS_VS_TPS_ENABLE_THRESHOLD) ||
            (throttleState.TPS2 > APPS_VS_TPS_ENABLE_THRESHOLD)))
            throttleState.APPSvsTPSerrorCount++;
        else
            throttleState.APPSvsTPSerrorCount = 0;

        // If significant errors have occurred, shut off ETC SAFE power rail
        if ((throttleState.APPSerrorCount >= ETHROTTLE_MAX_ERROR_COUNT) ||
            (throttleState.TPSerrorCount >= ETHROTTLE_MAX_ERROR_COUNT) ||
            (throttleState.APPSvsTPSerrorCount >= APPS_VS_TPS_MAX_ERROR_COUNT))
        {
            etcEnable.write(0);
            throttleState.eThrottleErrorOccurred = true;
        }
    }
}

void set_state() {
    switch (bcmState.state) {
        case safety:
            fan.write(FAN_ACTIVE_DC);
            pump.write(1);
            etcEnable.write(0);
            break;
        case engineOff:
            if (engineState.waterTemp > ENGINE_WARM_F)
            {
                fan.write(FAN_COOLDOWN_DC);
                pump.write(1);
            }
            else
            {
                fan.write(0);
                pump.write(0);
            }
            break;
        case hotRunning:
            fan.write(FAN_ACTIVE_DC);
            pump.write(1);
            break;
        case coldRunning:
            fan.write(0);
            pump.write(1);
            break;
        case override:
            etcEnable.write(1);
            fan.write(FAN_ACTIVE_DC);
            pump.write(1);
            break;
        default:
            printf("something is very broken");
            break;
    }
}