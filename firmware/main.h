#ifndef BCM_PREFERENCES
#define BCM_PREFERENCES

#define PRINT_STATUS                               // uncomment - print status
#define DEBUG_SWO                                  // Debug over SWO

#define SERIAL_BAUD                     921600
#define ECU_TIMEOUT_MS                  2000
#define CAN_TIMEOUT_MS                  2000
#define COOLING_KILL_MS                 10000

// Cooldown times in milliseconds
#define FAN_COOLDOWN_MS                 30000ms
#define WATERPUMP_COOLDOWN_MS           60000ms

// Active duty cycles
#define WATERPUMP_ACTIVE_DC             1.0f
#define FAN_ACTIVE_DC                   1.0f

// Cooldown duty cycles
#define WATERPUMP_COOLDOWN_DC           0.7f
#define FAN_COOLDOWN_DC                 0.5f

// For cooling fan and pump
#define PWM_PERIOD_US                   100

// Parameters
#define ENGINE_WARM_F                   100       // Fahrenheit
#define ENGINE_TEMP_DEADBAND            20        // Fahrenheit

// State Machine states
#define safetyState                     0
#define engineOffState                  1
#define cooldownState                   2
#define coldRunningState                3
#define hotRunningState                 4

// Shift times in milliseconds
#define DOWNSHIFT_TIME                  200
#define UPSHIFT_TIME                    150
#define HALFSHIFT_TIME                  150

// ethrottle safety
#define ETHROTTLE_MAX_ERROR_COUNT       10
#define APPS_VS_APPS_MAX_ERROR          10
#define TPS_VS_TPS_MAX_ERROR            10

#define APPS_VS_TPS_MAX_ERROR_COUNT     50
#define APPS_VS_TPS_MAX_ERROR           20        // include idle offset

// when TPS is above this value, APPS vs TPS error checking is active
#define APPS_VS_TPS_ENABLE_THRESHOLD    50

// Disable printing macro
#ifdef PRINT_STATUS
    #define printf(fmt, ...) (0)
#endif

// Pinouts
#define PIN_CAN_RX                      PA_11
#define PIN_CAN_TX                      PA_12
#define PIN_DOWNSHIFT                   PB_0
#define PIN_UPSHIFT                     PB_1
#define PIN_WATERPUMP                   PB_2
#define PIN_FAN                         PB_4
#define PIN_ETCPOWER                    PB_5
#define PIN_ETCENABLE                   PB_6

struct BCMState {
    Mutex mutex;
    volatile bool canReceived = false;
    volatile bool ECUConnected = false;
};

struct EngineState {
    Mutex mutex;
    volatile bool engineRunning = false;
    volatile float waterTemp = 0.0;
    volatile float batteryVoltage = 0.0;
    volatile int rpm = 0;
};

struct ThrottleState {
    Mutex mutex;
    volatile u_int8_t APPS1 = 0;
    volatile u_int8_t APPS2 = 0;
    volatile u_int8_t TPS1 = 0;
    volatile u_int8_t TPS2 = 0;
    volatile int APPSerrorCount = 0;
    volatile int TPSerrorCount = 0;
    volatile int APPSvsTPSerrorCount = 0;
    volatile bool eThrottleErrorOccurred = false;
};
#endif // end BCM_PREFERENCES
