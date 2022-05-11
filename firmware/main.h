#ifndef BCM_PREFERENCES
#define BCM_PREFERENCES


/*********** System Settings ***********/

//#define PRINT_STATUS                             // Uncomment - print status
//#define ETC_I_HARDLY_KNOW_HER                    // Uncomment - Disable ETC safety for testing ONLY, forces fan + pump on
#define STATUS_PRINT_INTERVAL           2000ms     // How often to print overall running status if PRINT_STATUS is enabled

// TIMEOUTS AND INTERVALS
#define ECU_TIMEOUT                     2000ms
#define CAN_TIMEOUT                     2000ms
#define COOLING_KILL                    10000ms
#define STATE_UPDATE_INTERVAL           1000ms
#define DBW_CHECK_INTERVAL              10ms        // Frequency to check DBW sensors for error
#define WATCHDOG_TIMEOUT                2000        // Watchdog timeout in ms

#define ECU_BOOT_TIME                   250ms
#define CAN_BOOT_TIME                   50ms

// Pinouts
#define PIN_CAN_RX                      PA_11
#define PIN_CAN_TX                      PA_12
#define PIN_DOWNSHIFT                   PB_0
#define PIN_UPSHIFT                     PB_1
#define PIN_WATERPUMP                   PB_2
#define PIN_FAN                         PB_4
#define PIN_ECUPOWER                    PB_6
#define PIN_ETCENABLE                   PB_5


/*********** Cooling Settings ***********/

// Cooldown times in milliseconds
#define FAN_COOLDOWN                    30000ms
#define WATERPUMP_COOLDOWN              60000ms

// Fan duty cycles
#define FAN_ACTIVE_DC                   1.0f
#define FAN_COOLDOWN_DC                 0.5f

// For cooling fan and pump
#define PWM_PERIOD_US                   1000

// Parameters
#define ENGINE_WARM                     38          // Celsius
#define ENGINE_TEMP_DEADBAND            7           // Celsius


/*********** Engine Settings ***********/

#define ENGINE_IDLE_RPM                 1000

// Shift times in microseconds
#define DOWNSHIFT_TIME                  140000us
#define UPSHIFT_TIME                    125000us


/*********** ETC Settings ***********/

#define ETHROTTLE_MAX_ERROR_COUNT       10
#define APPS_VS_APPS_MAX_ERROR          10
#define TPS_VS_TPS_MAX_ERROR            10

#define APPS_VS_TPS_MAX_ERROR_COUNT     50
#define APPS_VS_TPS_MAX_ERROR           21           // include idle offset

// when TPS is above this value, APPS vs TPS error checking is active
#define APPS_VS_TPS_ENABLE_THRESHOLD    11


/*********** Utility ***********/

// Vehicle states
enum State {
    override = -1,
    safety = 0,
    engineOff = 1,
    cooldown = 2,
    coldRunning = 3,
    hotRunning = 4,
};

// Disable printing macro
#ifndef PRINT_STATUS
    #define printf(fmt, ...) (0)
#endif

struct BCMState {
    Mutex mutex;
    volatile bool CANReceived = false;
    volatile bool CANConnected = true;
    volatile bool ECUConnected = false;
    State state = engineOff;
};

struct EngineState {
    volatile bool running = false;
    volatile float waterTemp = 0.0;
    volatile int rpm = 0;
};

struct ThrottleState {
    Mutex mutex;
    volatile uint8_t APPS1 = 0;
    volatile uint8_t APPS2 = 0;
    volatile uint8_t TPS1 = 0;
    volatile uint8_t TPS2 = 0;
    volatile int APPSerrorCount = 0;
    volatile int TPSerrorCount = 0;
    volatile int APPSvsTPSerrorCount = 0;
    volatile bool eThrottleErrorOccurred = false;
};

#endif // end BCM_PREFERENCES
