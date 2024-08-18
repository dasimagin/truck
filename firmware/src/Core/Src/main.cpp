#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

// #include "MotorControl.h"
#include "SensorPolling.h"
#include "Protocol.h"
#include "ServoController.h"

#include "board.h"

#define SIMPLEFOC_DISABLE_DEBUG

extern "C" void __aeabi_atexit() {};     // for virtual destructors
extern "C" void __cxa_pure_virtual();   // for abstract base class
// extern "C" void __cxa_guard_acquire();  // for local static init
// extern "C" void __cxa_guard_release();  // for local static init

#pragma weak __cxa_pure_virtual = __aeabi_atexit
#pragma weak __cxa_guard_acquire = __aeabi_atexit
#pragma weak __cxa_guard_release = __aeabi_atexit

int main() {
    board_init();
    printf("Start\n");

    // static MotorControl& MT = MotorControl::getInstance();
    static SensorPolling& SP = SensorPolling::getInstance();
    static ServoController& SC = ServoController::getInstance();
    static Protocol PR;
    SP.init();
    SP.start();
//    MT.init();
//    MT.calibrate();
//    PR.init();
//    SC.init();

    printf("Scheduling\n");
    board_start_rtos_timer();
    vTaskStartScheduler();

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
