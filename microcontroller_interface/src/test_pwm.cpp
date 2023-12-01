#include <lgpio.h>
#include <csignal>
#include <unistd.h>
#include <iostream>

// left wheel PWM pin
#define ENA 6
// right wheel PWM pin
#define ENB 26
// left wheel direction pins
#define AIN1 12
#define AIN2 13
// right wheel direction pins
#define BIN1 20
#define BIN2 21

#define PWM_FREQ 100
#define DUTY_CYCLE 50

#define CHECK_HANDLE(handle) \
    do { \
        if (handle < 0) { \
            std::cerr << "Bad handle at " << __FILE__ << ":" << __LINE__ << std::endl; \
            return -1; \
        } \
    } while(false) \

int h; // handle for lgpio library

void cleanUp()
{
    lgGpioWrite(h, AIN1, 0);
    lgGpioWrite(h, AIN2, 0);
    lgGpioWrite(h, BIN1, 0);
    lgGpioWrite(h, BIN2, 0);
    lgGpioWrite(h, ENA, 0);
    lgGpioWrite(h, ENB, 0);

    lgGpioFree(h, AIN1);
    lgGpioFree(h, AIN2);
    lgGpioFree(h, BIN1);
    lgGpioFree(h, BIN2);
    lgGpioFree(h, ENA);
    lgGpioFree(h, ENB);
 
    lgGpiochipClose(h);
}

void signalHandler(int signum)
{
    cleanUp();
    exit(signum);
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    int chip = 0;
    h = lgGpiochipOpen(chip);
    CHECK_HANDLE(h);

    // claim gpios
    lgGpioClaimOutput(h, 0, AIN1, 0);
    lgGpioClaimOutput(h, 0, AIN2, 0);
    lgGpioClaimOutput(h, 0, BIN1, 0);
    lgGpioClaimOutput(h, 0, BIN2, 0);
    lgGpioClaimOutput(h, 0, ENA, 0);
    lgGpioClaimOutput(h, 0, ENB, 0);

    // go forward
    lgGpioWrite(h, BIN1, 0);
    lgGpioWrite(h, BIN2, 1);
    lgGpioWrite(h, AIN1, 0);
    lgGpioWrite(h, AIN2, 1);
    lgTxPwm(h, ENB, PWM_FREQ, DUTY_CYCLE, 0, 0);
    lgTxPwm(h, ENA, PWM_FREQ, DUTY_CYCLE, 0, 0);

    while (true)
    {
        sleep(1); // avoid busy-looping
    }
    return 0;
}
