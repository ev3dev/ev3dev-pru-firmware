/*
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>

#include <am18xx/sys_gpio.h>

#include "resource_table_empty.h"

int main(void) {
    while (true) {
        /* blink the left green LED on the EV3 */
        GPIO.OUT_DATA67_bit.GP6P7 = 1;
        __delay_cycles(100000000);
        GPIO.OUT_DATA67_bit.GP6P7 = 0;
        __delay_cycles(100000000);
    }
}
