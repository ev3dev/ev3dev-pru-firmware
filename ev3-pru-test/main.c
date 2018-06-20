/*
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>

#include "gpio.h"
#include "resource_table_empty.h"

static volatile gpio *GPIO = (void *)(0x01e26000);

int main(void) {
    while (true) {
        /* blink the left green LED on the EV3 */
        GPIO->OUT_DATA67_bit.GP6P7 = 1;
        __delay_cycles(100000000);
        GPIO->OUT_DATA67_bit.GP6P7 = 0;
        __delay_cycles(100000000);
    }
}
