/*
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>

#include <am18xx/sys_gpio.h>
#include <am18xx/sys_timer.h>

#include "resource_table_empty.h"

int main(void) {
    uint32_t start;

    /* blink the left green LED on the EV3 */
    while (true) {
        /* LED is connected to GPIO 6[7] */
        GPIO.OUT_DATA67_bit.GP6P7 = 1;

        /* TIMER64P0.TIM34 is configured by Linux as a free run counter so we
         * can use it here to keep track of time. This timer runs off of the
         * external oscillator, so it runs at 24MHz (each count is 41.67ns).
         * Since it counts up to the full unsigned 32-bit value, we can
         * subtract without worrying about if the value wrapped around.
         */
        start = TIMER64P0.TIM34;
        while (TIMER64P0.TIM34 - start < 12000000) { }

        GPIO.OUT_DATA67_bit.GP6P7 = 0;

        start = TIMER64P0.TIM34;
        while (TIMER64P0.TIM34 - start < 12000000) { }
    }
}
