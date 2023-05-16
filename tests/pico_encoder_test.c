/**
 * Copyright (c) 2021 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include <mbot/encoder/encoder.h>
#define DELTA "\u0394"
#define PHI "\u03A6"
#define THETA "\u0398"
#define PSI "\u03A8"
int main() {
    int d1, d2, d3, t1, t2, t3 = 0;
    mbot_encoder_init();
    sleep_ms(2000);
    printf("\033[2J\r");
    printf("| ENC 0%s | ENC 1%s | ENC 2%s |  ENC 0 |  ENC 1 |  ENC 2 |\n", DELTA, DELTA, DELTA);
    while (1) {
        d1 = mbot_encoder_read_delta(1);
        d2 = mbot_encoder_read_delta(2);
        d3 = mbot_encoder_read_delta(3);
        t1 = mbot_encoder_read_count(1);
        t2 = mbot_encoder_read_count(2);
        t3 = mbot_encoder_read_count(3);
        printf("\r%7d| %7d| %7d| %7d| %7d| %7d|", d1, d2, d3, t1, t2, t3);
        sleep_ms(20);
    }
}

