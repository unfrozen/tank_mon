/*
 *  File name:  tank_mon.c
 *  Date first: 11/05/2018
 *  Date last:  12/18/2018
 *
 *  Description: Monitor water level in IBC (tote) tank using the
 *		HC-SR04 range finder.
 *
 *  Author:     Richard Hodges
 *
 *  Copyright (C) 2018 Richard Hodges. All rights reserved.
 *  Permission is hereby granted for any use.
 *
 *  This started from the lib_ping example code.
 *
 ******************************************************************************
 *
 */

#include "stm8s_header.h"

#include "lib_bindec.h"
#include "lib_clock.h"
#include "lib_ping.h"
#include "lib_tm1638.h"
#include "lib_uart.h"

void setup(void);

#define TANK_LITERS	1000	/* tank size */

/* Using pin A2 for trigger */

const IO_PIN pin_trig1 = {	/* A2 */
    &PA_ODR, 0x04
};

/* Using A3 to receive echo */

#define PING_CHAN PING_CHAN3

volatile char	clock_tenths;	/* 1/10 second counter 0-255 */
volatile char   clock_msecs;	/* millisecond counter */
volatile int	d1;		/* current distance */

/* callbacks provided by lib_clock */

void clock_ms(void);	/* millisecond callback */
void clock_10(void);	/* 1/10 second callback */

/* callbacks provided by lib_ping */

void ping_cb1(int);	/* ping channel 1 callback */

int dist_conv(int);	/* convert microseconds (5 samples) to liters */
int conv_gallons(int);	/* convert liters to gallons */
char conv_point(char);	/* get decimal point char */

char conv_percent(int);	/* convert liters to percent of tank */

void print_uart(int, int);	/* send data to UART */

/* Pins to use as triggers and callback functions */

const IO_CALL_INT ping_cfg[] = {
	{ &pin_trig1, ping_cb1 },
};

/* Define our output device calls */

#define putc	tm1638_putc
#define puts	tm1638_puts
#define getc	tm1638_getc
#define led_on(led)  tm1638_setled(led, 1)
#define led_off(led) tm1638_setled(led, 0)

#define BEEP_OFF	0
#define BEEP_ON		0x2f	/* 1khz beep */

#define MODE_GALLONS	1
#define MODE_LITERS	2
#define MODE_PERCENT	3
#define MODE_PING	4

const int ping_tab[];

/******************************************************************************
 *
 *  Monitor the water level
 *  Send percentage full to main display
 *  Send time and raw millisecond value to UART
 *  Use average of five counts each.
 */

int main() {
    char	display[10];
    char	last_tenth;
    char	dcount;
    unsigned int dsum;
    char	key, mode;
    unsigned int liters, gallons, percent;

    setup();
    clock_init(clock_ms, clock_10);
    ping_init(PING_CHAN, ping_cfg);
    tm1638_init(TM1638_8);	/* 8 key display */
    tm1638_bright(4);
    uart_init(BAUD_115200);

    last_tenth = 0;
    dcount = 0;
    dsum = 0;
    mode = MODE_GALLONS;
    led_on(0);

    do {
	while (last_tenth == clock_tenths);
	last_tenth = clock_tenths;
	BEEP_CSR = BEEP_OFF;

	if (d1 >= 0) {
	    dsum += d1;
	    dcount++;
	}
	d1 = -1;		/* no echo response */
	ping_send(PING_CHAN);

	if (dcount < 5)
	    continue;
	dcount = 0;

	tm1638_curs(0);

	liters = dist_conv(dsum / 5);

	switch (mode) {
	case MODE_GALLONS :
	    gallons = conv_gallons(liters);
	    bin16_dec_rlz(gallons >> 5, display);
	    display[5] = '.';
	    display[6] = conv_point(gallons & 31);
	    display[7] = 0;
	    puts("L ");
	    puts(display);
	    break;
	case MODE_LITERS :
	    bin16_dec_rlz(liters, display);
	    putc('L');
	    puts(display);
	    puts(" L");
	    break;
	case MODE_PERCENT :
	    percent = conv_percent(liters);
	    bin16_dec_rlz(percent, display);
	    puts("L ");
	    puts(display + 2);
	    puts(" PC");
	    break;
	case MODE_PING :
	    bin16_dec_rlz(dsum, display);
	    puts("P  ");
	    puts(display);
	    break;
	}
	print_uart(liters, dsum);

//	BEEP_CSR = BEEP_ON;
	dsum = 0;

	key = getc();
	if (!key || key & 0x80)	/* no key or key release */
	    continue;
	led_off(mode);
	mode = key & 7;
	led_on(mode);

    } while(1);
}

/******************************************************************************
 *
 *  Convert raw distance in microseconds to liters
 *
 *  in: microseconds
 * out: liters
 */

int dist_conv(int raw)
{
    unsigned int liters, diff, *ptr;
    unsigned int temp;

    ptr = ping_tab;

    if (raw >= *ptr)
	return 0;

    liters = 0;
    ptr++;
    while (*ptr > raw) {
	liters += 100;
	ptr ++;
    }
    ptr--;
    diff = *ptr - raw;		/* interpolation point */
    diff *= 100;		/* times 100 liters */
    temp = *ptr - ptr[1];
    diff /= temp;

    liters += diff;
    return liters;
}

/******************************************************************************
 *
 *  Got new ping value (callback)
 *  in: round  trip time in microseconds
 *      -1 (0xffff) indicates no echo response.
 */

void ping_cb1(int val)
{
    d1 = val;
}

/******************************************************************************
 *
 *  Millisecond callback
 */

void clock_ms(void)
{
    clock_msecs++;
    tm1638_poll();
}

/******************************************************************************
 *
 *  1/10 second callback
 */

void clock_10(void)
{
    clock_tenths++;
    if (clock_tenths & 3)
	PB_ODR |= 0x20;		/* LED off 75% */
    else
	PB_ODR &= 0xdf;		/* LED on 25% */
}

/******************************************************************************
 *
 *  Display modes: (press button #2 to #5)
 *
 *  1. "L  250.5"  Water level in Gallons
 *  2. "L 1000 L"  Water level in Liters
 *  3. "L 100 PC"  Water level in Percentage of full
 *  4. "P  12345"  Water level Ping time (for calibration)

 *  LCD: Use 2x16 or 2x20, show raw microseconds and calibrated percent
 *  __________________
 * | Day 255 00:00:00 |
 * | 100% uSecs=30000 |
 *  ------------------
 *
 *  UART: Show same info on one line.
 *
 *  DD HH:MM:SS liters ping_time
 *
 ******************************************************************************
 *
 *  Print clock, liters, and ping time to UART
 *
 *  in:  liters, ping time (sum of 5)
 */

void print_uart(int liters, int ping)
{
    char	display[10];
    char	time[4];

    clock_bin_get(time);
    bin8_dec2(time[0], display);
    uart_puts(display);
    uart_put(' ');

    clock_string(display);
    uart_puts(display);
    uart_put(' ');

    bin16_dec_rlz(liters, display);
    uart_puts(display);
    uart_put(' ');

    bin16_dec_rlz(ping, display);
    uart_puts(display);
    uart_crlf();
}

/******************************************************************************
 *
 *  Board and globals setup
 */

void setup(void)
{
    CLK_CKDIVR = 0x00;	/* clock 16mhz if STM8S103 */

    clock_tenths = 0;

    PA_ODR = 0;
    PA_DDR = 0x04;	/* A2 is trigger */
    PA_CR1 = 0xff;
    PA_CR2 = 0x00;

    PB_DDR = 0x20;	/* output LED */
    PB_CR1 = 0xff;     	/* inputs have pullup, outputs not open drain */
    PB_CR2 = 0x00;	/* no interrupts, 2mhz output */

    PC_CR1 = 0xff;

    PD_DDR = 0x00;
    PD_CR1 = 0xff;

    __asm__ ("rim");
}
/* Available ports on STM8S103P3:
 *
 * A1..A3	A3 is HS
 * B4..B5	Open drain
 * C3..C7	HS
 * D1..D6	HS
 */

/******************************************************************************
 *
 *  Convert liters to percent of tank
 */

char conv_percent(int liters)
{
    unsigned int percent;

    percent = liters * 50;		/* 0 to 50000 */
    percent /= (TANK_LITERS / 10);	/* 0 to 500 */
    percent /= 5;			/* 0 to 100 */

    return percent;
}

/******************************************************************************
 *
 *  Convert liters to gallons
 *
 *  in: liters
 * out: gallons * 32 (low 5 bits for decimal point)
 */

int conv_gallons(int liters)
{
    unsigned long temp;

    temp  = (unsigned int)liters;
    temp *= 541;
    temp >>= 6;

    return temp;
}

/******************************************************************************
 *
 *  Get decimal point char from bottom 5 bits
 *
 *  in: 5 bits of fractional number
 * out: decimal character
 */

char conv_point(char frac)
{
    return ((frac * 10) >> 5) + '0';
}

/******************************************************************************
 *
 *  Ping calibration table
 *  Values are ping time in microseconds and liters.
 */
const int ping_tab[] = {
    5960,	/* empty */
    5380,	/* 100  interpolated */
    4800,	/* 200  */
    4240,	/* 300  */
    3734,	/* 400  */
    3200,	/* 500  */
    2684,	/* 600  */
    2150,	/* 700  */
    1612,	/* 800  */
    1068,	/* 900  */
    536,	/* 1000 */
    4,		/* 1110, calculated */
};
