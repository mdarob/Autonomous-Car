/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define DEBUG 1
#define LOG_SD 1
 
/**
 * @file    ftm.c
 * @brief   Application entry point.
 *
 * HC-SR04 Ultrasonic distance sensor:
 * DIST TRIG: K82F-J2-8 (PORTD pin 2 as PTD2)
 * DIST ECHO: K82F-J1-8 (PORTD pin 0 as FTM3Chnl0 dual-edge input capture, combined with PORTD pin 1 as FTM3Chnl1)
 * Note: ECHO must be level shifted to 3.3V from 5V
 *
 * Adafruit Micro-SD Breakout board:
 * Middleware FATfs, from Memories->SDMMC include sdio and sdspi
 * ffconf.h -> define MSDK adaption configuration as SDSPI_DISK_ENABLE
 * ffconf.h -> define Locale and Namespace configuration as 437 (U.S.)
 * ff.c -> line 3268, change "fs->pdrv = ..." to 4
 * board.h -> define additional line under ERPC DSPI configuration: "#define BOARD_SDSPI_SPI_BASE SPI0_BASE"
 * Delete files: fsl_mmc_disk.c and fsl_ram_disk.c
 * SD CARD CLK: K82F-J1-15 (PORTA pin 15 as SPI0_SCK)
 * SD CARD DO:  K82F-J1-7  (PORTA pin 17 as SPI0_SIN)
 * SD CARD DI:  K82F-J1-13 (PORTA pin 16 as SPI0_SOUT)
 * SD CARD CS:  K82F-J1-9  (PORTA pin 14 as SPI0_PCS0)
 *
 * CAR SPEED PWM: K82F-J4-8 (PORTC pin 2 as FTM0Chnl1)
 * CAR STEER PWM: K82F-J4-6 (PORTC pin 1 as FTM0Chnl0)
 * CAM PAN PWM:   K82F-J2-2 (PORTC pin 3 as FTM0Chnl2)
 * CAM TILT PWM:  K82F-J2-4 (PORTA pin 1 as FTM0Chnl6)
 *
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK82F25615.h"
#include "fsl_debug_console.h"
#include "ff.h"

#define MODE_NONE      0
#define MODE_BASIC     1
#define MODE_SPEED     2
#define MODE_ACCURACY  3
#define MODE_IOT       4
#define NUM_MODES      5

/* FTM channel pair used for the dual-edge capture, channel pair 0 uses channels 0 and 1 */
#define FTM_DIST_CHANNEL_PAIR kFTM_Chnl_0

/* FTM_DIST channel used for first edge of dual-edge input capture */
#define FTM_DIST_LEAD_EDGE_FLAG kFTM_Chnl0Flag

/* FTM_DIST channel used for second edge of dual-edge input capture */
#define FTM_DIST_TRAIL_EDGE_FLAG kFTM_Chnl1Flag

/* FTM_DIST output compare channel used to assert DIST TRIG pulse (pulse is cleared by Timer Overflow) */
#define FTM_DIST_OC_TRIG_FLAG kFTM_Chnl2Flag

/* DIST TRIG pulse GPIO peripheral and pin. Don't forget to configure pin_mux */
#define DIST_TRIG_GPIO GPIOD
#define DIST_TRIG_GPIO_PIN 2U

/* DIST sensor state enumeration */
#define DIST_MAKE_TRIG 0
#define DIST_WAIT_ECHO 1
#define DIST_PROCESS   2

/* divisor of (134 ticks/inch) is obtained by: (143 us/inch) / (1.066 us/tick) */
#define DIST_UNIT_CONVERSION 134

/* Maximum distance value */
#define DIST_MAX 150

#define DIST_MAX_OVERFLOW 4

/* Maximum tolerable change in distance value between samples */
#define DIST_DELTA 20

#define PWM_MOD_5_PERCENT    1875
#define PWM_MOD_6_PERCENT    2250
#define PWM_MOD_7_5_PERCENT  2812
#define PWM_MOD_10_PERCENT   3750
#define PWM_MOD_13_5_PERCENT 5063
#define PWM_MOD_15_PERCENT   5625

#define PWM_OVERFLOW_COUNT 2
uint32_t PWM_overflow_number = 0;

#define WHEEL_ANGLE_PWM_CHANNEL kFTM_Chnl_0
#define WHEEL_SPEED_PWM_CHANNEL kFTM_Chnl_1
#define PAN_ANGLE_PWM_CHANNEL kFTM_Chnl_2
#define TILT_ANGLE_PWM_CHANNEL kFTM_Chnl_6

#define WHEEL_ANGLE_MAX 300
#define WHEEL_SPEED_MAX 1000
#define PAN_ANGLE_MAX 900
#define TILT_MOD_VALUE 8000

volatile int wheelAngle = 0;
volatile int wheelSpeed = 0;
volatile int panAngle = 0;

/* DIST sensor state variable */
unsigned int dist_capture_state = DIST_MAKE_TRIG;
/* DIST sensor overflow counter */
unsigned int dist_overflow_number = 0;
/* DIST sensor output, in inches */
volatile unsigned int dist_val = 0;

#if LOG_SD

	#define LOGGER_OVERFLOW_COUNT 25
	uint32_t LOGGER_overflow_number = 0;

	#define LOGGER_BUFFER_SIZE 64
	#define LOGGER_DIST_OFFSET 5
	char logger_status[LOGGER_BUFFER_SIZE] = "dist:XXX\r\n";

	const char * filname = "logdata.txt";
    FRESULT fr;
    FATFS fs;
    FIL fil;
    int fschars;
    bool sd_ok = false;

    bool sd_open(void) {
        fr = f_open(&fil, filname, FA_WRITE | FA_OPEN_APPEND);
		if (fr != FR_OK) {
#if DEBUG
			PRINTF("FAILED write to SD card (opening) error %d\r\n", fr);
#endif
			return false;
		}
		return true;
    }

    bool sd_write(const char * msg) {
        fschars = f_printf(&fil, msg);
        if (fschars == -1) {
#if DEBUG
        	PRINTF("FAILED write to SD card: possibly disk is full\r\n");
#endif
        	return false;
        }
        return true;
    }

    bool sd_close(void) {
        fr = f_close(&fil);
		if (fr != FR_OK) {
#if DEBUG
			PRINTF("FAILED write to SD card (closing) error %d\r\n", fr);
#endif
			return false;
		}
		return true;
    }

    // mount, open, write (initial message), and close
    void sd_init(void) {
        f_mount(&fs, filname, 0);
#if DEBUG
        PRINTF("SD card mounted\r\n");
#endif

        sd_ok = sd_open();
        if (!sd_ok) {
        	return;
        }
#if DEBUG
        else {
			PRINTF("SD card initial open\r\n");
		}
#endif

		sd_ok = sd_write("Mongoose Motorsports: started\r\n");
		if (!sd_ok) {
			return;
		}
#if DEBUG
		else {
			PRINTF("SD card initial write\r\n");
		}
#endif

		sd_ok = sd_close();
		if (!sd_ok) {
			return;
		}
#if DEBUG
		else {
			PRINTF("SD card initial close\r\n");
		}
#endif
    }

#endif

int32_t dist_capture1Val;
int32_t dist_capture2Val;
int32_t dist_raw_1 = 0u, dist_raw_2 = 0u, dist_raw_3 = 0u;

void UPDATE_DISTANCE(void) {
	dist_capture1Val = FTM_DIST_PERIPHERAL->CONTROLS[FTM_DIST_CHANNEL_PAIR * 2].CnV;
	dist_capture2Val = FTM_DIST_PERIPHERAL->CONTROLS[(FTM_DIST_CHANNEL_PAIR * 2) + 1].CnV;

	/* Distance calculation */
	dist_raw_3 = dist_raw_2;
	dist_raw_2 = dist_raw_1;
	dist_raw_1 = ((dist_capture2Val - dist_capture1Val)
			+ dist_overflow_number*(FTM_DIST_PERIPHERAL->MOD + 1)) / DIST_UNIT_CONVERSION;
	if (dist_raw_1 > DIST_MAX) {
		dist_raw_1 = DIST_MAX;
	}

	// if raw2 falls outside raw3 +/- delta
	if (dist_raw_2 > dist_raw_3 + DIST_DELTA || dist_raw_2 < dist_raw_3 - DIST_DELTA) {
		// skip raw2
		dist_val = dist_raw_1;
	} else {  // case where raw2 falls within raw3 +/- delta
		dist_val = dist_raw_2;
	}

	// Sample output:
	//  raw3  raw2  raw1| output
	// --------------------------
	//   26    19    18 |  19
	//   17    18    19 |  18
	//   18    19    28 |  19
	//   19    28   150 |  28
	//   28   150     8 |   8
	//  150     8     4 |   4

#if DEBUG
	PRINTF("D:%ir\t%ic\t%i\r\n", dist_raw_1, dist_val, dist_val - dist_raw_2);
#endif

	// next state
	dist_capture_state = DIST_MAKE_TRIG;
	dist_overflow_number = 0;
}

//bool wheelAngleRight = true;
//bool wheelSpeedUp = true;
//bool panAngleRight = true;

void FTM_0_IRQHANDLER(void) {
	if((FTM_GetStatusFlags(FTM_0_PERIPHERAL) & kFTM_TimeOverflowFlag) == kFTM_TimeOverflowFlag) {

		if (++LOGGER_overflow_number >= LOGGER_OVERFLOW_COUNT) {
			LOGGER_overflow_number = 0;

			logger_status[LOGGER_DIST_OFFSET] = dist_val / 100 + 0x30;
			logger_status[LOGGER_DIST_OFFSET+1] = (dist_val / 10) % 10 + 0x30;
			logger_status[LOGGER_DIST_OFFSET+2] = dist_val % 10 + 0x30;

#if DEBUG
			PRINTF(logger_status);
#endif
#if LOG_SD
			if (sd_ok) {
				sd_ok = sd_open();
				if (sd_ok) {
					sd_write(logger_status);
					sd_close();
				}
			}
#endif
		}

		// Update PWM
		if (++PWM_overflow_number >= PWM_OVERFLOW_COUNT) {
			PWM_overflow_number = 0;

			// clamp values
			if (wheelAngle > WHEEL_ANGLE_MAX) {
				wheelAngle = WHEEL_ANGLE_MAX;
			} else if (wheelAngle < -WHEEL_ANGLE_MAX) {
				wheelAngle = -WHEEL_ANGLE_MAX;
			}
			if (wheelSpeed > WHEEL_SPEED_MAX) {
				wheelSpeed = WHEEL_SPEED_MAX;
			} else if (wheelSpeed < -WHEEL_SPEED_MAX) {
				wheelSpeed = -WHEEL_SPEED_MAX;
			}
			if (panAngle > PAN_ANGLE_MAX) {
				panAngle = PAN_ANGLE_MAX;
			} else if (panAngle < -PAN_ANGLE_MAX) {
				panAngle = -PAN_ANGLE_MAX;
			}

			// write values
			FTM_0_PERIPHERAL->CONTROLS[WHEEL_ANGLE_PWM_CHANNEL].CnV = PWM_MOD_10_PERCENT + PWM_MOD_5_PERCENT*(wheelAngle+WHEEL_ANGLE_MAX)/WHEEL_ANGLE_MAX;
			FTM_0_PERIPHERAL->CONTROLS[WHEEL_SPEED_PWM_CHANNEL].CnV = PWM_MOD_10_PERCENT + PWM_MOD_5_PERCENT*(wheelSpeed+WHEEL_SPEED_MAX)/WHEEL_SPEED_MAX;
			FTM_0_PERIPHERAL->CONTROLS[PAN_ANGLE_PWM_CHANNEL].CnV = PWM_MOD_6_PERCENT + PWM_MOD_7_5_PERCENT*(panAngle+PAN_ANGLE_MAX)/PAN_ANGLE_MAX;

			// trigger/queue update
			FTM_SetSoftwareTrigger(FTM_0_PERIPHERAL, true);

			// debug only: scan through range of values
//			if (wheelAngleRight) {
//				if (++wheelAngle >= WHEEL_ANGLE_MAX) {
//					wheelAngleRight = false;
//				}
//			} else {
//				if (--wheelAngle <= -WHEEL_ANGLE_MAX) {
//					wheelAngleRight = true;
//				}
//			}
//			if (wheelSpeedUp) {
//				if (++wheelSpeed >= WHEEL_SPEED_MAX) {
//					wheelSpeedUp = false;
//				}
//			} else {
//				if (--wheelSpeed <= -WHEEL_SPEED_MAX) {
//					wheelSpeedUp = true;
//				}
//			}
//			if (panAngleRight) {
//				if (++panAngle >= PAN_ANGLE_MAX) {
//					panAngleRight = false;
//				}
//			} else {
//				if (--panAngle <= -PAN_ANGLE_MAX) {
//					panAngleRight = true;
//				}
//			}

		}

		// clear timer overflow flag
		FTM_ClearStatusFlags(FTM_0_PERIPHERAL, kFTM_TimeOverflowFlag);
	}
}

void FTM_DIST_IRQHANDLER(void) {


	/* Output compare used to assert TRIG pulse (pulse is cleared by Timer Overflow) */
	if((FTM_GetStatusFlags(FTM_DIST_PERIPHERAL) & FTM_DIST_OC_TRIG_FLAG) == FTM_DIST_OC_TRIG_FLAG) {
		FTM_ClearStatusFlags(FTM_DIST_PERIPHERAL, FTM_DIST_OC_TRIG_FLAG);
		if (dist_capture_state == DIST_MAKE_TRIG) {
			GPIO_PortSet(DIST_TRIG_GPIO, 1u << DIST_TRIG_GPIO_PIN);
		}
	}

	/* Timer Overflow detected, increment overflow counter */
	if((FTM_GetStatusFlags(FTM_DIST_PERIPHERAL) & kFTM_TimeOverflowFlag) == kFTM_TimeOverflowFlag) {
		FTM_ClearStatusFlags(FTM_DIST_PERIPHERAL, kFTM_TimeOverflowFlag);
		if (dist_capture_state == DIST_MAKE_TRIG) {
			GPIO_PortClear(DIST_TRIG_GPIO, 1u << DIST_TRIG_GPIO_PIN);
			dist_capture_state = DIST_WAIT_ECHO;
		} else if (dist_capture_state == DIST_WAIT_ECHO) {
			if (dist_overflow_number < DIST_MAX_OVERFLOW) {
				dist_overflow_number++;
			} else {
				dist_capture_state = DIST_PROCESS;
			}
		}
	}

	/* Falling edge of distance pulse detected */
	if((FTM_GetStatusFlags(FTM_DIST_PERIPHERAL) & FTM_DIST_TRAIL_EDGE_FLAG) == FTM_DIST_TRAIL_EDGE_FLAG) {
		FTM_ClearStatusFlags(FTM_DIST_PERIPHERAL, FTM_DIST_TRAIL_EDGE_FLAG);
		if (dist_capture_state == DIST_WAIT_ECHO) {
			dist_capture_state = DIST_PROCESS;
		}
	}
}

void BOARD_SW2_IRQ_HANDLER(void)
{
	GPIO_PortClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);
	mode++;
	mode %= NUM_MODES;

	if (mode == 0 && mode_set == 0){
    	LED_RED_OFF();
    	LED_BLUE_OFF();
    	LED_GREEN_OFF();
#if DEBUG
    	PRINTF("mode 0 \"none\"\r\n");
#endif
	}
	if (mode == 1 && mode_set == 0){
    	LED_RED_ON();
    	LED_BLUE_OFF();
    	LED_BLUE_OFF();
#if DEBUG
    	PRINTF("mode 1 \"basic\"\r\n");
#endif
	}
	if (mode == 2 && mode_set == 0){
    	LED_RED_OFF();
    	LED_BLUE_ON();
    	LED_GREEN_OFF();
#if DEBUG
    	PRINTF("mode 2 \"speed\"\r\n");
#endif
	}
	if (mode == 3 && mode_set == 0){
    	LED_RED_OFF();
    	LED_BLUE_OFF();
    	LED_GREEN_ON();
#if DEBUG
    	PRINTF("mode 3 \"accuracy\"\r\n");
#endif
	}
	if (mode == 4 && mode_set == 0){
    	LED_RED_ON();
    	LED_BLUE_ON();
    	LED_GREEN_OFF();
#if DEBUG
    	PRINTF("mode 4 \"IoT\"\r\n");
#endif
	}
}

void BOARD_SW3_IRQ_HANDLER(void)
{
	GPIO_PortClearInterruptFlags(BOARD_SW3_GPIO, 1U << BOARD_SW3_GPIO_PIN);
	if (mode_set == 1) mode_set = 0;
	else mode_set = 1;

	if (mode_set == 1){
		LED_RED_ON();
		LED_BLUE_ON();
		LED_GREEN_ON();
#if DEBUG
		PRINTF("mode %d selected\r\n", mode);
#endif
	}
	else {
		mode = 0;
		LED_RED_OFF();
		LED_BLUE_OFF();
		LED_GREEN_OFF();
#if DEBUG
		PRINTF("mode reset\r\n");
#endif
	}
}

int main(void) {

    /* Define the init structure for the DIST sensor TRIG pin */
    gpio_pin_config_t dist_trig_config = {
        kGPIO_DigitalOutput, 0,
    };

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#if DEBUG
    BOARD_InitDebugConsole();
    PRINTF("Mongoose Motorsports: debug console\r\n");
#endif

#if LOG_SD
    sd_init();
#else
    PRINTF("SD card logging disabled\r\n");
#endif

    GPIO_PinInit(DIST_TRIG_GPIO, DIST_TRIG_GPIO_PIN, &dist_trig_config);

    // stabilize servos
    FTM_0_PERIPHERAL->CONTROLS[WHEEL_ANGLE_PWM_CHANNEL].CnV = PWM_MOD_15_PERCENT;
	FTM_0_PERIPHERAL->CONTROLS[WHEEL_SPEED_PWM_CHANNEL].CnV = PWM_MOD_15_PERCENT;
	FTM_0_PERIPHERAL->CONTROLS[PAN_ANGLE_PWM_CHANNEL].CnV = PWM_MOD_13_5_PERCENT;
    FTM_0_PERIPHERAL->CONTROLS[TILT_ANGLE_PWM_CHANNEL].CnV = TILT_MOD_VALUE;
    FTM_StartTimer(FTM_0_PERIPHERAL, kFTM_SystemClock);

    // mode controller
    gpio_pin_config_t sw_config = { kGPIO_DigitalInput, 0, };
	PORT_SetPinInterruptConfig(BOARD_SW2_PORT, BOARD_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);
	EnableIRQ(BOARD_SW2_IRQ);
	GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);

	PORT_SetPinInterruptConfig(BOARD_SW3_PORT, BOARD_SW3_GPIO_PIN, kPORT_InterruptFallingEdge);
	EnableIRQ(BOARD_SW3_IRQ);
	GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);

	LED_RED_INIT(LOGIC_LED_OFF);
	LED_BLUE_INIT(LOGIC_LED_OFF);
	LED_GREEN_INIT(LOGIC_LED_OFF);

    // start dist timer
    FTM_StartTimer(FTM_DIST_PERIPHERAL, kFTM_SystemClock);

    while(1) {

    	// Calculate distance if capture complete
    	if (dist_capture_state == DIST_PROCESS) {
    		UPDATE_DISTANCE();
    	}

    }
    return 0 ;
}
