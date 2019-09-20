/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_i2c.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Definition of peripheral ID */
#define FTM_0_PERIPHERAL FTM0
/* Definition of the clock source frequency */
#define FTM_0_CLOCK_SOURCE CLOCK_GetFreq(kCLOCK_BusClk)
/* FTM_0 interrupt vector ID (number). */
#define FTM_0_IRQN FTM0_IRQn
/* FTM_0 interrupt vector priority. */
#define FTM_0_IRQ_PRIORITY 0
/* FTM_0 interrupt handler identifier. */
#define FTM_0_IRQHANDLER FTM0_IRQHandler
/* Definition of peripheral ID */
#define FTM_DIST_PERIPHERAL FTM3
/* Definition of the clock source frequency */
#define FTM_DIST_CLOCK_SOURCE CLOCK_GetFreq(kCLOCK_BusClk)
/* FTM_DIST interrupt vector ID (number). */
#define FTM_DIST_IRQN FTM3_IRQn
/* FTM_DIST interrupt vector priority. */
#define FTM_DIST_IRQ_PRIORITY 0
/* FTM_DIST interrupt handler identifier. */
#define FTM_DIST_IRQHANDLER FTM3_IRQHandler

/* Definitions for BOARD_InitBUTTONSPeripheral functional group */
/* Alias for GPIOC peripheral */
#define BOARD_SW3_GPIO GPIOC
/* Alias for GPIOA peripheral */
#define BOARD_SW2_GPIO GPIOA

/* Definitions for BOARD_InitLEDsPeripheral functional group */
/* Alias for GPIOC peripheral */
#define BOARD_LEDS_GPIO GPIOC

/* Definitions for BOARD_InitACCELPeripheral functional group */
/* BOARD_InitACCELPeripheral defines for I2C3 */
/* Definition of peripheral ID */
#define BOARD_ACCEL_PERIPHERAL I2C3
/* Definition of the clock source */
#define BOARD_ACCEL_CLOCK_SOURCE I2C3_CLK_SRC
/* Definition of the clock source frequency */
#define BOARD_ACCEL_CLK_FREQ CLOCK_GetFreq(BOARD_ACCEL_CLOCK_SOURCE)
/* Alias for GPIOC peripheral */
#define BOARD_ACCEL_INT1_GPIO GPIOC

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const ftm_config_t FTM_0_config;
extern const ftm_config_t FTM_DIST_config;
extern const i2c_master_config_t BOARD_ACCEL_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void);
void BOARD_InitBUTTONSPeripheral(void);
void BOARD_InitLEDsPeripheral(void);
void BOARD_InitDEBUG_UARTPeripheral(void);
void BOARD_InitACCELPeripheral(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
