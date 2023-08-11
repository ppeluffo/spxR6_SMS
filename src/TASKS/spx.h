/*
 * spx.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_H_
#define SRC_SPX_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include "avr_compiler.h"
#include "clksys_driver.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "wdt_driver.h"
#include <ctype.h>
#include <frtos_cmd.h>
#include <inttypes.h>


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"
#include "portable.h"
#include "math.h"

#include "l_iopines.h"
#include "l_nvm.h"
#include "l_printf.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "1.0.1"
#define SPX_FW_DATE "@ 20230809"

#define SPX_HW_MODELO "spxR6 HW:xmega256A3B R1.1"
#define SPX_FTROS_VERSION "FW:FRTOS10 SMS"

//#define F_CPU (32000000UL)
//#define SYSMAINCLK 2
//#define SYSMAINCLK 8
#define SYSMAINCLK 32
//
#define CHAR32	32
#define CHAR64	64
#define CHAR128	128
#define CHAR256	256

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		384
#define tkGprs_STACK_SIZE		384

StaticTask_t xTask_Ctl_Buffer_Ptr;
StackType_t xTask_Ctl_Buffer [tkCtl_STACK_SIZE];

StaticTask_t xTask_Cmd_Buffer_Ptr;
StackType_t xTask_Cmd_Buffer [tkCmd_STACK_SIZE];

StaticTask_t xTask_Gprs_Buffer_Ptr;
StackType_t xTask_Gprs_Buffer [tkGprs_STACK_SIZE];

#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkGprs_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

TaskHandle_t xHandle_idle, xHandle_tkCtl, xHandle_tkCmd, xHandle_tkComms, xHandle_tkGprs;

void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkGprs(void * pvParameters);

void initMCU(void);
void u_configure_systemMainClock(void);
void u_configure_RTC32(void);

xSemaphoreHandle sem_RXBUFF;
StaticSemaphore_t RXBUFF_xMutexBuffer;
#define MSTOTAKERXBUFFSEMPH ((  TickType_t ) 10 )


void gprs_init_outofrtos(void);
void gprs_rxbuffer_reset(void);
void gprs_txbuffer_reset(void);

bool gprs_rxbuffer_full(void);
bool gprs_rxbuffer_empty(void);
uint16_t gprs_rxbuffer_usedspace(void);

void gprs_rxbuffer_put( char data);
bool gprs_rxbuffer_put2( char data );
void gprs_flush_RX_buffer(void);
void gprs_flush_TX_buffer(void);
void gprs_print_RX_buffer(void);
bool gprs_check_response( const uint16_t timeout, const char *rsp );

void gprs_init(void);
void gprs_prender( void );
void gprs_hw_pwr_on(uint8_t delay_factor);
void gprs_sw_pwr(void);
void gprs_apagar(void);

int8_t FSM_sendATcmd( const uint8_t timeout, char *cmd );


//------------------------------------------------------------------------


#endif /* SRC_SPX_H_ */
