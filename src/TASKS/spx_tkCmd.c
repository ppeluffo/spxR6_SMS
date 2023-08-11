/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */


#include "spx.h"
#include "drv_dma_spx.h"


//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdKillFunction(void);


#define TERM_BUFFER_LENGTH 128

struct {
	char buffer[TERM_BUFFER_LENGTH];
	uint16_t ptr;
} TERM_buffer;

void CMD_process(char c);
void CMD_buffer_flush(void);
void CMD_buffer_execute(void);

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c = 0;
uint8_t ticks = 0;

( void ) pvParameters;

	// Espero la notificacion para arrancar

	vTaskDelay( ( TickType_t)( 700 / portTICK_RATE_MS ) );
	xprintf_P( PSTR("starting tkCmd..\r\n") );


	FRTOS_CMD_init( xputChar, xprintf_cmd );

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls", cmdClearScreen );
	FRTOS_CMD_register( "reset", cmdResetFunction);
	FRTOS_CMD_register( "write", cmdWriteFunction);
	FRTOS_CMD_register( "read", cmdReadFunction);
	FRTOS_CMD_register( "help", cmdHelpFunction );
	FRTOS_CMD_register( "status", cmdStatusFunction );
	FRTOS_CMD_register( "config", cmdConfigFunction );
	FRTOS_CMD_register( "kill", cmdKillFunction );

	// Fijo el timeout del READ
	ticks = 5;
	frtos_ioctl( fdTERM,ioctl_SET_TIMEOUT, &ticks );

	CMD_buffer_flush();
	// loop
	for( ;; )
	{

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		//while ( CMD_read( (char *)&c, 1 ) == 1 ) {
		while ( frtos_read( fdTERM, (char *)&c, 1 ) == 1 ) {
			//FRTOS_CMD_process(c);
			CMD_process(c);
		}

	}
}
//------------------------------------------------------------------------------------
void CMD_process(char c)
{
	// Bufferea los caracteres.
	// Cuando recibe un \r los analiza.
	// Si son locales, los ejecuta.
	// En otro caso los pasa al modem

	if (c == '\r') {
		// Analizo y proceso
		xputChar('\r');
		xputChar('\n');
		CMD_buffer_execute();
		CMD_buffer_flush();

	} else {
		// Almaceno
		// Si el caracter es imprimible lo almaceno
		if( (c >= 0x20) && (c < 0x7F) ) {
			// Echo local
			xputChar(c);
			// Almaceno
			if( TERM_buffer.ptr < TERM_BUFFER_LENGTH ) {
				TERM_buffer.buffer[ TERM_buffer.ptr ] = c;
				TERM_buffer.ptr++;
			}
		}
	}
}
//------------------------------------------------------------------------------------
void CMD_buffer_execute(void)
{
	//xnprint( TERM_buffer.buffer, TERM_BUFFER_LENGTH );
	//xprintf_P( PSTR ("\r\n[%d]\r\n"), TERM_buffer.ptr );

	// Comandos LOCALES: Los ejecuta el micro.
	if ( strcmp_P( TERM_buffer.buffer, PSTR("AT*PWRON")) == 0 ) {
		gprs_prender();
		return;
	}

	if ( strcmp_P( TERM_buffer.buffer, PSTR("AT*PWROFF")) == 0 ) {
		gprs_apagar();
		return;
	}

	// Comandos del MODEM: Se pasan directamente al modem
	gprs_flush_RX_buffer();
	xfprintf_P( fdGPRS, PSTR("%s\r"),TERM_buffer.buffer );

}
//------------------------------------------------------------------------------------
void CMD_buffer_flush(void)
{
	memset( TERM_buffer.buffer, '\0', TERM_BUFFER_LENGTH);
	TERM_buffer.ptr = 0;

}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

	FRTOS_CMD_makeArgv();

	xprintf_P( PSTR("\r\nSpymovil %s %s %s %s \r\n"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
	xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n"),SYSMAINCLK, configTICK_RATE_HZ );


}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	cmdClearScreen();

//	while(1)
//		;

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// GPRS
	// write gprs pwr|sw|rts {on|off}
	// write gprs cmd {atcmd}
	// write gprs sms nbr msg
	// write gprs qsms nbr msg
	if ( ( strcmp_P( strupr(argv[1]), PSTR("GPRS")) == 0) ) {

		// write gprs prender,apagar
		if ( strcmp_P( strupr(argv[2]), PSTR("PRENDER")) == 0 ) {
			gprs_prender();
			pv_snprintfP_OK();
			return;
		}

		if ( strcmp_P( strupr(argv[2]), PSTR("APAGAR")) == 0 ) {
			gprs_apagar();
			pv_snprintfP_OK();
			return;
		}

		// write gprs (pwr|sw|rts|dtr) {on|off}
		if ( strcmp_P( strupr(argv[2]), PSTR("PWR")) == 0 ) {
			if ( strcmp_P( strupr(argv[3]), PSTR("ON")) == 0 ) {
				IO_set_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			if ( strcmp_P( strupr(argv[3]), PSTR("OFF")) == 0 ) {
				IO_clr_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if ( strcmp_P( strupr(argv[2]), PSTR("SW"))  == 0 ) {
			if ( strcmp_P( strupr(argv[3]), PSTR("ON\0")) == 0 ) {
				IO_set_GPRS_SW();
				pv_snprintfP_OK(); return;
			}
			if ( strcmp_P( strupr(argv[3]), PSTR("OFF")) == 0 ) {
				IO_clr_GPRS_SW(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		return;
	}

	// ATCMD
	// write gprs cmd {atcmd}
	if ( strcmp_P(strupr(argv[1]), PSTR("ATCMD")) == 0 ) {
		//xprintf_P( PSTR("%s\r\0"),argv[3] );

		gprs_flush_RX_buffer();
		if (strcmp_P( argv[2], PSTR("+++")) == 0 ) {
			xfprintf_P( fdGPRS, PSTR("%s"),argv[2] );
		} else {
			xfprintf_P( fdGPRS, PSTR("%s\r"),argv[2] );
		}

		xprintf_P( PSTR("sent->%s\r\n\0"),argv[2] );
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{


	FRTOS_CMD_makeArgv();

	// read atcmd
	if (!strcmp_P( strupr(argv[1]), PSTR("ATRSP"))  ) {
		gprs_print_RX_buffer();
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	xprintf_P( PSTR("\x1B[2J\0"));
}
//------------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{


	FRTOS_CMD_makeArgv();

	return;
}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]), PSTR("WRITE"))) {
		xprintf_P( PSTR("-write\r\n"));
		xprintf_P( PSTR("  gprs pwr|sw {on|off}\r\n"));
		xprintf_P( PSTR("  gprs prender,apagar\r\n"));
		xprintf_P( PSTR("  atcmd\r\n"));
		return;
	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ"))) {
		xprintf_P( PSTR("-read\r\n"));
		xprintf_P( PSTR("  atrsp\r\n"));
		return;

	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		xprintf_P( PSTR("-reset\r\n\0"));
		return;

	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		xprintf_P( PSTR("-config\r\n\0"));
		xprintf_P( PSTR("  save\r\n\0"));
	}

	// HELP KILL
	else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0")) ) {
		xprintf_P( PSTR("-kill {data, app, commstx }\r\n\0"));
		return;

	} else {

		// HELP GENERAL
		xprintf_P( PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		xprintf_P( PSTR("Available commands are:\r\n\0"));
		xprintf_P( PSTR("-cls\r\n\0"));
		xprintf_P( PSTR("-help\r\n\0"));
		xprintf_P( PSTR("-status\r\n\0"));
		xprintf_P( PSTR("-reset...\r\n\0"));
		xprintf_P( PSTR("-kill...\r\n\0"));
		xprintf_P( PSTR("-write...\r\n\0"));
		xprintf_P( PSTR("-read...\r\n\0"));
		xprintf_P( PSTR("-config...\r\n\0"));

	}

	xprintf_P( PSTR("\r\n\0"));

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{

	FRTOS_CMD_makeArgv();

	pv_snprintfP_ERR();
	return;
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf_P( PSTR("ok\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf_P( PSTR("error\r\n\0"));
}
//------------------------------------------------------------------------------------
