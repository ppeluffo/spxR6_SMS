/*
 * spx_tkGprs.c
 *
 *  Created on: 9 ago. 2023
 *      Author: pablo
 */


#include "spx.h"

typedef enum { ATRSP_NONE, ATRSP_OK, ATRSP_ERROR, ATRSP_TIMEOUT, ATRSP_OUT_INMEDIATE, ATRSP_UNKNOWN } t_at_commands_responses;


#define GPRS_TXBUFFER_LEN	384
struct {
	char buffer[GPRS_TXBUFFER_LEN];
	uint16_t ptr;
} gprs_txbuffer;

#define GPRS_RXBUFFER_LEN	576
struct {
	char buffer[GPRS_RXBUFFER_LEN];
	uint16_t ptr;
} gprs_rxbuffer;


//------------------------------------------------------------------------------------
void tkGprs(void * pvParameters)
{
	// Esta tarea lee y procesa las respuestas del GPRS. Lee c/caracter recibido y lo va
	// metiendo en un buffer circular propio del GPRS que permite luego su impresion,
	// analisis, etc.

( void ) pvParameters;
char c;
uint8_t ticks = 0;

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	xprintf_P( PSTR("starting tkGprs..\r\n\0"));

	// Fijo el timeout del READ
	ticks = 5;
	frtos_ioctl( fdGPRS,ioctl_SET_TIMEOUT, &ticks );

	gprs_init();

	for( ;; )	{

		// Leo el UART de GPRS
		while ( frtos_read( fdGPRS, &c, 1 ) == 1 ) {
			//gprs_rxbuffer_put2(c);
			xputChar(c);
		}

		vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );
	}

}
//------------------------------------------------------------------------------------
void gprs_init_outofrtos(void)
{
	sem_RXBUFF = xSemaphoreCreateMutexStatic( &RXBUFF_xMutexBuffer );
}
//------------------------------------------------------------------------------------
void gprs_rxbuffer_reset(void)
{
	// Vacia el buffer y lo inicializa

	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		//taskYIELD();
		vTaskDelay( ( TickType_t)( 1 ) );

		memset( gprs_rxbuffer.buffer, '\0', GPRS_RXBUFFER_LEN);
		gprs_rxbuffer.ptr = 0;

	xSemaphoreGive( sem_RXBUFF );

}
//------------------------------------------------------------------------------------
void gprs_txbuffer_reset(void)
{
	// Vacia el buffer de trasmision y lo inicializa
	// No necesita semaforo

	memset( gprs_txbuffer.buffer, '\0', GPRS_TXBUFFER_LEN);
	gprs_txbuffer.ptr = 0;


}
//------------------------------------------------------------------------------------
bool gprs_rxbuffer_full(void)
{
	return ( gprs_rxbuffer.ptr == GPRS_RXBUFFER_LEN );
}
//------------------------------------------------------------------------------------
bool gprs_rxbuffer_empty(void)
{
	return (!gprs_rxbuffer_full());
}
//------------------------------------------------------------------------------------
uint16_t gprs_rxbuffer_usedspace(void)
{
uint16_t freespace;

	freespace = GPRS_RXBUFFER_LEN - gprs_rxbuffer.ptr;
	return (freespace);
}
//------------------------------------------------------------------------------------
void gprs_rxbuffer_put( char data)
{
	// Avanza sobreescribiendo el ultimo si esta lleno

	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		//taskYIELD();
		vTaskDelay( ( TickType_t)( 1 ) );

		gprs_rxbuffer.buffer[ gprs_rxbuffer.ptr ] = data;
		if ( gprs_rxbuffer.ptr < GPRS_RXBUFFER_LEN )
			gprs_rxbuffer.ptr++;

	xSemaphoreGive( sem_RXBUFF );
}
//------------------------------------------------------------------------------------
bool gprs_rxbuffer_put2( char data )
{
	// Solo inserta si hay lugar

bool retS = false;

	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		//taskYIELD();
		vTaskDelay( ( TickType_t)( 1 ) );

		if ( gprs_rxbuffer.ptr < GPRS_RXBUFFER_LEN ) {
			gprs_rxbuffer.buffer[ gprs_rxbuffer.ptr ] = data;
			gprs_rxbuffer.ptr++;
			retS = true;
		}

	xSemaphoreGive( sem_RXBUFF );

    return(retS);
}
//------------------------------------------------------------------------------------
void gprs_flush_RX_buffer(void)
{
	// El rx buffer de la uart es circular por lo tanto no los flusheo.
	frtos_ioctl( fdGPRS,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	gprs_rxbuffer_reset();
}
//------------------------------------------------------------------------------------
void gprs_flush_TX_buffer(void)
{
	frtos_ioctl( fdGPRS,ioctl_UART_CLEAR_TX_BUFFER, NULL);
}
//------------------------------------------------------------------------------------
void gprs_print_RX_buffer( void )
{
	// NO USO SEMAFORO PARA IMPRIMIR !!!!!

	xprintf_P( PSTR ("\r\nGPRS: rxbuff>\r\n"));

	// Imprimo todo el buffer local de RX. Sale por \0.
	// Uso esta funcion para imprimir un buffer largo, mayor al que utiliza xprintf_P. !!!
	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		//taskYIELD();
		vTaskDelay( ( TickType_t)( 1 ) );

	xnprint( gprs_rxbuffer.buffer, GPRS_RXBUFFER_LEN );
	xprintf_P( PSTR ("\r\n[%d]\r\n"), gprs_rxbuffer.ptr );

	xSemaphoreGive( sem_RXBUFF );
}
//------------------------------------------------------------------------------------
bool gprs_check_response( const uint16_t timeout, const char *rsp )
{
	// Espera una respuesta.
	// El timeout esta en intervalos de 100 milisegundos

bool retS = false;
int16_t local_timer = timeout;

	for(;;) {

		local_timer--;

		// Busco
		while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
			//taskYIELD();
			vTaskDelay( ( TickType_t)( 1 ) );

		if  ( strstr(gprs_rxbuffer.buffer, rsp) != NULL ) {
			retS = true;
		}
		xSemaphoreGive( sem_RXBUFF );

		// Respuesta correcta. Salgo enseguida
		if (retS) {
			return(retS);
		}

		// One time: salgo
		if (local_timer <= 0 ) {
			return(retS);
		}

		// Espero
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );


	}

	return(retS);
}
//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL
//------------------------------------------------------------------------------------
void gprs_init(void)
{
	// GPRS
	IO_config_GPRS_SW();
	IO_config_GPRS_PWR();
	IO_config_GPRS_RTS();
	IO_config_GPRS_CTS();
	IO_config_GPRS_DCD();
	IO_config_GPRS_RI();
	IO_config_GPRS_RX();
	IO_config_GPRS_TX();
	IO_config_GPRS_DTR();

	IO_set_GPRS_DTR();
	IO_set_GPRS_RTS();

}
//------------------------------------------------------------------------------------
void gprs_hw_pwr_on(uint8_t delay_factor)
{
	/*
	 * Prendo la fuente del modem y espero que se estabilize la fuente.
	 */

	//xprintf_P( PSTR("gprs: hw_pwr_on\r\n") );

	IO_clr_GPRS_SW();	// GPRS=0V, PWR_ON pullup 1.8V )
	IO_set_GPRS_PWR();	// Prendo la fuente ( alimento al modem ) HW
	vTaskDelay( (portTickType)( ( 2000 + 2000 * delay_factor) / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void gprs_prender(void)
{
	/*
	 * Apaga el dispositivo quitando la energia del mismo
	 *
	 */

	//xprintf_P( PSTR("gprs: prender\r\n") );

	gprs_hw_pwr_on(1);
	gprs_sw_pwr();
	xprintf_P( PSTR("OK\r\n") );

}
//------------------------------------------------------------------------------------
void gprs_apagar(void)
{
	/*
	 * Apaga el dispositivo quitando la energia del mismo
	 *
	 */

	//xprintf_P( PSTR("gprs: apagar\r\n") );

	IO_clr_GPRS_SW();	// Es un FET que lo dejo cortado
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	IO_clr_GPRS_PWR();	// Apago la fuente.
	//vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	xprintf_P( PSTR("OK\r\n") );

}
//------------------------------------------------------------------------------------
void gprs_sw_pwr(void)
{
	/*
	 * Genera un pulso en la linea PWR_SW. Como tiene un FET la senal se invierte.
	 * En reposo debe la linea estar en 0 para que el fet flote y por un pull-up del modem
	 * la entrada PWR_SW esta en 1.
	 * El PWR_ON se pulsa a 0 saturando el fet.
	 */
	//xprintf_P( PSTR("gprs: sw_pwr\r\n") );
	IO_set_GPRS_SW();	// GPRS_SW = 3V, PWR_ON = 0V.
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	IO_clr_GPRS_SW();	// GPRS_SW = 0V, PWR_ON = pullup, 1.8V

}
//------------------------------------------------------------------------------------
int8_t FSM_sendATcmd( const uint8_t timeout, char *cmd )
{
	// Envia un comando:
	// Sale por: OK/ERROR/BufferFull/TIMEOUT

	// Las respuestas pueden ser nula ( no espero ) u otra.

	// ATRSP_NONE, ATRSP_OK, ATRSP_ERROR, ATRSP_TIMEOUT, ATRSP_OUT_INMEDIATE, ATRSP_UNKNOWN

	// El comando lo doy una sola vez !!!.
	// Si me da error u respuesta no esperada, no tiene sentido repetir el comando.
	// El timeout es en segundos pero chequeo c/50 ms.

int8_t exit_code = -1;
int16_t ticks = (20 * timeout);			// Cuantos ticks de 50ms corresponden al timeout.
//int16_t ticks = ( 1000 * timeout / 50 );	// Cuantos ticks de 50ms corresponden al timeout.

	//xprintf_P(PSTR("DEBUG: timeout=%d\r\n"), timeout);
	//xprintf_P(PSTR("DEBUG: cmd=%s\r\n"), cmd );
	gprs_flush_RX_buffer();

	// Doy el comando.
	//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: (%d) %s\r\n"), local_timer, cmd );
	gprs_flush_TX_buffer();
	// Espera antes de c/comando. ( ver recomendaciones de TELIT )
	vTaskDelay( (portTickType)( 50 / portTICK_RATE_MS ) );
	xfprintf_P( fdGPRS , PSTR("%s"), cmd);

	// Respuesta inmediata: No importa el resultado.
	if ( timeout == 0 ) {
		// Salida inmediata:
		xprintf_P( PSTR("COMMS: FSM_sendATcmd: OUT INMEDIATE.\r\n"));
		exit_code = ATRSP_OUT_INMEDIATE;
		return(exit_code);
	}

	// Espero respuesta
	while ( exit_code == -1 ) {

		// Espero 1 tick: Cada 50 ms chequeo el buffer de salida del modem por una respuesta.
		vTaskDelay( (portTickType)( 50 / portTICK_RATE_MS ) );

		// Chequeo respuestas:
		while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
			//taskYIELD();
			vTaskDelay( ( TickType_t)( 1 ) );

			if  ( strstr(gprs_rxbuffer.buffer, "OK") != NULL ) {
				// Respuesta esperada: salgo
				exit_code = ATRSP_OK;

			} else if ( strstr(gprs_rxbuffer.buffer, "ERROR") != NULL ) {
				// Respuesta ERR: El comando respondio con error: salgo.
				exit_code = ATRSP_ERROR;

			} else if ( gprs_rxbuffer_full() ) {
				// Si el gprs buffer esta full es que recibio algo pero quedo trunco. ( CGDCONT )
				// Asumo que recibio una respuesta OK. !!!
				exit_code = ATRSP_OK;
			}

		xSemaphoreGive( sem_RXBUFF );

		// TIMEOUT
		if (  ticks-- <= 0 ) {
			exit_code = ATRSP_TIMEOUT;
		}
	}

	// Print debug causes
	switch( exit_code) {
	case ATRSP_OK:
		//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ATRSP_OK\r\n"));
		break;
	case ATRSP_ERROR:
		//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ERROR\r\n"));
		break;
	case ATRSP_TIMEOUT:
		//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: TIMEOUT\r\n"));
		break;
	default:
		//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ERROR RSP NO ESPERADA !!\r\n"));
		exit_code = ATRSP_UNKNOWN;
		break;
	}

	gprs_print_RX_buffer();
	return (exit_code);

}
//------------------------------------------------------------------------------------
