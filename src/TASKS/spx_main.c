/*
 * main.c
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 *
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e -U flash:w:spxR4.hex
 *  avrdude -v -Pusb -P /dev/ttyUSB0 -b 9600 -c avr109 -v -v -p x256a3 -F -e -U flash:w:spxR4.hex
 *
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e
 *
 *  REFERENCIA: /usr/lib/avr/include/avr/iox256a3b.h
 *
 *  El watchdog debe estar siempre prendido por fuses.
 *  FF 0A FD __ F5 D4
 *
 *  PROGRAMACION FUSES:
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Uflash:w:spx.hex:a -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -Uflash:w:/home/pablo/Spymovil/workspace/spxR5/Release/spxR5.hex:a
 *  /usr/bin/avrdude -p x256a3b -P /dev/ttyUSB0 -b 9600 -c avr109 -v -v -e -V -Uflash:w:/home/pablo/Spymovil/workspace/spxR5/Release/spxR5.hex
 *
 *  Para ver el uso de memoria usamos
 *  avr-nm -n spxR5.elf | more
 *  avr-nm -Crtd --size-sort spxR5.elf | grep -i ' [dbv] '
 *
 *  Archivos modificados en el server para version 4.0.0a
 *  spy.py
 *  spy.conf
 *  spy_raw_frame.py
 *  spy_raw_frame_test.py
 *  SPY_init_conf_global.py
 *  SPY_init_conf_analog.py
 *  SPY_init_conf_app.py
 *  SPY_init_conf_base.py
 *  SPY_init_conf_counter.py
 *  SPY_init_conf_digital.py
 *  spy_utils.py
 *
 *  Colores del hoover editor:
 *  https://stackoverflow.com/questions/3571850/eclipse-change-popup-text-background-color-when-hovering-the-mouse-on-a-keyword
 *  Preferences > C/C++ > Editor > Source hover background
 *
 * -------------------------------------------------------------------------------------------
 * R5.0.0 @ 20230626:
 * Modifico el protocolo para trabajar con las API's
 *
 * -------------------------------------------------------------------------------------------
 * R4.0.4g @ 20220622: issueLTE
 * Creo una branch y trabajo en esta para resolver los temas del funcionamiento con los modems
 * SIM7600 LTE
 * Con el comando ATI determinamos si estamos con un modem LTE o no.
 * Creo una variable en xCOMMS_stateVars que sea el tipo de modem detectado en ATI.
 *
 * -------------------------------------------------------------------------------------------
 * R4.0.4f @ 20220511:
 * -Elimino todo lo referente a modbus CBLOCK
 * -Agrego la aplicacion OCEANUS.
 *  Los canales se configuran como si fuesen modbus.Solo importa el nombre. El resto de los
 *  datos se ignoran.
 * -------------------------------------------------------------------------------------------
 * R4.0.4e @ 20220511:
 * * Habilito el test de output modbus desde cmdMode.
 *
 * -------------------------------------------------------------------------------------------
 *
 * R4.0.4c @ 20220502:
 * issue01: Los contadores cuando no estan en modo caudal, cuentan pero no muestran el valor
 *          El problema era un error en la funcion counters_read().
 * issue02: En el modo pilotos, la pA y pB que usa son la misma.
 *
 * -------------------------------------------------------------------------------------------
 *
 * R4.0.4b @ 20220329:
 * Agrego la aplicacion de GENPULSOS para poder generar los pulsos en las dosificadoras de
 * Colonia.
 * - Creo la aplicacion ( tkApp, tkCmd(help, config, status, save, reload) )
 * - Modifico tkData para que al arrancar toma nota de los canales pA,pB,Q que se usan en
 * las aplicaciones de piloto y genpulsos.
 * - Agrego las funciones de poder preguntar los valores.
 * - Elimino estas funciones de ul_pilotos para que use estas funciones.
 *
 * BUG001: Cuando transmito una ventana con varios frames, completo la ventana pero relleno con 0.
 * GET /cgi-bin/SPY/spy.py?DLGID=MER005&TYPE=DATA&VER=4.0.3d&PLOAD=
 * CTL:1910;DATE:220325;TIME:045120;PMP:1.08;bt:12.78;
 * CTL:1911;DATE:220325;TIME:045219;PMP:1.09;bt:12.82;
 * CTL:1912;DATE:220325;TIME:045318;PMP:1.10;bt:12.81;
 * CTL:1912;DATE:000000;TIME:000000;PMP:0.00;bt:0.00; <<<<<<<<<<<<
 * CTL:1912;DATE:000000;TIME:000000;PMP:0.00;bt:0.00  <<<<<<<<<<<<
 *
 * GET /cgi-bin/SPY/spy.py?DLGID=PABLO&TYPE=DATA&VER=4.0.4b&PLOAD=
 * CTL:5;DATE:010101;TIME:000509;pA:-2.50;pB:-2.50;q0:0.000;bt:9.79;
 * CTL:6;DATE:010101;TIME:000608;pA:-2.50;pB:-2.50;q0:0.000;bt:9.86;
 * CTL:6;DATE:000000;TIME:000000;pA:0.00;pB:0.00;q0:0.000;bt:0.00; <<<<<<<<<<<
 * CTL:6;DATE:000000;TIME:000000;pA:0.00;pB:0.00;q0:0.000;bt:0.00  <<<<<<<<<<<
 *
 * BUG002: En los contadores: Cuando no hay pulsos y aparecen cuenta un -1.
 * Hago un cambio en la forma de procesar los pulsos, tomando una base de tiempo de ticks.
 *
 * issue01: La aplicacion PILOTOS no detecta los sensores de presion.
 *          Estaba mal evaluada la condicion en PRESIONES_NO_CONFIGURADAS()
 *
 * issue02: Los modbus muchas veces dan error en la configuracion. Puede que sea un problema
 *          del driver de la uart porque parece como que se come caracteres.
 *          La solucion es 'marcar' los canales que no se configuraron y pedir reconfiguracion
 *          solo de estos.
 *          Antes modifico la configuracion modbus mandando al server que canales quiero que me
 *          mande la configuracion
 *
 * issue03: En cada respuesta que recibo del server, verifico su integridad con el checksum.
 *          Modifico la funcion rcvd_response().
 *          Vemos que hay muchos errores, se pierde en los frames largos algún byte.
 *          Modifico en los drivers de UARTS la prioridad de interrupccion y la pongo HIGH para
 *          gprs y aux1.
 *          Modifico la funcion frtos_uart_write. La pruebo con buffers de 313 bytes en 100 lineas
 *          consecutivas ( xprintf_test ) y funciona ok.
 *          Modifico bugs en RX_ISR para que use las correctas funciones de ringBuffer.
 *          Modifico el gprs_put2 para que use semaforo para todo el proceso.
 *
 *			Si el dlgid es DEFAULT y el frame AUTH devuelve error, entonces espero 1h antes de reintentar.
 *
 *          Vimos que el problema es que con todas las interrupciones de igual prioridad, a veces
 *          se pueden perder datos de la UART gprs. Si bien una solucion fue darle prioridad mas
 *          alta, podemos explorar otras alternativas.
 *          Una es usar DMA.
 *
 *
 * -------------------------------------------------------------------------------------------
 * R4.0.3f @ 20220308:
 * - Cuando recibe un frame modbus del server, chequea si los formatos están bien.
 * - Luego chequea el resultado de la operación de IO.
 * - Si alguno anduvo mal responde un NACK
 * - Si ambos anduvieron bien responde un ACK
 * - Si no hubo operacion de MODBUS, no responde nada.
 *  Todo el manejo de la flag se hace dentro del modulo de modbus.
 *
 * -------------------------------------------------------------------------------------------
 * R4.0.3d @ 20220224:
 * Aumento el xprintf buffer a 384 de modo de poder imprimir todo el gprsbuffer
 * Si hay luegar para 2 frames, los cargo y transmito juntos.
 * Resultados:
 *
 * Frente a problemas en el servidor cuando transmite varios frames de datos, lo limito a 1 solo.
 * Resultados: Funciona bien.
 * Como hago ahora abro_socket, transmito, proceso respuesta, cierro_socket.
 * Esto hace que en c/frame tome uno 10s de proceso.
 *
 * -------------------------------------------------------------------------------------------
 * R4.0.3c @ 20220222:
 * Incremento el tamano de los buffers de RX del GPRS de 512 a 576 bytes tanto a nivel del
 * driver como de la aplicacion tkComms
 *
 * -------------------------------------------------------------------------------------------
 *  R4.0.2h3 @ 20220202:
 *  Frente a los problemas de desconexiones cuando recibe datos modbus, modificamos en modo BETA
 *  la funcion data_process_response_MBUS para que solo loguee lo que parsea pero no lo aplique.
 *  Modifico modbus_write_output_register para controlar no dejar colgado el semaforo de modbus.
 *  Cambio el procesamiento de los mensajes: En la tkComms solo los parseo y encolo
 *  Luego en tkData los desencolo y proceso.
 *  Agrego un control para confirmar al servidor que los mensajes se recibieron y encolaron correctamente.
 *  Esto le da mas robustes al sistema ya que el server se entera como anduvo la cosa.
 *
 *
 *  R4.0.2h @ 2021-12-15:
 *  - Agrega un debug ALL
 *  - Corrige un bug que c/frame recibido modificaba la hora local y esto descontrolaba los timestamps
 *  - Aumente el wdt de la tkData
 *  - Sistema de manejo de SMS.
 *
 *  R4.0.2b @ 2021-10-18:
 *  - Uso una cola FIFO para setear las presiones.
 *  - Rediseño el sistema de estados
 *  - Luego de hacer un ajuste, chequear si cambio el slot y en caso afirmativo
 *    encolo la nueva peticion.
 *  - Luego de ajustar el piloto, esperar 30s o 1 minuto que la presion se
 *    estabilize antes de volver a medir. En particular en las valvulas grandes
 *    les lleva mas tiempo.
 *    Esto lo hago en el estado de LEER_INPUTS
 * - El cálculo del caudal genera mucho ruido.
 *    Determino si el equipo mide caudal y entonces medimos el intervalo entre el primer
 *    y ultimo pulso en el timerpoll
 *
 * B) Generar el pwrsave a partir del RTC
 *
 *  R4.0.2a @ 2021-10-12:
 *  - Modificado para soportar los micros A3U. Estos no tienen el RTC32 que se
 *    usa en modo tickeless por lo que ahora elimino dicha funcion desde
 *    FRTOSCOnfig ( configUSE_TICKLESS_IDLE )
 *  - Agrego en cmd::status que lea e indique los device ID.
 *  - Para programarlo se debe hacer desde cmd.mode ya que el tipo de micro cambia
 *  - Con fuses:
 *  avrdude -px256a3u -cavrispmkII -Pusb -V -u -e -Uflash:w:spxR6.hex:a -Ufuse0:w:0xff:m -Ufuse1:w:0xaa:m -Ufuse2:w:0xbd:m -Ufuse4:w:0xf5:m -Ufuse5:w:0xd6:m
 *  - Sin fuses
 *  avrdude -px256a3u -cavrispmkII -Pusb -V -u -e -Uflash:w:spxR6.hex:a
 *
 *
 *  R4.0.1a @ 2021-10-05:
 *  - Modificaciones al modbus:
 *  Agrego 2 parametros para indicar el dispositivo remoto modbus al que mandarle
 *  el status del datalogger.
 *  La configuracion se hace en BASE.(hash, online)
 *
 *  R4.0.0e @ 2021-09-24:
 *  - Modificaciones al modbus:
 *  Cada registro debe contener la dirección del dispositivo remoto.
 *  De este modo podemos polear a varios dispositivos en el mismo bus.
 *
 *
 *  R4.0.0d @ 2021-09-24:
 *  - Cuando tiene muchos datos para transmitir, se restea por wdg de comms.
 *    Se soluciona limpiando el wdg luego que transmite c/ventana.
 *  - Los reintentos de cerrar el socket no funcionan por lo que solo se demora mas.
 *    Pasamos los reintentos ( MAX_TRYES_SOCKET_CLOSE ) a 1.
 *  - Aumentamos los slots de pilotos a 12
 *  - Agregamos que remotamente se pueda fijar una presion momentaneamente.
 *  - Al arrancar el modem le doy ATE0 para evitar el echo.
 *
 *
 *
 *  R4.0.0c @ 2021-08-31:
 *  - Modbus: Polea e imprime todos los canales.
 *    NO TRANSMITE los canales modbus !!
 *    NO SE CONFIGURA online modbus !!
 *    REVISAR CONFIGRAR CANALES MODBUS LA ADDRESS !!!
 *  - Pilotos:
 *    Sale por wdg al ajustar. Corregido
 *    En el ajuste, considera pB y no pREF para el dP: Corregido.
 *
 *  R4.0.0b @ 2021-08-24:
 *  - Como los motores de los pilotos pueden cambiar los pulsoxrev, agrego 2 parametros a pilotos_conf
 *    que son pulsoXrev y pwidth.
 *    Estos se configuran por cmdMode y online.
 *  - Agrego setear en GPRS el modo,sat,band,pref.
 *  - Por un bug no se configuraba el modo hw de los contadores. Listo.
 *
 *  R4.0.0a @ 2021-08-16:
 *  - En modo apagado prendo ATE1 para seguir en el log los comandos enviados
 *    En modo online (entry) lo apago ATE0 para no llenar el rxbuffer con lineas de comandos
 *  - El CSQ no se estaba actualizando y mandaba 0. Actualizo la variable xCOMMS_stateVars.csq.
 *  - En modo online espera y modo apagado espera, monitoreo la senal SPX_REDIAL
 *  - Cuando apago el modem, actualizo xCOMMS_stateVars.csq a 0.
 *  - El config default pone aplicacion en 0 y modbus en off.
 *
 */

#include "dma_driver.h"
#include "spx.h"


//------------------------------------------------------------------------------------
int main( void )
{
	// Clock principal del sistema
	u_configure_systemMainClock();

	// Configuramos y habilitamos el watchdog a 8s.
	WDT_EnableAndSetTimeout(  WDT_PER_8KCLK_gc );
	if ( WDT_IsWindowModeEnabled() )
		WDT_DisableWindowMode();

	// Las interrupciones de UART GPRS deben ser HighLevelPriority
	PMIC_EnableHighLevel();

	initMCU();

	// EL manejo de la recepcion de datos de UARTS es por DMA.
	DMA_Enable();

	frtos_open(fdTERM, 9600 );
	frtos_open(fdGPRS, 115200);

	// Creo los semaforos
	xprintf_init();
	gprs_init_outofrtos();

	// Creamos las tareas
	xHandle_tkCtl = xTaskCreateStatic(tkCtl, "CTL", tkCtl_STACK_SIZE, (void *)1, tkCtl_TASK_PRIORITY, xTask_Ctl_Buffer, &xTask_Ctl_Buffer_Ptr );
	xHandle_tkCmd = xTaskCreateStatic(tkCmd, "CMD", tkCmd_STACK_SIZE, (void *)1, tkCmd_TASK_PRIORITY, xTask_Cmd_Buffer, &xTask_Cmd_Buffer_Ptr );
	xHandle_tkGprs = xTaskCreateStatic(tkGprs, "RX", tkGprs_STACK_SIZE, (void *)1, tkGprs_TASK_PRIORITY, xTask_Gprs_Buffer, &xTask_Gprs_Buffer_Ptr );

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);

}
//------------------------------------------------------------------------------------
void vApplicationIdleHook( void )
{
	// Como trabajo en modo tickless no entro mas en modo sleep aqui.
//	if ( sleepFlag == true ) {
//		sleep_mode();
//	}

}
//------------------------------------------------------------------------------------
void vApplicationTickHook( void )
{

}
//------------------------------------------------------------------------------------
// Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP().
//------------------------------------------------------------------------------------
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	// Es invocada si en algun context switch se detecta un stack corrompido !!
	// Cuando el sistema este estable la removemos.
	// En FreeRTOSConfig.h debemos habilitar
	// #define configCHECK_FOR_STACK_OVERFLOW          2

	xprintf_P( PSTR("PANIC:%s !!\r\n\0"),pcTaskName);

}
//------------------------------------------------------------------------------------
/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
//------------------------------------------------------------------------------------
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
//------------------------------------------------------------------------------------

