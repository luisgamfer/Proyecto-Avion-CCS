//*****************************************************************************
//
// Codigo de partida comunicacion TIVA-QT (Abril2021)
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
// Autores de las modificaciones: Miguel Burgos Márquez, Luis Francisco Gámez Fernandez
//  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
//  basada en la TIVA, en las que existe un intercambio de mensajes con un interfaz
//  grÃ¡fico (GUI) Qt.
//  La aplicacion se basa en un intercambio de mensajes con ordenes e informacion, a traves  de la
//  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
//  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
//   En el ejemplo basico de partida se implementara la recepcion de un mensaje
//  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
//  mensaje enviado desde la placa al GUI, para mostrar el estado de los botones.
//
//*****************************************************************************
#include<stdbool.h>
#include<stdint.h>
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADC
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de timers
#include <utils/uartstdioMod.h>     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
#include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
#include "utils/cpu_usage.h"
#include "commands.h"
#include <serial2USBprotocol.h>
#include <usb_dev_serial.h>
#include "usb_messages_table.h"
#include <math.h>


#define ADCTASKSTACKSIZE 128
#define RELOJTASKSTACKSIZE 128
#define COMBUSTIBLETASKSTACKSIZE 200
#define ALTURATASKSTACKSIZE 200
#define RADIOSKSTACKSIZE 100

// Variables globales "main"
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;

static QueueHandle_t cola_Mpotenciometros;
static QueueHandle_t cola_Reloj;
static QueueHandle_t cola_Vcombustible;
static QueueHandle_t cola_Tcombustible;
static QueueHandle_t cola_Valtura;
static QueueHandle_t cola_Maltura;
static QueueHandle_t cola_Caltura;
extern QueueHandle_t cola_comandos;
extern QueueHandle_t cola_radio;

static QueueSetHandle_t grupo_colas;
SemaphoreHandle_t mutexUSB;
SemaphoreHandle_t semaforo_tiempo;
SemaphoreHandle_t mutexRGB;
SemaphoreHandle_t mutexUART;


static uint32_t ui32Color[3];


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t ui8Count = 0;

	if (++ui8Count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		ui8Count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que se produce un fallo de asignacio de heap
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

//// Codigo para procesar los mensajes recibidos a traves del canal USB

static portTASK_FUNCTION( USBMessageProcessingTask, pvParameters ){

    uint8_t pui8Frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
    int32_t i32Numdatos;
    uint8_t ui8Message;
    void *ptrtoreceivedparam; // <---?
	uint32_t ui32Errors=0;
	static bool traza = false;

	/* The parameters are not used. */
	( void ) pvParameters;

	//
	// Mensaje de bienvenida inicial.
	//
	UARTprintf("\n\nBienvenido a la aplicacion Simulador de Vuelo (curso 2020/21)!\n");
	UARTprintf("\nAutores: Miguel y Luis ");

	for(;;)
	{
        //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
		i32Numdatos=receive_frame(pui8Frame,MAX_FRAME_SIZE); //Esta funcion es bloqueante
		if (i32Numdatos>0)
		{	//Si no hay error, proceso la trama que ha llegado.
			i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobación checksum
			if (i32Numdatos<0)
			{
				//Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
				ui32Errors++;
                // Procesamiento del error (TODO, POR HACER!!)
			}
			else
			{
			    //Lee de la cola el estado de la traza [on|off]
			    xQueuePeek(cola_comandos,&traza,0);
				//El paquete esta bien, luego procedo a tratarlo.
                //Obtiene el valor del campo mensaje
				ui8Message=decode_message_type(pui8Frame);
                //Obtiene un puntero al campo de parametros y su tamanio.
                i32Numdatos=get_message_param_pointer(pui8Frame,i32Numdatos,&ptrtoreceivedparam);
				switch(ui8Message)
				{
				case MENSAJE_PING :
					//A un mensaje de ping se responde con el propio mensaje
					i32Numdatos=create_frame(pui8Frame,ui8Message,0,0,MAX_FRAME_SIZE);
					if (i32Numdatos>=0)
					{
					    //Si la traza esta activa, notifico por consola la respuesta
					    if(traza){
					        xSemaphoreTake(mutexUART,portMAX_DELAY);
					        UARTprintf("Enviando respuesta PING\r\n\n");
					        xSemaphoreGive(mutexUART);
					    }

					    xSemaphoreTake(mutexUSB,portMAX_DELAY);
						send_frame(pui8Frame,i32Numdatos);
						xSemaphoreGive(mutexUSB);
					}else{
						//Error de creacion de trama: determinar el error y abortar operacion
						ui32Errors++;
						// Procesamiento del error (TODO)
//						// Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
//						// tener que copiar y pegar todo en cada operacion de creacion de paquete
						switch(i32Numdatos){
						case PROT_ERROR_NOMEM:
							// Procesamiento del error NO MEMORY (TODO)
							break;
						case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
//							// Procesamiento del error STUFFED_FRAME_TOO_LONG (TODO)
							break;
						case PROT_ERROR_MESSAGE_TOO_LONG:
//							// Procesamiento del error MESSAGE TOO LONG (TODO)
							break;
						}
                        case PROT_ERROR_INCORRECT_PARAM_SIZE:
                        {
                            // Procesamiento del error INCORRECT PARAM SIZE (TODO)
                        }
                        break;
					}
					break;

				case MENSAJE_INICIO:
				{
				    if(traza){
				        xSemaphoreTake(mutexUART,portMAX_DELAY);
				        UARTprintf("Botón de inicio pulsado\r\n\n");
				        xSemaphoreGive(mutexUART);
				    }

				          ADCSequenceEnable(ADC0_BASE, 1);
				          IntEnable(INT_ADC0SS1); // Habilitación a nivel global del sistema
				          ADCIntEnable(ADC0_BASE,1); //Habilitación del secuenciador dentro del periférico


				         // Activa el Timer4A (empezará a funcionar)
				          TimerEnable(TIMER4_BASE, TIMER_A);

				          // Activa el Timer4B (empezará a funcionar)
				          TimerEnable(TIMER5_BASE, TIMER_A);


				}
				    break;

				case MENSAJE_VELOCIDAD:
                {
                    PARAM_MENSAJE_VELOCIDAD velocidad;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(velocidad),&velocidad)>0){

                        xSemaphoreTake(mutexRGB,portMAX_DELAY);
                        ui32Color[BLUE] = velocidad.bIntensity*65535/200.0; // Minimo: 0; Maxima 0xFFFF
                        xSemaphoreGive(mutexRGB);

                        if(traza){
                            xSemaphoreTake(mutexUART,portMAX_DELAY);
                            UARTprintf("LED azul encendido con intensidad proporcional a la velocidad\r\n\n");
                            xSemaphoreGive(mutexUART);
                        }

                       // Intensidad= velocidad.bIntensity*65535/200; //Escalamos el resultado
                        RGBColorSet(ui32Color);

                        // Metemos la velocidad en la cola que conecta con la tarea combustible
                        xQueueSendToBack(cola_Vcombustible,&velocidad.bIntensity,portMAX_DELAY);

                        //Metemos la velocidad en la cola que conecta con la tarea de altura
                        xQueueSendToBack(cola_Valtura,&velocidad.bIntensity,portMAX_DELAY);

                    }else//Error de tamaño de parametro
                        ui32Errors++; // Tratamiento del error
                }
				    break;

				default:
				 {
					PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
					parametro.message=ui8Message;
					//El mensaje esta bien pero no esta implementado
					i32Numdatos=create_frame(pui8Frame,MENSAJE_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
					if (i32Numdatos>=0)
					{
						send_frame(pui8Frame,i32Numdatos);
					}
					break;
				 }
				}// switch
			}
		}else{ // if (ui32Numdatos >0)
			//Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
			ui32Errors++;
			// Procesamiento del error (TODO)
		}
	}
}

static portTASK_FUNCTION(ADCTask, pvParameters){

    //Variables usadas en la creacion de mensajes
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    PARAM_MENSAJE_POTENCIOMETRO giro;
    int32_t i32Numdatos;
    (void) pvParameters;

    uint32_t pui32_DatosPotenciometros_leidos[3]; //Creamos el array con los datos leidos de la cola

    while(1){

        if(xQueueReceive(cola_Mpotenciometros,&pui32_DatosPotenciometros_leidos,portMAX_DELAY)==pdTRUE){

            //Introducimos en el struct los valores leídos del ADC
              giro.roll=pui32_DatosPotenciometros_leidos[0];
              giro.pitch=pui32_DatosPotenciometros_leidos[1];
              giro.yaw=pui32_DatosPotenciometros_leidos[2];

              xQueueSendToBack(cola_Maltura,&giro.pitch,portMAX_DELAY); //Introducimos el pitch en la cola Maltura

            i32Numdatos=create_frame(pui8Frame,MENSAJE_POTENCIOMETRO,&giro,sizeof(giro),MAX_FRAME_SIZE);
            if (i32Numdatos>=0)
                        {
                            xSemaphoreTake(mutexUSB,portMAX_DELAY);
                            send_frame(pui8Frame,i32Numdatos); //Mandamos el dataframe
                            xSemaphoreGive(mutexUSB);
                        }
        }

    }

}

static portTASK_FUNCTION(RelojTask, pvParameters){

    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int32_t i32Numdatos;
    PARAM_MENSAJE_RELOJ m_reloj;
    (void) pvParameters;
    static bool traza = false;

    while(1){

        if(xQueueReceive(cola_Reloj,&m_reloj,portMAX_DELAY)==pdTRUE){

            xQueuePeek(cola_comandos,&traza,0);

            if(traza && m_reloj.reloj % 60 == 0){
                xSemaphoreTake(mutexUART,portMAX_DELAY);
                UARTprintf("%d minuto(s) desde el inicio\r\n\n",m_reloj.reloj/60);
                xSemaphoreGive(mutexUART);
            }

            i32Numdatos=create_frame(pui8Frame,MENSAJE_RELOJ,&m_reloj,sizeof(m_reloj),MAX_FRAME_SIZE);
            if (i32Numdatos>=0)
                        {
                            xSemaphoreTake(mutexUSB,portMAX_DELAY);
                            send_frame(pui8Frame,i32Numdatos); //Mandamos el dataframe
                            xSemaphoreGive(mutexUSB);
                        }
        }

    }
}

static portTASK_FUNCTION(CombustibleTask, pvParameters){

        (void) pvParameters;
        uint8_t pui8Frame[MAX_FRAME_SIZE];
        int32_t i32Numdatos;
        PARAM_MENSAJE_COMBUSTIBLE m_combustible;

        m_combustible.combustible = 100.0; // Combustible inicial
        float velocidad_cola = 0.0; // Variable donde se leeran los datos de la cola
        float suma_velocidad = 0.0; // Variable suma de las velocidades en 1 hora
        float velocidad_ant = 0.0; // Variable que guarda la velocidad de la iteración anterior
        int contador = 0; // Cuenta de veces que se cambia la velocidad
        uint32_t segundos;

        static bool traza = false;


        while(1){

            // Se recibe la velocidad que llega por mensaje. El tercer argumento es 0 para que la tarea no se bloquee esperando elementos de la cola
            if(xQueueReceive(cola_Vcombustible,&velocidad_cola,0)==pdTRUE){

                // Se leen las velocidades de la cola y se manipulan para poder hacer el promediado de velocidad
                suma_velocidad = suma_velocidad + velocidad_cola;
                contador++;
                velocidad_ant = velocidad_cola;

            }

            // Leemos los segundos de la cola
            if(xQueueReceive(cola_Tcombustible,&segundos,portMAX_DELAY)==pdTRUE){

                xQueuePeek(cola_comandos,&traza,0);

                // Comprobamos si el combustible tiene niveles aceptables, si no, se ejecutan acciones en la TIVA
                   if(segundos % 60 == 0 && segundos > 0){

                       // Hacemos el calculo de combustible restante para los distintos casos posibles
                       if(contador != 0 && m_combustible.combustible > 0){

                           // Formula usando el promediado
                           m_combustible.combustible = m_combustible.combustible - 0.0369*exp(0.02*((suma_velocidad/contador)*100.0/250.0));

                           // No se ha movido la palanca durante el minuto
                       }else if(contador == 0 && m_combustible.combustible > 0){

                           // Formula usando la velocidad anterior guardada ( si el contador es 0 al hacer promediado la formula se iria al infinito
                           m_combustible.combustible = m_combustible.combustible - 0.0369*exp(0.02*((velocidad_ant)*100.0/250.0));

                       }else{

                           // Cuando el combustible baja de 0 se deja ya siempre en 0
                           m_combustible.combustible = 0.0;

                       }

                       if(m_combustible.combustible < 20.0){ //Cuando el combustible baja de 20L

                                       xSemaphoreTake(mutexRGB,portMAX_DELAY);
                                       ui32Color[GREEN] = 0xFFFF;
                                       xSemaphoreGive(mutexRGB);

                                       RGBColorSet(ui32Color); //Se enciende el LED verde

                                       if(traza){
                                           xSemaphoreTake(mutexUART,portMAX_DELAY);
                                           UARTprintf("LED verde encendido por combustible bajo\r\n\n");
                                           xSemaphoreGive(mutexUART);
                                       }

                                       if(m_combustible.combustible <= 0.0){

                                                //ui32Color[BLUE] = 0x0;
                                                ADCSequenceDisable(ADC0_BASE, 1); //Deshabilitamos el ADC
                                                xQueueReset(cola_Mpotenciometros); //Vaciamos la cola para que no quede ningún mensaje que mandar
                                                //RGBColorSet(ui32Color);

                                       }

                        }

                       suma_velocidad = 0.0;
                       contador = 0;

                       xQueueSendToBack(cola_Caltura,&m_combustible.combustible,portMAX_DELAY); //Introducimos el combustible en la cola Caltura

                       if(traza){
                           xSemaphoreTake(mutexUART,portMAX_DELAY);
                           UARTprintf("Nivel de combustible modificado\r\n\n");
                           xSemaphoreGive(mutexUART);
                       }

                       i32Numdatos=create_frame(pui8Frame,MENSAJE_COMBUSTIBLE,&m_combustible,sizeof(m_combustible),MAX_FRAME_SIZE);
                       if (i32Numdatos>=0)
                                   {
                                      xSemaphoreTake(mutexUSB,portMAX_DELAY);
                                      send_frame(pui8Frame,i32Numdatos); //Mandamos el dataframe
                                      xSemaphoreGive(mutexUSB);
                                   }
                }
            }
        }
}

static portTASK_FUNCTION(AlturaTask, pvParameters){

        (void) pvParameters;
        QueueSetMemberHandle_t  Activado;
        uint8_t pui8Frame[MAX_FRAME_SIZE];
        int32_t i32Numdatos;
        uint16_t pitch;
        PARAM_MENSAJE_ALTURA m_altura;
        float angulo, angulo_rad,combustible,altura_anterior,blink,diferencia_altura;

        float velocidad_cola = 0.0;
        float velocidad_sin_combustible = 0.0; //Se inicializa la velocidad a usar en el caso en el que nos quedemos sin combustible
        altura_anterior = 3000.0; //Definimos la altura anterior

        m_altura.altura = 3000.0; //Definimos la altura inicial en metros

        blink = 1000.0/3000.0; //Inicializamos el blink

        combustible = 100.0; //Combustible inicial

        static bool traza = false;

        while(1){


            Activado = xQueueSelectFromSet( grupo_colas, portMAX_DELAY); //Creo que va dentro del while tal y como lo he puesto

            if(Activado == cola_Valtura){ //Si se ha recibido un dato de cola_Valtura

                xQueueReceive(cola_Valtura,&velocidad_cola,0); //No sé si este if haría falta o simplemente se podría poner fuera
                RGBBlinkRateSet(blink);

            }


            if(Activado == cola_Maltura){ //Si se ha recibido un dato de cola_Maltura

                xQueueReceive(cola_Maltura,&pitch,0); //Se lee los segundos de la cola. Entraremos en este if cada segundo del tiempo real

                //Se transforma el valor obtenido del pitch en un ángulo
                 pitch = pitch & 0xFFF;
                 angulo = ((float)pitch)/4096;
                 angulo = (angulo * (90 +90)) -90;
                 angulo_rad = -angulo*3.141592/180;
           }

            if(Activado == semaforo_tiempo){ //Si se ha abierto el semáforo del tiempo

                xSemaphoreTake(semaforo_tiempo,0);   //Nos aseguramos de que el semaforo esta cerrado

                xQueuePeek(cola_comandos,&traza,0);

                if(xQueueReceive(cola_Caltura,&combustible,0)==pdTRUE){


                }

                if(combustible > 0.0){ //Si hay combustible


                    m_altura.altura = m_altura.altura + velocidad_cola*1000.0/3600.0*sin(angulo_rad); //Se calcula la nueva altura

                }else{ //Para el caso en el que nos hayamos quedado sin combustible

                    velocidad_sin_combustible = velocidad_sin_combustible + 9.8; //Valor incrementándose por la caída del avión
                    m_altura.altura = m_altura.altura + velocidad_sin_combustible*sin(-3.141592/4.0); //Vamos actualizando el valor de la pérdida de altura


                }

                diferencia_altura = abs((int)(m_altura.altura - altura_anterior)); //Diferencia de altura con respecto la que había en el paso anterior (útil para actualizar parpadeo)

                if(m_altura.altura >= 0.0){ //Mientras que el valor de la altura sea mayor que 0


                          if(diferencia_altura > 200.0){ //Si la diferencia de altura es mayor que 200m

                              altura_anterior = m_altura.altura;
                              blink = 1000.0/m_altura.altura; //Se calcula el nuevo parpadeo
                              RGBBlinkRateSet(blink);
                              if(traza){
                                  xSemaphoreTake(mutexUART,portMAX_DELAY);
                                  UARTprintf("Cambio de la frecuencia de parpadeo por cambio en altitud de 200 metros\r\n\n");
                                  xSemaphoreGive(mutexUART);
                              }
                          }


                            if(m_altura.altura < 800.0){ //Si la altura es menor de 800m

                                   //Encendemos el LED ROJO
                                   xSemaphoreTake(mutexRGB,portMAX_DELAY);
                                   ui32Color[RED] = 0xFFFF;
                                   xSemaphoreGive(mutexRGB);

                                   RGBColorSet(ui32Color);

                                   if(traza){
                                       xSemaphoreTake(mutexUART,portMAX_DELAY);
                                       UARTprintf("LED rojo encendido por baja altitud\r\n\n");
                                       xSemaphoreGive(mutexUART);
                                   }


                               }

                                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALTURA,&m_altura,sizeof(m_altura),MAX_FRAME_SIZE);
                                if (i32Numdatos>=0)
                                            {
                                               xSemaphoreTake(mutexUSB,portMAX_DELAY);
                                               send_frame(pui8Frame,i32Numdatos); //Mandamos el dataframe
                                               xSemaphoreGive(mutexUSB);
                                            }
                    }else{

                               if(traza){
                                   xSemaphoreTake(mutexUART,portMAX_DELAY);
                                   UARTprintf("Colision con el suelo, es necesario reiniciar\r\n\n");
                                   xSemaphoreGive(mutexUART);
                               }

                               i32Numdatos = create_frame(pui8Frame,MENSAJE_COLISION, NULL, 0,MAX_FRAME_SIZE); //Mandamos un mensaje vacío en caso de que colisionemos
                               if (i32Numdatos>=0)
                                           {
                                              xSemaphoreTake(mutexUSB,portMAX_DELAY);
                                              send_frame(pui8Frame,i32Numdatos); //Mandamos el dataframe
                                              xSemaphoreGive(mutexUSB);
                                              TimerDisable(TIMER5_BASE, TIMER_A); //Deshabilitamos el timer para que se pare el Reloj
                                              TimerDisable(TIMER4_BASE, TIMER_A); //Deshabilitamos el timer para que no se obtengan más muestras de ADC
                                              RGBDisable();

                                           }
                           }


            }


        }

}

static portTASK_FUNCTION(RadioTask, pvParameters){

    //Variables usadas en la creacion de mensajes
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    PARAM_MENSAJE_MSG_RADIO mensaje_radio;
    int32_t i32Numdatos;
    (void) pvParameters;

    while(1){

        if(xQueueReceive(cola_radio,&mensaje_radio,portMAX_DELAY)==pdTRUE){ //Se leen los mensajes radio de la cola

            i32Numdatos=create_frame(pui8Frame,MENSAJE_MSG_RADIO,&mensaje_radio,sizeof(mensaje_radio),MAX_FRAME_SIZE);
            if (i32Numdatos>=0)
                        {
                            xSemaphoreTake(mutexUSB,portMAX_DELAY);
                            send_frame(pui8Frame,i32Numdatos); //Mandamos el dataframe
                            xSemaphoreGive(mutexUSB);
                        }
        }

    }

}



//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

    uint32_t ui32Period;


	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 50 MHz (200 Mhz del Pll dividido por 4)

	//Aqui vamos a configurar el ADC (hecho por nosotros)

	//CONFIGURACIÓN TIMER0
	  // Habilita periferico Timer4
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	  //Llamar a la función
	  SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER4);

	  // Configura el Timer4 para cuenta periodica de 32 bits (no lo separa en TIMER4A y TIMER4B)
	  TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
	  // Periodo de cuenta de 0.1s. SysCtlClockGet() te proporciona la frecuencia del reloj del sistema, por lo que una cuenta
	  // del Timer a SysCtlClockGet() tardara 1 segundo, a 0.5*SysCtlClockGet(), 0.5seg, etc...
	  ui32Period = (SysCtlClockGet() / 10);
	  // Carga la cuenta en el Timer0A
	  TimerLoadSet(TIMER4_BASE, TIMER_A, ui32Period -1);


	   TimerControlTrigger(TIMER4_BASE,TIMER_A,1); //Para decir que se habilita que el timer esté conectado al ADC

	    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //Habilitamos el ADC0
	    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

	   ADCHardwareOversampleConfigure(ADC0_BASE,64); //Promediado hardware de 64 bits

	   ADCSequenceDisable(ADC0_BASE, 1); //Deshabilitamos el secuenciador 1 del ADC0 para configurarlo

	   ADCSequenceConfigure(ADC0_BASE, 1,ADC_TRIGGER_TIMER, 0); //Disparo de muestreo por instrucciones de TIMER

	   //Configuramos los 3 conversores del secuenciador 1 para muestreo

	   ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH7 );
       ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH3 );
       ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END );


	    //Inicializa la biblioteca RGB (configurando las salidas como RGB)
	    RGBInit(1);
	    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);  //Esto es necesario para que el timer0 siga funcionando en bajo consumo
	    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);  //Esto es necesario para que el timer1 siga funcionando en bajo consumo
        SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);  //Esto es necesario para que el timer1 siga funcionando en bajo consumo


	// Get the system clock speed.
	g_ui32SystemClock = SysCtlClockGet();

    //CONFIGURACIÓN TIMER5

    // Habilita periferico Timer5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    //Llamar a la función
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER5);

    // Configura el Timer5 para cuenta periodica de 32 bits (no lo separa en TIMER5A y TIMER5B)
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
	  // Carga la cuenta en el Timer5A
	  TimerLoadSet(TIMER5_BASE, TIMER_A, g_ui32SystemClock -1);
	  // Habilita interrupcion del modulo TIMER
	  IntEnable(INT_TIMER5A);
	  // Y habilita, dentro del modulo TIMER5, la interrupcion de particular de "fin de cuenta"
	  TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);



	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	MAP_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);


	/**                                              Creacion de tareas 									**/

	// Inicializa el sistema de depuración por terminal UART
	if (initCommandLine(512,tskIDLE_PRIORITY + 1) != pdTRUE)
    {
        while(1);
    }

	USBSerialInit(32,32);	//Inicializo el  sistema USB
	//
	// Crea la tarea que gestiona los mensajes USB (definidos en USBMessageProcessingTask)
	//
	if(xTaskCreate(USBMessageProcessingTask,"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
	{
		while(1);
	}

    //
    // A partir de aquí se crean las tareas de usuario, y los recursos IPC que se vayan a necesitar
    //

	if(xTaskCreate(ADCTask,"Tarea_ADC",ADCTASKSTACKSIZE,NULL,tskIDLE_PRIORITY+1,NULL)!= pdTRUE){ //Creación de la tarea

	    while(1);

	}

    if(xTaskCreate(RelojTask,"Tarea_Reloj",RELOJTASKSTACKSIZE,NULL,tskIDLE_PRIORITY+1,NULL)!= pdTRUE){ //Creación de la tarea

        while(1);

    }

    if(xTaskCreate(CombustibleTask,"Tarea_Combustible",COMBUSTIBLETASKSTACKSIZE,NULL,tskIDLE_PRIORITY+1,NULL)!= pdTRUE){ //Creación de la tarea de combustible

        while(1);

    }

    if(xTaskCreate(AlturaTask,"Tarea_Altura",ALTURATASKSTACKSIZE,NULL,tskIDLE_PRIORITY+1,NULL)!= pdTRUE){ //Creación de la tarea de altura

        while(1);

    }

    if(xTaskCreate(RadioTask,"Tarea_Radio",RADIOSKSTACKSIZE,NULL,tskIDLE_PRIORITY+1,NULL)!= pdTRUE){ //Creación de la tarea de radio
        while(1);

    }


	cola_Mpotenciometros = xQueueCreate(3,sizeof(uint32_t)*3); //Creamos la cola de los mensajes
	if(NULL == cola_Mpotenciometros){
	    while(1); //Si entramos aqui es porque algo ha ido mal

	}

	cola_Reloj = xQueueCreate(3,sizeof(uint32_t)); //Creamos la cola del reloj
	if(NULL == cola_Reloj){
	    while(1);
	}

    cola_Vcombustible = xQueueCreate(3,sizeof(float)); //Creamos la cola de la velocidad usada en la tarea del combustible
    if(NULL == cola_Vcombustible){
        while(1);
    }

    cola_Tcombustible = xQueueCreate(3,sizeof(uint32_t)); //Creamos la cola del tiempo usada en la tarea del combustible
    if(NULL == cola_Tcombustible){
        while(1);
    }



    cola_Valtura = xQueueCreate(3, sizeof(float)); //Creamos la cola para la velocidad en la tarea de la altura
    if(NULL == cola_Valtura){

        while(1);

    }

    cola_Maltura = xQueueCreate(3,sizeof(uint16_t)); //Creamos la cola para el mensaje del pitch en la tarea de la altura
    if(NULL == cola_Maltura){

        while(1);

    }

    cola_Caltura = xQueueCreate(3,sizeof(float)); //Creamos la cola para el combustible que se usa en la tarea de la altura
    if(NULL == cola_Caltura){

        while(1);

    }

    // Cola para el envio del estado del estado de la traza [on|off]
    cola_comandos = xQueueCreate(1,sizeof(bool));
    if (NULL==cola_comandos)
        while(1);

    // Cola para el envio de un mensaje radio
    cola_radio = xQueueCreate(1,sizeof(char)*40);
    if (NULL==cola_radio)
        while(1);

    // Mutex para proteccion del canal USB (por si 2 tareas deciden usarlo a la vez!)
    mutexUSB=xSemaphoreCreateMutex();
    if(mutexUSB==NULL){ // Error de creacion de semaforos
        while(1);

    }

    mutexRGB = xSemaphoreCreateMutex(); //Mutex para protección de los cambios RGB
    if(mutexRGB==NULL){
        while(1);

    }

    mutexUART = xSemaphoreCreateMutex(); //Mutex para protección UART
    if(mutexUART==NULL){
        while(1);

    }

    semaforo_tiempo =xSemaphoreCreateBinary();
    if ((semaforo_tiempo==NULL))
    {
        while (1);  //No hay memoria para los semaforo
    }

    //Creamos el conjunto de colas (Queue Set) con el tamaño de la suma de las posiciones de las colas que lo forman

    grupo_colas = xQueueCreateSet(7); //No sé por qué me sale error
    if(grupo_colas == NULL){

        while(1);

    }

    //Añadimos cada una de las colas que forman el grupo de las colas

    if (xQueueAddToSet(cola_Valtura, grupo_colas)!=pdPASS)
    {
        while(1);
    }

    if (xQueueAddToSet(cola_Maltura, grupo_colas)!=pdPASS)
    {
        while(1);
    }

    if (xQueueAddToSet(semaforo_tiempo, grupo_colas)!=pdPASS)
    {
        while(1);
    }


	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

    void ADCHandler(void){

        BaseType_t higherPriorityTaskWoken=pdFALSE; //Hay que inicializarlo a False!!

        uint32_t pui32_DatosPotenciometros[3]; //Creamos el array con los datos
        ADCSequenceDataGet(ADC0_BASE,1, pui32_DatosPotenciometros);
        ADCIntClear(ADC0_BASE,1); //Limpiamos el flag de interrupción del ADC

        xQueueSendFromISR(cola_Mpotenciometros,pui32_DatosPotenciometros,&higherPriorityTaskWoken); //Escribe en la cola freeRTOS
        portEND_SWITCHING_ISR(higherPriorityTaskWoken);
        //xQueueSendFromISR(cola_Maltura,pui32_DatosPotenciometros,&higherPriorityTaskWoken); //Escribe en la cola freeRTOS

    }

    void RelojHandler(void){

        static uint32_t s_ui32Segundero = 0;

        BaseType_t higherPriorityTaskWoken1 = pdFALSE;
        BaseType_t higherPriorityTaskWoken2 = pdFALSE;
        BaseType_t higherPriorityTaskWoken3 = pdFALSE;

        TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT); // Borra la interrupcion de Timer

        s_ui32Segundero++; //Incrementamos nuestro contador


        xQueueSendFromISR(cola_Reloj,&s_ui32Segundero,&higherPriorityTaskWoken1); //Escribe en la cola freeRTOS
        xQueueSendFromISR(cola_Tcombustible,&s_ui32Segundero,&higherPriorityTaskWoken2); //Escribe en la cola freeRTOS
        xSemaphoreGiveFromISR(semaforo_tiempo,&higherPriorityTaskWoken3);//Abre el semaforo FreeRTOS

        portEND_SWITCHING_ISR(higherPriorityTaskWoken1 || higherPriorityTaskWoken2 || higherPriorityTaskWoken3);

    }

