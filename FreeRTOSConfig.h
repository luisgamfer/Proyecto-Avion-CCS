/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                1 // 1/0: Indica si FreeRTOS hará uso de una planificación expropiativa
#define configUSE_IDLE_HOOK                 1 // 1/0: Indica si la tarea IDLE llamará a una función de usuario al ejecutarse
#define configUSE_TICK_HOOK                 1 // 1/0: Indica si se ejecutará una función de usuario asociada al TICK del sistema
#define configMAX_PRIORITIES                ( 16 )  // Número de prioridades para las tareas
#define configCPU_CLOCK_HZ                  ( ( unsigned long ) MAP_SysCtlClockGet() ) // Frecuencia de reloj del sistema (debe coincidir con SyCtlClockSet)
#define configTICK_RATE_HZ                  ( ( portTickType ) 1000 ) // Número de TICKS por segundo --> precision del SO
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 128 )   // Tamaño de la pila de la tarea IDLE en objetos (ARM:1objeto->32bits)
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 20 * 1024 ) ) // Memoria dinámica reservada a FreeRTOS (en bytes)
#define configMAX_TASK_NAME_LEN             ( 12 ) // Longitud máxima de los nombres dados a las tareas
#define configUSE_TRACE_FACILITY            1 // 1/0: Activación del modo traza
#define configUSE_16_BIT_TICKS              0 // 1/0: Tamaño de la variable de cuenta de ticks (1:16bits, 0:32bits)
#define configIDLE_SHOULD_YIELD             1 // 1/0: Si 1, la tarea IDLE SIEMPRE cede el control a otra tarea, aunque tenga tambien prioridad 0
#define configUSE_MUTEXES                   1 // 1/0: Indica si se van a usar MUTEX en la aplicación
#define configUSE_RECURSIVE_MUTEXES         0 // 1/0: Indica si se van a usar MUTEX recursivos en la aplicación
#define configUSE_COUNTING_SEMAPHORES       0 // 1/0: Indica si se van a usar semaforos contadores en la aplicación
#define configUSE_MALLOC_FAILED_HOOK    1 // 1/0: Indica si se ejecutará una función de usuario en caso de fallo de memoria dinámica
#define configUSE_APPLICATION_TASK_TAG  0 // 1/0: Activa el modo TAG de las tareas (relacionado con la depuración)
#define configGENERATE_RUN_TIME_STATS   1 // 1/0: Activa la recogida de estadísticas (relacionado con la depuración)
#define configUSE_TICKLESS_IDLE     0  // 1/0: Desactiva la ejecución de la tarea IDLE si el sistema se suspende durante un tiempo
                                       // hasta que el sistema vuelva a reactivarse
#define configCHECK_FOR_STACK_OVERFLOW      (2) // 0/1/2: Activa alguno de los mecanismos de chequeo de desbordamiento en pila de tareas

#define configUSE_CO_ROUTINES               0 // 1/0: Activa el uso de mecanismos de corrutinas
#define configMAX_CO_ROUTINE_PRIORITIES     ( 2 ) // Indica el número de prioridades que podrán tener las tareas que funcionen como corrutinas
#define configQUEUE_REGISTRY_SIZE           10 // Define el máximo número de colas y semáforos que se pueden registrar en el sistema
#define configUSE_QUEUE_SETS                1 // 1/0: Indica si se van a usar conjuntos de colas


/* Software timer definitions. */
#define configUSE_TIMERS                1 // 1/0: Activa el uso de timers SW (basados en ticks)
#define configTIMER_TASK_PRIORITY       ( 2 ) // Fija la prioridad de la tareas interna que actualiza los timer SW
#define configTIMER_QUEUE_LENGTH        32 // Tama�o de la cola de comandos de control de Timers SW
#define configTIMER_TASK_STACK_DEPTH    ( configMINIMAL_STACK_SIZE * 2 ) // Tamaño de la pila de la tarea interna que gestiona los timer SW


#define configUSE_STATS_FORMATTING_FUNCTIONS 1 // 1/0: Formateo de parámetros estadísticos recogidos en depuración

//Esto es para las estadisticas
#if configGENERATE_RUN_TIME_STATS
#ifndef __ASM_HEADER__ /* elimina un monton de warnings */
#include"utils/RunTimeStatsConfig.h"
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()            vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()    GetOverflowCounts()
#endif
#endif


/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_pcTaskGetTaskName           1
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xEventGroupSetBitFromISR    1

//La definicion de INCLUDE_xTimerPendFunctionCallFromISR es necesaria para que funcionen
//Los flags de eventos. Ademas la utilizamos para lanzar codigo que se ejecute a nivel de tarea dese una ISR
#define INCLUDE_xTimerPendFunctionCallFromISR          1
//Esta de abajo por lo visto es necesaria para que funcionen los eventos ??
#define INCLUDE_xTimerPendFunctionCall          1


/* Be ENORMOUSLY careful if you want to modify these two values and make sure
 * you read http://www.freertos.org/a00110.html#kernel_priority first!
 */
#define configKERNEL_INTERRUPT_PRIORITY         ( 7 << 5 )    /* Priority 7, or 0xE0 as only the top three bits are implemented.  This is the lowest priority. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     ( 5 << 5 )  /* Priority 4, or 0x80 as only the top three bits are implemented. */

#endif /* FREERTOS_CONFIG_H */
