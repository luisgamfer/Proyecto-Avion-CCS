/*
 * Listado de los tipos de mensajes empleados en la aplicaci�n, as� como definiciones de sus par�metros.
 * (TODO) Incluir aqu� las respuestas a los mensajes
*/
#ifndef __USB_MESSAGES_TABLE_H
#define __USB_MESSAGES_TABLE_H

#include<stdint.h>
#include<stdbool.h>

//Codigos de los mensajes. EL estudiante deberá definir los c�digos para los mensajes que vaya
// a crear y usar. Estos deberan ser compatibles con los usados en la parte Qt

typedef enum {
    MENSAJE_NO_IMPLEMENTADO,
    MENSAJE_PING,
    MENSAJE_POTENCIOMETRO, //Mensaje usado para los potenciometros
    MENSAJE_VELOCIDAD,
    MENSAJE_RELOJ,
    MENSAJE_COMBUSTIBLE,
    MENSAJE_ALTURA,
    MENSAJE_COLISION,
    MENSAJE_INICIO,
    MENSAJE_MSG_RADIO,
    //etc, etc...
} messageTypes;

//Estructuras relacionadas con los parametros de los mensajes. El estuadiante debera crear las
// estructuras adecuadas a los mensajes usados, y asegurarse de su compatibilidad con el extremo Qt
#define PACKED __attribute__ ((packed))
//Estructura que se encarga de los mensajes de los potenciometros
typedef struct{
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
} PACKED PARAM_MENSAJE_POTENCIOMETRO;

//#pragma pack(1)   //Con esto consigo que el alineamiento de las estructuras en memoria del PC (32 bits) no tenga relleno.
//Con lo de abajo consigo que el alineamiento de las estructuras en memoria del microcontrolador no tenga relleno
//#define PACKED __attribute__ ((packed))

typedef struct {
    uint8_t message;
} PACKED PARAM_MENSAJE_NO_IMPLEMENTADO;

typedef struct {
    float bIntensity;
} PACKED PARAM_MENSAJE_VELOCIDAD;

typedef struct {
    uint32_t reloj;
} PACKED PARAM_MENSAJE_RELOJ;

typedef struct {
    float combustible;
} PACKED PARAM_MENSAJE_COMBUSTIBLE;

typedef struct {
    float altura;
} PACKED PARAM_MENSAJE_ALTURA;

typedef struct {
    char caracteres[40]; // 40 mas el terminador de string
} PACKED PARAM_MENSAJE_MSG_RADIO;


//#pragma pack()    //...Pero solo para los mensajes que voy a intercambiar, no para el resto





#endif
