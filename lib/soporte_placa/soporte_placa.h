#ifndef SOPORTE_PLACA_H
#define SOPORTE_PLACA_H

#include <stdbool.h>
#include <stdint.h>

// Declaraciones
// Espacio de nombres: SP_

/**
 * @brief Handles correspondientes a los pines de entrada/salida, para 
 * usar en el parámetro hPin (primer parámetro) de las funciones SP_Pin_xxx
 * 
 */
enum SP_Pines{
    SP_PB9, // 0
    SP_PC13, // 1
    SP_LED = SP_PC13, // LED verde integrado en la placa, pin PC13 
    SP_HPIN_LIMITE // 2
};

/**
 * @brief Handle que representa un objeto Pin.
 * Toma valores de las constantes SP_Pines
 */
typedef int SP_HPin;

/**
 * @brief Definición de modo de operación de un Pin
 * 
 */
typedef enum SP_Pin_Modo{
    SP_PIN_ENTRADA,          // Entrada flotante
    SP_PIN_ENTRADA_PULLUP,   // Entrada con resistencia pull-up interna
    SP_PIN_ENTRADA_PULLDN,   // Entrada con resistencia pull-down interna
    SP_PIN_SALIDA,           // Salida push-pull
    SP_PIN_SALIDA_OPEN_DRAIN // Salida con drenador abierto
}SP_Pin_Modo;

/**
 * @brief Configura el modo de un pin
 * 
 * @param pin Handle al objeto Pin
 * @param modo Modo a configurar
 */
void SP_Pin_setModo(SP_HPin hPin,SP_Pin_Modo modo);

/**
 * @brief Lee el buffer de entrada de un Pin
 * 
 * @param pin Handle al objeto Pin
 * @return true Entrada ALTA
 * @return false Entrada BAJA
 */
bool SP_Pin_read(SP_HPin hPin);

/**
 * @brief Escribe el buffer de salida de un Pin
 * 
 * @param pin Handle al objeto Pin
 * @param valor True: Salida ALTA. False: Salida BAJA.
 */
void SP_Pin_write(SP_HPin hPin, bool valor);

/**
 * @brief Inicializa la librería. Es necesario llamar a
 * esta función antes de usar la misma.
 * 
 */
void SP_init(void);

/**
 * @brief Retardo con bloqueo durante un tiempo dado en milisegundos
 * @note Llamar a SP_init antes de usar.
 * @param tiempo Tiempo en milisegundos
 */
void SP_delay(uint32_t tiempo);


void HSE_8M_CLOCK_CONFIG (void);

#endif