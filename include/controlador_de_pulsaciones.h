#ifndef CONTROLADOR_DE_PULSACIONES_H
#define CONTROLADOR_DE_PULSACIONES_H

#include <stdint.h>
#include <maquina_estado.h>
#include "eventos_aplicacion.h"
#include "controlador_luz.h"
#include "despacho_retardado.h"

/**
 * @brief Controlador de pulsaciones
 * Intermediario entre el pulsador y el controlador de luz
 * 
 */
typedef struct ControladorDePulsaciones {
    Maquina maquina;
    Maquina *destino;
    DespachoRetardado *despachoRetardado;
    uint32_t tiempoPulsaciones;
    uint32_t contadorPulsaciones;
}ControladorDePulsaciones;


/**
 * @brief Inicializador del controlador de pulsaciones
 * 
 * [*] --> ESPERA               : EV_RESET / *
 * ESPERA --> CUENTA            : EV_BOTON_PULSADO / Maquina_despacha(controlador_luz,EV_BOTON_PULSADO); cnt = 1;
 * CUENTA --> CUENTA            : EV_BOTON_PULSADO [cnt < 3] / cnt++;
 * CUENTA --> ESPERA            : EV_BOTON_PULSADO [cnt >= 3] / Maquina_despacha(controlador_luz,EV_TRIPLE_PULSACION); 
 * 
 * @param self     Puntero al controlador de pulsaciones
 * @param contador  Contador de pulsaciones
 */
void ControladorDePulsaciones_init (ControladorDePulsaciones *self, Maquina *maq_destino, DespachoRetardado *despachoRetardado, uint32_t tiempoPulsaciones);

/**
 * @brief Controlador de pulsaciones como maquina de estado
 * 
 * @param self  Puntero a controlador de pulsaciones
 * @return Maquina* Este objeto como maquina de estado
 */

Maquina * ControladorDePulsaciones_asMaquina(ControladorDePulsaciones *self);

#endif