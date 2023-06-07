#ifndef EVENTOS_APLICACION_H
#define EVENTOS_APLICACION_H
#include <maquina_estado.h>

/**
 * @brief Defino los eventos posibles que pueden ocurrir en la máquina
 * 
 */
enum EventoAplicacion{
    /**
     * @brief El botón de comando del controlador fue pulsado
     * 
     */
    EV_BOTON_PULSADO = EV_USUARIO,
    /**
     * @brief El botón se presionó 3 veces
     * 
     */
    EV_TRIPLE_PULSACION,
    /**
     * @brief Transcurrió el tiempo programado
     *  
     */
    EV_TIMEOUT,

};

#endif