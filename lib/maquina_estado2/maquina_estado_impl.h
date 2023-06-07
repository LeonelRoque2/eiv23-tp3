#ifndef MAQUINA_ESTADO_IMPL_H
#define MAQUINA_ESTADO_IMPL_H

#include <maquina_estado.h>

/**
 * @brief Codigo del resultado del procesamiento de un evento.
 * 
 * 0: Evento Ignorado
 * 1: Evento Procesado
 * 2: Evento en proceso
 * 
 */
typedef enum codigoResultado {
    RES_IGNORADO,
    RES_PROCESADO,
    RES_TRANSICION
} codigoResultado;

/**
 * @brief Resultado: El resultado de procesaro un evento
 * codigo: Numero que expresa el resultado luego del procesamiento
 * Estado: Variable que almacena el próximo estado a alcanzar por la máquina
 * 
 */
struct Resultado{
    codigoResultado codigo;
    Estado nuevoEstado; // Valido cuando resultado = RES_TRANSICION
};

/**
 * @brief Inicializa la máquina de estado
 * 
 * @param self Puntero a máquina
 * @param estadoInicial El estado inicial que queremos poner a la maquina
 */
void Maquina_init(Maquina *self, Estado estadoInicial);

#endif
