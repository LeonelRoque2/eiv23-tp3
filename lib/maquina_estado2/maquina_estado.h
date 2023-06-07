#ifndef MAQUINA_ESTADO_H
#define MAQUINA_ESTADO_H

#include <stdbool.h>

#ifndef MAX_EV_COLA
#define MAX_EV_COLA 16 /*Debe ser potencia de 2 para optimizar tiempo de cálculo*/
#endif

#if MAX_EV_COLA & (MAX_EV_COLA - 1) //La sentencia es verdadera si MAXIMOS_EVENTOS_EN_COLA NO es potencia de 2
#error MAX_EV_COLA debe ser potencia de dos 
#endif

typedef struct Maquina Maquina;                                     //"Maquina" es una variable tipo "struct Maquina"
typedef unsigned Evento;                                            //Evento es un numero
typedef struct Resultado Resultado;                                 //"Resultado" es una variable tipo "struct Resultado"
typedef Resultado (*Estado)(Maquina* contexto, Evento evento);      //Estado es un puntero a funcion, que recibe puntero a maquina y un evento, y devuelve un resultado 


enum EventoSistema{                                                 //Enumero los tipos de eventos posibles
    /**
     * @brief Significa que no hay eventos disponibles
     */
    EV_NULO,
    /**
     * @brief Inicia o reinicia la máquina de estado. Establece
     * como estado actual el estado inicial y luego envía el
     * evento de reset a dicho estado.
     * 
     */
    EV_RESET,
    /**
     * @brief Primer evento disponible para la aplicación
     */
    EV_USUARIO
};


/**
 * @brief Máquina de estado
 */
struct Maquina{                                             //Contiene: Cola (Espacios totales, leidos y escritos), Estado inicial, Estado final
    struct {
        /**
         * @brief Cola de eventos con 16 espacios libres
         */
        Evento eventos[MAX_EV_COLA];                               
        /**
         * @brief Cantidad de elementos tomados de la cola (0 < lecturas < escrituras).
         * Lecturas % MAX_EV_COLA : indice del proximo elemento a leer
         * SI lecturas != escrituras
         */
        unsigned lecturas;
        /**
         * @brief Cantidad de elementos ingresados por la maquina en la cola (lecturas < escrituras < 16)
         * Escrituras % MAX_EV_COLA : indice del proximo espacio libre 
         * SI (escrituras - lecturas) < MAX_EV_COLA
         */
        unsigned escrituras;
    }cola;
    Estado estadoInicial;                       //Variable tipo estado que representa el estado inicial de la maquina
    Estado estadoActual;                        //Variable tipo estado que representa el estado actual
};


/**
 * @brief Despacha (Atiende) un evento para posterior procesamiento. (Pone el evento en la cola FIFO)
 * 
 * @param self Puntero a la maquina
 * @param evento Evento a despachar en la cola.
 * @return true Evento despachado
 * @return false Falla al despachar evento
 */
bool Maquina_despacha(Maquina *self, Evento evento);

/**
 * @brief Procesa un evento disponible en la cola. Este método debe ser llamado
 * desde un solo punto del programa.
 * 
 * @param self Puntero a la maquina
 * @return true Evento procesado
 * @return false No había eventos disponibles
 */
bool Maquina_procesa(Maquina *self);


#endif
