#ifndef PULSADOR_H
#define PULSADOR_H
#include <maquina_estado.h>
#include <soporte_placa.h>


/**
 * @brief Objeto Pulsador.
 * Despacha un evento al detectar pulsación.
 * Implementado como máquina de estado de control
 * 
 */
typedef struct Pulsador{
    struct{
        bool nivelActivo;                   //Nivel de pulsador cuando esta activo
        SP_HPin pin;                        //Handle del pin donde se conecta            
        uint8_t histeresis;                 //Número de lecturas consecutivas idénticas necesarias para cambiar el estado filtrado
    }parametros;
    uint32_t t0;
    Maquina *destino;                      //Puntero a maquina de estado
    Evento evento;                         //Evento que una pulsacion manda a despachar
    struct{
        bool nivelAnterior;                
        uint8_t contador;
    }estado;
} Pulsador;


/**
 * @brief Inicializa la lectura de pulsador y configura el evento
 * de pulsado y su destino
 * 
 * @param self Puntero a objeto pulsador
 * @param destino Puntero a objeto maquina
 * @param evento Evento a despachar (agregar a la cola) cuando se detecta pulsación
 * @param pin Handle de pin donde está conectado el pulsador
 * @param nivelActivo Nivel lógico que toma el pin cuando el
 * pulsador es presionado
 * @param histeresis Número de lecturas consecutivas idénticas
 * necesarias para cambiar el estado filtrado
 */
void Pulsador_init(Pulsador *self, 
                   Maquina *destino,
                   Evento evento,
                   SP_HPin pin,
                   bool nivelActivo,
                   uint8_t histeresis);
/**
 * @brief Lee pulsador, actualiza la máquina de estado y
 * despacha evento si detecta pulsación
 * 
 * @param self 
 */
void Pulsador_procesa(Pulsador *self);

#endif