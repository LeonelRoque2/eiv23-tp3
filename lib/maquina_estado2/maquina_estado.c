#include "maquina_estado_impl.h"
#include <stddef.h>
#include <stm32f1xx.h>

void Maquina_init(Maquina *self, Estado estadoInicial){
    self->estadoInicial = estadoInicial;                    //Coloca el parámetro "estadoInicial" en el estado inicial de la maquina
    self->estadoActual = (Estado)0;                         //Puntero nulo a funcion (Estado actual no definido) 
    self->cola.lecturas = 0;                                //Lecturas al iniciar = 0
    self->cola.escrituras = 0;                              //Escrituras al iniciar = 0
    Maquina_despacha(self, EV_RESET);                       //Despacho el evento RESET
}

static bool Maquina__qEspacioEnCola (Maquina const *self) {
    return(self->cola.escrituras - self->cola.lecturas) < MAX_EV_COLA;
}

bool Maquina_despacha(Maquina *self, Evento evento){
    bool hecho = false;        
    unsigned const posicion = (self->cola.escrituras) % MAX_EV_COLA;                                             
    __disable_irq();                                                        
    if((EV_NULO != evento) && Maquina__qEspacioEnCola(self)){      //Si aun hay espacio en la cola agrego un elemento  
        self->cola.eventos[posicion] = evento;                                     
        self->cola.escrituras++;                                                
        hecho = true;
    }
    __enable_irq();
    return hecho;
}

static bool Maquina__qEventosDisponiblesEnCola(Maquina const *self){
    return self->cola.escrituras != self->cola.lecturas;               //Si no hay la misma cantidad de eventos en cola que procesados
}

/**
 * @brief Me da el siguiente evento a procesar
 * 
 * @param self Puntero a maquina
 * @return Evento a procesar
 */
static Evento Maquina_siguienteEvento(Maquina *self){
    Evento evento = EV_NULO;
    if(Maquina__qEventosDisponiblesEnCola(self)){                           //Hay eventos disponibles para procesar?
        unsigned const posicion = (self->cola.lecturas) % MAX_EV_COLA;      
        evento = self->cola.eventos[posicion];
        self->cola.lecturas++;
    }
    return evento;
}



bool Maquina_procesa(Maquina *self){
    Evento const evento = Maquina_siguienteEvento(self);            //"evento" almacena el siguiente evento a procesar
    bool procesado = false;
    if (evento != EV_NULO){
        procesado = true;                                       
        if (evento == EV_RESET || !self->estadoActual){             //Si el evento es reset o el estado actual no esta definido (Puntero nulo), reinicio la maquina
            self->estadoActual = self->estadoInicial;               
        }
        
        Resultado resultado = self->estadoActual(self,evento);      //"resultado" contiene el código de resultado y el nuevo estado de la máquina que procesa "evento"
        
        if (resultado.codigo == RES_TRANSICION){                 //Si el evento esta en proceso, seteo el estado actual en el nuevo estado
            self->estadoActual = resultado.nuevoEstado;
        }
    }
    return procesado;                                               //Devuelvo la variable que me confirma que el evento se proceso
}
