#include "controlador_de_pulsaciones.h"
#include <maquina_estado_impl.h>

static Resultado estadoEspera(Maquina *contexto, Evento evento);
static Resultado estadoCuenta(Maquina *contexto, Evento evento);

void ControladorDePulsaciones_init (ControladorDePulsaciones *self, Maquina *maq_destino, DespachoRetardado *despachoRetardado, uint32_t tiempoPulsaciones){
    Maquina_init(&self->maquina,estadoEspera);
    self->destino = maq_destino;
    self->despachoRetardado = despachoRetardado;
    self->tiempoPulsaciones = tiempoPulsaciones;
}

Maquina * ControladorDePulsaciones_asMaquina(ControladorDePulsaciones *self) {
    return &self->maquina;
}

static Resultado estadoEspera(Maquina *contexto, Evento evento) {
    ControladorDePulsaciones *self = (ControladorDePulsaciones*)contexto;   
    Resultado r = {0};
    switch (evento) {
    case EV_BOTON_PULSADO:
        Maquina_despacha(self->destino,EV_BOTON_PULSADO);
        DespachoRetardado_programarDespacho(self->despachoRetardado,contexto,EV_TIMEOUT,self->tiempoPulsaciones);
        self->contadorPulsaciones = 1;
        r.codigo = RES_TRANSICION;
        r.nuevoEstado = estadoCuenta;
    break;default:
        r.codigo = RES_IGNORADO;
    break;
    }
    return r;
}

static Resultado estadoCuenta(Maquina *contexto, Evento evento) {
    ControladorDePulsaciones *self = (ControladorDePulsaciones*)contexto; 
    Resultado r = {0};
    switch (evento) {
        case EV_BOTON_PULSADO:
        if(self->contadorPulsaciones < 2) {
            self->contadorPulsaciones++;
            r.codigo = RES_PROCESADO;
        }
        else {
            Maquina_despacha(self->destino,EV_TRIPLE_PULSACION);
            r.codigo = RES_TRANSICION;
            r.nuevoEstado = estadoEspera;
        }
        break;case EV_TIMEOUT:
            r.codigo = RES_TRANSICION;
            r.nuevoEstado = estadoEspera;
        break;default:
        r.codigo = RES_IGNORADO;
        break;
    }
    return r;
}