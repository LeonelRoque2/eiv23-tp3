#include <soporte_placa.h>
#include "controlador_luz.h"
#include "controlador_de_pulsaciones.h"
#include "pulsador.h"
#include "despacho_retardado.h"
#include <stddef.h>


#define PIN_LUZ SP_PIN_LED
#define PIN_PULSADOR SP_PB9

#define HISTERESIS_ANTIRREBOTE 5

#define LUZ_ON 0

#define PULSADOR_NIVEL_ACTIVO 0

#define TIEMPO_ON 60000

#define TIEMPO_TRIPLE_PULSACION 1000


static Maquina * controladorLuz;
static Maquina * controladorPulsaciones;
static Pulsador pulsador[1];
static DespachoRetardado despachoRetardado[1];

/**
 * @brief Inicializa el estado del programa para iniciar la ejecución
 * 
 */
static void setup(void);


int main(void){    
    setup();
    for (;;){
        Maquina_procesa(controladorPulsaciones);
        Maquina_procesa(controladorLuz);
        DespachoRetardado_procesarDespacho(despachoRetardado);
        Pulsador_procesa(pulsador);
    }
    return 0;
}



static void setup(void){
    static ControladorLuz instanciaControlador;
    static ControladorDePulsaciones instanciaPulsaciones;
    
    SP_init();
    
    DespachoRetardado_init(despachoRetardado);

    ControladorLuz_init(&instanciaControlador,TIEMPO_ON,PIN_LUZ,LUZ_ON,despachoRetardado);
    controladorLuz = ControladorLuz_asMaquina(&instanciaControlador);
    Maquina_procesa(controladorLuz); // Reset inicializa pin con luz apagada
    
    ControladorDePulsaciones_init(&instanciaPulsaciones,controladorLuz,despachoRetardado,TIEMPO_TRIPLE_PULSACION);
    controladorPulsaciones = ControladorDePulsaciones_asMaquina(&instanciaPulsaciones);

    Pulsador_init(pulsador, 
                  controladorPulsaciones,
                  EV_BOTON_PULSADO,
                  PIN_PULSADOR,
                  PULSADOR_NIVEL_ACTIVO,
                  HISTERESIS_ANTIRREBOTE);
}