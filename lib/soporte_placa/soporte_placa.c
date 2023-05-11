#include <soporte_placa.h>
#include <stm32f1xx.h>
#include <stdint.h>

// Implementación

/**
 * @brief Rutina de servicio de interrupción de timer SysTick
 * 
 */
void SysTick_Handler(void);

/* Inicialización general */

void SP_init(void){
    // Ver documentación CMSIS
    // https://arm-software.github.io/CMSIS_5/Core/html/group__system__init__gr.html#gae0c36a9591fe6e9c45ecb21a794f0f0f
    SystemCoreClockUpdate();
    
    uint32_t const frecuencia_hertz = SystemCoreClock;
    uint32_t const cuentas_por_milisgundo = frecuencia_hertz/1000;

    // https://arm-software.github.io/CMSIS_5/Core/html/group__SysTick__gr.html#gabe47de40e9b0ad465b752297a9d9f427
    SysTick_Config(cuentas_por_milisgundo); // Configura SysTick y la interrupción
}


/* Temporización */

/**
 * @brief Variable actualizada una vez por milisegundo en el handler
 * de interrupción del timer del sistema (SysTick)
 * 
 */
static uint32_t volatile ticks;

void SP_delay(uint32_t tiempo){
    uint32_t const ticks_inicial = ticks;
    uint32_t tiempo_transcurrido = ticks - ticks_inicial;
    while(tiempo_transcurrido < tiempo){
        // https://arm-software.github.io/CMSIS_5/Core/html/group__intrinsic__CPU__gr.html#gaed91dfbf3d7d7b7fba8d912fcbeaad88
        __WFI();
        tiempo_transcurrido = ticks - ticks_inicial;
    }

}


void SysTick_Handler(void){
    ++ticks;
}

/* GPIO */

typedef struct Pin{
    GPIO_TypeDef * puerto;
    int nrPin;
}Pin;

static Pin const pines[SP_HPIN_LIMITE] = {
    [SP_PB9]={.puerto=GPIOB,.nrPin=9},
    [SP_PC13]={.puerto=GPIOC,.nrPin=13}
};

/**
 * @brief Obtiene un puntero a Pin a partir de su handle
 * 
 * @param hPin Handle
 * @return Pin const* Puntero al objeto Pin (solo lectura) 
 */
static Pin const * pinDeHandle(SP_HPin hPin){
    return &pines[hPin];
}
/**
 * @brief Calcula la posición en del bit de habilitación
 * del puerto en APB2_ENR a partir de su dirección en memoria.
 */

/**
 * @brief Habilita el reloj de un puerto GPIO
 * @note Debe ser llamada antes de intentar usar el puerto
 * por primera vez.
 * @param puerto Puntero al puerto GPIO 
 * @return uint32_t Máscara de habilitación de reloj
 */
static void habilitaRelojPuerto(GPIO_TypeDef const *puerto){
    int const offset_habilitacion = (((uint32_t)(puerto) >> 10) & 0xF);
    RCC->APB2ENR |= 1 << offset_habilitacion;
}
// ... continúa implementación

/**
 * @brief Escribe los bits de modo en la posición adecuada
 * de CRL o CRH según el pin
 * 
 * @param pin 
 * @param bits_modo 
 */
static void config_modo(Pin const *pin, int bits_modo){
    // Ver Manual de referencia de la familia sec. 9.2.1/.2
    int const offset = (pin->nrPin % 8)*4;
    uint32_t const mascara = 0xF; // 4 bits de configuración
    if (pin->nrPin < 8){
        pin->puerto->CRL =  (pin->puerto->CRL & ~(mascara << offset))
                          | ((bits_modo & mascara)<<offset); 
    }else{ // 8..15
        pin->puerto->CRH =  (pin->puerto->CRH & ~(mascara << offset))
                          | ((bits_modo & mascara)<<offset); 
    }
}

void SP_Pin_setModo(SP_HPin hPin,SP_Pin_Modo modo){
    // Ver Manual de referencia de la familia sec. 9.2.1/.2
    enum ConfigsPin{
        /** 
         * Bits[1:0]: Modo E/S, 00 es modo entrada
         * Bits[3:2]: Configuración de entrada, 01 es entrada flotante
         */
        ENTRADA_FLOTANTE = 0b0100,
        /** 
         * Bits[1:0]: Modo E/S, 00 es modo entrada
         * Bits[3:2]: Configuración de entrada, 10 es entrada con pull-up/pull-dn
         */
        ENTRADA_PULLUP_PULLDN = 0b1000,
        /** 
         * Bits[1:0]: Modo E/S, 10 es modo salida con frec. máxima de 2MHz
         * Bits[3:2]: Configuración de salida, 00 es salida de propósito general normal (push/pull)
         */
        SALIDA_2MHz = 0b0010,
        /** 
         * Bits[1:0]: Modo E/S, 10 es modo salida con frec. máxima de 2MHz
         * Bits[3:2]: Configuración de salida, 01 es salida de propósito general open drain
         */
        SALIDA_2MHz_OPEN_DRAIN = 0b0110
    };
    if(hPin >= SP_HPIN_LIMITE) return; // debiera generar un error
    Pin const *pin = pinDeHandle(hPin);
    __disable_irq();
    habilitaRelojPuerto(pin->puerto);
    switch (modo)
    {
    case SP_PIN_ENTRADA:
        config_modo(pin,ENTRADA_FLOTANTE);
    break;case SP_PIN_ENTRADA_PULLUP:
        config_modo(pin,ENTRADA_PULLUP_PULLDN);
        pin->puerto->BSRR = 1 << pin->nrPin;
    break;case SP_PIN_ENTRADA_PULLDN:
        config_modo(pin,ENTRADA_PULLUP_PULLDN);
        pin->puerto->BRR = 1 << pin->nrPin;
    break;case SP_PIN_SALIDA:
        config_modo(pin,SALIDA_2MHz);
    break;case SP_PIN_SALIDA_OPEN_DRAIN:
        config_modo(pin,SALIDA_2MHz_OPEN_DRAIN);
    break;default:
    // Debiera generar un error
    break;
    }
    __enable_irq();
}

//El parametro pin toma valores entre 0 y 2

bool SP_Pin_read(SP_HPin hPin){              //Funcion para leer pin
    bool pin_state;                          //Declaro la variable de salida
    if(hPin >= SP_HPIN_LIMITE) return false;       //Si el pin no es válido deja de ejecutar
    Pin const *pin = pinDeHandle(hPin);      //Obtengo el puerto del pin y el numero del mismo
    uint32_t const PIN_ON = (1<<pin->nrPin); // Declaro una constante que es un 1 desplazado el numero de pin (9 o 13)
    if((pin->puerto->IDR & PIN_ON) == 0) {   //Comparo el registro ODR con la constante antes declarada
        pin_state = false;                   //Si el bit en la posición "nrPin" es cero, pin_state es 0
    }else{
        pin_state = true;                    //De lo contrario será 1
    }
    return pin_state;
    
}

void SP_Pin_write(SP_HPin hPin, bool valor){
    if(hPin >= SP_HPIN_LIMITE) return;            //Si el pin no es válido deja de ejecutar
    Pin const *pin = pinDeHandle(hPin);           //Obtengo el puerto del pin y el numero del mismo
    if (valor == true){                           //Si se requiere escribir un 1
        pin->puerto->BSRR = (1<<pin->nrPin);      //Modifico el bit correspondiente en el registro BSRR del puerto correspondiente (desde el bit 0 al 15)
    }else{                                        //Si, en cambio, se requiere esribir un 0
        pin->puerto->BSRR = (1<<(pin->nrPin+16)); //Modifico el bit correspondiente (desde el bit 16 al 32)
    }
}

void HSE_8M_CLOCK_CONFIG (void) {

    enum {
    CR_HSE_ON = (1<<16), //Un registro con todo 0 excepto el bit 16
    CR_HSE_READY = (1<<17), //Un registro con todo 0 excepto el bit 17
    CFGR_SW_MASK = (3<<0), //Un registro terminado en 11    
    CFGR_SW_HSE = (1<<0), //Registro terminado en 1
    CFGR_SWS_MASK = (3<<2), //Registro con 11 desplazado 2 bits
    CFGR_SWS_READY = (1<<2) //Registro con 1 desplazado 2 bits 
};

    RCC->CR = RCC->CR | CR_HSE_ON; //Activo el bit 16 de RCC->CR, equivalente a: RCC->CR |= CR_HSE_ON;
    while (!(RCC->CR & CR_HSE_READY)) continue; //Si el registro de la condicion tiene al menos un 1, la ejecucion continua
    RCC->CFGR = (RCC->CFGR & ~CFGR_SW_MASK) | CFGR_SW_HSE; //Pongo en 01 los dos ultimos bits de RCC->CFGR
    while((RCC->CFGR & CFGR_SWS_MASK) == CFGR_SWS_READY) continue; //Si CFGR[3,2] = [01] continuo
    RCC->CFGR &= ~(1<<7); //Configuro prescaler 
    RCC->CFGR &= ~(1<<10); //Configuro prescaler de baja velocidad
    RCC->CFGR &= ~(1<<13); //Configuro prescaler de alta velocidad 
    SystemCoreClockUpdate(); //Actualizo las configuraciones del reloj 
}

